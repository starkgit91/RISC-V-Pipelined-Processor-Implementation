// Standalone issue-queue tests. PRF values and accepted ROB completions are test drivers.
`timescale 1ns / 1ps

module tb_block06;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1, dispatch_hold, issue_hold, flush, flush_done;
  logic [1:0] dispatch_valid, dispatch_ready, dispatch_fire;
  renamed_pair_t dispatch_uop;
  logic [1:0] dispatch_src1_ready, dispatch_src2_ready;
  logic [1:0] completion_valid;
  completion_pair_t completion;
  logic rob_head_valid, mispredict_pending;
  rob_index_t rob_head_index;
  rob_tag_t rob_head_tag, mispredict_tag;
  logic alu0_ready, alu1_ready, muldiv_ready, memory_ready, system_ready;
  logic [1:0] issue_fire;
  execute_packet_pair_t issue_packet;
  phys_reg_t read0_rs1, read0_rs2, read1_rs1, read1_rs2;
  word_t read0_data1, read0_data2, read1_data1, read1_data2;
  issue_count_t occupancy;
  logic [ISSUE_DEPTH-1:0] occupied_mask;
  logic protocol_error;

  issue_entry_t ref_entries [0:ISSUE_DEPTH-1];
  word_t ref_values [0:PHYS_REG_COUNT-1];
  logic [ISSUE_DEPTH-1:0] expected_mask;
  logic [1:0] expected_dispatch, expected_issue;
  execute_packet_pair_t expected_packets;
  integer selected0, selected1, free0, free1;
  logic expected_protocol;
  renamed_uop_t a_uop, b_uop, c_uop, d_uop;
  integer checks = 0, failures = 0;
  integer n, kind0, kind1, resources, head_case, batch;
  word_t rng = 32'h43c0ffee;

  rv32_issue_queue dut (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_hold_i(dispatch_hold), .issue_hold_i(issue_hold),
    .dispatch_valid_i(dispatch_valid), .dispatch_uop_i(dispatch_uop),
    .dispatch_src1_ready_i(dispatch_src1_ready), .dispatch_src2_ready_i(dispatch_src2_ready),
    .dispatch_ready_o(dispatch_ready), .dispatch_fire_o(dispatch_fire),
    .completion_valid_i(completion_valid), .completion_i(completion),
    .rob_head_valid_i(rob_head_valid), .rob_head_index_i(rob_head_index),
    .rob_head_tag_i(rob_head_tag), .mispredict_pending_i(mispredict_pending),
    .mispredict_tag_i(mispredict_tag), .alu0_ready_i(alu0_ready), .alu1_ready_i(alu1_ready),
    .muldiv_ready_i(muldiv_ready), .memory_ready_i(memory_ready), .system_ready_i(system_ready),
    .issue_fire_o(issue_fire), .issue_packet_o(issue_packet),
    .read0_rs1_o(read0_rs1), .read0_rs2_o(read0_rs2),
    .read1_rs1_o(read1_rs1), .read1_rs2_o(read1_rs2),
    .read0_data1_i(read0_data1), .read0_data2_i(read0_data2),
    .read1_data1_i(read1_data1), .read1_data2_i(read1_data2),
    .flush_i(flush), .flush_done_o(flush_done), .occupancy_o(occupancy),
    .occupied_mask_o(occupied_mask), .protocol_error_o(protocol_error)
  );

  function automatic word_t next_random(input word_t state);
    next_random = {state[30:0], state[31] ^ state[21] ^ state[1] ^ state[0]};
  endfunction

  function automatic word_t source_value(input phys_reg_t physical);
    completion_t first, second;
    begin
      first = completion.lane0; second = completion.lane1;
      if (!rst_n || physical == 0) source_value = 0;
      else if (completion_valid[0] && first.writes_rd && !first.exception.valid &&
               first.pdst == physical) source_value = first.value;
      else if (completion_valid[1] && second.writes_rd && !second.exception.valid &&
               second.pdst == physical) source_value = second.value;
      else source_value = ref_values[physical];
    end
  endfunction

  // Test-only PRF/bypass model. Selection is independent of these returned data values.
  always_comb begin
    read0_data1 = source_value(read0_rs1);
    read0_data2 = source_value(read0_rs2);
    read1_data1 = source_value(read1_rs1);
    read1_data2 = source_value(read1_rs2);
  end

  function automatic logic awakened(input logic already, used, input phys_reg_t tag);
    completion_t packet;
    logic result;
    integer lane;
    begin
      result = already || !used || tag == 0;
      for (lane = 0; lane < 2; lane = lane + 1) begin
        packet = lane == 0 ? completion.lane0 : completion.lane1;
        if (completion_valid[lane] && packet.writes_rd && !packet.exception.valid &&
            packet.pdst != 0 && packet.pdst == tag) result = 1;
      end
      awakened = result;
    end
  endfunction

  function automatic integer age_of(input rob_tag_t tag);
    integer index_value;
    begin
      index_value = tag.index;
      age_of = (index_value - rob_head_index + ROB_DEPTH) % ROB_DEPTH;
    end
  endfunction

  function automatic logic duplicate_tag(input rob_tag_t tag);
    issue_entry_t entry;
    integer slot;
    begin
      duplicate_tag = 0;
      for (slot = 0; slot < ISSUE_DEPTH; slot = slot + 1) begin
        entry = ref_entries[slot];
        if (entry.valid && entry.uop.rob_tag == tag) duplicate_tag = 1;
      end
    end
  endfunction

  function automatic logic eligible(input integer slot, port_number);
    issue_entry_t entry;
    logic values_ready, control_allows, supports;
    begin
      entry = ref_entries[slot];
      values_ready = awakened(entry.src1_ready, entry.uop.decoded.uses_rs1, entry.uop.prs1) &&
                     awakened(entry.src2_ready, entry.uop.decoded.uses_rs2, entry.uop.prs2);
      control_allows = rst_n && !flush && !issue_hold && rob_head_valid && entry.valid &&
                       (!mispredict_pending || age_of(entry.uop.rob_tag) < age_of(mispredict_tag));
      supports = 0;
      case (entry.uop.decoded.op_class)
        UOP_INTEGER: supports = port_number == 0 ? alu0_ready : alu1_ready;
        UOP_BRANCH, UOP_JUMP: supports = port_number == 0 && alu0_ready;
        UOP_MULDIV: supports = port_number == 1 && muldiv_ready;
        UOP_LOAD, UOP_STORE: supports = port_number == 0 && memory_ready;
        UOP_SYSTEM: supports = port_number == 0 && system_ready &&
                               entry.uop.rob_tag == rob_head_tag;
        default: supports = 0;
      endcase
      eligible = values_ready && control_allows && supports;
    end
  endfunction

  function automatic renamed_uop_t operation(input integer id, kind, tag_index);
    renamed_uop_t value;
    begin
      value = '0;
      value.decoded.fetch.pc = 32'h100 + id * 4;
      value.decoded.fetch.instruction = 32'h002080b3;
      value.decoded.fetch.prediction.next_pc = value.decoded.fetch.pc + 4;
      // Distinct nonzero metadata detects truncation or accidental field reconstruction.
      value.decoded.fetch.prediction.taken = id % 2;
      value.decoded.fetch.prediction.btb_hit = (id / 2) % 2;
      value.decoded.fetch.prediction.pht_index = id * 7 + 3;
      value.decoded.fetch.exception.cause = 32'hc0000000 + id;
      value.decoded.fetch.exception.tval = 32'hd0000000 + id * 9;
      if (id % 2 == 0) begin
        value.decoded.src_a = SRC_A_PC;
        value.decoded.src_b = SRC_B_IMMEDIATE;
        value.decoded.branch_op = BRANCH_LT;
        value.decoded.mem_size = MEM_HALF;
        value.decoded.csr_op = CSR_RS;
      end else begin
        value.decoded.src_a = SRC_A_ZERO;
        value.decoded.src_b = SRC_B_REGISTER;
        value.decoded.branch_op = BRANCH_GEU;
        value.decoded.mem_size = MEM_WORD;
        value.decoded.csr_op = CSR_RCI;
      end
      value.decoded.load_unsigned = id % 2;
      value.decoded.jump_indirect = (id / 2) % 2;
      value.decoded.rs1 = 1 + (id * 3) % 31;
      value.decoded.rs2 = 1 + (id * 5) % 31;
      value.decoded.immediate = 32'h81230000 + id * 33;
      value.decoded.csr_address = 12'h340 + id % 16;
      value.decoded.csr_immediate = id * 3 + 1;
      value.old_pdst = 1 + (id * 9) % 63;
      value.decoded.op_class = UOP_INTEGER;
      if (id % 2 == 0) value.decoded.alu_op = ALU_AND;
      else value.decoded.alu_op = ALU_XOR;
      value.decoded.uses_rs1 = 1;
      value.decoded.uses_rs2 = 1;
      value.decoded.writes_rd = 1;
      value.decoded.rd = 1 + id % 31;
      value.prs1 = 1 + id % 31;
      value.prs2 = 1 + (id + 7) % 31;
      value.pdst = 32 + id % 32;
      value.rob_tag.index = tag_index;
      value.rob_tag.generation = 9;
      case (kind)
        1: begin
          value.decoded.op_class = UOP_MULDIV;
          if (id % 2 == 0) value.decoded.muldiv_op = MD_DIV;
          else value.decoded.muldiv_op = MD_REMU;
        end
        2: begin
          value.decoded.op_class = UOP_BRANCH;
          value.decoded.writes_rd = 0; value.pdst = 0;
        end
        3: begin value.decoded.op_class = UOP_JUMP; value.decoded.uses_rs2 = 0; end
        4: begin value.decoded.op_class = UOP_LOAD; value.decoded.uses_rs2 = 0; end
        5: begin
          value.decoded.op_class = UOP_STORE;
          value.decoded.writes_rd = 0; value.pdst = 0;
        end
        6: begin
          value.decoded.op_class = UOP_SYSTEM; value.decoded.system_op = SYS_CSR;
          value.decoded.uses_rs2 = 0;
        end
        default: begin end
      endcase
      operation = value;
    end
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 30)
          $display("FAIL: %s time=%0t count=%0d", label_text, $time, occupancy);
      end
    end
  endtask

  task automatic resources_for(input integer mask);
    begin
      alu0_ready = (mask & 1) != 0;
      alu1_ready = (mask & 2) != 0;
      muldiv_ready = (mask & 4) != 0;
      memory_ready = (mask & 8) != 0;
      system_ready = (mask & 16) != 0;
    end
  endtask

  task automatic neutral;
    begin
      dispatch_hold = 0; issue_hold = 0; flush = 0;
      dispatch_valid = 0; dispatch_uop = '0;
      dispatch_src1_ready = 0; dispatch_src2_ready = 0;
      completion_valid = 0; completion = '0;
      rob_head_valid = 1; rob_head_index = 0;
      rob_head_tag = '0; rob_head_tag.generation = 9;
      mispredict_pending = 0; mispredict_tag = '0;
      resources_for(0);
    end
  endtask

  task automatic offer(input renamed_uop_t older, younger, input logic [1:0] mask);
    begin
      dispatch_uop.lane0 = older; dispatch_uop.lane1 = younger;
      dispatch_valid = mask;
      dispatch_src1_ready = 3; dispatch_src2_ready = 3;
    end
  endtask

  // Enumerate every legal assignment, rather than reproduce the DUT's greedy selection.
  // Rank: oldest selected age, two before one, oldest partner, then older on port 0.
  task automatic select_reference;
    integer p0, p1, number, first_age, second_age, a0, a1, tie;
    integer best_first, best_count, best_second, best_tie;
    issue_entry_t e0, e1;
    logic legal, better;
    begin
      selected0 = -1; selected1 = -1;
      best_first = 100; best_count = 0; best_second = 100; best_tie = 100;
      for (p0 = -1; p0 < ISSUE_DEPTH; p0 = p0 + 1) begin
        for (p1 = -1; p1 < ISSUE_DEPTH; p1 = p1 + 1) begin
          legal = (p0 >= 0 || p1 >= 0) && (p0 < 0 || p1 < 0 || p0 != p1);
          e0 = '0; e1 = '0;
          if (p0 >= 0) begin e0 = ref_entries[p0]; legal = legal && eligible(p0, 0); end
          if (p1 >= 0) begin e1 = ref_entries[p1]; legal = legal && eligible(p1, 1); end
          if (p0 >= 0 && p1 >= 0 &&
              (e0.uop.decoded.op_class == UOP_SYSTEM || e1.uop.decoded.op_class == UOP_SYSTEM))
            legal = 0;
          if (legal) begin
            number = (p0 >= 0 ? 1 : 0) + (p1 >= 0 ? 1 : 0);
            a0 = p0 >= 0 ? age_of(e0.uop.rob_tag) : 100;
            a1 = p1 >= 0 ? age_of(e1.uop.rob_tag) : 100;
            first_age = a0 < a1 ? a0 : a1;
            second_age = number == 2 ? (a0 > a1 ? a0 : a1) : 100;
            tie = a0 <= a1 ? 0 : 1;
            better = first_age < best_first ||
                     (first_age == best_first && number > best_count) ||
                     (first_age == best_first && number == best_count &&
                      second_age < best_second) ||
                     (first_age == best_first && number == best_count &&
                      second_age == best_second && tie < best_tie);
            if (better) begin
              selected0 = p0; selected1 = p1;
              best_first = first_age; best_count = number;
              best_second = second_age; best_tie = tie;
            end
          end
        end
      end
    end
  endtask

  task automatic evaluate;
    integer index, free_count, expected_count;
    issue_entry_t entry;
    renamed_uop_t u0, u1;
    phys_reg_t p01, p02, p11, p12;
    logic ready0, ready1, skip0, skip1, bad0, bad1, duplicate0, duplicate1, allow;
    begin
      select_reference;
      expected_issue = 0;
      expected_packets = '0;
      p01 = 0; p02 = 0; p11 = 0; p12 = 0;
      if (selected0 >= 0) begin
        entry = ref_entries[selected0];
        expected_issue[0] = 1;
        expected_packets.lane0.uop = entry.uop;
        if (entry.uop.decoded.uses_rs1) p01 = entry.uop.prs1;
        if (entry.uop.decoded.uses_rs2) p02 = entry.uop.prs2;
        expected_packets.lane0.rs1_value = source_value(p01);
        expected_packets.lane0.rs2_value = source_value(p02);
      end
      if (selected1 >= 0) begin
        entry = ref_entries[selected1];
        expected_issue[1] = 1;
        expected_packets.lane1.uop = entry.uop;
        if (entry.uop.decoded.uses_rs1) p11 = entry.uop.prs1;
        if (entry.uop.decoded.uses_rs2) p12 = entry.uop.prs2;
        expected_packets.lane1.rs1_value = source_value(p11);
        expected_packets.lane1.rs2_value = source_value(p12);
      end
      check(issue_fire == expected_issue, "issue ports match exhaustive legal-assignment oracle");
      check(issue_packet == expected_packets, "selected owners and captured operand values");
      check(read0_rs1 == p01 && read0_rs2 == p02 && read1_rs1 == p11 && read1_rs2 == p12,
            "four PRF selectors belong to the selected execution ports");
      free0 = -1; free1 = -1; free_count = 0; expected_count = 0; expected_mask = 0;
      for (index = 0; index < ISSUE_DEPTH; index = index + 1) begin
        entry = ref_entries[index];
        if (entry.valid) begin expected_count = expected_count + 1; expected_mask[index] = 1; end
        else begin
          if (free0 < 0) free0 = index;
          else if (free1 < 0) free1 = index;
          free_count = free_count + 1;
        end
      end
      check(occupancy == expected_count && occupied_mask == expected_mask,
            "current occupied slots and count match reference");
      u0 = dispatch_uop.lane0; u1 = dispatch_uop.lane1;
      duplicate0 = duplicate_tag(u0.rob_tag); duplicate1 = duplicate_tag(u1.rob_tag);
      skip0 = u0.decoded.fetch.exception.valid; skip1 = u1.decoded.fetch.exception.valid;
      bad0 = u0.decoded.op_class == UOP_NONE && !skip0;
      bad1 = u1.decoded.op_class == UOP_NONE && !skip1;
      allow = rst_n && !flush && !dispatch_hold && !issue_hold && !mispredict_pending;
      ready0 = allow && !duplicate0 && !bad0 && (skip0 || free_count >= 1);
      expected_dispatch[0] = dispatch_valid[0] && ready0;
      ready1 = expected_dispatch[0] && !skip0 && u0.decoded.op_class != UOP_SYSTEM &&
               !duplicate1 && !bad1 && u1.rob_tag != u0.rob_tag && (skip1 || free_count >= 2);
      expected_dispatch[1] = dispatch_valid[1] && ready1;
      check(dispatch_ready == {ready1, ready0} && dispatch_fire == expected_dispatch,
            "admission uses current free capacity and accepts a valid prefix");
      expected_protocol = rst_n && !flush && (dispatch_valid == 2'b10 ||
                          (dispatch_valid[0] && (duplicate0 || bad0)) ||
                          (dispatch_valid[1] && (duplicate1 || bad1)) ||
                          (dispatch_valid == 3 && u0.rob_tag == u1.rob_tag));
      check(protocol_error == expected_protocol, "malformed or duplicate offers are diagnosed");
    end
  endtask

  task automatic update_reference;
    integer index, lane;
    issue_entry_t entry;
    renamed_uop_t value;
    completion_t wb;
    begin
      if (flush) begin
        for (index = 0; index < ISSUE_DEPTH; index = index + 1) ref_entries[index] = '0;
      end else begin
        for (index = 0; index < ISSUE_DEPTH; index = index + 1) begin
          entry = ref_entries[index];
          entry.src1_ready = awakened(entry.src1_ready, entry.uop.decoded.uses_rs1, entry.uop.prs1);
          entry.src2_ready = awakened(entry.src2_ready, entry.uop.decoded.uses_rs2, entry.uop.prs2);
          ref_entries[index] = entry;
        end
        if (selected0 >= 0) begin
          entry = ref_entries[selected0]; entry.valid = 0; ref_entries[selected0] = entry;
        end
        if (selected1 >= 0) begin
          entry = ref_entries[selected1]; entry.valid = 0; ref_entries[selected1] = entry;
        end
        for (lane = 0; lane < 2; lane = lane + 1) begin
          value = lane == 0 ? dispatch_uop.lane0 : dispatch_uop.lane1;
          if (expected_dispatch[lane] && !value.decoded.fetch.exception.valid) begin
            entry = '0; entry.valid = 1; entry.uop = value;
            entry.src1_ready = awakened(dispatch_src1_ready[lane],
                               value.decoded.uses_rs1, value.prs1);
            entry.src2_ready = awakened(dispatch_src2_ready[lane],
                               value.decoded.uses_rs2, value.prs2);
            ref_entries[lane == 0 ? free0 : free1] = entry;
          end
        end
      end
      // Same-edge accepted writes persist in the test-only PRF model; lane 0 wins a collision.
      for (lane = 1; lane >= 0; lane = lane - 1) begin
        wb = lane == 0 ? completion.lane0 : completion.lane1;
        if (completion_valid[lane] && wb.writes_rd && !wb.exception.valid && wb.pdst != 0)
          ref_values[wb.pdst] = wb.value;
      end
    end
  endtask

  task automatic cycle;
    logic applied;
    begin
      #1; evaluate;
      applied = flush;
      update_reference;
      #2; clk = 1; #1;
      check(flush_done == applied, "flush acknowledgement follows the application edge");
      clk = 0; neutral; #1; evaluate;
    end
  endtask

  task automatic reset_dut;
    integer index;
    begin
      clk = 0; neutral; rst_n = 0;
      for (index = 0; index < ISSUE_DEPTH; index = index + 1) ref_entries[index] = '0;
      for (index = 0; index < PHYS_REG_COUNT; index = index + 1)
        ref_values[index] = index == 0 ? 0 : 32'h11000000 + index * 32'h10201;
      #2; check(issue_fire == 0 && dispatch_fire == 0 && !flush_done, "reset dominates activity");
      rst_n = 1; #1; evaluate;
    end
  endtask

  task automatic flush_queue;
    begin neutral; flush = 1; cycle; neutral; cycle; end
  endtask

  task automatic queue_pair(input renamed_uop_t older, younger);
    begin neutral; offer(older, younger, 3); cycle; end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block06/block06.vcd");
      $dumpvars(0, tb_block06);
    end
    #3000000; $fatal(1, "Block 6 timed out");
  end

  initial begin
    reset_dut;
    check($bits(issue_count_t) == 4, "eight-entry occupancy includes full count");
    check($bits(execute_packet_pair_t) == 2 * $bits(execute_packet_t),
          "issue pair carries two complete owner/value records");

    // Oldest INTEGER may use port 1 so a younger port-0-only branch can issue alongside it.
    queue_pair(operation(0, 0, 0), operation(1, 2, 1));
    resources_for(31); #1; evaluate;
    check(issue_fire == 3 && issue_packet.lane0.uop.decoded.op_class == UOP_BRANCH &&
          issue_packet.lane1.uop.rob_tag.index == 0, "flexible ALU moves for branch-only partner");
    cycle;
    check(occupancy == 0, "both accepted issue ports remove their separate owners");

    // An older RAW-blocked instruction does not prevent an independent younger instruction.
    a_uop = operation(2, 0, 0); a_uop.prs1 = 37; a_uop.prs2 = 38;
    b_uop = operation(3, 0, 1);
    offer(a_uop, b_uop, 3); dispatch_src1_ready = 2; dispatch_src2_ready = 2; cycle;
    resources_for(31); #1;
    check(issue_fire == 1 && issue_packet.lane0.uop.rob_tag == b_uop.rob_tag,
          "younger ready operation bypasses an older RAW dependency"); cycle;
    resources_for(31); #1; check(issue_fire == 0, "unfinished source values prevent issue");
    completion_valid = 3;
    completion.lane0.writes_rd = 1; completion.lane0.pdst = 37;
    completion.lane0.value = 99;
    completion.lane1.writes_rd = 1; completion.lane1.pdst = 38;
    completion.lane1.value = 123; #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.rs1_value == 99 &&
          issue_packet.lane0.rs2_value == 123,
          "dual same-cycle wakeup forwards new operand values");
    cycle;

    // Each broadcast can wake several consumers, including both selected instructions.
    a_uop = operation(83, 0, 0); a_uop.prs1 = 42; a_uop.prs2 = 43;
    b_uop = operation(84, 0, 1); b_uop.prs1 = 43; b_uop.prs2 = 42;
    offer(a_uop, b_uop, 3); dispatch_src1_ready = 0; dispatch_src2_ready = 0; cycle;
    completion_valid = 3;
    completion.lane0.writes_rd = 1; completion.lane0.pdst = 42;
    completion.lane0.value = 32'h12345678;
    completion.lane1.writes_rd = 1; completion.lane1.pdst = 43;
    completion.lane1.value = 32'h89abcdef;
    resources_for(31); #1; evaluate;
    check(issue_fire == 3 && issue_packet.lane0.rs1_value == 32'h12345678 &&
          issue_packet.lane0.rs2_value == 32'h89abcdef &&
          issue_packet.lane1.rs1_value == 32'h89abcdef &&
          issue_packet.lane1.rs2_value == 32'h12345678,
          "dual completion wakes both consumers and forwards all four source values"); cycle;

    // Wakeups are sticky even if issue is held; bad/nonwriter/zero completions do not wake.
    a_uop = operation(4, 0, 0); a_uop.prs1 = 40; a_uop.prs2 = 41;
    offer(a_uop, '0, 1); dispatch_src1_ready = 0; dispatch_src2_ready = 0; cycle;
    completion_valid = 1; completion.lane0.writes_rd = 1; completion.lane0.pdst = 40;
    completion.lane0.exception.valid = 1; resources_for(31); #1;
    check(issue_fire == 0, "fault completion does not wake a dependency"); cycle;
    completion_valid = 1; completion.lane0.pdst = 40; completion.lane0.writes_rd = 0; cycle;
    completion_valid = 1; completion.lane0.pdst = 0; completion.lane0.writes_rd = 1; cycle;
    issue_hold = 1; completion_valid = 3;
    completion.lane0.writes_rd = 1; completion.lane0.pdst = 40; completion.lane0.value = 444;
    completion.lane1.writes_rd = 1; completion.lane1.pdst = 41; completion.lane1.value = 555;
    resources_for(31); #1; check(issue_fire == 0, "issue hold blocks awakened transfer"); cycle;
    resources_for(31); #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.rs1_value == 444 &&
          issue_packet.lane0.rs2_value == 555,
          "stored readiness and values survive broadcast removal");
    cycle;

    // Unused/p0 operands are ready even when the incoming ready hints are false.
    a_uop = operation(5, 0, 0); a_uop.decoded.uses_rs1 = 0; a_uop.prs1 = 60;
    a_uop.prs2 = 0;
    offer(a_uop, '0, 1); dispatch_src1_ready = 0; dispatch_src2_ready = 0; cycle;
    resources_for(31); #1; evaluate;
    check(issue_fire == 1 && read0_rs1 == 0 && read0_rs2 == 0,
          "unused field and p0 create no false dependencies"); cycle;

    // A busy iterative engine does not block an eligible operation on the other execution port.
    queue_pair(operation(6, 1, 0), operation(7, 0, 1));
    resources_for(3); #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.uop.decoded.op_class == UOP_INTEGER,
          "busy mul/div leaves younger integer work eligible"); cycle;
    resources_for(4); #1; evaluate;
    check(issue_fire == 2 && issue_packet.lane1.uop.decoded.op_class == UOP_MULDIV,
          "mul/div issues on its port when its own engine becomes available"); cycle;

    // Compare every class pair against all 32 resource masks, including ROB age wrap.
    for (head_case = 0; head_case < 2; head_case = head_case + 1) begin
      for (kind0 = 0; kind0 < 7; kind0 = kind0 + 1) begin
        for (kind1 = 0; kind1 < 7; kind1 = kind1 + 1) begin
          flush_queue;
          rob_head_index = head_case == 0 ? 0 : 15;
          rob_head_tag.index = rob_head_index; rob_head_tag.generation = 9;
          a_uop = operation(10+kind0, kind0, rob_head_index);
          b_uop = operation(20+kind1, kind1, (rob_head_index + 1) % ROB_DEPTH);
          offer(a_uop, b_uop, 3); cycle;
          for (resources = 0; resources < 32; resources = resources + 1) begin
            rob_head_index = head_case == 0 ? 0 : 15;
            rob_head_tag.index = rob_head_index; rob_head_tag.generation = 9;
            resources_for(resources); #1; evaluate;
          end
        end
      end
    end
    flush_queue;

    // SYSTEM needs the full ROB head identity, and it issues alone.
    queue_pair(operation(30, 6, 0), operation(31, 0, 1));
    resources_for(31); rob_head_tag.generation = 10; #1; evaluate;
    check(issue_fire == 0, "wrong-generation system owner cannot issue as the head");
    rob_head_tag.generation = 9; #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.uop.decoded.op_class == UOP_SYSTEM,
          "matching head system instruction takes one exclusive issue slot"); cycle;
    check(occupancy == 0, "lane-0 system admission blocked its younger dispatch partner");
    queue_pair(operation(32, 0, 0), operation(33, 6, 1));
    resources_for(31); #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.uop.decoded.op_class == UOP_INTEGER,
          "younger system instruction waits for actual head ownership"); cycle;
    offer(operation(85, 0, 2), '0, 1); cycle;
    rob_head_index = 1; rob_head_tag.index = 1; resources_for(31); #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.uop.decoded.op_class == UOP_SYSTEM,
          "matching-head system issues alone despite a ready younger integer"); cycle;
    resources_for(31); cycle;

    // Known misprediction drains strict older ages only, including across 15->0 wrap.
    flush_queue;
    rob_head_index = 14; rob_head_tag.index = 14;
    offer(operation(34, 0, 14), operation(35, 1, 15), 3); cycle;
    rob_head_index = 14; rob_head_tag.index = 14;
    offer(operation(36, 0, 0), operation(37, 0, 1), 3); cycle;
    rob_head_index = 14; rob_head_tag.index = 14;
    mispredict_pending = 1; mispredict_tag.index = 0; mispredict_tag.generation = 9;
    resources_for(31); #1; evaluate;
    check(issue_fire == 3 && issue_packet.lane0.uop.rob_tag.index == 14 &&
          issue_packet.lane1.uop.rob_tag.index == 15, "strict older work drains across ROB wrap");
    offer(operation(38, 0, 2), '0, 1); #1;
    check(dispatch_fire == 0, "pending recovery blocks new admission"); cycle;
    rob_head_index = 14; rob_head_tag.index = 14;
    mispredict_pending = 1; mispredict_tag.index = 0; resources_for(31); #1; evaluate;
    check(issue_fire == 0 && occupancy == 2,
          "branch boundary and younger work wait for cancellation");
    flush_queue;

    // A current full queue cannot allocate slots that are being issued on this edge.
    for (n = 0; n < 4; n = n + 1)
      queue_pair(operation(40+n*2, 0, n*2), operation(41+n*2, 0, n*2+1));
    offer(operation(48, 0, 8), operation(49, 0, 9), 3); resources_for(31); #1; evaluate;
    check(issue_fire == 3 && dispatch_fire == 0,
          "full queue uses current capacity, not issue lookahead");
    cycle;
    offer(operation(48, 0, 8), operation(49, 0, 9), 3); #1;
    check(dispatch_fire == 3, "released issue entries accept a pair on the following cycle"); cycle;
    flush_queue;

    // One free entry accepts an older prefix; a fault consumes no issue entry.
    for (n = 0; n < 3; n = n + 1)
      queue_pair(operation(50+n*2, 0, n*2), operation(51+n*2, 0, n*2+1));
    offer(operation(56, 0, 6), '0, 1); cycle;
    offer(operation(57, 0, 7), operation(58, 0, 8), 3); #1;
    check(dispatch_fire == 1, "one free entry truncates a two-instruction offer"); cycle;
    check(occupancy == ISSUE_DEPTH, "accepted prefix fills the final entry");
    a_uop = operation(59, 0, 8); a_uop.decoded.op_class = UOP_NONE;
    a_uop.decoded.fetch.exception.valid = 1;
    offer(a_uop, operation(60, 0, 9), 3); #1; evaluate;
    check(dispatch_fire == 1 && !protocol_error,
          "valid fault is accepted without an issue slot and blocks its younger partner"); cycle;
    check(occupancy == ISSUE_DEPTH, "fault admission does not consume queue storage");
    flush_queue;

    // Fault in lane 1 can follow a real operation with only one physical queue slot needed.
    for (n = 0; n < 3; n = n + 1)
      queue_pair(operation(61+n*2, 0, n*2), operation(62+n*2, 0, n*2+1));
    offer(operation(67, 0, 6), '0, 1); cycle;
    a_uop = operation(68, 0, 7); b_uop = operation(69, 0, 8);
    b_uop.decoded.op_class = UOP_NONE; b_uop.decoded.fetch.exception.valid = 1;
    offer(a_uop, b_uop, 3); #1; evaluate;
    check(dispatch_fire == 3, "younger fault needs no second queue slot"); cycle;
    check(occupancy == ISSUE_DEPTH, "only the real instruction enters the queue"); flush_queue;

    // Duplicate owners and malformed NONE are diagnostics, while raw valid10 never transfers.
    queue_pair(operation(70, 0, 0), operation(71, 0, 1));
    a_uop = operation(72, 0, 0); offer(a_uop, '0, 1); #1; evaluate;
    check(protocol_error && dispatch_fire == 0,
          "existing full tag blocks duplicate admission"); cycle;
    a_uop = operation(73, 0, 2); b_uop = a_uop; offer(a_uop, b_uop, 3); #1;
    check(dispatch_fire == 1 && protocol_error, "duplicate within pair accepts only older owner");
    cycle;
    a_uop = operation(74, 0, 3); a_uop.decoded.op_class = UOP_NONE;
    offer(a_uop, '0, 1); #1;
    check(dispatch_fire == 0 && protocol_error,
          "nonfault NONE is not silently stored forever"); cycle;
    offer(operation(75, 0, 3), operation(76, 0, 4), 2'b10); #1;
    check(dispatch_fire == 0 && protocol_error,
          "younger-only offer never skips an older lane"); cycle;

    // Admission and issue holds, missing ROB context, and flush have explicit boundaries.
    offer(operation(77, 0, 3), '0, 1); dispatch_hold = 1; resources_for(31); #1; evaluate;
    check(dispatch_fire == 0 && issue_fire != 0,
          "admission hold does not stop existing ready work");
    cycle;
    queue_pair(operation(78, 0, 4), operation(79, 0, 5));
    offer(operation(80, 0, 6), '0, 1); issue_hold = 1; resources_for(31); #1;
    check(dispatch_fire == 0 && issue_fire == 0,
          "issue hold freezes both issue and admission"); cycle;
    resources_for(31); rob_head_valid = 0; #1;
    check(issue_fire == 0, "missing live ROB head context prevents age-based issue"); cycle;
    resources_for(31); flush = 1; offer(operation(81, 0, 6), '0, 1); #1;
    check(issue_fire == 0 && dispatch_fire == 0, "safe flush dominates current transfers"); cycle;
    check(occupancy == 0, "flush clears waiting owners");

    // New dispatch cannot issue immediately, but can capture a concurrent accepted wakeup.
    a_uop = operation(82, 0, 0); a_uop.prs1 = 45; a_uop.prs2 = 0;
    offer(a_uop, '0, 1); dispatch_src1_ready = 0; dispatch_src2_ready = 0;
    completion_valid = 2; completion.lane1.writes_rd = 1;
    completion.lane1.pdst = 45; completion.lane1.value = 32'habcdef01;
    resources_for(31); #1;
    check(issue_fire == 0, "no dispatch-to-issue combinational bypass"); cycle;
    resources_for(31); #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.rs1_value == 32'habcdef01,
          "incoming operand readiness captures the same accepted completion"); cycle;

    // Four-entry cases make the independent pair oracle choose among multiple partners.
    for (batch = 0; batch < 32; batch = batch + 1) begin
      flush_queue;
      a_uop = operation(100+batch*4, batch % 6, 14);
      b_uop = operation(101+batch*4, (batch+1) % 6, 15);
      c_uop = operation(102+batch*4, (batch+2) % 6, 0);
      d_uop = operation(103+batch*4, (batch+3) % 6, 1);
      offer(a_uop, b_uop, 3); cycle;
      offer(c_uop, d_uop, 3); cycle;
      for (resources = 0; resources < 32; resources = resources + 1) begin
        rob_head_index = 14; rob_head_tag.index = 14; resources_for(resources);
        #1; evaluate;
      end
    end
    flush_queue;

    // Mixed resource/backpressure/wakeup traffic checks the physical slot bookkeeping.
    for (n = 0; n < 192; n = n + 1) begin
      neutral;
      rng = next_random(rng); resources_for(rng[4:0]);
      issue_hold = n % 19 == 0; dispatch_hold = n % 13 == 0;
      if (n % 5 != 0) begin
        a_uop = operation(300+n*2, n % 6, n % 16);
        b_uop = operation(301+n*2, (n+1) % 6, (n+1) % 16);
        a_uop.rob_tag.generation = 20+n/16;
        b_uop.rob_tag.generation = 20+(n+1)/16;
        offer(a_uop, b_uop, n % 3 == 0 ? 2'b01 : 2'b11);
        dispatch_src1_ready = n % 4 == 0 ? 0 : 3;
        dispatch_src2_ready = n % 7 == 0 ? 0 : 3;
      end
      completion_valid = 3;
      rng = next_random(rng); completion.lane0.writes_rd = 1;
      completion.lane0.pdst = 1 + rng % 31; completion.lane0.value = rng;
      rng = next_random(rng); completion.lane1.writes_rd = 1;
      completion.lane1.pdst = 1 + rng % 31; completion.lane1.value = rng;
      if (n % 47 == 46) flush = 1;
      cycle;
    end
    flush_queue;

    neutral; resources_for(31); offer(operation(900, 0, 0), operation(901, 0, 1), 3);
    flush = 1; rst_n = 0; #2;
    check(issue_fire == 0 && dispatch_fire == 0 && occupancy == 0 && !flush_done,
          "global reset wins held traffic and flush");

    $display("BLOCK06 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Block 6 issue-queue tests failed");
    $finish;
  end
endmodule
