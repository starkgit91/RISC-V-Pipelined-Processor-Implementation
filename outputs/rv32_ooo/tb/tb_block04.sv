// Two-wide rename tests. The driver supplies retirement/completion events.
// The software queue is a test stimulus source, not a hardware ROB.
`timescale 1ns / 1ps

module tb_block04;
  import rv32_ooo_pkg::*;

  logic clk = 0;
  logic rst_n = 1;
  logic [1:0] dispatch_valid, resources_ready, dispatch_ready, dispatch_fire;
  decoded_pair_t decoded;
  rob_tag_pair_t dispatch_tag;
  renamed_pair_t renamed, retire_uop, captured;
  logic [1:0] src1_ready, src2_ready, completion_valid, retire_valid;
  completion_pair_t completion;
  logic recover, recover_done, protocol_error;
  arch_reg_t committed_query;
  phys_reg_t committed_result;
  phys_mask_t free_mask, ready_mask;
  phys_count_t free_count;

  integer ref_spec [0:ARCH_REG_COUNT-1];
  integer ref_commit [0:ARCH_REG_COUNT-1];
  logic ref_free [0:PHYS_REG_COUNT-1];
  logic ref_ready [0:PHYS_REG_COUNT-1];
  integer preview_map [0:ARCH_REG_COUNT-1];
  logic preview_free [0:PHYS_REG_COUNT-1];
  logic preview_ready [0:PHYS_REG_COUNT-1];
  renamed_pair_t expected_pair;
  logic [1:0] expected_ready, expected_fire, expected_src1, expected_src2, captured_fire;
  logic expected_error;
  renamed_uop_t a_uop, b_uop, c_uop, d_uop, saved, temp_uop;
  renamed_uop_t queue [0:127];
  integer queue_head, queue_tail, queue_count;
  integer checks = 0;
  integer failures = 0;
  integer serial = 0;
  integer n, i, j, index, chosen, completed_first, retired_count;
  word_t rng = 32'h53c09a71;
  logic was_recovered;
  decoded_uop_t suffix;

  rv32_rename dut (
    .clk_i(clk), .rst_ni(rst_n),
    .dispatch_valid_i(dispatch_valid), .dispatch_resources_ready_i(resources_ready),
    .decoded_i(decoded), .dispatch_rob_tag_i(dispatch_tag),
    .dispatch_ready_o(dispatch_ready), .dispatch_fire_o(dispatch_fire), .renamed_o(renamed),
    .src1_ready_o(src1_ready), .src2_ready_o(src2_ready),
    .completion_valid_i(completion_valid), .completion_i(completion),
    .retire_valid_i(retire_valid), .retire_uop_i(retire_uop),
    .recover_i(recover), .recover_done_o(recover_done),
    .committed_query_arch_i(committed_query), .committed_query_phys_o(committed_result),
    .free_mask_o(free_mask), .ready_mask_o(ready_mask), .free_count_o(free_count),
    .protocol_error_o(protocol_error)
  );

  function automatic word_t next_random(input word_t state);
    next_random = {state[30:0], state[31] ^ state[21] ^ state[1] ^ state[0]};
  endfunction

  function automatic decoded_uop_t make_writer(input integer rd, rs1, rs2, pc);
    decoded_uop_t value;
    begin
      value = '0;
      value.op_class = UOP_INTEGER;
      value.alu_op = ALU_ADD;
      value.writes_rd = 1;
      value.uses_rs1 = 1;
      value.uses_rs2 = 1;
      value.rd = rd;
      value.rs1 = rs1;
      value.rs2 = rs2;
      value.fetch.pc = pc;
      value.fetch.prediction.next_pc = pc + 4;
      make_writer = value;
    end
  endfunction

  function automatic logic needs_dest(input decoded_uop_t operation);
    needs_dest = operation.writes_rd && operation.rd != 0 && !operation.fetch.exception.valid;
  endfunction

  function automatic integer available_count;
    integer r;
    begin
      available_count = 0;
      for (r = 1; r < PHYS_REG_COUNT; r = r + 1)
        if (ref_free[r]) available_count = available_count + 1;
    end
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 24)
          $display("FAIL: %s time=%0t free=%0d", label_text, $time, free_count);
      end
    end
  endtask

  task automatic neutral;
    begin
      dispatch_valid = 0;
      resources_ready = 2'b11;
      decoded = '0;
      dispatch_tag = '0;
      completion_valid = 0;
      completion = '0;
      retire_valid = 0;
      retire_uop = '0;
      recover = 0;
      committed_query = 0;
    end
  endtask

  task automatic set_lane(input integer lane, input decoded_uop_t operation);
    rob_tag_t tag;
    begin
      tag.index = serial % ROB_DEPTH;
      tag.generation = serial / ROB_DEPTH;
      serial = serial + 1;
      if (lane == 0) begin decoded.lane0 = operation; dispatch_tag.lane0 = tag; end
      else begin decoded.lane1 = operation; dispatch_tag.lane1 = tag; end
      dispatch_valid[lane] = 1;
    end
  endtask

  task automatic complete_lane(input integer lane, input renamed_uop_t operation);
    completion_t result;
    begin
      result = '0;
      result.rob_tag = operation.rob_tag;
      result.pdst = operation.pdst;
      result.writes_rd = operation.decoded.writes_rd;
      result.control_flow.next_pc = operation.decoded.fetch.pc + 4;
      if (lane == 0) completion.lane0 = result;
      else completion.lane1 = result;
      completion_valid[lane] = 1;
    end
  endtask

  task automatic reset_reference;
    integer r;
    begin
      for (r = 0; r < ARCH_REG_COUNT; r = r + 1) begin
        ref_spec[r] = r;
        ref_commit[r] = r;
      end
      for (r = 0; r < PHYS_REG_COUNT; r = r + 1) begin
        ref_free[r] = r >= ARCH_REG_COUNT;
        ref_ready[r] = r < ARCH_REG_COUNT;
      end
    end
  endtask

  // Construct a sequential two-instruction preview using integer maps and boolean tables.
  task automatic predict;
    integer r, lane, candidate;
    decoded_uop_t operation;
    renamed_uop_t result;
    completion_t wb0, wb1;
    logic eligible0, eligible1, collision, need, can_transfer;
    begin
      wb0 = completion.lane0;
      wb1 = completion.lane1;
      eligible0 = completion_valid[0] && wb0.writes_rd && !wb0.exception.valid &&
                  wb0.pdst != 0 && !ref_free[wb0.pdst];
      eligible1 = completion_valid[1] && wb1.writes_rd && !wb1.exception.valid &&
                  wb1.pdst != 0 && !ref_free[wb1.pdst];
      collision = eligible0 && eligible1 && wb0.pdst == wb1.pdst;
      if (collision) eligible1 = 0;
      expected_error = rst_n && (dispatch_valid == 2'b10 || retire_valid == 2'b10 || collision);
      for (r = 0; r < ARCH_REG_COUNT; r = r + 1) preview_map[r] = ref_spec[r];
      for (r = 0; r < PHYS_REG_COUNT; r = r + 1) begin
        preview_free[r] = ref_free[r];
        preview_ready[r] = ref_ready[r];
      end
      if (eligible0) preview_ready[wb0.pdst] = 1;
      if (eligible1) preview_ready[wb1.pdst] = 1;
      expected_pair = '0;
      expected_ready = 0;
      expected_fire = 0;
      for (lane = 0; lane < 2; lane = lane + 1) begin
        operation = lane == 0 ? decoded.lane0 : decoded.lane1;
        need = needs_dest(operation);
        candidate = 0;
        // Reverse priority scan is independent of the RTL's packaged mask helper.
        for (r = PHYS_REG_COUNT-1; r >= 1; r = r - 1)
          if (preview_free[r]) candidate = r;
        can_transfer = lane == 0 ?
          (rst_n && !recover && resources_ready[0]) :
          (expected_fire[0] && resources_ready[1]);
        expected_ready[lane] = can_transfer && (!need || candidate != 0);
        expected_fire[lane] = dispatch_valid[lane] && expected_ready[lane];
        result = '0;
        result.decoded = operation;
        result.decoded.writes_rd = need;
        result.rob_tag = lane == 0 ? dispatch_tag.lane0 : dispatch_tag.lane1;
        if (operation.fetch.exception.valid) begin
          result.decoded.uses_rs1 = 0;
          result.decoded.uses_rs2 = 0;
        end else begin
          if (operation.uses_rs1) result.prs1 = preview_map[operation.rs1];
          if (operation.uses_rs2) result.prs2 = preview_map[operation.rs2];
        end
        if (need) begin
          result.pdst = candidate;
          result.old_pdst = preview_map[operation.rd];
        end
        expected_src1[lane] = result.prs1 == 0 || preview_ready[result.prs1];
        expected_src2[lane] = result.prs2 == 0 || preview_ready[result.prs2];
        if (lane == 0) expected_pair.lane0 = result;
        else expected_pair.lane1 = result;
        if (expected_fire[lane] && need) begin
          preview_map[operation.rd] = candidate;
          preview_free[candidate] = 0;
          preview_ready[candidate] = 0;
        end
      end
    end
  endtask

  task automatic check_comb;
    begin
      predict;
      check(dispatch_ready == expected_ready && dispatch_fire == expected_fire,
            "dispatch transfers an available ordered prefix");
      check(renamed == expected_pair,
            "both renamed records preserve controls and correct mappings");
      check(src1_ready == expected_src1 && src2_ready == expected_src2,
            "both source pairs use completion and intra-pair readiness");
      check(protocol_error == expected_error, "protocol diagnostics match the supplied events");
    end
  endtask

  task automatic update_reference;
    integer r, lane;
    renamed_uop_t value;
    completion_t wb0, wb1;
    logic good0, good1;
    begin
      wb0 = completion.lane0;
      wb1 = completion.lane1;
      good0 = completion_valid[0] && wb0.writes_rd && !wb0.exception.valid &&
              wb0.pdst != 0 && !ref_free[wb0.pdst];
      good1 = completion_valid[1] && wb1.writes_rd && !wb1.exception.valid &&
              wb1.pdst != 0 && !ref_free[wb1.pdst];
      if (good0 && good1 && wb0.pdst == wb1.pdst) good1 = 0;
      if (good0) ref_ready[wb0.pdst] = 1;
      if (good1) ref_ready[wb1.pdst] = 1;
      for (lane = 0; lane < 2; lane = lane + 1) begin
        value = lane == 0 ? retire_uop.lane0 : retire_uop.lane1;
        if (retire_valid[lane] && (lane == 0 || retire_valid[0]) &&
            needs_dest(value.decoded)) begin
          check(value.old_pdst == ref_commit[value.decoded.rd], "ordered retirement history");
          check(ref_ready[value.pdst], "driver retires only a ready physical version");
          ref_commit[value.decoded.rd] = value.pdst;
          if (value.old_pdst != 0) begin
            ref_free[value.old_pdst] = 1;
            ref_ready[value.old_pdst] = 0;
          end
        end
      end
      for (lane = 0; lane < 2; lane = lane + 1) begin
        value = lane == 0 ? expected_pair.lane0 : expected_pair.lane1;
        if (expected_fire[lane] && value.decoded.writes_rd) begin
          ref_spec[value.decoded.rd] = value.pdst;
          ref_free[value.pdst] = 0;
          ref_ready[value.pdst] = 0;
        end
      end
      if (recover) begin
        for (r = 0; r < PHYS_REG_COUNT; r = r + 1) begin ref_free[r] = 1; ref_ready[r] = 0; end
        for (r = 0; r < ARCH_REG_COUNT; r = r + 1) begin
          ref_spec[r] = ref_commit[r];
          ref_free[ref_commit[r]] = 0;
          ref_ready[ref_commit[r]] = 1;
        end
      end
      ref_spec[0] = 0;
      ref_commit[0] = 0;
      ref_free[0] = 0;
      ref_ready[0] = 1;
    end
  endtask

  task automatic check_state;
    integer r;
    begin
      check(free_count == available_count(), "free count matches reference ownership");
      for (r = 0; r < PHYS_REG_COUNT; r = r + 1) begin
        check(free_mask[r] == ref_free[r], "free-list bit");
        check(ready_mask[r] == ref_ready[r], "ready-table bit");
      end
      decoded = '0;
      decoded.lane0.uses_rs1 = 1;
      decoded.lane0.uses_rs2 = 1;
      decoded.lane1.uses_rs1 = 1;
      decoded.lane1.uses_rs2 = 1;
      for (r = 0; r < ARCH_REG_COUNT; r = r + 1) begin
        committed_query = r;
        decoded.lane0.rs1 = r;
        decoded.lane0.rs2 = ARCH_REG_COUNT-1-r;
        decoded.lane1.rs1 = (r+5) % ARCH_REG_COUNT;
        decoded.lane1.rs2 = (r+11) % ARCH_REG_COUNT;
        #1;
        check(committed_result == ref_commit[r], "committed-map query");
        check(renamed.lane0.prs1 == ref_spec[r] &&
              renamed.lane0.prs2 == ref_spec[ARCH_REG_COUNT-1-r], "lane-0 map queries");
        check(renamed.lane1.prs1 == ref_spec[(r+5) % ARCH_REG_COUNT] &&
              renamed.lane1.prs2 == ref_spec[(r+11) % ARCH_REG_COUNT], "lane-1 map queries");
        check(!free_mask[ref_spec[r]] && !free_mask[ref_commit[r]],
              "mapped versions stay reserved");
      end
      check(!free_mask[0] && ready_mask[0], "p0 is reserved and ready");
    end
  endtask

  task automatic cycle;
    logic recovering;
    begin
      #1;
      check_comb;
      captured = renamed;
      captured_fire = dispatch_fire;
      recovering = recover;
      update_reference;
      #2; clk = 1; #1;
      check(recover_done == recovering, "recovery acknowledgement follows the apply edge");
      neutral;
      clk = 0;
      #1;
      check_state;
    end
  endtask

  task automatic reset_dut;
    begin
      clk = 0;
      neutral;
      rst_n = 0;
      reset_reference;
      #2;
      check(!dispatch_fire && !recover_done && !protocol_error, "reset dominates control events");
      rst_n = 1;
      #1;
      check_state;
    end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block04/block04.vcd");
      $dumpvars(0, tb_block04);
    end
    #1000000;
    $fatal(1, "Two-wide block 4 timed out");
  end

  initial begin
    reset_dut;
    check(free_count == 32, "initial spare physical versions");

    // WAW and RAW in one accepted pair: the younger x5 writer reads the new older version.
    neutral;
    set_lane(0, make_writer(5, 1, 2, 0));
    set_lane(1, make_writer(5, 5, 4, 4));
    #1;
    check(renamed.lane0.pdst == 32 && renamed.lane1.pdst == 33 &&
          renamed.lane1.old_pdst == 32, "same-pair x5 writers receive distinct linked versions");
    check(renamed.lane1.prs1 == 32 && !src1_ready[1], "pair RAW waits for lane-0 producer");
    cycle; a_uop = captured.lane0; b_uop = captured.lane1;
    complete_lane(0, a_uop); complete_lane(1, b_uop); cycle;
    retire_valid = 2'b11; retire_uop.lane0 = a_uop; retire_uop.lane1 = b_uop; cycle;
    check(ref_commit[5] == 33 && free_mask[5] && free_mask[32] && !free_mask[33],
          "two ordered x5 retirements leave only the youngest mapping committed");

    // WAR retains an older reader's physical source before a younger writer changes the name.
    neutral;
    set_lane(0, make_writer(6, 5, 1, 8));
    set_lane(1, make_writer(5, 3, 4, 12));
    #1;
    check(renamed.lane0.prs1 == 33 && renamed.lane1.pdst != 33,
          "older reader retains its source across a younger x5 rename"); cycle;
    neutral;
    set_lane(0, make_writer(7, 7, 7, 16));
    set_lane(1, make_writer(7, 7, 7, 20));
    #1;
    check(renamed.lane0.prs1 == 7 && renamed.lane0.prs2 == 7 &&
          renamed.lane1.prs1 == renamed.lane0.pdst && !src1_ready[1],
          "self-read uses old mapping and lane 1 sees the accepted replacement"); cycle;

    // Partial pair acceptance requires compaction of the younger suffix next cycle.
    reset_dut;
    neutral; set_lane(0, make_writer(5, 1, 2, 24));
    suffix = make_writer(6, 5, 3, 28); set_lane(1, suffix); resources_ready = 2'b01;
    #1; check(dispatch_fire == 2'b01, "only the available prefix is accepted"); cycle;
    a_uop = captured.lane0;
    neutral; set_lane(0, suffix); #1;
    check(renamed.lane0.prs1 == a_uop.pdst && !src1_ready[0],
          "compacted suffix retains the true dependency"); cycle;
    b_uop = captured.lane0;
    neutral; set_lane(0, make_writer(8, 1, 2, 32));
    set_lane(1, make_writer(9, 3, 4, 36)); resources_ready = 2'b10;
    #1; check(dispatch_fire == 0, "younger slot cannot bypass unavailable older resources"); cycle;
    neutral; set_lane(1, make_writer(8, 1, 2, 40));
    #1; check(protocol_error && dispatch_fire == 0, "10 dispatch mask is rejected"); cycle;
    neutral; retire_valid = 2'b10; retire_uop.lane1 = b_uop;
    #1; check(protocol_error, "10 retirement mask is diagnosed and ignored"); cycle;

    // Both accepted completion lanes can wake either source of either new instruction.
    reset_dut;
    neutral; set_lane(0, make_writer(5, 1, 2, 44)); set_lane(1, make_writer(6, 3, 4, 48));
    cycle; a_uop = captured.lane0; b_uop = captured.lane1;
    neutral;
    decoded.lane0 = make_writer(0, 5, 6, 52);
    decoded.lane1 = make_writer(0, 6, 5, 56);
    complete_lane(0, a_uop); complete_lane(1, b_uop); #1;
    check(src1_ready == 2'b11 && src2_ready == 2'b11 &&
          !ready_mask[a_uop.pdst] && !ready_mask[b_uop.pdst], "dual readiness bypass before edge");
    cycle;
    neutral; set_lane(0, make_writer(7, 1, 2, 60)); cycle; c_uop = captured.lane0;
    complete_lane(1, c_uop); #1;
    check(!protocol_error, "completion lane 1 alone is legal"); cycle;
    neutral; set_lane(0, make_writer(8, 1, 2, 64)); cycle; d_uop = captured.lane0;
    complete_lane(0, d_uop); completion.lane0.exception.valid = 1; cycle;
    check(!ready_mask[d_uop.pdst], "fault completion does not publish readiness");
    complete_lane(0, d_uop); completion.lane0.writes_rd = 0; cycle;
    check(!ready_mask[d_uop.pdst], "non-register completion is ignored");
    neutral; completion_valid = 2'b11; completion.lane0.writes_rd = 1;
    completion.lane1.writes_rd = 1; completion.lane0.pdst = 63; completion.lane1.pdst = 0;
    cycle; check(!ready_mask[63] && ready_mask[0], "free and p0 result updates are ignored");
    complete_lane(0, d_uop); complete_lane(1, d_uop); #1;
    check(protocol_error, "duplicate physical completion is diagnosed"); cycle;
    check(ready_mask[d_uop.pdst], "lane-0 duplicate priority still marks one version ready");

    // Concurrent two-result, two-retirement, and two-allocation update.
    reset_dut;
    neutral; set_lane(0, make_writer(5, 1, 2, 68)); set_lane(1, make_writer(6, 3, 4, 72));
    cycle; a_uop = captured.lane0; b_uop = captured.lane1;
    complete_lane(0, a_uop); complete_lane(1, b_uop); cycle;
    neutral; set_lane(0, make_writer(7, 1, 2, 76)); set_lane(1, make_writer(8, 3, 4, 80));
    cycle; c_uop = captured.lane0; d_uop = captured.lane1;
    neutral; set_lane(0, make_writer(9, 7, 8, 84)); set_lane(1, make_writer(10, 8, 7, 88));
    complete_lane(0, c_uop); complete_lane(1, d_uop);
    retire_valid = 2'b11; retire_uop.lane0 = a_uop; retire_uop.lane1 = b_uop;
    #1; check(renamed.lane0.pdst == 36 && renamed.lane1.pdst == 37 &&
              src1_ready == 2'b11 && src2_ready == 2'b11,
              "concurrent sources and current-free tags");
    cycle;
    neutral; set_lane(0, make_writer(11, 1, 2, 92)); set_lane(1, make_writer(12, 3, 4, 96));
    #1; check(renamed.lane0.pdst == 5 && renamed.lane1.pdst == 6,
              "two retirement releases become available on the following cycle"); cycle;

    // Exhaustion is unit-only: the future 16-entry ROB would backpressure sooner.
    reset_dut;
    for (n = 0; n < 31; n = n + 1) begin
      neutral; set_lane(0, make_writer(5, 5, 0, 100+n*4)); cycle;
      if (n == 0) a_uop = captured.lane0;
    end
    check(free_count == 1, "one destination remains in allocator stress");
    neutral; set_lane(0, make_writer(6, 1, 2, 228)); set_lane(1, make_writer(7, 3, 4, 232));
    #1; check(dispatch_fire == 2'b01 && renamed.lane0.pdst == 63,
              "one free identity accepts only the first writer"); cycle;
    neutral; set_lane(0, make_writer(7, 3, 4, 232)); #1;
    check(dispatch_fire == 0, "compacted writer waits for a free destination"); cycle;
    neutral; set_lane(0, make_writer(0, 1, 2, 236));
    decoded.lane1 = '0; decoded.lane1.op_class = UOP_BRANCH; dispatch_valid[1] = 1;
    #1; check(dispatch_fire == 2'b11, "two nonwriters can pass a full physical pool"); cycle;
    neutral; set_lane(0, make_writer(8, 1, 2, 240));
    decoded.lane0.fetch.exception.valid = 1;
    decoded.lane0.fetch.exception.cause = CAUSE_ILLEGAL_INSTR;
    decoded.lane0.op_class = UOP_NONE; #1;
    check(dispatch_fire == 2'b01 && renamed.lane0.pdst == 0 && src1_ready[0],
          "valid fault token needs neither a destination nor a source wait"); cycle;
    complete_lane(0, a_uop); cycle;
    neutral; set_lane(0, make_writer(8, 1, 2, 244));
    retire_valid = 2'b01; retire_uop.lane0 = a_uop;
    #1; check(dispatch_fire == 0, "same-edge release is not an allocator bypass"); cycle;
    neutral; set_lane(0, make_writer(8, 1, 2, 244));
    set_lane(1, make_writer(0, 3, 4, 248)); #1;
    check(dispatch_fire == 2'b11 && renamed.lane0.pdst == 5,
          "one writer and one nonwriter fit the newly released identity"); cycle;
    recover = 1; cycle; check(free_count == 32, "safe apply reclaims uncommitted versions");

    // Apply recovery after two simultaneous ordered x5 retirements, including a jump link.
    reset_dut;
    neutral; set_lane(0, make_writer(5, 1, 2, 252)); set_lane(1, make_writer(5, 3, 4, 256));
    decoded.lane1.op_class = UOP_JUMP;
    cycle; a_uop = captured.lane0; b_uop = captured.lane1;
    complete_lane(0, a_uop); complete_lane(1, b_uop); cycle;
    neutral; set_lane(0, make_writer(5, 5, 1, 260)); set_lane(1, make_writer(6, 5, 2, 264));
    cycle;
    neutral; set_lane(0, make_writer(9, 1, 2, 268)); set_lane(1, make_writer(10, 3, 4, 272));
    retire_valid = 2'b11; retire_uop.lane0 = a_uop; retire_uop.lane1 = b_uop;
    recover = 1; #1; check(dispatch_fire == 0, "recovery blocks both dispatch lanes"); cycle;
    check(ref_spec[5] == b_uop.pdst && ref_commit[5] == b_uop.pdst &&
          !free_mask[b_uop.pdst] && free_mask[a_uop.pdst],
          "recovery keeps final retiring link map");
    neutral; cycle; check(!recover_done, "recovery acknowledgement clears after the pulse");

    // Mixed legal traffic uses a small software queue only to drive in-order retirement.
    reset_dut;
    queue_head = 0; queue_tail = 0; queue_count = 0;
    for (n = 0; n < 512; n = n + 1) begin
      neutral;
      completed_first = -1;
      if (queue_count != 0) begin
        rng = next_random(rng);
        chosen = (queue_head + ((rng >> 3) % queue_count)) % 128;
        temp_uop = queue[chosen];
        if (!ref_ready[temp_uop.pdst]) begin
          complete_lane(0, temp_uop); completed_first = chosen;
        end
        for (j = 0; j < queue_count; j = j + 1) begin
          index = (chosen+j+1) % 128;
          // Only choose an entry inside the live circular queue.
          if (index >= queue_head) i = index - queue_head;
          else i = index + 128 - queue_head;
          if (i < queue_count && index != completed_first && !completion_valid[1]) begin
            temp_uop = queue[index];
            if (!ref_ready[temp_uop.pdst]) complete_lane(1, temp_uop);
          end
        end
      end
      if (queue_count != 0 && n % 4 != 0) begin
        temp_uop = queue[queue_head];
        if (ref_ready[temp_uop.pdst]) begin
          retire_valid[0] = 1; retire_uop.lane0 = temp_uop;
          if (queue_count > 1 && n % 3 != 0) begin
            temp_uop = queue[(queue_head+1) % 128];
            if (ref_ready[temp_uop.pdst]) begin
              retire_valid[1] = 1; retire_uop.lane1 = temp_uop;
            end
          end
        end
      end
      rng = next_random(rng);
      if (queue_count < ROB_DEPTH && rng[0]) begin
        set_lane(0, make_writer(1+(rng[9:5] % 31), rng[14:10], rng[19:15], 280+n*8));
        if (queue_count < ROB_DEPTH-1 && rng[1]) begin
          rng = next_random(rng);
          set_lane(1, make_writer(1+(rng[9:5] % 31), rng[14:10], rng[19:15], 284+n*8));
          if (n % 4 == 0) decoded.lane1.rs1 = decoded.lane0.rd;
          if (n % 8 == 0) decoded.lane1.rd = decoded.lane0.rd;
        end
        resources_ready = n % 11 == 0 ? 2'b01 : 2'b11;
      end
      if (n % 79 == 78) recover = 1;
      retired_count = retire_valid[0] + retire_valid[1];
      was_recovered = recover;
      cycle;
      queue_head = (queue_head+retired_count) % 128;
      queue_count = queue_count-retired_count;
      if (captured_fire[0]) begin
        queue[queue_tail] = captured.lane0;
        queue_tail = (queue_tail+1) % 128; queue_count = queue_count+1;
      end
      if (captured_fire[1]) begin
        queue[queue_tail] = captured.lane1;
        queue_tail = (queue_tail+1) % 128; queue_count = queue_count+1;
      end
      if (was_recovered) begin queue_head = 0; queue_tail = 0; queue_count = 0; end
    end

    neutral; set_lane(0, make_writer(5, 1, 2, 5000));
    set_lane(1, make_writer(6, 3, 4, 5004)); recover = 1; #1;
    rst_n = 0; reset_reference; #2;
    check(dispatch_fire == 0 && !recover_done && !protocol_error, "reset wins held pair requests");
    neutral; rst_n = 1; #1; check_state;

    $display("BLOCK04 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Two-wide rename tests failed");
    $finish;
  end
endmodule
