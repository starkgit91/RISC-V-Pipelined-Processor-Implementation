// Block 5 supplies controlled completions; it does not execute DIV/ADD or manage interrupts.
`timescale 1ns / 1ps

module tb_block05;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1, dispatch_hold, head_override, flush;
  logic [1:0] dispatch_valid, dispatch_ready, dispatch_fire;
  logic [1:0] completion_valid, completion_ready, completion_accepted, completion_rejected;
  logic [1:0] head_valid, head_complete, head_ready, retire_fire, trap_fire;
  renamed_pair_t dispatch_uop;
  rob_tag_pair_t dispatch_tag;
  completion_pair_t completion;
  rob_entry_pair_t head_entry;
  retire_trace_pair_t rob_event, injected_event, recorder_event, trace;
  logic recovery_required, mispredict_pending, serial_pending, fault_pending;
  logic flush_done, query_live, full, empty, protocol_error, recorder_error;
  rob_tag_t mispredict_tag, query_tag;
  word_t mispredict_next_pc, trap_target;
  rob_index_t query_age, head_index;
  rob_count_t occupancy;

  // The reference locates completion owners by scanning live records, not indexing by packet tag.
  rob_entry_t ref_entries [0:ROB_DEPTH-1];
  integer ref_generation [0:ROB_DEPTH-1];
  integer ref_head = 0, ref_tail = 0, ref_count = 0;
  logic ref_recovery = 0;
  logic [63:0] next_trace_order = 0;
  logic [1:0] expected_push, expected_accept, expected_reject, expected_pop;
  logic [1:0] expected_retire, expected_trap, expected_head_valid, expected_head_complete;
  logic expected_mispredict, expected_serial, expected_fault, expected_protocol;
  logic expected_completion_conflict;
  rob_tag_t expected_mispredict_tag;
  word_t expected_mispredict_pc;
  retire_trace_pair_t expected_event, expected_trace;
  integer expected_trace_count;
  logic expected_recorder_error;
  renamed_uop_t last0, last1, a_uop, b_uop, c_uop, d_uop, temp_uop;
  renamed_uop_t saved [0:31];
  completion_t temp_completion;
  rob_entry_t temp_entry;
  integer checks = 0, failures = 0;
  integer i, n, selected, serial_id = 0;
  word_t rng = 32'h17a5c0de;

  rv32_reorder_buffer dut (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_hold_i(dispatch_hold),
    .dispatch_valid_i(dispatch_valid), .dispatch_uop_i(dispatch_uop),
    .dispatch_ready_o(dispatch_ready), .dispatch_fire_o(dispatch_fire),
    .dispatch_tag_o(dispatch_tag), .completion_valid_i(completion_valid),
    .completion_i(completion), .completion_ready_o(completion_ready),
    .completion_accepted_o(completion_accepted), .completion_rejected_o(completion_rejected),
    .head_valid_o(head_valid), .head_complete_o(head_complete), .head_entry_o(head_entry),
    .head_ready_i(head_ready), .head_trap_target_i(trap_target),
    .retire_fire_o(retire_fire), .trap_fire_o(trap_fire), .retirement_event_o(rob_event),
    .recovery_required_o(recovery_required), .mispredict_pending_o(mispredict_pending),
    .mispredict_tag_o(mispredict_tag), .mispredict_next_pc_o(mispredict_next_pc),
    .serial_pending_o(serial_pending), .fault_pending_o(fault_pending),
    .flush_i(flush), .flush_done_o(flush_done), .query_tag_i(query_tag),
    .query_live_o(query_live), .query_age_o(query_age), .head_index_o(head_index),
    .occupancy_o(occupancy), .full_o(full), .empty_o(empty), .protocol_error_o(protocol_error)
  );
  assign recorder_event = head_override ? injected_event : rob_event;
  rv32_trace_recorder recorder (
    .clk_i(clk), .rst_ni(rst_n), .event_i(recorder_event),
    .trace_o(trace), .protocol_error_o(recorder_error)
  );

  function automatic word_t next_random(input word_t state);
    next_random = {state[30:0], state[31] ^ state[21] ^ state[1] ^ state[0]};
  endfunction

  function automatic rob_tag_t reference_tag(input integer index);
    rob_tag_t value;
    begin
      value.index = index;
      value.generation = ref_generation[index];
      reference_tag = value;
    end
  endfunction

  function automatic rob_entry_t logical_entry(input integer offset);
    begin
      if (offset < ref_count) logical_entry = ref_entries[(ref_head + offset) % ROB_DEPTH];
      else logical_entry = '0;
    end
  endfunction

  function automatic integer find_owner(input completion_t packet);
    integer index;
    rob_entry_t entry;
    begin
      find_owner = -1;
      for (index = 0; index < ROB_DEPTH; index = index + 1) begin
        entry = ref_entries[index];
        if (entry.valid && !entry.complete && entry.uop.rob_tag == packet.rob_tag &&
            entry.uop.pdst == packet.pdst &&
            entry.uop.decoded.writes_rd == packet.writes_rd) find_owner = index;
      end
    end
  endfunction

  function automatic phys_reg_t unused_physical(input phys_reg_t exclude);
    integer physical, index;
    logic used;
    rob_entry_t entry;
    begin
      unused_physical = 0;
      for (physical = 63; physical >= 32; physical = physical - 1) begin
        used = physical == exclude;
        for (index = 0; index < ROB_DEPTH; index = index + 1) begin
          entry = ref_entries[index];
          if (entry.valid && entry.uop.decoded.writes_rd && entry.uop.pdst == physical) used = 1;
        end
        if (!used) unused_physical = physical;
      end
    end
  endfunction

  function automatic renamed_uop_t writer(input integer id, rd, physical);
    renamed_uop_t value;
    begin
      value = '0;
      value.decoded.fetch.pc = 32'h100 + id * 4;
      value.decoded.fetch.instruction = 32'h00208033 | (rd << 7);
      value.decoded.fetch.prediction.next_pc = value.decoded.fetch.pc + 4;
      value.decoded.op_class = UOP_INTEGER;
      value.decoded.alu_op = ALU_ADD;
      value.decoded.writes_rd = rd != 0;
      value.decoded.rd = rd;
      value.pdst = rd != 0 ? physical : 0;
      value.old_pdst = rd;
      writer = value;
    end
  endfunction

  function automatic completion_t result_for(input renamed_uop_t uop, input word_t value);
    completion_t packet;
    begin
      packet = '0;
      packet.rob_tag = uop.rob_tag;
      packet.pdst = uop.pdst;
      packet.writes_rd = uop.decoded.writes_rd;
      packet.value = value;
      packet.control_flow.next_pc = uop.decoded.fetch.pc + 4;
      result_for = packet;
    end
  endfunction

  function automatic retire_trace_t architectural_event(input rob_entry_t entry);
    retire_trace_t value;
    begin
      value = '0;
      value.valid = 1;
      value.pc = entry.uop.decoded.fetch.pc;
      value.instruction = entry.uop.decoded.fetch.instruction;
      if (entry.exception.valid) begin
        value.trap = 1;
        value.cause = entry.exception.cause;
        value.tval = entry.exception.tval;
        value.next_pc = trap_target;
      end else begin
        value.next_pc = entry.control_flow.next_pc;
        if (entry.uop.decoded.writes_rd && entry.uop.decoded.rd != 0) begin
          value.rd = entry.uop.decoded.rd;
          value.rd_value = entry.result;
        end
        if (entry.memory_access.valid) begin
          value.memory_write = entry.memory_access.write;
          value.memory_address = entry.memory_access.address;
          value.memory_wdata = entry.memory_access.wdata;
          value.memory_wstrb = entry.memory_access.wstrb;
        end
      end
      architectural_event = value;
    end
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 30)
          $display("FAIL: %s time=%0t occupancy=%0d", label_text, $time, occupancy);
      end
    end
  endtask

  task automatic neutral;
    begin
      dispatch_hold = 0; dispatch_valid = 0; dispatch_uop = '0;
      completion_valid = 0; completion = '0; head_ready = 0;
      flush = 0; query_tag = '0; trap_target = 32'h800;
      head_override = 0; injected_event = '0;
    end
  endtask

  task automatic offer(input renamed_uop_t older, younger, input logic [1:0] mask);
    begin
      dispatch_uop.lane0 = older;
      dispatch_uop.lane1 = younger;
      dispatch_uop.lane0.rob_tag = reference_tag(ref_tail);
      dispatch_uop.lane1.rob_tag = reference_tag((ref_tail + 1) % ROB_DEPTH);
      dispatch_valid = mask;
    end
  endtask

  task automatic reference_pending;
    integer offset;
    rob_entry_t entry;
    begin
      expected_mispredict = 0; expected_serial = 0; expected_fault = 0;
      expected_mispredict_tag = '0; expected_mispredict_pc = 0;
      for (offset = 0; offset < ref_count; offset = offset + 1) begin
        entry = logical_entry(offset);
        if (entry.uop.decoded.op_class == UOP_SYSTEM) expected_serial = 1;
        if (entry.exception.valid) expected_fault = 1;
        if (!expected_mispredict && entry.complete && !entry.exception.valid &&
            entry.control_flow.mispredicted) begin
          expected_mispredict = 1;
          expected_mispredict_tag = entry.uop.rob_tag;
          expected_mispredict_pc = entry.control_flow.next_pc;
        end
      end
    end
  endtask

  task automatic evaluate;
    rob_entry_t first, second;
    completion_t c0, c1;
    renamed_uop_t u0, u1;
    logic allowed, ready0, ready1;
    logic serial0, serial1;
    retire_trace_pair_t source;
    begin
      first = logical_entry(0); second = logical_entry(1);
      c0 = completion.lane0; c1 = completion.lane1;
      u0 = dispatch_uop.lane0; u1 = dispatch_uop.lane1;
      reference_pending;
      allowed = rst_n && !flush && !ref_recovery && !dispatch_hold &&
                !expected_mispredict && !expected_serial && !expected_fault;
      ready0 = allowed && ref_count < ROB_DEPTH && u0.rob_tag == reference_tag(ref_tail);
      expected_push[0] = dispatch_valid[0] && ready0;
      ready1 = expected_push[0] && ref_count < ROB_DEPTH-1 &&
               u1.rob_tag == reference_tag((ref_tail + 1) % ROB_DEPTH) &&
               !u0.decoded.fetch.exception.valid && u0.decoded.op_class != UOP_SYSTEM;
      expected_push[1] = dispatch_valid[1] && ready1;
      check(dispatch_tag.lane0 == reference_tag(ref_tail) &&
            dispatch_tag.lane1 == reference_tag((ref_tail + 1) % ROB_DEPTH),
            "candidate identities");
      check(dispatch_ready == {ready1, ready0} && dispatch_fire == expected_push,
            "dispatch accepts only the legal capacity-limited prefix");
      expected_accept[0] = completion_valid[0] && rst_n && !flush && find_owner(c0) >= 0;
      expected_accept[1] = completion_valid[1] && rst_n && !flush && find_owner(c1) >= 0;
      expected_completion_conflict = (&expected_accept) &&
        (c0.rob_tag == c1.rob_tag || (c0.writes_rd && c1.writes_rd &&
         c0.pdst != 0 && c0.pdst == c1.pdst));
      if (expected_completion_conflict) expected_accept[1] = 0;
      expected_reject = completion_valid & {2{rst_n && !flush}} & ~expected_accept;
      check(completion_ready == (rst_n && !flush ? 2'b11 : 2'b00), "raw completion consumption");
      check(completion_accepted == expected_accept && completion_rejected == expected_reject,
            "only live matching unfinished owners accept results");
      expected_head_valid = 0;
      if (rst_n && !flush && !ref_recovery && ref_count > 0) begin
        expected_head_valid[0] = 1;
        expected_head_valid[1] = ref_count > 1;
      end
      expected_head_complete[0] = expected_head_valid[0] && first.complete;
      expected_head_complete[1] = expected_head_valid[1] && second.complete;
      check(head_valid == expected_head_valid && head_complete == expected_head_complete,
            "head visibility and independent completion state");
      check(head_entry.lane0 == (expected_head_valid[0] ? first : '0), "oldest record contents");
      check(head_entry.lane1 == (expected_head_valid[1] ? second : '0), "second record contents");
      serial0 = first.uop.decoded.op_class == UOP_SYSTEM;
      serial1 = second.uop.decoded.op_class == UOP_SYSTEM;
      expected_pop[0] = expected_head_complete[0] && head_ready[0];
      expected_pop[1] = expected_pop[0] && expected_head_complete[1] && head_ready[1] &&
                        !first.exception.valid && !first.control_flow.mispredicted &&
                        !serial0 && !serial1;
      expected_retire[0] = expected_pop[0] && !first.exception.valid;
      expected_retire[1] = expected_pop[1] && !second.exception.valid;
      expected_trap[0] = expected_pop[0] && first.exception.valid;
      expected_trap[1] = expected_pop[1] && second.exception.valid;
      check(retire_fire == expected_retire && trap_fire == expected_trap,
            "architectural events preserve the oldest prefix and precise fault boundary");
      expected_event = '0;
      if (expected_pop[0]) expected_event.lane0 = architectural_event(first);
      if (expected_pop[1]) expected_event.lane1 = architectural_event(second);
      check(rob_event == expected_event, "retirement event values and suppressed trap effects");
      expected_protocol = rst_n && (dispatch_valid == 2'b10 || head_ready == 2'b10 ||
                                   expected_completion_conflict);
      check(protocol_error == expected_protocol, "ROB protocol diagnostics");
      check(mispredict_pending == (rst_n && !flush && expected_mispredict) &&
            serial_pending == (rst_n && !flush && expected_serial) &&
            fault_pending == (rst_n && !flush && expected_fault), "pending control scan");
      if (rst_n && !flush && expected_mispredict)
        check(mispredict_tag == expected_mispredict_tag &&
              mispredict_next_pc == expected_mispredict_pc,
              "oldest resolved misprediction has priority");
      source = head_override ? injected_event : expected_event;
      expected_recorder_error = rst_n && !source.lane0.valid && source.lane1.valid;
      check(recorder_error == expected_recorder_error, "trace prefix protocol diagnostic");
      expected_trace = '0;
      expected_trace_count = 0;
      if (!expected_recorder_error && source.lane0.valid) begin
        expected_trace.lane0 = source.lane0;
        expected_trace.lane0.order = next_trace_order;
        expected_trace_count = 1;
        if (source.lane1.valid) begin
          expected_trace.lane1 = source.lane1;
          expected_trace.lane1.order = next_trace_order + 1;
          expected_trace_count = 2;
        end
      end
    end
  endtask

  task automatic update_reference;
    integer lane, owner, index, pops, pushes;
    completion_t packet;
    renamed_uop_t operation;
    rob_entry_t entry, old0, old1;
    begin
      old0 = logical_entry(0); old1 = logical_entry(1);
      if (flush) begin
        for (index = 0; index < ROB_DEPTH; index = index + 1) ref_entries[index] = '0;
        ref_head = 0; ref_tail = 0; ref_count = 0; ref_recovery = 0;
      end else begin
        for (lane = 0; lane < 2; lane = lane + 1) begin
          packet = lane == 0 ? completion.lane0 : completion.lane1;
          if (expected_accept[lane]) begin
            owner = find_owner(packet);
            entry = ref_entries[owner];
            entry.complete = 1;
            entry.result = packet.value;
            entry.exception = packet.exception;
            entry.control_flow = packet.control_flow;
            entry.memory_access = packet.memory_access;
            ref_entries[owner] = entry;
          end
        end
        pops = (expected_pop[0] ? 1 : 0) + (expected_pop[1] ? 1 : 0);
        if (expected_pop[0]) begin
          ref_entries[ref_head] = '0;
          if (old0.exception.valid || old0.control_flow.mispredicted ||
              old0.uop.decoded.system_op == SYS_MRET ||
              old0.uop.decoded.system_op == SYS_FENCE_I) ref_recovery = 1;
        end
        if (expected_pop[1]) begin
          ref_entries[(ref_head+1) % ROB_DEPTH] = '0;
          if (old1.exception.valid || old1.control_flow.mispredicted) ref_recovery = 1;
        end
        ref_head = (ref_head + pops) % ROB_DEPTH;
        pushes = (expected_push[0] ? 1 : 0) + (expected_push[1] ? 1 : 0);
        for (lane = 0; lane < pushes; lane = lane + 1) begin
          operation = lane == 0 ? dispatch_uop.lane0 : dispatch_uop.lane1;
          entry = '0;
          entry.valid = 1;
          entry.uop = operation;
          entry.exception = operation.decoded.fetch.exception;
          entry.complete = entry.exception.valid;
          entry.control_flow.next_pc = operation.decoded.fetch.pc + 4;
          ref_entries[(ref_tail + lane) % ROB_DEPTH] = entry;
          ref_generation[(ref_tail + lane) % ROB_DEPTH] =
            (ref_generation[(ref_tail + lane) % ROB_DEPTH] + 1) % 256;
        end
        ref_tail = (ref_tail + pushes) % ROB_DEPTH;
        ref_count = ref_count + pushes - pops;
      end
      next_trace_order = next_trace_order + expected_trace_count;
    end
  endtask

  task automatic check_state;
    integer index, age;
    rob_entry_t entry;
    begin
      check(occupancy == ref_count && full == (ref_count == ROB_DEPTH) && empty == (ref_count == 0),
            "occupancy and full/empty state");
      check(head_index == ref_head && recovery_required == ref_recovery, "head and recovery state");
      for (index = 0; index < ROB_DEPTH; index = index + 1) begin
        entry = ref_entries[index];
        query_tag = entry.valid ? entry.uop.rob_tag : reference_tag(index);
        #1;
        check(query_live == entry.valid, "live identity query");
        age = entry.valid ? (index - ref_head + ROB_DEPTH) % ROB_DEPTH : 0;
        check(query_age == age, "age relative to current head");
      end
    end
  endtask

  task automatic cycle;
    logic apply_flush;
    begin
      #1; evaluate;
      if (expected_push[0]) last0 = dispatch_uop.lane0;
      if (expected_push[1]) last1 = dispatch_uop.lane1;
      apply_flush = flush;
      update_reference;
      #2; clk = 1; #1;
      check(trace == expected_trace,
            "registered trace has accepted fields and monotonic event order");
      check(flush_done == apply_flush, "safe flush acknowledgement follows the apply edge");
      clk = 0; neutral; #1; check_state;
    end
  endtask

  task automatic reset_dut;
    integer index;
    begin
      clk = 0; neutral; rst_n = 0;
      ref_head = 0; ref_tail = 0; ref_count = 0; ref_recovery = 0; next_trace_order = 0;
      for (index = 0; index < ROB_DEPTH; index = index + 1) begin
        ref_entries[index] = '0; ref_generation[index] = 0;
      end
      #2;
      check(trace == '0 && !recovery_required && !flush_done,
            "global reset clears control and trace");
      rst_n = 1; #1; check_state;
    end
  endtask

  task automatic flush_dut;
    begin neutral; flush = 1; cycle; neutral; cycle; end
  endtask

  task automatic queue_pair(input renamed_uop_t older, younger);
    begin neutral; offer(older, younger, 2'b11); cycle; end
  endtask

  task automatic queue_single(input renamed_uop_t operation);
    begin neutral; offer(operation, '0, 2'b01); cycle; end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block05/block05.vcd");
      $dumpvars(0, tb_block05);
    end
    #2000000;
    $fatal(1, "Block 5 timed out");
  end

  initial begin
    reset_dut;

    // Lead example: controlled DIV completion is late; the younger ADD is already finished.
    a_uop = writer(0, 5, 32); a_uop.decoded.op_class = UOP_MULDIV;
    a_uop.decoded.muldiv_op = MD_DIV;
    b_uop = writer(1, 6, 33);
    queue_pair(a_uop, b_uop); a_uop = last0; b_uop = last1;
    completion_valid = 2'b10; completion.lane1 = result_for(b_uop, 19); cycle;
    check(head_complete == 2'b10 && occupancy == 2, "younger ADD completes before older DIV");
    head_ready = 2'b11; #1;
    check(retire_fire == 0 && trap_fire == 0, "completed younger instruction cannot skip DIV");
    cycle;
    completion_valid = 2'b01; completion.lane0 = result_for(a_uop, 6); head_ready = 2'b11;
    #1; check(retire_fire == 0, "no same-edge completion-to-retirement bypass"); cycle;
    head_ready = 2'b11; #1;
    check(retire_fire == 2'b11, "both completed instructions retire in order");
    cycle;
    check(trace.lane0.order == 0 && trace.lane1.order == 1 &&
          trace.lane0.rd_value == 6 && trace.lane1.rd_value == 19,
          "DIV then ADD appear as ordered architectural trace events");
    check(empty, "dual retirement empties the two-instruction ROB");
    cycle;
    check(trace == '0, "idle trace contains no accidental repeat event");

    // Same architectural destination still has two independent physical versions/results.
    a_uop = writer(2, 5, 34); b_uop = writer(3, 5, 35); b_uop.old_pdst = 34;
    queue_pair(a_uop, b_uop); a_uop = last0; b_uop = last1;
    completion_valid = 2'b11;
    completion.lane0 = result_for(b_uop, 200); completion.lane1 = result_for(a_uop, 100); cycle;
    head_ready = 2'b11; cycle;
    check(trace.lane0.order == 2 && trace.lane1.order == 3 &&
          trace.lane0.rd == 5 && trace.lane1.rd == 5 &&
          trace.lane0.rd_value == 100 && trace.lane1.rd_value == 200,
          "WAW versions retire in instruction order rather than completion-lane order");

    // Mismatched and duplicate packets are consumed without changing live results.
    reset_dut;
    queue_pair(writer(0, 5, 32), writer(1, 6, 33)); a_uop = last0; b_uop = last1;
    completion_valid = 2'b01; completion.lane0 = result_for(a_uop, 11);
    completion.lane0.rob_tag.generation = a_uop.rob_tag.generation + 1;
    #1; check(completion_rejected == 1, "wrong generation is rejected"); cycle;
    completion_valid = 1; completion.lane0 = result_for(a_uop, 11); completion.lane0.pdst = 50;
    cycle;
    completion_valid = 1; completion.lane0 = result_for(a_uop, 11); completion.lane0.writes_rd = 0;
    cycle;
    completion_valid = 2'b11; completion.lane0 = result_for(a_uop, 11);
    completion.lane1 = result_for(a_uop, 99); #1;
    check(completion_accepted == 1 && completion_rejected == 2 && protocol_error,
          "duplicate completion owner gives lane 0 priority"); cycle;
    completion_valid = 1; completion.lane0 = result_for(a_uop, 77); cycle;
    completion_valid = 2'b10; completion.lane1 = result_for(b_uop, 22); cycle;
    head_ready = 2'b11; cycle;
    check(trace.lane0.rd_value == 11 && trace.lane1.rd_value == 22,
          "rejected duplicates do not overwrite completed results");
    completion_valid = 1; completion.lane0 = result_for(a_uop, 88); cycle;
    check(empty, "completion after retirement cannot resurrect an owner");

    // Deliberately violate rename uniqueness to check the ROB's second-write protection.
    reset_dut;
    queue_pair(writer(0, 5, 32), writer(1, 6, 32)); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 11);
    completion.lane1 = result_for(b_uop, 22); #1;
    check(completion_accepted == 1 && completion_rejected == 2 && protocol_error,
          "distinct owners cannot publish two physical writes to the same destination");
    cycle; flush_dut;

    // Explicit protocol violations cannot create a younger-only architectural transfer.
    offer(writer(2, 7, 34), writer(3, 8, 35), 2'b10); #1;
    check(dispatch_fire == 0 && protocol_error, "younger-only dispatch is rejected"); cycle;
    offer(writer(2, 7, 34), '0, 2'b01); dispatch_uop.lane0.rob_tag.generation = 99;
    #1; check(!dispatch_ready[0], "dispatch identity must match the offered ROB slot"); cycle;
    queue_pair(writer(2, 7, 34), writer(3, 8, 35)); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 7);
    completion.lane1 = result_for(b_uop, 8); cycle;
    head_ready = 2'b10; #1;
    check(retire_fire == 0 && protocol_error, "head ready cannot skip lane 0");
    cycle;
    dispatch_hold = 1; offer(writer(4, 9, 36), '0, 1); #1;
    check(dispatch_fire == 0, "external dispatch hold does not consume a ROB slot");
    head_ready = 1; cycle;
    check(occupancy == 1, "one-ready mask retires only the older instruction");
    head_ready = 3; cycle;

    // Capacity and generation state at ring wrap; no same-edge reuse when currently full.
    reset_dut;
    for (n = 0; n < 8; n = n + 1) begin
      queue_pair(writer(n*2, 1+n*2, 32+n*2), writer(n*2+1, 2+n*2, 33+n*2));
      saved[n*2] = last0; saved[n*2+1] = last1;
    end
    check(full && occupancy == 16, "sixteen live instructions fill the ROB");
    completion_valid = 3; completion.lane0 = result_for(saved[0], 1);
    completion.lane1 = result_for(saved[1], 2); cycle;
    offer(writer(16, 20, 48), writer(17, 21, 49), 3); head_ready = 3;
    #1; check(dispatch_fire == 0 && retire_fire == 3, "full ROB does not borrow retiring slots");
    cycle;
    check(occupancy == 14, "two retired entries release capacity for the following cycle");
    queue_pair(writer(16, 20, 48), writer(17, 21, 49)); c_uop = last0;
    temp_uop = saved[0];
    check(c_uop.rob_tag.index == 0 && c_uop.rob_tag.generation != temp_uop.rob_tag.generation,
          "slot reuse advances its allocation generation");
    completion_valid = 1; completion.lane0 = result_for(saved[0], 123); cycle;
    check(full, "late old-generation result does not consume or change a new owner");
    flush_dut;
    queue_single(writer(18, 22, 50)); a_uop = last0;
    check(a_uop.rob_tag.generation > c_uop.rob_tag.generation,
          "safe flush retains per-slot generation history");
    completion_valid = 1; completion.lane0 = result_for(c_uop, 123); cycle;
    flush_dut;

    // Fifteen occupied entries allow an older-only dispatch prefix and later compaction.
    reset_dut;
    for (n = 0; n < 7; n = n + 1) begin
      queue_pair(writer(n*2, 1+n*2, 32+n*2), writer(n*2+1, 2+n*2, 33+n*2));
      saved[n*2] = last0; saved[n*2+1] = last1;
    end
    queue_single(writer(14, 15, 46));
    offer(writer(15, 16, 47), writer(16, 17, 48), 3); #1;
    check(dispatch_fire == 1, "one free entry accepts only the older dispatch lane"); cycle;
    completion_valid = 1; completion.lane0 = result_for(saved[0], 1); cycle;
    head_ready = 1; cycle;
    queue_single(writer(16, 17, 48));
    check(last0.decoded.fetch.pc == 32'h140, "stalled suffix can be compacted and accepted once");
    flush_dut;

    // Three independent update paths in the same clock edge.
    queue_pair(writer(0, 5, 32), writer(1, 6, 33)); a_uop = last0; b_uop = last1;
    queue_pair(writer(2, 7, 34), writer(3, 8, 35)); c_uop = last0; d_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 1);
    completion.lane1 = result_for(b_uop, 2); cycle;
    offer(writer(4, 9, 36), writer(5, 10, 37), 3);
    completion_valid = 3; completion.lane0 = result_for(c_uop, 3);
    completion.lane1 = result_for(d_uop, 4); head_ready = 3; cycle;
    check(occupancy == 4 && trace.lane0.rd_value == 1 && trace.lane1.rd_value == 2,
          "two pushes, completions, and retirements coexist without slot conflicts");
    flush_dut;

    // A fault at the oldest instruction suppresses every younger architectural effect.
    reset_dut;
    queue_pair(writer(0, 5, 32), writer(1, 6, 33)); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 77);
    completion.lane0.exception.valid = 1; completion.lane0.exception.cause = CAUSE_LOAD_ACCESS;
    completion.lane0.exception.tval = 32'hbad00000;
    completion.lane0.memory_access.valid = 1; completion.lane0.memory_access.write = 1;
    completion.lane1 = result_for(b_uop, 88); cycle;
    check(fault_pending && completion_accepted == 0,
          "registered fault is pending after acceptance");
    head_ready = 3; #1;
    check(trap_fire == 1 && retire_fire == 0, "oldest fault prevents younger retirement"); cycle;
    check(trace.lane0.trap && trace.lane0.rd == 0 && !trace.lane0.memory_write &&
          trace.lane0.next_pc == 32'h800 && !trace.lane1.valid,
          "fault trace reports target and suppresses GPR/store effects");
    head_ready = 3; offer(writer(2, 7, 34), '0, 1); #1;
    check(head_valid == 0 && dispatch_fire == 0 && recovery_required,
          "held readiness cannot retire younger work before safe recovery"); cycle;
    flush_dut;

    // The older success can retire before a second-slot precise fault in the same edge.
    reset_dut;
    queue_pair(writer(0, 5, 32), writer(1, 6, 33)); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 42);
    completion.lane1 = result_for(b_uop, 99); completion.lane1.exception.valid = 1;
    completion.lane1.exception.cause = CAUSE_ILLEGAL_INSTR;
    completion.lane1.exception.tval = b_uop.decoded.fetch.instruction; cycle;
    head_ready = 3; #1;
    check(retire_fire == 1 && trap_fire == 2, "second-slot fault preserves older retirement");
    cycle;
    check(trace.lane0.rd_value == 42 && !trace.lane0.trap && trace.lane1.trap &&
          trace.lane1.order == trace.lane0.order+1,
          "retirement then trap have distinct ordered events");
    flush_dut;

    // A valid decode fault needs no execution completion, and stops the pair at that instruction.
    reset_dut;
    a_uop = writer(0, 0, 0); a_uop.decoded.op_class = UOP_NONE;
    a_uop.decoded.fetch.exception.valid = 1;
    a_uop.decoded.fetch.exception.cause = CAUSE_ILLEGAL_INSTR;
    a_uop.decoded.fetch.exception.tval = 32'hffffffff;
    offer(a_uop, writer(1, 5, 32), 3); #1;
    check(dispatch_fire == 1, "decode fault truncates a dispatch pair"); cycle;
    check(head_complete == 1 && fault_pending, "decode fault is complete-for-trap, not a bubble");
    head_ready = 3; cycle; check(trace.lane0.trap, "decode fault reaches architectural trap event");
    flush_dut;

    // An older late-resolving misprediction overrides a younger pending branch.
    reset_dut;
    a_uop = writer(0, 0, 0); a_uop.decoded.op_class = UOP_BRANCH;
    b_uop = writer(1, 0, 0); b_uop.decoded.op_class = UOP_BRANCH;
    queue_pair(a_uop, b_uop); a_uop = last0; b_uop = last1;
    queue_single(writer(2, 5, 32)); c_uop = last0;
    completion_valid = 2; completion.lane1 = result_for(b_uop, 0);
    completion.lane1.control_flow.mispredicted = 1;
    completion.lane1.control_flow.next_pc = 32'h500; cycle;
    check(mispredict_tag == b_uop.rob_tag, "younger resolved misprediction is initially pending");
    offer(writer(3, 6, 33), '0, 1); #1;
    check(dispatch_fire == 0, "known misprediction freezes allocation");
    completion_valid = 1; completion.lane0 = result_for(a_uop, 0);
    completion.lane0.control_flow.mispredicted = 1;
    completion.lane0.control_flow.next_pc = 32'h400; cycle;
    check(mispredict_tag == a_uop.rob_tag && mispredict_next_pc == 32'h400,
          "older branch takes recovery priority when it resolves later");
    head_ready = 3; #1; check(retire_fire == 1, "mispredicted first slot prevents younger commit");
    cycle;
    completion_valid = 1; completion.lane0 = result_for(c_uop, 7); #1;
    check(completion_accepted == 1 && head_valid == 0,
          "held results may drain while architectural retirement is recovery-blocked"); cycle;
    flush_dut;
    completion_valid = 1; completion.lane0 = result_for(c_uop, 7); cycle;

    // System instructions serialize dispatch and retire alone; MRET/FENCE.I require front flush.
    reset_dut;
    a_uop = writer(0, 0, 0); a_uop.decoded.op_class = UOP_SYSTEM;
    a_uop.decoded.system_op = SYS_FENCE;
    offer(a_uop, writer(1, 5, 32), 3); #1;
    check(dispatch_fire == 1, "system instruction in lane 0 truncates dispatch"); cycle;
    a_uop = last0;
    offer(writer(1, 5, 32), '0, 1); #1;
    check(serial_pending && dispatch_fire == 0, "queued system instruction serializes admission");
    completion_valid = 1; completion.lane0 = result_for(a_uop, 0); cycle;
    head_ready = 3; cycle;
    check(!recovery_required, "ordinary FENCE retires without forced recovery");
    queue_pair(writer(2, 5, 32), a_uop); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 10);
    completion.lane1 = result_for(b_uop, 0); cycle;
    head_ready = 3; #1;
    check(retire_fire == 1, "a system instruction cannot retire in slot 1"); cycle;
    head_ready = 3; cycle;
    for (n = 0; n < 2; n = n + 1) begin
      a_uop = writer(4+n, 0, 0); a_uop.decoded.op_class = UOP_SYSTEM;
      if (n == 0) a_uop.decoded.system_op = SYS_FENCE_I;
      else a_uop.decoded.system_op = SYS_MRET;
      queue_single(a_uop); a_uop = last0;
      completion_valid = 1; completion.lane0 = result_for(a_uop, 0); cycle;
      head_ready = 3; cycle;
      check(recovery_required, "FENCE.I/MRET retirement requires a safe front-end flush");
      flush_dut;
    end

    // Successful stores trace the acknowledged access; fault events suppress that effect.
    reset_dut;
    a_uop = writer(0, 0, 0); a_uop.decoded.op_class = UOP_STORE;
    queue_single(a_uop); a_uop = last0;
    completion_valid = 1; completion.lane0 = result_for(a_uop, 0);
    completion.lane0.memory_access.valid = 1; completion.lane0.memory_access.write = 1;
    completion.lane0.memory_access.address = 32'h100;
    completion.lane0.memory_access.wdata = 32'h00ab0000;
    completion.lane0.memory_access.wstrb = 4'b0100; cycle;
    head_ready = 3; cycle;
    check(trace.lane0.memory_write && trace.lane0.memory_address == 32'h100 &&
          trace.lane0.memory_wdata == 32'h00ab0000 && trace.lane0.memory_wstrb == 4'b0100,
          "store trace preserves byte-address, shifted data, and byte strobes");
    a_uop = writer(1, 0, 0); a_uop.decoded.op_class = UOP_STORE;
    queue_single(a_uop); a_uop = last0;
    completion_valid = 1; completion.lane0 = result_for(a_uop, 0);
    completion.lane0.exception.valid = 1; completion.lane0.exception.cause = CAUSE_STORE_ACCESS;
    completion.lane0.exception.tval = 32'h100;
    completion.lane0.memory_access.valid = 1; completion.lane0.memory_access.write = 1; cycle;
    head_ready = 3; cycle;
    check(trace.lane0.trap && !trace.lane0.memory_write && trace.lane0.memory_wstrb == 0,
          "faulted store produces a trap, not a successful-store trace");
    flush_dut;

    // Safe flush dominates traffic and does not reset architectural event ordering.
    queue_pair(writer(2, 5, 32), writer(3, 6, 33)); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 3);
    completion.lane1 = result_for(b_uop, 4); cycle;
    offer(writer(4, 7, 34), writer(5, 8, 35), 3); completion_valid = 3;
    completion.lane0 = result_for(a_uop, 8); completion.lane1 = result_for(b_uop, 9);
    head_ready = 3; flush = 1; #1;
    check(dispatch_fire == 0 && completion_ready == 0 && retire_fire == 0 && trap_fire == 0,
          "safe flush dominates every instruction handshake"); cycle;
    check(empty && !recovery_required, "safe apply empties all speculative state");
    head_override = 1; injected_event.lane1.valid = 1; #1;
    check(recorder_error, "trace recorder rejects a younger-only event prefix"); cycle;
    head_override = 1; injected_event.lane0.valid = 1;
    injected_event.lane0.trap = 1; injected_event.lane0.interrupt = 1;
    injected_event.lane0.pc = 32'h44; injected_event.lane0.next_pc = 32'h800;
    injected_event.lane0.cause = CAUSE_IRQ_EXTERNAL;
    injected_event.lane0.order = 64'hffffffff; cycle;
    check(trace.lane0.interrupt && trace.lane0.cause == CAUSE_IRQ_EXTERNAL,
          "recorder can stamp an externally selected IRQ-format event without an IRQ controller");

    // Mixed traffic uses the independent live-record model; no ISA execution is being inferred.
    reset_dut; serial_id = 0;
    for (n = 0; n < 256; n = n + 1) begin
      neutral;
      rng = next_random(rng);
      if (ref_count != 0 && rng[0]) begin
        selected = (rng >> 4) % ref_count;
        temp_entry = logical_entry(selected);
        completion_valid[0] = 1;
        completion.lane0 = result_for(temp_entry.uop, rng);
      end
      rng = next_random(rng);
      if (ref_count != 0 && rng[0]) begin
        selected = (rng >> 4) % ref_count;
        temp_entry = logical_entry(selected);
        completion_valid[1] = 1;
        completion.lane1 = result_for(temp_entry.uop, rng);
      end
      if (n % 4 == 0) head_ready = 0;
      else if (n % 4 == 1) head_ready = 1;
      else head_ready = 3;
      rng = next_random(rng);
      if (rng[2:0] != 0) begin
        a_uop = writer(serial_id, 1 + serial_id % 31, unused_physical(0));
        serial_id = serial_id + 1;
        b_uop = writer(serial_id, 1 + serial_id % 31, unused_physical(a_uop.pdst));
        serial_id = serial_id + 1;
        offer(a_uop, b_uop, n % 3 == 0 ? 2'b01 : 2'b11);
      end
      dispatch_hold = n % 17 == 0;
      if (n % 71 == 70) flush = 1;
      cycle;
    end

    // Reset wins over held requests and clears the event-order sequence.
    neutral; offer(writer(900, 5, 32), writer(901, 6, 33), 3);
    head_ready = 3; flush = 1; rst_n = 0; #2;
    check(dispatch_fire == 0 && completion_ready == 0 && retire_fire == 0 && trace == '0,
          "asynchronous global reset dominates active requests");

    $display("BLOCK05 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Block 5 ROB/retirement/trace tests failed");
    $finish;
  end
endmodule
