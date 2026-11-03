// Two-wide WB/commit: allocate in program order, complete by tag, retire an ordered prefix.
// Safe flush applies only after the controller has cancelled/drained younger work.
`timescale 1ns / 1ps

module rv32_reorder_buffer (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  input  logic                         dispatch_hold_i,
  input  logic [1:0]                   dispatch_valid_i,
  input  rv32_ooo_pkg::renamed_pair_t   dispatch_uop_i,
  output logic [1:0]                   dispatch_ready_o,
  output logic [1:0]                   dispatch_fire_o,
  output rv32_ooo_pkg::rob_tag_pair_t   dispatch_tag_o,
  input  logic [1:0]                   completion_valid_i,
  input  rv32_ooo_pkg::completion_pair_t completion_i,
  output logic [1:0]                   completion_ready_o,
  output logic [1:0]                   completion_accepted_o,
  output logic [1:0]                   completion_rejected_o,
  output logic [1:0]                   head_valid_o,
  output logic [1:0]                   head_complete_o,
  output rv32_ooo_pkg::rob_entry_pair_t head_entry_o,
  input  logic [1:0]                   head_ready_i,
  input  rv32_ooo_pkg::word_t           head_trap_target_i,
  output logic [1:0]                   retire_fire_o,
  output logic [1:0]                   trap_fire_o,
  output rv32_ooo_pkg::retire_trace_pair_t retirement_event_o,
  output logic                         recovery_required_o,
  output logic                         mispredict_pending_o,
  output rv32_ooo_pkg::rob_tag_t        mispredict_tag_o,
  output rv32_ooo_pkg::word_t           mispredict_next_pc_o,
  output logic                         serial_pending_o,
  output logic                         fault_pending_o,
  input  logic                         flush_i,
  output logic                         flush_done_o,
  input  rv32_ooo_pkg::rob_tag_t        query_tag_i,
  output logic                         query_live_o,
  output rv32_ooo_pkg::rob_index_t      query_age_o,
  output rv32_ooo_pkg::rob_index_t      head_index_o,
  output rv32_ooo_pkg::rob_count_t      occupancy_o,
  output logic                         full_o,
  output logic                         empty_o,
  output logic                         protocol_error_o
);
  import rv32_ooo_pkg::*;

  rob_entry_t entries_q [0:ROB_DEPTH-1];
  rob_entry_t entries_d [0:ROB_DEPTH-1];
  logic [ROB_GENERATION_W-1:0] generation_q [0:ROB_DEPTH-1];
  logic [ROB_GENERATION_W-1:0] generation_d [0:ROB_DEPTH-1];
  rob_index_t head_q, head_d, tail_q, tail_d, head1_index, tail1_index;
  rob_count_t count_q, count_d;
  logic recovery_q, recovery_d;
  renamed_uop_t dispatch0, dispatch1;
  completion_t completion0, completion1;
  rob_entry_t head0, head1, owner0, owner1, query_owner, scan_entry;
  rob_entry_t updated0, updated1, new0, new1;
  rob_tag_t tag0, tag1;
  logic [1:0] head_fire;
  logic owner0_matches, owner1_matches, completion_conflict;
  logic head0_serial, head1_serial, allocation_allowed;
  integer scan_offset, scan_index, state_index, reset_index;
  integer push_count, pop_count;

  function automatic rob_index_t advance_index(input rob_index_t base, input integer amount);
    integer next_index;
    begin
      next_index = base + amount;
      if (next_index >= ROB_DEPTH) next_index = next_index - ROB_DEPTH;
      advance_index = next_index[ROB_INDEX_W-1:0];
    end
  endfunction

  // Read a whole entry before selecting its fields for Icarus/Vivado portability.
  assign dispatch0 = dispatch_uop_i.lane0;
  assign dispatch1 = dispatch_uop_i.lane1;
  assign completion0 = completion_i.lane0;
  assign completion1 = completion_i.lane1;
  assign head1_index = advance_index(head_q, 1);
  assign tail1_index = advance_index(tail_q, 1);
  assign head0 = entries_q[head_q];
  assign head1 = entries_q[head1_index];
  assign owner0 = entries_q[completion0.rob_tag.index];
  assign owner1 = entries_q[completion1.rob_tag.index];
  assign query_owner = entries_q[query_tag_i.index];
  assign head_index_o = head_q;
  assign occupancy_o = count_q;
  assign full_o = count_q == ROB_DEPTH;
  assign empty_o = count_q == 0;
  assign recovery_required_o = recovery_q;
  assign dispatch_tag_o.lane0 = tag0;
  assign dispatch_tag_o.lane1 = tag1;

  always_comb begin
    mispredict_pending_o = 1'b0;
    mispredict_tag_o = '0;
    mispredict_next_pc_o = '0;
    serial_pending_o = 1'b0;
    fault_pending_o = 1'b0;
    scan_entry = '0;
    scan_index = 0;
    // Age-order scan permits an older, later-resolving misprediction to take priority.
    for (scan_offset = 0; scan_offset < ROB_DEPTH; scan_offset = scan_offset + 1) begin
      scan_index = head_q + scan_offset;
      if (scan_index >= ROB_DEPTH) scan_index = scan_index - ROB_DEPTH;
      scan_entry = entries_q[scan_index];
      if (scan_entry.valid) begin
        if (scan_entry.uop.decoded.op_class == UOP_SYSTEM) serial_pending_o = 1'b1;
        if (scan_entry.exception.valid) fault_pending_o = 1'b1;
        if (!mispredict_pending_o && scan_entry.complete && !scan_entry.exception.valid &&
            scan_entry.control_flow.mispredicted) begin
          mispredict_pending_o = 1'b1;
          mispredict_tag_o = scan_entry.uop.rob_tag;
          mispredict_next_pc_o = scan_entry.control_flow.next_pc;
        end
      end
    end
    if (!rst_ni || flush_i) begin
      mispredict_pending_o = 1'b0;
      mispredict_tag_o = '0;
      mispredict_next_pc_o = '0;
      serial_pending_o = 1'b0;
      fault_pending_o = 1'b0;
    end
  end

  always_comb begin
    tag0.index = tail_q;
    tag0.generation = generation_q[tail_q];
    tag1.index = tail1_index;
    tag1.generation = generation_q[tail1_index];
    allocation_allowed = rst_ni && !flush_i && !recovery_q && !dispatch_hold_i &&
                         !mispredict_pending_o && !serial_pending_o && !fault_pending_o;
    // Capacity is current occupancy: slots released at this edge are usable next cycle.
    dispatch_ready_o[0] = allocation_allowed && count_q < ROB_DEPTH &&
                          dispatch0.rob_tag == tag0;
    dispatch_fire_o[0] = dispatch_valid_i[0] && dispatch_ready_o[0];
    // Mask 10 cannot transfer: lane 1 requires lane 0's fire. Ready 0 stays valid-independent.
    dispatch_ready_o[1] = dispatch_fire_o[0] && count_q < ROB_DEPTH-1 &&
                          dispatch1.rob_tag == tag1 && !dispatch0.decoded.fetch.exception.valid &&
                          dispatch0.decoded.op_class != UOP_SYSTEM;
    dispatch_fire_o[1] = dispatch_valid_i[1] && dispatch_ready_o[1];

    completion_ready_o = rst_ni && !flush_i ? 2'b11 : 2'b00;
    owner0_matches = owner0.valid && !owner0.complete &&
                     owner0.uop.rob_tag == completion0.rob_tag &&
                     owner0.uop.pdst == completion0.pdst &&
                     owner0.uop.decoded.writes_rd == completion0.writes_rd;
    owner1_matches = owner1.valid && !owner1.complete &&
                     owner1.uop.rob_tag == completion1.rob_tag &&
                     owner1.uop.pdst == completion1.pdst &&
                     owner1.uop.decoded.writes_rd == completion1.writes_rd;
    completion_accepted_o[0] = completion_valid_i[0] && completion_ready_o[0] && owner0_matches;
    completion_accepted_o[1] = completion_valid_i[1] && completion_ready_o[1] && owner1_matches;
    completion_conflict = (&completion_accepted_o) &&
                          ((completion0.rob_tag == completion1.rob_tag) ||
                           (completion0.writes_rd && completion1.writes_rd &&
                            completion0.pdst != 0 && completion0.pdst == completion1.pdst));
    // Invalid packets are consumed but never broadcast; duplicate lane 1 loses to lane 0.
    if (completion_conflict) completion_accepted_o[1] = 1'b0;
    completion_rejected_o = completion_valid_i & completion_ready_o & ~completion_accepted_o;

    head_valid_o = '0;
    if (rst_ni && !flush_i && !recovery_q && count_q != 0 && head0.valid) begin
      head_valid_o[0] = 1'b1;
      if (count_q > 1 && head1.valid) head_valid_o[1] = 1'b1;
    end
    head_complete_o[0] = head_valid_o[0] && head0.complete;
    head_complete_o[1] = head_valid_o[1] && head1.complete;
    head_entry_o = '0;
    if (head_valid_o[0]) head_entry_o.lane0 = head0;
    if (head_valid_o[1]) head_entry_o.lane1 = head1;
    head0_serial = head0.uop.decoded.op_class == UOP_SYSTEM;
    head1_serial = head1.uop.decoded.op_class == UOP_SYSTEM;
    head_fire[0] = head_complete_o[0] && head_ready_i[0];
    head_fire[1] = head_fire[0] && head_complete_o[1] && head_ready_i[1] &&
                   !head0.exception.valid && !head0.control_flow.mispredicted &&
                   !head0_serial && !head1_serial;
    retire_fire_o[0] = head_fire[0] && !head0.exception.valid;
    retire_fire_o[1] = head_fire[1] && !head1.exception.valid;
    trap_fire_o[0] = head_fire[0] && head0.exception.valid;
    trap_fire_o[1] = head_fire[1] && head1.exception.valid;

    query_live_o = rst_ni && !flush_i && query_owner.valid &&
                   query_owner.uop.rob_tag == query_tag_i;
    query_age_o = '0;
    // The fixed power-of-two ring makes subtraction a modulo-16 age offset.
    if (query_live_o) query_age_o = query_tag_i.index - head_q;
    protocol_error_o = rst_ni && ((dispatch_valid_i == 2'b10) ||
                       (head_ready_i == 2'b10) || completion_conflict);
  end

  function automatic retire_trace_t event_template(input rob_entry_t entry, input logic trap);
    retire_trace_t event_record;
    begin
      event_record = '0;
      event_record.valid = 1'b1;
      event_record.pc = entry.uop.decoded.fetch.pc;
      event_record.instruction = entry.uop.decoded.fetch.instruction;
      if (trap) begin
        event_record.trap = 1'b1;
        event_record.cause = entry.exception.cause;
        event_record.tval = entry.exception.tval;
        event_record.next_pc = head_trap_target_i;
      end else begin
        event_record.next_pc = entry.control_flow.next_pc;
        if (entry.uop.decoded.writes_rd && entry.uop.decoded.rd != 0) begin
          event_record.rd = entry.uop.decoded.rd;
          event_record.rd_value = entry.result;
        end
        if (entry.memory_access.valid) begin
          event_record.memory_write = entry.memory_access.write;
          event_record.memory_address = entry.memory_access.address;
          event_record.memory_wdata = entry.memory_access.wdata;
          event_record.memory_wstrb = entry.memory_access.wstrb;
        end
      end
      event_template = event_record;
    end
  endfunction

  always_comb begin
    retirement_event_o = '0;
    if (head_fire[0]) retirement_event_o.lane0 = event_template(head0, trap_fire_o[0]);
    if (head_fire[1]) retirement_event_o.lane1 = event_template(head1, trap_fire_o[1]);
  end

  always_comb begin
    for (state_index = 0; state_index < ROB_DEPTH; state_index = state_index + 1) begin
      entries_d[state_index] = entries_q[state_index];
      generation_d[state_index] = generation_q[state_index];
    end
    head_d = head_q;
    tail_d = tail_q;
    count_d = count_q;
    recovery_d = recovery_q;
    updated0 = owner0;
    updated1 = owner1;
    new0 = '0;
    new1 = '0;
    push_count = 0;
    pop_count = 0;

    if (completion_accepted_o[0]) begin
      updated0.complete = 1'b1;
      updated0.result = completion0.value;
      updated0.exception = completion0.exception;
      updated0.control_flow = completion0.control_flow;
      updated0.memory_access = completion0.memory_access;
      entries_d[completion0.rob_tag.index] = updated0;
    end
    if (completion_accepted_o[1]) begin
      updated1.complete = 1'b1;
      updated1.result = completion1.value;
      updated1.exception = completion1.exception;
      updated1.control_flow = completion1.control_flow;
      updated1.memory_access = completion1.memory_access;
      entries_d[completion1.rob_tag.index] = updated1;
    end

    if (head_fire[0]) begin
      entries_d[head_q] = '0;
      pop_count = pop_count + 1;
      if (trap_fire_o[0] || head0.control_flow.mispredicted ||
          head0.uop.decoded.system_op == SYS_MRET ||
          head0.uop.decoded.system_op == SYS_FENCE_I) recovery_d = 1'b1;
    end
    if (head_fire[1]) begin
      entries_d[head1_index] = '0;
      pop_count = pop_count + 1;
      if (trap_fire_o[1] || head1.control_flow.mispredicted) recovery_d = 1'b1;
    end

    if (dispatch_fire_o[0]) begin
      new0.valid = 1'b1;
      new0.uop = dispatch0;
      new0.exception = dispatch0.decoded.fetch.exception;
      new0.complete = dispatch0.decoded.fetch.exception.valid;
      new0.control_flow.next_pc = dispatch0.decoded.fetch.pc + 32'd4;
      entries_d[tail_q] = new0;
      generation_d[tail_q] = generation_q[tail_q] + 1'b1;
      push_count = push_count + 1;
    end
    if (dispatch_fire_o[1]) begin
      new1.valid = 1'b1;
      new1.uop = dispatch1;
      new1.exception = dispatch1.decoded.fetch.exception;
      new1.complete = dispatch1.decoded.fetch.exception.valid;
      new1.control_flow.next_pc = dispatch1.decoded.fetch.pc + 32'd4;
      entries_d[tail1_index] = new1;
      generation_d[tail1_index] = generation_q[tail1_index] + 1'b1;
      push_count = push_count + 1;
    end
    head_d = advance_index(head_q, pop_count);
    tail_d = advance_index(tail_q, push_count);
    count_d = count_q + push_count - pop_count;

    if (flush_i) begin
      // Dominant safe APPLY: no retirement event or new packet is accepted on this edge.
      // Generation counters survive so the next occupant differs from a flushed owner.
      for (state_index = 0; state_index < ROB_DEPTH; state_index = state_index + 1)
        entries_d[state_index] = '0;
      head_d = '0;
      tail_d = '0;
      count_d = '0;
      recovery_d = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (reset_index = 0; reset_index < ROB_DEPTH; reset_index = reset_index + 1) begin
        entries_q[reset_index] <= '0;
        generation_q[reset_index] <= '0;
      end
      head_q <= '0;
      tail_q <= '0;
      count_q <= '0;
      recovery_q <= 1'b0;
      flush_done_o <= 1'b0;
    end else begin
      for (reset_index = 0; reset_index < ROB_DEPTH; reset_index = reset_index + 1) begin
        entries_q[reset_index] <= entries_d[reset_index];
        generation_q[reset_index] <= generation_d[reset_index];
      end
      head_q <= head_d;
      tail_q <= tail_d;
      count_q <= count_d;
      recovery_q <= recovery_d;
      flush_done_o <= flush_i;
    end
  end
endmodule
