// WB/commit section: instructions enter in program order and may complete out of order.
// Only a completed head can retire or take a precise trap. No result is committed on arrival.
`timescale 1ns / 1ps

module rv32_reorder_buffer (
  input  logic                       clk_i,
  input  logic                       rst_ni,
  input  logic                       dispatch_hold_i,
  input  logic                       dispatch_valid_i,
  input  rv32_ooo_pkg::renamed_uop_t  dispatch_uop_i,
  output logic                       dispatch_ready_o,
  output rv32_ooo_pkg::rob_tag_t      dispatch_tag_o,
  input  logic                       completion_valid_i,
  input  rv32_ooo_pkg::completion_t   completion_i,
  output logic                       completion_ready_o,
  output logic                       completion_accepted_o,
  output logic                       completion_rejected_o,
  output logic                       head_valid_o,
  output logic                       head_complete_o,
  output rv32_ooo_pkg::rob_entry_t    head_entry_o,
  input  logic                       head_ready_i,
  input  rv32_ooo_pkg::word_t         head_trap_target_i,
  output logic                       retire_fire_o,
  output logic                       trap_fire_o,
  output rv32_ooo_pkg::retire_trace_t retirement_event_o,
  output logic                       recovery_required_o,
  output logic                       mispredict_pending_o,
  output rv32_ooo_pkg::rob_tag_t      mispredict_tag_o,
  output rv32_ooo_pkg::word_t         mispredict_next_pc_o,
  input  logic                       flush_i,
  output logic                       flush_done_o,
  input  rv32_ooo_pkg::rob_tag_t      query_tag_i,
  output logic                       query_live_o,
  output rv32_ooo_pkg::rob_index_t    query_age_o,
  output rv32_ooo_pkg::rob_index_t    head_index_o,
  output rv32_ooo_pkg::rob_count_t    occupancy_o,
  output logic                       full_o,
  output logic                       empty_o
);
  import rv32_ooo_pkg::*;

  rob_entry_t entries_q [0:ROB_DEPTH-1];
  rob_entry_t entries_d [0:ROB_DEPTH-1];
  logic [ROB_GENERATION_W-1:0] generation_q [0:ROB_DEPTH-1];
  logic [ROB_GENERATION_W-1:0] generation_d [0:ROB_DEPTH-1];
  rob_index_t head_q, head_d, tail_q, tail_d;
  rob_count_t count_q, count_d;
  logic recovery_q, recovery_d;
  rob_entry_t current_head;
  rob_entry_t completion_owner;
  rob_entry_t query_owner;
  rob_entry_t updated_completion;
  rob_entry_t new_entry;
  rob_entry_t scan_entry;
  logic dispatch_fire;
  logic head_fire;
  logic completion_owner_matches;
  logic head_fault_pending;
  integer scan_offset;
  integer scan_index;
  integer state_index;
  integer reset_index;

  // Read whole packed records before inspecting fields; this also avoids tool-specific
  // limitations on a variable array index followed by nested struct-field selection.
  assign current_head = entries_q[head_q];
  assign completion_owner = entries_q[completion_i.rob_tag.index];
  assign query_owner = entries_q[query_tag_i.index];
  assign head_index_o = head_q;
  assign occupancy_o = count_q;
  assign full_o = count_q == ROB_DEPTH;
  assign empty_o = count_q == 0;
  assign recovery_required_o = recovery_q;

  always_comb begin
    head_fault_pending = !empty_o && current_head.valid && current_head.complete &&
                         current_head.exception.valid;
    mispredict_pending_o = 1'b0;
    mispredict_tag_o = '0;
    mispredict_next_pc_o = '0;
    scan_entry = '0;
    scan_index = 0;
    // Scan from head to tail by age, so an older late-resolving branch wins.
    for (scan_offset = 0; scan_offset < ROB_DEPTH; scan_offset = scan_offset + 1) begin
      scan_index = head_q + scan_offset;
      if (scan_index >= ROB_DEPTH) scan_index = scan_index - ROB_DEPTH;
      scan_entry = entries_q[scan_index];
      if (!mispredict_pending_o && scan_entry.valid && scan_entry.complete &&
          !scan_entry.exception.valid && scan_entry.control_flow.mispredicted) begin
        mispredict_pending_o = 1'b1;
        mispredict_tag_o = scan_entry.uop.rob_tag;
        mispredict_next_pc_o = scan_entry.control_flow.next_pc;
      end
    end
    if (!rst_ni || flush_i) begin
      mispredict_pending_o = 1'b0;
      mispredict_tag_o = '0;
      mispredict_next_pc_o = '0;
    end
  end

  always_comb begin
    dispatch_tag_o.index = tail_q;
    dispatch_tag_o.generation = generation_q[tail_q];
    dispatch_ready_o = rst_ni && !flush_i && !recovery_q && !dispatch_hold_i &&
                       !mispredict_pending_o && !head_fault_pending && !full_o &&
                       (dispatch_uop_i.rob_tag == dispatch_tag_o);
    dispatch_fire = dispatch_valid_i && dispatch_ready_o;

    completion_ready_o = rst_ni && !flush_i;
    completion_owner_matches = completion_owner.valid && !completion_owner.complete &&
                               (completion_owner.uop.rob_tag == completion_i.rob_tag) &&
                               (completion_owner.uop.pdst == completion_i.pdst) &&
                               (completion_owner.uop.decoded.writes_rd == completion_i.writes_rd);
    completion_accepted_o = completion_valid_i && completion_ready_o && completion_owner_matches;
    completion_rejected_o = completion_valid_i && completion_ready_o && !completion_owner_matches;

    head_valid_o = rst_ni && !flush_i && !recovery_q && !empty_o && current_head.valid;
    head_complete_o = head_valid_o && current_head.complete;
    head_entry_o = '0;
    if (head_valid_o) head_entry_o = current_head;
    head_fire = head_complete_o && head_ready_i;
    retire_fire_o = head_fire && !current_head.exception.valid;
    trap_fire_o = head_fire && current_head.exception.valid;

    query_live_o = rst_ni && !flush_i && query_owner.valid &&
                   (query_owner.uop.rob_tag == query_tag_i);
    // The fixed 16-entry ring makes four-bit subtraction the modulo-16 age offset.
    query_age_o = '0;
    if (query_live_o) query_age_o = query_tag_i.index - head_q;
  end

  always_comb begin
    // This accepted-event template is stamped and registered by rv32_trace_recorder.
    retirement_event_o = '0;
    if (head_fire) begin
      retirement_event_o.valid = 1'b1;
      retirement_event_o.pc = current_head.uop.decoded.fetch.pc;
      retirement_event_o.instruction = current_head.uop.decoded.fetch.instruction;
      if (trap_fire_o) begin
        retirement_event_o.trap = 1'b1;
        retirement_event_o.cause = current_head.exception.cause;
        retirement_event_o.tval = current_head.exception.tval;
        retirement_event_o.next_pc = head_trap_target_i;
      end else begin
        retirement_event_o.next_pc = current_head.control_flow.next_pc;
        if (current_head.uop.decoded.writes_rd && current_head.uop.decoded.rd != 0) begin
          retirement_event_o.rd = current_head.uop.decoded.rd;
          retirement_event_o.rd_value = current_head.result;
        end
        if (current_head.memory_access.valid) begin
          retirement_event_o.memory_address = current_head.memory_access.address;
          retirement_event_o.memory_write = current_head.memory_access.write;
          retirement_event_o.memory_wdata = current_head.memory_access.wdata;
          retirement_event_o.memory_wstrb = current_head.memory_access.wstrb;
        end
      end
    end
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
    updated_completion = completion_owner;
    new_entry = '0;

    if (completion_accepted_o) begin
      updated_completion.complete = 1'b1;
      updated_completion.result = completion_i.value;
      updated_completion.exception = completion_i.exception;
      updated_completion.control_flow = completion_i.control_flow;
      updated_completion.memory_access = completion_i.memory_access;
      entries_d[completion_i.rob_tag.index] = updated_completion;
    end

    if (head_fire) begin
      entries_d[head_q] = '0;
      if (head_q == ROB_DEPTH-1) head_d = '0;
      else head_d = head_q + 1'b1;
      // A precise trap or retiring misprediction blocks younger effects until safe flush.
      if (trap_fire_o || current_head.control_flow.mispredicted) recovery_d = 1'b1;
    end

    if (dispatch_fire) begin
      new_entry.valid = 1'b1;
      new_entry.uop = dispatch_uop_i;
      new_entry.exception = dispatch_uop_i.decoded.fetch.exception;
      new_entry.complete = dispatch_uop_i.decoded.fetch.exception.valid;
      new_entry.control_flow.next_pc = dispatch_uop_i.decoded.fetch.pc + 32'd4;
      entries_d[tail_q] = new_entry;
      generation_d[tail_q] = generation_q[tail_q] + 1'b1;
      if (tail_q == ROB_DEPTH-1) tail_d = '0;
      else tail_d = tail_q + 1'b1;
    end

    case ({dispatch_fire, head_fire})
      2'b10: count_d = count_q + 1'b1;
      2'b01: count_d = count_q - 1'b1;
      default: count_d = count_q;
    endcase

    if (flush_i) begin
      // Safe APPLY only: the recovery controller has cancelled/drained older transactions.
      // Retain generation counters so flushed tags cannot match the next use of a slot.
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
