// Custom two-issue RV32IM machine-mode core. Five logical sections use explicit queues.
// Instruction storage and data transport are external handshake endpoints.
`timescale 1ns / 1ps
module rv32_ooo_core
  import rv32_ooo_pkg::*;
(
  input logic clk_i, rst_ni,
  input logic predict_enable_i, halt_i,
  output logic instruction_request_valid_o,
  input logic instruction_request_ready_i,
  output word_t instruction_request_address_o,
  input logic instruction_response_valid_i,
  output logic instruction_response_ready_o,
  input fetch_row_t instruction_response_data_i,
  input logic instruction_response_error_i,
  output logic data_request_valid_o,
  input logic data_request_ready_i,
  output data_request_t data_request_o,
  input logic data_response_valid_i,
  output logic data_response_ready_o,
  input memory_response_t data_response_i,
  input logic irq_external_i, irq_software_i, irq_timer_i,
  output retire_trace_pair_t trace_o,
  output logic empty_o,
  output logic protocol_error_o
);
  logic [1:0] front_valid, rename_ready, rename_fire, rob_ready, rob_fire;
  logic [1:0] iq_ready, iq_fire, src1_ready, src2_ready, wb_valid;
  logic [1:0] wb_ready, wb_accepted, wb_rejected, head_valid, head_complete;
  logic [1:0] head_ready, retire_fire, trap_fire, write_enable, issue_fire, cluster_issue_fire;
  logic [1:0] alu_valid, alu_ready, wb_raw_valid, broadcast_valid, writeback_occupancy;
  decoded_pair_t decoded;
  renamed_pair_t renamed, retiring;
  completion_sources_t sources;
  logic [4:0] source_valid, source_ready;
  logic writeback_done, writeback_quiescent, writeback_error;
  completion_pair_t wb_raw, broadcast, wb, alu_completion;
  completion_t md_completion, lsu_completion, csr_completion;
  phys_reg_t write0_addr, write1_addr;
  word_t write0_data, write1_data;
  rob_tag_pair_t tags;
  rob_entry_pair_t heads;
  retire_trace_pair_t events, trace_events, trace;
  execute_packet_pair_t issued;
  memory_entry_t memory;
  phys_reg_t r00, r01, r10, r11, committed_phys;
  word_t v00, v01, v10, v11, mispredict_pc;
  phys_mask_t free_mask, ready_mask;
  phys_count_t free_count;
  rob_index_t head_index, query_age;
  rob_count_t rob_occupancy;
  issue_count_t iq_occupancy;
  logic [ISSUE_DEPTH-1:0] occupied;
  rob_tag_t mispredict_tag;
  logic alu0_available, alu1_available, md_available, memory_available;
  logic md_valid, md_ready, memory_valid, apply, cancel, cancel_done;
  logic [2:0] cluster_busy;
  logic cluster_quiescent, cluster_error, wb_allow, admission_hold;
  logic rename_done, rob_done, iq_done, rename_error, rob_error, iq_error;
  logic trace_error, write_error, recovery, mispredict, serial_pending, fault_pending;
  logic query_live, full, empty;
  logic [1:0] fetched_valid, fetched_fire;
  fetch_pair_t fetched;
  fetch_count_t fetch_occupancy;
  logic fetch_transaction, fetch_stale, fetch_fault, fetch_error, redirect_done;
  word_t prediction_pc0, prediction_pc1, redirect_pc;
  prediction_pair_t prediction;
  logic [1:0] counter0, counter1;
  logic predictor_invalidate, invalidate_done, predictor_error;
  logic lsu_prepare_ready, lsu_completion_valid, lsu_flush_done, lsu_quiescent, lsu_error;
  logic [1:0] lsu_dispatch_ready, lsu_dispatch_fire;
  logic [2:0] lsu_occupancy;
  logic csr_request_ready, csr_completion_valid, csr_cancel_done, csr_quiescent, csr_error;
  logic irq_pending, irq_take, wfi_wait, port0_system;
  word_t irq_cause, csr_trap_target, csr_trap_pc, csr_trap_cause, csr_trap_tval;
  logic synchronous_trap, recovery_request, recovery_busy, recovery_done;
  logic restart0, restart1;
  word_t recovery_target, architectural_next_pc_q;
  rob_entry_t fault_owner;

  assign wb_allow = 1'b1;
  assign admission_hold = halt_i || recovery_busy || irq_pending || wfi_wait;
  assign front_valid = rst_ni && !admission_hold ? fetched_valid : 2'b00;
  assign head_ready = rst_ni && !recovery_busy ? 2'b11 : 2'b00;
  assign port0_system = issued.lane0.uop.decoded.op_class == UOP_SYSTEM;
  assign cluster_issue_fire = {issue_fire[1], issue_fire[0] && !port0_system};
  assign trace_o = trace;
  assign empty_o = empty && iq_occupancy == 0 && cluster_quiescent
                   && writeback_quiescent && lsu_quiescent && csr_quiescent;
  assign irq_take = rst_ni && !halt_i && !recovery_busy && irq_pending && empty_o;
  assign synchronous_trap = |trap_fire;
  assign fault_owner = trap_fire[0] ? heads.lane0 : heads.lane1;
  assign csr_trap_pc = synchronous_trap ? fault_owner.uop.decoded.fetch.pc
                                       : architectural_next_pc_q;
  assign csr_trap_cause = synchronous_trap ? fault_owner.exception.cause : irq_cause;
  assign csr_trap_tval = synchronous_trap ? fault_owner.exception.tval : 32'b0;

  function automatic logic needs_restart(input rob_entry_t entry);
    needs_restart = entry.control_flow.mispredicted
      || (entry.uop.decoded.op_class == UOP_SYSTEM
          && (entry.uop.decoded.system_op == SYS_MRET
              || entry.uop.decoded.system_op == SYS_FENCE_I));
  endfunction
  assign restart0 = retire_fire[0] && needs_restart(heads.lane0);
  assign restart1 = retire_fire[1] && needs_restart(heads.lane1);
  assign recovery_request = !recovery_busy
                            && (synchronous_trap || irq_take || restart0 || restart1);
  assign recovery_target = synchronous_trap || irq_take ? csr_trap_target
                            : restart0 ? heads.lane0.control_flow.next_pc
                                       : heads.lane1.control_flow.next_pc;
  assign predictor_invalidate = (retire_fire[0]
      && heads.lane0.uop.decoded.system_op == SYS_FENCE_I)
      || (retire_fire[1] && heads.lane1.uop.decoded.system_op == SYS_FENCE_I);

  // An interrupt is a separate precise event after all admitted instructions drain.
  always_comb begin
    trace_events = events;
    if (irq_take) begin
      trace_events = '0;
      trace_events.lane0.valid = 1'b1;
      trace_events.lane0.trap = 1'b1;
      trace_events.lane0.interrupt = 1'b1;
      trace_events.lane0.pc = architectural_next_pc_q;
      trace_events.lane0.next_pc = csr_trap_target;
      trace_events.lane0.cause = irq_cause;
    end
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) architectural_next_pc_q <= RESET_PC;
    else begin
      if (trace_events.lane0.valid) architectural_next_pc_q <= trace_events.lane0.next_pc;
      if (trace_events.lane1.valid) architectural_next_pc_q <= trace_events.lane1.next_pc;
    end
  end

  rv32_recovery_controller recovery_control (
    .clk_i(clk_i), .rst_ni(rst_ni), .request_i(recovery_request), .target_i(recovery_target),
    .units_quiescent_i(cluster_quiescent && writeback_quiescent && lsu_quiescent && csr_quiescent),
    .cancel_ack_i(cancel_done && writeback_done && csr_cancel_done),
    .cancel_o(cancel), .apply_o(apply), .redirect_pc_o(redirect_pc),
    .busy_o(recovery_busy), .done_o(recovery_done)
  );
  rv32_fetch fetch (
    .clk_i(clk_i), .rst_ni(rst_ni), .enable_i(!admission_hold && !mispredict),
    .redirect_i(apply), .redirect_pc_i(redirect_pc), .redirect_done_o(redirect_done),
    .predict_enable_i(predict_enable_i), .prediction_pc0_o(prediction_pc0),
    .prediction_pc1_o(prediction_pc1), .prediction_i(prediction),
    .request_valid_o(instruction_request_valid_o), .request_ready_i(instruction_request_ready_i),
    .request_address_o(instruction_request_address_o),
    .response_valid_i(instruction_response_valid_i),
    .response_ready_o(instruction_response_ready_o),
    .response_data_i(instruction_response_data_i), .response_error_i(instruction_response_error_i),
    .fetch_valid_o(fetched_valid), .fetch_ready_i(rename_fire), .fetch_o(fetched),
    .fetch_fire_o(fetched_fire), .occupancy_o(fetch_occupancy),
    .transaction_pending_o(fetch_transaction), .stale_pending_o(fetch_stale),
    .fault_pending_o(fetch_fault), .protocol_error_o(fetch_error)
  );
  rv32_branch_predictor predictor (
    .clk_i(clk_i), .rst_ni(rst_ni), .enable_i(predict_enable_i),
    .query_pc0_i(prediction_pc0), .query_pc1_i(prediction_pc1), .prediction_o(prediction),
    .retire_valid_i(retire_fire), .retire_entry_i(heads), .invalidate_i(predictor_invalidate),
    .invalidate_done_o(invalidate_done), .query0_counter_o(counter0), .query1_counter_o(counter1),
    .protocol_error_o(predictor_error)
  );
  rv32_decoder decoder0 (.fetch_i(fetched.lane0), .decoded_o(decoded.lane0));
  rv32_decoder decoder1 (.fetch_i(fetched.lane1), .decoded_o(decoded.lane1));

  rv32_rename rename (
    .clk_i(clk_i), .rst_ni(rst_ni), .dispatch_valid_i(front_valid),
    .dispatch_resources_ready_i(rob_ready & iq_ready & lsu_dispatch_ready), .decoded_i(decoded),
    .dispatch_rob_tag_i(tags), .dispatch_ready_o(rename_ready), .dispatch_fire_o(rename_fire),
    .renamed_o(renamed), .src1_ready_o(src1_ready), .src2_ready_o(src2_ready),
    .completion_valid_i(broadcast_valid), .completion_i(broadcast),
    .retire_valid_i(retire_fire), .retire_uop_i(retiring),
    .recover_i(apply), .recover_done_o(rename_done),
    .committed_query_arch_i(5'd0), .committed_query_phys_o(committed_phys),
    .free_mask_o(free_mask), .ready_mask_o(ready_mask), .free_count_o(free_count),
    .protocol_error_o(rename_error)
  );
  assign retiring.lane0 = heads.lane0.uop;
  assign retiring.lane1 = heads.lane1.uop;
  rv32_reorder_buffer rob (
    .clk_i(clk_i), .rst_ni(rst_ni), .dispatch_hold_i(admission_hold),
    .dispatch_valid_i(rename_fire), .dispatch_uop_i(renamed),
    .dispatch_ready_o(rob_ready), .dispatch_fire_o(rob_fire), .dispatch_tag_o(tags),
    .completion_valid_i(wb_valid), .completion_i(wb), .completion_ready_o(wb_ready),
    .completion_accepted_o(wb_accepted), .completion_rejected_o(wb_rejected),
    .head_valid_o(head_valid), .head_complete_o(head_complete), .head_entry_o(heads),
    .head_ready_i(head_ready), .head_trap_target_i(csr_trap_target),
    .retire_fire_o(retire_fire), .trap_fire_o(trap_fire), .retirement_event_o(events),
    .recovery_required_o(recovery), .mispredict_pending_o(mispredict),
    .mispredict_tag_o(mispredict_tag), .mispredict_next_pc_o(mispredict_pc),
    .serial_pending_o(serial_pending), .fault_pending_o(fault_pending),
    .flush_i(apply), .flush_done_o(rob_done), .query_tag_i(wb.lane0.rob_tag),
    .query_live_o(query_live), .query_age_o(query_age), .head_index_o(head_index),
    .occupancy_o(rob_occupancy), .full_o(full), .empty_o(empty), .protocol_error_o(rob_error)
  );
  rv32_physical_regfile prf (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .read0_rs1_i(r00), .read0_rs2_i(r01),
    .read1_rs1_i(r10), .read1_rs2_i(r11),
    .read0_data1_o(v00), .read0_data2_o(v01),
    .read1_data1_o(v10), .read1_data2_o(v11),
    .write_enable_i(write_enable), .write0_addr_i(write0_addr),
    .write1_addr_i(write1_addr), .write0_data_i(write0_data),
    .write1_data_i(write1_data), .write_conflict_o(write_error)
  );
  rv32_issue_queue iq (
    .clk_i(clk_i), .rst_ni(rst_ni), .dispatch_hold_i(admission_hold),
    .issue_hold_i(recovery || recovery_busy),
    .dispatch_valid_i(rename_fire), .dispatch_uop_i(renamed),
    .dispatch_src1_ready_i(src1_ready), .dispatch_src2_ready_i(src2_ready),
    .dispatch_ready_o(iq_ready), .dispatch_fire_o(iq_fire),
    .completion_valid_i(broadcast_valid), .completion_i(broadcast),
    .rob_head_valid_i(head_valid[0]), .rob_head_index_i(head_index),
    .rob_head_tag_i(heads.lane0.uop.rob_tag),
    .mispredict_pending_i(mispredict), .mispredict_tag_i(mispredict_tag),
    .alu0_ready_i(alu0_available), .alu1_ready_i(alu1_available),
    .muldiv_ready_i(md_available), .memory_ready_i(memory_available),
    .system_ready_i(csr_request_ready),
    .issue_fire_o(issue_fire), .issue_packet_o(issued),
    .read0_rs1_o(r00), .read0_rs2_o(r01), .read1_rs1_o(r10), .read1_rs2_o(r11),
    .read0_data1_i(v00), .read0_data2_i(v01),
    .read1_data1_i(v10), .read1_data2_i(v11),
    .flush_i(apply), .flush_done_o(iq_done), .occupancy_o(iq_occupancy),
    .occupied_mask_o(occupied), .protocol_error_o(iq_error)
  );
  rv32_execution_cluster execute (
    .clk_i(clk_i), .rst_ni(rst_ni), .issue_fire_i(cluster_issue_fire), .issue_packet_i(issued),
    .alu0_ready_o(alu0_available), .alu1_ready_o(alu1_available),
    .muldiv_ready_o(md_available), .memory_ready_o(memory_available),
    .alu_completion_valid_o(alu_valid), .alu_completion_ready_i(alu_ready),
    .alu_completion_o(alu_completion), .muldiv_completion_valid_o(md_valid),
    .muldiv_completion_ready_i(md_ready), .muldiv_completion_o(md_completion),
    .memory_valid_o(memory_valid), .memory_ready_i(lsu_prepare_ready), .memory_o(memory),
    .cancel_i(cancel), .cancel_done_o(cancel_done), .busy_o(cluster_busy),
    .quiescent_o(cluster_quiescent), .protocol_error_o(cluster_error)
  );
  rv32_trace_recorder recorder (
    .clk_i(clk_i), .rst_ni(rst_ni), .event_i(trace_events), .trace_o(trace),
    .protocol_error_o(trace_error)
  );

  // ROB ownership checks gate all speculative register effects.
  assign wb = wb_raw;
  assign wb_valid = wb_raw_valid & {2{wb_allow}};
  assign source_valid = {csr_completion_valid, lsu_completion_valid, md_valid, alu_valid};
  assign alu_ready = source_ready[1:0];
  assign md_ready = source_ready[2];
  always_comb begin
    sources = '0;
    sources.alu0_result = alu_completion.lane0;
    sources.alu1_result = alu_completion.lane1;
    sources.muldiv_result = md_completion;
    // Memory and serialized system units share the retained completion path.
    sources.memory_result = lsu_completion;
    sources.system_result = csr_completion;
  end
  rv32_writeback writeback (
    .clk_i(clk_i), .rst_ni(rst_ni), .source_valid_i(source_valid),
    .source_ready_o(source_ready), .sources_i(sources), .rob_head_index_i(head_index),
    .rob_completion_valid_o(wb_raw_valid),
    .rob_completion_ready_i(wb_ready & {2{wb_allow}}), .rob_completion_o(wb_raw),
    .rob_completion_accepted_i(wb_accepted),
    .broadcast_valid_o(broadcast_valid), .broadcast_o(broadcast),
    .write_enable_o(write_enable), .write0_addr_o(write0_addr), .write1_addr_o(write1_addr),
    .write0_data_o(write0_data), .write1_data_o(write1_data),
    .cancel_i(cancel), .cancel_done_o(writeback_done), .quiescent_o(writeback_quiescent),
    .occupancy_o(writeback_occupancy), .protocol_error_o(writeback_error)
  );

  rv32_load_store_unit lsu (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .dispatch_valid_i(rename_fire), .dispatch_uop_i(renamed),
    .dispatch_ready_o(lsu_dispatch_ready), .dispatch_fire_o(lsu_dispatch_fire),
    .prepare_valid_i(memory_valid), .prepare_ready_o(lsu_prepare_ready), .prepare_i(memory),
    .head_valid_i(head_valid[0]), .head_complete_i(head_complete[0]),
    .head_tag_i(heads.lane0.uop.rob_tag),
    .data_request_valid_o(data_request_valid_o), .data_request_ready_i(data_request_ready_i),
    .data_request_o(data_request_o), .data_response_valid_i(data_response_valid_i),
    .data_response_ready_o(data_response_ready_o), .data_response_i(data_response_i),
    .completion_valid_o(lsu_completion_valid), .completion_ready_i(source_ready[3]),
    .completion_o(lsu_completion), .flush_i(apply), .flush_done_o(lsu_flush_done),
    .quiescent_o(lsu_quiescent), .occupancy_o(lsu_occupancy), .protocol_error_o(lsu_error)
  );
  rv32_csr_file machine (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .request_valid_i(issue_fire[0] && port0_system), .request_ready_o(csr_request_ready),
    .request_i(issued.lane0), .completion_valid_o(csr_completion_valid),
    .completion_ready_i(source_ready[4]), .completion_o(csr_completion),
    .retire_valid_i(retire_fire), .retire_entry_i(heads),
    .irq_external_i(irq_external_i), .irq_software_i(irq_software_i), .irq_timer_i(irq_timer_i),
    .interrupt_pending_o(irq_pending), .interrupt_cause_o(irq_cause),
    .trap_enter_i(synchronous_trap || irq_take), .trap_pc_i(csr_trap_pc),
    .trap_cause_i(csr_trap_cause), .trap_tval_i(csr_trap_tval), .trap_target_o(csr_trap_target),
    .wfi_wait_o(wfi_wait), .cancel_i(cancel), .cancel_done_o(csr_cancel_done),
    .quiescent_o(csr_quiescent), .protocol_error_o(csr_error)
  );
  assign protocol_error_o = rename_error || rob_error || iq_error || trace_error || write_error
      || cluster_error || writeback_error || fetch_error || predictor_error || lsu_error
      || csr_error || (rename_fire != rob_fire) || (rename_fire != iq_fire)
      || (rename_fire != lsu_dispatch_fire) || (rename_fire != fetched_fire);
endmodule
