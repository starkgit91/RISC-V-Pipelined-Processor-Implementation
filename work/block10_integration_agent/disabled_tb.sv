// Actual RAM loop with production fetch/prediction/OoO execution and ordered training.
// Recovery sequencing below is a TEST DRIVER; the production controller is block 11.
`timescale 1ns / 1ps
module tb_block10_integration;
  import rv32_ooo_pkg::*;

  typedef enum logic [1:0] {RUN, CANCEL, APPLY} recovery_phase_e;
  recovery_phase_e phase = RUN;
  logic clk = 0, rst_n = 1, predict_enabled = 0, finished = 0, inspect_mode = 0;
  logic [1:0] front_valid, rename_ready, rename_fire, rob_ready, rob_fire;
  logic [1:0] iq_ready, iq_fire, src1_ready, src2_ready, wb_valid;
  logic [1:0] wb_ready, wb_accepted, wb_rejected, head_valid, head_complete;
  logic [1:0] head_ready, retire_fire, trap_fire, write_enable, issue_fire;
  logic [1:0] alu_valid, alu_ready, wb_raw_valid, broadcast_valid, writeback_occupancy;
  decoded_pair_t decoded;
  renamed_pair_t renamed, retiring;
  completion_sources_t sources;
  logic [4:0] source_valid, source_ready;
  logic writeback_done, writeback_quiescent, writeback_error;
  completion_pair_t wb_raw, broadcast, wb, alu_completion;
  completion_t md_completion;
  phys_reg_t write0_addr, write1_addr;
  word_t write0_data, write1_data;
  rob_tag_pair_t tags;
  rob_entry_pair_t heads;
  retire_trace_pair_t events, trace;
  execute_packet_pair_t issued;
  memory_entry_t memory;
  phys_reg_t r00, r01, r10, r11, committed_phys;
  arch_reg_t committed_arch;
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
  logic cluster_quiescent, cluster_error, wb_allow, issue_enable, admission_hold;
  logic rename_done, rob_done, iq_done, rename_error, rob_error, iq_error;
  logic trace_error, write_error, recovery, mispredict, serial_pending, fault_pending;
  logic query_live, full, empty;

  logic [1:0] fetched_valid, fetched_fire;
  fetch_pair_t fetched;
  fetch_count_t fetch_occupancy;
  logic fetch_transaction, fetch_stale, fetch_fault, fetch_error, redirect_done;
  word_t prediction_pc0, prediction_pc1, recovery_pc = 0;
  prediction_pair_t prediction;
  logic [1:0] counter0, counter1;
  logic invalidate = 0, invalidate_done, predictor_error;
  logic request_valid, request_ready, response_valid, response_ready, response_error;
  word_t request_address;
  fetch_row_t response_data, bram_data = 0;
  logic bram_enable, adapter_pending, adapter_error;
  logic [BRAM_FETCH_ADDR_W-1:0] bram_address;
  fetch_row_t rows [0:RAM_FETCH_ROWS-1];

  word_t reference_regs [0:31];
  word_t reference_pc;
  retire_trace_t enabled_trace [0:14];
  integer checks = 0, failures = 0, cycle_number = 0;
  integer retired_count = 0, branch_count = 0, recovery_count = 0;
  integer dispatched_count = 0, accepted_count = 0, predicted_taken_count = 0;
  integer enabled_cycles, disabled_cycles, enabled_recoveries, disabled_recoveries;
  integer expected_branch_number, watchdog, row, mode;

  assign wb_allow = 1'b1;
  assign cancel = phase != RUN;
  assign apply = phase == APPLY;
  assign admission_hold = phase != RUN || finished;
  assign issue_enable = phase == RUN && !finished;
  always_comb begin
    front_valid = '0;
    if (rst_n && phase == RUN && !finished) front_valid = fetched_valid;
    head_ready = '0;
    if (phase == RUN && !finished) begin
      head_ready = 2'b11;
      // Stop at the program's final instruction without retiring a following NOP.
      if (head_valid[0] && heads.lane0.uop.decoded.fetch.pc == 20) head_ready = 2'b01;
    end
  end

  rv32_fetch fetch (
    .clk_i(clk), .rst_ni(rst_n),
    .enable_i(phase == RUN && !finished && !mispredict),
    .redirect_i(apply), .redirect_pc_i(recovery_pc), .redirect_done_o(redirect_done),
    .predict_enable_i(1'b0), .prediction_pc0_o(prediction_pc0),
    .prediction_pc1_o(prediction_pc1), .prediction_i(prediction),
    .request_valid_o(request_valid), .request_ready_i(request_ready),
    .request_address_o(request_address), .response_valid_i(response_valid),
    .response_ready_o(response_ready), .response_data_i(response_data),
    .response_error_i(response_error), .fetch_valid_o(fetched_valid),
    .fetch_ready_i(rename_fire), .fetch_o(fetched), .fetch_fire_o(fetched_fire),
    .occupancy_o(fetch_occupancy), .transaction_pending_o(fetch_transaction),
    .stale_pending_o(fetch_stale), .fault_pending_o(fetch_fault), .protocol_error_o(fetch_error)
  );
  rv32_branch_predictor predictor (
    .clk_i(clk), .rst_ni(rst_n), .enable_i(predict_enabled),
    .query_pc0_i(prediction_pc0), .query_pc1_i(prediction_pc1), .prediction_o(prediction),
    .retire_valid_i(retire_fire), .retire_entry_i(heads),
    .invalidate_i(invalidate), .invalidate_done_o(invalidate_done),
    .query0_counter_o(counter0), .query1_counter_o(counter1), .protocol_error_o(predictor_error)
  );
  rv32_instruction_adapter instruction_memory (
    .clk_i(clk), .rst_ni(rst_n), .request_valid_i(request_valid),
    .request_ready_o(request_ready), .request_address_i(request_address),
    .response_valid_o(response_valid), .response_ready_i(response_ready),
    .response_data_o(response_data), .response_error_o(response_error),
    .bram_block_i(1'b0), .bram_enable_o(bram_enable), .bram_address_o(bram_address),
    .bram_data_i(bram_data), .transaction_pending_o(adapter_pending),
    .protocol_error_o(adapter_error)
  );
  always_ff @(posedge clk) begin
    if (bram_enable) bram_data <= rows[bram_address];
  end
  rv32_decoder decoder0 (.fetch_i(fetched.lane0), .decoded_o(decoded.lane0));
  rv32_decoder decoder1 (.fetch_i(fetched.lane1), .decoded_o(decoded.lane1));

  rv32_rename rename (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_valid_i(front_valid),
    .dispatch_resources_ready_i(rob_ready & iq_ready), .decoded_i(decoded),
    .dispatch_rob_tag_i(tags), .dispatch_ready_o(rename_ready), .dispatch_fire_o(rename_fire),
    .renamed_o(renamed), .src1_ready_o(src1_ready), .src2_ready_o(src2_ready),
    .completion_valid_i(broadcast_valid), .completion_i(broadcast),
    .retire_valid_i(retire_fire), .retire_uop_i(retiring),
    .recover_i(apply), .recover_done_o(rename_done),
    .committed_query_arch_i(committed_arch), .committed_query_phys_o(committed_phys),
    .free_mask_o(free_mask), .ready_mask_o(ready_mask), .free_count_o(free_count),
    .protocol_error_o(rename_error)
  );
  assign retiring.lane0 = heads.lane0.uop;
  assign retiring.lane1 = heads.lane1.uop;
  rv32_reorder_buffer rob (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_hold_i(admission_hold),
    .dispatch_valid_i(rename_fire), .dispatch_uop_i(renamed),
    .dispatch_ready_o(rob_ready), .dispatch_fire_o(rob_fire), .dispatch_tag_o(tags),
    .completion_valid_i(wb_valid), .completion_i(wb), .completion_ready_o(wb_ready),
    .completion_accepted_o(wb_accepted), .completion_rejected_o(wb_rejected),
    .head_valid_o(head_valid), .head_complete_o(head_complete), .head_entry_o(heads),
    .head_ready_i(head_ready), .head_trap_target_i(32'h100),
    .retire_fire_o(retire_fire), .trap_fire_o(trap_fire), .retirement_event_o(events),
    .recovery_required_o(recovery), .mispredict_pending_o(mispredict),
    .mispredict_tag_o(mispredict_tag), .mispredict_next_pc_o(mispredict_pc),
    .serial_pending_o(serial_pending), .fault_pending_o(fault_pending),
    .flush_i(apply), .flush_done_o(rob_done), .query_tag_i(wb.lane0.rob_tag),
    .query_live_o(query_live), .query_age_o(query_age), .head_index_o(head_index),
    .occupancy_o(rob_occupancy), .full_o(full), .empty_o(empty), .protocol_error_o(rob_error)
  );
  rv32_physical_regfile prf (
    .clk_i(clk), .rst_ni(rst_n),
    .read0_rs1_i(inspect_mode ? committed_phys : r00), .read0_rs2_i(r01),
    .read1_rs1_i(r10), .read1_rs2_i(r11),
    .read0_data1_o(v00), .read0_data2_o(v01),
    .read1_data1_o(v10), .read1_data2_o(v11),
    .write_enable_i(write_enable), .write0_addr_i(write0_addr),
    .write1_addr_i(write1_addr), .write0_data_i(write0_data),
    .write1_data_i(write1_data), .write_conflict_o(write_error)
  );
  rv32_issue_queue iq (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_hold_i(admission_hold),
    .issue_hold_i(recovery || cancel || !issue_enable),
    .dispatch_valid_i(rename_fire), .dispatch_uop_i(renamed),
    .dispatch_src1_ready_i(src1_ready), .dispatch_src2_ready_i(src2_ready),
    .dispatch_ready_o(iq_ready), .dispatch_fire_o(iq_fire),
    .completion_valid_i(broadcast_valid), .completion_i(broadcast),
    .rob_head_valid_i(head_valid[0]), .rob_head_index_i(head_index),
    .rob_head_tag_i(heads.lane0.uop.rob_tag),
    .mispredict_pending_i(mispredict), .mispredict_tag_i(mispredict_tag),
    .alu0_ready_i(alu0_available), .alu1_ready_i(alu1_available),
    .muldiv_ready_i(md_available), .memory_ready_i(1'b0), .system_ready_i(1'b0),
    .issue_fire_o(issue_fire), .issue_packet_o(issued),
    .read0_rs1_o(r00), .read0_rs2_o(r01), .read1_rs1_o(r10), .read1_rs2_o(r11),
    .read0_data1_i(v00), .read0_data2_i(v01),
    .read1_data1_i(v10), .read1_data2_i(v11),
    .flush_i(apply), .flush_done_o(iq_done), .occupancy_o(iq_occupancy),
    .occupied_mask_o(occupied), .protocol_error_o(iq_error)
  );
  rv32_execution_cluster execute (
    .clk_i(clk), .rst_ni(rst_n), .issue_fire_i(issue_fire), .issue_packet_i(issued),
    .alu0_ready_o(alu0_available), .alu1_ready_o(alu1_available),
    .muldiv_ready_o(md_available), .memory_ready_o(memory_available),
    .alu_completion_valid_o(alu_valid), .alu_completion_ready_i(alu_ready),
    .alu_completion_o(alu_completion), .muldiv_completion_valid_o(md_valid),
    .muldiv_completion_ready_i(md_ready), .muldiv_completion_o(md_completion),
    .memory_valid_o(memory_valid), .memory_ready_i(1'b0), .memory_o(memory),
    .cancel_i(cancel), .cancel_done_o(cancel_done), .busy_o(cluster_busy),
    .quiescent_o(cluster_quiescent), .protocol_error_o(cluster_error)
  );
  rv32_trace_recorder recorder (
    .clk_i(clk), .rst_ni(rst_n), .event_i(events), .trace_o(trace),
    .protocol_error_o(trace_error)
  );

  // Stall both sides of the real ROB interface together. The DUT retains results;
  // the ROB must not consume a packet on an edge that writeback sees as blocked.
  assign wb = wb_raw;
  assign wb_valid = wb_raw_valid & {2{wb_allow}};
  assign source_valid = {2'b00, md_valid, alu_valid};
  assign alu_ready = source_ready[1:0];
  assign md_ready = source_ready[2];
  always_comb begin
    sources = '0;
    sources.alu0_result = alu_completion.lane0;
    sources.alu1_result = alu_completion.lane1;
    sources.muldiv_result = md_completion;
    // Memory/system completion producers are inactive in this arithmetic program.
    sources.memory_result = '0;
    sources.system_result = '0;
  end
  rv32_writeback writeback (
    .clk_i(clk), .rst_ni(rst_n), .source_valid_i(source_valid),
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

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 35)
          $display("FAIL: %s mode=%0d cycle=%0d time=%0t", label_text,
                   predict_enabled, cycle_number, $time);
      end
    end
  endtask

  function automatic word_t instruction_at(input word_t pc);
    case (pc)
      0: instruction_at = 32'h00400093;  // addi x1,x0,4
      4: instruction_at = 32'h00000113;  // addi x2,x0,0
      8: instruction_at = 32'h00310113;  // addi x2,x2,3
      12: instruction_at = 32'hfff08093; // addi x1,x1,-1
      16: instruction_at = 32'hfe009ce3; // bne x1,x0,-8
      20: instruction_at = 32'h00110193; // addi x3,x2,1
      default: instruction_at = NOP_INSTRUCTION;
    endcase
  endfunction

  // The independent architectural reference follows retired PCs, not speculative dispatch.
  task automatic observe_retirement(
    input retire_trace_t event_record, input rob_entry_t owner, output logic final_event
  );
    word_t value, next_pc;
    arch_reg_t destination;
    retire_trace_t stamped;
    logic taken;
    begin
      final_event = 0;
      destination = 0; value = 0; next_pc = reference_pc + 4; taken = 0;
      check(retired_count < 15 && event_record.pc == reference_pc
            && event_record.instruction == instruction_at(reference_pc),
            "only the next reference-path RAM instruction retires");
      case (reference_pc)
        0: begin destination = 1; value = 4; end
        4: begin destination = 2; value = 0; end
        8: begin destination = 2; value = reference_regs[2] + 3; end
        12: begin destination = 1; value = reference_regs[1] - 1; end
        16: begin
          taken = reference_regs[1] != 0;
          next_pc = taken ? 8 : 20;
          check(branch_count < 4 && taken == (branch_count < 3),
                "loop retirement outcomes are taken, taken, taken, then exit");
          check(owner.control_flow.is_control && owner.control_flow.is_conditional
                && owner.control_flow.taken == taken && owner.control_flow.next_pc == next_pc,
                "real comparator and target adder resolve the reference branch");
          check(owner.uop.decoded.fetch.prediction.pht_index == 4,
                "retired branch retains its actual PC-indexed PHT identity");
          if (owner.uop.decoded.fetch.prediction.next_pc == 8)
            predicted_taken_count = predicted_taken_count + 1;
          if (predict_enabled && branch_count != 0)
            check(owner.uop.decoded.fetch.prediction.next_pc == 8,
                  "after first training, the loop branch is fetched with its BTB target");
          else check(owner.uop.decoded.fetch.prediction.next_pc == 20,
                     "cold or disabled lookup predicts the branch fall-through");
          branch_count = branch_count + 1;
        end
        20: begin destination = 3; value = reference_regs[2] + 1; final_event = 1; end
        default: check(0, "no following NOP or wrong-path instruction may retire");
      endcase
      check(event_record.rd == destination && event_record.rd_value == value
            && event_record.next_pc == next_pc,
            "retirement value, destination, and next PC match the architectural reference");
      check(!event_record.trap && !event_record.interrupt && !event_record.memory_write,
            "loop has no trap, interrupt, or memory side effect");
      stamped = event_record;
      stamped.order = retired_count;
      if (retired_count < 15) begin
        if (predict_enabled) enabled_trace[retired_count] = stamped;
        else check(stamped == enabled_trace[retired_count],
                   "disabled and enabled prediction produce identical architectural events");
      end
      if (destination != 0) reference_regs[destination] = value;
      reference_regs[0] = 0;
      reference_pc = next_pc;
      retired_count = retired_count + 1;
      $display("BLOCK10 RETIRE: enabled=%0d cycle=%0d pc=%0d rd=x%0d value=%0d next=%0d",
               predict_enabled, cycle_number, event_record.pc, destination, value, next_pc);
    end
  endtask

  task automatic check_register_result(input integer lane, input completion_t packet);
    logic should_write;
    completion_t observed;
    begin
      should_write = wb_accepted[lane] && packet.writes_rd && packet.pdst != 0
                     && !packet.exception.valid;
      check(broadcast_valid[lane] == should_write && write_enable[lane] == should_write,
            "only an accepted real register result writes and wakes dependents");
      observed = lane == 0 ? broadcast.lane0 : broadcast.lane1;
      if (should_write)
        check(observed == packet, "accepted value and identity reach wakeup intact");
      else check(observed == '0, "nonwriting branches publish no register-wakeup packet");
    end
  endtask

  task automatic edge_step;
    retire_trace_pair_t expected_trace;
    integer order_base;
    recovery_phase_e sampled_phase;
    logic final0, final1, recover_now;
    word_t resolved_target;
    begin
      #1;
      sampled_phase = phase;
      recover_now = 0; resolved_target = recovery_pc;
      final0 = 0; final1 = 0;
      check(rename_fire == rob_fire && rename_fire == iq_fire && rename_fire == fetched_fire,
            "paired fetch consumption matches atomic backend allocation");
      check(!rename_error && !rob_error && !iq_error && !trace_error && !write_error
            && !cluster_error && !writeback_error && !fetch_error && !adapter_error
            && !predictor_error, "every production connection obeys its protocol");
      check(wb_rejected == 0 && trap_fire == 0 && !memory_valid && !fetch_fault,
            "legal loop execution has no rejected completion or architectural fault");
      check(fetch_occupancy <= FETCH_DEPTH, "front-end credits stay within the four-entry FIFO");
      if (sampled_phase != RUN)
        check(rename_fire == 0 && issue_fire == 0 && wb_valid == 0 && retire_fire == 0,
              "recovery barrier suppresses dispatch, issue, writeback, and retirement");
      dispatched_count = dispatched_count + int'(rename_fire[0]) + int'(rename_fire[1]);
      accepted_count = accepted_count + int'(wb_accepted[0]) + int'(wb_accepted[1]);
      check_register_result(0, wb.lane0);
      check_register_result(1, wb.lane1);
      order_base = retired_count;
      expected_trace = '0;
      if (events.lane0.valid) begin
        expected_trace.lane0 = events.lane0;
        expected_trace.lane0.order = order_base;
        observe_retirement(events.lane0, heads.lane0, final0);
      end
      if (events.lane1.valid) begin
        expected_trace.lane1 = events.lane1;
        expected_trace.lane1.order = order_base + 1;
        observe_retirement(events.lane1, heads.lane1, final1);
      end
      if (retire_fire[0] && heads.lane0.control_flow.mispredicted) begin
        recover_now = 1; resolved_target = heads.lane0.control_flow.next_pc;
      end else if (retire_fire[1] && heads.lane1.control_flow.mispredicted) begin
        recover_now = 1; resolved_target = heads.lane1.control_flow.next_pc;
      end
      if (recover_now)
        check(sampled_phase == RUN && !final0 && !final1,
              "test-only recovery begins only after a mispredicted branch actually retires");
      #1; clk = 1; #1;
      check(trace == expected_trace,
            "registered trace preserves complete ordered reference events");
      clk = 0;
      // Change driver control only AFTER the retirement/cancellation edge it describes.
      case (sampled_phase)
        RUN: begin
          if (recover_now) begin
            recovery_pc = resolved_target;
            recovery_count = recovery_count + 1;
            phase = CANCEL;
          end
          if (final0 || final1) finished = 1;
        end
        CANCEL: begin
          check(cancel_done && writeback_done && cluster_quiescent && writeback_quiescent,
                "execution and completion buffers acknowledge empty ownership before safe apply");
          if (cancel_done && writeback_done && cluster_quiescent && writeback_quiescent)
            phase = APPLY;
        end
        APPLY: begin
          check(rename_done && rob_done && iq_done && redirect_done
                && empty && iq_occupancy == 0,
                "safe apply restores maps/queues and redirects fetch after all producer clears");
          phase = RUN;
        end
        default: check(0, "test-only recovery driver has a defined phase");
      endcase
      cycle_number = cycle_number + 1;
      #1;
    end
  endtask

  task automatic reset_run(input logic enabled);
    begin
      clk = 0; rst_n = 0; phase = RUN; finished = 0; inspect_mode = 0;
      predict_enabled = enabled; recovery_pc = 0; invalidate = 0; committed_arch = 0;
      reference_pc = 0; retired_count = 0; branch_count = 0; recovery_count = 0;
      dispatched_count = 0; accepted_count = 0; predicted_taken_count = 0; cycle_number = 0;
      for (integer reg_id = 0; reg_id < 32; reg_id++) reference_regs[reg_id] = 0;
      #3;
      check(empty && free_count == 32 && cluster_quiescent && writeback_quiescent,
            "global reset starts the next architectural comparison with no old owners");
      rst_n = 1; #1;
    end
  endtask

  task automatic inspect_committed(input arch_reg_t architectural, input word_t expected_value);
    begin
      committed_arch = architectural; #1;
      check(ready_mask[committed_phys] && v00 == expected_value,
            "restored committed physical mapping contains the final reference value");
    end
  endtask

  initial begin #100000; $fatal(1, "Block 10 integration timed out"); end
  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block10/block10_integration.vcd");
      $dumpvars(0, tb_block10_integration);
    end
    for (row = 0; row < RAM_FETCH_ROWS; row++) rows[row] = {NOP_INSTRUCTION, NOP_INSTRUCTION};
    for (row = 0; row < 3; row++) rows[row] = {instruction_at(row*8+4), instruction_at(row*8)};

    for (mode = 0; mode < 2; mode++) begin
      reset_run(mode == 0);
      watchdog = 0;
      while (!finished && watchdog < 500) begin edge_step; watchdog = watchdog + 1; end
      check(watchdog < 500 && retired_count == 15 && branch_count == 4,
            "four loop iterations retire exactly fifteen architectural instructions");
      check(reference_pc == 24 && reference_regs[1] == 0 && reference_regs[2] == 12
            && reference_regs[3] == 13, "loop arithmetic and exit reach the expected final state");
      check(recovery_count == (predict_enabled ? 2 : 3)
            && predicted_taken_count == (predict_enabled ? 3 : 0),
            "retirement learning avoids repeated taken misses but still detects loop exit");
      if (predict_enabled) begin
        enabled_cycles = cycle_number; enabled_recoveries = recovery_count;
      end
      else begin disabled_cycles = cycle_number; disabled_recoveries = recovery_count; end
      $display("BLOCK10 LOOP: enabled=%0d cycles=%0d recoveries=%0d retired=%0d",
               predict_enabled, cycle_number, recovery_count, retired_count);

      // Test-only final cleanup removes younger speculative NOPs without retiring them.
      phase = CANCEL; recovery_pc = 16;
      watchdog = 0;
      while (phase != RUN && watchdog < 5) begin edge_step; watchdog = watchdog + 1; end
      check(empty && iq_occupancy == 0 && cluster_quiescent && writeback_quiescent
            && free_count == 32 && retired_count == 15,
            "post-program speculative cleanup preserves exactly the committed architectural state");
      inspect_mode = 1;
      inspect_committed(0, 0); inspect_committed(1, 0);
      inspect_committed(2, 12); inspect_committed(3, 13);
      check(prediction_pc0 == 16 && counter0 == 2'b10,
            "TTTN retirement training leaves the branch PHT counter weakly taken");
      if (predict_enabled)
        check(prediction.lane0.btb_hit && prediction.lane0.taken
              && prediction.lane0.next_pc == 8,
              "loop BTB retains its static target even after a not-taken exit");
      else check(!prediction.lane0.btb_hit && !prediction.lane0.taken
                 && prediction.lane0.next_pc == 20,
                 "disabled lookup stays sequential while retirement training remains active");
      invalidate = 1; #1;
      check(!prediction.lane0.btb_hit && prediction.lane0.next_pc == 20,
            "external invalidation immediately masks stale lookup results");
      edge_step; invalidate = 0; #1;
      check(invalidate_done && counter0 == 2'b01 && !prediction.lane0.btb_hit,
            "invalidation resets loop target/direction state without inventing a FENCE.I engine");
    end
    check(enabled_recoveries < disabled_recoveries && enabled_cycles < disabled_cycles,
          "this measured loop uses fewer recoveries and cycles with prediction enabled");
    $display("BLOCK10 COMPARISON: enabled=%0d cycles/%0d recoveries, disabled=%0d/%0d",
             enabled_cycles, enabled_recoveries, disabled_cycles, disabled_recoveries);
    $display("BLOCK10_INTEGRATION SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Block 10 predictor integration failed");
    $finish;
  end
endmodule
