// Connected OoO execution test. Only completion selection is a test driver.
// Every arithmetic value comes from real ALU or iterative multiply/divide RTL.
`timescale 1ns / 1ps
module tb_block07_integration;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1;
  logic [1:0] front_valid, rename_ready, rename_fire, rob_ready, rob_fire;
  logic [1:0] iq_ready, iq_fire, src1_ready, src2_ready, wb_valid;
  logic [1:0] wb_ready, wb_accepted, wb_rejected, head_valid, head_complete;
  logic [1:0] head_ready, retire_fire, trap_fire, write_enable, issue_fire;
  logic [1:0] alu_valid, alu_ready;
  decoded_pair_t decoded;
  renamed_pair_t renamed, retiring, demo_pair, demo_consumer, saved_pair, replay_pair;
  rob_tag_pair_t tags;
  completion_pair_t wb, alu_completion;
  completion_t md_completion, held0, held1, held_md;
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
  rob_tag_t mispredict_tag, cancelled_div_tag, cancelled_add_tag;
  logic alu0_available, alu1_available, md_available, memory_available;
  logic md_valid, md_ready, memory_valid, apply, cancel, cancel_done;
  logic [2:0] cluster_busy;
  logic cluster_quiescent, cluster_error, wb_allow, issue_enable, admission_hold;
  logic rename_done, rob_done, iq_done, rename_error, rob_error, iq_error;
  logic trace_error, write_error, recovery, mispredict, serial_pending, fault_pending;
  logic query_live, full, empty, guard_cancelled;
  renamed_uop_t expected_uop [0:63];
  word_t expected_pc [0:63];
  word_t expected_value [0:63];
  arch_reg_t expected_rd [0:63];
  integer expected_count = 0, retired_count = 0;
  integer checks = 0, failures = 0, cycle_number = 0, watchdog;
  integer div_issue_cycle = -1, add_issue_cycle = -1, consumer_issue_cycle = -1;
  integer div_complete_cycle = -1, add_complete_cycle = -1;
  integer completion_count = 0, cancelled_completion_count = 0;

  rv32_rename rename (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_valid_i(front_valid),
    .dispatch_resources_ready_i(rob_ready & iq_ready), .decoded_i(decoded),
    .dispatch_rob_tag_i(tags), .dispatch_ready_o(rename_ready), .dispatch_fire_o(rename_fire),
    .renamed_o(renamed), .src1_ready_o(src1_ready), .src2_ready_o(src2_ready),
    .completion_valid_i(wb_accepted), .completion_i(wb),
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
  assign write_enable[0] = wb_accepted[0] && wb.lane0.writes_rd && !wb.lane0.exception.valid;
  assign write_enable[1] = wb_accepted[1] && wb.lane1.writes_rd && !wb.lane1.exception.valid;
  rv32_physical_regfile prf (
    .clk_i(clk), .rst_ni(rst_n),
    .read0_rs1_i(r00), .read0_rs2_i(r01), .read1_rs1_i(r10), .read1_rs2_i(r11),
    .read0_data1_o(v00), .read0_data2_o(v01),
    .read1_data1_o(v10), .read1_data2_o(v11),
    .write_enable_i(write_enable), .write0_addr_i(wb.lane0.pdst),
    .write1_addr_i(wb.lane1.pdst), .write0_data_i(wb.lane0.value),
    .write1_data_i(wb.lane1.value), .write_conflict_o(write_error)
  );
  rv32_issue_queue iq (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_hold_i(admission_hold),
    .issue_hold_i(recovery || cancel || !issue_enable),
    .dispatch_valid_i(rename_fire), .dispatch_uop_i(renamed),
    .dispatch_src1_ready_i(src1_ready), .dispatch_src2_ready_i(src2_ready),
    .dispatch_ready_o(iq_ready), .dispatch_fire_o(iq_fire),
    .completion_valid_i(wb_accepted), .completion_i(wb),
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

  // Test-only two-result selection. Real completion arbitration is block 8.
  // An unselected third result receives no ready and must remain in its unit.
  always_comb begin : test_completion_selection
    integer slot;
    slot = 0;
    wb = '0; wb_valid = 0; alu_ready = 0; md_ready = 0;
    if (rst_n && wb_allow && !cancel && !apply) begin
      if (alu_valid[0]) begin
        wb.lane0 = alu_completion.lane0; wb_valid[0] = 1;
        alu_ready[0] = wb_ready[0]; slot = 1;
      end
      if (alu_valid[1]) begin
        if (slot == 0) begin
          wb.lane0 = alu_completion.lane1; wb_valid[0] = 1;
          alu_ready[1] = wb_ready[0];
        end else begin
          wb.lane1 = alu_completion.lane1; wb_valid[1] = 1;
          alu_ready[1] = wb_ready[1];
        end
        slot = slot + 1;
      end
      if (md_valid && slot < 2) begin
        if (slot == 0) begin
          wb.lane0 = md_completion; wb_valid[0] = 1; md_ready = wb_ready[0];
        end else begin
          wb.lane1 = md_completion; wb_valid[1] = 1; md_ready = wb_ready[1];
        end
      end
    end
  end

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        $display("FAIL: %s cycle=%0d time=%0t", label_text, cycle_number, $time);
      end
    end
  endtask

  function automatic decoded_uop_t operation(
    input integer pc, kind, rd, rs1, rs2, immediate
  );
    decoded_uop_t result;
    begin
      result = '0;
      result.fetch.pc = pc;
      result.fetch.prediction.next_pc = pc + 4;
      result.op_class = UOP_INTEGER; result.alu_op = ALU_ADD;
      result.src_a = SRC_A_REGISTER; result.src_b = SRC_B_REGISTER;
      result.rd = rd; result.rs1 = rs1; result.rs2 = rs2;
      result.uses_rs1 = 1; result.uses_rs2 = kind != 0; result.writes_rd = rd != 0;
      if (kind == 0) begin
        result.src_b = SRC_B_IMMEDIATE;
        result.immediate = immediate;
        result.fetch.instruction = ((immediate & 4095) << 20) | (rs1 << 15)
                                   | (rd << 7) | 7'h13;
      end else begin
        result.fetch.instruction = (rs2 << 20) | (rs1 << 15) | (rd << 7) | 7'h33;
        if (kind == 2) begin
          result.op_class = UOP_MULDIV; result.muldiv_op = MD_DIV;
          result.fetch.instruction = result.fetch.instruction | 32'h02004000;
        end
      end
      operation = result;
    end
  endfunction

  task automatic observe_issue(input execute_packet_t packet);
    begin
      if (packet.uop.decoded.fetch.pc == 16) begin
        div_issue_cycle = cycle_number;
        check(packet.rs1_value == 100 && packet.rs2_value == 4,
              "DIV receives committed ADDI seed operands");
      end
      if (packet.uop.decoded.fetch.pc == 20) begin
        add_issue_cycle = cycle_number;
        check(packet.rs1_value == 7 && packet.rs2_value == 9,
              "independent ADD receives real physical-register values");
      end
      if (packet.uop.decoded.fetch.pc == 24) begin
        consumer_issue_cycle = cycle_number;
        check(div_complete_cycle == cycle_number,
              "dependent issues on the accepted DIV completion edge");
        check(packet.rs1_value == 25 && packet.rs2_value == 16,
              "dependent ADD captures DIV bypass and earlier ADD value");
      end
    end
  endtask

  task automatic observe_completion(input completion_t packet);
    integer index;
    logic found;
    renamed_uop_t owner;
    begin
      found = 0;
      completion_count = completion_count + 1;
      for (index = retired_count; index < expected_count; index = index + 1) begin
        owner = expected_uop[index];
        if (packet.rob_tag == owner.rob_tag) begin
          found = 1;
          check(packet.pdst == owner.pdst && packet.value == expected_value[index],
                "real execution result matches its allocated owner and expected arithmetic");
          check(!packet.exception.valid && packet.control_flow.next_pc == expected_pc[index] + 4,
                "real completion carries sequential next PC without a spurious fault");
        end
      end
      check(found, "accepted completion belongs to an expected live instruction");
      if (packet.rob_tag == demo_pair.lane0.rob_tag && expected_count >= 7) begin
        div_complete_cycle = cycle_number;
        check(packet.value == 25, "iterative DIV produces 100 / 4 = 25");
      end
      if (packet.rob_tag == demo_pair.lane1.rob_tag && expected_count >= 7) begin
        add_complete_cycle = cycle_number;
        check(packet.value == 16 && div_complete_cycle < 0,
              "independent ADD completes while older DIV remains unfinished");
      end
      if (guard_cancelled && (packet.rob_tag == cancelled_div_tag
                            || packet.rob_tag == cancelled_add_tag)) begin
        cancelled_completion_count = cancelled_completion_count + 1;
        check(0, "cancelled execution owner must never publish a delayed result");
      end
    end
  endtask

  task automatic observe_retirement(input retire_trace_t event_record);
    begin
      check(retired_count < expected_count, "retirement has an expected architectural instruction");
      if (retired_count < expected_count) begin
        check(event_record.pc == expected_pc[retired_count]
              && event_record.rd == expected_rd[retired_count]
              && event_record.rd_value == expected_value[retired_count],
              "retirement follows program order and expected architectural values");
      end
      check(!event_record.trap && !event_record.interrupt && !event_record.memory_write,
            "arithmetic demonstration retires without trap or memory side effects");
      if (event_record.pc == 16 || event_record.pc == 20 || event_record.pc == 24)
        $display("BLOCK07 RETIRE: cycle=%0d pc=%0d rd=x%0d value=%0d",
                 cycle_number, event_record.pc, event_record.rd, event_record.rd_value);
      retired_count = retired_count + 1;
    end
  endtask

  task automatic edge_step;
    retire_trace_pair_t expected_trace;
    integer order_base;
    begin
      #1;
      check(rename_fire == rob_fire && rename_fire == iq_fire,
            "atomic dispatch agrees across rename, ROB, and IQ");
      check(!rename_error && !rob_error && !iq_error && !trace_error
            && !write_error && !cluster_error,
            "connected execution has no protocol violations");
      check(wb_rejected == 0 && trap_fire == 0 && !memory_valid && !mispredict,
            "legal live execution has no rejected result, trap, MMIO, or redirect");
      if (wb_accepted[0]) observe_completion(wb.lane0);
      if (wb_accepted[1]) observe_completion(wb.lane1);
      if (issue_fire[0]) observe_issue(issued.lane0);
      if (issue_fire[1]) observe_issue(issued.lane1);
      order_base = retired_count;
      expected_trace = '0;
      if (events.lane0.valid) begin
        expected_trace.lane0 = events.lane0;
        expected_trace.lane0.order = order_base;
        observe_retirement(events.lane0);
      end
      if (events.lane1.valid) begin
        expected_trace.lane1 = events.lane1;
        expected_trace.lane1.order = order_base + 1;
        observe_retirement(events.lane1);
      end
      #1; clk = 1; #1;
      check(trace == expected_trace, "registered trace preserves full ordered event records");
      clk = 0; cycle_number = cycle_number + 1; #1;
    end
  endtask

  task automatic send_instructions(
    input decoded_uop_t older, younger,
    input logic [1:0] valid_mask,
    input word_t result0, result1,
    output renamed_pair_t captured
  );
    begin
      decoded.lane0 = older; decoded.lane1 = younger; front_valid = valid_mask; #1;
      check(rename_fire == valid_mask, "available resources accept the offered instruction prefix");
      captured = renamed;
      if (rename_fire[0]) begin
        expected_uop[expected_count] = renamed.lane0;
        expected_pc[expected_count] = older.fetch.pc;
        expected_rd[expected_count] = older.rd;
        expected_value[expected_count] = result0;
        expected_count = expected_count + 1;
      end
      if (rename_fire[1]) begin
        expected_uop[expected_count] = renamed.lane1;
        expected_pc[expected_count] = younger.fetch.pc;
        expected_rd[expected_count] = younger.rd;
        expected_value[expected_count] = result1;
        expected_count = expected_count + 1;
      end
      edge_step; front_valid = 0; #1;
    end
  endtask

  task automatic drain;
    begin
      watchdog = 0;
      while ((!empty || iq_occupancy != 0 || !cluster_quiescent) && watchdog < 100) begin
        edge_step; watchdog = watchdog + 1;
      end
      check(watchdog < 100 && empty && iq_occupancy == 0 && cluster_quiescent,
            "all dispatched work drains with no lost owner or completion");
      check(retired_count == expected_count && free_count == 32,
            "ordered retirement returns replaced physical identities to the free list");
    end
  endtask

  initial begin #100000; $fatal(1, "Connected execution test timed out"); end
  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block07/block07_integration.vcd");
      $dumpvars(0, tb_block07_integration);
    end
    front_valid = 0; decoded = '0; apply = 0; cancel = 0; guard_cancelled = 0;
    cancelled_div_tag = '0; cancelled_add_tag = '0;
    demo_pair = '0; demo_consumer = '0; saved_pair = '0;
    head_ready = 3; wb_allow = 1; issue_enable = 1; admission_hold = 0; committed_arch = 5;
    #1; rst_n = 0; #2; rst_n = 1; #1;
    check(empty && iq_occupancy == 0 && free_count == 32 && cluster_quiescent,
          "reset initializes the connected architectural and execution structures");

    // Real ADDI instructions establish x1..x4 through rename, issue, execution, and retirement.
    send_instructions(operation(0, 0, 1, 0, 0, 100), operation(4, 0, 2, 0, 0, 4),
                      3, 100, 4, saved_pair);
    send_instructions(operation(8, 0, 3, 0, 0, 7), operation(12, 0, 4, 0, 0, 9),
                      3, 7, 9, saved_pair);
    drain;
    check(retired_count == 4, "four seed instructions retire without direct storage injection");

    // The ADD executes independently; the final ADD remains dependent on both physical values.
    send_instructions(operation(16, 2, 5, 1, 2, 0), operation(20, 1, 6, 3, 4, 0),
                      3, 25, 16, demo_pair);
    send_instructions(operation(24, 1, 7, 5, 6, 0), '0, 1, 41, 0, demo_consumer);
    check(consumer_issue_cycle < 0 && div_complete_cycle < 0,
          "dependent ADD waits while iterative DIV runs");
    drain;
    check(add_issue_cycle >= 0 && div_issue_cycle >= 0
          && add_complete_cycle < div_complete_cycle
          && consumer_issue_cycle == div_complete_cycle,
          "trace proves out-of-order execution/completion and same-cycle RAW wakeup");
    check(retired_count == 7 && committed_phys == demo_pair.lane0.pdst,
          "DIV, independent ADD, and dependent ADD commit in their original order");
    $display("BLOCK07 OOO: DIV issue=%0d, ADD issue=%0d, ADD complete=%0d,",
             div_issue_cycle, add_issue_cycle, add_complete_cycle);
    $display("             DIV complete=%0d, dependent issue=%0d; results 25, 16, 41",
             div_complete_cycle, consumer_issue_cycle);

    // Hold three real results while the two-write-port completion interface is stopped.
    wb_allow = 0; head_ready = 0;
    send_instructions(operation(28, 2, 8, 1, 2, 0), '0, 1, 25, 0, saved_pair);
    send_instructions(operation(32, 1, 9, 3, 4, 0), operation(36, 1, 10, 1, 2, 0),
                      3, 16, 104, saved_pair);
    watchdog = 0;
    while (!md_valid && watchdog < 40) begin edge_step; watchdog = watchdog + 1; end
    check(alu_valid == 3 && md_valid && wb_valid == 0,
          "three independently buffered completions coexist behind two writeback ports");
    held0 = alu_completion.lane0; held1 = alu_completion.lane1; held_md = md_completion;
    repeat (4) begin
      edge_step;
      check(alu_valid == 3 && md_valid && alu_completion.lane0 == held0
            && alu_completion.lane1 == held1 && md_completion == held_md,
            "all three completion packets remain stable under sustained backpressure");
      check(!alu0_available && !alu1_available && !md_available,
            "full execution buffers advertise no replacement capacity while blocked");
    end
    wb_allow = 1; #1;
    check(wb_valid == 3 && alu_ready == 3 && !md_ready,
          "two selected ALU results transfer while the third result is retained");
    edge_step;
    check(alu_valid == 0 && md_valid && md_completion == held_md,
          "unselected DIV result survives acceptance of the two ALU results");
    edge_step;
    check(cluster_quiescent && head_complete == 3,
          "DIV transfers next and all execution buffers become quiescent");
    head_ready = 3; drain;

    // Establish a reusable ROB slot boundary without changing committed values or trace order.
    issue_enable = 0; admission_hold = 1; apply = 1; edge_step; apply = 0; #1;
    check(rename_done && rob_done && iq_done && head_index == 0 && cluster_quiescent,
          "safe clear resets speculative queues after execution is already quiescent");
    issue_enable = 1; admission_hold = 0; wb_allow = 0; head_ready = 0;
    send_instructions(operation(40, 2, 11, 1, 2, 0), operation(44, 1, 12, 3, 4, 0),
                      3, 25, 16, saved_pair);
    cancelled_div_tag = saved_pair.lane0.rob_tag;
    cancelled_add_tag = saved_pair.lane1.rob_tag;
    repeat (4) edge_step;
    check(alu_valid != 0 && !md_valid && cluster_busy != 0 && !md_available,
          "cancellation fixture has a running DIV and a held integer result");
    issue_enable = 0; admission_hold = 1; cancel = 1; #1;
    check(alu_valid == 0 && !md_valid && wb_valid == 0 && issue_fire == 0,
          "cancel masks execution publication and issue before the cancellation edge");
    edge_step;
    check(cancel_done && cluster_quiescent && cluster_busy == 0,
          "cancellation acknowledges only after running and buffered owners are cleared");
    guard_cancelled = 1; cancel = 0;
    expected_count = retired_count;
    apply = 1; edge_step; apply = 0; #1;
    check(rename_done && rob_done && iq_done && empty && iq_occupancy == 0 && free_count == 32,
          "safe recovery restores committed rename state after execution cancellation");
    committed_arch = 12; #1;
    check(committed_phys == 12,
          "cancelled younger ADD cannot change its architectural destination mapping");

    // The same ROB index is allocated with a new generation; cancelled results cannot corrupt it.
    issue_enable = 1; admission_hold = 0; wb_allow = 1; head_ready = 3; committed_arch = 11;
    send_instructions(operation(48, 0, 11, 0, 0, 77), '0, 1, 77, 0, replay_pair);
    check(replay_pair.lane0.rob_tag.index == cancelled_div_tag.index
          && replay_pair.lane0.rob_tag.generation != cancelled_div_tag.generation,
          "post-cancel allocation reuses a ROB index with a different owner generation");
    drain;
    check(committed_phys == replay_pair.lane0.pdst,
          "replayed instruction establishes the correct committed physical mapping");
    repeat (40) edge_step;
    check(cancelled_completion_count == 0 && empty && cluster_quiescent
          && retired_count == 11 && expected_count == 11 && completion_count == 11,
          "no delayed cancelled result or extra retirement appears after the original DIV latency");

    $display("BLOCK07_INTEGRATION SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Connected execution test failed");
    $finish;
  end
endmodule
