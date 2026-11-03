// Connected rename/ROB/PRF/IQ test. Execution values are deliberately injected.
`timescale 1ns / 1ps
module tb_block06_integration;
  import rv32_ooo_pkg::*;
  logic clk = 0, rst_n = 1;
  logic [1:0] front_valid, rename_ready, rename_fire, rob_ready, rob_fire;
  logic [1:0] iq_ready, iq_fire, src1_ready, src2_ready, wb_valid;
  logic [1:0] wb_ready, wb_accepted, wb_rejected, head_valid, head_complete;
  logic [1:0] head_ready, retire_fire, trap_fire, write_enable, issue_fire;
  decoded_pair_t decoded;
  renamed_pair_t renamed, retiring, producer_pair, independent_pair;
  rob_tag_pair_t tags;
  completion_pair_t wb;
  rob_entry_pair_t heads;
  retire_trace_pair_t events, trace;
  execute_packet_pair_t issued;
  phys_reg_t r00, r01, r10, r11, committed_phys;
  word_t v00, v01, v10, v11, mispredict_pc;
  phys_mask_t free_mask, ready_mask;
  phys_count_t free_count;
  rob_index_t head_index, query_age;
  rob_count_t rob_occupancy;
  issue_count_t iq_occupancy;
  logic [ISSUE_DEPTH-1:0] occupied;
  rob_tag_t mispredict_tag;
  logic alu0_ready, alu1_ready, md_ready, apply;
  logic rename_done, rob_done, iq_done, rename_error, rob_error, iq_error;
  logic trace_error, write_error, recovery, mispredict, serial_pending, fault_pending;
  logic query_live, full, empty;
  integer checks = 0, failures = 0;

  rv32_rename rename (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_valid_i(front_valid),
    .dispatch_resources_ready_i(rob_ready & iq_ready), .decoded_i(decoded),
    .dispatch_rob_tag_i(tags), .dispatch_ready_o(rename_ready), .dispatch_fire_o(rename_fire),
    .renamed_o(renamed), .src1_ready_o(src1_ready), .src2_ready_o(src2_ready),
    .completion_valid_i(wb_accepted), .completion_i(wb),
    .retire_valid_i(retire_fire), .retire_uop_i(retiring),
    .recover_i(apply), .recover_done_o(rename_done),
    .committed_query_arch_i(5'd5), .committed_query_phys_o(committed_phys),
    .free_mask_o(free_mask), .ready_mask_o(ready_mask), .free_count_o(free_count),
    .protocol_error_o(rename_error)
  );
  assign retiring.lane0 = heads.lane0.uop;
  assign retiring.lane1 = heads.lane1.uop;
  rv32_reorder_buffer rob (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_hold_i(1'b0),
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
    .clk_i(clk), .rst_ni(rst_n), .dispatch_hold_i(1'b0), .issue_hold_i(recovery),
    .dispatch_valid_i(rename_fire), .dispatch_uop_i(renamed),
    .dispatch_src1_ready_i(src1_ready), .dispatch_src2_ready_i(src2_ready),
    .dispatch_ready_o(iq_ready), .dispatch_fire_o(iq_fire),
    .completion_valid_i(wb_accepted), .completion_i(wb),
    .rob_head_valid_i(head_valid[0]), .rob_head_index_i(head_index),
    .rob_head_tag_i(heads.lane0.uop.rob_tag),
    .mispredict_pending_i(mispredict), .mispredict_tag_i(mispredict_tag),
    .alu0_ready_i(alu0_ready), .alu1_ready_i(alu1_ready), .muldiv_ready_i(md_ready),
    .memory_ready_i(1'b0), .system_ready_i(1'b0),
    .issue_fire_o(issue_fire), .issue_packet_o(issued),
    .read0_rs1_o(r00), .read0_rs2_o(r01), .read1_rs1_o(r10), .read1_rs2_o(r11),
    .read0_data1_i(v00), .read0_data2_i(v01),
    .read1_data1_i(v10), .read1_data2_i(v11),
    .flush_i(apply), .flush_done_o(iq_done), .occupancy_o(iq_occupancy),
    .occupied_mask_o(occupied), .protocol_error_o(iq_error)
  );
  rv32_trace_recorder recorder (
    .clk_i(clk), .rst_ni(rst_n), .event_i(events), .trace_o(trace),
    .protocol_error_o(trace_error)
  );

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        $display("FAIL: %s time=%0t", label_text, $time);
      end
    end
  endtask
  task automatic edge_step;
    begin
      #1;
      check(rename_fire == rob_fire && rename_fire == iq_fire,
            "atomic dispatch agrees across rename, ROB, and IQ");
      check(!rename_error && !rob_error && !iq_error && !trace_error && !write_error,
            "legal connected traffic has no protocol violations");
      #1; clk = 1; #1; clk = 0; #1;
    end
  endtask
  function automatic decoded_uop_t operation(input integer pc, rd, rs1, rs2);
    decoded_uop_t result;
    begin
      result = '0;
      result.fetch.pc = pc;
      result.fetch.instruction = (rs2 << 20) | (rs1 << 15) | (rd << 7) | 7'h33;
      result.fetch.prediction.next_pc = pc + 4;
      result.op_class = UOP_INTEGER;
      result.alu_op = ALU_ADD;
      result.rd = rd;
      result.rs1 = rs1;
      result.rs2 = rs2;
      result.uses_rs1 = 1;
      result.uses_rs2 = 1;
      result.writes_rd = rd != 0;
      operation = result;
    end
  endfunction
  function automatic completion_t completed(input renamed_uop_t instruction, input word_t value);
    completion_t result;
    begin
      result = '0;
      result.rob_tag = instruction.rob_tag;
      result.pdst = instruction.pdst;
      result.writes_rd = instruction.decoded.writes_rd;
      result.value = value;
      result.control_flow.next_pc = instruction.decoded.fetch.pc + 4;
      completed = result;
    end
  endfunction

  initial begin #10000; $fatal(1, "Connected scheduler test timed out"); end
  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block06/block06_integration.vcd");
      $dumpvars(0, tb_block06_integration);
    end
    front_valid = 0; decoded = '0; wb_valid = 0; wb = '0; apply = 0;
    head_ready = 0; alu0_ready = 0; alu1_ready = 0; md_ready = 0;
    #1; rst_n = 0; #2; rst_n = 1; #1;
    check(empty && iq_occupancy == 0 && free_count == 32, "coupled startup state");

    decoded.lane0 = operation(0, 5, 1, 2);
    decoded.lane0.op_class = UOP_MULDIV;
    decoded.lane0.muldiv_op = MD_DIV;
    decoded.lane0.fetch.instruction = 32'h0220c2b3;
    decoded.lane1 = operation(4, 7, 5, 1);
    front_valid = 2'b11; #1; producer_pair = renamed;
    check(rename_fire == 2'b11 && !src1_ready[1],
          "intra-pair RAW captures the producer's new unready physical tag");
    edge_step; front_valid = 0; md_ready = 1; #1;
    check(issue_fire == 2'b10 && issued.lane1.uop == producer_pair.lane0,
          "older DIV launches on the multiply/divide path");
    edge_step; md_ready = 0; #1;
    check(iq_occupancy == 1 && issue_fire == 0,
          "dependent older ADD waits after DIV leaves the issue queue");

    decoded.lane0 = operation(8, 6, 3, 4);
    decoded.lane1 = operation(12, 8, 3, 4);
    front_valid = 2'b11; #1; independent_pair = renamed;
    check(rename_fire == 2'b11, "two independent younger instructions dispatch together");
    edge_step; front_valid = 0; alu0_ready = 1; alu1_ready = 1; #1;
    check(issue_fire == 2'b11 && issued.lane0.uop == independent_pair.lane0
          && issued.lane1.uop == independent_pair.lane1,
          "two younger ready ADDs bypass the older waiting ADD");
    check(issued.lane0.rs1_value == 0 && issued.lane1.rs2_value == 0,
          "issued packet carries PRF operand values as well as identities");
    edge_step; alu0_ready = 0; alu1_ready = 0; #1;
    check(iq_occupancy == 1 && rob_occupancy == 4,
          "issue removes queue entries without removing ROB owners");

    wb.lane0 = completed(independent_pair.lane0, 7);
    wb.lane1 = completed(independent_pair.lane1, 9); wb_valid = 2'b11; #1;
    check(wb_accepted == 2'b11 && issue_fire == 0,
          "both younger results are accepted while the RAW consumer still waits");
    edge_step; wb_valid = 0; #1;
    check(head_complete == 0 && retire_fire == 0 && committed_phys == 5,
          "younger completion cannot commit past incomplete DIV and dependent ADD");

    wb.lane0 = completed(producer_pair.lane0, 999);
    wb.lane0.rob_tag.generation = wb.lane0.rob_tag.generation + 1'b1;
    wb_valid = 2'b01; alu0_ready = 1; alu1_ready = 1; #1;
    check(wb_rejected == 2'b01 && write_enable == 0 && issue_fire == 0,
          "stale generation cannot write PRF or wake the waiting physical source");
    edge_step; wb_valid = 0; #1;
    check(iq_occupancy == 1 && !ready_mask[producer_pair.lane0.pdst],
          "rejected completion leaves readiness and queue ownership intact");

    wb.lane0 = completed(producer_pair.lane0, 40); wb_valid = 2'b01; #1;
    check(wb_accepted == 2'b01 && issue_fire == 2'b01,
          "accepted producer completion wakes the dependent in the same cycle");
    check(issued.lane0.uop == producer_pair.lane1
          && issued.lane0.rs1_value == 40 && issued.lane0.rs2_value == 0,
          "four-port PRF bypass supplies the newly awakened source before the edge");
    edge_step; wb_valid = 0; alu0_ready = 0; alu1_ready = 0; #1;
    check(iq_occupancy == 0 && rob_occupancy == 4 && head_complete == 2'b01,
          "completion and issue remain distinct from retirement");

    wb.lane0 = completed(producer_pair.lane1, 40); wb_valid = 2'b01;
    edge_step; wb_valid = 0; head_ready = 2'b11; #1;
    check(retire_fire == 2'b11 && events.lane0.pc == 0 && events.lane1.pc == 4,
          "first retirement pair is DIV then its consumer despite younger execution");
    edge_step;
    check(trace.lane0.order == 0 && trace.lane1.order == 1
          && trace.lane0.rd == 5 && trace.lane1.rd == 7,
          "trace records ordered architectural results for the first pair");
    check(retire_fire == 2'b11 && events.lane0.pc == 8 && events.lane1.pc == 12,
          "independent younger pair retires only after older pair");
    edge_step;
    check(empty && iq_occupancy == 0 && free_count == 32 && committed_phys == 32,
          "all owners retire and old physical mappings return to the free list");
    check(trace.lane0.order == 2 && trace.lane1.order == 3
          && trace.lane0.rd_value == 7 && trace.lane1.rd_value == 9,
          "younger results retain their values and retirement order");

    apply = 1; edge_step; apply = 0; #1;
    check(rename_done && rob_done && iq_done && empty && iq_occupancy == 0,
          "one safe recovery application clears all built speculative structures");
    $display("BLOCK06_INTEGRATION SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Connected scheduler test failed");
    $finish;
  end
endmodule
