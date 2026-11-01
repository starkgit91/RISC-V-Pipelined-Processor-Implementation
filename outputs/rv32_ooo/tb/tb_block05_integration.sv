// Couple the built control/storage blocks; results are injected, not executed by a CPU.
`timescale 1ns / 1ps
module tb_block05_integration;
  import rv32_ooo_pkg::*;
  logic clk = 0, rst_n = 1;
  logic [1:0] front_valid, external_resources;
  decoded_pair_t decoded;
  renamed_pair_t renamed, retiring, saved;
  rob_tag_pair_t tags;
  logic [1:0] rename_ready, rename_fire, rob_ready, rob_fire, source1_ready, source2_ready;
  logic [1:0] wb_valid, wb_ready, wb_accepted, wb_rejected, head_valid, head_complete;
  logic [1:0] retire_fire, trap_fire, write_enable;
  completion_pair_t wb;
  rob_entry_pair_t heads;
  retire_trace_pair_t events, trace;
  logic apply, rename_done, rob_done, rename_error, rob_error, trace_error, write_error;
  logic recovery, mispredict, serial_pending, fault_pending, query_live, full, empty;
  rob_tag_t mispredict_tag;
  word_t mispredict_pc;
  rob_index_t query_age, head_index;
  rob_count_t occupancy;
  phys_mask_t free_mask, ready_mask;
  phys_count_t free_count;
  arch_reg_t committed_query;
  phys_reg_t committed_phys;
  word_t value0, value1, value2, value3;
  phys_reg_t read0, read1, read2, read3;
  integer checks = 0, failures = 0;

  rv32_rename rename (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_valid_i(front_valid),
    .dispatch_resources_ready_i(rob_ready & external_resources), .decoded_i(decoded),
    .dispatch_rob_tag_i(tags), .dispatch_ready_o(rename_ready), .dispatch_fire_o(rename_fire),
    .renamed_o(renamed), .src1_ready_o(source1_ready), .src2_ready_o(source2_ready),
    .completion_valid_i(wb_accepted), .completion_i(wb),
    .retire_valid_i(retire_fire), .retire_uop_i(retiring),
    .recover_i(apply), .recover_done_o(rename_done),
    .committed_query_arch_i(committed_query), .committed_query_phys_o(committed_phys),
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
    .head_ready_i(2'b11), .head_trap_target_i(32'h100),
    .retire_fire_o(retire_fire), .trap_fire_o(trap_fire), .retirement_event_o(events),
    .recovery_required_o(recovery), .mispredict_pending_o(mispredict),
    .mispredict_tag_o(mispredict_tag), .mispredict_next_pc_o(mispredict_pc),
    .serial_pending_o(serial_pending), .fault_pending_o(fault_pending),
    .flush_i(apply), .flush_done_o(rob_done), .query_tag_i(wb.lane0.rob_tag),
    .query_live_o(query_live), .query_age_o(query_age), .head_index_o(head_index),
    .occupancy_o(occupancy), .full_o(full), .empty_o(empty), .protocol_error_o(rob_error)
  );
  assign write_enable[0] = wb_accepted[0] && wb.lane0.writes_rd && !wb.lane0.exception.valid;
  assign write_enable[1] = wb_accepted[1] && wb.lane1.writes_rd && !wb.lane1.exception.valid;
  rv32_physical_regfile prf (
    .clk_i(clk), .rst_ni(rst_n),
    .read0_rs1_i(read0), .read0_rs2_i(read1), .read1_rs1_i(read2), .read1_rs2_i(read3),
    .read0_data1_o(value0), .read0_data2_o(value1),
    .read1_data1_o(value2), .read1_data2_o(value3),
    .write_enable_i(write_enable), .write0_addr_i(wb.lane0.pdst),
    .write1_addr_i(wb.lane1.pdst), .write0_data_i(wb.lane0.value),
    .write1_data_i(wb.lane1.value), .write_conflict_o(write_error)
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
      check(rename_fire == rob_fire, "atomic dispatch agrees between rename and ROB");
      check(!rename_error && !rob_error && !trace_error && !write_error,
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

  initial begin #10000; $fatal(1, "Connected control test timed out"); end
  initial begin
    front_valid = 0; external_resources = 2'b11; decoded = '0;
    wb_valid = 0; wb = '0; apply = 0; committed_query = 5;
    read0 = 32; read1 = 33; read2 = 32; read3 = 33;
    #1; rst_n = 0; #2; rst_n = 1; #1;
    check(empty && free_count == 32 && committed_phys == 5, "coupled startup state");
    decoded.lane0 = operation(0, 5, 1, 2);
    decoded.lane0.op_class = UOP_MULDIV;
    decoded.lane0.muldiv_op = MD_DIV;
    decoded.lane0.fetch.instruction = 32'h0220c2b3;
    decoded.lane1 = operation(4, 6, 3, 4);
    front_valid = 2'b11; #1; saved = renamed;
    check(rename_fire == 2'b11 && saved.lane0.pdst == 32 && saved.lane1.pdst == 33,
          "two distinct physical destinations dispatch in one edge");
    edge_step; front_valid = 0; #1;
    check(occupancy == 2 && free_count == 30 && head_complete == 0,
          "both instructions are tracked without premature retirement");

    wb.lane0 = completed(saved.lane1, 19); wb_valid = 2'b01; #1;
    check(wb_accepted == 2'b01 && value1 == 19 && value3 == 19,
          "younger accepted result bypasses to both instruction lanes");
    edge_step; wb_valid = 0; #1;
    check(ready_mask[33] && !ready_mask[32] && retire_fire == 0,
          "younger result ready while older DIV blocks commitment");
    committed_query = 6; #1;
    check(committed_phys == 6 && value1 == 19, "speculative data does not change committed map");

    wb.lane0 = completed(saved.lane0, 6); wb_valid = 2'b01; #1;
    check(retire_fire == 0 && value0 == 6, "head completion does not bypass retirement state");
    edge_step; wb_valid = 0; #1;
    check(retire_fire == 2'b11, "completed older and younger instructions form retirement pair");
    edge_step;
    check(empty && free_count == 32, "dual retirement releases both replaced versions");
    check(trace.lane0.valid && trace.lane1.valid &&
          trace.lane0.order == 0 && trace.lane1.order == 1,
          "registered architectural trace preserves program order");
    check(trace.lane0.rd == 5 && trace.lane0.rd_value == 6 &&
          trace.lane1.rd == 6 && trace.lane1.rd_value == 19, "trace contains committed values");
    committed_query = 5; #1; check(committed_phys == 32, "x5 committed mapping advanced");
    committed_query = 6; #1; check(committed_phys == 33, "x6 committed mapping advanced");

    wb.lane1 = completed(saved.lane1, 999); wb_valid = 2'b10; #1;
    check(wb_rejected == 2'b10 && write_enable == 0 && value1 == 19,
          "retired-owner result is dropped before PRF bypass or write");
    edge_step; wb_valid = 0; #1;
    check(value1 == 19 && value3 == 19, "stale completion cannot corrupt stored value");

    apply = 1; edge_step; apply = 0; #1;
    check(empty && rename_done && rob_done && value0 == 6 && value1 == 19,
          "safe apply preserves committed data and clears ROB control");
    check(tags.lane0.generation == 1, "micro-flush retains the next allocation generation");

    // One backend slot permits only the older lane; the caller compacts the suffix next.
    decoded.lane0 = operation(8, 7, 5, 6);
    decoded.lane1 = operation(12, 8, 7, 6);
    front_valid = 2'b11; external_resources = 2'b01; #1;
    check(rename_fire == 2'b01 && rob_fire == 2'b01, "partial pair is accepted atomically");
    edge_step;
    decoded.lane0 = decoded.lane1; decoded.lane1 = '0;
    front_valid = 2'b01; external_resources = 2'b11; #1;
    check(renamed.lane0.prs1 == 5 && !source1_ready[0],
          "compacted suffix sees its older producer's new physical destination");
    edge_step; front_valid = 0;
    apply = 1; edge_step; apply = 0; #1;
    check(empty && free_count == 32 && value0 == 6 && value1 == 19,
          "cancelled unexecuted pair leaves committed values intact");

    $display("BLOCK05_INTEGRATION SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Connected two-wide control/storage test failed");
    $finish;
  end
endmodule
