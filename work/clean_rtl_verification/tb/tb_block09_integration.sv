// Synchronous native instruction memory through fetch/decode to real OoO retirement.
// The eight-instruction program is stored in RAM; no decoded controls or values are injected.
`timescale 1ns / 1ps
module tb_block09_integration;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1, probe_rst_n = 1;
  logic [1:0] front_valid, rename_ready, rename_fire, rob_ready, rob_fire;
  logic [1:0] iq_ready, iq_fire, src1_ready, src2_ready, wb_valid;
  logic [1:0] wb_ready, wb_accepted, wb_rejected, head_valid, head_complete;
  logic [1:0] head_ready, retire_fire, trap_fire, write_enable, issue_fire;
  logic [1:0] alu_valid, alu_ready, wb_raw_valid, broadcast_valid, writeback_occupancy;
  decoded_pair_t decoded;
  renamed_pair_t renamed, retiring;
  renamed_uop_t final_owner;
  completion_sources_t sources;
  logic [4:0] source_valid, source_ready;
  logic [1:0] synthetic_valid;
  completion_t synthetic_memory, synthetic_system;
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

  logic fetch_run, admission_enable, single_dispatch;
  logic [1:0] fetched_valid, fetched_fire;
  fetch_pair_t fetched, held_fetch;
  fetch_count_t fetch_occupancy;
  logic fetch_transaction, fetch_stale, fetch_fault, fetch_error, redirect_done;
  word_t prediction_pc0, prediction_pc1;
  logic request_valid, request_ready, response_valid, response_ready, response_error;
  word_t request_address;
  fetch_row_t response_data, bram_data;
  logic bram_block, bram_enable, adapter_pending, adapter_error;
  logic [BRAM_FETCH_ADDR_W-1:0] bram_address;
  fetch_row_t rows [0:RAM_FETCH_ROWS-1];
  word_t program_words [0:7];
  word_t expected_value [0:7];
  renamed_uop_t expected_owner [0:7];
  integer dispatched_count = 0, retired_count = 0, completion_count = 0;
  integer memory_inflight = 0, reads_requested = 0, responses_consumed = 0;
  word_t pending_address, stalled_address;
  logic request_was_stalled = 0;
  integer checks = 0, failures = 0, cycle_number = 0, watchdog, row;
  integer div_issue_cycle = -1, add_issue_cycle = -1, consumer_issue_cycle = -1;
  integer div_complete_cycle = -1, add_complete_cycle = -1;

  logic probe_request_valid, probe_request_ready, probe_response_valid, probe_response_ready;
  word_t probe_address;
  fetch_row_t probe_response_data, probe_bram_data, saved_probe_data;
  logic probe_response_error, probe_block, probe_enable, probe_pending, probe_error;
  logic [BRAM_FETCH_ADDR_W-1:0] probe_bram_address;

  rv32_fetch fetch (
    .clk_i(clk), .rst_ni(rst_n), .enable_i(fetch_run && dispatched_count < 8),
    .redirect_i(1'b0), .redirect_pc_i(32'd0), .redirect_done_o(redirect_done),
    .predict_enable_i(1'b0), .prediction_pc0_o(prediction_pc0),
    .prediction_pc1_o(prediction_pc1), .prediction_i(prediction_pair_t'('0)),
    .request_valid_o(request_valid), .request_ready_i(request_ready),
    .request_address_o(request_address), .response_valid_i(response_valid),
    .response_ready_o(response_ready), .response_data_i(response_data),
    .response_error_i(response_error), .fetch_valid_o(fetched_valid),
    .fetch_ready_i(rename_fire), .fetch_o(fetched), .fetch_fire_o(fetched_fire),
    .occupancy_o(fetch_occupancy), .transaction_pending_o(fetch_transaction),
    .stale_pending_o(fetch_stale), .fault_pending_o(fetch_fault), .protocol_error_o(fetch_error)
  );
  rv32_instruction_adapter instruction_memory (
    .clk_i(clk), .rst_ni(rst_n), .request_valid_i(request_valid),
    .request_ready_o(request_ready), .request_address_i(request_address),
    .response_valid_o(response_valid), .response_ready_i(response_ready),
    .response_data_o(response_data), .response_error_o(response_error),
    .bram_block_i(bram_block), .bram_enable_o(bram_enable), .bram_address_o(bram_address),
    .bram_data_i(bram_data), .transaction_pending_o(adapter_pending),
    .protocol_error_o(adapter_error)
  );
  always_ff @(posedge clk) begin
    if (bram_enable) bram_data <= rows[bram_address];
  end
  rv32_decoder decoder0 (.fetch_i(fetched.lane0), .decoded_o(decoded.lane0));
  rv32_decoder decoder1 (.fetch_i(fetched.lane1), .decoded_o(decoded.lane1));
  always_comb begin
    front_valid = '0;
    if (rst_n && admission_enable && dispatched_count < 8) begin
      front_valid[0] = fetched_valid[0];
      if (!single_dispatch && dispatched_count < 7) front_valid[1] = fetched_valid[1];
    end
  end

  // A separate adapter instance tests defensive address/error and native-port contracts.
  rv32_instruction_adapter adapter_probe (
    .clk_i(clk), .rst_ni(probe_rst_n), .request_valid_i(probe_request_valid),
    .request_ready_o(probe_request_ready), .request_address_i(probe_address),
    .response_valid_o(probe_response_valid), .response_ready_i(probe_response_ready),
    .response_data_o(probe_response_data), .response_error_o(probe_response_error),
    .bram_block_i(probe_block), .bram_enable_o(probe_enable),
    .bram_address_o(probe_bram_address), .bram_data_i(probe_bram_data),
    .transaction_pending_o(probe_pending), .protocol_error_o(probe_error)
  );
  function automatic fetch_row_t probe_pattern(input integer line_address);
    word_t low_word, high_word;
    begin
      low_word = 32'ha0000000 | (line_address * 2);
      high_word = low_word + 1;
      probe_pattern = {high_word, low_word};
    end
  endfunction
  always_ff @(posedge clk) begin
    if (probe_enable) probe_bram_data <= probe_pattern(probe_bram_address);
  end

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
    .read0_rs1_i(r00), .read0_rs2_i(r01), .read1_rs1_i(r10), .read1_rs2_i(r11),
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
  assign source_valid = {synthetic_valid, md_valid, alu_valid};
  assign alu_ready = source_ready[1:0];
  assign md_ready = source_ready[2];
  always_comb begin
    sources = '0;
    sources.alu0_result = alu_completion.lane0;
    sources.alu1_result = alu_completion.lane1;
    sources.muldiv_result = md_completion;
    // Memory/system completion producers are inactive in this arithmetic program.
    sources.memory_result = synthetic_memory;
    sources.system_result = synthetic_system;
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
        $display("FAIL: %s cycle=%0d time=%0t", label_text, cycle_number, $time);
      end
    end
  endtask

  task automatic raw_edge;
    begin #1; clk = 1; #1; clk = 0; #1; end
  endtask

  task automatic observe_dispatch(input renamed_uop_t owner, input integer position);
    begin
      check(position < 8, "only the eight RAM-program instructions are admitted");
      if (position < 8) begin
        check(owner.decoded.fetch.pc == position * 4
              && owner.decoded.fetch.instruction == program_words[position],
              "RAM instruction bits and matching PC survive fetch, decode, and rename");
        check(!owner.decoded.fetch.exception.valid && owner.decoded.rd == position + 1
              && owner.decoded.writes_rd,
              "real decoder recognizes the expected nonfaulting program destination");
        check(owner.decoded.fetch.prediction.next_pc == position * 4 + 4,
              "prediction-disabled fetch retains sequential next-PC metadata");
        expected_owner[position] = owner;
      end
    end
  endtask

  task automatic observe_issue(input execute_packet_t packet);
    begin
      case (packet.uop.decoded.fetch.pc)
        16: begin
          div_issue_cycle = cycle_number;
          check(packet.rs1_value == 100 && packet.rs2_value == 4,
                "DIV reads the actual ADDI-produced source values from the PRF");
        end
        20: begin
          add_issue_cycle = cycle_number;
          check(packet.rs1_value == 7 && packet.rs2_value == 9,
                "independent ADD reads real seeded register values");
        end
        24: begin
          consumer_issue_cycle = cycle_number;
          check(div_complete_cycle == cycle_number,
                "dependent ADD issues on the accepted DIV completion/bypass edge");
          check(packet.rs1_value == 25 && packet.rs2_value == 16,
                "dependent ADD receives both physical producer results");
        end
        28: check(packet.rs1_value == 41 && packet.uop.decoded.immediate == 1,
                  "final ADDI consumes the real dependent ADD result");
        default: begin end
      endcase
    end
  endtask

  task automatic observe_completion(input completion_t packet);
    integer index;
    logic found;
    renamed_uop_t owner;
    begin
      found = 0;
      completion_count = completion_count + 1;
      for (index = retired_count; index < dispatched_count; index = index + 1) begin
        owner = expected_owner[index];
        if (packet.rob_tag == owner.rob_tag) begin
          found = 1;
          check(packet.pdst == owner.pdst && packet.value == expected_value[index],
                "actual execution result matches its allocated program instruction");
          check(!packet.exception.valid && packet.control_flow.next_pc == index * 4 + 4,
                "actual completion has a sequential next PC and no spurious exception");
          if (index == 4) div_complete_cycle = cycle_number;
          if (index == 5) begin
            add_complete_cycle = cycle_number;
            check(div_complete_cycle < 0,
                  "independent ADD completes before the older DIV finishes");
          end
        end
      end
      check(found, "accepted completion belongs to a still-live RAM-program owner");
    end
  endtask

  task automatic observe_retirement(input retire_trace_t event_record);
    begin
      check(retired_count < 8, "retirement count stays within the program budget");
      if (retired_count < 8)
        check(event_record.pc == retired_count * 4
              && event_record.instruction == program_words[retired_count]
              && event_record.rd == retired_count + 1
              && event_record.rd_value == expected_value[retired_count],
              "RAM instruction retires in program order with the expected architectural value");
      check(!event_record.trap && !event_record.interrupt && !event_record.memory_write,
            "arithmetic program retires without trap or external side effects");
      $display("BLOCK09 RETIRE: cycle=%0d pc=%0d rd=x%0d value=%0d", cycle_number,
               event_record.pc, event_record.rd, event_record.rd_value);
      retired_count = retired_count + 1;
    end
  endtask

  task automatic edge_step;
    retire_trace_pair_t expected_trace;
    integer accepted_this_cycle, base_count, order_base;
    begin
      #1;
      check(rename_fire == rob_fire && rename_fire == iq_fire && rename_fire == fetched_fire,
            "fetch consumption agrees with atomic rename, ROB, and IQ admission");
      check(!rename_error && !rob_error && !iq_error && !trace_error && !write_error
            && !cluster_error && !writeback_error && !fetch_error && !adapter_error,
            "all connected production modules see legal protocol traffic");
      check(wb_rejected == 0 && trap_fire == 0 && !memory_valid && !mispredict
            && !fetch_stale && !fetch_fault,
            "straight-line executable RAM produces no fault, rejection, or redirect");
      check(fetch_occupancy <= FETCH_DEPTH, "fetch buffering never exceeds four instructions");
      check(broadcast_valid == wb_accepted && write_enable == wb_accepted,
            "every accepted program register result reaches both PRF and dependency wakeup");
      if (request_was_stalled)
        check(request_valid && request_address == stalled_address,
              "instruction request remains stable while the native port blocks acceptance");
      request_was_stalled = request_valid && !request_ready;
      stalled_address = request_address;
      if (response_valid && response_ready) begin
        check(memory_inflight == 1 && !response_error,
              "native response belongs to the one accepted outstanding request");
        check(response_data == rows[pending_address[15:3]],
              "synchronous native BRAM returns the accepted aligned row contents");
        memory_inflight = memory_inflight - 1;
        responses_consumed = responses_consumed + 1;
      end
      if (request_valid && request_ready) begin
        check(bram_enable && request_address[2:0] == 0
              && bram_address == request_address[15:3],
              "native read enable/address correspond to the accepted eight-byte row");
        memory_inflight = memory_inflight + 1;
        pending_address = request_address;
        reads_requested = reads_requested + 1;
      end
      check(memory_inflight >= 0 && memory_inflight <= 1,
            "instruction port keeps at most one accepted request outstanding");
      accepted_this_cycle = 0;
      base_count = dispatched_count;
      if (rename_fire[0]) begin
        observe_dispatch(renamed.lane0, base_count);
        accepted_this_cycle = accepted_this_cycle + 1;
      end
      if (rename_fire[1]) begin
        observe_dispatch(renamed.lane1, base_count + 1);
        accepted_this_cycle = accepted_this_cycle + 1;
      end
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
      // This counter controls the program admission budget, so change it only after capture.
      dispatched_count = dispatched_count + accepted_this_cycle;
      check(trace == expected_trace,
            "registered retirement trace preserves the full ordered events");
      clk = 0; cycle_number = cycle_number + 1; #1;
    end
  endtask

  initial begin #100000; $fatal(1, "Block 9 integration timed out"); end
  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block09/block09_integration.vcd");
      $dumpvars(0, tb_block09_integration);
    end
    program_words[0] = 32'h06400093; // addi x1,x0,100
    program_words[1] = 32'h00400113; // addi x2,x0,4
    program_words[2] = 32'h00700193; // addi x3,x0,7
    program_words[3] = 32'h00900213; // addi x4,x0,9
    program_words[4] = 32'h0220c2b3; // div  x5,x1,x2
    program_words[5] = 32'h00418333; // add  x6,x3,x4
    program_words[6] = 32'h006283b3; // add  x7,x5,x6
    program_words[7] = 32'h00138413; // addi x8,x7,1
    expected_value[0] = 100; expected_value[1] = 4;
    expected_value[2] = 7; expected_value[3] = 9;
    expected_value[4] = 25; expected_value[5] = 16;
    expected_value[6] = 41; expected_value[7] = 42;
    for (row = 0; row < RAM_FETCH_ROWS; row = row + 1)
      rows[row] = {NOP_INSTRUCTION, NOP_INSTRUCTION};
    for (row = 0; row < 4; row = row + 1)
      rows[row] = {program_words[row*2+1], program_words[row*2]};
    bram_data = 0; probe_bram_data = 0;
    fetch_run = 0; admission_enable = 0; single_dispatch = 0; bram_block = 0;
    head_ready = 3; wb_allow = 1; issue_enable = 1; admission_hold = 0;
    apply = 0; cancel = 0; synthetic_valid = 0;
    synthetic_memory = '0; synthetic_system = '0; committed_arch = 8;
    probe_request_valid = 0; probe_response_ready = 0; probe_address = 0; probe_block = 0;
    #1; rst_n = 0; probe_rst_n = 0; #2;
    check(!probe_request_ready && !probe_response_valid && !probe_enable,
          "adapter asynchronous reset masks request, response, and native read enable");
    probe_rst_n = 1; #1;

    // Native read result appears after its enabled edge and is held while the consumer stalls.
    probe_address = 8; probe_request_valid = 1; #1;
    check(probe_request_ready && probe_enable && probe_bram_address == 1,
          "valid byte address selects its native 64-bit row");
    raw_edge; probe_request_valid = 0; #1;
    check(probe_response_valid && !probe_response_error && probe_pending
          && probe_response_data == probe_pattern(1),
          "one synchronous native read becomes one matching successful response");
    saved_probe_data = probe_response_data;
    probe_address = 24; probe_request_valid = 1;
    for (row = 0; row < 3; row = row + 1) begin
      #1; check(!probe_request_ready && !probe_enable
                && probe_response_data == saved_probe_data && probe_response_valid,
                "unconsumed response blocks a new read and holds all returned data");
      raw_edge;
    end
    probe_block = 1; probe_response_ready = 1; #1;
    check(!probe_request_ready && !probe_enable && probe_response_valid,
          "native collision block prevents a new read but not an old response pop");
    raw_edge;
    check(!probe_response_valid && !probe_pending && !probe_request_ready,
          "old response drains while bram_block remains asserted");
    probe_block = 0; probe_response_ready = 0; #1;
    check(probe_request_ready && probe_enable && probe_bram_address == 3,
          "blocked request becomes eligible once the native port is released");
    raw_edge;
    check(probe_response_valid && probe_response_data == probe_pattern(3),
          "released native read retains the correct row identity");
    probe_address = 40; probe_response_ready = 1; #1;
    check(probe_request_ready && probe_enable,
          "response pop and next native read can share an edge");
    raw_edge;
    check(probe_response_valid && probe_response_data == probe_pattern(5),
          "same-edge refill publishes the new row without duplicating the old response");
    probe_request_valid = 0; raw_edge;
    check(!probe_response_valid && !probe_pending, "last native response is consumed once");

    probe_address = 32'h00010000; probe_request_valid = 1; probe_response_ready = 0; #1;
    check(probe_request_ready && !probe_enable && !probe_error,
          "out-of-RAM fetch accepts an access error without enabling native memory");
    raw_edge; probe_request_valid = 0; #1;
    check(probe_response_valid && probe_response_error && probe_response_data == 0,
          "out-of-RAM response reports error with no stale RAM data");
    raw_edge;
    check(probe_response_valid && probe_response_error && probe_response_data == 0,
          "error response also remains stable under backpressure");
    probe_response_ready = 1; raw_edge; probe_response_ready = 0;
    probe_address = 4; probe_request_valid = 1; #1;
    check(probe_request_ready && !probe_enable && probe_error,
          "misaligned adapter request is diagnosed and never reaches BRAM");
    raw_edge; probe_request_valid = 0; #1;
    check(probe_response_valid && probe_response_error && probe_response_data == 0,
          "defensive unaligned request produces a retained error response");
    probe_response_ready = 1; raw_edge;
    probe_rst_n = 0; probe_response_ready = 0; #1;

    // The executable program now traverses every implemented section from IF to retirement.
    rst_n = 1; fetch_run = 1; #1;
    check(empty && iq_occupancy == 0 && free_count == 32,
          "reset initializes the backend before instruction fetching starts");
    edge_step;
    bram_block = 1; edge_step;
    check(fetched_valid == 3 && fetched.lane0.pc == 0 && fetched.lane1.pc == 4
          && fetched.lane0.instruction == program_words[0]
          && fetched.lane1.instruction == program_words[1],
          "one native 64-bit read supplies two distinct 32-bit instructions and PCs");
    check(!adapter_pending && fetch_transaction && request_valid && !request_ready
          && request_address == 8 && fetch_occupancy == 2,
          "old response drains while the next offered row waits on native blocking");
    held_fetch = fetched;
    for (row = 0; row < 3; row = row + 1) begin
      edge_step;
      check(fetched == held_fetch && fetch_occupancy == 2,
            "stalled decode retains the original instruction pair while a request waits");
    end
    bram_block = 0; edge_step; edge_step;
    check(fetch_occupancy == 4 && fetched == held_fetch,
          "the four-entry fetch buffer fills without changing its blocked oldest pair");
    edge_step;
    check(!request_valid && fetch_occupancy == 4,
          "full fetch buffering stops fresh memory requests");
    admission_enable = 1; single_dispatch = 1; #1;
    check(rename_fire == 1 && fetched_fire == 1,
          "decode can accept just the older instruction of a fetched pair");
    edge_step; single_dispatch = 0; #1;
    check(fetched.lane0.pc == 4 && fetched.lane1.pc == 8
          && fetched.lane0.instruction == program_words[1]
          && fetched.lane1.instruction == program_words[2],
          "unaccepted suffix compacts and pairs with an instruction from the next RAM row");
    watchdog = 0;
    while ((retired_count < 8 || !empty || iq_occupancy != 0 || !cluster_quiescent
            || !writeback_quiescent) && watchdog < 160) begin
      edge_step; watchdog = watchdog + 1;
    end
    check(watchdog < 160 && dispatched_count == 8 && retired_count == 8 && completion_count == 8,
          "the complete eight-instruction RAM program executes and retires exactly once");
    final_owner = expected_owner[7];
    check(empty && iq_occupancy == 0 && cluster_quiescent && writeback_quiescent
          && free_count == 32 && committed_phys == final_owner.pdst,
          "drained backend preserves final committed mapping and returns old physical identities");
    check(add_issue_cycle >= 0 && div_issue_cycle >= 0
          && add_complete_cycle < div_complete_cycle
          && consumer_issue_cycle == div_complete_cycle,
          "RAM-fed program demonstrates out-of-order completion and accepted-result RAW bypass");
    check(reads_requested >= 4 && responses_consumed == reads_requested && memory_inflight == 0,
          "native instruction requests and responses finish without loss or duplication");
    check(!fetch_run || dispatched_count == 8,
          "admission budget stops fresh fetch requests after the complete program enters");
    $display("BLOCK09 PROGRAM: DIV issue=%0d, ADD issue=%0d, ADD complete=%0d,",
             div_issue_cycle, add_issue_cycle, add_complete_cycle);
    $display("                 DIV complete=%0d, dependent issue=%0d; x8=42",
             div_complete_cycle, consumer_issue_cycle);
    $display("BLOCK09_INTEGRATION SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Block 9 fetch-to-retirement integration failed");
    $finish;
  end
endmodule
