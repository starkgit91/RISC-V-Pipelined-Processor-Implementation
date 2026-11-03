// Simulation-only ACT4 execution boundary. ACT4 itself supplies the self-checks.
// A PASS is recognized only after a tohost store responds and retires.
`timescale 1ns / 1ps
module tb_act4;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1;
  logic i_valid, i_ready, i_rsp_valid, i_rsp_ready, i_error;
  word_t i_address;
  fetch_row_t i_data, bram_data = 0;
  logic bram_enable, bram_block, fetch_pending, adapter_error;
  logic [BRAM_FETCH_ADDR_W-1:0] bram_address;
  logic d_valid, d_ready, d_rsp_valid, d_rsp_ready;
  data_request_t d_request, saved_request;
  word_t write_value;
  logic [3:0] write_lanes;
  memory_response_t d_response, saved_response;
  logic d_pending = 0;
  integer d_delay = 0, data_latency = 2;
  retire_trace_pair_t trace;
  logic empty, core_error, prediction = 1;
  word_t memory [0:RAM_WORDS-1];
  fetch_row_t initial_rows [0:RAM_FETCH_ROWS-1];
  word_t tohost = 32'h1000_fff0, mailbox_value = 0;
  logic mailbox_requested = 0, mailbox_responded = 0;
  integer max_cycles = 1000000, cycles = 0, retirements = 0, traps = 0;
  integer stores_completed = 0, stores_retired = 0;
  integer n, byte_lane, image_file, selftest_code = 1, prediction_arg = 1;
  string image_path, vcd_path;

  rv32_ooo_core dut (
    .clk_i(clk), .rst_ni(rst_n), .predict_enable_i(prediction), .halt_i(1'b0),
    .instruction_request_valid_o(i_valid), .instruction_request_ready_i(i_ready),
    .instruction_request_address_o(i_address), .instruction_response_valid_i(i_rsp_valid),
    .instruction_response_ready_o(i_rsp_ready), .instruction_response_data_i(i_data),
    .instruction_response_error_i(i_error), .data_request_valid_o(d_valid),
    .data_request_ready_i(d_ready), .data_request_o(d_request),
    .data_response_valid_i(d_rsp_valid), .data_response_ready_o(d_rsp_ready),
    .data_response_i(d_response), .irq_external_i(1'b0), .irq_software_i(1'b0),
    .irq_timer_i(1'b0), .trace_o(trace), .empty_o(empty), .protocol_error_o(core_error)
  );

  rv32_instruction_adapter instruction_adapter (
    .clk_i(clk), .rst_ni(rst_n), .request_valid_i(i_valid), .request_ready_o(i_ready),
    .request_address_i(i_address), .response_valid_o(i_rsp_valid),
    .response_ready_i(i_rsp_ready), .response_data_o(i_data), .response_error_o(i_error),
    .bram_block_i(bram_block), .bram_enable_o(bram_enable),
    .bram_address_o(bram_address), .bram_data_i(bram_data),
    .transaction_pending_o(fetch_pending), .protocol_error_o(adapter_error)
  );

  // A zero mailbox value is ignored after retirement; hold the next request
  // until that event has consumed the current mailbox ownership record.
  assign d_ready = rst_n && !d_pending && !mailbox_responded;
  assign d_rsp_valid = rst_n && d_pending && d_delay == 0;
  assign d_response = saved_response;
  assign write_value = d_request.wdata;
  assign write_lanes = d_request.wstrb;
  // Prevent a simultaneous native fetch and data write to the same RAM row.
  // Older fetch responses are still consumed while a new read is blocked.
  assign bram_block = d_valid && d_ready && d_request.write
    && d_request.address < RAM_BYTES && d_request.address[31:3] == bram_address;

  always #10 clk = !clk;

  function automatic word_t op_i(input integer immediate, rs1, rd);
    op_i = ((immediate & 4095) << 20) | (rs1 << 15) | (rd << 7) | 7'h13;
  endfunction

  initial begin
    if ($value$plusargs("tohost=%h", tohost)) begin end
    if ($value$plusargs("max_cycles=%d", max_cycles)) begin end
    if ($value$plusargs("data_latency=%d", data_latency)) begin end
    if ($value$plusargs("predict=%d", prediction_arg)) begin end
    prediction = prediction_arg != 0;
    if (tohost[1:0] != 0 || tohost < RAM_BYTES)
      $fatal(1, "ACT4_HARNESS CONFIG tohost must be word-aligned outside RAM");
    if (max_cycles <= 0 || data_latency < 0)
      $fatal(1, "ACT4_HARNESS CONFIG invalid cycle limit or data latency");
    for (n = 0; n < RAM_WORDS; n++) memory[n] = 0;
    for (n = 0; n < RAM_FETCH_ROWS; n++) initial_rows[n] = 0;
    if ($test$plusargs("selftest")) begin
      if ($value$plusargs("selftest_code=%d", selftest_code)) begin end
      // LUI/ADDI form the configured mailbox address, including a negative low 12 bits.
      memory[0] = ((tohost + 32'h800) & 32'hfffff000) | 32'h000000b7;
      memory[1] = op_i(tohost[11:0], 1, 1);
      memory[2] = op_i(selftest_code, 0, 2);
      memory[3] = 32'h0020a023; // sw x2,0(x1)
      memory[4] = 32'h0000006f; // jal x0,0
    end else begin
      if (!$value$plusargs("imem=%s", image_path))
        $fatal(1, "ACT4_HARNESS CONFIG provide +imem=<64-bit-row hex> or +selftest");
      image_file = $fopen(image_path, "r");
      if (image_file == 0) $fatal(1, "ACT4_HARNESS CONFIG cannot open %s", image_path);
      $fclose(image_file);
      $readmemh(image_path, initial_rows);
      for (n = 0; n < RAM_FETCH_ROWS; n++) begin
        memory[2*n] = initial_rows[n][31:0];
        memory[2*n+1] = initial_rows[n][63:32];
      end
    end
    if ($value$plusargs("vcd=%s", vcd_path)) begin
      $dumpfile(vcd_path);
      $dumpvars(0, tb_act4);
    end
    #1 rst_n = 0;
    repeat (4) @(posedge clk);
    @(negedge clk) rst_n = 1;
  end

  // One shared array models the dual-port RAM. This matters for FENCE.I tests:
  // later instruction reads see data-port stores, rather than an old image copy.
  always @(posedge clk) begin
    if (!rst_n) begin
      bram_data <= 0;
      d_pending <= 0;
      d_delay <= 0;
      saved_request <= '0;
      saved_response <= '0;
      mailbox_requested <= 0;
      mailbox_responded <= 0;
      mailbox_value <= 0;
      stores_completed <= 0;
    end else begin
      if (bram_enable)
        bram_data <= {memory[2*bram_address+1], memory[2*bram_address]};
      if (d_pending && d_delay > 0) d_delay <= d_delay - 1;
      if (d_rsp_valid && d_rsp_ready) begin
        d_pending <= 0;
        if (saved_request.write && !saved_response.error) begin
          stores_completed <= stores_completed + 1;
          if (saved_request.address == tohost) mailbox_responded <= 1;
        end
      end
      if (d_valid && d_ready) begin
        d_pending <= 1;
        d_delay <= data_latency;
        saved_request <= d_request;
        saved_response <= '0;
        if (d_request.address[1:0] != 0) begin
          saved_response.error <= 1;
        end else if (d_request.address < RAM_BYTES) begin
          if (d_request.write) begin
            for (byte_lane = 0; byte_lane < 4; byte_lane++)
              if (write_lanes[byte_lane])
                memory[d_request.address >> 2][8*byte_lane +: 8]
                  <= write_value[8*byte_lane +: 8];
          end else saved_response.rdata <= memory[d_request.address >> 2];
        end else if (d_request.address == tohost) begin
          if (d_request.write) begin
            if (d_request.wstrb != 4'b1111 || mailbox_requested)
              $fatal(1, "ACT4_HARNESS PROTOCOL mailbox requires one complete word store");
            mailbox_requested <= 1;
            mailbox_responded <= 0;
            mailbox_value <= d_request.wdata;
          end else saved_response.rdata <= 0;
        end else saved_response.error <= 1;
      end
    end
  end

  task automatic observe(input retire_trace_t event_record);
    begin
      if (event_record.valid) begin
        if (event_record.trap || event_record.interrupt) traps++;
        else retirements++;
        if ($test$plusargs("trace"))
          $display("ACT4_TRACE cycle=%0d pc=%08h instruction=%08h trap=%0d cause=%08h",
                   cycles, event_record.pc, event_record.instruction,
                   event_record.trap, event_record.cause);
        if (event_record.memory_write) begin
          stores_retired++;
          if (event_record.memory_address == tohost) begin
            if (!mailbox_requested || !mailbox_responded
                || event_record.memory_wdata !== mailbox_value
                || event_record.memory_wstrb != 4'b1111 || event_record.trap)
              $fatal(1, "ACT4_HARNESS PROTOCOL termination was not completed and retired");
            mailbox_requested <= 0;
            mailbox_responded <= 0;
            if (mailbox_value == 1) begin
              $display("ACT4_HARNESS PASS cycles=%0d retired=%0d traps=%0d stores=%0d/%0d",
                       cycles, retirements, traps, stores_completed, stores_retired);
              $finish;
            end else if (mailbox_value != 0)
              $fatal(1, "ACT4_HARNESS FAIL code=%08h cycles=%0d retired=%0d",
                     mailbox_value, cycles, retirements);
          end
        end
      end
    end
  endtask

  // Trace is registered by the core, so these events describe architectural
  // retirement from the preceding edge. Never terminate on an execution result.
  always @(posedge clk) begin
    if (rst_n) begin
      cycles++;
      if (core_error || adapter_error)
        $fatal(1, "ACT4_HARNESS PROTOCOL core=%0d adapter=%0d", core_error, adapter_error);
      observe(trace.lane0);
      observe(trace.lane1);
      if (cycles >= max_cycles)
        $fatal(1, "ACT4_HARNESS TIMEOUT cycles=%0d retired=%0d traps=%0d",
               cycles, retirements, traps);
    end
  end
endmodule
