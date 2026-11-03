// Native instruction adapter verification with a synchronous enable-held RAM model.
`timescale 1ns / 1ps

module tb_block09_adapter;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1;
  logic request_valid, request_ready, response_valid, response_ready, response_error;
  logic block_read, bram_enable, pending, protocol_error;
  word_t address;
  fetch_row_t response_data, bram_data = 0;
  logic [BRAM_FETCH_ADDR_W-1:0] bram_address;
  logic reference_valid = 0, reference_error = 0;
  fetch_row_t reference_data = 0;
  word_t rng = 32'h31415927;
  integer checks = 0, failures = 0, accepts = 0, responses = 0;
  logic expected_ready, fire, valid_address;

  rv32_instruction_adapter dut (
    .clk_i(clk), .rst_ni(rst_n), .request_valid_i(request_valid),
    .request_ready_o(request_ready), .request_address_i(address),
    .response_valid_o(response_valid), .response_ready_i(response_ready),
    .response_data_o(response_data), .response_error_o(response_error),
    .bram_block_i(block_read), .bram_enable_o(bram_enable), .bram_address_o(bram_address),
    .bram_data_i(bram_data), .transaction_pending_o(pending), .protocol_error_o(protocol_error)
  );

  function automatic fetch_row_t pattern(input integer row_index);
    word_t low_word, high_word;
    begin
      low_word = 32'h10000000 + row_index * 2;
      high_word = low_word + 1;
      pattern = {high_word, low_word};
    end
  endfunction

  // Only an enabled rising edge changes the native memory's output latch.
  always_ff @(posedge clk) begin
    if (bram_enable) bram_data <= pattern(bram_address);
  end

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        $display("FAIL: %s", label_text);
      end
    end
  endtask

  task automatic observe;
    begin
      expected_ready = rst_n && !block_read && (!reference_valid || response_ready);
      valid_address = address < 65536 && address % 8 == 0;
      fire = request_valid && expected_ready;
      check(request_ready == expected_ready, "request capacity matches response ownership");
      check(response_valid == (rst_n && reference_valid) && pending == reference_valid,
            "response valid and pending describe the retained owner");
      check(response_error == (rst_n && reference_valid && reference_error),
            "error flag stays with its matching response");
      check(response_data == ((rst_n && reference_valid && !reference_error)
                             ? reference_data : 64'b0), "matching payload remains stable");
      check(bram_enable == (fire && valid_address),
            "native reads require a valid accepted address");
      check(bram_address == ((address / 8) % 8192), "byte-to-row address is visible before enable");
      check(protocol_error == (fire && address % 8 != 0), "misaligned captures are diagnosed");
    end
  endtask

  task automatic step;
    begin
      #1; observe;
      if (reference_valid && response_ready) begin
        reference_valid = 0;
        responses = responses + 1;
      end
      if (fire) begin
        reference_valid = 1;
        reference_error = !valid_address;
        reference_data = pattern(address / 8);
        accepts = accepts + 1;
      end
      #1; clk = 1; #1; clk = 0; #1; observe;
    end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block09/block09_adapter.vcd");
      $dumpvars(0, tb_block09_adapter);
    end
    #100000; $fatal(1, "Native instruction adapter timed out");
  end

  initial begin
    request_valid = 0; response_ready = 0; block_read = 0; address = 0;
    #1; rst_n = 0; #2; observe; rst_n = 1;
    request_valid = 1; address = 0; step; request_valid = 0;
    repeat (3) step;

    // A collision hold prevents refill but never traps an already completed response.
    block_read = 1; response_ready = 1; request_valid = 1; address = 8; step;
    check(!response_valid && !pending && !request_ready,
          "blocked refill does not block old-response consumption");
    block_read = 0; step;
    for (integer i = 0; i < 40; i++) begin address = (i + 2) * 8; step; end
    request_valid = 0; step;

    address = 32'h0000fff8; request_valid = 1; step; // Last complete row.
    address = 32'h00010000; step; // First unmapped row.
    address = 32'h0000fffc; step; // Misaligned row request.
    address = 32'hffffffff; step;
    request_valid = 0; step;

    // Keep an unaccepted producer request stable; vary the two sides' stalls independently.
    for (integer i = 0; i < 2000; i++) begin
      rng = {rng[30:0], rng[31] ^ rng[21] ^ rng[1] ^ rng[0]};
      if (!(request_valid && !request_ready)) begin
        request_valid = rng[0];
        case (rng[4:2])
          0: address = rng & 32'h0000fff8;
          1: address = 32'h0000fff8;
          2: address = 32'h00010000;
          3: address = 32'hffffffff;
          4: address = 32'h00000004;
          default: address = rng & 32'h0000fff8;
        endcase
      end
      block_read = rng[5] && rng[6];
      response_ready = rng[7];
      step;
    end
    request_valid = 0; response_ready = 1; block_read = 0; step;
    request_valid = 1; address = 32; response_ready = 0; step;

    // One final occupied response is intentionally removed by global reset.
    rst_n = 0; reference_valid = 0; reference_error = 0; #2; observe;
    rst_n = 1; request_valid = 0; step;
    $display("BLOCK09_ADAPTER TRAFFIC: %0d accepts, %0d responses (one cleared by reset)",
             accepts, responses);
    $display("BLOCK09_ADAPTER SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Native instruction adapter failed");
    $finish;
  end
endmodule
