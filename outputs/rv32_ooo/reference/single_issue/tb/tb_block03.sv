// Block 3 checks stored physical values and same-cycle completion bypass.
// Readiness, allocation, ROB validation, and retirement belong to later blocks.
`timescale 1ns / 1ps

module tb_block03;
  import rv32_ooo_pkg::*;

  logic clk = 0;
  logic rst_n = 1;
  phys_reg_t read_rs1;
  phys_reg_t read_rs2;
  word_t read_data1;
  word_t read_data2;
  logic write_enable;
  phys_reg_t write_addr;
  word_t write_data;
  word_t model [0:PHYS_REG_COUNT-1];
  word_t rng = 32'h31415927;
  word_t saved_value;
  integer checks = 0;
  integer failures = 0;
  integer i;
  integer j;
  integer n;
  integer bit_number;
  integer model_index;

  always #10 clk = ~clk; // 50 MHz example clock; reads remain combinational.

  rv32_physical_regfile dut (
    .clk_i(clk),
    .rst_ni(rst_n),
    .read_rs1_i(read_rs1),
    .read_rs2_i(read_rs2),
    .read_data1_o(read_data1),
    .read_data2_o(read_data2),
    .write_enable_i(write_enable),
    .write_addr_i(write_addr),
    .write_data_i(write_data)
  );

  // Independent storage reference: reset or one accepted edge write.
  // Checks run after settling, so this blocking reference update cannot race the DUT.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (model_index = 0; model_index < PHYS_REG_COUNT; model_index = model_index + 1)
        model[model_index] = 0;
    end else if (write_enable && write_addr != 0) begin
      model[write_addr] = write_data;
    end
  end

  function automatic word_t expected_read(input phys_reg_t address);
    begin
      if (!rst_n || address == 0) expected_read = 0;
      else if (write_enable && write_addr != 0 && write_addr == address)
        expected_read = write_data;
      else expected_read = model[address];
    end
  endfunction

  function automatic word_t next_random(input word_t state);
    next_random = {state[30:0], state[31] ^ state[21] ^ state[1] ^ state[0]};
  endfunction

  task automatic check_word(input word_t got, expected, input string label_text);
    begin
      checks = checks + 1;
      if (got !== expected) begin
        failures = failures + 1;
        if (failures <= 24)
          $display("FAIL: %s got=%h expected=%h time=%0t", label_text, got, expected, $time);
      end
    end
  endtask

  task automatic check_reads(input string label_text);
    begin
      check_word(read_data1, expected_read(read_rs1), {label_text, " read port 1"});
      check_word(read_data2, expected_read(read_rs2), {label_text, " read port 2"});
    end
  endtask

  task automatic write_word(input phys_reg_t address, input word_t value);
    begin
      @(negedge clk);
      read_rs1 = address;
      read_rs2 = 0;
      write_addr = address;
      write_data = value;
      write_enable = 1;
      #1;
      check_reads("before write edge");
      @(posedge clk);
      #1;
      write_enable = 0;
      #1;
      check_reads("stored value after bypass disabled");
    end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block03/block03.vcd");
      $dumpvars(0, tb_block03);
    end
    #100000;
    $fatal(1, "Block 3 timed out");
  end

  initial begin
    read_rs1 = 0;
    read_rs2 = 63;
    write_enable = 0;
    write_addr = 0;
    write_data = 0;
    #1;
    rst_n = 0; // Assert away from a clock edge to test asynchronous reset.
    #1;
    check_reads("initial reset");
    @(negedge clk);
    rst_n = 1;
    for (i = 0; i < PHYS_REG_COUNT; i = i + 1) begin
      read_rs1 = i;
      read_rs2 = PHYS_REG_COUNT - 1 - i;
      #1;
      check_reads("every reset register");
    end

    // Give every writable address a distinctive value, including the highest tag.
    for (i = 1; i < PHYS_REG_COUNT; i = i + 1)
      write_word(i, 32'hc0010000 ^ (i * 32'h10204081));

    // Both read ports must select independently without needing a clock edge.
    for (i = 0; i < PHYS_REG_COUNT; i = i + 1) begin
      for (j = 0; j < PHYS_REG_COUNT; j = j + 1) begin
        read_rs1 = i;
        read_rs2 = j;
        #1;
        check_reads("all independent read-address pairs");
      end
    end

    // Lead lesson: p37's new value is visible to BOTH reads before it is stored.
    write_word(37, 32'd7);
    @(negedge clk);
    read_rs1 = 37;
    read_rs2 = 37;
    write_addr = 37;
    write_data = 32'd99;
    write_enable = 1;
    #1;
    check_word(model[37], 32'd7, "stored reference remains old before the edge");
    check_word(read_data1, 32'd99, "new completion bypasses to first reader");
    check_word(read_data2, 32'd99, "new completion bypasses to second reader");
    @(posedge clk);
    #1;
    write_enable = 0;
    #1;
    check_reads("completion persisted after the write edge");

    // A bypass value must not become storage if its write is withdrawn before the edge.
    @(negedge clk);
    saved_value = model[37];
    write_addr = 37;
    write_data = 32'hdead1234;
    write_enable = 1;
    #1;
    check_reads("temporary bypass before withdrawal");
    write_enable = 0;
    #1;
    check_word(read_data1, saved_value, "withdrawal restores stored value without an edge");
    check_word(read_data2, saved_value, "second port also restores the stored value");
    @(posedge clk);
    #1;
    check_reads("withdrawn write did not persist");

    // Disabled write data cannot override either read port.
    @(negedge clk);
    read_rs1 = 37;
    read_rs2 = 12;
    write_addr = 37;
    write_data = 32'h12345678;
    write_enable = 0;
    #1;
    check_reads("disabled completion does not bypass");

    write_word(0, 32'hffffffff);
    read_rs1 = 0;
    read_rs2 = 0;
    #1;
    check_reads("p0 ignores writes on both read ports");

    // Exercise all data bits at low/high writable addresses and preserve neighbours.
    for (bit_number = 0; bit_number < XLEN; bit_number = bit_number + 1) begin
      write_word(1, 32'h1 << bit_number);
      write_word(63, ~(32'h1 << bit_number));
      read_rs1 = 1;
      read_rs2 = 63;
      #1;
      check_reads("walking data bits at address endpoints");
    end

    // Random edge writes plus forced same-port/both-port/nonmatching bypass cases.
    for (n = 0; n < 1024; n = n + 1) begin
      @(negedge clk);
      rng = next_random(rng);
      write_addr = rng[PHYS_REG_W-1:0];
      rng = next_random(rng);
      write_data = rng;
      write_enable = n % 8 != 0;
      rng = next_random(rng);
      read_rs1 = rng[PHYS_REG_W-1:0];
      read_rs2 = rng[PHYS_REG_W*2-1:PHYS_REG_W];
      case (n % 4)
        0: read_rs1 = write_addr;
        1: read_rs2 = write_addr;
        2: begin read_rs1 = write_addr; read_rs2 = write_addr; end
        default: begin end
      endcase
      #1;
      check_reads("random pre-edge read/bypass");
      @(posedge clk);
      #1;
      write_enable = 0;
      #1;
      check_reads("random stored result after edge");
    end

    // Detect accidental changes to unrelated registers after the random writes.
    for (i = 0; i < PHYS_REG_COUNT; i = i + 1) begin
      for (j = 0; j < PHYS_REG_COUNT; j = j + 1) begin
        read_rs1 = i;
        read_rs2 = j;
        #1;
        check_reads("all stored register pairs after random writes");
      end
    end

    // Reset must beat a held write and its bypass, including before the next edge.
    @(negedge clk);
    read_rs1 = 37;
    read_rs2 = 63;
    write_enable = 1;
    write_addr = 37;
    write_data = 32'h76543210;
    #1;
    check_reads("write visible before reset assertion");
    #2;
    rst_n = 0;
    #1;
    check_reads("asynchronous reset overrides bypass immediately");
    @(posedge clk);
    #1;
    check_reads("reset overrides held write at the clock edge");
    write_enable = 0;
    @(negedge clk);
    rst_n = 1;
    for (i = 0; i < PHYS_REG_COUNT; i = i + 1) begin
      read_rs1 = i;
      read_rs2 = PHYS_REG_COUNT - 1 - i;
      #1;
      check_reads("all values cleared by mid-run reset");
    end

    $display("BLOCK03 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Block 3 physical register/bypass tests failed");
    $finish;
  end
endmodule
