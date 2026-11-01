// Two-issue block 3: four reads, two writes, and completion bypass.
// The reference array tracks stored values only; readiness and retirement are separate blocks.
`timescale 1ns / 1ps

module tb_block03;
  import rv32_ooo_pkg::*;

  logic clk = 0;
  logic rst_n = 1;
  phys_reg_t read_addr [0:3];
  word_t read_data [0:3];
  logic [1:0] write_enable;
  phys_reg_t write_addr0;
  phys_reg_t write_addr1;
  word_t write_data0;
  word_t write_data1;
  logic write_conflict;
  word_t model [0:PHYS_REG_COUNT-1];
  word_t rng = 32'h31415927;
  word_t saved37;
  word_t saved38;
  integer checks = 0;
  integer failures = 0;
  integer i;
  integer j;
  integer n;
  integer bit_number;
  integer model_index;

  always #10 clk = ~clk; // Functional stimulus only; this does not prove FPGA timing.

  rv32_physical_regfile dut (
    .clk_i(clk), .rst_ni(rst_n),
    .read0_rs1_i(read_addr[0]), .read0_rs2_i(read_addr[1]),
    .read1_rs1_i(read_addr[2]), .read1_rs2_i(read_addr[3]),
    .read0_data1_o(read_data[0]), .read0_data2_o(read_data[1]),
    .read1_data1_o(read_data[2]), .read1_data2_o(read_data[3]),
    .write_enable_i(write_enable),
    .write0_addr_i(write_addr0), .write1_addr_i(write_addr1),
    .write0_data_i(write_data0), .write1_data_i(write_data1),
    .write_conflict_o(write_conflict)
  );

  // Apply lane 1 first, then lane 0: this independently models defined conflict priority.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (model_index = 0; model_index < PHYS_REG_COUNT; model_index = model_index + 1)
        model[model_index] = 0;
    end else begin
      if (write_enable[1] && write_addr1 != 0) model[write_addr1] = write_data1;
      if (write_enable[0] && write_addr0 != 0) model[write_addr0] = write_data0;
    end
  end

  function automatic word_t next_random(input word_t state);
    next_random = {state[30:0], state[31] ^ state[21] ^ state[1] ^ state[0]};
  endfunction

  function automatic word_t expected_read(input phys_reg_t address);
    begin
      if (!rst_n || address == 0) expected_read = 0;
      else if (write_enable[0] && write_addr0 == address) expected_read = write_data0;
      else if (write_enable[1] && write_addr1 == address) expected_read = write_data1;
      else expected_read = model[address];
    end
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 24) $display("FAIL: %s time=%0t", label_text, $time);
      end
    end
  endtask

  task automatic check_reads(input string label_text);
    integer port_index;
    begin
      for (port_index = 0; port_index < 4; port_index = port_index + 1)
        check(read_data[port_index] === expected_read(read_addr[port_index]), label_text);
      check(write_conflict === (rst_n && write_enable == 2'b11 && write_addr0 != 0 &&
            write_addr0 == write_addr1), "conflict flag matches enabled nonzero collision");
    end
  endtask

  task automatic write_pair(input logic [1:0] enables, input integer dest0, dest1,
                            input word_t value0, value1);
    begin
      @(negedge clk);
      write_enable = enables;
      write_addr0 = dest0;
      write_addr1 = dest1;
      write_data0 = value0;
      write_data1 = value1;
      read_addr[0] = dest0;
      read_addr[1] = dest1;
      read_addr[2] = dest1;
      read_addr[3] = dest0;
      #1;
      check_reads("both writers bypass to all matching read ports before the edge");
      @(posedge clk);
      #1;
      write_enable = 0;
      #1;
      check_reads("both accepted writes persist after bypass is disabled");
    end
  endtask

  task automatic sweep_reads;
    integer first;
    integer second;
    begin
      for (first = 0; first < PHYS_REG_COUNT; first = first + 1) begin
        for (second = 0; second < PHYS_REG_COUNT; second = second + 1) begin
          read_addr[0] = first;
          read_addr[1] = second;
          read_addr[2] = PHYS_REG_COUNT - 1 - first;
          read_addr[3] = PHYS_REG_COUNT - 1 - second;
          #1;
          check_reads("all independent read-address pairs on both instruction lanes");
        end
      end
    end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block03/block03.vcd");
      $dumpvars(0, tb_block03);
    end
    #250000;
    $fatal(1, "Two-issue block 3 timed out");
  end

  initial begin
    write_enable = 0;
    write_addr0 = 0;
    write_addr1 = 0;
    write_data0 = 0;
    write_data1 = 0;
    for (i = 0; i < 4; i = i + 1) read_addr[i] = i;
    #1; rst_n = 0; #1;
    check_reads("asynchronous startup reset clears all outputs");
    @(negedge clk); rst_n = 1;
    sweep_reads;

    // Initialize every writable location through two genuinely concurrent write ports.
    for (i = 1; i < PHYS_REG_COUNT; i = i + 2)
      write_pair(2'b11, i, (i + 1) % PHYS_REG_COUNT,
                 32'hc0010000 ^ (i * 32'h10204081), 32'h80000000 ^ (i * 32'h00010001));
    sweep_reads;

    // Familiar same-cycle example, now with two new values and four readers.
    write_pair(2'b11, 37, 38, 7, 8);
    @(negedge clk);
    write_enable = 2'b11;
    write_addr0 = 37; write_data0 = 99;
    write_addr1 = 38; write_data1 = 123;
    read_addr[0] = 37; read_addr[1] = 38;
    read_addr[2] = 38; read_addr[3] = 37;
    #1;
    check(model[37] == 7 && model[38] == 8, "stored values remain old before the edge");
    check(read_data[0] == 99 && read_data[3] == 99, "writer 0 bypasses to both lanes");
    check(read_data[1] == 123 && read_data[2] == 123, "writer 1 bypasses to both lanes");
    check_reads("two independent accepted completions before the same clock edge");
    @(posedge clk); #1; write_enable = 0; #1;
    check_reads("p37=99 and p38=123 persist together");

    // Neither bypassed value may persist when both writes are withdrawn before the edge.
    @(negedge clk);
    saved37 = model[37]; saved38 = model[38];
    write_enable = 2'b11;
    write_data0 = 32'hdead0001; write_data1 = 32'hdead0002;
    #1; check_reads("temporary two-writer bypass");
    write_enable = 0;
    #1;
    check(read_data[0] == saved37 && read_data[1] == saved38,
          "withdrawing writes restores old values without a clock");
    @(posedge clk); #1; check_reads("withdrawn writes did not persist");

    // Both equal destinations is an explicit protocol-error test, never legal live traffic.
    write_pair(2'b11, 37, 37, 32'h11111111, 32'h22222222);
    check(model[37] == 32'h11111111, "lane 0 wins a diagnosed collision in storage");
    write_pair(2'b01, 37, 37, 32'h33333333, 32'h44444444);
    write_pair(2'b10, 37, 37, 32'h55555555, 32'h66666666);
    write_pair(2'b00, 37, 37, 32'h77777777, 32'h88888888);
    write_pair(2'b11, 0, 0, 32'hffffffff, 32'hffffffff);
    write_pair(2'b11, 0, 63, 32'hffffffff, 32'h12345678);
    write_pair(2'b11, 63, 0, 32'h87654321, 32'hffffffff);

    // Every destination pair checks dual decoders, collision priority, and endpoint addresses.
    for (i = 0; i < PHYS_REG_COUNT; i = i + 1) begin
      for (j = 0; j < PHYS_REG_COUNT; j = j + 1)
        write_pair(2'b11, i, j, 32'h10000000 ^ (i << 8) ^ j,
                   32'h90000000 ^ (j << 8) ^ i);
    end
    for (bit_number = 0; bit_number < XLEN; bit_number = bit_number + 1)
      write_pair(2'b11, 1, 63, 32'h1 << bit_number, ~(32'h1 << bit_number));
    sweep_reads;

    // Random legal traffic has no duplicate enabled nonzero physical destination.
    for (n = 0; n < 1024; n = n + 1) begin
      @(negedge clk);
      rng = next_random(rng); write_addr0 = rng[PHYS_REG_W-1:0];
      rng = next_random(rng); write_addr1 = rng[PHYS_REG_W-1:0];
      if (write_addr0 != 0 && write_addr0 == write_addr1) write_addr1 = write_addr1 + 1'b1;
      rng = next_random(rng); write_enable = rng[1:0]; write_data0 = rng;
      rng = next_random(rng); write_data1 = rng;
      for (i = 0; i < 4; i = i + 1) begin
        rng = next_random(rng); read_addr[i] = rng[PHYS_REG_W-1:0];
      end
      if (n % 4 == 0) read_addr[0] = write_addr0;
      if (n % 4 == 1) read_addr[1] = write_addr1;
      if (n % 4 == 2) begin read_addr[2] = write_addr0; read_addr[3] = write_addr1; end
      #1;
      check(!write_conflict, "legal concurrent completions have distinct physical destinations");
      check_reads("random four-port pre-edge reads");
      @(posedge clk); #1; write_enable = 0; #1;
      check_reads("random stored values after dual-write edge");
    end
    sweep_reads;

    // Reset must override even a held colliding write and all four bypass paths.
    @(negedge clk);
    write_addr0 = 37; write_addr1 = 37;
    write_data0 = 32'hffffffff; write_data1 = 32'h12345678; write_enable = 2'b11;
    for (i = 0; i < 4; i = i + 1) read_addr[i] = 37;
    #1; check_reads("colliding writes are diagnosed before reset");
    #2; rst_n = 0; #1;
    check_reads("reset overrides held writes, bypass, and conflict indication");
    @(posedge clk); #1; check_reads("reset wins at the write edge");
    write_enable = 0;
    @(negedge clk); rst_n = 1;
    sweep_reads;

    $display("BLOCK03 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Two-issue physical register-file tests failed");
    $finish;
  end
endmodule
