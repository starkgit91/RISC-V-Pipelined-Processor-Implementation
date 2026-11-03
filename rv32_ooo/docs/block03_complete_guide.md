# Block 3, two-issue revision: four reads, two writes, and completion bypass

This is the active two-issue register file. The earlier two-read/one-write lesson and verified
source are preserved under `reference/single_issue`.

## 1. Start with the same-cycle example

Suppose p37 stores 7 and p38 stores 8. Two accepted completions supply p37=99 and p38=123.
Instruction lane 0 reads p37 and p38; instruction lane 1 can read the same pair or two other tags.
Before the rising edge, every matching read port selects its corresponding completion value.
At the edge, both distinct physical destinations update. After write enable falls, the stored
99 and 123 remain available.

This gives two two-source instructions enough operand bandwidth in the same cycle. It does
not make dependent instructions independent: the scheduler must still wait for producer readiness.

## 2. Ports and connections

| Ports | Width | Source/destination | Role |
|---|---:|---|---|
| clk_i / rst_ni | 1 each | Common clock/global reset | Edge writes and asynchronous clear |
| read0_rs1_i / read0_rs2_i | 6 each | First issued instruction | Physical source tags |
| read1_rs1_i / read1_rs2_i | 6 each | Second issued instruction | Physical source tags |
| read0_data1_o / read0_data2_o | 32 each | First execution operand path | Values, including completion bypass |
| read1_data1_o / read1_data2_o | 32 each | Second execution operand path | Values, including completion bypass |
| write_enable_i | 2 | Accepted completion arbiter outputs | One independent enable per result |
| write0_addr_i / write1_addr_i | 6 each | Accepted completions | Physical destinations |
| write0_data_i / write1_data_i | 32 each | Accepted completions | New values |
| write_conflict_o | 1 | Debug/protocol checking | Both enabled writes target the same nonzero physical tag |

The write lanes carry accepted, live, non-faulting results. A waiting execution result must
not assert a write until the arbiter and ROB identity checks accept it. There is no write-ready
port because the file can accept both distinct writes on each enabled edge.

## 3. Circuit view

```text
                    63 stored 32-bit words (p1..p63)
                           |  |  |  |
                    four address-selected reads
                           |  |  |  |
 write0 tag/value -------> match and bypass selection for each read
 write1 tag/value -------> match and bypass selection for each read
                           |  |  |  |
                    reset/p0 zero selection
                           |  |  |  |
                   lane0 operands   lane1 operands
```

p0 has no writable storage and always reads zero. The stored state remains 2,016 bits; wider
ports increase selection, decode, and bypass logic rather than requiring a second architectural
copy of every value. The exact FPGA cell counts and timing require Vivado reports.

Each stored word has two decoded write enables. Different destinations update simultaneously.
Four independent read selectors can request any combination of tags, including four reads of
the same register. Reset clears all stored values and overrides bypass on all read ports.

## 4. Write conflicts and priority

Legal live completions have different physical destinations even if their architectural
instructions both write x5. Renaming gives those instructions different physical versions.

If a caller violates this contract and sends two enabled writes to the same nonzero tag,
write_conflict_o asserts and lane 0 wins consistently for both storage and bypass. This is a
protocol diagnostic, not an ISA exception or a normal scheduling policy. The completion arbiter
must prevent such a conflict during legal operation. Two attempted writes to p0 are ignored.

Read priority is: reset zero, p0 zero, write-lane-0 match, write-lane-1 match, stored value.
This makes the result independent of simulator nonblocking-assignment ordering.

## 5. Out-of-order state remains separate

The register file stores speculative and committed physical values together. It does not own
readiness, rename allocation, ROB generations, or the committed map. Completion can update a
physical version before that instruction retires.

Branch recovery must preserve this array and restore mappings after safe cancellation; it
must not pulse the file's global reset. Readiness cannot be inferred from a value of zero.

The asynchronous reads and reset describe register-based storage. A synchronous block RAM is
not a drop-in replacement: it would change operand latency. The larger program/data RAM will
have a separate synchronous memory interface.

## 6. Tests and waveforms

From the project directory:

```sh
python3 scripts/run_checks.py --block 3 --waves
```

The clocked test compares four reads and two writes against an independent reference array.
It checks distinct simultaneous writes, all read ports, pre-edge bypass, persisted values,
withdrawn/disabled writes, reset, p0, conflicts, and randomized cycles. The report identifies
exact counts and source hashes; it is not a complete processor or timing test.

The optional waveform is `build/block03/block03.vcd`. Add both write enables/addresses/data,
the four read tags/values, clock, reset, and write_conflict. Look for bypass values before an
edge and persistence after write enable falls. In a legal two-result example, conflict stays low.

For Vivado XSim on Linux after sourcing your installation's settings file:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_physical_regfile.sv tb/tb_block03.sv
xelab tb_block03 -s block03_sim
xsim block03_sim --runall
```

Vivado execution and the 50 MHz target remain unverified until the actual Linux build runs.

## 7. Interview questions

**Why are four reads useful for two-issue?** Two issued integer instructions can each require
two independent source values in the same cycle.

**Why do two writes to x5 not collide here?** The instructions have different renamed physical
destinations. Architectural naming and physical storage addressing are different layers.

**Does two-issue guarantee two instructions execute every cycle?** No. Dependencies, resource
availability, prediction, and memory latency determine which instructions can issue.

**Why bypass both write lanes to every read port?** Either source of either instruction may
need a newly accepted completion, so the match cannot be limited to one fixed pairing.

**Does a write mean architectural commitment?** No. Only in-order retirement updates the
committed register map.

The module and testbench source below are the files actually compiled. Historical single-issue
results remain in the reference checkpoint and are not evidence for this revised interface.

## Complete physical register file

Source: `rtl/rv32_physical_regfile.sv`.

```systemverilog
// Two-issue register read / completion boundary: four reads and two accepted writes.
// Readiness, live ROB ownership, and committed mappings are checked in other blocks.
`timescale 1ns / 1ps

module rv32_physical_regfile (
  input  logic                     clk_i,
  input  logic                     rst_ni,
  input  rv32_ooo_pkg::phys_reg_t   read0_rs1_i,
  input  rv32_ooo_pkg::phys_reg_t   read0_rs2_i,
  input  rv32_ooo_pkg::phys_reg_t   read1_rs1_i,
  input  rv32_ooo_pkg::phys_reg_t   read1_rs2_i,
  output rv32_ooo_pkg::word_t       read0_data1_o,
  output rv32_ooo_pkg::word_t       read0_data2_o,
  output rv32_ooo_pkg::word_t       read1_data1_o,
  output rv32_ooo_pkg::word_t       read1_data2_o,
  input  logic [1:0]               write_enable_i,
  input  rv32_ooo_pkg::phys_reg_t   write0_addr_i,
  input  rv32_ooo_pkg::phys_reg_t   write1_addr_i,
  input  rv32_ooo_pkg::word_t       write0_data_i,
  input  rv32_ooo_pkg::word_t       write1_data_i,
  output logic                     write_conflict_o
);
  import rv32_ooo_pkg::*;

  // p0 is constant. Each stored word has two decoded write enables and four readers.
  word_t values_q [1:PHYS_REG_COUNT-1];
  integer reg_index;

  assign write_conflict_o = rst_ni && (&write_enable_i) && (write0_addr_i != 0) &&
                            (write0_addr_i == write1_addr_i);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (reg_index = 1; reg_index < PHYS_REG_COUNT; reg_index = reg_index + 1)
        values_q[reg_index] <= '0;
    end else begin
      for (reg_index = 1; reg_index < PHYS_REG_COUNT; reg_index = reg_index + 1) begin
        if (write_enable_i[0] && write0_addr_i == reg_index)
          values_q[reg_index] <= write0_data_i;
        else if (write_enable_i[1] && write1_addr_i == reg_index)
          values_q[reg_index] <= write1_data_i;
      end
    end
  end

  // Legal live completions have different destinations. Diagnose a duplicate and let lane 0 win.
  function automatic word_t read_value(input phys_reg_t address);
    begin
      if (!rst_ni || address == 0) read_value = '0;
      else if (write_enable_i[0] && write0_addr_i == address) read_value = write0_data_i;
      else if (write_enable_i[1] && write1_addr_i == address) read_value = write1_data_i;
      else read_value = values_q[address];
    end
  endfunction

  always_comb begin
    read0_data1_o = read_value(read0_rs1_i);
    read0_data2_o = read_value(read0_rs2_i);
    read1_data1_o = read_value(read1_rs1_i);
    read1_data2_o = read_value(read1_rs2_i);
  end
endmodule
```

## Complete self-checking testbench

Source: `tb/tb_block03.sv`.

```systemverilog
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
```
