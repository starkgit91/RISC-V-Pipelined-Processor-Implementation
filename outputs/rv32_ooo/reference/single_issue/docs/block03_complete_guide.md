# Block 3: physical register storage and same-cycle completion bypass

This block implements the physical register file used between register read in EX and
completion/writeback in WB. It contains two combinational read ports and one clocked write port.
There are 64 physical identities, p0 through p63. p0 is a constant zero; p1 through p63 hold values.

The code and clocked testbench for this block are complete. This is not a complete processor:
rename maps, readiness, scheduling, ROB ownership checks, and architectural retirement come later.

## 1. Start with the same-cycle example

Suppose p37 already stores 7. An execution unit finishes a new value, 99, that the completion
arbiter accepts for p37. At the same time, register read requests p37 as an operand.

Without a bypass, a consumer sampling on the same edge as the write could capture the old 7.
The bypass selects the accepted completion value before the clocked array update occurs.

| Event | Write enable | Write address/data | Read address | Read result |
|---|---:|---|---|---:|
| Before the new completion | 0 | Irrelevant | p37 | 7 from storage |
| Completion accepted, before the rising edge | 1 | p37 / 99 | p37 | 99 through bypass |
| Rising edge | 1 | p37 / 99 | p37 | 99; storage also updates |
| Write enable removed after the edge | 0 | Irrelevant | p37 | 99 from storage |

Both read ports can match p37 and see 99 simultaneously. If only one port matches, only that
port bypasses; the other still reads its own stored value. p0 always returns zero, even if an
incorrect request attempts to write it.

This is forwarding at a concrete circuit boundary. It does not make an unfinished result ready:
the later scheduler must still wait for the producer or an accepted completion/wakeup event.

## 2. Connections to the five logical sections

```text
ID: future rename maps architectural source names to physical tags
                         |
EX: future scheduler selects a ready instruction
                         |
                   read_rs1_i / read_rs2_i
                         |
                         v
              rv32_physical_regfile ------> read_data1_o / read_data2_o
                         ^                              |
                         |                              v
                  accepted completion             EX operand selection
             write_enable_i/address/data               |
                         ^                              v
                         +------ WB <------------ execution result

The future committed map and ROB define architectural state separately.
```

The input tags are physical names, not x-register names. The values returned are raw 32-bit
data. The ALU from block 2 will interpret them according to the decoded operation.

There is no ROB tag input on this register file. A later completion arbiter checks ownership
and faults before asserting the write port. That decision must happen before this module sees
an enabled write; the register file cannot determine whether a completion belongs to a live instruction.

## 3. Port-by-port explanation

| Port | Width | Producer/consumer | Meaning |
|---|---:|---|---|
| `clk_i` | 1 | Common processor clock | Storage updates on its rising edge |
| `rst_ni` | 1 | Global reset control | Active-low asynchronous reset; reset release is synchronized by the future wrapper |
| `read_rs1_i` | 6 | Scheduler/register-read controls | First physical source identity |
| `read_rs2_i` | 6 | Scheduler/register-read controls | Second physical source identity |
| `read_data1_o` | 32 | Execution operand-selection path | First stored or bypassed value |
| `read_data2_o` | 32 | Execution operand-selection path | Second stored or bypassed value |
| `write_enable_i` | 1 | Completion arbiter | One accepted, live, non-faulting register-result write |
| `write_addr_i` | 6 | Accepted completion | Physical destination identity |
| `write_data_i` | 32 | Accepted completion | Entire new register value |

The write port has no ready signal: it can accept one enabled write on every rising edge when
reset is released. The upstream arbiter handles competition between execution units. It must not
assert write_enable merely because a result is waiting; it asserts it for the result selected
and accepted on the completion bus.

Read ports also have no ready signal. They continuously describe their selected values. The
scheduler decides when those outputs are valid operands for a live issued instruction.

## 4. Storage, reset, and zero

The source declares storage only for p1 through p63. p0 needs no writable element because its
output is hardwired to zero. Nominal stored data is therefore 63 x 32 = 2,016 bits.

`always_ff @(posedge clk_i or negedge rst_ni)` describes edge-triggered storage with an
asynchronous clear. The reset loop initializes all stored values to zero for deterministic
simulation and startup. On an ordinary rising edge, only the enabled nonzero destination changes.
Nonblocking assignments express the state update at that edge.

The outputs are also explicitly zero while reset is low. This prevents a held completion input
from bypassing a nonzero value during reset. Reset wins over an enabled write.

**Do not use this reset to flush a branch misprediction.** The physical register file contains
both speculative values and committed values. Recovery must restore mappings, cancel younger
work, and reclaim physical identities while preserving committed values. Global reset and branch
recovery are different operations.

Zero data does not imply readiness. A newly allocated p37 may contain an old or reset value;
the readiness table must keep dependent instructions waiting until its producer completes.

## 5. Read circuits and priority

Each port implements this conceptual selection:

```text
all stored p1..p63 values --> address-selected stored value ----+
                                                               |
write_data_i ------------------------------------------------> bypass mux
                                                               |
                                         reset / p0 check ---> zero-or-value mux
                                                               |
                                                               v
                                                          read_data*_o

bypass select = write_enable_i AND (write_addr_i == requested physical tag)
```

Priority is reset zero, then p0 zero, then matching accepted completion, then stored value.
The source expresses this priority using nested conditions, with a complete default assignment
for both outputs. Both read ports have independent address selection and bypass selection.

Changing a read address changes its output without waiting for a clock. Changing a matching
accepted completion value also changes the bypass output combinationally. Neither change alone
updates storage: storage still requires an enabled rising edge.

This distinction is tested by presenting a write that is withdrawn before the clock edge.
The bypass temporarily shows the candidate value, then the old stored value returns; no write
was captured. In the integrated design, accepted-completion controls must remain stable for
the consuming edge rather than being withdrawn this way.

## 6. FPGA implementation implications

The asynchronous read ports and asynchronous reset intentionally describe a small register-based
file. Do not replace it with a synchronous block RAM and assume the same timing: that would add
read latency and change scheduling and operand capture.

This differs from the large program/data memory. That memory will use synchronous block RAM
and its own response handling. A reset loop that is reasonable for this small register file
should not be copied into the entire 64 KiB RAM.

The exact FF/LUT usage and the delay through read-selection/bypass logic require Vivado reports.
The 50 MHz target is not proven by Icarus simulation. Two simultaneous reads and one write are
the behavior contract; no claim of achieved resource use or frequency is made here.

## 7. Verification and waveform reading

Run the clocked self-checking simulation from the project directory:

```sh
python3 scripts/run_checks.py --block 3
```

For all implemented blocks, use `--block all`. For an optional VCD waveform:

```sh
python3 scripts/run_checks.py --block 3 --waves
```

The VCD is written under `build/block03/block03.vcd`. Test evidence and source hashes are in
`reports/block03_results.json`; the simulator log is in `reports/block03_simulation.log`.

The tests exercise reset, both read ports, all register addresses, matching and nonmatching
completion bypass, clocked persistence, disabled/withdrawn writes, p0 protection, unrelated
register preservation, and deterministic random transactions against a separate reference array.
They verify this storage/bypass block, not allocation, readiness, ownership validation, or retirement.

When inspecting the waveform, add these signals:

- clk_i and rst_ni for clock/reset context.
- write_enable_i, write_addr_i, write_data_i for the selected completion.
- read_rs1_i/read_rs2_i and read_data1_o/read_data2_o for each operand.

In the p37 example, find the transition from 7 to 99 before the write edge, then observe 99
remaining after write_enable falls. Next compare the disabled/withdrawn-write example: the
candidate bypass value disappears because no enabled clock edge stored it.

The supplied testbench's measured waveform has the following landmarks (these are simulation
timestamps, not processor instruction counts):

| Time | Observation |
|---|---|
| 5472 ns | Read port 1 sees the previously stored 7 |
| 5481 ns | Clock is low; both ports already see bypassed 99 |
| 5490 ns | Rising edge captures the enabled write |
| 5492 ns | Write enable is low; both ports still see stored 99 |

`reports/block03_bypass_example.csv` contains these values extracted from the generated VCD.

The testbench clock is a simulation stimulus, not a timing-closure measurement. Its timeout
turns a stalled test into a clear failure. The runner also enforces an execution timeout.

For Vivado XSim on Linux, after sourcing your installation's settings file:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_physical_regfile.sv tb/tb_block03.sv
xelab tb_block03 -s block03_sim
xsim block03_sim --runall
```

Icarus simulation is run here. XSim, synthesis, implementation, and the bitstream remain unverified
until the corresponding Linux Vivado commands are actually executed.

## 8. Interview questions and model answers

**Why is the physical register file separate from the rename map?**
The file stores values. The map says which physical value corresponds to an architectural name.
Changing a mapping does not require copying all register data.

**Why have two read ports and one write port?**
An integer instruction may need two source values. The selected one-wide completion path writes
at most one destination per cycle. Multiple waiting results are arbitrated upstream.

**Why bypass a result if the register is written on the same edge?**
A consumer capturing on that edge could otherwise see the old stored value. Bypass supplies
the accepted producer value directly, without depending on simulation assignment ordering.

**What happens if both read ports request the written destination?**
Both independently select the new write data. There is no destructive read or read-port conflict.

**What protects x0?**
The renamer will map x0 to p0 and avoid allocating a new destination. Independently, this file
hardwires p0 reads to zero and ignores writes to p0.

**Does writing a physical register make it architecturally committed?**
No. A younger instruction may complete speculatively. Only in-order retirement advances the
committed map and makes that version part of architectural state.

**Does a misprediction clear this register array?**
No. That would destroy committed values. Recovery changes mappings and ownership, preserving
the committed physical registers.

**Does zeroing the file on reset mark every allocated register ready?**
No. Readiness is control state in the future rename/readiness block. Value bits alone do not
tell the scheduler whether a producer has completed.

## 9. References

- [BOOM physical renaming and committed mappings](https://docs.boom-core.org/en/latest/sections/rename-stage.html)
- [lowRISC SystemVerilog coding style](https://github.com/lowRISC/style-guides/blob/master/VerilogCodingStyle.md)
- [AMD RAM HDL coding techniques](https://docs.amd.com/r/en-US/ug901-vivado-synthesis/RAM-HDL-Coding-Techniques)

This module is original project source. The references provide architectural and coding context;
they are not claims that this custom register file has been verified by those projects.

## Complete physical register file

Source: `rtl/rv32_physical_regfile.sv`.

```systemverilog
// EX register-read / WB completion boundary: two read ports and one write port.
// Register readiness and committed mappings belong to other blocks, not this storage.
`timescale 1ns / 1ps

module rv32_physical_regfile (
  input  logic                     clk_i,
  input  logic                     rst_ni,
  input  rv32_ooo_pkg::phys_reg_t   read_rs1_i,
  input  rv32_ooo_pkg::phys_reg_t   read_rs2_i,
  output rv32_ooo_pkg::word_t       read_data1_o,
  output rv32_ooo_pkg::word_t       read_data2_o,
  input  logic                     write_enable_i,
  input  rv32_ooo_pkg::phys_reg_t   write_addr_i,
  input  rv32_ooo_pkg::word_t       write_data_i
);
  import rv32_ooo_pkg::*;

  // p0 is a constant, so only p1..p63 need storage. Reset makes startup deterministic.
  // The asynchronous reset and reads intentionally favor registers, not block RAM.
  word_t values_q [1:PHYS_REG_COUNT-1];
  integer reg_index;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (reg_index = 1; reg_index < PHYS_REG_COUNT; reg_index = reg_index + 1) begin
        values_q[reg_index] <= '0;
      end
    end else if (write_enable_i && (write_addr_i != 0)) begin
      values_q[write_addr_i] <= write_data_i;
    end
  end

  always_comb begin
    read_data1_o = '0;
    read_data2_o = '0;
    if (rst_ni) begin
      if (read_rs1_i != 0) begin
        // The write port carries an accepted, live, non-faulting completion.
        // Bypass it before waiting for the stored value's clock-edge update.
        if (write_enable_i && (write_addr_i == read_rs1_i)) begin
          read_data1_o = write_data_i;
        end else begin
          read_data1_o = values_q[read_rs1_i];
        end
      end
      if (read_rs2_i != 0) begin
        if (write_enable_i && (write_addr_i == read_rs2_i)) begin
          read_data2_o = write_data_i;
        end else begin
          read_data2_o = values_q[read_rs2_i];
        end
      end
    end
  end
endmodule
```

## Complete self-checking testbench

Source: `tb/tb_block03.sv`.

```systemverilog
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
```
