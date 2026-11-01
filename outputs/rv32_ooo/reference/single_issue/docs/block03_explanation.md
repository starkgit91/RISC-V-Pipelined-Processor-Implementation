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
