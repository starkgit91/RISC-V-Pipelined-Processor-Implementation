5-Stage Pipelined RISC-V Processor

This project implements a 32-bit RISC-V processor (RV32I subset) using a 5-stage pipelined architecture. It is written in Verilog and includes a Python-based behavioral simulator for visualization.

Features

5-Stage Pipeline: Fetch (IF), Decode (ID), Execute (EX), Memory (MEM), Writeback (WB).

Hazard Handling:

Data Hazards: Full forwarding unit (EX-to-EX, MEM-to-EX) to resolve Read-After-Write (RAW) dependencies without stalling when possible.

Load-Use Hazards: Detection unit stalls the pipeline for one cycle if a dependent instruction follows a Load immediately.

Control Hazards: Branch prediction assumes "branch not taken". If a branch is taken, the pipeline flushes instructions in the IF and ID stages.

Instruction Set: Supports a subset of RV32I:

Arithmetic: ADD, SUB, ADDI

Memory: LW, SW

Control Flow: BEQ

File Structure

riscv_pipeline.v: The synthesized Verilog source code containing the Top Module, Datapath, Control Unit, ALU, Hazard Detection, Forwarding Unit, and Memories.

pipeline_sim.py: A cycle-accurate Python simulator that visualizes the pipeline stages, register values, and hazard handling textually.

How to Run

1. Verilog Simulation (RTL)

You can simulate the hardware design using any standard Verilog simulator (e.g., ModelSim, Vivado, Verilator, Icarus Verilog).

Using Icarus Verilog:

iverilog -o riscv_sim riscv_pipeline.v
vvp riscv_sim


Expected Output:
The testbench (tb_riscv) monitors the Program Counter (PC), fetched instructions, and Writeback data.

2. Python Behavioral Simulation

For a quick visualization of how instructions move through the pipeline stages without needing EDA tools:

python3 pipeline_sim.py


This will print a table showing the state of every pipeline stage (IF, ID, EX, MEM, WB) per clock cycle, highlighting stalls, bubbles, and forwarding.

Architecture Overview

The design follows the classic RISC pipeline structure:

Instruction Fetch (IF): Updates PC, fetches instruction from IMEM.

Instruction Decode (ID): Decodes opcode, reads Register File, generates immediate values.

Execute (EX): Performs ALU operations. The Forwarding Unit muxes operands here to bypass stale register data.

Memory (MEM): Accesses Data Memory for Load/Store operations.

Writeback (WB): Writes results back to the Register File.

Test Program

Both the Verilog testbench and Python simulator run the following hardcoded assembly program to demonstrate pipeline features:

0x00: addi x1, x0, 10   # Initialize x1 = 10
0x04: addi x2, x0, 5    # Initialize x2 = 5
0x08: sub  x3, x1, x2   # x3 = x1 - x2 = 5 (Hazard: Forwarding used)
0x0C: sw   x3, 0(x0)    # Store 5 to address 0
0x10: lw   x4, 0(x0)    # Load 5 into x4
0x14: beq  x3, x4, +8   # Branch to 0x1C if x3 == x4 (Taken)
0x18: addi x5, x0, 99   # Flushed instruction (due to branch)
0x1C: addi x6, x0, 100  # Target of branch


Continuation To upgrade: **RISCV 5-Staged Pipelined Processor Design with Two issue Out-of-Order Execution** : Enabling Robust Hazard handling, Dynamic Predictor and two issue out of order execution for Instruction level Parallelism.


# Two-issue RV32 out-of-order learning project

The target is one RV32IM_Zicsr_Zifencei processor, organized as five logical sections:
IF, ID/rename, EX/scheduling, MEM, and WB/commit. The approved revision is two-wide fetch,
dispatch, issue, completion, and retirement. Instructions still commit in program order.

**Current state: blocks 1 through 7 are implemented and tested at block level.** These include
shared definitions, decoder/ALU/comparator, four-read/two-write physical registers, two-wide
renaming, a two-wide ROB/trace recorder, the two-issue scheduler, and real integer/multiply/divide
execution paths. This is not yet a complete CPU. Completion arbitration, fetch, LSU, prediction,
machine control, AXI, firmware, and Vivado integration remain.

## Read the lessons in order

Each complete guide includes the actual source files compiled by the tests, explanatory
connections, signal roles, examples, and interview questions.

| Block | Complete guide | Implemented behavior |
|---|---|---|
| 1 | docs/block01_complete_guide.md | Configuration, per-instruction records, and paired transport |
| 2 | docs/block02_complete_guide.md | Full supported encoding decode, integer ALU, branch comparisons |
| 3 | docs/block03_complete_guide.md | Four reads, two writes, p0, dual completion bypass |
| 4 | docs/block04_complete_guide.md | Pair dependencies, allocation, readiness, ordered map updates |
| 5 | docs/block05_complete_guide.md | Dual completion, ordered retirement/traps, recovery lock, trace |
| 6 | docs/block06_complete_guide.md | Oldest-ready two-port matching, accepted-result wakeup, recovery gating |
| 7 | docs/block07_complete_guide.md | Independent execution, iterative RV32M, held results, cancellation |

The decoder is a reusable per-instruction module; the later front end will decode a pair.
Both integer execution instances reuse the block-2 ALU. Multiply/divide runs in its separate
iterative unit; CSR/system execution and the machine-state controller remain later blocks.

## Run the available tests

From this directory with Python 3, Icarus Verilog, and vvp available:

```sh
python3 scripts/run_checks.py --block all
python3 scripts/build_guide.py
```

Select one block with `--block 1` through `--block 7`. Add `--waves` for optional clocked VCDs
under build/block03 through build/block07. No external Python packages are required.
The runner returns a nonzero status on compilation error, failed checks, or timeout.

Each reports/blockNN_results.json identifies scope, test cases, counts, and exact source hashes.
The accompanying log retains simulator output, including Icarus's known always_comb whole-vector
sensitivity notices. A successful block test is not whole-core ISA compliance or timing closure.

Block 5 includes an additional connected control/storage test: rename and ROB agree on atomic
pair acceptance, accepted completions reach physical storage/readiness, retirement advances the
committed map, and stale results cannot overwrite stored values. Values are injected by the
bench; this does not simulate a real DIV unit.

Block 6 adds an independent scheduler test and a connected rename/ROB/PRF/IQ/trace test. Two
ready younger instructions issue while an older instruction waits; an accepted producer result
wakes that instruction with same-cycle PRF bypass. The scheduler is real RTL, but execution
engines in that historical test are modeled. Block 7 connects real execution engines and uses a
test-only completion selector; production completion arbitration remains block 8.

Block 7's actual RTL demo issues DIV100/4 and ADD7+9 together at cycle 6. ADD completes at cycle 7,
DIV at 39, and the dependent ADD captures 25+16 through bypass on that same cycle. DIV and ADD
retire in order at cycle 40; the dependent result 41 retires at cycle 41. Separate tests cover
RV32M arithmetic, branch/address faults, retained outputs, cancellation, and safe owner reuse.

## Vivado XSim on Linux

Source your Vivado installation's settings64.sh first. For the standalone ROB test:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_reorder_buffer.sv rtl/rv32_trace_recorder.sv tb/tb_block05.sv
xelab tb_block05 -s block05_sim
xsim block05_sim --runall
```

Each lesson includes its own source list and XSim commands. The package must compile first.
Vivado/XSim, synthesis, placement/routing, timing, DRC, and bitstream generation have not been
run in this workspace. Their reproducible FPGA flow belongs to the later integration blocks.

## Fixed target

| Item | Configuration |
|---|---|
| ISA | RV32IM_Zicsr_Zifencei, machine mode only |
| Width target | Up to two fetch/dispatch/issue/completion/retirement operations per cycle |
| Physical register file | 64 identities, four reads, two writes; p0 constant |
| Queues | 16 ROB, 8 issue, 4 ordered memory, 4 fetch |
| Prediction target | 64-entry BTB and 128 two-bit PHT counters |
| Memory target | 64 KiB shared executable dual-port BRAM |
| FPGA target | xc7a35tcpg236-1, Basys-3 reference pins |
| Tools | Vivado 2026.1 on Linux; portable simulation with Icarus |
| Clock target | 100 MHz reference input, 50 MHz system |

Two-issue permits two independent instructions to issue together; it does not guarantee IPC=2
on every program. Dependencies, resource availability, branches, and ordered memory still matter.
The physical/committed versions, completion/retirement distinction, and precise recovery rules
remain essential.

Memory requests will start only at a live incomplete ROB head, one outstanding data transaction
at a time. Branch recovery restores committed mappings after cancellation/draining; it preserves
physical values and slot generations. Actual FPGA fit and timing remain to be verified.

## Historical checkpoint

reference/single_issue contains the verified original blocks 1–4, including their source-hashed
reports and lessons. reference/single_issue_drafts contains the unverified old ROB/trace drafts.
These are teaching references, not part of the active two-issue source lists. Historical files
retain their original module names: compiling reference files with active RTL would create
duplicate definitions. Use the provided active rtl/tb lists.

STATUS.md tracks remaining blocks and the next user learning checkpoint. The overall processor
objective remains active and incomplete.


# Implementation evidence and next checkpoint

The overall objective remains incomplete. Completion requires the processor, AXI system,
firmware, documentation, and the agreed verification/build deliverables.

## Current architecture decision

The user selected two-issue after the architecture discussion. Blocks 1–7 now use the revised
interfaces for two-wide dispatch, issue, completion, and retirement; fetch remains a future
block. The active register file has four read ports and two write ports; two-wide renaming and
the ROB are tested as well.
The verified single-issue checkpoint is preserved under reference/single_issue. Reports from
that checkpoint do not prove the revised interfaces; current tests must be rerun after revision.

Block 5's provisional single-issue ROB/trace-recorder drafts are saved outside active RTL.
The active two-wide ROB/trace revision and connected control/storage test pass Icarus. The user's
chosen teaching example remains an older DIV and a younger ADD completing first.

| Block | Deliverable | Current state |
|---|---|---|
| 1 | Configuration, instruction definitions, physical tags, queue records | Implemented; Icarus declaration tests pass |
| 2 | Decoder, immediate generator, ALU, shifts, comparisons | Implemented; Icarus combinational tests pass |
| 3 | Physical register file and bypass | Four-read/two-write revision tests pass |
| 4 | Rename maps, free list, readiness | Two-wide revision tests pass |
| 5 | Reorder buffer and retirement | Two-wide unit and connected control/storage tests pass |
| 6 | Issue queue and oldest-ready scheduling | Two-issue unit and connected scheduling tests pass |
| 7 | Independent integer and iterative multiply/divide execution | Unit and connected real-execution tests pass |
| 8 | Completion arbitration and speculative writeback | Not implemented |
| 9 | Fetch unit and synchronous BRAM adapter | Not implemented |
| 10 | BTB/PHT predictor | Not implemented |
| 11 | Misprediction recovery and cancellation | Not implemented |
| 12 | Ordered memory queue and LSU | Not implemented |
| 13 | CSRs, exceptions, interrupts, system instructions | Not implemented |
| 14 | AXI4-Lite master and machine timer | Not implemented |
| 15 | Complete processor integration and OoO demonstrations | Not implemented |
| 16 | Vivado block design, FPGA wrapper, constraints | Not implemented |
| 17 | Firmware, architectural tests, implementation and bitstream flow | Not implemented |

## Current evidence

Block 1's compilation and tests cover configuration dimensions, ROB identity representation,
record transport, physical-destination metadata, exception metadata, and memory request fields.
They do not simulate instruction execution, allocation, scheduling, recovery, AXI, or interrupts.
See `reports/block01_results.json` for source hashes and the exact check count.

Block 2's compilation and tests cover encoding legality, immediate generation, source-use flags,
integer ALU results, and branch conditions. The testbench covers all R-type selector combinations,
all signed 12-bit I/S immediates, shift restrictions, edge/random operands, and other directed cases.
It does not implement or test the full execution wrapper, renaming, retirement, memory effects,
CSR side effects, or trap entry. See `reports/block02_results.json` for exact evidence and scope.

Block 3's clocked tests cover physical value storage, asynchronous reset, four independent read
ports, two writes, dual completion bypass, p0 protection, and enabled-edge persistence.
They use an independent reference array and include all write-address pairs, read-port combinations,
conflict diagnostics, and randomized legal cycles. They do not validate readiness, rename allocation,
ROB ownership checks, completion arbitration, or architectural commitment. Exact evidence is in
`reports/block03_results.json`; an optional VCD is generated with the runner's `--waves` option.

Block 4 checks paired speculative/committed maps, physical allocation, readiness, completion bypass,
dual ordered retirement supplied by the testbench, and recovery application. It includes
within-pair dependencies, prefix/compaction, concurrency, unit exhaustion, and mixed traffic against
an independent reference. It does not implement the ROB, scheduler, completion ownership checks,
or the execution-cancellation barrier. See `reports/block04_results.json` for exact evidence.

Block 5 checks two-wide dispatch/completion/retirement, ownership/generation rejection, precise
fault prefixes, serialization, recovery locking, safe apply, and ordered trace registration.
An additional connected test couples rename, ROB, physical registers, and trace with injected
completion values. It verifies atomic pair handshakes and accepted-versus-stale write gating.
Actual execution units, cancellation, memory transactions, and CSR updates remain
unimplemented. `reports/block05_results.json` lists the two test cases and their source hashes.

Vivado is not installed in this workspace. Its compilation, synthesis, timing, DRC, and bitstream
results remain unverified until the corresponding Linux flow is actually run.

Block 6 checks the eight-entry queue, two-port oldest-ready matching, source readiness,
accepted dual-completion wakeup, operand capture, current-capacity prefix admission, resource
availability, wraparound, system-head restrictions, misprediction filtering, and safe clear.
The standalone selection oracle enumerates legal assignments independently of the RTL's greedy
matching. Its 34,054 checks and the connected test's 43 checks pass: 34,097 total. The connected
case includes rename, ROB, PRF, IQ, and trace, with simulated execution sinks and injected values.
It demonstrates out-of-order issue and in-order retirement, not real DIV execution.
See reports/block06_results.json for exact source hashes and scope.

Block 7 implements two integer instances, branch/jump/address preparation on port 0, and a
separate 32-iteration multiply/divide unit. Result buffers retain owner metadata under
backpressure. All three resources cancel running/held work and acknowledge quiescence.
The memory preparation record now preserves control-flow metadata until the eventual LSU
completion. No memory request or CSR side effect is implemented by these units.

121,060 standalone checks and 658 connected checks pass: 121,718 total. Actual decoded ADDI
instructions seed operands; real DIV and ADD both issue at cycle 6, ADD completes at 7, DIV at
39, and the dependent ADD issues at 39. Retirement is DIV/independent ADD at 40, dependent ADD
at 41. Three held results, cancellation, generation reuse, and delayed-result suppression are
also tested. The connected completion selector is test-only; block 8 remains unimplemented.
See reports/block07_results.json for source hashes and precise scope. Earlier source-hashed
reports remain valid against the updated shared package after regression.

## Block 8 learning checkpoint

The completion arbiter will receive independently held ALU-0, ALU-1, multiply/divide, and later
memory/system results. It must select at most two per cycle, retain selection under downstream
backpressure, consume rejected/stale owners safely, and send only ROB-accepted nonfaulting
values to the PRF/readiness/wakeup paths. It will distinguish result consumption from valid
speculative writeback and from separate architectural retirement.

Question before block 8 implementation: start with three units finishing together while only
two writeback slots are available, or trace accepted versus stale completion ownership? Both
are covered; the answer selects the leading example. No block 8 RTL is written yet.
