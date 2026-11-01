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
