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
