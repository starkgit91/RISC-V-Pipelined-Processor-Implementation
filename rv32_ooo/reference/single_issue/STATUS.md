# Implementation evidence and next checkpoint

The overall objective remains incomplete. Completion requires the processor, AXI system,
firmware, documentation, and the agreed verification/build deliverables.

| Block | Deliverable | Current state |
|---|---|---|
| 1 | Configuration, instruction definitions, physical tags, queue records | Implemented; Icarus declaration tests pass |
| 2 | Decoder, immediate generator, ALU, shifts, comparisons | Implemented; Icarus combinational tests pass |
| 3 | Physical register file and bypass | Implemented; Icarus clocked tests pass |
| 4 | Rename maps, free list, readiness | Implemented; Icarus control-state tests pass |
| 5 | Reorder buffer and retirement | Awaiting user learning checkpoint |
| 6 | Issue queue and oldest-ready scheduling | Not implemented |
| 7 | Independent integer and iterative multiply/divide execution | Not implemented |
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

Block 3's clocked tests cover physical value storage, asynchronous reset, two independent read
ports, accepted-completion bypass, p0 protection, and persistence only at enabled write edges.
They use an independent reference array and include all 64 x 64 read-address pairs, walking data
bits, and 1,024 deterministic random cycles. They do not validate readiness, rename allocation,
ROB ownership checks, completion arbitration, or architectural commitment. Exact evidence is in
`reports/block03_results.json`; an optional VCD is generated with the runner's `--waves` option.

Block 4 checks speculative/committed maps, physical allocation, readiness, completion bypass,
in-order retirement updates supplied by the testbench, and recovery application. It includes
controlled concurrent events, unit-level pool exhaustion, and mixed transactions checked against
an independent reference. It does not implement the ROB, scheduler, completion ownership checks,
or the execution-cancellation barrier. See `reports/block04_results.json` for exact evidence.

Vivado is not installed in this workspace. Its compilation, synthesis, timing, DRC, and bitstream
results remain unverified until the corresponding Linux flow is actually run.

## Block 5 learning checkpoint

The reorder buffer will receive renamed instructions in program order and tagged completions
from execution. It will retain each instruction's current result/exception/control-flow state,
validate completion identity, and expose the oldest instruction for in-order retirement or traps.
Completion alone must not advance architectural state or release the wrong physical version.

The rename block's retirement inputs will come from this oldest-instruction decision. Branch
recovery and traps must preserve older work and reject results from cancelled/recycled identities.

Question before block 5 implementation: begin with an older DIV and a younger ADD that completes
first, or with an older fault and a younger completed instruction to explain precise exceptions?
Both scenarios are included; the answer determines the first worked example.
