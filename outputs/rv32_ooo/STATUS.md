# Implementation and verification status

The two-issue out-of-order processor and its portable AXI wrapper are implemented.
All 15 local regression groups pass on the current sources. Block 15 exercises
the assembled production core, including its production recovery controller.
Official ACT4 setup, final documentation, and delivery audit are in progress.
The overall task remains active until that work is finished.

Firmware and FPGA/IP integration were removed from scope by the user.
The agreed part and clock targets remain in `docs/vivado_target.md`.
No Vivado synthesis, timing, DRC, or bitstream result is claimed.
The initial 20-minute limit was exceeded; the user authorized continued
full-core verification and waived checkpoint waits.

| Block | Checks passed | Scope |
|---|---:|---|
| 1 | 130 | Shared configuration and packed records |
| 2 | 22,708 | Decode, immediate generation, ALU, comparisons |
| 3 | 155,389 | Four-read/two-write physical register file and bypass |
| 4 | 155,233 | Dual rename, allocation, readiness, committed mapping |
| 5 | 18,611 | ROB ownership, ordered retirement, precise fault trace |
| 6 | 34,097 | Two-port oldest-ready scheduling and dependency wakeup |
| 7 | 121,718 | Integer/branch/address execution and iterative RV32M |
| 8 | 53,111 | Completion arbitration, backpressure, accepted writeback |
| 9 | 66,609 | Paired synchronous fetch, buffering, stale responses |
| 10 | 75,980 | BTB/PHT lookup, ordered training, loop prediction |
| 11 | 19,625 | Production cancellation/quiescence/recovery protocol |
| 12 | 1,067 | Ordered memory queue, lanes, extension, fault handling |
| 13 | 511 | Committed CSRs, counters, traps, IRQ eligibility, WFI |
| 14 | 862 | Independent AXI channels, errors, machine timer |
| 15 | 29,474 | Assembled core architectural, trap, and AXI programs |

Check counts are assertions at the stated test scope, not numbers of distinct
instructions or a measure of exhaustive coverage. Exact source hashes and logs
are recorded in `reports/blockNN_results.json` and accompanying logs.

Block 15 comprises 16 ISA/data programs checked against an independently written
architectural reference, 25 trap/interrupt scenarios, and seven AXI-connected
programs. It covers real out-of-order completion with ordered retirement,
wrong-path MMIO suppression, older-fault/younger-branch priority, replacement of
a younger recovery boundary by an older branch, interrupt draining and MRET,
read/write bus errors, and a prefetched old instruction replaced using FENCE.I.
Prediction-enabled and disabled loop runs have equal architectural traces.

Tests found and resolved a counter-write priority bug: an explicit write now
suppresses the implicit update to the entire 64-bit counter on that edge.
The LSU also rejects a preparation packet with mismatched signedness metadata.
Both fixes have directed regressions and pass the full-core suites.

Block 10's early loop test uses test-only recovery. Blocks 11 and 15 supply the
separate evidence for the production recovery implementation. Earlier connected
block tests retain their documented test models; their pass is not substituted
for the later full-core evidence.

The AXI wrapper exposes a native instruction-RAM port and an AXI4-Lite data
master. RAM, address decoding, and peripherals around that wrapper are testbench
models here. The custom timer is synthesizable RTL. AMD block design, board
wrapper, UART/GPIO IP, firmware, and bitstream generation are outside the revised
scope. The RTL is written for synthesis, but FPGA fit and timing are unverified.

No advance guarantee, exhaustive formal proof, architectural certification,
silicon validation, or production-readiness claim is made. ACT4 results will be
reported separately from these local directed and randomized tests.
