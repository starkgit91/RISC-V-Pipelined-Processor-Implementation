# Feature checklist and evidence boundaries

"Implemented" means functional RTL is supplied and connected where applicable.
"Tested" identifies actual executable evidence. It does not promise exhaustive
coverage, hardware timing, or certification. The source-hashed JSON reports and
their logs are authoritative for the current regression results.

| Feature | Implemented behavior | Local verification evidence |
|---|---|---|
| IF/ID/EX/MEM/WB | Five logical sections with queues, valid ownership, and multicycle operations | Blocks 1–15; memory-fed complete-core programs |
| Two-issue OoO | Two-wide admission/issue/completion/retirement; younger ready work bypasses blocked older work | Blocks 5–8 and 15; independent ADD completes before older DIV |
| Integer ISA | RV32I arithmetic, logic, shifts, comparisons, branches, jumps, upper immediates, loads/stores | Block 2 encoding tests; block 15 independent architectural reference |
| RV32M | All eight multiply/divide/remainder operations, iterative execution | Block 7 boundary/random tests and full-core M programs |
| Zicsr/Zifencei | Six CSR forms, serialized machine state, FENCE.I | Blocks 2/13/15; executable RAM update with old word already prefetched |
| Physical register file | 64 entries, four reads, two writes, constant p0, accepted-result bypass | Block 3 and connected back-end tests |
| Renaming | Speculative and committed maps, free allocation, ready bits, ordered release | Block 4 WAR/WAW/RAW cases; full-core repeated destinations |
| RAW handling | Physical producer tags, source-use decode, readiness, wakeup, completion bypass | Blocks 2–8; dependent arithmetic and load consumers in block 15 |
| Capacity stalls | Atomic ROB/IQ/physical/memory allocation; no partial dispatch | Unit exhaustion tests; full-core ROB/IQ/memory-queue pressure |
| Retirement | Complete oldest instructions retire in order; younger completed values remain speculative | Block 5 and all block 15 architectural traces |
| Completion pressure | Five sources arbitrate onto two retained slots; producers hold results | Block 8 three-way completion and backpressure cases |
| Prediction | 64-entry tagged BTB; 128 two-bit counters; two lookups; ordered retirement training | Block 10 reference model and loops; block 15 enabled/disabled trace equivalence |
| Branch recovery | Wait for retirement boundary, cancel/drain producers, restore committed mappings, redirect | Block 11 protocol tests; block 15 real mispredictions with DIV, older faults, younger completed work |
| Owner reuse | Full ROB index/generation and destination checks; quiescence before reuse | Blocks 4/5/7/8/11 and full-core recovery programs |
| Fetch backpressure | Synchronous 64-bit reads, four-instruction FIFO, reserved credits, partial-prefix compaction | Block 9 throughput, stalls, repeated redirects, fault/stale-response tests |
| Memory operations | Byte/halfword/word lanes, signed/unsigned extension, alignment faults | Blocks 2/7/12 and complete-core data programs |
| Memory ordering | Prepare early; one actual transaction only at live incomplete ROB head | Block 12 ownership tests; block 15 bus/head assertions and architectural memory reference |
| Wrong-path effects | Younger memory instructions cannot access RAM/MMIO before reaching head | Block 15 wrong-path read/write MMIO cases and fault-versus-branch scenarios |
| Access errors | Instruction/load/store errors become precise exceptions; faulting destination suppressed | Blocks 9/12/15, native and AXI SLVERR programs |
| Machine CSRs | Documented implemented subset, masks, fixed/read-only values, 64-bit counters/high halves | Block 13 and full-core CSR forms/trap handlers; explicit counter-write regressions |
| Precise synchronous traps | Oldest fault at ROB head; older retirements preserved, younger effects discarded | 11 synchronous fault fixtures and mixed-fault/recovery scenarios in block 15 |
| Interrupts | External/software/timer, priority, mask/recheck, drain, architectural next PC | Block 13 state tests; direct/vectored full-core cases; actual AXI timer/MSIP programs |
| MRET/WFI | Retirement-controlled status return; wait without clock gating; local-pending wake rules | Block 13 and full-core trap/IRQ/WFI tests |
| AXI4-Lite | Independent AW/W acceptance, held AR, B/R completion, strobes, error propagation | Block 14 protocol tests; seven complete AXI-connected programs |
| Machine timer | AXI slave for MSIP, MTIME, MTIMECMP; level IRQ outputs | Block 14 plus actual timer-connected core programs |
| Diagnostic trace | Ordered retirement/trap/interrupt records and ownership/protocol diagnostics | Trace tests in blocks 5–15; custom trace, not an RVFI compliance claim |

The free-list unit is tested to exhaustion independently. With 64 physical
registers and a 16-entry ROB, ROB capacity normally stalls integrated dispatch
before all spare physical registers are consumed. A unit stress result is not
described as a reachable full-core condition.

The complete core has **755,125 local regression checks** across 15 groups.
This count includes repeated comparisons and protocol assertions. Architectural
program execution is separate: all **135 delivered ACT4-derived ELFs pass** with
zero failures, conversion rejections, or timeouts. Their exact manifest and
generation adaptations are retained in `verification/act4/` and explained in
[act4_verification.md](act4_verification.md). The loader/harness/batch checks
have 10/seven/49 synthetic expectations and are not architectural test counts.

The fresh-directory waveform run repeats all 29,474 complete-core checks with
zero failures and produces the three documented VCDs. It is recorded separately
in `reports/clean_waveform_run.json`; it is not added again to the 755,125 total.

## Deliberate boundaries

| Area | Current boundary |
|---|---|
| Memory-level parallelism | One outstanding data transaction, no speculative loads, replay, or store-to-load forwarding |
| Prediction | No global history, dedicated loop-count predictor, or return-address stack |
| Execution | Two-wide scalar issue; no vector/FPU/atomic/compressed extensions |
| Privilege | M-mode only; no U/S/H modes, MMU, virtual memory, PMP, or OS boot |
| Architectural tests | I/M/Zicsr/Zifencei selection; privileged tests excluded. One preserved JAL program's cumulative layout exceeds 64 KiB and is unexecuted, not counted as a pass. |
| Cache/external memory | No instruction/data caches, coherence fabric, DDR controller, or DMA |
| Debug/multicore | No JTAG/debug module, hardware breakpoints, external debug halt protocol, or multicore coherence |
| FPGA integration | Removed from scope: board/IP wrapper, AMD interconnect/peripherals, constraints, implementation and bitstream flow |
| Firmware | Removed from scope: application firmware, startup/runtime, drivers, and board demonstrations |
| FPGA specification | Retained: Artix-7 xc7a35tcpg236-1, Vivado 2026.1 Linux, 100 MHz reference / 50 MHz target |
| Signoff evidence | No Vivado fit/timing/DRC, power, CDC/RDC analysis, exhaustive formal proof, silicon result, or certification claim |

External clock/reset, interrupt sources, instruction storage, and AXI fabric
must satisfy their documented interface contracts. Reset the external bus
endpoints consistently with the core; an endpoint must not deliver a pre-reset
transaction's response as a new post-reset response. Interrupt inputs at the
core boundary must already be synchronous to the processor clock.

These boundaries make the implemented learning architecture reviewable. They
do not turn missing product or signoff features into passing test results.
