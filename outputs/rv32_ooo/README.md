# Two-issue RV32 out-of-order processor

This project implements a custom RV32IM_Zicsr_Zifencei processor with two-wide
fetch, dispatch, issue, completion, and in-order retirement. Its five logical
sections are IF, ID/rename, EX/scheduling, MEM, and WB/commit. Queues and
multicycle execution make instruction latency variable.

All 15 local regression groups pass, including the assembled production core.
The integrated tests comprise 16 ISA/data programs with an independent
architectural reference, 25 fault/interrupt scenarios, and seven AXI-connected
programs. See [STATUS.md](STATUS.md) for exact coverage and remaining work.
Official ACT4 results are tracked separately in
[the architectural-test guide](docs/act4_verification.md).

Firmware and FPGA/IP integration were removed from scope by the user.
The agreed Vivado/Artix-7 specifications remain in
[the target note](docs/vivado_target.md). No synthesis, timing closure, or
bitstream result is claimed.

## Read the architecture in order

Each complete guide contains the actual complete RTL and relevant self-checking
SystemVerilog testbenches, preceded by the block's connections, circuit behavior,
cycle examples, signal roles, waveform guidance, and interview questions.

| Block | Complete guide | Architectural function |
|---|---|---|
| 1 | [Configuration](docs/block01_complete_guide.md) | Instruction records, physical/ROB identities, paired transport |
| 2 | [Decoder and integer ALU](docs/block02_complete_guide.md) | Encoding checks, immediates, arithmetic, shifts, comparisons |
| 3 | [Physical register file](docs/block03_complete_guide.md) | Four reads, two writes, constant p0, completion bypass |
| 4 | [Rename and readiness](docs/block04_complete_guide.md) | Pair dependencies, free allocation, speculative/committed maps |
| 5 | [ROB and retirement](docs/block05_complete_guide.md) | Completion ownership, ordered commitment, precise trap trace |
| 6 | [Issue scheduler](docs/block06_complete_guide.md) | Oldest-ready matching, RAW wakeup, execution-resource checks |
| 7 | [Execution units](docs/block07_complete_guide.md) | Two integer lanes and independent iterative multiply/divide |
| 8 | [Completion and writeback](docs/block08_complete_guide.md) | Five held sources, two retained outputs, accepted-result broadcast |
| 9 | [Instruction fetch](docs/block09_complete_guide.md) | 64-bit synchronous memory, paired buffering, stale responses |
| 10 | [Branch prediction](docs/block10_complete_guide.md) | Tagged BTB, two-bit PHT, retirement-ordered training |
| 11 | [Recovery](docs/block11_complete_guide.md) | Cancel/drain barrier, safe map restoration, redirect |
| 12 | [Load/store engine](docs/block12_complete_guide.md) | Ordered memory queue, head-only transactions, byte lanes, faults |
| 13 | [Machine state](docs/block13_complete_guide.md) | CSR commitment, counters, precise exceptions, interrupts, WFI |
| 14 | [AXI and timer](docs/block14_complete_guide.md) | Independent channels, held requests, errors, software/timer IRQs |
| 15 | [Complete processor](docs/block15_complete_guide.md) | Production wiring, AXI wrapper, architectural/trap/bus tests |

[Feature checklist](docs/feature_checklist.md) maps implemented behavior to test evidence
and lists the deliberate boundaries.

[Instruction and hazard reference](docs/isa_and_hazards.md) relates these
instructions to loops, if/else, register renaming, forwarding, and stalls.
The final top-level modules are `rtl/rv32_ooo_core.sv` (native memory handshakes)
and `rtl/rv32_ooo_axi_top.sv` (native instruction RAM plus AXI4-Lite data master).
The wrapper exposes interrupt inputs and retirement trace outputs.

## Run the local tests

Use Python 3, Icarus Verilog, and vvp; no external Python package is needed for
these RTL regressions:

```sh
python3 scripts/run_checks.py --block all
python3 scripts/run_checks.py --block 15 --waves
python3 scripts/test_act4_elf_to_mem.py
python3 scripts/run_act4.py --selftest
python3 scripts/build_guide.py
```

Select any block from `1` through `15`. A nonzero exit indicates a compile error,
failed check, or timeout. Optional VCDs are generated under `build/`. Reports in
`reports/` retain exact source SHA-256 values, scope, counts, and simulator logs.
Assertion counts describe each test's observations; they do not establish
exhaustive coverage or architectural certification.

Block 15 connects the real fetch, rename, scheduler, execution engines,
completion arbiter, ROB, recovery controller, LSU, and CSR unit. Earlier block
tests deliberately isolate components; their documented models remain useful
for learning but are superseded by the full-core evidence for composition.
The AXI-connected tests also use the actual AXI master and custom timer.
Their RAM/address fabric is a simulation model, not delivered FPGA integration.

The ELF harness has its own PASS/FAIL/timeout checks. Those seven infrastructure
cases are locally encoded programs and must not be described as official ACT4
tests. ACT4 dependency versions, configuration, provenance, and execution results
belong to its separate guide and reports.

## Circuit and timing contracts

- Rename removes WAR/WAW false dependencies; physical tags/readiness preserve RAW.
- An independent younger ADD can issue/complete while an older DIV runs. The ROB
  commits their results in program order, at up to two instructions per cycle.
- The two-write-port completion path may backpressure a third finished unit. A
  producer holds its entire result until accepted; a speculative result becomes
  architectural only at retirement.
- Loads/stores prepare addresses ahead of retirement, but actual data requests
  issue only for the live incomplete ROB head. One data transaction is outstanding.
  Stores wait for the response, preventing wrong-path memory/peripheral effects.
- Branch mismatch stops new admission while older work finishes. Recovery waits
  for cancellation acknowledgments and quiet execution before restoring the
  committed rename map and reusing identities.
- Interrupts stop admission, drain admitted work, recheck enables, and save the
  next unexecuted architectural PC. Synchronous faults take priority.
- Native instruction memory has one synchronous read latency. Fetch credits and
  stale-response state preserve ownership through stalls and repeated redirects.
- FENCE.I orders older stores, flushes prefetched instructions/prediction, and
  restarts fetch so modified executable RAM becomes visible.

## Fixed configuration

| Item | Selection |
|---|---|
| ISA/privilege | RV32IM_Zicsr_Zifencei, machine mode only |
| Width | Up to two fetch/dispatch/issue/completion/retirement operations per cycle |
| Registers | 32 architectural; 64 physical; four reads/two writes; p0 constant |
| Queues | 16 ROB, 8 issue, 4 ordered memory, 4 fetch |
| Prediction | 64-entry tagged BTB; 128 two-bit PHT counters |
| Memory | 64 KiB shared executable RAM; 64-bit instruction rows, 32-bit data |
| FPGA specification | xc7a35tcpg236-1, Basys 3 reference, Vivado 2026.1 Linux |
| Clock target | 100 MHz reference; 50 MHz processor/AXI target, unverified |

Actual throughput depends on dependencies, functional-unit availability,
branches, and memory. Two-issue does not guarantee IPC=2 or CPI=1 on every
program. Global-history/loop predictors, RAS, caches, speculative memory replay,
MMU, U/S modes, OS boot, floating point, vectors, atomics, and compressed
instructions are outside this version.

## Historical learning material

`reference/single_issue` preserves the original verified blocks 1–4 and reports.
`reference/single_issue_drafts` holds old unverified ROB drafts. These are
historical teaching files, excluded from all active build lists. Their original
module names duplicate active modules; do not recursively compile reference
folders with active RTL. Use `rtl/files.f` or the scripts' explicit source lists.
