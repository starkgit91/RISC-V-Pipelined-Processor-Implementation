# ACT4 configuration, execution, and evidence

**Official execution status: not run. Zero official ACT4 test ELFs have executed on this
processor at this checkpoint.** Configuration and tool preparation are underway. The local
RTL regressions and synthetic infrastructure tests are separate evidence; no architectural
certification or advance guarantee of test outcomes is claimed.

[ACT4](https://github.com/riscv/riscv-arch-test/tree/2dc36dd2f3d25383039effa10f1a3930a78a945a)
builds self-checking executables using a configured Sail reference model. The final executables
must then run on the actual DUT. A successful reference build, ELF conversion, or test-harness
selftest is not a passing DUT architectural test.

## Pinned inputs and current preparation

The upstream checkout is pinned to `2dc36dd2f3d25383039effa10f1a3930a78a945a` in the workspace's
`work/act4_upstream`. The delivered configuration is in
[verification/act4](../verification/act4/README.md): `test_config.yaml`, `custom-rv32im.yaml`,
`sail.json`, `link.ld`, and `rvmodel_macros.h`. Adapted upstream material retains its copyright,
Apache-2.0 notices, and the accompanying license.

The isolated Linux environment starts from the official ACT4 build image:

```text
ghcr.io/riscv/act4-build@sha256:6fdbb2a5acadd6e9a3aecf8081bc73d25f1e50e55ae8d23dbec03ec0959e9c0f
```

That image supplies RISC-V GCC 16.1 and an older Sail 0.13.1. The official Sail 0.16 Linux
AArch64 release was added; its downloaded archive SHA-256 is:

```text
92163c1b02b416ae6a031283e0b2085674daed5dfa2d79b1c929ef23da51da73
```

The custom Sail configuration validates with the newer model. Preparation also uses Python
3.14.6, uv 0.11.33, Ruby 3.4.10, Bundler 4.0.23, and UDB 0.1.18. The Ruby/native build dependencies
and UDB solver setup are being completed inside the container. A first solver download was
truncated to approximately 22.8 MB of an expected 33 MB; checksum verification rejected it and
it was quarantined. A resumed fresh download is being verified against the upstream checksum.
Neither an incomplete download nor an unverified executable is treated as a usable dependency.

The earlier `reports/act4_prerequisites.json` and `reports/act4_docker_startup.json` describe
historical host probes. Their missing tools/daemon findings do not describe the later isolated
container. They are diagnostic snapshots, not official test results. Tool preparation does not
yet establish that all selected test ELFs build, fit RAM, or pass the DUT.

## Applicable selection and platform limits

The initial build selects **I, M, Zicsr, Zifencei** and sets `include_priv_tests: false`.
Zmmul is declared as the subset contained in M. The configuration's Sm vocabulary and
`STANDARD_SM_SUPPORTED` hook enable shared machine CSR/trap infrastructure; they do not claim
Sm/profile certification or a completed privileged test campaign.

| Area | Configuration and evidence boundary |
|---|---|
| Integer ISA | RV32I plus all eight M instructions; 32 registers, XLEN=32, IALIGN=32 |
| Ordering/CSR ISA | Zicsr and Zifencei; actual implemented CSR subset only |
| Memory | One little-endian 64 KiB RAM at zero; word/halfword misalignment traps |
| Machine platform | One hart, M-mode only, reset entry zero, fixed documented CSR policies |
| Privileged tests | Excluded from this initial build; no ACT4 interrupt coverage claimed |
| Absent features | No C/A/F/D/V, RV64, U/S modes, PMP, MMU, or cache-dependent platform |

The linker checks entry zero and RAM fit, including code, data, BSS, stack, signatures, and trap
code. Oversized tests are build gaps to resolve using supported generation/splitting, not tests
to silently omit or accommodate by enlarging the DUT RAM. The custom boot initializes the
implemented machine state without writing absent optional HPM/event CSRs. The configuration
README records the remaining WARL/privilege limitations.

The standalone DUT harness ties external/software/timer IRQ inputs low. Sail-only reference
CLINT/interrupt services do not add those peripherals to the DUT. The separate core/AXI
interrupt regressions remain independent evidence. Firmware and FPGA/IP integration are outside
the revised delivery scope; these generated assembly executables are verification inputs.

## Build final self-checking ELFs

In a prepared environment, use the pinned checkout and the delivered configuration. For a
container with the checkout at `/act4` and this project mounted at `/project`:

```sh
make -C /act4 \
  CONFIG_FILES=/project/verification/act4/test_config.yaml \
  WORKDIR=/project/build/act4 \
  EXTENSIONS=I,M,Zicsr,Zifencei
```

Ensure the selected `sail_riscv_sim` is the newer validated executable, not the older executable
in the base image. Retain the compiler, Sail, Python, Ruby/UDB versions and full build log.
The expected final output directory is `build/act4/custom-rv32im/elfs`. The upstream build first
runs signature-generating `.sig.elf` files on Sail and then produces final self-checking ELFs.
Only the final `elfs` directory is a DUT input. Do not point the batch runner at the surrounding
build tree, where intermediate signature ELFs can also exist.

## Verify the loader and harness separately

From the delivered project root, with Python 3, Icarus Verilog, and vvp available:

```sh
python3 scripts/test_act4_elf_to_mem.py
python3 scripts/run_act4.py --selftest
```

The recorded infrastructure evidence comprises:

| Evidence | Recorded outcome | What it checks |
|---|---|---|
| `reports/act4_loader_results.json` | 10 synthetic loader tests pass | ELF validation, memory layout/BSS, bounds, and rejection paths |
| `reports/act4_harness_results.json` | Seven expected outcomes match | Real-core PASS, intentional FAIL/timeout, alternate mailbox/latency, byte lanes, missing image |
| `reports/act4_suite_infrastructure.json` | 31 expectations match | Batch accounting, mixed outcomes, exact manifest inventory/hashes, empty-suite rejection |

The batch infrastructure fixtures are locally encoded synthetic ELFs. Its intentional failures
are successful checks of failure handling, not failed official ACT4 tests. None of these three
reports establishes that an upstream architectural executable passed.

The loader accepts ELF32 little-endian RISC-V integer-ABI `ET_EXEC`, with entry zero and all
loadable segments inside RAM. It rejects unsupported flags, malformed/overlapping segments,
virtual/physical address disagreement, and overflow. It zero-fills BSS and unused RAM; it does
not relocate an executable. A direct conversion example is:

```sh
python3 scripts/act4_elf_to_mem.py build/act4/example.elf \
  --words build/act4/example.words.hex \
  --rows build/act4/example.rows.hex \
  --report build/act4/example.load.json
```

The word image has 16,384 32-bit words; the row image has 8,192 64-bit rows. Row `n` contains the
word at byte address `8*n` in its low half and the word at `8*n+4` in its high half. The harness
uses one shared RAM so data stores followed by FENCE.I can change fetched instructions.
A conversion report deliberately says `converted_not_executed`.

## Execute a single ELF or the entire final directory

`tb/tb_act4.sv` connects the production core and synchronous instruction adapter to shared RAM.
It models strobed data requests, configurable response delay, access faults outside RAM, and a
termination mailbox at `0x1000fff0`. It contains no AXI fabric, timer, UART, or GPIO model.

```sh
python3 scripts/run_act4.py --elf build/act4/example.elf \
  --tohost 0x1000fff0 --max-cycles 1000000 --data-latency 2 --predict 1 \
  --vcd build/act4/example.vcd

python3 scripts/run_act4_suite.py \
  --elf-dir build/act4/custom-rv32im/elfs \
  --manifest build/act4/build_manifest.json \
  --output-dir build/act4_dut_results \
  --tohost 0x1000fff0 --max-cycles 1000000 --data-latency 2 --predict 1
```

Create the build manifest only after generation finishes. It must identify the pinned upstream
revision and retained build configuration, and map **every relative final ELF filename** to its
64-hex-digit SHA-256. The JSON shape is:

```json
{
  "upstream_revision": "2dc36dd2f3d25383039effa10f1a3930a78a945a",
  "config": "verification/act4/test_config.yaml; retain config hashes with build log",
  "elfs": {
    "relative/test.elf": "<actual SHA-256 of that final ELF>"
  }
}
```

The illustrated hash is a format marker, not a valid manifest entry. The runner compares exact
filename sets and hashes: unlisted ELFs, missing listed ELFs, or changed contents are errors.
It recursively discovers every `.elf`, case-insensitively, with no test-name filters. Without
a manifest it can still execute programs, but marks provenance unverified. A matching manifest
checks inventory integrity; it does not independently authenticate the checkout or certify an
architecture. Retain upstream generation/applicability/exclusion information to account for
what the build should have produced, not merely what happened to be in a directory.

Each batch invocation compiles the core harness once and creates a separate timestamped run
directory. It records every input as passed, failed, conversion_failed, or timeout, retains
per-test conversion/simulation logs, hashes inputs/core sources, and writes `results.json` both
in that run directory and at the selected output directory. An empty inventory cannot pass;
conversion errors, missing tools, manifest errors, and failures produce a nonzero exit.

The platform macros store 1 for PASS or 2 for FAIL to the configured mailbox. A mailbox value
of zero does not finish a test. PASS requires an accepted request, successful response, and
matching architectural store retirement. A speculative result or just a bus request cannot
finish the simulation. Cycle and host wall-clock timeouts are separate failure mechanisms.

A single-program `program_reported_pass` or batch `all_input_programs_reported_pass` means those
self-checking inputs reported success on this core. The harness does not compare every retired
instruction with another ISA simulator. Official execution evidence must additionally retain
suite/configuration provenance, complete generated inventory, individual outcomes, and unresolved
build/applicability gaps. Privileged testing, exhaustive formal proof, silicon validation, and
Vivado implementation remain outside these results.
