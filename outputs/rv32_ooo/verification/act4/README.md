# Custom core ACT4 platform configuration

These files target the local processor and `tb/tb_act4.sv`, using upstream
`riscv/riscv-arch-test` commit
`2dc36dd2f3d25383039effa10f1a3930a78a945a`. They are configuration inputs, not
passing-test evidence. The build and per-ELF execution reports establish what
actually ran.

## Initial selection

`test_config.yaml` sets `include_priv_tests: false`. Build with the upstream
extension filter `I,M,Zicsr,Zifencei`. `Zmmul` is also declared because it is
contained in M; it does not add an execution resource or an extra ISA promise.
Do not interpret the UDB `Sm` declaration or `STANDARD_SM_SUPPORTED` macro as
machine-mode certification. They provide the framework's machine CSR/trap
vocabulary and shared trap-handler interface.

The core is RV32, little endian, M-mode only, with no C/U/S/PMP/MMU/cache/FPU.
Instructions start at PC zero. Machine CSRs implemented by `rv32_csr_file.sv`
include the trap/status/interrupt registers, identification registers (zero),
`mstatush` (zero), `mcountinhibit` (writable mask 5), and the machine cycle/retired
counters. User counter aliases and HPM/event-selector CSRs are absent. The custom
boot override avoids writing the absent optional `mhpmevent3..31` registers that
upstream's generic boot initializes. It still installs upstream's actual machine
trap prolog and initializes the implemented machine state.

`mtvec` supports direct and vectored modes with four-byte base alignment. Invalid
MODE writes coerce to direct mode with the newly written base. The UDB marks
that policy `custom`; Sail's generic reserved-mode policy is not an assertion
that every implementation-specific WARL write has been matched. Such privileged
corner-case tests are outside this initial selection.

## RAM and termination

`link.ld` uses one 64 KiB loadable RAM image at address zero. It includes code,
data, BSS, signatures, a stack, and model/trap code in the same physical region.
The fallback stack is 4 KiB; upstream passes a 128-byte override for assembly
tests that do not need a C stack. Explicit assertions reject an entry other than
zero or an oversized image. Padding is kept small; no RAM enlargement or test
omission is implied by these files. If tests overflow, retain every test chunk
and use the upstream generator's supported splitting facilities.

`RVMODEL_HALT_PASS` performs one word store of 1 to `0x1000fff0`.
`RVMODEL_HALT_FAIL` stores 2. The harness accepts termination only after a
successful response and matching architectural store retirement. Keep the
runner's mailbox argument at the same address. Console output is deliberately
empty in the DUT platform hook; it does not suppress test self-checks.

## Reference-only services and pending privilege work

The current ACT framework requires Sail's CLINT and simple interrupt generator
even when the DUT has no matching devices. `sail.json` places those reference
services at `0x40000000` and `0x50000000`, outside DUT RAM/mailbox. They are not
peripherals added to the delivered core or to the DUT harness. Sail replaces
the signature ELF's IO/termination macros with its own RAM-based HTIF implementation.
The DUT-only mailbox is consequently absent from Sail's memory map; its model
requires page-aligned regions, and no signature-generation access uses that word.
The signature-only boot writes an all-ones timer comparison value to disarm
Sail's reference CLINT.

Sail's machine interrupt CSR capability remains enabled to represent the core's
MSIE/MTIE/MEIE fields. The DUT harness ties IRQ inputs low and cannot raise or
clear them through MMIO. UDB's `{MSI,MTI,MEI}_INTR_IMPL` flags describe what the
test platform can raise and are therefore false. Do not enable ACT4 interrupt
tests or report them as covered by this configuration. The project's separate
directed interrupt tests remain separate evidence.

Privileged architectural testing, optional-CSR behavior, full WARL equivalence,
and any certification profile require a reviewed privilege configuration and
matching platform hooks. They remain explicit gaps until corresponding runs
are recorded.

## Preserving the full ISA inventory within 64 KiB

`generate_small_tests.py` invokes the pinned upstream unprivileged generator for
RV32 I/M/Zicsr and changes `testgen.generate.unpriv.TESTCASES_PER_FILE` to 128
inside that Python process. No upstream constant or source file is edited. The
ordinary upstream chunk splitter and writer remain responsible for file
boundaries, pointer restoration, self-checks, and signatures. Zifencei and the
environment are copied from the pinned upstream checkout.

Run it inside the prepared ACT4 environment, with `/act4` as the working directory:

```sh
uv run --frozen --no-dev python /project/verification/act4/generate_small_tests.py
```

The verified generation produced 136 programs representing all 54 original
programs: 13,441 testcase/debug labels, 23,240 operand literal directives, and
12,258 generated chunks. It regenerates the default-size baseline too, compares
every complete chunk before writing, and compares the output's testcase/label
inventory with the original committed sources. The upstream seed policy remains
`seed(reproducible_hash(instruction + coverpoint))`; neither splitting nor this
wrapper substitutes new random operands. Added file boundaries do change
addresses, per-file canaries, and pointer reset code. Expected signatures must
therefore be generated again by the actual reference model.

The largest upstream indivisible chunk contains 256 cases and remains intact.
The 128-case setting is a target used by upstream's splitter, not permission to
break a chunk or omit an oversized program. Linker assertions still enforce the
fixed RAM size. The generator does not compile or execute architectural tests.

The copied environment has three explicit non-C alignment adaptations, changing
only `.option rvc` to `.option norvc`: `rvtest_setup.h:46`,
`rvtest_trap_handler.h:1220`, and `utils.h:434`. These select 32-bit NOP alignment
and prevent incorrect RVC ELF metadata on this processor. Instruction test bodies
and self-checks remain unchanged. The script requires each authorized site to
match exactly once and scans for remaining RVC alignment directives.

Splitting also exposed a pointer-initialization issue in the upstream writer:
a chunk's recorded start register can already describe the destination of its
opening relocation move. Initializing that destination leaves the move's source
uninitialized in a new independent executable. The wrapper corrects only the
per-file initialization prefix, using the preceding chunk's logical signature
and data pointer registers. It passes a shallow metadata copy to the upstream
writer; original chunk instruction lists and self-checks are unchanged. This
corrected exactly three files: `M-div-03.S`, `M-mulhsu-03.S`, and `M-mulhu-03.S`.
The manifest records each old/new prefix and complete file hashes. New expected
signatures must be generated after this adaptation.

The remaining `I-jal-00.S` size gap is explicit. Upstream groups all JAL immediate
edge cases into one chunk; its tested offsets reach only ±8192, but cumulative
alignment padding takes `.text.rvtest` to `0x24000` and the complete signature
image to `0x279c0`, beyond the fixed `0x10000` RAM. The wrapper currently preserves
that chunk intact and the linker rejects it. No offsets were shortened, RAM
enlarged, or test cases quietly removed. Resolving this layout would require an
additional reviewed division into independent case blocks with every original
instruction, label, target distance, and check preserved. Until then it is an
unexecuted architectural-test coverage gap, not a pass or a demonstrated RTL
failure.

`build/act4_small/generation_manifest.json` records original/output file hashes,
before/after hashes and lines for those adaptations, the complete comparison
results, generator-source hashes, the pinned revision, and the wrapper hash.
Only a successful build followed by real DUT runs establishes test results.

## Provenance and license

The configuration parameter vocabulary, compact linker layout, boot-override
pattern, and Sail starting configuration were adapted from:

- [Official SERV ACT4 configuration](https://github.com/riscv/riscv-arch-test/tree/2dc36dd2f3d25383039effa10f1a3930a78a945a/config/cores/serv/serv-rv32imc).
- [Official Wally RV32I configuration](https://github.com/riscv/riscv-arch-test/tree/2dc36dd2f3d25383039effa10f1a3930a78a945a/config/cores/cvw/cvw-rv32i).

Adapted text retains `Copyright (c) 2026, Harvey Mudd College` and
`SPDX-License-Identifier: Apache-2.0`. The accompanying `COPYING.APACHE` preserves
the upstream license. The JSON was derived from the official SERV Sail
configuration at the pinned revision; its comments were removed, unsupported
extensions were disabled, and memory/CSR/platform values were changed for this
core. This is custom configuration work; no upstream RTL was copied.
