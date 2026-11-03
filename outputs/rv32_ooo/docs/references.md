# References and attribution

The RTL in `rtl/` is a custom implementation written for this learning project.
No third-party processor RTL is embedded in these modules. The following
primary references informed architecture terminology, ISA behavior, and style:

- [RISC-V ISA specifications](https://docs.riscv.org/reference/isa/): normative
  integer, multiply/divide, CSR, fence, exception, and interrupt behavior.
- [BOOM pipeline](https://docs.boom-core.org/en/latest/sections/intro-overview/boom-pipeline.html),
  [renaming](https://docs.boom-core.org/en/latest/sections/rename-stage.html), and
  [ROB](https://docs.boom-core.org/en/latest/sections/reorder-buffer.html): clear
  separation of physical completion, architectural commitment, and recovery.
- [lowRISC SystemVerilog guide](https://github.com/lowRISC/style-guides/blob/master/VerilogCodingStyle.md):
  descriptive names, combinational/sequential separation, and intent-oriented
  comments. This project uses compact paired records for its two-wide paths.
- [ultraembedded/riscv](https://github.com/ultraembedded/riscv): reference processor
  organization; its code is not copied into this processor.
- [lowRISC Ibex counter](https://github.com/lowRISC/ibex/blob/master/rtl/ibex_counter.sv):
  counter-write priority review reference; the custom counter fix was independently
  implemented and regression-tested here.
- [RISC-V architectural tests](https://github.com/riscv/riscv-arch-test): official
  ACT4 framework, generated assembly, and reference-model signature flow.

ACT4 configuration text adapted from upstream retains its copyright and
Apache-2.0 notices under `verification/act4/`, with a full license copy there.
Any retained generated official test artifacts preserve their upstream
provenance and applicable license notices. These references do not imply that
this custom core is a BOOM, Ibex, lowRISC, or upstream-certified design.

The project reports distinguish local directed/random tests, test-harness
infrastructure checks, official architectural ELF execution, and unrun FPGA
implementation. A reference link is not a passing verification result.
