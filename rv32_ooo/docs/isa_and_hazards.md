# Instruction and hazard reference

The architecture is RISC-V RV32IM_Zicsr_Zifencei. The register width is 32 bits;
all supported instructions occupy four bytes. There are no MIPS encodings or
delay slots. The decoder validates opcode, funct3, and applicable funct7 fields
before selecting hardware. Unsupported encodings become precise illegal
instruction exceptions instead of accidentally executing an ALU operation.

| Family | Implemented instructions | Circuit/use |
|---|---|---|
| Register arithmetic | ADD, SUB, SLT, SLTU | Add/subtract and signed/unsigned comparisons |
| Immediate arithmetic | ADDI, SLTI, SLTIU | Sign-extended immediate selects ALU operand B |
| Register logic | XOR, OR, AND | Per-bit logical gates |
| Immediate logic | XORI, ORI, ANDI | Same logic after immediate selection |
| Shifts | SLL, SRL, SRA, SLLI, SRLI, SRAI | Low five shift-amount bits; arithmetic right shift copies the sign |
| Upper immediates | LUI, AUIPC | Copy upper immediate, or add it to the instruction's own PC |
| Loads | LB, LH, LW, LBU, LHU | Byte/halfword/word selection; signed or zero extension |
| Stores | SB, SH, SW | Lane strobes and shifted write data |
| Conditional control | BEQ, BNE, BLT, BGE, BLTU, BGEU | Comparison selects fall-through or PC-relative target |
| Jumps | JAL, JALR | Link value PC+4; JALR clears target bit zero |
| Multiply | MUL, MULH, MULHSU, MULHU | Iterative product; low half or appropriately signed high half |
| Divide | DIV, DIVU, REM, REMU | Iterative quotient/remainder, including specified zero/overflow cases |
| CSR register forms | CSRRW, CSRRS, CSRRC | Read old CSR, prepare replacement/set/clear intent |
| CSR immediate forms | CSRRWI, CSRRSI, CSRRCI | Zero-extended five-bit immediate replaces the register operand |
| Ordering | FENCE, FENCE.I | Serialize ordering; FENCE.I also discards prefetched code and prediction |
| System | ECALL, EBREAK, MRET, WFI | Precise trap, return, and interrupt wait |

Signed interpretation changes comparisons, arithmetic shifts, high products,
division, and remainder. The stored register bits are the same in signed and
unsigned operations. Addition/subtraction wrap to 32 bits and do not trap on
arithmetic overflow. DIV by zero returns all ones; REM by zero returns its
dividend. Signed minimum divided by -1 returns that minimum with remainder zero.
These are ISA rules, not accidental behaviors of the HDL simulator.

## Names in the original request

Some requested operations are assembly aliases or software structures rather
than additional RISC-V instruction encodings:

| Requested name | RV32 expression |
|---|---|
| SUBI | ADDI with a negated constant when it fits the signed 12-bit immediate; otherwise materialize the constant and SUB |
| BEQZ | BEQ rs, x0, target |
| BNEQZ / BNEZ | BNE rs, x0, target |
| JMP / J | JAL x0, target; indirect jump uses JALR x0, rs, offset |
| NOT | XORI rd, rs, -1 |
| MOVE / MV | ADDI rd, rs, 0 |
| NOP | ADDI x0, x0, 0 |
| Return / RET | JALR x0, x1, 0 under the standard calling convention |
| If/else | A comparison/conditional branch and, where needed, a jump around the other arm |
| Loop | Body instructions, induction-variable update, and a backward conditional branch |
| LD / SD | RV64 doubleword instructions, absent from this RV32 core; use LW/SW or two words for 64-bit data |

For example, a counted loop needs no dedicated LOOP decoder:

```asm
loop:
    add  x10, x10, x11
    addi x12, x12, -1
    bne  x12, x0, loop
```

The predictor learns this branch's direction and target. A bimodal two-bit
counter does not know the exact loop count in advance; the final exit may still
be mispredicted. A dedicated loop predictor is outside this version.

## What happens to hazards

| Hazard | Example | Implemented handling |
|---|---|---|
| RAW, true dependency | ADD writes x5; SUB reads x5 | Rename gives the consumer the producer's physical tag. Readiness and completion wakeup delay issue until the value is available. PRF bypass supplies a same-cycle accepted result. |
| WAR, false dependency | Older ADD reads x5; younger SUB writes x5 | The older reader retains the old physical mapping. The younger writer gets a different physical destination. |
| WAW, false dependency | DIV and younger ADD both write x5 | Each gets its own physical destination. In-order retirement updates the committed map in program order. |
| Control | Predicted branch target/direction differs from execution | Record the mismatch, stop new admission, retire older work, then cancel younger work and restore committed mappings before redirect. |
| Structural | Three units finish but only two writeback slots exist | Arbitration selects available outputs; other producers hold valid results. Resource capacity also stalls dispatch or issue. |
| Memory ordering | Younger load could pass an older store | Addresses may be prepared early, but actual requests issue only at the ROB head. One transaction is outstanding; stores await their response. |

A stall preserves complete instruction state. A canceled instruction loses its
valid ownership and cannot retire, update a CSR, train prediction, or perform a
new memory transaction. Generation fields reject delayed completions; the
cancellation/quiescence barrier prevents unsafe reuse even after generation
wraparound. Tags alone are not a replacement for draining retained results.

The four PRF reads supply two pairs of operands for two issue ports. Two integer
instructions can execute together. A multiply/divide operation occupies its
independent iterative engine, allowing later independent integer work to
continue. Single-issue can also benefit from independent multicycle engines;
the approved two-issue version additionally permits two admissions/issues in
one cycle when its resource and dependency rules allow them.

## Reading completion and retirement correctly

Consider an older DIV and a younger independent ADD. The ADD may produce a value
and write a speculative physical register before DIV finishes. A later consumer
can use that accepted value. The committed rename map still represents the last
retired instructions. The ROB prevents ADD from retiring before DIV.

This separation permits useful work during long latency while retaining precise
exceptions. An older fault discards the younger ADD's speculative effects. A
store cannot escape this rule because its bus request is restricted to the ROB
head and its response is required before retirement.

## Basic limits and interview answers

- **Is CPI always one?** No. The two-wide paths can sustain up to two operations
  per cycle under favorable conditions, but memory, DIV, dependencies, redirects,
  and serialization add delay. Five logical sections do not fix instruction
  latency at five cycles.
- **Does renaming remove RAW?** No. It removes naming conflicts, WAR and WAW.
  The consumer still needs its actual producer's value.
- **Why train prediction at retirement?** It keeps squashed instructions from
  changing the predictor. Multiple retirement updates are applied oldest first.
- **Why is memory conservative?** Head-only requests avoid speculative MMIO and
  memory replay logic. This reduces memory-level parallelism and limits throughput.
- **Does the test count prove all interleavings?** No. The reports establish the
  enumerated directed/random test results. Formal proof and FPGA implementation
  are separate work, and architectural tests are reported separately.

The normative instruction semantics are in the
[RISC-V ISA specifications](https://docs.riscv.org/reference/isa/). The complete
decoder/ALU source and its encoding tests are in block 2; blocks 3–15 show the
actual implementation of the dataflow and ownership rules above.
