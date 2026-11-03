# Block 2: from register ADD to decoder controls and ALU circuits

Two-issue revision: each decoder/ALU remains a per-instruction combinational block. The wider
core will use two decoder instances and two integer ALUs; this lesson's functional tests check
the reusable unit behavior, not simultaneous two-instruction scheduling.

This lesson starts with register ADD, as you requested. The implemented circuits are the
instruction decoder, integer ALU, and conditional-branch comparator. The physical register file,
renamer, scheduler, execution buffers, and retirement controller are later blocks.

## 1. Follow one ADD from its bits to its result

Use this instruction:

```asm
add x5, x3, x4
```

Its machine word is `0x004182b3`. The decoder divides it into fields:

| Bits | Value | Meaning |
|---|---|---|
| [6:0] | 0110011 | Register arithmetic instruction family |
| [11:7] | 00101 | Destination x5 |
| [14:12] | 000 | ADD/SUB operation selector within this family |
| [19:15] | 00011 | Source x3 |
| [24:20] | 00100 | Source x4 |
| [31:25] | 0000000 | Select ADD, distinguishing it from SUB and MUL |

The decoded record says: use two integer-register sources, write a nonzero destination, select
`UOP_INTEGER`, and select `ALU_ADD`. The decoder passes through the instruction PC, prediction,
and any existing fetch exception.

```text
IF fetch packet
  instruction = 0x004182b3, PC, prediction, fault information
            |
            v
ID: rv32_decoder
  rs1=x3, rs2=x4, rd=x5
  uses_rs1=1, uses_rs2=1, writes_rd=1, alu_op=ALU_ADD
            |
            v
ID: future renamer captures physical source mappings and allocates a destination
  for example prs1=p12, prs2=p20, pdst=p37
            |
            v
EX: future scheduler and register-read block wait for p12/p20, then read VALUES
  for example a=12, b=7
            |
            v
EX: rv32_alu -> result=19
            |
            v
Future completion writes p37; future ROB retirement commits the mapping for x5
```

The physical tags above are illustrative, not allocated by this block. The testbench drives
example operand values directly. Adding the register numbers 3 and 4 would be wrong: the ALU
adds the contents selected by the source mappings, not the register identifiers.

## 2. Modules, inputs, and outputs

| Module / port | Width/type | Comes from | Goes to | Meaning |
|---|---|---|---|---|
| decoder `fetch_i` | `fetch_packet_t` | Instruction-fetch buffer | Decoder | Instruction plus PC, prediction, and fetch fault |
| decoder `decoded_o` | `decoded_uop_t` | Decoder | Rename/dispatch | Source names, use flags, operation selectors, immediate, and metadata |
| ALU `a_i`, `b_i` | 32 bits each | Future operand-selection path | ALU | Actual selected operand values |
| ALU `op_i` | `alu_op_e` | Decoded controls carried through the scheduler | ALU | Arithmetic, logical, comparison, or shift selection |
| ALU `result_o` | 32 bits | ALU | Future execution-result buffer | Computed value |
| comparator `rs1_i`, `rs2_i` | 32 bits each | Future register read/bypass | Branch comparator | Original register operand values |
| comparator `op_i` | `branch_op_e` | Decoded branch selector | Branch comparator | Which conditional comparison to perform |
| comparator `taken_o` | 1 bit | Comparator | Future control-flow unit | Whether the specified condition is true |

All three modules are combinational. They have no clock, reset, storage, or handshake of their
own. Their outputs settle after their inputs change; later pipeline/queue registers capture the
outputs on clock edges. The owning interface supplies validity. A decoder may interpret unused
bits while the fetch queue is empty, but those results must not be dispatched without a valid input.

## 3. Why full encoding checks are necessary

ADD, SUB, and MUL share the same main opcode and funct3, but use different funct7 values.
Checking only the main opcode would silently turn instructions into the wrong operation.

The decoder validates:

- R-type funct7/funct3 combinations for the ten base integer operations and eight M operations.
- The upper shift-immediate bits: RV32 SLLI/SRLI/SRAI have only a five-bit shift count.
- Legal byte/halfword/word load/store forms. RV64 LD/SD are not accepted.
- Six supported conditional-branch conditions and the required JALR funct3.
- The six CSR instruction forms and exact ECALL/EBREAK/MRET/WFI encodings.

An unsupported encoding produces an illegal-instruction exception containing the original
instruction in `tval`, with its PC preserved. It disables source-use and destination-write
controls and emits `UOP_NONE`; a future fault path will carry the exception to the ROB.

An existing fetch exception takes precedence over decoding. The instruction bits returned with
a fetch error might not be meaningful, so they must not replace an access fault with an illegal
instruction fault.

A valid fault record must still be admitted to the ROB for precise trapping. `UOP_NONE` means
do not issue an execution engine in this case; it does not mean discard the instruction as an
empty bubble. Validity comes from the owning fetch/dispatch interface.

CSR instruction encoding is checked here. Whether the addressed CSR exists, is writable, or
requires a suppressed read/write is checked in the later serialized CSR controller at ROB head.
Recognizing ECALL here similarly does not implement trap entry yet.

FENCE and FENCE.I deliberately ignore the fields that their specifications reserve for future
use. Their selectors will request conservative serialization in the later system controller.

## 4. Source-use flags prevent false dependencies

For register ADD, both `uses_rs1` and `uses_rs2` are set. For ADDI, only `uses_rs1` is set:

```asm
addi x5, x3, 4
```

The bits that resemble rs2 are part of the immediate 4. Waiting for x4 would create a false
dependency. The decoder still extracts the raw fields, but source-use flags decide whether
rename/scheduling should interpret them as actual register operands.

LUI and AUIPC use no integer-register sources. CSR immediate forms use a five-bit immediate,
not an integer-register value. Stores need both a base register and store-data register even
though the ALU's second operand is the address offset.

`writes_rd` is cleared for rd=x0. This must not remove the operation itself: a load to x0 still
performs its memory access and can fault, and a CSR operation with rd=x0 can still modify a CSR.
Canonical NOP is a legal ADDI to x0; the pipeline must not mistake every rd=x0 instruction for NOP.

## 5. Immediate generation and operand selection

| Format | Construction/use |
|---|---|
| I | Sign-extend bits [31:20] for arithmetic, loads, and JALR |
| S | Join [31:25] and [11:7], then sign-extend for stores |
| B | Reassemble the scattered signed PC offset, including its implicit zero low bit |
| U | Take [31:12] and append 12 zero bits for LUI/AUIPC |
| J | Reassemble the scattered signed JAL offset, including its implicit zero low bit |
| Shift immediate | Validate the encoding, then carry the zero-extended five-bit shift count |
| CSR immediate | Carry the zero-extended five-bit source field separately |

The decoder supplies `src_a` and `src_b` selectors. The future execution wrapper selects:

```text
src_a: register value / zero / instruction PC
src_b: register value / decoded immediate
```

The testbench includes this simple selection for demonstration; the integrated execution wrapper
does not exist yet. For SLTIU the immediate is still sign-extended, and then compared as an
unsigned 32-bit value. For example, an immediate of -1 becomes 0xffffffff.

The branch comparator uses the two original register values. Branch-target generation is a
separate future PC-plus-offset path; branch decode does not select that target through the
ordinary ALU operand muxes. JAL/JALR link writeback is PC+4, not the computed target. JALR target
bit-zero clearing and taken-target alignment faults also belong to the later execution wrapper.

## 6. What the ALU describes at circuit level

The `case` statement chooses a result. Synthesis may share or optimize arithmetic hardware;
the exact mapped FPGA cells must be checked in a later Vivado report.

- **ADD/SUB:** 32-bit arithmetic with the low 32 result bits retained. Integer overflow wraps;
  it does not raise a base RISC-V arithmetic exception.
- **AND/OR/XOR:** bitwise gates operating independently on the 32 bit positions.
- **SLT/SLTU:** a comparator followed by a 32-bit result containing only zero or one.
- **SLL/SRL/SRA:** a variable-shift datapath. Conceptually it can use mux layers selecting
  shifts of 1, 2, 4, 8, and 16 positions. Only operand B's low five bits select the amount.
- **COPY_B:** route the second operand through, used by LUI.

The vectors themselves are unsigned `logic` values. `$signed` tells comparison or arithmetic
right shift to interpret their bits as two's-complement values. It does not change their width.

For `a=0x80000000, b=1`:

| Operation | Result | Reason |
|---|---|---|
| SRL | 0x40000000 | Vacated high bit becomes zero |
| SRA | 0xc0000000 | Vacated high bit copies the original sign |
| SLT | 1 | Signed minimum is less than positive one |
| SLTU | 0 | Unsigned 0x80000000 is greater than one |

## 7. Branch conditions do not reuse an arbitrary ALU zero flag

The comparator has explicit selectors for EQ, NE, LT, GE, LTU, and GEU. NONE returns false.
Signed conditions use signed interpretation; unsigned conditions compare the raw nonnegative
bit-vector magnitudes. Two equal register values make both GE and GEU true, but LT and LTU false.

The comparator does not update PC, consult the BTB, or decide whether an instruction is live.
The future execution/recovery logic combines those controls. An ADD producing zero cannot
accidentally redirect fetch merely because the comparator exists.

## 8. Timing and verification scope

These are combinational blocks. The testbench changes inputs, allows a simulation-time step for
settling, and checks outputs. That step is not a measured FPGA propagation delay and is not an
extra pipeline cycle.

For the ADD example, the measured block-level path is:

1. Supply instruction 0x004182b3 in the fetch record.
2. Observe x3/x4/x5 fields and ADD controls at the decoder output.
3. Supply example source values 12 and 7 through testbench operand selection.
4. Observe result 19 from the ALU.

There is no register write or retirement in this isolated test. Blocks 7–15 add actual
multicycle instruction traces through the scheduler, execution buffers, physical registers,
and ROB; block 15 tests the assembled core.

Run `python3 scripts/run_checks.py --block 2`, or `--block all` for all 15 regression groups.
The self-checking testbench covers all 1,024 R-type funct7/funct3 combinations, all RV32 shift
upper-field combinations, every signed 12-bit I/S immediate, B/J boundary offsets, load/store
widths, CSR/system/fence forms, source-use behavior, ALU edge/random cases, all shift counts
from 0 through 95, branch conditions, and fetch-fault priority.
Exact counts and source hashes are saved in `reports/block02_results.json`.

This block is combinational: there is no clock edge, pipeline register, or default VCD dump.
The runner does not enable `--waves` recording for blocks 1 or 2. The testbench changes inputs,
waits `#1` for the combinational paths to settle, then compares outputs. This delay separates
test observations; it is not a measured one-nanosecond hardware timing result.

To inspect the paths, open the compiled test in a simulator waveform viewer, add the signals
before running, and zoom to an input change followed by the settled output. Follow
`fetch.instruction` into `decoded.rs1`, `decoded.rs2`, `decoded.rd`, source-use flags,
`decoded.alu_op`, and `decoded.immediate`. For the ADD example, instruction `0x004182b3`
selects x3/x4/x5 and register operands; with `a=12` and `b=7`, `selected_a/selected_b` are
12/7 and `decoded_result` settles to 19. Separately follow `branch_op`, `a/b`, and `taken`
to distinguish signed comparisons from unsigned ones. An illegal encoding should set the
decoded exception rather than produce executable controls.

For a VCD, a separate scratch wrapper can instantiate the same modules and add `$dumpfile`
and `$dumpvars` before driving inputs, or the simulator can export its recorded signals.
No functional RTL change is needed. Only interpret settled values after the input update;
transient simulator delta-cycle ordering is not an extra processor stage or instruction event.

Icarus may print a diagnostic that constant selects in `always_comb` are treated as sensitive
to the whole vector. This broadens simulation sensitivity; it is not a compile failure. The
complete diagnostic is retained in the log. Vivado compilation and synthesis are still unverified.

For XSim on Linux, after sourcing Vivado's settings file:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_decoder.sv rtl/rv32_alu.sv rtl/rv32_branch_compare.sv tb/tb_block02.sv
xelab tb_block02 -s block02_sim
xsim block02_sim --runall
```

The source files can be added as SystemVerilog in a GUI project instead; compile the package
before the modules and testbench. Complete source listings follow this lesson.

## 9. Interview questions and model answers

**Why are opcode, funct3, and funct7 all needed?**
The opcode selects an instruction family. Other fields distinguish operations within that
family, such as ADD, SUB, and MUL. Ignoring them can execute a valid instruction incorrectly.

**Where do the physical tags come from?**
The future renamer maps architectural source names to physical storage identifiers and allocates
a new destination. The decoder only identifies the architectural fields and operation.

**Does the ALU receive x3 or the contents of x3?**
It receives operand values after register read and any required forwarding. Register names and
physical tags are identifiers used to locate those values.

**Why does a store read rs2 if its ALU operand B is an immediate?**
The immediate is the address offset. The rs2 value is separate data to write to that address.

**How are negative immediates represented?**
The immediate generator sign-extends them to 32 bits. Signedness of a later comparison is a
separate decision; SLTIU still uses a sign-extended immediate.

**Why is an arithmetic right shift different from a logical right shift?**
It fills vacated high bits with the sign bit, preserving two's-complement sign behavior.

**Does branch decoding mean branch prediction is implemented?**
No. This block identifies and compares branch conditions. Prediction, target selection, and
pipeline recovery are subsequent blocks.

**Why not perform CSR updates immediately in the decoder?**
In an out-of-order core the instruction might be speculative. CSR changes must wait until
the serialized architectural boundary, with permissions and exceptions checked there.

## 10. References

- [RISC-V RV32I specification](https://docs.riscv.org/reference/isa/v20260120/unpriv/rv32.html)
- [RISC-V M extension](https://docs.riscv.org/reference/isa/v20260120/unpriv/m-st-ext.html)
- [RISC-V Zicsr specification](https://docs.riscv.org/reference/isa/v20260120/unpriv/zicsr.html)
- [Official ISA snapshot, including Zifencei](https://riscv.github.io/riscv-isa-manual/snapshot/spec/)
- [lowRISC coding style](https://github.com/lowRISC/style-guides/blob/master/VerilogCodingStyle.md)

The three RTL modules are original project source. References establish semantics and style;
this lesson does not claim implementation of later architectural blocks.
