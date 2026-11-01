# Block 2: from register ADD to decoder controls and ALU circuits

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

There is no register write or retirement in this test. Actual multicycle instruction traces
will be added when the scheduler, execution buffers, physical registers, and ROB exist.

Run `python3 scripts/run_checks.py --block 2`, or `--block all` for both implemented blocks.
The self-checking testbench covers all 1,024 R-type funct7/funct3 combinations, all RV32 shift
upper-field combinations, every signed 12-bit I/S immediate, B/J boundary offsets, load/store
widths, CSR/system/fence forms, source-use behavior, ALU edge/random cases, all shift counts
from 0 through 95, branch conditions, and fetch-fault priority.
Exact counts and source hashes are saved in `reports/block02_results.json`.

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

## Complete instruction decoder

Source: `rtl/rv32_decoder.sv`.

```systemverilog
// ID section: turn a fetched instruction into controls for rename and execution.
// This block does not read register values, allocate tags, or change architectural state.
`timescale 1ns / 1ps

module rv32_decoder (
  input  rv32_ooo_pkg::fetch_packet_t fetch_i,
  output rv32_ooo_pkg::decoded_uop_t decoded_o
);
  import rv32_ooo_pkg::*;

  logic [31:0] instruction;
  logic [6:0] opcode;
  logic [2:0] funct3;
  logic [6:0] funct7;
  logic legal_encoding;

  assign instruction = fetch_i.instruction;
  assign opcode = instruction[6:0];
  assign funct3 = instruction[14:12];
  assign funct7 = instruction[31:25];

  always_comb begin
    decoded_o = '0;
    decoded_o.fetch = fetch_i;
    decoded_o.rs1 = instruction[19:15];
    decoded_o.rs2 = instruction[24:20];
    decoded_o.rd = instruction[11:7];
    decoded_o.alu_op = ALU_ADD;
    decoded_o.src_a = SRC_A_REGISTER;
    decoded_o.src_b = SRC_B_REGISTER;
    decoded_o.mem_size = MEM_WORD;
    legal_encoding = 1'b1;

    case (opcode)
      7'b0110011: begin // Register arithmetic: both source fields are real registers.
        decoded_o.op_class = UOP_INTEGER;
        decoded_o.uses_rs1 = 1'b1;
        decoded_o.uses_rs2 = 1'b1;
        decoded_o.writes_rd = 1'b1;
        case (funct7)
          7'b0000000: begin
            case (funct3)
              3'b000: decoded_o.alu_op = ALU_ADD;
              3'b001: decoded_o.alu_op = ALU_SLL;
              3'b010: decoded_o.alu_op = ALU_SLT;
              3'b011: decoded_o.alu_op = ALU_SLTU;
              3'b100: decoded_o.alu_op = ALU_XOR;
              3'b101: decoded_o.alu_op = ALU_SRL;
              3'b110: decoded_o.alu_op = ALU_OR;
              3'b111: decoded_o.alu_op = ALU_AND;
              default: legal_encoding = 1'b0;
            endcase
          end
          7'b0100000: begin
            case (funct3)
              3'b000: decoded_o.alu_op = ALU_SUB;
              3'b101: decoded_o.alu_op = ALU_SRA;
              default: legal_encoding = 1'b0;
            endcase
          end
          7'b0000001: begin // M-extension execution belongs to the separate mul/div engine.
            decoded_o.op_class = UOP_MULDIV;
            case (funct3)
              3'b000: decoded_o.muldiv_op = MD_MUL;
              3'b001: decoded_o.muldiv_op = MD_MULH;
              3'b010: decoded_o.muldiv_op = MD_MULHSU;
              3'b011: decoded_o.muldiv_op = MD_MULHU;
              3'b100: decoded_o.muldiv_op = MD_DIV;
              3'b101: decoded_o.muldiv_op = MD_DIVU;
              3'b110: decoded_o.muldiv_op = MD_REM;
              3'b111: decoded_o.muldiv_op = MD_REMU;
              default: legal_encoding = 1'b0;
            endcase
          end
          default: legal_encoding = 1'b0;
        endcase
      end

      7'b0010011: begin // Immediate arithmetic: bits [24:20] are not an rs2 source.
        decoded_o.op_class = UOP_INTEGER;
        decoded_o.uses_rs1 = 1'b1;
        decoded_o.writes_rd = 1'b1;
        decoded_o.src_b = SRC_B_IMMEDIATE;
        decoded_o.immediate = {{20{instruction[31]}}, instruction[31:20]};
        case (funct3)
          3'b000: decoded_o.alu_op = ALU_ADD;
          3'b010: decoded_o.alu_op = ALU_SLT;
          3'b011: decoded_o.alu_op = ALU_SLTU;
          3'b100: decoded_o.alu_op = ALU_XOR;
          3'b110: decoded_o.alu_op = ALU_OR;
          3'b111: decoded_o.alu_op = ALU_AND;
          3'b001: begin
            decoded_o.alu_op = ALU_SLL;
            decoded_o.immediate = {27'b0, instruction[24:20]};
            if (funct7 != 7'b0000000) legal_encoding = 1'b0;
          end
          3'b101: begin
            decoded_o.immediate = {27'b0, instruction[24:20]};
            case (funct7)
              7'b0000000: decoded_o.alu_op = ALU_SRL;
              7'b0100000: decoded_o.alu_op = ALU_SRA;
              default: legal_encoding = 1'b0;
            endcase
          end
          default: legal_encoding = 1'b0;
        endcase
      end

      7'b0110111: begin // LUI: copy the upper immediate; no register source exists.
        decoded_o.op_class = UOP_INTEGER;
        decoded_o.writes_rd = 1'b1;
        decoded_o.src_a = SRC_A_ZERO;
        decoded_o.src_b = SRC_B_IMMEDIATE;
        decoded_o.alu_op = ALU_COPY_B;
        decoded_o.immediate = {instruction[31:12], 12'b0};
      end

      7'b0010111: begin // AUIPC: PC is the first ALU operand.
        decoded_o.op_class = UOP_INTEGER;
        decoded_o.writes_rd = 1'b1;
        decoded_o.src_a = SRC_A_PC;
        decoded_o.src_b = SRC_B_IMMEDIATE;
        decoded_o.immediate = {instruction[31:12], 12'b0};
      end

      7'b0000011: begin // Load size/extension are carried to the LSU.
        decoded_o.op_class = UOP_LOAD;
        decoded_o.uses_rs1 = 1'b1;
        decoded_o.writes_rd = 1'b1;
        decoded_o.src_b = SRC_B_IMMEDIATE;
        decoded_o.immediate = {{20{instruction[31]}}, instruction[31:20]};
        case (funct3)
          3'b000: decoded_o.mem_size = MEM_BYTE;
          3'b001: decoded_o.mem_size = MEM_HALF;
          3'b010: decoded_o.mem_size = MEM_WORD;
          3'b100: begin
            decoded_o.mem_size = MEM_BYTE;
            decoded_o.load_unsigned = 1'b1;
          end
          3'b101: begin
            decoded_o.mem_size = MEM_HALF;
            decoded_o.load_unsigned = 1'b1;
          end
          default: legal_encoding = 1'b0;
        endcase
      end

      7'b0100011: begin // Store has a base source and a separate store-data source.
        decoded_o.op_class = UOP_STORE;
        decoded_o.uses_rs1 = 1'b1;
        decoded_o.uses_rs2 = 1'b1;
        decoded_o.src_b = SRC_B_IMMEDIATE;
        decoded_o.immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        case (funct3)
          3'b000: decoded_o.mem_size = MEM_BYTE;
          3'b001: decoded_o.mem_size = MEM_HALF;
          3'b010: decoded_o.mem_size = MEM_WORD;
          default: legal_encoding = 1'b0;
        endcase
      end

      7'b1100011: begin // Comparison uses both registers; the immediate is a PC offset.
        decoded_o.op_class = UOP_BRANCH;
        decoded_o.uses_rs1 = 1'b1;
        decoded_o.uses_rs2 = 1'b1;
        decoded_o.immediate = {{19{instruction[31]}}, instruction[31], instruction[7],
                               instruction[30:25], instruction[11:8], 1'b0};
        case (funct3)
          3'b000: decoded_o.branch_op = BRANCH_EQ;
          3'b001: decoded_o.branch_op = BRANCH_NE;
          3'b100: decoded_o.branch_op = BRANCH_LT;
          3'b101: decoded_o.branch_op = BRANCH_GE;
          3'b110: decoded_o.branch_op = BRANCH_LTU;
          3'b111: decoded_o.branch_op = BRANCH_GEU;
          default: legal_encoding = 1'b0;
        endcase
      end

      7'b1101111: begin // JAL target is PC+offset; its link value is PC+4, generated later.
        decoded_o.op_class = UOP_JUMP;
        decoded_o.writes_rd = 1'b1;
        decoded_o.src_a = SRC_A_PC;
        decoded_o.src_b = SRC_B_IMMEDIATE;
        decoded_o.immediate = {{11{instruction[31]}}, instruction[31], instruction[19:12],
                               instruction[20], instruction[30:21], 1'b0};
      end

      7'b1100111: begin // JALR's execute path clears target bit zero after addition.
        decoded_o.op_class = UOP_JUMP;
        decoded_o.writes_rd = 1'b1;
        decoded_o.uses_rs1 = 1'b1;
        decoded_o.jump_indirect = 1'b1;
        decoded_o.src_b = SRC_B_IMMEDIATE;
        decoded_o.immediate = {{20{instruction[31]}}, instruction[31:20]};
        if (funct3 != 3'b000) legal_encoding = 1'b0;
      end

      7'b0001111: begin
        decoded_o.op_class = UOP_SYSTEM;
        // Reserved fields are ignored as specified; all FENCE forms use a full drain.
        case (funct3)
          3'b000: decoded_o.system_op = SYS_FENCE;
          3'b001: decoded_o.system_op = SYS_FENCE_I;
          default: legal_encoding = 1'b0;
        endcase
      end

      7'b1110011: begin
        decoded_o.op_class = UOP_SYSTEM;
        if (funct3 == 3'b000) begin
          case (instruction)
            32'h0000_0073: decoded_o.system_op = SYS_ECALL;
            32'h0010_0073: decoded_o.system_op = SYS_EBREAK;
            32'h3020_0073: decoded_o.system_op = SYS_MRET;
            32'h1050_0073: decoded_o.system_op = SYS_WFI;
            default: legal_encoding = 1'b0;
          endcase
        end else begin
          decoded_o.system_op = SYS_CSR;
          decoded_o.writes_rd = 1'b1;
          decoded_o.csr_address = instruction[31:20];
          decoded_o.csr_immediate = instruction[19:15];
          case (funct3)
            3'b001: begin decoded_o.csr_op = CSR_RW;  decoded_o.uses_rs1 = 1'b1; end
            3'b010: begin decoded_o.csr_op = CSR_RS;  decoded_o.uses_rs1 = 1'b1; end
            3'b011: begin decoded_o.csr_op = CSR_RC;  decoded_o.uses_rs1 = 1'b1; end
            3'b101: decoded_o.csr_op = CSR_RWI;
            3'b110: decoded_o.csr_op = CSR_RSI;
            3'b111: decoded_o.csr_op = CSR_RCI;
            default: legal_encoding = 1'b0;
          endcase
          // CSR existence, permissions, and read/write suppression are checked at ROB head.
        end
      end
      default: legal_encoding = 1'b0;
    endcase

    // A load or CSR operation with rd=x0 still executes; only its GPR result is discarded.
    if (instruction[11:7] == 0) decoded_o.writes_rd = 1'b0;

    // Fetch faults take precedence over interpretation of potentially meaningless data bits.
    if (!legal_encoding && !fetch_i.exception.valid) begin
      decoded_o.fetch.exception.valid = 1'b1;
      decoded_o.fetch.exception.cause = CAUSE_ILLEGAL_INSTR;
      decoded_o.fetch.exception.tval = instruction;
    end
    if (!legal_encoding || fetch_i.exception.valid) begin
      decoded_o.op_class = UOP_NONE;
      decoded_o.uses_rs1 = 1'b0;
      decoded_o.uses_rs2 = 1'b0;
      decoded_o.writes_rd = 1'b0;
      decoded_o.system_op = SYS_NONE;
      decoded_o.csr_op = CSR_NONE;
    end
  end
endmodule
```

## Complete integer ALU

Source: `rtl/rv32_alu.sv`.

```systemverilog
// EX section: combinational integer datapath. Operand selection happens outside this block.
// Values are raw 32-bit vectors; signed interpretation is selected only where required.
`timescale 1ns / 1ps

module rv32_alu (
  input  rv32_ooo_pkg::word_t   a_i,
  input  rv32_ooo_pkg::word_t   b_i,
  input  rv32_ooo_pkg::alu_op_e op_i,
  output rv32_ooo_pkg::word_t   result_o
);
  import rv32_ooo_pkg::*;

  always_comb begin
    case (op_i)
      ALU_ADD:    result_o = a_i + b_i;
      ALU_SUB:    result_o = a_i - b_i;
      ALU_SLL:    result_o = a_i << b_i[4:0];
      ALU_SLT:    result_o = {31'b0, ($signed(a_i) < $signed(b_i))};
      ALU_SLTU:   result_o = {31'b0, (a_i < b_i)};
      ALU_XOR:    result_o = a_i ^ b_i;
      ALU_SRL:    result_o = a_i >> b_i[4:0];
      ALU_SRA:    result_o = $signed(a_i) >>> b_i[4:0];
      ALU_OR:     result_o = a_i | b_i;
      ALU_AND:    result_o = a_i & b_i;
      ALU_COPY_B: result_o = b_i;
      default:    result_o = '0; // Decoder prevents unsupported operations from issuing.
    endcase
  end
endmodule
```

## Complete branch comparator

Source: `rtl/rv32_branch_compare.sv`.

```systemverilog
// EX section: evaluate branch direction without depending on an ALU zero flag.
// Target generation, prediction comparison, and redirects are separate execution controls.
`timescale 1ns / 1ps

module rv32_branch_compare (
  input  rv32_ooo_pkg::word_t      rs1_i,
  input  rv32_ooo_pkg::word_t      rs2_i,
  input  rv32_ooo_pkg::branch_op_e op_i,
  output logic                    taken_o
);
  import rv32_ooo_pkg::*;

  always_comb begin
    case (op_i)
      BRANCH_EQ:  taken_o = rs1_i == rs2_i;
      BRANCH_NE:  taken_o = rs1_i != rs2_i;
      BRANCH_LT:  taken_o = $signed(rs1_i) < $signed(rs2_i);
      BRANCH_GE:  taken_o = $signed(rs1_i) >= $signed(rs2_i);
      BRANCH_LTU: taken_o = rs1_i < rs2_i;
      BRANCH_GEU: taken_o = rs1_i >= rs2_i;
      default:    taken_o = 1'b0;
    endcase
  end
endmodule
```

## Complete self-checking testbench

Source: `tb/tb_block02.sv`.

```systemverilog
// Block 2 tests encoding legality and combinational datapaths, not rename or retirement.
`timescale 1ns / 1ps

module tb_block02;
  import rv32_ooo_pkg::*;

  fetch_packet_t fetch;
  decoded_uop_t decoded;
  word_t a;
  word_t b;
  alu_op_e alu_op;
  word_t result;
  branch_op_e branch_op;
  logic taken;
  word_t selected_a;
  word_t selected_b;
  word_t decoded_result;
  integer checks = 0;
  integer failures = 0;
  integer f7;
  integer f3;
  integer imm;
  integer n;
  integer k;
  integer op_number;
  integer legal_r_count = 0;
  logic expected_legal;
  alu_op_e expected_alu;
  word_t expected_result;
  logic expected_taken;
  word_t operand_edges [0:7];
  integer branch_offsets [0:7];
  integer jump_offsets [0:7];
  logic [31:0] rng = 32'h1badf00d;

  rv32_decoder decoder (.fetch_i(fetch), .decoded_o(decoded));
  rv32_alu alu (.a_i(a), .b_i(b), .op_i(alu_op), .result_o(result));
  rv32_branch_compare compare (.rs1_i(a), .rs2_i(b), .op_i(branch_op), .taken_o(taken));

  // A test-only model of the future operand muxes. a/b represent values read from the PRF.
  // There is no allocator, physical register file, or scheduler in this testbench.
  always_comb begin
    case (decoded.src_a)
      SRC_A_ZERO: selected_a = 0;
      SRC_A_PC: selected_a = decoded.fetch.pc;
      default: selected_a = a;
    endcase
    selected_b = decoded.src_b == SRC_B_IMMEDIATE ? decoded.immediate : b;
  end
  rv32_alu decoded_alu (
    .a_i(selected_a), .b_i(selected_b), .op_i(decoded.alu_op), .result_o(decoded_result)
  );

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 32)
          $display("FAIL: %s instr=%h a=%h b=%h", label_text, fetch.instruction, a, b);
      end
    end
  endtask

  task automatic drive(input word_t instruction);
    begin
      fetch = '0;
      fetch.pc = 32'h100;
      fetch.instruction = instruction;
      fetch.prediction.next_pc = 32'h104;
      fetch.prediction.btb_hit = 1;
      fetch.prediction.pht_index = 17;
      #1;
    end
  endtask

  task automatic check_illegal;
    begin
      check(decoded.fetch.exception.valid, "unsupported encoding raises a fault");
      check(decoded.fetch.exception.cause == CAUSE_ILLEGAL_INSTR &&
            decoded.fetch.exception.tval == fetch.instruction,
            "illegal fault preserves instruction");
      check(decoded.op_class == UOP_NONE && !decoded.writes_rd &&
            !decoded.uses_rs1 && !decoded.uses_rs2, "fault has no execution dependencies/effects");
    end
  endtask

  function automatic word_t enc_r(input integer upper, rs2, rs1, func, rd);
    enc_r = (upper << 25) | (rs2 << 20) | (rs1 << 15) | (func << 12) | (rd << 7) | 'h33;
  endfunction
  function automatic word_t enc_i(input integer value, rs1, func, rd, opcode);
    enc_i = ((value & 4095) << 20) | (rs1 << 15) | (func << 12) | (rd << 7) | opcode;
  endfunction
  function automatic word_t enc_s(input integer value, rs2, rs1, func);
    enc_s = (((value >> 5) & 127) << 25) | (rs2 << 20) | (rs1 << 15) |
            (func << 12) | ((value & 31) << 7) | 'h23;
  endfunction
  function automatic word_t enc_b(input integer value, rs2, rs1, func);
    enc_b = (((value >> 12) & 1) << 31) | (((value >> 5) & 63) << 25) |
            (rs2 << 20) | (rs1 << 15) | (func << 12) |
            (((value >> 1) & 15) << 8) | (((value >> 11) & 1) << 7) | 'h63;
  endfunction
  function automatic word_t enc_j(input integer value, rd);
    enc_j = (((value >> 20) & 1) << 31) | (((value >> 1) & 1023) << 21) |
            (((value >> 11) & 1) << 20) | (((value >> 12) & 255) << 12) | (rd << 7) | 'h6f;
  endfunction

  // Fixed numeric selectors keep the oracle independent of the DUT's enum-case arrangement.
  function automatic word_t arithmetic_reference(input integer op, input word_t x, y);
    integer shift;
    logic signed [31:0] sx;
    logic signed [31:0] sy;
    begin
      shift = y % 32;
      sx = x;
      sy = y;
      case (op)
        0: arithmetic_reference = x + y;
        1: arithmetic_reference = x - y;
        2: arithmetic_reference = x * (64'd1 << shift);
        3: arithmetic_reference = sx < sy ? 1 : 0;
        4: arithmetic_reference = x < y ? 1 : 0;
        5: arithmetic_reference = x ^ y;
        6: arithmetic_reference = x / (64'd1 << shift);
        7: begin
          // Logical right shift plus an explicit sign mask, rather than repeating >>>.
          arithmetic_reference = x / (64'd1 << shift);
          if (x[31] && shift != 0)
            arithmetic_reference = arithmetic_reference | (32'hffffffff << (32 - shift));
        end
        8: arithmetic_reference = x | y;
        9: arithmetic_reference = x & y;
        default: arithmetic_reference = y;
      endcase
    end
  endfunction

  task automatic select_alu(input integer op);
    begin
      case (op)
        0: alu_op = ALU_ADD;
        1: alu_op = ALU_SUB;
        2: alu_op = ALU_SLL;
        3: alu_op = ALU_SLT;
        4: alu_op = ALU_SLTU;
        5: alu_op = ALU_XOR;
        6: alu_op = ALU_SRL;
        7: alu_op = ALU_SRA;
        8: alu_op = ALU_OR;
        9: alu_op = ALU_AND;
        default: alu_op = ALU_COPY_B;
      endcase
    end
  endtask

  initial begin
    #100000;
    $fatal(1, "Block 2 timed out");
  end

  initial begin
    a = 0;
    b = 0;
    alu_op = ALU_ADD;
    branch_op = BRANCH_NONE;
    operand_edges[0] = 0;
    operand_edges[1] = 1;
    operand_edges[2] = 32'hffffffff;
    operand_edges[3] = 32'h80000000;
    operand_edges[4] = 32'h7fffffff;
    operand_edges[5] = 32'haaaaaaaa;
    operand_edges[6] = 32'h55555555;
    operand_edges[7] = 31;
    branch_offsets[0] = -4096;
    branch_offsets[1] = -2048;
    branch_offsets[2] = -2;
    branch_offsets[3] = 0;
    branch_offsets[4] = 2;
    branch_offsets[5] = 2048;
    branch_offsets[6] = 4092;
    branch_offsets[7] = 4094;
    jump_offsets[0] = -1048576;
    jump_offsets[1] = -524288;
    jump_offsets[2] = -2;
    jump_offsets[3] = 0;
    jump_offsets[4] = 2;
    jump_offsets[5] = 524288;
    jump_offsets[6] = 1048572;
    jump_offsets[7] = 1048574;

    // Exhaust the complete R-type funct7/funct3 space at fixed nonzero register fields.
    for (f7 = 0; f7 < 128; f7 = f7 + 1) begin
      for (f3 = 0; f3 < 8; f3 = f3 + 1) begin
        drive(enc_r(f7, 2, 1, f3, 31));
        expected_legal = f7 == 0 || f7 == 1 || (f7 == 32 && (f3 == 0 || f3 == 5));
        if (!expected_legal) begin
          check_illegal;
        end else begin
          legal_r_count = legal_r_count + 1;
          check(!decoded.fetch.exception.valid && decoded.writes_rd,
                "legal R-type encoding writes its nonzero destination");
          check(decoded.uses_rs1 && decoded.uses_rs2 && decoded.rs1 == 1 &&
                decoded.rs2 == 2 && decoded.rd == 31, "R-type source fields and dependencies");
          if (f7 == 1) begin
            check(decoded.op_class == UOP_MULDIV && decoded.muldiv_op == f3,
                  "all eight M encodings select the mul/div engine");
          end else begin
            if (f7 == 32) begin
              if (f3 == 0) expected_alu = ALU_SUB;
              else expected_alu = ALU_SRA;
            end else begin
              case (f3)
                0: expected_alu = ALU_ADD;
                1: expected_alu = ALU_SLL;
                2: expected_alu = ALU_SLT;
                3: expected_alu = ALU_SLTU;
                4: expected_alu = ALU_XOR;
                5: expected_alu = ALU_SRL;
                6: expected_alu = ALU_OR;
                default: expected_alu = ALU_AND;
              endcase
            end
            check(decoded.op_class == UOP_INTEGER && decoded.alu_op == expected_alu,
                  "base R-type operation selects the right ALU function");
          end
        end
      end
    end
    check(legal_r_count == 18, "exactly ten base and eight M R-type selectors are legal");

    // Exhaust all shift upper fields; RV32 does not permit a sixth shift-count bit.
    for (f7 = 0; f7 < 128; f7 = f7 + 1) begin
      drive(enc_i((f7 << 5) | 31, 1, 1, 5, 'h13));
      if (f7 != 0) check_illegal;
      else check(decoded.alu_op == ALU_SLL && decoded.immediate == 31,
                 "SLLI legal shift amount");
      drive(enc_i((f7 << 5) | 31, 1, 5, 5, 'h13));
      if (f7 != 0 && f7 != 32) check_illegal;
      else check(decoded.alu_op == (f7 == 0 ? ALU_SRL : ALU_SRA) && decoded.immediate == 31,
                 "SRLI/SRAI legal shift amount");
    end

    // Full 12-bit immediate space catches split-field and sign-extension mistakes.
    for (imm = -2048; imm <= 2047; imm = imm + 1) begin
      drive(enc_i(imm, 9, 0, 5, 'h13));
      check(!decoded.fetch.exception.valid && decoded.immediate == imm,
            "ADDI immediate reconstructs all signed 12-bit values");
      check(decoded.uses_rs1 && !decoded.uses_rs2 && decoded.src_b == SRC_B_IMMEDIATE,
            "ADDI never invents an rs2 dependency");
      drive(enc_s(imm, 8, 9, 2));
      check(!decoded.fetch.exception.valid && decoded.immediate == imm,
            "store immediate reconstructs all signed 12-bit values");
      check(decoded.uses_rs1 && decoded.uses_rs2 && !decoded.writes_rd,
            "store keeps base/data sources and has no GPR destination");
    end

    for (n = 0; n < 8; n = n + 1) begin
      for (f3 = 0; f3 < 8; f3 = f3 + 1) begin
        drive(enc_b(branch_offsets[n], 2, 1, f3));
        if (f3 == 2 || f3 == 3) check_illegal;
        else begin
          check(!decoded.fetch.exception.valid && decoded.immediate == branch_offsets[n],
                "B immediate preserves signed offset including two-byte targets");
          check(decoded.op_class == UOP_BRANCH && decoded.uses_rs1 && decoded.uses_rs2 &&
                !decoded.writes_rd, "conditional branch dependency flags");
          case (f3)
            0: check(decoded.branch_op == BRANCH_EQ, "BEQ selector");
            1: check(decoded.branch_op == BRANCH_NE, "BNE selector");
            4: check(decoded.branch_op == BRANCH_LT, "BLT selector");
            5: check(decoded.branch_op == BRANCH_GE, "BGE selector");
            6: check(decoded.branch_op == BRANCH_LTU, "BLTU selector");
            7: check(decoded.branch_op == BRANCH_GEU, "BGEU selector");
          endcase
        end
      end
      drive(enc_j(jump_offsets[n], 5));
      check(!decoded.fetch.exception.valid && decoded.immediate == jump_offsets[n],
            "J immediate preserves signed offset including two-byte targets");
      check(decoded.op_class == UOP_JUMP && !decoded.jump_indirect &&
            !decoded.uses_rs1 && !decoded.uses_rs2 && decoded.src_a == SRC_A_PC,
            "JAL target uses PC without register dependencies");
    end

    for (f3 = 0; f3 < 8; f3 = f3 + 1) begin
      drive(enc_i(-8, 9, f3, 5, 'h03));
      if (f3 == 3 || f3 >= 6) check_illegal;
      else begin
        check(decoded.op_class == UOP_LOAD && decoded.uses_rs1 && !decoded.uses_rs2 &&
              decoded.writes_rd && decoded.immediate == -8, "load dependency/address controls");
        check(decoded.mem_size == (f3 == 0 || f3 == 4 ? MEM_BYTE :
              f3 == 1 || f3 == 5 ? MEM_HALF : MEM_WORD), "load width");
        check(decoded.load_unsigned == (f3 == 4 || f3 == 5), "load extension selection");
      end
      drive(enc_s(-8, 8, 9, f3));
      if (f3 > 2) check_illegal;
      else check(decoded.mem_size == f3 && decoded.op_class == UOP_STORE, "store widths");
      drive(enc_i(-8, 9, f3, 5, 'h67));
      if (f3 != 0) check_illegal;
      else check(decoded.jump_indirect && decoded.uses_rs1 && !decoded.uses_rs2 &&
                 decoded.immediate == -8, "JALR base/offset controls");
    end

    // Immediate arithmetic functions beyond ADDI must not silently turn into ADD.
    for (f3 = 0; f3 < 8; f3 = f3 + 1) begin
      if (f3 != 1 && f3 != 5) begin
        drive(enc_i(-1, 3, f3, 5, 'h13));
        case (f3)
          0: expected_alu = ALU_ADD;
          2: expected_alu = ALU_SLT;
          3: expected_alu = ALU_SLTU;
          4: expected_alu = ALU_XOR;
          6: expected_alu = ALU_OR;
          default: expected_alu = ALU_AND;
        endcase
        check(decoded.alu_op == expected_alu && decoded.immediate == 32'hffffffff,
              "immediate ALU selection and sign extension including SLTIU");
      end
    end

    for (f3 = 0; f3 < 8; f3 = f3 + 1) begin
      drive(enc_i('h340, 31, f3, 5, 'h73));
      if (f3 == 0 || f3 == 4) check_illegal;
      else begin
        check(decoded.op_class == UOP_SYSTEM && decoded.system_op == SYS_CSR &&
              decoded.csr_op == f3 && decoded.csr_address == CSR_MSCRATCH,
              "all six CSR forms retain address and operation");
        check(decoded.uses_rs1 == (f3 < 4) && !decoded.uses_rs2 &&
              decoded.rs1 == 31 && decoded.csr_immediate == 31,
              "CSR immediate forms do not read an integer register");
      end
    end
    drive(32'h00000073); check(decoded.system_op == SYS_ECALL, "ECALL recognition");
    drive(32'h00100073); check(decoded.system_op == SYS_EBREAK, "EBREAK recognition");
    drive(32'h30200073); check(decoded.system_op == SYS_MRET, "MRET recognition");
    drive(32'h10500073); check(decoded.system_op == SYS_WFI, "WFI recognition");
    drive(32'h302000f3); check_illegal; // MRET with a nonzero rd is not MRET.
    drive(32'h10200073); check_illegal; // SRET is outside this machine-only core.
    drive(32'h00000000); check_illegal;
    drive(32'hffffffff); check_illegal;
    drive(32'h0000001b); check_illegal; // RV64 OP-IMM-32 is outside RV32.
    for (f3 = 0; f3 < 8; f3 = f3 + 1) begin
      drive(enc_i('habc, 31, f3, 31, 'h0f));
      if (f3 > 1) check_illegal;
      else check(decoded.system_op == (f3 == 0 ? SYS_FENCE : SYS_FENCE_I) &&
                 !decoded.writes_rd && !decoded.uses_rs1 && !decoded.uses_rs2,
                 "FENCE/FENCE.I ignore reserved fields without invented GPR effects");
    end

    drive(enc_i(0, 1, 2, 0, 'h03));
    check(decoded.op_class == UOP_LOAD && !decoded.writes_rd,
          "load to x0 still performs the load but discards the GPR result");
    drive(enc_i('h340, 0, 2, 0, 'h73));
    check(decoded.system_op == SYS_CSR && decoded.rs1 == 0 && !decoded.writes_rd,
          "CSR zero register fields remain available for later side-effect rules");
    drive(NOP_INSTRUCTION);
    check(!decoded.fetch.exception.valid && !decoded.writes_rd && !decoded.uses_rs2,
          "architectural NOP is legal and has no physical destination");
    drive(32'hffffffff);
    fetch.exception.valid = 1;
    fetch.exception.cause = CAUSE_INSTR_ACCESS;
    fetch.exception.tval = 32'h100;
    #1;
    check(decoded.fetch.exception == fetch.exception, "fetch fault wins over illegal instruction");
    check(decoded.fetch.pc == fetch.pc && decoded.fetch.prediction == fetch.prediction,
          "fault token preserves PC and prediction metadata");
    check(decoded.op_class == UOP_NONE && !decoded.writes_rd && !decoded.uses_rs1 &&
          !decoded.uses_rs2, "fetch fault cannot issue register operations");

    // Arithmetic oracle uses edge cross-products and deterministic random values.
    for (op_number = 0; op_number < 11; op_number = op_number + 1) begin
      select_alu(op_number);
      for (n = 0; n < 8; n = n + 1) begin
        for (k = 0; k < 8; k = k + 1) begin
          a = operand_edges[n]; b = operand_edges[k]; #1;
          expected_result = arithmetic_reference(op_number, a, b);
          check(result == expected_result, "ALU edge operands");
        end
      end
      for (n = 0; n < 64; n = n + 1) begin
        rng = {rng[30:0], rng[31] ^ rng[21] ^ rng[1] ^ rng[0]}; a = rng;
        rng = {rng[30:0], rng[31] ^ rng[21] ^ rng[1] ^ rng[0]}; b = rng; #1;
        expected_result = arithmetic_reference(op_number, a, b);
        check(result == expected_result, "ALU deterministic random operands");
      end
    end
    for (op_number = 2; op_number <= 7; op_number = op_number + 1) begin
      if (op_number == 2 || op_number == 6 || op_number == 7) begin
        select_alu(op_number);
        for (n = 0; n < 96; n = n + 1) begin
          a = 32'h81234567; b = n; #1;
          check(result == arithmetic_reference(op_number, a, b),
                "RV32 shifts use only five low count bits");
        end
      end
    end
    for (op_number = 0; op_number < 7; op_number = op_number + 1) begin
      case (op_number)
        0: branch_op = BRANCH_NONE;
        1: branch_op = BRANCH_EQ;
        2: branch_op = BRANCH_NE;
        3: branch_op = BRANCH_LT;
        4: branch_op = BRANCH_GE;
        5: branch_op = BRANCH_LTU;
        default: branch_op = BRANCH_GEU;
      endcase
      for (n = 0; n < 8; n = n + 1) begin
        for (k = 0; k < 8; k = k + 1) begin
          a = operand_edges[n]; b = operand_edges[k]; #1;
          case (op_number)
            0: expected_taken = 0;
            1: expected_taken = a == b;
            2: expected_taken = a != b;
            3: expected_taken = (a ^ 32'h80000000) < (b ^ 32'h80000000);
            4: expected_taken = !((a ^ 32'h80000000) < (b ^ 32'h80000000));
            5: expected_taken = a < b;
            default: expected_taken = !(a < b);
          endcase
          check(taken == expected_taken, "branch comparison edge operands");
        end
      end
    end

    // ADD is the first learning example: names decode here; PRF values enter the ALU later.
    drive(32'h004182b3); // add x5,x3,x4
    a = 12; b = 7; #1;
    check(decoded.rs1 == 3 && decoded.rs2 == 4 && decoded.rd == 5 &&
          decoded_result == 19, "decoded ADD fields select the register-value datapath");
    drive(enc_i(-1, 3, 3, 5, 'h13)); a = 32'h80000000; #1;
    check(decoded_result == 1, "SLTIU compares against a sign-extended immediate");
    drive(32'habcde2b7); a = 32'hffffffff; b = 32'hffffffff; #1;
    check(decoded_result == 32'habcde000 && !decoded.uses_rs1 && !decoded.uses_rs2,
          "LUI ignores register-like immediate fields");
    drive(32'habcde297); #1;
    check(decoded_result == 32'habcde100 && decoded.src_a == SRC_A_PC,
          "AUIPC adds the instruction PC to the upper immediate");

    $display("BLOCK02 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Block 2 decode/datapath tests failed");
    $finish;
  end
endmodule
```
