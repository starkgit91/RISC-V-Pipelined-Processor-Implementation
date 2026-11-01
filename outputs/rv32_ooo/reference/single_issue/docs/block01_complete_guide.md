# Block 1: the common language between processor blocks

This block defines the formats used to carry instructions and results through the processor.
The package and its testbench are complete for this block. They are not a complete processor.
Later blocks will implement the circuits that allocate, store, inspect, and update these records.

## 1. Where this block operates

Our design has five logical architectural sections, not a fixed five-clock instruction latency.

```text
 IF                 ID                    EX                 MEM              WB
 Fetch       Decode -> Rename       Schedule -> Execute   Ordered access   Complete -> Commit
   |                  |                      |                  |                 |
 fetch_packet_t -> decoded_uop_t -> renamed_uop_t -> execute_packet_t       completion_t
                        |                    |                  |                 |
                        +---------- ROB keeps program order ----+----> retire_trace_t
```

These arrows show record ownership and data flow, not a complete wiring diagram. A load/store
uses the memory path; an ADD does not issue a memory transaction. Completion informs both the
physical register file and ROB. Retirement is a separate decision made at ROB head.

There are no runtime input or output pins on a SystemVerilog package. A module imports the
definitions when compiled. Later module ports carry records of these types and separate
valid/ready signals indicate when a record is meaningful and when it transfers.

## 2. Configuration and hardware dimensions

| Name | Value | What later hardware uses it for |
|---|---:|---|
| XLEN | 32 | Operand, result, address, and instruction width |
| ARCH_REG_COUNT | 32 | Program-visible x0 through x31 |
| PHYS_REG_COUNT | 64 | Storage for committed and speculative register versions |
| ROB_DEPTH | 16 | Maximum instructions tracked in the reorder buffer |
| ISSUE_DEPTH | 8 | Instructions waiting to issue |
| MEMORY_DEPTH | 4 | Load/store records waiting for preparation or completion |
| FETCH_DEPTH | 2 | Buffer fetched instructions when decode waits |
| BTB_ENTRIES | 64 | Remember branch/jump PCs and target addresses |
| PHT_ENTRIES | 128 | Two-bit branch-direction counters |

The source derives register tag widths with `$clog2`. Naming 32 architectural registers takes
5 bits; naming 64 physical registers takes 6 bits. A 16-entry ROB slot index takes 4 bits.

The configuration does not instantiate RAMs or registers. For example, a later declaration of
an array of 64 `word_t` elements will describe the physical register storage. Constants can set
its size without describing the read multiplexers or write logic, which belong to block 3.

The 64 KiB RAM contains 16,384 words of four bytes. Requests remain byte-addressed. The memory
adapter will translate the address to the BRAM's word location; callers must not pre-divide
the address and then accidentally divide it again.

## 3. Architectural registers, physical registers, and instruction identity

An architectural register is a name in the program. A physical register is storage for a
particular version of a value. A ROB tag identifies an instruction, not a register.

Consider two instructions that both write x5:

```asm
div x5, x1, x2
add x5, x3, x4
```

Illustrative mappings, assuming p37 and p38 are available:

| Instruction | Architectural destination | New physical destination | Previous mapping |
|---|---|---|---|
| DIV | x5 | p37 | p5 |
| ADD | x5 | p38 | p37 |

The DIV and ADD keep separate values. A later instruction reading x5 after the ADD is renamed
to read p38. An instruction between the two writers would retain p37 as its source. This is
how renaming removes false dependencies without changing program semantics.

The numbers p37/p38 are examples, not a required allocator sequence. The testbench constructs
these records directly; it does not yet implement allocation or execute either instruction.

`writes_rd` means a real write to a nonzero architectural register. The decoder will clear it
for rd=x0; renaming must never allocate a physical destination for x0. The register file will
also independently protect physical p0 so correctness does not depend on one control signal.

### A conceptual timeline

| Event | Meaning |
|---|---|
| Rename DIV | Capture x1/x2 source mappings; assign x5 a new destination p37 |
| Rename ADD | Capture x3/x4 mappings; replace x5's speculative mapping with p38 |
| Issue DIV | Divider starts working on its captured operand values |
| Issue ADD | Independent integer work can proceed while DIV is busy |
| Complete ADD | p38 becomes ready; ADD's ROB entry becomes complete |
| DIV still running | ADD cannot retire past the older incomplete DIV |
| Complete and retire DIV | Committed x5 becomes p37; old committed p5 is released |
| Retire ADD | Committed x5 becomes p38; old committed p37 is released |

These are intended behavior examples for future blocks, not measured cycle counts from this
block's declaration-only simulation. The distinction between readiness and commitment is central:
ready means a value can feed dependent instructions; committed means it belongs to architectural state.

## 4. Field map: which block produces and consumes each record

| Record / fields | Width or shape | Produced by | Consumed by | Meaning |
|---|---|---|---|---|
| `word_t` | 32 bits | Datapaths and memories | Datapaths and memories | Raw data/address/instruction bits; signedness is selected by the operation |
| `arch_reg_t` | 5 bits | Decoder | Rename and retirement | Program register name |
| `phys_reg_t` | 6 bits | Rename | Scheduler, register file, completion | Internal storage identifier |
| `rob_tag_t.index` | 4 bits | ROB allocator | Queues and completion | Slot occupied by this instruction |
| `rob_tag_t.generation` | 8 bits | ROB allocator | Queues and completion | Distinguish recycled uses of a slot |
| `exception_t` | valid + 32-bit cause + 32-bit tval | Fetch/decode/execute/memory | ROB and trap control | Fault ownership and details |
| `prediction_t` | next PC + flags + 7-bit PHT index | Fetch predictor | Execution and retirement | What was predicted, and which predictor counter was used |
| `fetch_packet_t` | PC + instruction + prediction + exception | IF | Decoder | One fetched instruction or fetch fault |
| `decoded_uop_t` | Fetch metadata + controls + architectural sources/destination | Decoder | Rename and execution paths | Internal operation description |
| `renamed_uop_t` | Decoded operation + ROB tag + four physical tags | Rename | Scheduler, execution, ROB | Captured sources, new destination, and old destination |
| `issue_entry_t` | valid + renamed operation + readiness flags | Dispatch/wakeup | Scheduler | Instruction waiting for a suitable execution engine |
| `execute_packet_t` | Renamed operation + two 32-bit values | Register read | Execution engine | Resolved operand values, not only their tags |
| `resolved_control_t` | Classification, outcome, mismatch, next PC | Execution | ROB/recovery/predictor update | Actual control-flow result |
| `completion_t` | ROB/physical identity + value + exception/control result | Execution or memory | WB arbiter, PRF, ROB | Speculative completion; not an architectural commit |
| `rob_entry_t` | Valid/completion state + original operation + result/fault/control | Dispatch and completion | Retirement/recovery | Instruction state in program order |
| `memory_entry_t` | Identity + prepared address/data + access metadata | Dispatch/address generation | Memory engine | Load/store preparation and response ownership |
| `data_request_t` | write + 32-bit address/data + four strobes | LSU | Native/AXI adapter | One aligned word transaction with selected write bytes |
| `memory_response_t` | 32-bit data + error | Memory adapter | Fetch/LSU | Completion of a request, including errors |
| `retire_trace_t` | Architectural event, operands/results, optional store information | Retirement/trap controller | Testbench/debug | What actually became architecturally visible |

The ROB stores the original renamed operation so retirement has the instruction PC, architectural
destination, and old/new mappings. It also has a current exception field: initialize that from
the incoming fetch/decode fault, then update it if execution discovers a fault. The nested incoming
metadata remains the original record; the current ROB exception is authoritative at retirement.

Every completion must supply the actual successor PC. For an ordinary arithmetic, multiply/divide,
or memory instruction this is pc+4. Leaving it zero would corrupt committed-PC tracking and
interrupt return addresses. A cleared record is an empty default, not a valid completion result.

Memory responses have no transaction ID because each port permits only one outstanding request.
The requesting engine keeps that request's tag and access metadata until it consumes the response.
This restriction must be enforced by the later memory engine and adapters.

## 5. Important contracts the later circuits must enforce

### Validity is not the instruction value

A zeroed queue entry has `valid=0`. Its other bits must not cause execution, allocation, or
retirement. Do not infer validity from instruction bits. An actual fetched all-zero instruction
is illegal; an invalid queue entry is simply empty. Canonical NOP is `0x00000013`.

Handshake payloads do not contain another `valid` bit. Their interface has a separate valid signal.
Stored queue entries do contain validity because the owning queue must remember occupied slots.

### Tags are not values

`prs1=37` means read p37, not use decimal 37 as an operand. `rs1_value` is the actual 32-bit operand
captured for execution. Keeping these names different makes waveforms easier to read.

### Source-use flags matter

For ADDI, instruction bits [24:20] are part of the immediate, not a second register source. The
decoder sets `uses_rs2=0`, so the scheduler must not wait for a nonexistent second dependency.
Similarly, immediate CSR operations use `csr_immediate`, not the contents of an integer register.

### Completion is not commitment

An execution result may wake younger instructions while an older instruction still blocks retirement.
Only the committed rename map defines architectural register contents. A testbench comparing the
program's x5 must consult committed state or retirement events, not an arbitrary speculative PRF entry.

### Generation matching is necessary but not sufficient

A generation field eventually wraps. Recovery must cancel or drain old operations and their held
completions before reusing their ROB/physical resources. A generation counter is not a substitute
for this cancellation guarantee. Block 11 will implement the recovery logic.

### Memory acceptance is not completion

`data_request_t` has aligned byte addresses and byte strobes. A byte store at lane 2 can carry
`wstrb=0100` and its byte in `wdata[23:16]`. This represents a transfer; it does not yet implement
alignment, extraction, or AXI handshaking. The LSU and AXI adapter implement those later.

The original effective address remains in the memory entry for byte extraction and trap reporting.
Memory requests will start only at ROB head. A store cannot retire just because its request was accepted.

### Internal operation codes are not instruction encodings

`ALU_ADD` is a compact internal selector, not the 32-bit encoding of ADD. Most enum encodings are
private implementation details. M-extension and CSR-operation enum values intentionally match their
funct3 fields after the decoder validates the full instruction. Unsupported encodings still trap.

## 6. What exists at circuit level after this block

The package describes widths and layouts. It does not instantiate a clocked register, memory,
scheduler, divider, or comparator. A later queue module will use these types to declare storage
and write `always_ff` logic. A packed record can travel over a group of wires or be stored in
flip-flops; `packed` makes the bit layout definite, rather than choosing an implementation technology.

Use one shared package instead of independently redefining ports in each module. A mismatch in
destination width or tag ordering could otherwise connect the wrong result to a register.

Clock targets, CSR addresses, and exception causes are also definitions, not implementations.
For example, declaring `CSR_MTVEC` does not create a trap-vector register or enforce its write rules.

## 7. Verification and waveform expectations

Run `python3 scripts/run_checks.py --block 1` from the project directory. The testbench checks:

- Agreed dimensions, derived tag widths, clock constants, and memory windows.
- Every ROB index across several generations.
- Distinct physical destinations for two modeled writers of x5.
- Full packed-record round trips, including nested fields.
- Preservation of exception and completion identity information.
- Invalid/empty queue representation and byte-strobe representation.

These checks establish representation contracts. They do not prove a scheduler, register allocator,
or CPU correct. The records are filled directly by the testbench. Simulation ends at time zero,
because no clocked logic exists in this block. A cycle waveform is therefore not useful yet;
later module tests will generate clocked waveforms and measured instruction traces.

The README includes equivalent `xvlog`, `xelab`, and `xsim` commands for your Linux Vivado install.
Icarus compilation/simulation is verified here; Vivado has not been run here.

With 64 physical registers and a 16-entry ROB, normal full-core operation cannot use up all
32 initially free physical registers: at most 16 unretired destinations are allocated. Later
free-list exhaustion tests will therefore exercise the allocator unit with controlled availability
or a smaller test configuration, rather than claim an unreachable full-core scenario was tested.

## 8. Interview questions and model answers

**Why have more physical registers than architectural registers?**
To keep separate in-flight versions of a program register, removing WAR and WAW false dependencies.

**Does renaming remove RAW dependencies?**
No. A consumer still needs its producer's actual value. The physical source tag identifies that producer.

**Why save `old_pdst`?**
The old mapping remains needed by older instructions and committed state. It can be released when
the replacement instruction retires, rather than when the replacement merely completes.

**Why not identify an instruction only by ROB slot number?**
Slots are reused. A tag includes a generation, and cancellation rules prevent late old results from
being accepted for a new occupant.

**Why is a completed instruction not necessarily retired?**
An older instruction might still be running or might fault. In-order retirement preserves precise
architectural state while allowing execution to occur out of order.

**Does a SystemVerilog struct automatically create hardware storage?**
No. It defines a bit layout. Storage and combinational behavior come from declarations and logic
that use the type inside actual modules.

**Can this block generate a processor bitstream by itself?**
No. This is the shared foundation. The processor modules and FPGA integration are subsequent blocks.

## 9. References and coding style

This is original project source. The following sources informed its conventions and explanations:

- [lowRISC Verilog coding style](https://github.com/lowRISC/style-guides/blob/master/VerilogCodingStyle.md):
  readable names, consistent formatting, and explicit combinational/sequential intent.
- [BOOM rename-stage documentation](https://docs.boom-core.org/en/latest/sections/rename-stage.html):
  physical versions, maps, readiness, old destinations, and committed-map recovery.
- [RISC-V ISA specifications](https://docs.riscv.org/reference/isa/): architectural names and encodings.

The custom core will use conservative retirement-based branch recovery. This is a project design
choice, not a claim that it implements BOOM's checkpointed branch-recovery mechanism.

## Complete shared package

Source: `rtl/rv32_ooo_pkg.sv`.

```systemverilog
// Shared definitions for the custom RV32IM_Zicsr_Zifencei processor.
// Import this package before declaring a module's ports or internal records.
// Types describe wires and register contents; the package itself creates no storage.
`timescale 1ns / 1ps

package rv32_ooo_pkg;

  // Configuration: one dispatch, issue, completion, and retirement per cycle.
  localparam int XLEN             = 32;
  localparam int ARCH_REG_COUNT   = 32;
  localparam int PHYS_REG_COUNT   = 64;
  localparam int ROB_DEPTH        = 16;
  localparam int ISSUE_DEPTH      = 8;
  localparam int MEMORY_DEPTH     = 4;
  localparam int FETCH_DEPTH      = 2;
  localparam int BTB_ENTRIES      = 64;
  localparam int PHT_ENTRIES      = 128;

  localparam int ARCH_REG_W       = $clog2(ARCH_REG_COUNT);
  localparam int PHYS_REG_W       = $clog2(PHYS_REG_COUNT);
  localparam int ROB_INDEX_W      = $clog2(ROB_DEPTH);
  localparam int ROB_GENERATION_W = 8;
  localparam int PHT_INDEX_W      = $clog2(PHT_ENTRIES);

  typedef logic [XLEN-1:0]           word_t;
  typedef logic [ARCH_REG_W-1:0]     arch_reg_t;
  typedef logic [PHYS_REG_W-1:0]     phys_reg_t;
  typedef logic [PHYS_REG_COUNT-1:0] phys_mask_t;
  typedef logic [$clog2(PHYS_REG_COUNT+1)-1:0] phys_count_t;
  typedef logic [11:0]               csr_addr_t;
  typedef logic [PHT_INDEX_W-1:0]    pht_index_t;

  localparam word_t RESET_PC       = 32'h0000_0000;
  localparam word_t NOP_INSTRUCTION = 32'h0000_0013;
  localparam word_t MISA_VALUE     = 32'h4000_1100; // RV32, I and M; Z extensions have no misa bit.
  localparam int REFERENCE_HZ      = 100_000_000;
  localparam int SYSTEM_HZ         = 50_000_000;
  localparam int UART_BAUD         = 115_200;

  // All addresses are BYTE addresses, including requests to the BRAM adapter.
  localparam word_t RAM_BASE       = 32'h0000_0000;
  localparam int RAM_BYTES         = 65_536;
  localparam int RAM_WORDS         = RAM_BYTES / (XLEN / 8);
  localparam word_t TIMER_BASE     = 32'h0200_0000;
  localparam word_t UART_BASE      = 32'h1000_0000;
  localparam word_t GPIO_BASE      = 32'h1001_0000;
  localparam word_t MMIO_BYTES     = 32'h0001_0000;
  localparam word_t MSIP_OFFSET    = 32'h0000_0000;
  localparam word_t MTIMECMP_OFFSET = 32'h0000_4000;
  localparam word_t MTIME_OFFSET   = 32'h0000_bff8;

  // CSR addresses the decoder will recognize; access rules belong to the CSR block.
  localparam csr_addr_t CSR_MSTATUS       = 12'h300;
  localparam csr_addr_t CSR_MISA          = 12'h301;
  localparam csr_addr_t CSR_MIE           = 12'h304;
  localparam csr_addr_t CSR_MTVEC         = 12'h305;
  localparam csr_addr_t CSR_MSTATUSH      = 12'h310;
  localparam csr_addr_t CSR_MCOUNTINHIBIT = 12'h320;
  localparam csr_addr_t CSR_MSCRATCH      = 12'h340;
  localparam csr_addr_t CSR_MEPC          = 12'h341;
  localparam csr_addr_t CSR_MCAUSE        = 12'h342;
  localparam csr_addr_t CSR_MTVAL         = 12'h343;
  localparam csr_addr_t CSR_MIP           = 12'h344;
  localparam csr_addr_t CSR_MCYCLE        = 12'hb00;
  localparam csr_addr_t CSR_MINSTRET      = 12'hb02;
  localparam csr_addr_t CSR_MCYCLEH       = 12'hb80;
  localparam csr_addr_t CSR_MINSTRETH     = 12'hb82;
  localparam csr_addr_t CSR_MVENDORID     = 12'hf11;
  localparam csr_addr_t CSR_MARCHID       = 12'hf12;
  localparam csr_addr_t CSR_MIMPID        = 12'hf13;
  localparam csr_addr_t CSR_MHARTID       = 12'hf14;
  localparam csr_addr_t CSR_MCONFIGPTR    = 12'hf15;

  localparam word_t CAUSE_INSTR_MISALIGNED = 32'd0;
  localparam word_t CAUSE_INSTR_ACCESS     = 32'd1;
  localparam word_t CAUSE_ILLEGAL_INSTR    = 32'd2;
  localparam word_t CAUSE_BREAKPOINT       = 32'd3;
  localparam word_t CAUSE_LOAD_MISALIGNED  = 32'd4;
  localparam word_t CAUSE_LOAD_ACCESS      = 32'd5;
  localparam word_t CAUSE_STORE_MISALIGNED = 32'd6;
  localparam word_t CAUSE_STORE_ACCESS     = 32'd7;
  localparam word_t CAUSE_ECALL_M          = 32'd11;
  localparam word_t CAUSE_IRQ_SOFTWARE     = 32'h8000_0003;
  localparam word_t CAUSE_IRQ_TIMER        = 32'h8000_0007;
  localparam word_t CAUSE_IRQ_EXTERNAL     = 32'h8000_000b;

  // Operation names are internal control encodings, not RISC-V instruction words.
  typedef enum logic [2:0] {
    UOP_NONE, UOP_INTEGER, UOP_MULDIV, UOP_BRANCH,
    UOP_JUMP, UOP_LOAD, UOP_STORE, UOP_SYSTEM
  } uop_class_e;

  typedef enum logic [3:0] {
    ALU_ADD, ALU_SUB, ALU_SLL, ALU_SLT, ALU_SLTU, ALU_XOR,
    ALU_SRL, ALU_SRA, ALU_OR, ALU_AND, ALU_COPY_B
  } alu_op_e;

  typedef enum logic [1:0] {
    SRC_A_REGISTER, SRC_A_ZERO, SRC_A_PC
  } src_a_e;

  typedef enum logic {
    SRC_B_REGISTER, SRC_B_IMMEDIATE
  } src_b_e;

  typedef enum logic [2:0] {
    BRANCH_NONE, BRANCH_EQ, BRANCH_NE, BRANCH_LT,
    BRANCH_GE, BRANCH_LTU, BRANCH_GEU
  } branch_op_e;

  // The eight values match the M-extension funct3 field after encoding validation.
  typedef enum logic [2:0] {
    MD_MUL, MD_MULH, MD_MULHSU, MD_MULHU, MD_DIV, MD_DIVU, MD_REM, MD_REMU
  } muldiv_op_e;

  typedef enum logic [1:0] {
    MEM_BYTE, MEM_HALF, MEM_WORD
  } mem_size_e;

  typedef enum logic [3:0] {
    SYS_NONE, SYS_CSR, SYS_ECALL, SYS_EBREAK,
    SYS_MRET, SYS_WFI, SYS_FENCE, SYS_FENCE_I
  } system_op_e;

  typedef enum logic [2:0] {
    CSR_NONE = 3'b000,
    CSR_RW   = 3'b001,
    CSR_RS   = 3'b010,
    CSR_RC   = 3'b011,
    CSR_RWI  = 3'b101,
    CSR_RSI  = 3'b110,
    CSR_RCI  = 3'b111
  } csr_op_e;

  // A recycled ROB slot must not accept an earlier instruction's result.
  // Generation matching AND cancellation/drain before reuse are both required.
  typedef struct packed {
    logic [ROB_GENERATION_W-1:0] generation;
    logic [ROB_INDEX_W-1:0]      index;
  } rob_tag_t;

  typedef struct packed {
    logic  valid;
    word_t cause;
    word_t tval;
  } exception_t;

  typedef struct packed {
    word_t      next_pc;
    logic       taken;
    logic       btb_hit;
    pht_index_t pht_index;
  } prediction_t;

  // IF -> ID. A fetch fault travels with the instruction and is not executed.
  // Handshake valid/ready signals belong to the owning interface, outside this payload.
  typedef struct packed {
    word_t       pc;
    word_t       instruction;
    prediction_t prediction;
    exception_t  exception;
  } fetch_packet_t;

  // ID -> rename. Only the fields selected by op_class are used by an engine.
  // uses_rs* prevent immediate bits from being mistaken for source registers.
  // writes_rd means a real nonzero destination: the decoder clears it when rd == x0.
  typedef struct packed {
    fetch_packet_t fetch;
    uop_class_e    op_class;
    alu_op_e       alu_op;
    src_a_e        src_a;
    src_b_e        src_b;
    branch_op_e    branch_op;
    muldiv_op_e    muldiv_op;
    mem_size_e     mem_size;
    logic          load_unsigned;
    logic          jump_indirect;
    logic          uses_rs1;
    logic          uses_rs2;
    logic          writes_rd;
    arch_reg_t     rs1;
    arch_reg_t     rs2;
    arch_reg_t     rd;
    word_t         immediate;
    system_op_e    system_op;
    csr_op_e       csr_op;
    csr_addr_t     csr_address;
    logic [4:0]    csr_immediate;
  } decoded_uop_t;

  // Rename -> scheduler. Capture source mappings before changing rd's mapping.
  // old_pdst is released when the replacement retires, not when it completes.
  typedef struct packed {
    decoded_uop_t decoded;
    rob_tag_t     rob_tag;
    phys_reg_t    prs1;
    phys_reg_t    prs2;
    phys_reg_t    pdst;
    phys_reg_t    old_pdst;
  } renamed_uop_t;

  typedef struct packed {
    logic         valid;
    renamed_uop_t uop;
    logic         src1_ready;
    logic         src2_ready;
  } issue_entry_t;

  // Scheduler/register read -> execution. These are captured operand VALUES.
  typedef struct packed {
    renamed_uop_t uop;
    word_t        rs1_value;
    word_t        rs2_value;
  } execute_packet_t;

  // Every engine supplies the actual next_pc (pc + 4 for ordinary instructions).
  // Even a non-branch checks it, so a stale BTB hit can be corrected.
  typedef struct packed {
    logic  is_control;
    logic  is_conditional;
    logic  taken;
    logic  mispredicted;
    word_t next_pc;
  } resolved_control_t;

  // Execution/memory -> WB arbiter -> PRF and ROB. This is NOT a commit event.
  // A fault suppresses the destination write, even if writes_rd was decoded.
  typedef struct packed {
    rob_tag_t          rob_tag;
    phys_reg_t         pdst;
    logic              writes_rd;
    word_t             value;
    exception_t        exception;
    resolved_control_t control_flow;
  } completion_t;

  // ROB storage keeps the original uop for precise PC, rename, and system state.
  // exception is the current fault state; initialize it from uop.decoded.fetch.exception.
  typedef struct packed {
    logic              valid;
    logic              complete;
    renamed_uop_t      uop;
    word_t             result;
    exception_t        exception;
    resolved_control_t control_flow;
  } rob_entry_t;

  // Address/data may be prepared early; only the ROB head may send a data request.
  // store_value is unshifted; the LSU selects byte lanes when it builds a request.
  typedef struct packed {
    logic       valid;
    rob_tag_t   rob_tag;
    logic       prepared;
    logic       is_store;
    mem_size_e  size;
    logic       load_unsigned;
    phys_reg_t  pdst;
    logic       writes_rd;
    word_t      address;
    word_t      store_value;
    exception_t exception;
  } memory_entry_t;

  // LSU -> memory/AXI adapter. address is word-aligned; wstrb selects written bytes.
  // A write completes only when its response arrives; acceptance alone is not completion.
  typedef struct packed {
    logic       write;
    word_t      address;
    word_t      wdata;
    logic [3:0] wstrb;
  } data_request_t;

  typedef struct packed {
    word_t rdata;
    logic  error;
  } memory_response_t;

  // No transaction ID is needed here: each port permits one outstanding request.
  // Its owner retains the ROB tag and access metadata until this response is consumed.

  // WB/commit -> verification/debug. This describes architectural events only.
  // A trap event does not increment minstret; order numbers events, not just retirement.
  // This is a project trace format, not a claim of implementing the RVFI specification.
  typedef struct packed {
    logic        valid;
    logic [63:0] order;
    logic        trap;
    logic        interrupt;
    word_t       pc;
    word_t       instruction;
    word_t       next_pc;
    arch_reg_t   rd;
    word_t       rd_value;
    word_t       cause;
    word_t       tval;
    logic        memory_write;
    word_t       memory_address;
    logic [3:0]  memory_wstrb;
    word_t       memory_wdata;
  } retire_trace_t;

endpackage : rv32_ooo_pkg
```

## Complete self-checking testbench

Source: `tb/tb_block01.sv`.

```systemverilog
`timescale 1ns / 1ps

module tb_block01;
  import rv32_ooo_pkg::*;

  integer checks = 0;
  integer failures = 0;
  integer i;
  integer j;
  rob_tag_t tag_a;
  rob_tag_t tag_b;
  decoded_uop_t decoded;
  renamed_uop_t older;
  renamed_uop_t younger;
  renamed_uop_t copied;
  logic [$bits(renamed_uop_t)-1:0] packed_uop;
  completion_t completion;
  completion_t copied_completion;
  logic [$bits(completion_t)-1:0] packed_completion;
  issue_entry_t issue_entry;
  rob_entry_t rob_entry;
  memory_entry_t memory_entry;
  data_request_t request;
  retire_trace_t trace_event;

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        $display("FAIL: %s", label_text);
      end else begin
        $display("PASS: %s", label_text);
      end
    end
  endtask

  initial begin
    check(XLEN == 32 && ARCH_REG_COUNT == 32, "architectural width and register count");
    check(PHYS_REG_COUNT == 64 && ROB_DEPTH == 16, "physical register and ROB capacity");
    check(ISSUE_DEPTH == 8 && MEMORY_DEPTH == 4, "issue and memory queue capacity");
    check(ARCH_REG_W == 5 && PHYS_REG_W == 6, "register tags cover their complete ranges");
    check(ROB_INDEX_W == 4 && $bits(rob_tag_t) == 12, "ROB identity includes generation");
    check(BTB_ENTRIES == 64 && PHT_ENTRIES == 128, "predictor dimensions");
    check(RAM_BYTES == 65536 && RAM_WORDS == 16384, "byte-addressed RAM capacity");
    check(REFERENCE_HZ == 100000000 && SYSTEM_HZ == 50000000, "selected clock targets");
    check(RESET_PC == 0 && NOP_INSTRUCTION == 32'h00000013, "reset and architectural NOP");
    check(MISA_VALUE == 32'h40001100, "RV32 I/M misa constant");
    check(TIMER_BASE >= RAM_BASE + RAM_BYTES, "RAM and timer windows do not overlap");
    check(UART_BASE >= TIMER_BASE + MMIO_BYTES, "timer and UART windows do not overlap");
    check(GPIO_BASE >= UART_BASE + MMIO_BYTES, "UART and GPIO windows do not overlap");
    check(MTIME_OFFSET + 8 <= MMIO_BYTES, "64-bit timer fits its MMIO window");
    check(CSR_MEPC == 12'h341 && CSR_MCAUSE == 12'h342, "machine trap CSR addresses");
    check(CAUSE_ECALL_M == 11 && CAUSE_IRQ_TIMER == 32'h80000007,
          "exceptions and interrupts have distinct cause encodings");

    // Exhaust every slot through several generations; do not merely test slot zero.
    for (i = 0; i < ROB_DEPTH; i = i + 1) begin
      for (j = 0; j < 3; j = j + 1) begin
        tag_a.index = i;
        tag_a.generation = j;
        tag_b = tag_a;
        check(tag_a.index == i && tag_a.generation == j, "ROB identity retains its fields");
        tag_b.generation = j + 1;
        check(tag_a != tag_b, "same slot in another generation is a different instruction");
      end
    end

    decoded = '0;
    decoded.fetch.pc = 32'h00000020;
    decoded.fetch.instruction = 32'h0220c2b3; // div x5,x1,x2; decoder comes in block 2.
    decoded.fetch.prediction.next_pc = 32'h00000024;
    decoded.op_class = UOP_MULDIV;
    decoded.muldiv_op = MD_DIV;
    decoded.uses_rs1 = 1;
    decoded.uses_rs2 = 1;
    decoded.writes_rd = 1;
    decoded.rs1 = 1;
    decoded.rs2 = 2;
    decoded.rd = 5;
    older = '0;
    older.decoded = decoded;
    older.rob_tag.index = 14;
    older.rob_tag.generation = 9;
    older.prs1 = 1;
    older.prs2 = 2;
    older.pdst = 37;
    older.old_pdst = 5;

    // Represent a younger writer without implementing rename allocation here.
    younger = older;
    younger.decoded.fetch.pc = 32'h00000024;
    younger.decoded.fetch.instruction = 32'h004182b3; // add x5,x3,x4
    younger.decoded.op_class = UOP_INTEGER;
    younger.decoded.alu_op = ALU_ADD;
    younger.decoded.rs1 = 3;
    younger.decoded.rs2 = 4;
    younger.prs1 = 3;
    younger.prs2 = 4;
    younger.pdst = 38;
    younger.old_pdst = 37;
    younger.rob_tag.index = 15;
    check(older.decoded.rd == younger.decoded.rd && older.pdst != younger.pdst,
          "two versions of x5 can have different physical destinations");
    check(older.pdst == younger.old_pdst, "replacement retains the previous mapping");
    check(older.rob_tag != younger.rob_tag, "writers retain independent instruction identities");
    packed_uop = younger;
    copied = packed_uop;
    check(copied == younger, "full renamed record survives packed-wire transport");
    check(copied.decoded.fetch.pc == 32'h24 && copied.pdst == 38,
          "nested PC and destination fields are preserved");

    completion = '0;
    completion.rob_tag = younger.rob_tag;
    completion.pdst = younger.pdst;
    completion.writes_rd = 1;
    completion.value = 32'hfeedcafe;
    completion.control_flow.next_pc = 32'h28;
    packed_completion = completion;
    copied_completion = packed_completion;
    check(copied_completion == completion, "completion transport preserves identity and result");
    check(copied_completion.rob_tag == younger.rob_tag && copied_completion.pdst == 38,
          "writeback carries the owning ROB tag and physical destination");
    completion.exception.valid = 1;
    completion.exception.cause = CAUSE_LOAD_ACCESS;
    completion.exception.tval = 32'hf0000000;
    check(completion.exception.valid && completion.exception.tval == 32'hf0000000,
          "fault completion retains cause and fault address");

    issue_entry = '0;
    rob_entry = '0;
    memory_entry = '0;
    trace_event = '0;
    check(!issue_entry.valid && !rob_entry.valid && !memory_entry.valid && !trace_event.valid,
          "cleared validity distinguishes empty storage from an instruction");
    issue_entry.valid = 1;
    issue_entry.uop = older;
    check(issue_entry.uop.decoded.op_class == UOP_MULDIV,
          "issue queue record carries execution classification");
    rob_entry.valid = 1;
    rob_entry.uop = older;
    rob_entry.complete = 0;
    check(rob_entry.uop.old_pdst == 5 && !rob_entry.complete,
          "ROB retains rename history independently of completion");

    request = '0;
    request.write = 1;
    request.address = 32'h100;
    request.wdata = 32'h00ab0000;
    request.wstrb = 4'b0100;
    check(request.wstrb == 4'b0100 && request.wdata[23:16] == 8'hab,
          "data request can represent an isolated byte write");
    check(MD_MUL == 0 && MD_DIV == 4 && MD_REMU == 7, "M operations preserve funct3 encoding");
    check(CSR_RWI == 5 && CSR_RCI == 7, "CSR immediate operations preserve funct3 encoding");

    $display("BLOCK01 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Block 1 record/configuration checks failed");
    $finish;
  end
endmodule
```
