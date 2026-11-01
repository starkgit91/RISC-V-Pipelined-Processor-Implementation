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
