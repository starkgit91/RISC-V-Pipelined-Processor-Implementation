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
