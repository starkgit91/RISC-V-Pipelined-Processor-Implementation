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
