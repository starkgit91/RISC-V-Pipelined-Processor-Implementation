// Execution-unit verification. Behavioral *, /, and % below are test-only oracles.
`timescale 1ns / 1ps

module tb_block07;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1;
  logic v0, v1, vm, r0, r1, rm, c0v, c1v, cmv, c0r, c1r, cmr;
  logic m0v, m1v, m0r, m1r, cancel0, cancel1, cancelm;
  logic done0, done1, donem, busy0, busy1, busym, quiet0, quiet1, quietm;
  logic error0, error1, errorm;
  execute_packet_t p0, p1, pm, packet, saved_packet;
  completion_t c0, c1, cm, expected, held_completion;
  memory_entry_t m0, m1, held_memory;
  typedef union packed { mem_size_e encoded; logic [1:0] bits; } raw_size_t;
  raw_size_t bad_size;
  word_t vectors [0:15];
  word_t rng = 32'h5a31beef;
  integer checks = 0, failures = 0, serial_number = 1;
  integer a, b, op, port_number, sample, wait_count, phase, mode;
  logic waves;

  rv32_integer_execute #(.CONTROL_ENABLE(1'b1)) integer0 (
    .clk_i(clk), .rst_ni(rst_n), .request_valid_i(v0), .request_ready_o(r0), .request_i(p0),
    .completion_valid_o(c0v), .completion_ready_i(c0r), .completion_o(c0),
    .memory_valid_o(m0v), .memory_ready_i(m0r), .memory_o(m0),
    .cancel_i(cancel0), .cancel_done_o(done0), .busy_o(busy0), .quiescent_o(quiet0),
    .protocol_error_o(error0)
  );
  rv32_integer_execute #(.CONTROL_ENABLE(1'b0)) integer1 (
    .clk_i(clk), .rst_ni(rst_n), .request_valid_i(v1), .request_ready_o(r1), .request_i(p1),
    .completion_valid_o(c1v), .completion_ready_i(c1r), .completion_o(c1),
    .memory_valid_o(m1v), .memory_ready_i(m1r), .memory_o(m1),
    .cancel_i(cancel1), .cancel_done_o(done1), .busy_o(busy1), .quiescent_o(quiet1),
    .protocol_error_o(error1)
  );
  rv32_muldiv_execute multiply_divide (
    .clk_i(clk), .rst_ni(rst_n), .request_valid_i(vm), .request_ready_o(rm), .request_i(pm),
    .completion_valid_o(cmv), .completion_ready_i(cmr), .completion_o(cm),
    .cancel_i(cancelm), .cancel_done_o(donem), .busy_o(busym), .quiescent_o(quietm),
    .protocol_error_o(errorm)
  );

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 40) $display("FAIL: %s time=%0t", label_text, $time);
      end
    end
  endtask

  task automatic edge_step;
    begin #1; clk = 1; #1; clk = 0; #1; end
  endtask

  task automatic neutral;
    begin
      v0 = 0; v1 = 0; vm = 0; p0 = '0; p1 = '0; pm = '0;
      c0r = 0; c1r = 0; cmr = 0; m0r = 0; m1r = 0;
      cancel0 = 0; cancel1 = 0; cancelm = 0;
    end
  endtask

  function automatic word_t next_random(input word_t value);
    next_random = {value[30:0], value[31] ^ value[21] ^ value[1] ^ value[0]};
  endfunction

  function automatic alu_op_e integer_op(input integer selector);
    case (selector)
      0: integer_op = ALU_ADD;
      1: integer_op = ALU_SUB;
      2: integer_op = ALU_SLL;
      3: integer_op = ALU_SLT;
      4: integer_op = ALU_SLTU;
      5: integer_op = ALU_XOR;
      6: integer_op = ALU_SRL;
      7: integer_op = ALU_SRA;
      8: integer_op = ALU_OR;
      9: integer_op = ALU_AND;
      default: integer_op = ALU_COPY_B;
    endcase
  endfunction

  function automatic muldiv_op_e multiply_op(input integer selector);
    case (selector)
      0: multiply_op = MD_MUL;
      1: multiply_op = MD_MULH;
      2: multiply_op = MD_MULHSU;
      3: multiply_op = MD_MULHU;
      4: multiply_op = MD_DIV;
      5: multiply_op = MD_DIVU;
      6: multiply_op = MD_REM;
      default: multiply_op = MD_REMU;
    endcase
  endfunction

  function automatic branch_op_e branch_op(input integer selector);
    case (selector)
      0: branch_op = BRANCH_EQ;
      1: branch_op = BRANCH_NE;
      2: branch_op = BRANCH_LT;
      3: branch_op = BRANCH_GE;
      4: branch_op = BRANCH_LTU;
      default: branch_op = BRANCH_GEU;
    endcase
  endfunction

  function automatic execute_packet_t operation(input integer id);
    execute_packet_t result;
    begin
      result = '0;
      result.uop.rob_tag.index = id % ROB_DEPTH;
      result.uop.rob_tag.generation = id / ROB_DEPTH;
      result.uop.pdst = 32 + id % 32;
      result.uop.old_pdst = 1 + id % 31;
      result.uop.prs1 = 1 + (id * 3) % 63;
      result.uop.prs2 = 1 + (id * 7) % 63;
      result.uop.decoded.fetch.pc = 32'h200 + id * 4;
      result.uop.decoded.fetch.instruction = 32'h002080b3 ^ (id << 7);
      result.uop.decoded.fetch.prediction.next_pc = result.uop.decoded.fetch.pc + 4;
      if (id % 3 == 0)
        result.uop.decoded.fetch.prediction.next_pc = result.uop.decoded.fetch.pc + 16;
      result.uop.decoded.fetch.prediction.taken = id % 2;
      result.uop.decoded.fetch.prediction.btb_hit = id % 2;
      result.uop.decoded.fetch.prediction.pht_index = id % PHT_ENTRIES;
      result.uop.decoded.op_class = UOP_INTEGER;
      result.uop.decoded.alu_op = ALU_ADD;
      result.uop.decoded.uses_rs1 = 1;
      result.uop.decoded.uses_rs2 = 1;
      result.uop.decoded.writes_rd = 1;
      result.uop.decoded.rs1 = id % 32;
      result.uop.decoded.rs2 = (id + 3) % 32;
      result.uop.decoded.rd = 1 + id % 31;
      if (id % 7 == 0) begin
        result.uop.decoded.writes_rd = 0;
        result.uop.decoded.rd = 0;
        result.uop.pdst = 0;
      end
      result.rs1_value = 32'h12345678;
      result.rs2_value = 32'habcd0123;
      operation = result;
    end
  endfunction

  function automatic word_t alu_reference(input execute_packet_t instruction);
    word_t lhs, rhs;
    begin
      lhs = instruction.rs1_value;
      if (instruction.uop.decoded.src_a == SRC_A_ZERO) lhs = 0;
      else if (instruction.uop.decoded.src_a == SRC_A_PC)
        lhs = instruction.uop.decoded.fetch.pc;
      rhs = instruction.uop.decoded.src_b == SRC_B_IMMEDIATE ?
            instruction.uop.decoded.immediate : instruction.rs2_value;
      case (instruction.uop.decoded.alu_op)
        ALU_ADD: alu_reference = lhs + rhs;
        ALU_SUB: alu_reference = lhs - rhs;
        ALU_SLL: alu_reference = lhs << (rhs % 32);
        ALU_SLT: alu_reference = $signed(lhs) < $signed(rhs) ? 1 : 0;
        ALU_SLTU: alu_reference = lhs < rhs ? 1 : 0;
        ALU_XOR: alu_reference = lhs ^ rhs;
        ALU_SRL: alu_reference = lhs >> (rhs % 32);
        ALU_SRA: alu_reference = $signed(lhs) >>> (rhs % 32);
        ALU_OR: alu_reference = lhs | rhs;
        ALU_AND: alu_reference = lhs & rhs;
        ALU_COPY_B: alu_reference = rhs;
        default: alu_reference = 0;
      endcase
    end
  endfunction

  function automatic logic branch_reference(input execute_packet_t instruction);
    word_t lhs, rhs;
    begin
      lhs = instruction.rs1_value; rhs = instruction.rs2_value;
      case (instruction.uop.decoded.branch_op)
        BRANCH_EQ: branch_reference = lhs == rhs;
        BRANCH_NE: branch_reference = lhs != rhs;
        BRANCH_LT: branch_reference = $signed(lhs) < $signed(rhs);
        BRANCH_GE: branch_reference = $signed(lhs) >= $signed(rhs);
        BRANCH_LTU: branch_reference = lhs < rhs;
        BRANCH_GEU: branch_reference = lhs >= rhs;
        default: branch_reference = 0;
      endcase
    end
  endfunction

  function automatic word_t multiply_reference(input execute_packet_t instruction);
    logic signed [65:0] left_wide, right_wide, signed_product;
    logic [63:0] unsigned_product;
    logic signed [63:0] signed_left, signed_right, quotient, remainder_value;
    word_t lhs, rhs;
    begin
      lhs = instruction.rs1_value; rhs = instruction.rs2_value;
      left_wide = {{34{lhs[31]}}, lhs};
      right_wide = {{34{rhs[31]}}, rhs};
      signed_left = {{32{lhs[31]}}, lhs};
      signed_right = {{32{rhs[31]}}, rhs};
      unsigned_product = {32'b0, lhs} * {32'b0, rhs};
      signed_product = left_wide * right_wide;
      quotient = 0; remainder_value = 0;
      case (instruction.uop.decoded.muldiv_op)
        MD_MUL: multiply_reference = unsigned_product[31:0];
        MD_MULH: multiply_reference = signed_product[63:32];
        MD_MULHSU: begin
          right_wide = {34'b0, rhs};
          signed_product = left_wide * right_wide;
          multiply_reference = signed_product[63:32];
        end
        MD_MULHU: multiply_reference = unsigned_product[63:32];
        MD_DIV, MD_REM: begin
          if (rhs == 0) begin
            quotient = -1;
            remainder_value = signed_left;
          end else begin
            quotient = signed_left / signed_right;
            remainder_value = signed_left % signed_right;
          end
          if (instruction.uop.decoded.muldiv_op == MD_DIV)
            multiply_reference = quotient[31:0];
          else multiply_reference = remainder_value[31:0];
        end
        MD_DIVU: multiply_reference = rhs == 0 ? 32'hffffffff : lhs / rhs;
        MD_REMU: multiply_reference = rhs == 0 ? lhs : lhs % rhs;
        default: multiply_reference = 0;
      endcase
    end
  endfunction

  function automatic completion_t integer_reference(
    input execute_packet_t instruction, input logic controls
  );
    completion_t result;
    logic taken, supported;
    word_t target;
    begin
      result = '0;
      result.rob_tag = instruction.uop.rob_tag;
      result.pdst = instruction.uop.pdst;
      result.writes_rd = instruction.uop.decoded.writes_rd;
      result.exception = instruction.uop.decoded.fetch.exception;
      result.control_flow.next_pc = instruction.uop.decoded.fetch.pc + 4;
      target = instruction.uop.decoded.fetch.pc + instruction.uop.decoded.immediate;
      taken = branch_reference(instruction);
      supported = instruction.uop.decoded.op_class == UOP_INTEGER ||
                  (controls && (instruction.uop.decoded.op_class == UOP_BRANCH ||
                                instruction.uop.decoded.op_class == UOP_JUMP));
      case (instruction.uop.decoded.op_class)
        UOP_INTEGER: result.value = alu_reference(instruction);
        UOP_BRANCH: if (controls) begin
          result.control_flow.is_control = 1;
          result.control_flow.is_conditional = 1;
          result.control_flow.taken = taken;
          if (taken) result.control_flow.next_pc = target;
        end
        UOP_JUMP: if (controls) begin
          if (instruction.uop.decoded.jump_indirect)
            target = (instruction.rs1_value + instruction.uop.decoded.immediate) & 32'hfffffffe;
          result.value = instruction.uop.decoded.fetch.pc + 4;
          result.control_flow.is_control = 1;
          result.control_flow.taken = 1;
          result.control_flow.next_pc = target;
        end
        default: begin end
      endcase
      if (!result.exception.valid) begin
        if (!supported) begin
          result.exception.valid = 1;
          result.exception.cause = CAUSE_ILLEGAL_INSTR;
          result.exception.tval = instruction.uop.decoded.fetch.instruction;
        end else if (result.control_flow.taken && result.control_flow.next_pc[1:0] != 0) begin
          result.exception.valid = 1;
          result.exception.cause = CAUSE_INSTR_MISALIGNED;
          result.exception.tval = result.control_flow.next_pc;
        end
      end
      result.control_flow.mispredicted = !result.exception.valid &&
        result.control_flow.next_pc != instruction.uop.decoded.fetch.prediction.next_pc;
      integer_reference = result;
    end
  endfunction

  function automatic memory_entry_t memory_reference(input execute_packet_t instruction);
    memory_entry_t result;
    logic bad_alignment;
    begin
      result = '0;
      result.valid = 1; result.prepared = 1;
      result.rob_tag = instruction.uop.rob_tag;
      result.is_store = instruction.uop.decoded.op_class == UOP_STORE;
      result.size = instruction.uop.decoded.mem_size;
      result.load_unsigned = instruction.uop.decoded.load_unsigned;
      result.pdst = instruction.uop.pdst;
      result.writes_rd = instruction.uop.decoded.writes_rd;
      result.address = instruction.rs1_value + instruction.uop.decoded.immediate;
      result.store_value = instruction.rs2_value;
      result.exception = instruction.uop.decoded.fetch.exception;
      result.control_flow.next_pc = instruction.uop.decoded.fetch.pc + 4;
      bad_alignment = (result.size == MEM_HALF && result.address[0]) ||
                      (result.size == MEM_WORD && result.address[1:0] != 0);
      if (!result.exception.valid) begin
        if (result.size != MEM_BYTE && result.size != MEM_HALF && result.size != MEM_WORD) begin
          result.exception.valid = 1;
          result.exception.cause = CAUSE_ILLEGAL_INSTR;
          result.exception.tval = instruction.uop.decoded.fetch.instruction;
        end else if (bad_alignment) begin
          result.exception.valid = 1;
          result.exception.cause = result.is_store ? CAUSE_STORE_MISALIGNED : CAUSE_LOAD_MISALIGNED;
          result.exception.tval = result.address;
        end
      end
      result.control_flow.mispredicted = !result.exception.valid &&
        result.control_flow.next_pc != instruction.uop.decoded.fetch.prediction.next_pc;
      memory_reference = result;
    end
  endfunction

  function automatic completion_t md_reference(input execute_packet_t instruction);
    completion_t result;
    begin
      result = '0;
      result.rob_tag = instruction.uop.rob_tag;
      result.pdst = instruction.uop.pdst;
      result.writes_rd = instruction.uop.decoded.writes_rd;
      result.value = multiply_reference(instruction);
      result.exception = instruction.uop.decoded.fetch.exception;
      result.control_flow.next_pc = instruction.uop.decoded.fetch.pc + 4;
      if (!result.exception.valid && instruction.uop.decoded.op_class != UOP_MULDIV) begin
        result.exception.valid = 1;
        result.exception.cause = CAUSE_ILLEGAL_INSTR;
        result.exception.tval = instruction.uop.decoded.fetch.instruction;
      end
      result.control_flow.mispredicted = !result.exception.valid &&
        result.control_flow.next_pc != instruction.uop.decoded.fetch.prediction.next_pc;
      md_reference = result;
    end
  endfunction

  task automatic compare_completion(
    input completion_t actual, reference, input string label_text
  );
    completion_t compared_actual, compared_reference;
    begin
      compared_actual = actual; compared_reference = reference;
      // A faulting result value is architecturally unusable; all ownership/fault fields matter.
      if (reference.exception.valid) begin
        compared_actual.value = 0; compared_reference.value = 0;
        check(!actual.control_flow.mispredicted, "fault result cannot request branch recovery");
        if (reference.exception.cause == CAUSE_ILLEGAL_INSTR) begin
          // A misrouted instruction traps; its unused target fields have no architectural effect.
          compared_actual.control_flow = '0; compared_reference.control_flow = '0;
        end
      end
      check(compared_actual == compared_reference, label_text);
      if (compared_actual !== compared_reference && failures <= 40)
        $display("  actual=%h expected=%h", compared_actual, compared_reference);
    end
  endtask

  task automatic integer_case(
    input integer port_id, input execute_packet_t instruction, input integer hold_cycles
  );
    completion_t reference, held;
    memory_entry_t memory_expected, memory_held;
    logic is_memory, actual_ready, actual_error;
    begin
      is_memory = port_id == 0 && (instruction.uop.decoded.op_class == UOP_LOAD ||
                                  instruction.uop.decoded.op_class == UOP_STORE);
      reference = integer_reference(instruction, port_id == 0);
      memory_expected = memory_reference(instruction);
      if (port_id == 0) begin p0 = instruction; v0 = 1; end
      else begin p1 = instruction; v1 = 1; end
      #1;
      actual_ready = port_id == 0 ? r0 : r1;
      actual_error = port_id == 0 ? error0 : error1;
      check(actual_ready, "empty integer sink accepts an issue packet");
      if (is_memory && instruction.uop.decoded.mem_size != MEM_BYTE &&
          instruction.uop.decoded.mem_size != MEM_HALF &&
          instruction.uop.decoded.mem_size != MEM_WORD)
        check(actual_error, "unsupported memory width is diagnosed without a bus request");
      else if (is_memory || reference.exception.cause != CAUSE_ILLEGAL_INSTR ||
               instruction.uop.decoded.fetch.exception.valid)
        check(!actual_error, "well-routed integer/control/address request has no diagnostic");
      else check(actual_error, "misrouted class is diagnosed");
      edge_step;
      if (port_id == 0) v0 = 0; else v1 = 0;
      #1;
      if (is_memory) begin
        check(m0v && !c0v && busy0 && !quiet0, "address result occupies the memory output slot");
        check(m0 == memory_expected,
              "prepared address/data/fault/next-PC metadata match reference");
        memory_held = m0;
      end else begin
        check((port_id == 0 ? c0v : c1v) && !(port_id == 0 ? m0v : m1v),
              "integer result occupies only the completion output");
        compare_completion(port_id == 0 ? c0 : c1, reference, "integer result and owner metadata");
        held = port_id == 0 ? c0 : c1;
      end
      for (integer count = 0; count < hold_cycles; count = count + 1) begin
        if (port_id == 0) p0 = operation(8000+count); else p1 = operation(9000+count);
        #1;
        check(!(port_id == 0 ? r0 : r1), "occupied integer sink applies backpressure");
        if (is_memory) check(m0v && m0 == memory_held, "address packet stable under backpressure");
        else check((port_id == 0 ? c0v : c1v) && (port_id == 0 ? c0 : c1) == held,
                   "completion packet stable under backpressure");
        edge_step;
      end
      if (is_memory) m0r = 1;
      else if (port_id == 0) c0r = 1;
      else c1r = 1;
      #1; check(port_id == 0 ? r0 : r1, "integer slot supports consumption-edge replacement");
      edge_step;
      c0r = 0; c1r = 0; m0r = 0; m1r = 0;
      #1;
      check((port_id == 0 ? quiet0 : quiet1) && !(port_id == 0 ? busy0 : busy1),
            "consumed integer/address result leaves a quiescent unit");
    end
  endtask

  task automatic md_case(input execute_packet_t instruction, input integer hold_cycles);
    completion_t reference, held;
    integer clocks;
    logic immediate_fault;
    begin
      reference = md_reference(instruction);
      immediate_fault = instruction.uop.decoded.fetch.exception.valid ||
                        instruction.uop.decoded.op_class != UOP_MULDIV;
      pm = instruction; vm = 1; #1;
      check(rm && quietm && !busym, "idle iterative unit accepts one owner");
      if (!immediate_fault) check(!errorm, "valid RV32M request has no protocol diagnostic");
      edge_step; vm = 0; #1;
      clocks = 0;
      while (!cmv && clocks < 40) begin
        check(busym && !quietm && !rm, "iterative owner excludes a second request");
        pm = operation(7000+clocks);
        edge_step; clocks = clocks + 1;
      end
      check(cmv, "iterative completion arrives within timeout");
      if (!immediate_fault) check(clocks == 32, "valid RV32M performs exactly 32 iteration edges");
      else check(clocks == 0, "malformed/fault token completes without arithmetic iterations");
      compare_completion(cm, reference, "RV32M result, exception, owner, and sequential PC");
      held = cm;
      for (integer count = 0; count < hold_cycles; count = count + 1) begin
        #1; check(cmv && cm == held && !rm && busym && !quietm,
                  "iterative result remains stable until completion acceptance");
        pm = operation(6000+count); edge_step;
      end
      cmr = 1; #1;
      check(!rm, "iterative result pop does not promise same-edge request replacement");
      edge_step; cmr = 0; #1;
      check(rm && quietm && !busym && !cmv, "iterative result consumed exactly once");
    end
  endtask

  task automatic reset_units;
    begin
      neutral; rst_n = 0; #2;
      check(!r0 && !r1 && !rm && !c0v && !c1v && !cmv && !m0v && !m1v,
            "asynchronous reset masks all handshakes");
      check(quiet0 && quiet1 && quietm && !busy0 && !busy1 && !busym,
            "asynchronous reset clears executing owners and result buffers");
      rst_n = 1; #1;
      check(r0 && r1 && rm && !done0 && !done1 && !donem,
            "idle resources are available after reset release");
    end
  endtask

  initial begin
    waves = $test$plusargs("waves");
    if (waves) begin
      $dumpfile("build/block07/block07.vcd");
      $dumpvars(0, tb_block07);
    end
    #3000000; $fatal(1, "Block 7 unit tests timed out");
  end

  initial begin
    vectors[0] = 0; vectors[1] = 1; vectors[2] = 2; vectors[3] = 3;
    vectors[4] = 32'hffffffff; vectors[5] = 32'hfffffffe;
    vectors[6] = 32'h80000000; vectors[7] = 32'h7fffffff;
    vectors[8] = 32'h80000001; vectors[9] = 32'h40000000;
    vectors[10] = 32'h00010000; vectors[11] = 32'hffff0000;
    vectors[12] = 31; vectors[13] = 32; vectors[14] = 33; vectors[15] = 32'h55555555;
    reset_units;

    // Lead example: a long DIV owns the iterative path while an independent ADD completes.
    pm = operation(serial_number); serial_number = serial_number + 1;
    pm.uop.decoded.op_class = UOP_MULDIV; pm.uop.decoded.muldiv_op = MD_DIV;
    pm.rs1_value = 100; pm.rs2_value = 3; saved_packet = pm; vm = 1; edge_step; vm = 0;
    packet = operation(serial_number); serial_number = serial_number + 1;
    packet.rs1_value = 7; packet.rs2_value = 8; p0 = packet; v0 = 1; edge_step; v0 = 0;
    #1; check(c0v && c0.value == 15 && busym && !cmv,
              "independent ADD completes while older DIV is still iterating");
    c0r = 1; edge_step; c0r = 0;
    wait_count = 0;
    while (!cmv && wait_count < 40) begin edge_step; wait_count = wait_count + 1; end
    check(cmv && cm.value == 33, "older DIV eventually completes with its own result");
    compare_completion(cm, md_reference(saved_packet), "overlapped DIV retains its original owner");
    cmr = 1; edge_step; cmr = 0;

    // Exercise all ALU operations, signed boundaries, shifts beyond 31, and both integer lanes.
    for (port_number = 0; port_number < 2; port_number = port_number + 1) begin
      for (op = 0; op < 11; op = op + 1) begin
        for (sample = 0; sample < 32; sample = sample + 1) begin
          packet = operation(serial_number); serial_number = serial_number + 1;
          packet.uop.decoded.alu_op = integer_op(op);
          packet.rs1_value = vectors[sample % 16];
          packet.rs2_value = vectors[(sample * 7 + op) % 16];
          integer_case(port_number, packet, sample % 11 == 0 ? 2 : 0);
        end
      end
    end
    // All operand-mux combinations cover PC-relative AUIPC and immediate/zero LUI paths.
    for (a = 0; a < 3; a = a + 1) begin
      for (b = 0; b < 2; b = b + 1) begin
        for (op = 0; op < 11; op = op + 1) begin
          packet = operation(serial_number); serial_number = serial_number + 1;
          packet.uop.decoded.alu_op = integer_op(op);
          if (a == 1) packet.uop.decoded.src_a = SRC_A_ZERO;
          if (a == 2) packet.uop.decoded.src_a = SRC_A_PC;
          if (b == 1) packet.uop.decoded.src_b = SRC_B_IMMEDIATE;
          packet.uop.decoded.immediate = 32'hfffff01f;
          integer_case(op % 2, packet, 0);
        end
      end
    end

    // Branch direction is independent of ALU operand selection; only taken targets align-trap.
    for (op = 0; op < 6; op = op + 1) begin
      for (a = 0; a < 16; a = a + 1) begin
        for (b = 0; b < 4; b = b + 1) begin
          packet = operation(serial_number); serial_number = serial_number + 1;
          packet.uop.decoded.op_class = UOP_BRANCH;
          packet.uop.decoded.branch_op = branch_op(op);
          packet.uop.decoded.writes_rd = 0; packet.uop.pdst = 0;
          packet.rs1_value = vectors[a]; packet.rs2_value = vectors[(a+b) % 16];
          packet.uop.decoded.immediate = b % 2 == 0 ? 32'hfffffff8 : 32'd6;
          integer_case(0, packet, a == 0 ? 1 : 0);
        end
      end
    end
    // JAL/JALR wraparound, bit-zero clearing, and bit-one misalignment.
    for (mode = 0; mode < 2; mode = mode + 1) begin
      for (a = 0; a < 16; a = a + 1) begin
        packet = operation(serial_number); serial_number = serial_number + 1;
        packet.uop.decoded.op_class = UOP_JUMP;
        packet.uop.decoded.jump_indirect = mode;
        packet.uop.decoded.fetch.pc = 32'hfffffffc;
        packet.uop.decoded.immediate = a;
        packet.rs1_value = vectors[a];
        integer_case(0, packet, a % 5 == 0 ? 2 : 0);
      end
    end

    // Address preparation preserves byte addresses, data, ownership, and next-PC recovery data.
    for (mode = 0; mode < 2; mode = mode + 1) begin
      for (op = 0; op < 3; op = op + 1) begin
        for (a = 0; a < 8; a = a + 1) begin
          packet = operation(serial_number); serial_number = serial_number + 1;
          if (mode == 0) packet.uop.decoded.op_class = UOP_LOAD;
          else packet.uop.decoded.op_class = UOP_STORE;
          if (op == 0) packet.uop.decoded.mem_size = MEM_BYTE;
          if (op == 1) packet.uop.decoded.mem_size = MEM_HALF;
          if (op == 2) packet.uop.decoded.mem_size = MEM_WORD;
          packet.uop.decoded.load_unsigned = a % 2;
          packet.uop.decoded.immediate = -4;
          packet.rs1_value = 32'h1004 + a;
          packet.rs2_value = 32'h91abcdef ^ a;
          if (mode == 1) begin packet.uop.decoded.writes_rd = 0; packet.uop.pdst = 0; end
          integer_case(0, packet, a % 3 == 0 ? 2 : 0);
        end
      end
    end

    packet = operation(serial_number); serial_number = serial_number + 1;
    packet.uop.decoded.op_class = UOP_LOAD;
    packet.uop.decoded.mem_size = MEM_WORD;
    bad_size.bits = 2'b11; // Test the reserved width without relying on enum casts.
    packet.uop.decoded.mem_size = bad_size.encoded;
    integer_case(0, packet, 2);

    // Fetch exceptions retain precedence over new target/alignment/unsupported-operation faults.
    for (mode = 0; mode < 3; mode = mode + 1) begin
      packet = operation(serial_number); serial_number = serial_number + 1;
      packet.uop.decoded.fetch.exception.valid = 1;
      packet.uop.decoded.fetch.exception.cause = CAUSE_INSTR_ACCESS;
      packet.uop.decoded.fetch.exception.tval = 32'hfeed0010;
      if (mode == 1) begin
        packet.uop.decoded.op_class = UOP_JUMP;
        packet.uop.decoded.immediate = 2;
      end
      if (mode == 2) begin
        packet.uop.decoded.op_class = UOP_LOAD;
        packet.uop.decoded.mem_size = MEM_WORD; packet.rs1_value = 3;
      end
      integer_case(0, packet, 2);
    end
    packet = operation(serial_number); serial_number = serial_number + 1;
    packet.uop.decoded.op_class = UOP_BRANCH; packet.uop.decoded.branch_op = BRANCH_EQ;
    integer_case(1, packet, 1);
    packet.uop.decoded.op_class = UOP_SYSTEM; integer_case(0, packet, 1);
    packet.uop.decoded.op_class = UOP_MULDIV; integer_case(0, packet, 1);
    packet.uop.decoded.op_class = UOP_LOAD; integer_case(1, packet, 1);
    packet = operation(serial_number); serial_number = serial_number + 1;
    packet.uop.decoded.fetch.pc = 32'hfffffffc;
    packet.uop.decoded.fetch.prediction.next_pc = 0;
    packet.uop.decoded.src_a = SRC_A_PC; packet.uop.decoded.src_b = SRC_B_IMMEDIATE;
    packet.uop.decoded.immediate = 8; integer_case(0, packet, 1);

    // Only the selected output consumer can release the common integer/address slot.
    p0 = operation(serial_number); serial_number = serial_number + 1;
    saved_packet = p0; v0 = 1; edge_step; v0 = 0; held_completion = c0;
    m0r = 1; #1;
    check(c0v && !r0, "memory acceptance cannot pop a buffered arithmetic completion");
    edge_step; check(c0 == held_completion, "wrong consumer leaves arithmetic packet stable");
    p0 = operation(serial_number); serial_number = serial_number + 1;
    p0.uop.decoded.op_class = UOP_LOAD; p0.uop.decoded.mem_size = MEM_WORD;
    p0.rs1_value = 32'h3000; p0.uop.decoded.immediate = 4;
    c0r = 1; v0 = 1; #1; check(r0, "completion consumption permits address replacement");
    edge_step; v0 = 0; c0r = 1; m0r = 0; held_memory = m0;
    check(m0v && !c0v && m0 == memory_reference(p0),
          "completion slot can become an address slot on the same edge");
    #1; check(!r0, "completion acceptance cannot pop a buffered address packet");
    edge_step; check(m0 == held_memory, "wrong consumer leaves address packet stable");
    p0 = operation(serial_number); serial_number = serial_number + 1;
    p0.rs1_value = 40; p0.rs2_value = 3; m0r = 1; v0 = 1; #1;
    check(r0, "address consumption permits arithmetic replacement");
    edge_step; v0 = 0; m0r = 0; c0r = 0;
    check(c0v && !m0v && c0.value == 43, "address slot becomes an arithmetic slot");
    c0r = 1; edge_step; c0r = 0;

    // A protocol-invalid extra issue cannot overwrite an already-held owner.
    p0 = operation(serial_number); serial_number = serial_number + 1;
    v0 = 1; edge_step; held_completion = c0;
    p0 = operation(serial_number); serial_number = serial_number + 1; #1;
    check(!r0 && error0, "issue pulse into unavailable integer resource is diagnosed");
    edge_step; v0 = 0;
    check(c0v && c0 == held_completion, "unaccepted integer issue cannot replace held owner");
    c0r = 1; edge_step; c0r = 0;

    // A consumed completion can be replaced by another instruction on the same clock edge.
    p0 = operation(serial_number); serial_number = serial_number + 1;
    p0.rs1_value = 10; p0.rs2_value = 20; v0 = 1; edge_step;
    check(c0v && c0.value == 30, "integer replacement first owner is buffered");
    p0 = operation(serial_number); serial_number = serial_number + 1;
    p0.rs1_value = 40; p0.rs2_value = 2; c0r = 1; #1;
    check(r0, "integer old-result acceptance permits simultaneous next request");
    edge_step; v0 = 0; c0r = 0;
    check(c0v && c0.value == 42 && c0.rob_tag == p0.uop.rob_tag,
          "same-edge replacement retains the new result and owner");
    c0r = 1; edge_step; c0r = 0;

    // Cancellation dominates an occupied completion or address output and a new request.
    for (mode = 0; mode < 3; mode = mode + 1) begin
      packet = operation(serial_number); serial_number = serial_number + 1;
      if (mode == 2) begin packet.uop.decoded.op_class = UOP_STORE; end
      if (mode == 1) begin p1 = packet; v1 = 1; end else begin p0 = packet; v0 = 1; end
      edge_step; v0 = 0; v1 = 0;
      if (mode == 1) begin cancel1 = 1; v1 = 1; c1r = 1; end
      else begin cancel0 = 1; v0 = 1; c0r = 1; m0r = 1; end
      #1;
      check(mode == 1 ? (!r1 && !c1v && !m1v) : (!r0 && !c0v && !m0v),
            "cancel masks integer/address acceptance and publication immediately");
      edge_step;
      check(mode == 1 ? (done1 && quiet1 && !busy1) : (done0 && quiet0 && !busy0),
            "cancel acknowledges cleared integer/address owner after edge");
      neutral; edge_step;
      check(!c0v && !c1v && !m0v && !m1v && !done0 && !done1,
            "cancelled integer/address result cannot reappear");
    end

    // Every M operation against a complete edge-value cross product plus random operands.
    // Bound waveform size by recording the lead example and early unit checks only.
    if (waves) $dumpoff;
    for (op = 0; op < 8; op = op + 1) begin
      for (a = 0; a < 16; a = a + 1) begin
        for (b = 0; b < 16; b = b + 1) begin
          packet = operation(serial_number); serial_number = serial_number + 1;
          packet.uop.decoded.op_class = UOP_MULDIV;
          packet.uop.decoded.muldiv_op = multiply_op(op);
          packet.rs1_value = vectors[a]; packet.rs2_value = vectors[b];
          md_case(packet, (a+b) % 13 == 0 ? 3 : 0);
        end
      end
      for (sample = 0; sample < 96; sample = sample + 1) begin
        packet = operation(serial_number); serial_number = serial_number + 1;
        packet.uop.decoded.op_class = UOP_MULDIV;
        packet.uop.decoded.muldiv_op = multiply_op(op);
        rng = next_random(rng); packet.rs1_value = rng;
        rng = next_random(rng); packet.rs2_value = rng;
        md_case(packet, sample % 17 == 0 ? 2 : 0);
      end
    end
    packet = operation(serial_number); serial_number = serial_number + 1;
    md_case(packet, 2);
    packet.uop.decoded.op_class = UOP_MULDIV;
    packet.uop.decoded.fetch.exception.valid = 1;
    packet.uop.decoded.fetch.exception.cause = CAUSE_INSTR_ACCESS;
    packet.uop.decoded.fetch.exception.tval = 32'hffff0000;
    md_case(packet, 2);
    packet = operation(serial_number); serial_number = serial_number + 1;
    packet.uop.decoded.op_class = UOP_MULDIV; packet.uop.decoded.muldiv_op = MD_REM;
    packet.uop.decoded.fetch.pc = 32'hfffffffc;
    packet.uop.decoded.fetch.prediction.next_pc = 0;
    packet.rs1_value = -17; packet.rs2_value = 5; md_case(packet, 2);

    pm = operation(serial_number); serial_number = serial_number + 1;
    pm.uop.decoded.op_class = UOP_MULDIV; pm.uop.decoded.muldiv_op = MD_DIVU;
    pm.rs1_value = 27; pm.rs2_value = 4; saved_packet = pm; vm = 1; edge_step;
    pm = operation(serial_number); serial_number = serial_number + 1; #1;
    check(!rm && errorm, "issue pulse into busy iterative resource is diagnosed");
    edge_step; vm = 0; wait_count = 0;
    while (!cmv && wait_count < 40) begin edge_step; wait_count = wait_count + 1; end
    check(cmv, "original iterative owner survives an unavailable extra request");
    compare_completion(cm, md_reference(saved_packet),
                       "busy iterative owner cannot be overwritten");
    cmr = 1; edge_step; cmr = 0;

    // Cancel at every iterative phase, including the last compute edge and held-result state.
    if (waves) $dumpon;
    for (phase = 0; phase <= 32; phase = phase + 1) begin
      pm = operation(serial_number); serial_number = serial_number + 1;
      pm.uop.decoded.op_class = UOP_MULDIV;
      if (phase % 2 == 0) pm.uop.decoded.muldiv_op = MD_DIV;
      else pm.uop.decoded.muldiv_op = MD_MULHSU;
      pm.rs1_value = 32'h81234567; pm.rs2_value = 32'h12345678;
      saved_packet = pm; vm = 1; edge_step; vm = 0;
      for (a = 0; a < phase; a = a + 1) edge_step;
      cancelm = 1; cmr = 1; vm = 1; #1;
      check(!rm && !cmv, "cancel masks iterative result and request on every phase");
      edge_step;
      check(donem && quietm && !busym && !cmv,
            "cancel acknowledges no active or held iterative owner");
      neutral; #1;
      check(rm && quietm, "cancel acknowledgement permits immediate safe owner reuse");
      packet = operation(serial_number); serial_number = serial_number + 1;
      packet.uop.rob_tag = saved_packet.uop.rob_tag;
      packet.uop.pdst = saved_packet.uop.pdst;
      packet.uop.decoded.writes_rd = saved_packet.uop.decoded.writes_rd;
      packet.uop.decoded.op_class = UOP_MULDIV; packet.uop.decoded.muldiv_op = MD_DIVU;
      packet.rs1_value = 35; packet.rs2_value = 5; md_case(packet, 0);
      for (a = 0; a < 34; a = a + 1) begin
        check(!cmv && !busym, "cancelled iterative owner cannot publish a delayed completion");
        edge_step;
      end
    end

    // Reset while an iterative operation and an integer/address result are active.
    pm = operation(serial_number); serial_number = serial_number + 1;
    pm.uop.decoded.op_class = UOP_MULDIV; pm.uop.decoded.muldiv_op = MD_MUL;
    vm = 1; p0 = operation(serial_number); v0 = 1; edge_step;
    reset_units;
    for (a = 0; a < 35; a = a + 1) begin
      check(!cmv && !c0v && !m0v, "reset removed all previous owner publications"); edge_step;
    end

    $display("BLOCK07 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Block 7 execution tests failed");
    $finish;
  end
endmodule
