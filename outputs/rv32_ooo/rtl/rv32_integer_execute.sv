// Registered integer/control/address execution with one retained output slot.
// Port 0 enables control/address work; port 1 reuses the integer-only configuration.
`timescale 1ns / 1ps

module rv32_integer_execute
  import rv32_ooo_pkg::*;
#(
  parameter bit CONTROL_ENABLE = 1'b1
) (
  input  logic            clk_i,
  input  logic            rst_ni,
  input  logic            request_valid_i,
  output logic            request_ready_o,
  input  execute_packet_t request_i,
  output logic            completion_valid_o,
  input  logic            completion_ready_i,
  output completion_t     completion_o,
  output logic            memory_valid_o,
  input  logic            memory_ready_i,
  output memory_entry_t   memory_o,
  input  logic            cancel_i,
  output logic            cancel_done_o,
  output logic            busy_o,
  output logic            quiescent_o,
  output logic            protocol_error_o
);

  logic valid_q, is_memory_q;
  completion_t completion_q, calculated_completion;
  memory_entry_t memory_q, calculated_memory;
  word_t operand_a, operand_b, alu_result;
  logic branch_taken;
  logic calculated_is_memory, supported, bad_size;

  // These muxes select architectural operands; PRF reads remain raw register values.
  always_comb begin
    case (request_i.uop.decoded.src_a)
      SRC_A_ZERO: operand_a = '0;
      SRC_A_PC:   operand_a = request_i.uop.decoded.fetch.pc;
      default:    operand_a = request_i.rs1_value;
    endcase
    operand_b = request_i.uop.decoded.src_b == SRC_B_IMMEDIATE
                ? request_i.uop.decoded.immediate : request_i.rs2_value;
  end

  rv32_alu alu (
    .a_i(operand_a), .b_i(operand_b), .op_i(request_i.uop.decoded.alu_op),
    .result_o(alu_result)
  );
  rv32_branch_compare comparator (
    .rs1_i(request_i.rs1_value), .rs2_i(request_i.rs2_value),
    .op_i(request_i.uop.decoded.branch_op), .taken_o(branch_taken)
  );

  always_comb begin : calculate
    decoded_uop_t decoded;
    word_t sequential_pc, target, address;
    logic target_taken, misaligned;

    decoded = request_i.uop.decoded;
    sequential_pc = decoded.fetch.pc + 32'd4;
    target = sequential_pc;
    address = request_i.rs1_value + decoded.immediate;
    target_taken = 1'b0;
    misaligned = 1'b0;
    bad_size = 1'b0;
    supported = decoded.op_class == UOP_INTEGER;
    calculated_is_memory = CONTROL_ENABLE
                           && (decoded.op_class == UOP_LOAD || decoded.op_class == UOP_STORE);
    calculated_completion = '0;
    calculated_completion.rob_tag = request_i.uop.rob_tag;
    calculated_completion.pdst = request_i.uop.pdst;
    calculated_completion.writes_rd = decoded.writes_rd;
    calculated_completion.exception = decoded.fetch.exception;
    calculated_completion.control_flow.next_pc = sequential_pc;

    case (decoded.op_class)
      UOP_INTEGER: calculated_completion.value = alu_result;
      UOP_BRANCH: begin
        supported = CONTROL_ENABLE;
        calculated_completion.control_flow.is_control = 1'b1;
        calculated_completion.control_flow.is_conditional = 1'b1;
        target_taken = branch_taken;
        target = decoded.fetch.pc + decoded.immediate;
      end
      UOP_JUMP: begin
        supported = CONTROL_ENABLE;
        calculated_completion.control_flow.is_control = 1'b1;
        target_taken = 1'b1;
        if (decoded.jump_indirect)
          target = (request_i.rs1_value + decoded.immediate) & 32'hffff_fffe;
        else target = decoded.fetch.pc + decoded.immediate;
        calculated_completion.value = sequential_pc;
      end
      UOP_LOAD, UOP_STORE: begin
        supported = CONTROL_ENABLE;
        case (decoded.mem_size)
          MEM_BYTE: misaligned = 1'b0;
          MEM_HALF: misaligned = address[0];
          MEM_WORD: misaligned = |address[1:0];
          default: bad_size = 1'b1;
        endcase
      end
      default: supported = 1'b0;
    endcase

    calculated_completion.control_flow.taken = target_taken;
    if (target_taken) calculated_completion.control_flow.next_pc = target;
    if (!decoded.fetch.exception.valid) begin
      if (!supported || bad_size) begin
        calculated_completion.exception.valid = 1'b1;
        calculated_completion.exception.cause = CAUSE_ILLEGAL_INSTR;
        calculated_completion.exception.tval = decoded.fetch.instruction;
      end else if (target_taken && |target[1:0]) begin
        // IALIGN=32: JALR has already cleared bit 0 before this check.
        calculated_completion.exception.valid = 1'b1;
        calculated_completion.exception.cause = CAUSE_INSTR_MISALIGNED;
        calculated_completion.exception.tval = target;
      end else if (calculated_is_memory && misaligned) begin
        calculated_completion.exception.valid = 1'b1;
        calculated_completion.exception.cause = decoded.op_class == UOP_STORE
                                                ? CAUSE_STORE_MISALIGNED
                                                : CAUSE_LOAD_MISALIGNED;
        calculated_completion.exception.tval = address;
      end
    end
    calculated_completion.control_flow.mispredicted = !calculated_completion.exception.valid
      && calculated_completion.control_flow.next_pc != decoded.fetch.prediction.next_pc;
    if (calculated_completion.exception.valid) calculated_completion.value = '0;

    // Preparation retains the original byte address and unshifted store value.
    // No memory request, register write, or architectural completion occurs here.
    calculated_memory = '0;
    calculated_memory.valid = 1'b1;
    calculated_memory.prepared = 1'b1;
    calculated_memory.rob_tag = request_i.uop.rob_tag;
    calculated_memory.is_store = decoded.op_class == UOP_STORE;
    calculated_memory.size = decoded.mem_size;
    calculated_memory.load_unsigned = decoded.load_unsigned;
    calculated_memory.pdst = request_i.uop.pdst;
    calculated_memory.writes_rd = decoded.writes_rd;
    calculated_memory.address = address;
    calculated_memory.store_value = request_i.rs2_value;
    calculated_memory.exception = calculated_completion.exception;
    calculated_memory.control_flow = calculated_completion.control_flow;
  end

  // A slot can be popped and replaced on the same edge, sustaining one per cycle.
  // Capacity is independent of the offered payload, as required by the IQ selector.
  assign request_ready_o = rst_ni && !cancel_i
                           && (!valid_q || (is_memory_q ? memory_ready_i
                                                        : completion_ready_i));
  assign completion_valid_o = rst_ni && !cancel_i && valid_q && !is_memory_q;
  assign memory_valid_o = rst_ni && !cancel_i && valid_q && is_memory_q;
  assign completion_o = completion_valid_o ? completion_q : completion_t'('0);
  assign memory_o = memory_valid_o ? memory_q : memory_entry_t'('0);
  assign busy_o = valid_q;
  assign quiescent_o = !valid_q;
  assign protocol_error_o = rst_ni && !cancel_i && request_valid_i
                            && (!request_ready_o || !supported || bad_size);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      valid_q <= 1'b0;
      is_memory_q <= 1'b0;
      completion_q <= '0;
      memory_q <= '0;
      cancel_done_o <= 1'b0;
    end else begin
      cancel_done_o <= cancel_i;
      if (cancel_i) begin
        valid_q <= 1'b0;
        is_memory_q <= 1'b0;
        completion_q <= '0;
        memory_q <= '0;
      end else if (request_ready_o) begin
        valid_q <= request_valid_i;
        if (request_valid_i) begin
          is_memory_q <= calculated_is_memory;
          completion_q <= calculated_completion;
          memory_q <= calculated_memory;
        end
      end
    end
  end
endmodule
