// EX section: one iterative multiply/divide owner, independent of both integer ALUs.
// Valid M operations take 32 iteration edges; a held result keeps its ROB identity.
`timescale 1ns / 1ps

module rv32_muldiv_execute
  import rv32_ooo_pkg::*;
(
  input  logic            clk_i,
  input  logic            rst_ni,
  input  logic            request_valid_i,
  output logic            request_ready_o,
  input  execute_packet_t request_i,
  output logic            completion_valid_o,
  input  logic            completion_ready_i,
  output completion_t     completion_o,
  input  logic            cancel_i,
  output logic            cancel_done_o,
  output logic            busy_o,
  output logic            quiescent_o,
  output logic            protocol_error_o
);

  typedef enum logic [1:0] {IDLE, MULTIPLY, DIVIDE, RESPONSE} state_e;
  state_e state_q, state_d;
  completion_t result_q, result_d;
  muldiv_op_e operation_q, operation_d;
  logic [4:0] iteration_q, iteration_d;

  logic [63:0] product_q, product_d;
  logic [63:0] multiplicand_q, multiplicand_d;
  word_t multiplier_q, multiplier_d;
  logic negate_product_q, negate_product_d;

  logic [32:0] remainder_q, remainder_d;
  word_t quotient_q, quotient_d;
  word_t divisor_q, divisor_d;
  word_t dividend_q, dividend_d;
  logic negate_quotient_q, negate_quotient_d;
  logic negate_remainder_q, negate_remainder_d;
  logic divide_by_zero_q, divide_by_zero_d;

  // Capacity is advertised before the scheduler chooses an operation. A result
  // must first leave RESPONSE; this engine does not refill on the same pop edge.
  always_comb begin
    request_ready_o = rst_ni && !cancel_i && state_q == IDLE;
    completion_valid_o = rst_ni && !cancel_i && state_q == RESPONSE;
    completion_o = '0;
    if (completion_valid_o) completion_o = result_q;
    busy_o = state_q != IDLE;
    quiescent_o = state_q == IDLE;
    protocol_error_o = rst_ni && !cancel_i && request_valid_i
                       && (!request_ready_o
                           || request_i.uop.decoded.op_class != UOP_MULDIV);
  end

  always_comb begin : arithmetic_and_control
    completion_t incoming;
    logic signed_a, signed_b, negative_a, negative_b;
    word_t magnitude_a, magnitude_b;
    logic [63:0] next_product, corrected_product;
    logic [32:0] trial_remainder, next_remainder;
    word_t next_quotient, corrected_quotient, corrected_remainder;

    state_d = state_q;
    result_d = result_q;
    operation_d = operation_q;
    iteration_d = iteration_q;
    product_d = product_q;
    multiplicand_d = multiplicand_q;
    multiplier_d = multiplier_q;
    negate_product_d = negate_product_q;
    remainder_d = remainder_q;
    quotient_d = quotient_q;
    divisor_d = divisor_q;
    dividend_d = dividend_q;
    negate_quotient_d = negate_quotient_q;
    negate_remainder_d = negate_remainder_q;
    divide_by_zero_d = divide_by_zero_q;

    // Raw operand bits are interpreted as signed only for the selected operation.
    signed_a = request_i.uop.decoded.muldiv_op == MD_MULH
               || request_i.uop.decoded.muldiv_op == MD_MULHSU
               || request_i.uop.decoded.muldiv_op == MD_DIV
               || request_i.uop.decoded.muldiv_op == MD_REM;
    signed_b = request_i.uop.decoded.muldiv_op == MD_MULH
               || request_i.uop.decoded.muldiv_op == MD_DIV
               || request_i.uop.decoded.muldiv_op == MD_REM;
    negative_a = signed_a && request_i.rs1_value[31];
    negative_b = signed_b && request_i.rs2_value[31];
    magnitude_a = negative_a ? (~request_i.rs1_value + 32'd1) : request_i.rs1_value;
    magnitude_b = negative_b ? (~request_i.rs2_value + 32'd1) : request_i.rs2_value;

    incoming = '0;
    incoming.rob_tag = request_i.uop.rob_tag;
    incoming.pdst = request_i.uop.pdst;
    incoming.writes_rd = request_i.uop.decoded.writes_rd;
    incoming.exception = request_i.uop.decoded.fetch.exception;
    incoming.control_flow.next_pc = request_i.uop.decoded.fetch.pc + 32'd4;
    if (!incoming.exception.valid && request_i.uop.decoded.op_class != UOP_MULDIV) begin
      incoming.exception.valid = 1'b1;
      incoming.exception.cause = CAUSE_ILLEGAL_INSTR;
      incoming.exception.tval = request_i.uop.decoded.fetch.instruction;
    end
    incoming.control_flow.mispredicted = !incoming.exception.valid
        && incoming.control_flow.next_pc != request_i.uop.decoded.fetch.prediction.next_pc;

    // Multiply one multiplier bit per cycle. The final correction applies to the
    // whole 64-bit product before selecting its high or low architectural half.
    next_product = product_q;
    if (multiplier_q[0]) next_product = product_q + multiplicand_q;
    corrected_product = negate_product_q ? (~next_product + 64'd1) : next_product;

    // Restoring division shifts one dividend bit into a 33-bit trial remainder.
    // A subtract that fits emits quotient bit 1; otherwise retain the trial value.
    trial_remainder = {remainder_q[31:0], quotient_q[31]};
    next_remainder = trial_remainder;
    next_quotient = {quotient_q[30:0], 1'b0};
    if (trial_remainder >= {1'b0, divisor_q}) begin
      next_remainder = trial_remainder - {1'b0, divisor_q};
      next_quotient[0] = 1'b1;
    end
    corrected_quotient = negate_quotient_q ? (~next_quotient + 32'd1) : next_quotient;
    corrected_remainder = negate_remainder_q
                          ? (~next_remainder[31:0] + 32'd1) : next_remainder[31:0];

    case (state_q)
      IDLE: begin
        if (request_valid_i && request_ready_o) begin
          result_d = incoming;
          operation_d = request_i.uop.decoded.muldiv_op;
          iteration_d = '0;
          if (incoming.exception.valid) begin
            state_d = RESPONSE;
          end else if (!request_i.uop.decoded.muldiv_op[2]) begin
            product_d = '0;
            multiplicand_d = {32'b0, magnitude_a};
            multiplier_d = magnitude_b;
            negate_product_d = negative_a ^ negative_b;
            state_d = MULTIPLY;
          end else begin
            remainder_d = '0;
            quotient_d = magnitude_a;
            divisor_d = magnitude_b;
            dividend_d = request_i.rs1_value;
            negate_quotient_d = negative_a ^ negative_b;
            negate_remainder_d = negative_a;
            divide_by_zero_d = request_i.rs2_value == 0;
            state_d = DIVIDE;
          end
        end
      end
      MULTIPLY: begin
        product_d = next_product;
        multiplicand_d = multiplicand_q << 1;
        multiplier_d = multiplier_q >> 1;
        iteration_d = iteration_q + 5'd1;
        if (iteration_q == 5'd31) begin
          result_d.value = operation_q == MD_MUL
                           ? corrected_product[31:0] : corrected_product[63:32];
          state_d = RESPONSE;
        end
      end
      DIVIDE: begin
        quotient_d = next_quotient;
        remainder_d = next_remainder;
        iteration_d = iteration_q + 5'd1;
        if (iteration_q == 5'd31) begin
          if (divide_by_zero_q) begin
            // RISC-V defines values rather than a trap for integer division by 0.
            result_d.value = operation_q[1] ? dividend_q : 32'hffff_ffff;
          end else begin
            // INT_MIN / -1 also follows this magnitude/sign datapath: INT_MIN, 0.
            result_d.value = operation_q[1] ? corrected_remainder : corrected_quotient;
          end
          state_d = RESPONSE;
        end
      end
      RESPONSE: begin
        if (completion_valid_o && completion_ready_i) state_d = IDLE;
      end
      default: state_d = IDLE;
    endcase
  end

  // Cancellation is an ownership barrier. Clear both arithmetic progress and the
  // held completion so no result can emerge after the acknowledgment edge.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= IDLE;
      result_q <= '0;
      operation_q <= MD_MUL;
      iteration_q <= '0;
      product_q <= '0;
      multiplicand_q <= '0;
      multiplier_q <= '0;
      negate_product_q <= 1'b0;
      remainder_q <= '0;
      quotient_q <= '0;
      divisor_q <= '0;
      dividend_q <= '0;
      negate_quotient_q <= 1'b0;
      negate_remainder_q <= 1'b0;
      divide_by_zero_q <= 1'b0;
      cancel_done_o <= 1'b0;
    end else begin
      cancel_done_o <= cancel_i;
      if (cancel_i) begin
        state_q <= IDLE;
        result_q <= '0;
        operation_q <= MD_MUL;
        iteration_q <= '0;
        product_q <= '0;
        multiplicand_q <= '0;
        multiplier_q <= '0;
        negate_product_q <= 1'b0;
        remainder_q <= '0;
        quotient_q <= '0;
        divisor_q <= '0;
        dividend_q <= '0;
        negate_quotient_q <= 1'b0;
        negate_remainder_q <= 1'b0;
        divide_by_zero_q <= 1'b0;
      end else begin
        state_q <= state_d;
        result_q <= result_d;
        operation_q <= operation_d;
        iteration_q <= iteration_d;
        product_q <= product_d;
        multiplicand_q <= multiplicand_d;
        multiplier_q <= multiplier_d;
        negate_product_q <= negate_product_d;
        remainder_q <= remainder_d;
        quotient_q <= quotient_d;
        divisor_q <= divisor_d;
        dividend_q <= dividend_d;
        negate_quotient_q <= negate_quotient_d;
        negate_remainder_q <= negate_remainder_d;
        divide_by_zero_q <= divide_by_zero_d;
      end
    end
  end

endmodule
