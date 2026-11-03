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
