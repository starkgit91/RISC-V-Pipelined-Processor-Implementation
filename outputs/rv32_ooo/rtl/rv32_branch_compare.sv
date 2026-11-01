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
