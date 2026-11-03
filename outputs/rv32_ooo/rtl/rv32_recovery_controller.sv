// Retirement-boundary recovery: cancel producers before flushing reusable ownership.
`timescale 1ns / 1ps
module rv32_recovery_controller
  import rv32_ooo_pkg::*;
(
  input logic clk_i, rst_ni,
  input logic request_i,
  input word_t target_i,
  input logic units_quiescent_i,
  input logic cancel_ack_i,
  output logic cancel_o,
  output logic apply_o,
  output word_t redirect_pc_o,
  output logic busy_o,
  output logic done_o
);
  typedef enum logic [1:0] {RUN, CANCEL, APPLY} state_e;
  state_e state_q;
  word_t target_q;
  assign cancel_o = rst_ni && state_q != RUN;
  assign apply_o = rst_ni && state_q == APPLY;
  assign redirect_pc_o = target_q;
  assign busy_o = state_q != RUN;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= RUN;
      target_q <= RESET_PC;
      done_o <= 1'b0;
    end else begin
      done_o <= 1'b0;
      case (state_q)
        RUN: if (request_i) begin target_q <= target_i; state_q <= CANCEL; end
        CANCEL: if (units_quiescent_i && cancel_ack_i) state_q <= APPLY;
        APPLY: begin state_q <= RUN; done_o <= 1'b1; end
        default: state_q <= CANCEL;
      endcase
    end
  end
endmodule
