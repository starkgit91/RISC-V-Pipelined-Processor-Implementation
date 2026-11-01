// Register an ordered prefix of up to two architectural events and stamp event order.
// A later trap/interrupt controller may merge its events before this recorder.
`timescale 1ns / 1ps

module rv32_trace_recorder (
  input  logic                            clk_i,
  input  logic                            rst_ni,
  input  rv32_ooo_pkg::retire_trace_pair_t  event_i,
  output rv32_ooo_pkg::retire_trace_pair_t  trace_o,
  output logic                            protocol_error_o
);
  import rv32_ooo_pkg::*;
  logic [63:0] next_order_q, next_order_d;
  retire_trace_t event0, event1, stamped0, stamped1;
  retire_trace_pair_t next_trace;

  assign event0 = event_i.lane0;
  assign event1 = event_i.lane1;
  assign protocol_error_o = rst_ni && !event0.valid && event1.valid;

  always_comb begin
    stamped0 = event0;
    stamped1 = event1;
    stamped0.order = next_order_q;
    stamped1.order = next_order_q + 64'd1;
    next_trace = '0;
    next_order_d = next_order_q;
    if (!protocol_error_o && event0.valid) begin
      next_trace.lane0 = stamped0;
      next_order_d = next_order_q + 64'd1;
      if (event1.valid) begin
        next_trace.lane1 = stamped1;
        next_order_d = next_order_q + 64'd2;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      trace_o <= '0;
      next_order_q <= '0;
    end else begin
      trace_o <= next_trace;
      next_order_q <= next_order_d;
    end
  end
endmodule
