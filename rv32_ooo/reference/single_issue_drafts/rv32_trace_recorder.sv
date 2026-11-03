// Verification/debug boundary: register one accepted architectural event per cycle.
// A later trap/interrupt controller can merge its events before this recorder.
`timescale 1ns / 1ps

module rv32_trace_recorder (
  input  logic                       clk_i,
  input  logic                       rst_ni,
  input  rv32_ooo_pkg::retire_trace_t event_i,
  output rv32_ooo_pkg::retire_trace_t trace_o
);
  logic [63:0] next_order_q;
  rv32_ooo_pkg::retire_trace_t stamped_event;

  always_comb begin
    stamped_event = event_i;
    stamped_event.order = next_order_q;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      trace_o <= '0;
      next_order_q <= '0;
    end else begin
      trace_o <= '0;
      if (event_i.valid) begin
        trace_o <= stamped_event;
        next_order_q <= next_order_q + 1'b1;
      end
    end
  end
endmodule
