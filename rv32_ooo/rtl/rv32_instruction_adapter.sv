// IF memory boundary: one retained response over a synchronous 64-bit BRAM port.
// The native BRAM output updates on an enabled clock edge and holds while disabled.
`timescale 1ns / 1ps

module rv32_instruction_adapter
  import rv32_ooo_pkg::*;
(
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  logic                 request_valid_i,
  output logic                 request_ready_o,
  input  word_t                request_address_i,
  output logic                 response_valid_o,
  input  logic                 response_ready_i,
  output fetch_row_t           response_data_o,
  output logic                 response_error_o,
  input  logic                 bram_block_i,
  output logic                 bram_enable_o,
  output logic [BRAM_FETCH_ADDR_W-1:0] bram_address_o,
  input  fetch_row_t           bram_data_i,
  output logic                 transaction_pending_o,
  output logic                 protocol_error_o
);

  localparam int ROW_OFFSET_W = $clog2(FETCH_ROW_BYTES);
  logic response_valid_q, response_error_q;
  logic request_fire, misaligned, address_valid;
  word_t relative_address;

  assign relative_address = request_address_i - RAM_BASE;
  assign misaligned = |request_address_i[ROW_OFFSET_W-1:0];
  assign address_valid = request_address_i >= RAM_BASE
                         && relative_address < RAM_BYTES && !misaligned;

  // A consumed response can be replaced on the same edge. A shared-memory
  // collision hold blocks only NEW reads, never consumption of the old response.
  assign request_ready_o = rst_ni && !bram_block_i
                           && (!response_valid_q || response_ready_i);
  assign request_fire = request_valid_i && request_ready_o;
  assign bram_enable_o = request_fire && address_valid;
  // Keep the proposed row visible before enable, so collision hold can compare
  // addresses without feeding back through the enable that it is suppressing.
  assign bram_address_o = relative_address[BRAM_FETCH_ADDR_W+ROW_OFFSET_W-1:ROW_OFFSET_W];

  // BRAM must have one-cycle native read latency, with additional output
  // registers disabled. No new read is enabled while its response is stalled,
  // so the native output register itself retains the matching 64-bit payload.
  assign response_valid_o = rst_ni && response_valid_q;
  assign response_error_o = response_valid_o && response_error_q;
  assign response_data_o = response_valid_o && !response_error_q ? bram_data_i : '0;
  assign transaction_pending_o = response_valid_q;
  assign protocol_error_o = request_fire && misaligned;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      response_valid_q <= 1'b0;
      response_error_q <= 1'b0;
    end else begin
      if (response_valid_o && response_ready_i) response_valid_q <= 1'b0;
      if (request_fire) begin
        response_valid_q <= 1'b1;
        response_error_q <= !address_valid;
      end
    end
  end
endmodule
