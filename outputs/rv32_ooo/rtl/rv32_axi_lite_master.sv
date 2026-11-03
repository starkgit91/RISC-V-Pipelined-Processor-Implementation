// One-outstanding data-port bridge. AW and W remain independent until both transfer.
`timescale 1ns / 1ps
module rv32_axi_lite_master
  import rv32_ooo_pkg::*;
(
  input logic clk_i, rst_ni,
  input logic request_valid_i,
  output logic request_ready_o,
  input data_request_t request_i,
  output logic response_valid_o,
  input logic response_ready_i,
  output memory_response_t response_o,
  output word_t m_axi_awaddr_o,
  output logic [2:0] m_axi_awprot_o,
  output logic m_axi_awvalid_o,
  input logic m_axi_awready_i,
  output word_t m_axi_wdata_o,
  output logic [3:0] m_axi_wstrb_o,
  output logic m_axi_wvalid_o,
  input logic m_axi_wready_i,
  input logic [1:0] m_axi_bresp_i,
  input logic m_axi_bvalid_i,
  output logic m_axi_bready_o,
  output word_t m_axi_araddr_o,
  output logic [2:0] m_axi_arprot_o,
  output logic m_axi_arvalid_o,
  input logic m_axi_arready_i,
  input word_t m_axi_rdata_i,
  input logic [1:0] m_axi_rresp_i,
  input logic m_axi_rvalid_i,
  output logic m_axi_rready_o,
  output logic busy_o,
  output logic protocol_error_o
);
  typedef enum logic [1:0] {IDLE, READ, WRITE, RESPONSE} state_e;
  state_e state_q;
  data_request_t request_q;
  memory_response_t response_q;
  logic aw_pending_q, w_pending_q, ar_pending_q;

  assign request_ready_o = rst_ni && state_q == IDLE;
  assign response_valid_o = rst_ni && state_q == RESPONSE;
  assign response_o = response_valid_o ? response_q : memory_response_t'('0);
  assign busy_o = state_q != IDLE;
  assign m_axi_awaddr_o = request_q.address;
  assign m_axi_wdata_o = request_q.wdata;
  assign m_axi_wstrb_o = request_q.wstrb;
  assign m_axi_araddr_o = request_q.address;
  assign m_axi_awprot_o = 3'b001; // Privileged, secure data access in machine mode.
  assign m_axi_arprot_o = 3'b001;
  assign m_axi_awvalid_o = rst_ni && state_q == WRITE && aw_pending_q;
  assign m_axi_wvalid_o = rst_ni && state_q == WRITE && w_pending_q;
  assign m_axi_bready_o = rst_ni && state_q == WRITE && !aw_pending_q && !w_pending_q;
  assign m_axi_arvalid_o = rst_ni && state_q == READ && ar_pending_q;
  assign m_axi_rready_o = rst_ni && state_q == READ && !ar_pending_q;
  assign protocol_error_o = rst_ni
    && ((request_valid_i && request_ready_o && |request_i.address[1:0])
        || (m_axi_bvalid_i && state_q != WRITE)
        || (m_axi_rvalid_i && state_q != READ));

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= IDLE;
      request_q <= '0;
      response_q <= '0;
      aw_pending_q <= 0;
      w_pending_q <= 0;
      ar_pending_q <= 0;
    end else begin
      case (state_q)
        IDLE: if (request_valid_i && request_ready_o) begin
          request_q <= request_i;
          response_q <= '0;
          if (|request_i.address[1:0]) begin
            response_q.error <= 1;
            state_q <= RESPONSE;
          end else if (request_i.write) begin
            aw_pending_q <= 1;
            w_pending_q <= 1;
            state_q <= WRITE;
          end else begin
            ar_pending_q <= 1;
            state_q <= READ;
          end
        end
        WRITE: begin
          if (m_axi_awvalid_o && m_axi_awready_i) aw_pending_q <= 0;
          if (m_axi_wvalid_o && m_axi_wready_i) w_pending_q <= 0;
          if (m_axi_bvalid_i && m_axi_bready_o) begin
            response_q.rdata <= '0;
            response_q.error <= m_axi_bresp_i != 2'b00;
            state_q <= RESPONSE;
          end
        end
        READ: begin
          if (m_axi_arvalid_o && m_axi_arready_i) ar_pending_q <= 0;
          if (m_axi_rvalid_i && m_axi_rready_o) begin
            response_q.rdata <= m_axi_rdata_i;
            response_q.error <= m_axi_rresp_i != 2'b00;
            state_q <= RESPONSE;
          end
        end
        RESPONSE: if (response_valid_o && response_ready_i) state_q <= IDLE;
        default: state_q <= IDLE;
      endcase
    end
  end
endmodule
