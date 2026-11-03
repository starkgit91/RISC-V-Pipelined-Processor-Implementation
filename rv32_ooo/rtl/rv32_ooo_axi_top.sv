// Portable core boundary: native synchronous instruction RAM and AXI4-Lite data.
// Clocks, reset generation, address decoding, RAM and peripherals are external.
`timescale 1ns / 1ps
module rv32_ooo_axi_top
  import rv32_ooo_pkg::*;
(
  input logic clk_i, rst_ni,
  input logic predict_enable_i, halt_i,
  input logic irq_external_i, irq_software_i, irq_timer_i,
  input logic bram_block_i,
  output logic bram_enable_o,
  output logic [BRAM_FETCH_ADDR_W-1:0] bram_address_o,
  input fetch_row_t bram_data_i,
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
  output retire_trace_pair_t trace_o,
  output logic empty_o,
  output logic protocol_error_o
);
  logic instruction_valid, instruction_ready, instruction_response_valid;
  logic instruction_response_ready, instruction_error, instruction_pending, instruction_protocol;
  word_t instruction_address;
  fetch_row_t instruction_data;
  logic data_valid, data_ready, data_response_valid, data_response_ready;
  data_request_t data_request;
  memory_response_t data_response;
  logic core_error, data_error, data_busy;

  rv32_ooo_core core (
    .clk_i(clk_i), .rst_ni(rst_ni), .predict_enable_i(predict_enable_i), .halt_i(halt_i),
    .instruction_request_valid_o(instruction_valid),
    .instruction_request_ready_i(instruction_ready),
    .instruction_request_address_o(instruction_address),
    .instruction_response_valid_i(instruction_response_valid),
    .instruction_response_ready_o(instruction_response_ready),
    .instruction_response_data_i(instruction_data),
    .instruction_response_error_i(instruction_error),
    .data_request_valid_o(data_valid), .data_request_ready_i(data_ready),
    .data_request_o(data_request), .data_response_valid_i(data_response_valid),
    .data_response_ready_o(data_response_ready), .data_response_i(data_response),
    .irq_external_i(irq_external_i), .irq_software_i(irq_software_i), .irq_timer_i(irq_timer_i),
    .trace_o(trace_o), .empty_o(empty_o), .protocol_error_o(core_error)
  );
  rv32_instruction_adapter instruction_port (
    .clk_i(clk_i), .rst_ni(rst_ni), .request_valid_i(instruction_valid),
    .request_ready_o(instruction_ready), .request_address_i(instruction_address),
    .response_valid_o(instruction_response_valid), .response_ready_i(instruction_response_ready),
    .response_data_o(instruction_data), .response_error_o(instruction_error),
    .bram_block_i(bram_block_i), .bram_enable_o(bram_enable_o), .bram_address_o(bram_address_o),
    .bram_data_i(bram_data_i), .transaction_pending_o(instruction_pending),
    .protocol_error_o(instruction_protocol)
  );
  rv32_axi_lite_master data_port (
    .clk_i(clk_i), .rst_ni(rst_ni), .request_valid_i(data_valid), .request_ready_o(data_ready),
    .request_i(data_request), .response_valid_o(data_response_valid),
    .response_ready_i(data_response_ready), .response_o(data_response),
    .m_axi_awaddr_o(m_axi_awaddr_o), .m_axi_awprot_o(m_axi_awprot_o),
    .m_axi_awvalid_o(m_axi_awvalid_o), .m_axi_awready_i(m_axi_awready_i),
    .m_axi_wdata_o(m_axi_wdata_o), .m_axi_wstrb_o(m_axi_wstrb_o),
    .m_axi_wvalid_o(m_axi_wvalid_o), .m_axi_wready_i(m_axi_wready_i),
    .m_axi_bresp_i(m_axi_bresp_i), .m_axi_bvalid_i(m_axi_bvalid_i),
    .m_axi_bready_o(m_axi_bready_o), .m_axi_araddr_o(m_axi_araddr_o),
    .m_axi_arprot_o(m_axi_arprot_o), .m_axi_arvalid_o(m_axi_arvalid_o),
    .m_axi_arready_i(m_axi_arready_i), .m_axi_rdata_i(m_axi_rdata_i),
    .m_axi_rresp_i(m_axi_rresp_i), .m_axi_rvalid_i(m_axi_rvalid_i),
    .m_axi_rready_o(m_axi_rready_o), .busy_o(data_busy), .protocol_error_o(data_error)
  );
  assign protocol_error_o = core_error || instruction_protocol || data_error;
endmodule
