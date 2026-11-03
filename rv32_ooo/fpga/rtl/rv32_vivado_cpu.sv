// Plain-port IP Integrator boundary; architectural types remain inside the wrapper.
`timescale 1ns / 1ps
module rv32_vivado_cpu (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk_i CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF M_AXI, ASSOCIATED_RESET rst_ni, FREQ_HZ 50000000" *)
  input logic clk_i,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst_ni RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input logic rst_ni,
  input logic irq_external_i, irq_software_i, irq_timer_i,
  input logic bram_block_i,
  output logic bram_enable_o,
  output logic [12:0] bram_address_o,
  input logic [63:0] bram_data_i,
  (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME M_AXI, PROTOCOL AXI4LITE, ADDR_WIDTH 32, DATA_WIDTH 32, FREQ_HZ 50000000, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1" *)
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWADDR" *)
  output logic [31:0] m_axi_awaddr,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWPROT" *)
  output logic [2:0] m_axi_awprot,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWVALID" *)
  output logic m_axi_awvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWREADY" *)
  input logic m_axi_awready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WDATA" *)
  output logic [31:0] m_axi_wdata,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WSTRB" *)
  output logic [3:0] m_axi_wstrb,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WVALID" *)
  output logic m_axi_wvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WREADY" *)
  input logic m_axi_wready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BRESP" *)
  input logic [1:0] m_axi_bresp,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BVALID" *)
  input logic m_axi_bvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BREADY" *)
  output logic m_axi_bready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI ARADDR" *)
  output logic [31:0] m_axi_araddr,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI ARPROT" *)
  output logic [2:0] m_axi_arprot,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI ARVALID" *)
  output logic m_axi_arvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI ARREADY" *)
  input logic m_axi_arready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI RDATA" *)
  input logic [31:0] m_axi_rdata,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI RRESP" *)
  input logic [1:0] m_axi_rresp,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI RVALID" *)
  input logic m_axi_rvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI RREADY" *)
  output logic m_axi_rready,
  output logic activity_o,
  output logic error_o
);
  import rv32_ooo_pkg::*;
  retire_trace_pair_t trace;
  logic empty, protocol_error;
  rv32_ooo_axi_top processor (
    .clk_i(clk_i), .rst_ni(rst_ni), .predict_enable_i(1'b1), .halt_i(1'b0),
    .irq_external_i(irq_external_i), .irq_software_i(irq_software_i), .irq_timer_i(irq_timer_i),
    .bram_block_i(bram_block_i), .bram_enable_o(bram_enable_o),
    .bram_address_o(bram_address_o), .bram_data_i(bram_data_i),
    .m_axi_awaddr_o(m_axi_awaddr),
    .m_axi_awprot_o(m_axi_awprot),
    .m_axi_awvalid_o(m_axi_awvalid),
    .m_axi_awready_i(m_axi_awready),
    .m_axi_wdata_o(m_axi_wdata),
    .m_axi_wstrb_o(m_axi_wstrb),
    .m_axi_wvalid_o(m_axi_wvalid),
    .m_axi_wready_i(m_axi_wready),
    .m_axi_bresp_i(m_axi_bresp),
    .m_axi_bvalid_i(m_axi_bvalid),
    .m_axi_bready_o(m_axi_bready),
    .m_axi_araddr_o(m_axi_araddr),
    .m_axi_arprot_o(m_axi_arprot),
    .m_axi_arvalid_o(m_axi_arvalid),
    .m_axi_arready_i(m_axi_arready),
    .m_axi_rdata_i(m_axi_rdata),
    .m_axi_rresp_i(m_axi_rresp),
    .m_axi_rvalid_i(m_axi_rvalid),
    .m_axi_rready_o(m_axi_rready),
    .trace_o(trace), .empty_o(empty), .protocol_error_o(protocol_error)
  );
  // These pins retain observable core activity and a sticky ownership/protocol diagnostic.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin activity_o <= 1'b0; error_o <= 1'b0; end
    else begin
      if (trace.lane0.valid || trace.lane1.valid) activity_o <= ~activity_o;
      if (protocol_error) error_o <= 1'b1;
    end
  end
endmodule
