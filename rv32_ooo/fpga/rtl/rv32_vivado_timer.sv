// Machine timer IP Integrator boundary; the tested peripheral RTL is unchanged.
`timescale 1ns / 1ps
module rv32_vivado_timer (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk_i CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET rst_ni, FREQ_HZ 50000000" *)
  input logic clk_i,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst_ni RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input logic rst_ni,
  (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI, PROTOCOL AXI4LITE, ADDR_WIDTH 32, DATA_WIDTH 32, FREQ_HZ 50000000, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1" *)
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
  input logic [31:0] s_axi_awaddr,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *)
  input logic [2:0] s_axi_awprot,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *)
  input logic s_axi_awvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *)
  output logic s_axi_awready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *)
  input logic [31:0] s_axi_wdata,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *)
  input logic [3:0] s_axi_wstrb,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *)
  input logic s_axi_wvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *)
  output logic s_axi_wready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *)
  output logic [1:0] s_axi_bresp,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *)
  output logic s_axi_bvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *)
  input logic s_axi_bready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *)
  input logic [31:0] s_axi_araddr,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *)
  input logic [2:0] s_axi_arprot,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *)
  input logic s_axi_arvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *)
  output logic s_axi_arready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *)
  output logic [31:0] s_axi_rdata,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *)
  output logic [1:0] s_axi_rresp,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *)
  output logic s_axi_rvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *)
  input logic s_axi_rready,
  output logic timer_irq_o, software_irq_o
);
  rv32_machine_timer timer (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .s_axi_awaddr_i(s_axi_awaddr),
    .s_axi_awprot_i(s_axi_awprot),
    .s_axi_awvalid_i(s_axi_awvalid),
    .s_axi_awready_o(s_axi_awready),
    .s_axi_wdata_i(s_axi_wdata),
    .s_axi_wstrb_i(s_axi_wstrb),
    .s_axi_wvalid_i(s_axi_wvalid),
    .s_axi_wready_o(s_axi_wready),
    .s_axi_bresp_o(s_axi_bresp),
    .s_axi_bvalid_o(s_axi_bvalid),
    .s_axi_bready_i(s_axi_bready),
    .s_axi_araddr_i(s_axi_araddr),
    .s_axi_arprot_i(s_axi_arprot),
    .s_axi_arvalid_i(s_axi_arvalid),
    .s_axi_arready_o(s_axi_arready),
    .s_axi_rdata_o(s_axi_rdata),
    .s_axi_rresp_o(s_axi_rresp),
    .s_axi_rvalid_o(s_axi_rvalid),
    .s_axi_rready_i(s_axi_rready),
    .timer_irq_o(timer_irq_o), .software_irq_o(software_irq_o)
  );
endmodule
