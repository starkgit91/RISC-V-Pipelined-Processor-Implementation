// AXI4-Lite machine timer/MSIP peripheral. No speculative core access may reach this slave.
`timescale 1ns / 1ps
module rv32_machine_timer
  import rv32_ooo_pkg::*;
(
  input logic clk_i, rst_ni,
  input word_t s_axi_awaddr_i,
  input logic [2:0] s_axi_awprot_i,
  input logic s_axi_awvalid_i,
  output logic s_axi_awready_o,
  input word_t s_axi_wdata_i,
  input logic [3:0] s_axi_wstrb_i,
  input logic s_axi_wvalid_i,
  output logic s_axi_wready_o,
  output logic [1:0] s_axi_bresp_o,
  output logic s_axi_bvalid_o,
  input logic s_axi_bready_i,
  input word_t s_axi_araddr_i,
  input logic [2:0] s_axi_arprot_i,
  input logic s_axi_arvalid_i,
  output logic s_axi_arready_o,
  output word_t s_axi_rdata_o,
  output logic [1:0] s_axi_rresp_o,
  output logic s_axi_rvalid_o,
  input logic s_axi_rready_i,
  output logic timer_irq_o,
  output logic software_irq_o
);
  logic [63:0] mtime_q, mtime_d, mtimecmp_q, mtimecmp_d;
  logic msip_q, msip_d;
  logic aw_held_q, w_held_q, b_valid_q, r_valid_q;
  word_t aw_address_q, w_data_q, r_data_q;
  logic [3:0] w_strobes_q;
  logic [1:0] b_response_q, r_response_q;
  logic commit_write, write_address_valid, read_address_valid;
  word_t read_value;

  assign s_axi_awready_o = rst_ni && !aw_held_q && !b_valid_q;
  assign s_axi_wready_o = rst_ni && !w_held_q && !b_valid_q;
  assign s_axi_bvalid_o = rst_ni && b_valid_q;
  assign s_axi_bresp_o = b_response_q;
  assign s_axi_arready_o = rst_ni && !r_valid_q;
  assign s_axi_rvalid_o = rst_ni && r_valid_q;
  assign s_axi_rdata_o = r_data_q;
  assign s_axi_rresp_o = r_response_q;
  assign timer_irq_o = rst_ni && mtime_q >= mtimecmp_q;
  assign software_irq_o = rst_ni && msip_q;
  assign commit_write = aw_held_q && w_held_q && !b_valid_q;

  always_comb begin
    write_address_valid = aw_address_q[1:0] == 0;
    case (aw_address_q[15:0])
      16'h0000, 16'h4000, 16'h4004, 16'hbff8, 16'hbffc: begin end
      default: write_address_valid = 0;
    endcase
    read_address_valid = s_axi_araddr_i[1:0] == 0;
    read_value = '0;
    case (s_axi_araddr_i[15:0])
      16'h0000: read_value = {31'b0, msip_q};
      16'h4000: read_value = mtimecmp_q[31:0];
      16'h4004: read_value = mtimecmp_q[63:32];
      16'hbff8: read_value = mtime_q[31:0];
      16'hbffc: read_value = mtime_q[63:32];
      default: read_address_valid = 0;
    endcase
    if (!read_address_valid) read_value = '0;

    mtime_d = mtime_q + 64'd1;
    mtimecmp_d = mtimecmp_q;
    msip_d = msip_q;
    if (commit_write && write_address_valid) begin
      case (aw_address_q[15:0])
        16'h0000: if (w_strobes_q[0]) msip_d = w_data_q[0];
        16'h4000: for (int lane = 0; lane < 4; lane++)
          if (w_strobes_q[lane]) mtimecmp_d[lane*8 +: 8] = w_data_q[lane*8 +: 8];
        16'h4004: for (int lane = 0; lane < 4; lane++)
          if (w_strobes_q[lane]) mtimecmp_d[32+lane*8 +: 8] = w_data_q[lane*8 +: 8];
        16'hbff8: if (|w_strobes_q) begin
          mtime_d = mtime_q; // A strobed software write takes priority over this tick.
          for (int lane = 0; lane < 4; lane++)
            if (w_strobes_q[lane]) mtime_d[lane*8 +: 8] = w_data_q[lane*8 +: 8];
        end
        16'hbffc: if (|w_strobes_q) begin
          mtime_d = mtime_q;
          for (int lane = 0; lane < 4; lane++)
            if (w_strobes_q[lane]) mtime_d[32+lane*8 +: 8] = w_data_q[lane*8 +: 8];
        end
        default: begin end
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mtime_q <= 0;
      mtimecmp_q <= 64'hffff_ffff_ffff_ffff;
      msip_q <= 0;
      aw_held_q <= 0;
      w_held_q <= 0;
      b_valid_q <= 0;
      r_valid_q <= 0;
      aw_address_q <= 0;
      w_data_q <= 0;
      w_strobes_q <= 0;
      b_response_q <= 0;
      r_response_q <= 0;
      r_data_q <= 0;
    end else begin
      mtime_q <= mtime_d;
      mtimecmp_q <= mtimecmp_d;
      msip_q <= msip_d;
      if (s_axi_bvalid_o && s_axi_bready_i) b_valid_q <= 0;
      if (s_axi_awvalid_i && s_axi_awready_o) begin
        aw_address_q <= s_axi_awaddr_i;
        aw_held_q <= 1;
      end
      if (s_axi_wvalid_i && s_axi_wready_o) begin
        w_data_q <= s_axi_wdata_i;
        w_strobes_q <= s_axi_wstrb_i;
        w_held_q <= 1;
      end
      if (commit_write) begin
        aw_held_q <= 0;
        w_held_q <= 0;
        b_valid_q <= 1;
        b_response_q <= write_address_valid ? 2'b00 : 2'b10;
      end
      if (s_axi_rvalid_o && s_axi_rready_i) r_valid_q <= 0;
      if (s_axi_arvalid_i && s_axi_arready_o) begin
        r_valid_q <= 1;
        r_data_q <= read_value; // Snapshot before this edge's timer tick/write.
        r_response_q <= read_address_valid ? 2'b00 : 2'b10;
      end
    end
  end
endmodule
