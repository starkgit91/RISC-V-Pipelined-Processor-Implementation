// EX register-read / WB completion boundary: two read ports and one write port.
// Register readiness and committed mappings belong to other blocks, not this storage.
`timescale 1ns / 1ps

module rv32_physical_regfile (
  input  logic                     clk_i,
  input  logic                     rst_ni,
  input  rv32_ooo_pkg::phys_reg_t   read_rs1_i,
  input  rv32_ooo_pkg::phys_reg_t   read_rs2_i,
  output rv32_ooo_pkg::word_t       read_data1_o,
  output rv32_ooo_pkg::word_t       read_data2_o,
  input  logic                     write_enable_i,
  input  rv32_ooo_pkg::phys_reg_t   write_addr_i,
  input  rv32_ooo_pkg::word_t       write_data_i
);
  import rv32_ooo_pkg::*;

  // p0 is a constant, so only p1..p63 need storage. Reset makes startup deterministic.
  // The asynchronous reset and reads intentionally favor registers, not block RAM.
  word_t values_q [1:PHYS_REG_COUNT-1];
  integer reg_index;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (reg_index = 1; reg_index < PHYS_REG_COUNT; reg_index = reg_index + 1) begin
        values_q[reg_index] <= '0;
      end
    end else if (write_enable_i && (write_addr_i != 0)) begin
      values_q[write_addr_i] <= write_data_i;
    end
  end

  always_comb begin
    read_data1_o = '0;
    read_data2_o = '0;
    if (rst_ni) begin
      if (read_rs1_i != 0) begin
        // The write port carries an accepted, live, non-faulting completion.
        // Bypass it before waiting for the stored value's clock-edge update.
        if (write_enable_i && (write_addr_i == read_rs1_i)) begin
          read_data1_o = write_data_i;
        end else begin
          read_data1_o = values_q[read_rs1_i];
        end
      end
      if (read_rs2_i != 0) begin
        if (write_enable_i && (write_addr_i == read_rs2_i)) begin
          read_data2_o = write_data_i;
        end else begin
          read_data2_o = values_q[read_rs2_i];
        end
      end
    end
  end
endmodule
