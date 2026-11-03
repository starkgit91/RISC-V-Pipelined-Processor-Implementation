// Two-issue register read / completion boundary: four reads and two accepted writes.
// Readiness, live ROB ownership, and committed mappings are checked in other blocks.
`timescale 1ns / 1ps

module rv32_physical_regfile (
  input  logic                     clk_i,
  input  logic                     rst_ni,
  input  rv32_ooo_pkg::phys_reg_t   read0_rs1_i,
  input  rv32_ooo_pkg::phys_reg_t   read0_rs2_i,
  input  rv32_ooo_pkg::phys_reg_t   read1_rs1_i,
  input  rv32_ooo_pkg::phys_reg_t   read1_rs2_i,
  output rv32_ooo_pkg::word_t       read0_data1_o,
  output rv32_ooo_pkg::word_t       read0_data2_o,
  output rv32_ooo_pkg::word_t       read1_data1_o,
  output rv32_ooo_pkg::word_t       read1_data2_o,
  input  logic [1:0]               write_enable_i,
  input  rv32_ooo_pkg::phys_reg_t   write0_addr_i,
  input  rv32_ooo_pkg::phys_reg_t   write1_addr_i,
  input  rv32_ooo_pkg::word_t       write0_data_i,
  input  rv32_ooo_pkg::word_t       write1_data_i,
  output logic                     write_conflict_o
);
  import rv32_ooo_pkg::*;

  // p0 is constant. Each stored word has two decoded write enables and four readers.
  word_t values_q [1:PHYS_REG_COUNT-1];
  integer reg_index;

  assign write_conflict_o = rst_ni && (&write_enable_i) && (write0_addr_i != 0) &&
                            (write0_addr_i == write1_addr_i);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (reg_index = 1; reg_index < PHYS_REG_COUNT; reg_index = reg_index + 1)
        values_q[reg_index] <= '0;
    end else begin
      for (reg_index = 1; reg_index < PHYS_REG_COUNT; reg_index = reg_index + 1) begin
        if (write_enable_i[0] && write0_addr_i == reg_index)
          values_q[reg_index] <= write0_data_i;
        else if (write_enable_i[1] && write1_addr_i == reg_index)
          values_q[reg_index] <= write1_data_i;
      end
    end
  end

  // Legal live completions have different destinations. Diagnose a duplicate and let lane 0 win.
  function automatic word_t read_value(input phys_reg_t address);
    begin
      if (!rst_ni || address == 0) read_value = '0;
      else if (write_enable_i[0] && write0_addr_i == address) read_value = write0_data_i;
      else if (write_enable_i[1] && write1_addr_i == address) read_value = write1_data_i;
      else read_value = values_q[address];
    end
  endfunction

  always_comb begin
    read0_data1_o = read_value(read0_rs1_i);
    read0_data2_o = read_value(read0_rs2_i);
    read1_data1_o = read_value(read1_rs1_i);
    read1_data2_o = read_value(read1_rs2_i);
  end
endmodule
