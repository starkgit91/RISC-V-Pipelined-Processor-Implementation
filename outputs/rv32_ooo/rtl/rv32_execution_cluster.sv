// EX resources for two issue ports. Completion arbitration belongs to block 8.
// Three independently held results can coexist despite a two-result writeback width.
`timescale 1ns / 1ps

module rv32_execution_cluster
  import rv32_ooo_pkg::*;
(
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  logic [1:0]           issue_fire_i,
  input  execute_packet_pair_t issue_packet_i,
  output logic                 alu0_ready_o,
  output logic                 alu1_ready_o,
  output logic                 muldiv_ready_o,
  output logic                 memory_ready_o,
  output logic [1:0]           alu_completion_valid_o,
  input  logic [1:0]           alu_completion_ready_i,
  output completion_pair_t     alu_completion_o,
  output logic                 muldiv_completion_valid_o,
  input  logic                 muldiv_completion_ready_i,
  output completion_t          muldiv_completion_o,
  output logic                 memory_valid_o,
  input  logic                 memory_ready_i,
  output memory_entry_t        memory_o,
  input  logic                 cancel_i,
  output logic                 cancel_done_o,
  output logic [2:0]           busy_o,
  output logic                 quiescent_o,
  output logic                 protocol_error_o
);
  logic [2:0] unit_error, cancel_done, quiescent;
  logic port1_is_muldiv;
  logic unused_memory_valid;
  memory_entry_t unused_memory;

  assign port1_is_muldiv = issue_packet_i.lane1.uop.decoded.op_class == UOP_MULDIV;

  rv32_integer_execute #(.CONTROL_ENABLE(1'b1)) integer0 (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .request_valid_i(issue_fire_i[0]), .request_ready_o(alu0_ready_o),
    .request_i(issue_packet_i.lane0),
    .completion_valid_o(alu_completion_valid_o[0]),
    .completion_ready_i(alu_completion_ready_i[0]), .completion_o(alu_completion_o.lane0),
    .memory_valid_o(memory_valid_o), .memory_ready_i(memory_ready_i), .memory_o(memory_o),
    .cancel_i(cancel_i), .cancel_done_o(cancel_done[0]), .busy_o(busy_o[0]),
    .quiescent_o(quiescent[0]), .protocol_error_o(unit_error[0])
  );
  assign memory_ready_o = alu0_ready_o;

  rv32_integer_execute #(.CONTROL_ENABLE(1'b0)) integer1 (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .request_valid_i(issue_fire_i[1] && !port1_is_muldiv), .request_ready_o(alu1_ready_o),
    .request_i(issue_packet_i.lane1),
    .completion_valid_o(alu_completion_valid_o[1]),
    .completion_ready_i(alu_completion_ready_i[1]), .completion_o(alu_completion_o.lane1),
    .memory_valid_o(unused_memory_valid), .memory_ready_i(1'b0), .memory_o(unused_memory),
    .cancel_i(cancel_i), .cancel_done_o(cancel_done[1]), .busy_o(busy_o[1]),
    .quiescent_o(quiescent[1]), .protocol_error_o(unit_error[1])
  );
  rv32_muldiv_execute muldiv (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .request_valid_i(issue_fire_i[1] && port1_is_muldiv), .request_ready_o(muldiv_ready_o),
    .request_i(issue_packet_i.lane1),
    .completion_valid_o(muldiv_completion_valid_o),
    .completion_ready_i(muldiv_completion_ready_i), .completion_o(muldiv_completion_o),
    .cancel_i(cancel_i), .cancel_done_o(cancel_done[2]), .busy_o(busy_o[2]),
    .quiescent_o(quiescent[2]), .protocol_error_o(unit_error[2])
  );

  assign cancel_done_o = &cancel_done;
  assign quiescent_o = &quiescent;
  assign protocol_error_o = |unit_error;
endmodule
