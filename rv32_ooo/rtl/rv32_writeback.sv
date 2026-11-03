// Speculative writeback boundary: source buffering, ROB checks, and accepted-value broadcast.
// Architectural commitment remains a separate, program-ordered ROB operation.
`timescale 1ns / 1ps

module rv32_writeback
  import rv32_ooo_pkg::*;
(
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  logic [COMPLETION_SOURCE_COUNT-1:0] source_valid_i,
  output logic [COMPLETION_SOURCE_COUNT-1:0] source_ready_o,
  input  completion_sources_t  sources_i,
  input  rob_index_t           rob_head_index_i,
  output logic [1:0]           rob_completion_valid_o,
  input  logic [1:0]           rob_completion_ready_i,
  output completion_pair_t     rob_completion_o,
  input  logic [1:0]           rob_completion_accepted_i,
  output logic [1:0]           broadcast_valid_o,
  output completion_pair_t     broadcast_o,
  output logic [1:0]           write_enable_o,
  output phys_reg_t            write0_addr_o,
  output phys_reg_t            write1_addr_o,
  output word_t                write0_data_o,
  output word_t                write1_data_o,
  input  logic                 cancel_i,
  output logic                 cancel_done_o,
  output logic                 quiescent_o,
  output logic [1:0]           occupancy_o,
  output logic                 protocol_error_o
);
  logic arbiter_error;

  rv32_completion_arbiter arbiter (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .source_valid_i(source_valid_i), .source_ready_o(source_ready_o), .sources_i(sources_i),
    .rob_head_index_i(rob_head_index_i), .completion_valid_o(rob_completion_valid_o),
    .completion_ready_i(rob_completion_ready_i), .completion_o(rob_completion_o),
    .cancel_i(cancel_i), .cancel_done_o(cancel_done_o), .quiescent_o(quiescent_o),
    .occupancy_o(occupancy_o), .protocol_error_o(arbiter_error)
  );

  always_comb begin : accepted_writeback
    logic [1:0] consumed, accepted, register_result;
    logic owner_conflict, destination_conflict;
    completion_t first, second;

    first = rob_completion_o.lane0;
    second = rob_completion_o.lane1;
    consumed = rob_completion_valid_o & rob_completion_ready_i;
    accepted = rob_completion_accepted_i & consumed;
    register_result[0] = accepted[0] && first.writes_rd && first.pdst != 0
                         && !first.exception.valid;
    register_result[1] = accepted[1] && second.writes_rd && second.pdst != 0
                         && !second.exception.valid;
    owner_conflict = (&accepted) && first.rob_tag == second.rob_tag;
    destination_conflict = (&register_result) && first.pdst == second.pdst;
    // A conforming ROB already rejects the second conflicting owner. This
    // defensive gate also protects the PRF if accepted feedback is miswired.
    if (owner_conflict || destination_conflict) register_result[1] = 1'b0;

    broadcast_valid_o = '0;
    broadcast_o = '0;
    protocol_error_o = 1'b0;
    if (rst_ni && !cancel_i) begin
      broadcast_valid_o = register_result;
      if (register_result[0]) broadcast_o.lane0 = first;
      if (register_result[1]) broadcast_o.lane1 = second;
      protocol_error_o = arbiter_error || owner_conflict || destination_conflict
                         || |(rob_completion_accepted_i & ~consumed);
    end
    write_enable_o = broadcast_valid_o;
    write0_addr_o = broadcast_o.lane0.pdst;
    write1_addr_o = broadcast_o.lane1.pdst;
    write0_data_o = broadcast_o.lane0.value;
    write1_data_o = broadcast_o.lane1.value;
  end
endmodule
