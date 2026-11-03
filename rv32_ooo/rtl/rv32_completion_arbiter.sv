// WB section: five retained producer channels share two registered result slots.
// Capturing a source frees its producer, but does not yet write any physical register.
`timescale 1ns / 1ps

module rv32_completion_arbiter
  import rv32_ooo_pkg::*;
(
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  logic [COMPLETION_SOURCE_COUNT-1:0] source_valid_i,
  output logic [COMPLETION_SOURCE_COUNT-1:0] source_ready_o,
  input  completion_sources_t  sources_i,
  input  rob_index_t           rob_head_index_i,
  output logic [1:0]           completion_valid_o,
  input  logic [1:0]           completion_ready_i,
  output completion_pair_t     completion_o,
  input  logic                 cancel_i,
  output logic                 cancel_done_o,
  output logic                 quiescent_o,
  output logic [1:0]           occupancy_o,
  output logic                 protocol_error_o
);
  completion_t slots_q [2];
  completion_t slots_d [2];
  logic [1:0] valid_q, valid_d;

  function automatic int program_age(input rob_index_t index, input rob_index_t head);
    rob_index_t distance;
    begin
      distance = index - head;
      program_age = int'(distance);
    end
  endfunction

  // Registered slots own the output payload. A stalled lane never moves lanes,
  // even if an older result appears or the other lane is consumed/refilled.
  always_comb begin
    completion_valid_o = '0;
    completion_o = '0;
    if (rst_ni && !cancel_i) begin
      completion_valid_o = valid_q;
      if (valid_q[0]) completion_o.lane0 = slots_q[0];
      if (valid_q[1]) completion_o.lane1 = slots_q[1];
    end
    occupancy_o = {1'b0, valid_q[0]} + {1'b0, valid_q[1]};
    quiescent_o = valid_q == 0;
  end

  always_comb begin : select_sources
    completion_t packets [COMPLETION_SOURCE_COUNT];
    completion_t packet, other_packet, buffered;
    logic [COMPLETION_SOURCE_COUNT-1:0] selected;
    int ages [COMPLETION_SOURCE_COUNT];
    int best;

    packets[0] = sources_i.alu0_result;
    packets[1] = sources_i.alu1_result;
    packets[2] = sources_i.muldiv_result;
    packets[3] = sources_i.memory_result;
    packets[4] = sources_i.system_result;
    for (int i = 0; i < COMPLETION_SOURCE_COUNT; i++) begin
      packet = packets[i];
      ages[i] = program_age(packet.rob_tag.index, rob_head_index_i);
    end
    selected = '0;
    source_ready_o = '0;
    valid_d = valid_q;
    for (int lane = 0; lane < 2; lane++) slots_d[lane] = slots_q[lane];
    best = -1;
    protocol_error_o = 1'b0;
    packet = '0;
    other_packet = '0;
    buffered = '0;

    if (rst_ni && !cancel_i) begin
      for (int lane = 0; lane < 2; lane++) begin
        // Only this lane's consumer can free its slot. Pop and refill can coincide.
        if (!valid_q[lane] || completion_ready_i[lane]) begin
          valid_d[lane] = 1'b0;
          best = -1;
          for (int i = 0; i < COMPLETION_SOURCE_COUNT; i++) begin
            if (source_valid_i[i] && !selected[i]
                && (best < 0 || ages[i] < ages[best])) best = i;
          end
          if (best >= 0) begin
            slots_d[lane] = packets[best];
            valid_d[lane] = 1'b1;
            selected[best] = 1'b1;
            source_ready_o[best] = 1'b1;
          end
        end
      end

      // Each live instruction has one completion owner. Duplicate packets still
      // flow to ROB rejection; this diagnostic never substitutes for its checks.
      for (int i = 0; i < COMPLETION_SOURCE_COUNT; i++) begin
        packet = packets[i];
        for (int j = 0; j < COMPLETION_SOURCE_COUNT; j++) begin
          other_packet = packets[j];
          if (j > i && source_valid_i[i] && source_valid_i[j]
              && packet.rob_tag == other_packet.rob_tag) protocol_error_o = 1'b1;
        end
        for (int lane = 0; lane < 2; lane++) begin
          buffered = slots_q[lane];
          if (source_valid_i[i] && valid_q[lane]
              && packet.rob_tag == buffered.rob_tag) protocol_error_o = 1'b1;
        end
      end
      packet = slots_q[0];
      other_packet = slots_q[1];
      if (&valid_q && packet.rob_tag == other_packet.rob_tag) protocol_error_o = 1'b1;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      valid_q <= '0;
      slots_q[0] <= '0;
      slots_q[1] <= '0;
      cancel_done_o <= 1'b0;
    end else begin
      cancel_done_o <= cancel_i;
      if (cancel_i) begin
        valid_q <= '0;
        slots_q[0] <= '0;
        slots_q[1] <= '0;
      end else begin
        valid_q <= valid_d;
        slots_q[0] <= slots_d[0];
        slots_q[1] <= slots_d[1];
      end
    end
  end
endmodule
