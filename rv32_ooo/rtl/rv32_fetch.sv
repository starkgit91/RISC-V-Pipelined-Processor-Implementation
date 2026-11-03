// IF section: two instructions per 64-bit row, with reserved response capacity.
// Accepted reads are never cancelled on the memory interface; stale data is drained.
`timescale 1ns / 1ps

module rv32_fetch
  import rv32_ooo_pkg::*;
(
  input  logic             clk_i,
  input  logic             rst_ni,
  input  logic             enable_i,
  input  logic             redirect_i,
  input  word_t            redirect_pc_i,
  output logic             redirect_done_o,
  input  logic             predict_enable_i,
  output word_t            prediction_pc0_o,
  output word_t            prediction_pc1_o,
  input  prediction_pair_t prediction_i,

  output logic             request_valid_o,
  input  logic             request_ready_i,
  output word_t            request_address_o,
  input  logic             response_valid_i,
  output logic             response_ready_o,
  input  fetch_row_t       response_data_i,
  input  logic             response_error_i,

  output logic [1:0]       fetch_valid_o,
  input  logic [1:0]       fetch_ready_i,
  output fetch_pair_t      fetch_o,
  output logic [1:0]       fetch_fire_o,
  output fetch_count_t     occupancy_o,
  output logic             transaction_pending_o,
  output logic             stale_pending_o,
  output logic             fault_pending_o,
  output logic             protocol_error_o
);

  typedef struct packed {
    fetch_pair_t packets;
    word_t address;
    logic [1:0] count;
    logic upper_word;
  } descriptor_t;

  fetch_packet_t fifo_q [FETCH_DEPTH];
  fetch_packet_t fifo_d [FETCH_DEPTH];
  fetch_count_t count_q, count_d;
  word_t next_pc_q, next_pc_d;
  descriptor_t held_q, held_d, inflight_q, inflight_d, candidate;
  logic held_valid_q, held_valid_d, inflight_valid_q, inflight_valid_d;
  logic held_stale_q, held_stale_d, inflight_stale_q, inflight_stale_d;
  logic fault_wait_q, fault_wait_d;
  word_t candidate_next_pc;

  assign prediction_pc0_o = next_pc_q;
  assign prediction_pc1_o = next_pc_q + 32'd4;
  assign occupancy_o = count_q;
  assign transaction_pending_o = held_valid_q || inflight_valid_q;
  assign stale_pending_o = (held_valid_q && held_stale_q)
                            || (inflight_valid_q && inflight_stale_q);
  assign fault_pending_o = fault_wait_q;

  // Snapshot prediction when first offering a row, including a stalled request.
  // A prediction in word 0 can exclude word 1 before that word enters the FIFO.
  always_comb begin : make_candidate
    prediction_t first, second;
    first = '0;
    second = '0;
    first.next_pc = prediction_pc0_o + 32'd4;
    second.next_pc = prediction_pc1_o + 32'd4;
    first.pht_index = prediction_pc0_o[8:2];
    second.pht_index = prediction_pc1_o[8:2];
    if (predict_enable_i) begin
      first = prediction_i.lane0;
      second = prediction_i.lane1;
    end
    candidate = '0;
    candidate.address = {next_pc_q[31:3], 3'b000};
    candidate.upper_word = next_pc_q[2];
    candidate.packets.lane0.pc = next_pc_q;
    candidate.packets.lane0.prediction = first;
    candidate.packets.lane1.pc = next_pc_q + 32'd4;
    candidate.packets.lane1.prediction = second;
    candidate.count = 2'd1;
    candidate_next_pc = first.next_pc;
    if (!next_pc_q[2] && first.next_pc == next_pc_q + 32'd4) begin
      candidate.count = 2'd2;
      candidate_next_pc = second.next_pc;
    end
  end

  always_comb begin : movement
    int pop_count, used, reserved, tail;
    logic response_fire, request_fire, fresh_offer, local_fault, port_available;
    descriptor_t offered;
    fetch_packet_t first_packet, second_packet, fault_packet;

    count_d = count_q;
    next_pc_d = next_pc_q;
    held_d = held_q;
    inflight_d = inflight_q;
    held_valid_d = held_valid_q;
    inflight_valid_d = inflight_valid_q;
    held_stale_d = held_stale_q;
    inflight_stale_d = inflight_stale_q;
    fault_wait_d = fault_wait_q;
    fetch_valid_o = '0;
    fetch_fire_o = '0;
    fetch_o = '0;
    request_valid_o = 1'b0;
    request_address_o = '0;
    response_ready_o = rst_ni && inflight_valid_q;
    response_fire = response_ready_o && response_valid_i;
    request_fire = 1'b0;
    fresh_offer = 1'b0;
    local_fault = 1'b0;
    offered = '0;
    first_packet = '0;
    second_packet = '0;
    fault_packet = '0;
    protocol_error_o = rst_ni && (fetch_ready_i == 2'b10
                                  || (response_valid_i && !inflight_valid_q));

    if (rst_ni && !redirect_i) begin
      if (count_q != 0) begin
        fetch_valid_o[0] = 1'b1;
        fetch_o.lane0 = fifo_q[0];
      end
      if (count_q > 1) begin
        fetch_valid_o[1] = 1'b1;
        fetch_o.lane1 = fifo_q[1];
      end
      if (fetch_ready_i != 2'b10) begin
        fetch_fire_o[0] = fetch_valid_o[0] && fetch_ready_i[0];
        fetch_fire_o[1] = fetch_fire_o[0] && fetch_valid_o[1] && fetch_ready_i[1];
      end
    end
    pop_count = int'(fetch_fire_o[0]) + int'(fetch_fire_o[1]);
    tail = int'(count_q) - pop_count;
    for (int i = 0; i < FETCH_DEPTH; i++) begin
      fifo_d[i] = '0;
      if (i + pop_count < FETCH_DEPTH) fifo_d[i] = fifo_q[i + pop_count];
    end

    // Include the accepted read's full reserved capacity even before it returns.
    // Same-edge FIFO consumption makes fresh space usable without a refill bubble.
    reserved = 0;
    if (inflight_valid_q && !inflight_stale_q) reserved = int'(inflight_q.count);
    used = tail + reserved;
    port_available = !inflight_valid_q || response_fire;
    if (rst_ni && !redirect_i && enable_i && !fault_wait_q && !held_valid_q
        && port_available
        && !(response_fire && !inflight_stale_q && response_error_i)) begin
      if (|next_pc_q[1:0]) begin
        local_fault = used < FETCH_DEPTH;
      end else if (used + int'(candidate.count) <= FETCH_DEPTH) begin
        fresh_offer = 1'b1;
      end
    end

    // Held unaccepted requests retain their payload across pause and redirect.
    if (rst_ni && held_valid_q) begin
      offered = held_q;
      request_valid_o = 1'b1;
    end else if (fresh_offer) begin
      offered = candidate;
      request_valid_o = 1'b1;
    end
    if (request_valid_o) request_address_o = offered.address;
    request_fire = request_valid_o && request_ready_i;

    if (response_fire) begin
      inflight_valid_d = 1'b0;
      inflight_stale_d = 1'b0;
      if (!inflight_stale_q && !redirect_i) begin
        first_packet = inflight_q.packets.lane0;
        second_packet = inflight_q.packets.lane1;
        first_packet.instruction = inflight_q.upper_word
                                   ? response_data_i[63:32] : response_data_i[31:0];
        second_packet.instruction = response_data_i[63:32];
        if (response_error_i) begin
          first_packet.instruction = '0;
          first_packet.exception.valid = 1'b1;
          first_packet.exception.cause = CAUSE_INSTR_ACCESS;
          first_packet.exception.tval = first_packet.pc;
          fault_wait_d = 1'b1;
          fifo_d[tail] = first_packet;
          tail = tail + 1;
        end else begin
          fifo_d[tail] = first_packet;
          tail = tail + 1;
          if (inflight_q.count == 2) begin
            fifo_d[tail] = second_packet;
            tail = tail + 1;
          end
        end
      end
    end
    if (local_fault) begin
      fault_packet = candidate.packets.lane0;
      fault_packet.exception.valid = 1'b1;
      fault_packet.exception.cause = CAUSE_INSTR_MISALIGNED;
      fault_packet.exception.tval = next_pc_q;
      fifo_d[tail] = fault_packet;
      tail = tail + 1;
      fault_wait_d = 1'b1;
    end
    if (fresh_offer) begin
      next_pc_d = candidate_next_pc;
      if (!request_fire) begin
        held_d = offered;
        held_valid_d = 1'b1;
        held_stale_d = 1'b0;
      end
    end
    if (request_fire) begin
      inflight_d = offered;
      inflight_valid_d = 1'b1;
      inflight_stale_d = (held_valid_q && held_stale_q) || redirect_i;
      held_valid_d = 1'b0;
      held_stale_d = 1'b0;
    end
    count_d = fetch_count_t'(tail);

    if (redirect_i) begin
      for (int i = 0; i < FETCH_DEPTH; i++) fifo_d[i] = '0;
      count_d = '0;
      next_pc_d = redirect_pc_i;
      fault_wait_d = 1'b0;
      // Sticky state cannot alias after two or more redirects during one old read.
      if (held_valid_d) held_stale_d = 1'b1;
      if (inflight_valid_d) inflight_stale_d = 1'b1;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < FETCH_DEPTH; i++) fifo_q[i] <= '0;
      count_q <= '0;
      next_pc_q <= RESET_PC;
      held_q <= '0;
      inflight_q <= '0;
      held_valid_q <= 1'b0;
      inflight_valid_q <= 1'b0;
      held_stale_q <= 1'b0;
      inflight_stale_q <= 1'b0;
      fault_wait_q <= 1'b0;
      redirect_done_o <= 1'b0;
    end else begin
      for (int i = 0; i < FETCH_DEPTH; i++) fifo_q[i] <= fifo_d[i];
      count_q <= count_d;
      next_pc_q <= next_pc_d;
      held_q <= held_d;
      inflight_q <= inflight_d;
      held_valid_q <= held_valid_d;
      inflight_valid_q <= inflight_valid_d;
      held_stale_q <= held_stale_d;
      inflight_stale_q <= inflight_stale_d;
      fault_wait_q <= fault_wait_d;
      redirect_done_o <= redirect_i;
    end
  end
endmodule
