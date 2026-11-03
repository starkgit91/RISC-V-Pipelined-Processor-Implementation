// Five held result sources -> two retained completion lanes -> guarded register broadcasts.
`timescale 1ns / 1ps

module tb_block08;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1, cancel;
  logic [4:0] source_valid, arb_source_ready, wb_source_ready;
  completion_sources_t sources;
  completion_t source_data [0:4];
  rob_index_t head_index;
  logic [1:0] sink_ready, arb_valid, wb_valid, accepted, accept_allow, accepted_override;
  logic override_feedback;
  completion_pair_t arb_packet, wb_packet, broadcast;
  logic arb_done, wb_done, arb_quiet, wb_quiet, arb_error, wb_error;
  logic [1:0] arb_occupancy, wb_occupancy, broadcast_valid, write_enable;
  phys_reg_t write0_addr, write1_addr;
  word_t write0_data, write1_data;

  logic [1:0] ref_valid;
  completion_t ref_packet [0:1];
  logic ref_ack;
  logic [4:0] selected_sources;
  integer selected_for_slot [0:1];
  logic [1:0] expected_visible;
  completion_pair_t expected_packets;
  logic expected_arb_error;
  integer checks = 0, failures = 0, serial_number = 1;
  integer source_accept_count = 0, sink_consume_count = 0;
  integer a, b, c, d, e, mask, allow_mask, case0, case1, n, i, mode;
  integer permutation [0:4];
  integer source_before, sink_before;
  completion_t packet, other_packet, retained;
  word_t rng = 32'h8c4e19ab;

  assign sources.alu0_result = source_data[0];
  assign sources.alu1_result = source_data[1];
  assign sources.muldiv_result = source_data[2];
  assign sources.memory_result = source_data[3];
  assign sources.system_result = source_data[4];
  assign accepted = override_feedback ? accepted_override : arb_valid & sink_ready & accept_allow;

  rv32_completion_arbiter arbiter (
    .clk_i(clk), .rst_ni(rst_n), .source_valid_i(source_valid),
    .source_ready_o(arb_source_ready), .sources_i(sources), .rob_head_index_i(head_index),
    .completion_valid_o(arb_valid), .completion_ready_i(sink_ready), .completion_o(arb_packet),
    .cancel_i(cancel), .cancel_done_o(arb_done), .quiescent_o(arb_quiet),
    .occupancy_o(arb_occupancy), .protocol_error_o(arb_error)
  );
  rv32_writeback writeback (
    .clk_i(clk), .rst_ni(rst_n), .source_valid_i(source_valid),
    .source_ready_o(wb_source_ready), .sources_i(sources), .rob_head_index_i(head_index),
    .rob_completion_valid_o(wb_valid), .rob_completion_ready_i(sink_ready),
    .rob_completion_o(wb_packet), .rob_completion_accepted_i(accepted),
    .broadcast_valid_o(broadcast_valid), .broadcast_o(broadcast), .write_enable_o(write_enable),
    .write0_addr_o(write0_addr), .write1_addr_o(write1_addr),
    .write0_data_o(write0_data), .write1_data_o(write1_data),
    .cancel_i(cancel), .cancel_done_o(wb_done), .quiescent_o(wb_quiet),
    .occupancy_o(wb_occupancy), .protocol_error_o(wb_error)
  );

  function automatic word_t next_random(input word_t value);
    next_random = {value[30:0], value[31] ^ value[21] ^ value[1] ^ value[0]};
  endfunction

  function automatic completion_t result_packet(input integer id, index_value);
    completion_t result;
    begin
      result = '0;
      result.rob_tag.index = index_value;
      result.rob_tag.generation = 17 + id / ROB_DEPTH;
      result.pdst = 1 + id % 63;
      result.writes_rd = id % 3 != 0;
      result.value = 32'hac510000 ^ (id * 32'h10201);
      result.exception.valid = id % 7 == 0;
      result.exception.cause = CAUSE_LOAD_ACCESS;
      result.exception.tval = 32'h20f00000 + id * 4;
      result.control_flow.is_control = id % 2;
      result.control_flow.is_conditional = id % 3 == 0;
      result.control_flow.taken = id % 4 == 0;
      result.control_flow.mispredicted = id % 5 == 0;
      result.control_flow.next_pc = 32'h400 + id * 4;
      result.memory_access.valid = id % 2;
      result.memory_access.write = id % 3 == 0;
      result.memory_access.address = 32'h1000 + id * 4;
      result.memory_access.wdata = 32'h81828384 ^ id;
      result.memory_access.wstrb = id % 16;
      result_packet = result;
    end
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 35) $display("FAIL: %s time=%0t", label_text, $time);
      end
    end
  endtask

  // Queue state and tag-bucket selection are independent of the RTL's comparator selection.
  // Walk ages 0..15, then source indices 0..4, and place into available slots in lane order.
  task automatic evaluate;
    integer age, source_id, source_age, slot, next_slot, count;
    completion_t first, second, left, right;
    completion_pair_t expected_broadcast;
    logic [1:0] effective_accept, candidates, expected_enables;
    logic free0, free1, duplicate_owner, duplicate_destination, feedback_error;
    logic valid_left, valid_right;
    begin
      expected_visible = rst_n && !cancel ? ref_valid : 2'b00;
      expected_packets = '0;
      if (expected_visible[0]) expected_packets.lane0 = ref_packet[0];
      if (expected_visible[1]) expected_packets.lane1 = ref_packet[1];
      check(arb_valid == expected_visible && wb_valid == expected_visible,
            "registered lane validity equals independent retained-slot model");
      check(arb_packet == expected_packets && wb_packet == expected_packets,
            "each valid lane retains its complete payload; invalid lanes are zero");
      count = (ref_valid[0] ? 1 : 0) + (ref_valid[1] ? 1 : 0);
      check(arb_occupancy == count && wb_occupancy == count,
            "occupancy counts physical retained output slots");
      check(arb_quiet == (ref_valid == 0) && wb_quiet == (ref_valid == 0),
            "quiescence describes actual empty state, including during cancel");
      check(arb_done == ref_ack && wb_done == ref_ack,
            "cancel acknowledgement follows the sampled cancellation edge");

      selected_sources = 0; selected_for_slot[0] = -1; selected_for_slot[1] = -1;
      free0 = !ref_valid[0] || sink_ready[0];
      free1 = !ref_valid[1] || sink_ready[1];
      next_slot = free0 ? 0 : (free1 ? 1 : 2);
      if (rst_n && !cancel) begin
        for (age = 0; age < ROB_DEPTH; age = age + 1) begin
          for (source_id = 0; source_id < 5; source_id = source_id + 1) begin
            first = source_data[source_id];
            source_age = (int'(first.rob_tag.index) - int'(head_index) + ROB_DEPTH) % ROB_DEPTH;
            if (source_valid[source_id] && source_age == age && next_slot < 2) begin
              selected_sources[source_id] = 1;
              selected_for_slot[next_slot] = source_id;
              if (next_slot == 0 && free1) next_slot = 1;
              else next_slot = 2;
            end
          end
        end
      end
      check(arb_source_ready == selected_sources && wb_source_ready == selected_sources,
            "source acceptance follows age buckets, source tie order, and available slot count");
      check((arb_source_ready & ~source_valid) == 0,
            "only a selected valid source observes ready");

      expected_arb_error = 0;
      if (rst_n && !cancel) begin
        for (integer item0 = 0; item0 < 7; item0 = item0 + 1) begin
          if (item0 < 5) begin left = source_data[item0]; valid_left = source_valid[item0]; end
          else begin left = ref_packet[item0-5]; valid_left = ref_valid[item0-5]; end
          for (integer item1 = item0 + 1; item1 < 7; item1 = item1 + 1) begin
            if (item1 < 5) begin right = source_data[item1]; valid_right = source_valid[item1]; end
            else begin right = ref_packet[item1-5]; valid_right = ref_valid[item1-5]; end
            if (valid_left && valid_right && left.rob_tag == right.rob_tag)
              expected_arb_error = 1;
          end
        end
      end
      check(arb_error == expected_arb_error,
            "duplicate full owners are diagnosed without filtering");

      first = ref_packet[0]; second = ref_packet[1];
      effective_accept = accepted & expected_visible & sink_ready;
      candidates[0] = effective_accept[0] && first.writes_rd && !first.exception.valid &&
                      first.pdst != 0;
      candidates[1] = effective_accept[1] && second.writes_rd && !second.exception.valid &&
                      second.pdst != 0;
      duplicate_owner = effective_accept == 3 && first.rob_tag == second.rob_tag;
      duplicate_destination = candidates == 3 && first.pdst == second.pdst;
      expected_enables = candidates;
      if (duplicate_owner || duplicate_destination) expected_enables[1] = 0;
      expected_broadcast = '0;
      if (expected_enables[0]) expected_broadcast.lane0 = first;
      if (expected_enables[1]) expected_broadcast.lane1 = second;
      check(write_enable == expected_enables && broadcast_valid == expected_enables,
            "only accepted nonfaulting nonzero physical writes broadcast");
      check(broadcast == expected_broadcast, "disabled broadcast lanes carry zero payload");
      check(write0_addr == (expected_enables[0] ? first.pdst : phys_reg_t'(0)) &&
            write1_addr == (expected_enables[1] ? second.pdst : phys_reg_t'(0)),
            "PRF write selectors follow guarded accepted results");
      check(write0_data == (expected_enables[0] ? first.value : word_t'(0)) &&
            write1_data == (expected_enables[1] ? second.value : word_t'(0)),
            "PRF write values follow guarded accepted results");
      feedback_error = rst_n && !cancel && (accepted & ~(expected_visible & sink_ready)) != 0;
      check(wb_error == (expected_arb_error || feedback_error || duplicate_owner ||
                         duplicate_destination),
            "writeback combines ownership, acceptance-feedback, and destination diagnostics");
    end
  endtask

  task automatic cycle;
    logic [1:0] next_valid;
    logic [4:0] consumed;
    completion_t next0, next1;
    begin
      #1; evaluate;
      next_valid = ref_valid; next0 = ref_packet[0]; next1 = ref_packet[1];
      consumed = selected_sources & source_valid;
      if (!rst_n || cancel) begin next_valid = 0; next0 = '0; next1 = '0; end
      else begin
        for (integer slot = 0; slot < 2; slot = slot + 1) begin
          if (ref_valid[slot] && sink_ready[slot]) begin
            next_valid[slot] = 0;
            sink_consume_count = sink_consume_count + 1;
          end
          if (selected_for_slot[slot] >= 0) begin
            next_valid[slot] = 1;
            if (slot == 0) next0 = source_data[selected_for_slot[slot]];
            else next1 = source_data[selected_for_slot[slot]];
          end
        end
        for (integer source_id = 0; source_id < 5; source_id = source_id + 1)
          if (consumed[source_id]) source_accept_count = source_accept_count + 1;
      end
      #1; clk = 1; #1; clk = 0;
      ref_valid = next_valid; ref_packet[0] = next0; ref_packet[1] = next1;
      ref_ack = rst_n && cancel;
      source_valid = source_valid & ~consumed;
      #1; evaluate;
    end
  endtask

  task automatic neutral;
    begin
      source_valid = 0; sink_ready = 0; accept_allow = 3;
      head_index = 0; cancel = 0; override_feedback = 0; accepted_override = 0;
      for (integer source_id = 0; source_id < 5; source_id = source_id + 1)
        source_data[source_id] = '0;
    end
  endtask

  task automatic reset_dut;
    begin
      neutral; rst_n = 0; ref_valid = 0; ref_packet[0] = '0; ref_packet[1] = '0;
      ref_ack = 0; #2; evaluate;
      check(!arb_valid && !wb_valid && !arb_source_ready && !wb_source_ready,
            "reset masks input/output handshakes");
      rst_n = 1; #1; evaluate;
    end
  endtask

  task automatic clear_queues;
    begin
      cancel = 1; cycle;
      source_valid = 0; cancel = 0; sink_ready = 0; accept_allow = 3;
      override_feedback = 0; accepted_override = 0;
      cycle;
    end
  endtask

  task automatic offer(input integer source_id, index_value);
    begin
      check(!source_valid[source_id], "test producer replaces only an empty accepted source");
      source_data[source_id] = result_packet(serial_number, index_value);
      serial_number = serial_number + 1;
      source_valid[source_id] = 1;
    end
  endtask

  task automatic drain;
    integer limit;
    begin
      sink_ready = 3; override_feedback = 0; accept_allow = 3; limit = 0;
      while ((source_valid != 0 || ref_valid != 0) && limit < 12) begin
        cycle; limit = limit + 1;
      end
      check(source_valid == 0 && ref_valid == 0, "all buffered/source owners drain within timeout");
      sink_ready = 0;
    end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block08/block08.vcd");
      $dumpvars(0, tb_block08);
    end
    #1000000; $fatal(1, "Block 8 testbench timed out");
  end

  initial begin
    reset_dut;
    check(COMPLETION_SOURCE_COUNT == 5 && $bits(completion_sources_t) == 1110,
          "five complete source payloads use the agreed packed interface");

    // Leading case: three results arrive, only two can enter retained output slots.
    offer(0, 1); offer(1, 2); offer(2, 0); #1; evaluate;
    check(arb_source_ready == 5'b00101 && wb_valid == 0 && write_enable == 0,
          "older DIV and first ADD capture first; no source-to-ROB transparent bypass");
    cycle;
    check(source_valid == 5'b00010 && arb_valid == 3,
          "third result stays with its producer after two captures");
    retained = arb_packet.lane1;
    for (n = 0; n < 5; n = n + 1) begin
      cycle; check(arb_source_ready == 0, "full retained slots backpressure the third producer");
    end
    sink_ready = 1; cycle;
    check(arb_valid == 3 && arb_packet.lane1 == retained && source_valid == 0,
          "one consumed lane refills without moving its blocked neighbor");
    drain;

    // Arbitrary independent output readiness can leave only lane 1 valid; it must not compact.
    offer(0, 4); offer(1, 5); cycle;
    retained = arb_packet.lane1; sink_ready = 1; cycle;
    check(arb_valid == 2 && arb_packet.lane0 == '0 && arb_packet.lane1 == retained,
          "partial consumption leaves the blocked lane stationary with valid mask 10");
    for (n = 0; n < 3; n = n + 1) begin cycle; check(arb_valid == 2, "lane 1 remains blocked"); end
    offer(4, 0); cycle;
    check(arb_valid == 3 && arb_packet.lane1 == retained,
          "free lane 0 accepts a new owner independently of blocked lane 1");
    drain;

    // All source-age permutations, wrapping around ROB slot 15, have an independent oracle.
    for (a = 0; a < 5; a = a + 1) begin
      for (b = 0; b < 5; b = b + 1) begin
        for (c = 0; c < 5; c = c + 1) begin
          for (d = 0; d < 5; d = d + 1) begin
            for (e = 0; e < 5; e = e + 1) begin
              if (a != b && a != c && a != d && a != e && b != c && b != d && b != e &&
                  c != d && c != e && d != e) begin
                permutation[0] = a; permutation[1] = b; permutation[2] = c;
                permutation[3] = d; permutation[4] = e;
                head_index = (a+b) % 2 == 0 ? 0 : 14;
                source_before = source_accept_count; sink_before = sink_consume_count;
                for (i = 0; i < 5; i = i + 1) offer(i, (head_index+permutation[i]) % 16);
                cycle;
                sink_ready = (a+c) % 4; cycle;
                drain;
                check(source_accept_count-source_before == 5 && sink_consume_count-sink_before == 5,
                      "every source is captured and consumed exactly once across contention");
              end
            end
          end
        end
      end
    end

    // Equal index age, distinct generations: lower source index breaks the tie.
    for (i = 0; i < 5; i = i + 1) begin
      offer(i, 15); packet = source_data[i]; packet.rob_tag.generation = 80+i;
      source_data[i] = packet;
    end
    head_index = 15; #1; evaluate;
    check(arb_source_ready == 5'b00011, "source index supplies deterministic age ties");
    cycle; head_index = 0; sink_ready = 3; cycle; drain;

    // Every independent ROB-ready/accepted mask combination gates broadcasts and PRF writes.
    for (mask = 0; mask < 4; mask = mask + 1) begin
      for (allow_mask = 0; allow_mask < 4; allow_mask = allow_mask + 1) begin
        clear_queues;
        offer(0, 0); offer(1, 1);
        for (i = 0; i < 2; i = i + 1) begin
          packet = source_data[i]; packet.writes_rd = 1; packet.exception.valid = 0;
          packet.pdst = 32+i; source_data[i] = packet;
        end
        cycle; sink_ready = mask; accept_allow = allow_mask; #1; evaluate;
        check(write_enable == (mask & allow_mask),
              "ROB consumption without ownership acceptance creates no register effect");
        cycle; clear_queues;
      end
    end

    // Fault, nonwriter, p0, and normal-write cases on both output lanes.
    for (case0 = 0; case0 < 4; case0 = case0 + 1) begin
      for (case1 = 0; case1 < 4; case1 = case1 + 1) begin
        offer(0, 0); offer(1, 1);
        for (i = 0; i < 2; i = i + 1) begin
          packet = source_data[i]; packet.writes_rd = 1; packet.exception.valid = 0;
          packet.pdst = 32+i;
          case (i == 0 ? case0 : case1)
            1: packet.exception.valid = 1;
            2: packet.writes_rd = 0;
            3: packet.pdst = 0;
            default: begin end
          endcase
          source_data[i] = packet;
        end
        cycle; sink_ready = 3; #1; evaluate;
        check(write_enable == {(case1 == 0), (case0 == 0)},
              "faults/nonwriters/p0 never publish usable physical values");
        cycle; sink_ready = 0;
      end
    end

    // Invalid acceptance feedback is diagnosed and cannot fabricate a transfer.
    override_feedback = 1; accepted_override = 3; sink_ready = 3; #1; evaluate;
    check(wb_error && broadcast_valid == 0 && write_enable == 0,
          "accept feedback on empty lanes is rejected");
    override_feedback = 0; offer(0, 0); offer(1, 1); cycle;
    override_feedback = 1; accepted_override = 3; sink_ready = 0; #1; evaluate;
    check(wb_error && write_enable == 0, "accept feedback without ready is ignored");
    sink_ready = 1; cycle;
    sink_ready = 2; accepted_override = 3; #1; evaluate;
    check(wb_error && wb_valid == 2, "spurious acceptance of empty lane 0 is diagnosed");
    cycle; override_feedback = 0; sink_ready = 0;

    // Duplicates are forwarded to the ROB; lane-1 suppression also applies behind a fault.
    for (mode = 0; mode < 3; mode = mode + 1) begin
      offer(0, 0); offer(1, 1);
      packet = source_data[0]; other_packet = source_data[1];
      packet.writes_rd = mode != 2; packet.exception.valid = mode == 1; packet.pdst = 32;
      other_packet.writes_rd = mode != 2; other_packet.exception.valid = 0; other_packet.pdst = 33;
      other_packet.rob_tag = packet.rob_tag;
      source_data[0] = packet; source_data[1] = other_packet; #1; evaluate;
      check(arb_error && arb_source_ready == 3, "duplicate source owners diagnose but still drain");
      cycle; sink_ready = 3; #1; evaluate;
      check(arb_valid == 3 && wb_error && write_enable == (mode == 0 ? 1 : 0) &&
            broadcast_valid == (mode == 0 ? 1 : 0),
            "fake dual acceptance of duplicate owner always suppresses lane-1 register effects");
      cycle; sink_ready = 0;
    end

    offer(0, 0); cycle;
    packet = source_data[0]; source_data[4] = packet; source_valid[4] = 1; #1; evaluate;
    check(arb_error, "a source repeating a still-buffered owner is diagnosed");
    drain;

    // Distinct accepted owners cannot both write one physical destination.
    for (mode = 0; mode < 3; mode = mode + 1) begin
      offer(0, 0); offer(1, 1);
      packet = source_data[0]; other_packet = source_data[1];
      packet.writes_rd = 1; packet.exception.valid = mode == 1; packet.pdst = 41;
      other_packet.writes_rd = mode != 2; other_packet.exception.valid = 0; other_packet.pdst = 41;
      source_data[0] = packet; source_data[1] = other_packet; cycle;
      sink_ready = 3; #1; evaluate;
      if (mode == 0) check(!arb_error && wb_error && write_enable == 1,
                          "physical-destination conflict keeps lane 0 and diagnoses lane 1");
      else check(!wb_error, "fault/nonwriter cannot create a false physical-write conflict");
      cycle; sink_ready = 0;
    end

    // Cancellation kills retained state and masks otherwise-ready transfers before the edge.
    for (i = 0; i < 5; i = i + 1) offer(i, i);
    cycle; packet = arb_packet.lane0; cancel = 1; sink_ready = 3;
    override_feedback = 1; accepted_override = 3; #1; evaluate;
    check(!arb_quiet && arb_valid == 0 && arb_source_ready == 0 && write_enable == 0,
          "cancel masks transfer immediately while quiescence waits for state clearance");
    cycle; check(arb_quiet && wb_quiet && arb_done && wb_done,
                 "cancel acknowledgement follows clearing both retained slots");
    source_valid = 0; cancel = 0; override_feedback = 0; sink_ready = 0;
    packet.value = 32'h12345678; source_data[2] = packet; source_valid = 4;
    cycle; sink_ready = 3; #1; evaluate;
    check(arb_valid == 1 && arb_packet.lane0.value == 32'h12345678,
          "same owner identity can be reused after cancellation acknowledgement");
    cycle; sink_ready = 0;

    // Held producers inject only into their empty source slots during randomized traffic.
    for (n = 0; n < 1200; n = n + 1) begin
      rng = next_random(rng); head_index = rng % ROB_DEPTH;
      sink_ready = rng[1:0]; accept_allow = rng[3:2];
      for (i = 0; i < 5; i = i + 1) begin
        rng = next_random(rng);
        if (!source_valid[i] && rng[0]) offer(i, rng % ROB_DEPTH);
      end
      override_feedback = n % 47 == 0;
      accepted_override = rng[9:8];
      if (n % 79 == 78) begin
        clear_queues;
      end else cycle;
    end
    override_feedback = 0; drain;

    // Asynchronous reset removes occupied state while traffic and false feedback are present.
    offer(0, 1); offer(1, 2); offer(2, 3); cycle;
    override_feedback = 1; accepted_override = 3;
    rst_n = 0; ref_valid = 0; ref_packet[0] = '0; ref_packet[1] = '0; ref_ack = 0;
    #2; evaluate;
    check(arb_quiet && wb_quiet && !arb_done && !wb_done && write_enable == 0,
          "global reset clears retained owners and cancellation acknowledgements");
    source_valid = 0; override_feedback = 0; sink_ready = 3; rst_n = 1;
    for (n = 0; n < 4; n = n + 1) cycle;

    $display("BLOCK08 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Block 8 completion/writeback tests failed");
    $finish;
  end
endmodule
