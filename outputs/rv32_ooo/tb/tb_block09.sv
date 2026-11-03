// Fetch verification with an independent single-response memory and expected packet queue.
`timescale 1ns / 1ps

module tb_block09;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1, enable, redirect, redirect_done, predict_enable;
  word_t redirect_pc, prediction_pc0, prediction_pc1, request_address;
  prediction_pair_t prediction;
  logic request_valid, request_ready, response_valid, response_ready, response_error;
  fetch_row_t response_data;
  logic [1:0] fetch_valid, fetch_ready, fetch_fire;
  fetch_pair_t fetched;
  fetch_count_t occupancy;
  logic transaction_pending, stale_pending, fault_pending, protocol_error;

  typedef struct packed {
    fetch_pair_t packets;
    word_t address;
    logic [1:0] count;
    logic stale;
  } expected_read_t;
  expected_read_t offered_read, memory_read;
  logic offered_valid, memory_pending, memory_error;
  integer memory_delay, next_delay;
  logic allow_request, release_response, next_error, force_spurious;
  integer prediction_mode;
  word_t prediction_anchor, prediction_target, prediction_salt;
  word_t reference_pc, rng = 32'h91c0ffee;
  fetch_packet_t expected_queue [0:7];
  integer queue_count;
  logic reference_halt, reference_ack;
  logic [1:0] last_fire;
  fetch_pair_t last_delivered;
  integer checks = 0, failures = 0, cycle_number = 0;
  integer accepted_reads = 0, delivered_packets = 0, stale_returns = 0, fault_tokens = 0;
  integer held_cycles = 0, paused_pending = 0, redirect_events = 0, delayed_cycles = 0;
  integer request_before, delivered_before, n, mode, limit;
  fetch_pair_t held_pair;
  prediction_t retained_prediction;

  rv32_fetch dut (
    .clk_i(clk), .rst_ni(rst_n), .enable_i(enable),
    .redirect_i(redirect), .redirect_pc_i(redirect_pc), .redirect_done_o(redirect_done),
    .predict_enable_i(predict_enable), .prediction_pc0_o(prediction_pc0),
    .prediction_pc1_o(prediction_pc1), .prediction_i(prediction),
    .request_valid_o(request_valid), .request_ready_i(request_ready),
    .request_address_o(request_address), .response_valid_i(response_valid),
    .response_ready_o(response_ready), .response_data_i(response_data),
    .response_error_i(response_error), .fetch_valid_o(fetch_valid), .fetch_ready_i(fetch_ready),
    .fetch_o(fetched), .fetch_fire_o(fetch_fire), .occupancy_o(occupancy),
    .transaction_pending_o(transaction_pending), .stale_pending_o(stale_pending),
    .fault_pending_o(fault_pending), .protocol_error_o(protocol_error)
  );

  function automatic word_t next_random(input word_t value);
    next_random = {value[30:0], value[31] ^ value[21] ^ value[1] ^ value[0]};
  endfunction

  function automatic word_t memory_word(input word_t address);
    memory_word = 32'ha5000013 ^ (address * 32'h00010101);
  endfunction

  function automatic prediction_t predicted(
    input word_t pc, input logic enabled, input integer kind,
    input word_t anchor, target, salt
  );
    prediction_t result;
    begin
      result = '0;
      result.next_pc = pc + 32'd4;
      result.pht_index = pc[8:2];
      if (enabled) begin
        result.pht_index = pc[8:2] ^ salt[6:0];
        result.taken = (pc[2] ^ salt[0]);
        result.btb_hit = pc[3] ^ salt[1];
        if (kind != 0 && pc == anchor) begin
          result.next_pc = target; result.taken = 1; result.btb_hit = 1;
        end
      end
      predicted = result;
    end
  endfunction

  always_comb begin
    prediction.lane0 = predicted(prediction_pc0, 1'b1, prediction_mode,
                                 prediction_anchor, prediction_target, prediction_salt);
    prediction.lane1 = predicted(prediction_pc1, 1'b1, prediction_mode,
                                 prediction_anchor, prediction_target, prediction_salt);
    response_valid = force_spurious
                     || (memory_pending && memory_delay == 0 && release_response);
    response_data = '0;
    response_error = 0;
    if (memory_pending) begin
      response_data = {memory_word(memory_read.address + 32'd4), memory_word(memory_read.address)};
      response_error = memory_error;
    end
    request_ready = allow_request && (!memory_pending || (response_valid && response_ready));
  end

  function automatic expected_read_t describe_read(input word_t first_pc);
    expected_read_t result;
    begin
      result = '0;
      result.address = {first_pc[31:3], 3'b0};
      result.packets.lane0.pc = first_pc;
      result.packets.lane1.pc = first_pc + 32'd4;
      result.packets.lane0.prediction = predicted(first_pc, predict_enable, prediction_mode,
                                                prediction_anchor, prediction_target,
                                                prediction_salt);
      result.packets.lane1.prediction = predicted(first_pc + 32'd4, predict_enable,
                                                prediction_mode, prediction_anchor,
                                                prediction_target, prediction_salt);
      result.count = 1;
      if (!first_pc[2]
          && result.packets.lane0.prediction.next_pc == first_pc + 32'd4) result.count = 2;
      describe_read = result;
    end
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 40)
          $display("FAIL: %s cycle=%0d time=%0t", label_text, cycle_number, $time);
      end
    end
  endtask

  task automatic push_expected(input fetch_packet_t packet);
    begin
      check(queue_count < FETCH_DEPTH, "response/local fault has reserved buffer capacity");
      if (queue_count < 8) expected_queue[queue_count] = packet;
      queue_count = queue_count + 1;
    end
  endtask

  // This memory model owns one accepted read. Delays and request backpressure are independent.
  // Expected packets are constructed from accepted descriptors, not DUT internal state.
  task automatic cycle;
    logic request_transfer, response_transfer, was_error, was_pending, stale_old;
    logic local_fault_observed;
    logic [1:0] expected_valid, expected_fire;
    fetch_pair_t expected_output;
    expected_read_t accepted_read, old_read;
    fetch_packet_t first, second, local_packet;
    word_t old_reference_pc;
    prediction_t local_prediction;
    integer consumed, index;
    begin
      #1;
      expected_valid = 0; expected_output = '0; expected_fire = 0;
      if (!redirect) begin
        if (queue_count > 0) begin
          expected_valid[0] = 1; expected_output.lane0 = expected_queue[0];
        end
        if (queue_count > 1) begin
          expected_valid[1] = 1; expected_output.lane1 = expected_queue[1];
        end
        if (fetch_ready != 2'b10) begin
          expected_fire[0] = expected_valid[0] && fetch_ready[0];
          expected_fire[1] = expected_fire[0] && expected_valid[1] && fetch_ready[1];
        end
      end
      check(fetch_valid == expected_valid && fetched == expected_output,
            "visible instruction records match independent response/redirect queue");
      check(fetch_fire == expected_fire, "downstream consumes only a legal ordered prefix");
      check(occupancy == queue_count && queue_count <= FETCH_DEPTH,
            "FIFO count matches expected queue");
      check(prediction_pc0 == reference_pc && prediction_pc1 == reference_pc + 32'd4,
            "prediction lookup follows the saved future PC");
      check(response_ready == memory_pending,
            "reserved response capacity drains pending reads even during downstream stalls");
      check(queue_count + ((memory_pending && !memory_read.stale) ? memory_read.count : 0)
            + ((offered_valid && !offered_read.stale) ? offered_read.count : 0) <= FETCH_DEPTH,
            "buffer contents and every live promised response fit the four-entry credit budget");
      check(transaction_pending == (offered_valid || memory_pending),
            "pending status distinguishes held offers/accepted transactions from fresh wires");
      check(stale_pending == ((offered_valid && offered_read.stale)
                             || (memory_pending && memory_read.stale)),
            "stale ownership is sticky until the matching response drains");
      check(fault_pending == reference_halt, "fetch-fault stop state persists until redirect");
      check(redirect_done == reference_ack,
            "redirect acknowledgement follows its application edge");
      check(protocol_error == (fetch_ready == 2'b10 || (force_spurious && !memory_pending)),
            "only illegal prefix/unowned-response traffic diagnoses protocol errors");
      if (offered_valid) begin
        check(request_valid && request_address == offered_read.address,
              "unaccepted request remains stable across pause, metadata changes, and redirects");
        held_cycles = held_cycles + 1;
      end
      if (!enable && (offered_valid || memory_pending)) paused_pending = paused_pending + 1;
      if (memory_pending && memory_delay != 0) delayed_cycles = delayed_cycles + 1;
      old_reference_pc = reference_pc;
      local_prediction = predicted(reference_pc, predict_enable, prediction_mode,
                                   prediction_anchor, prediction_target, prediction_salt);
      local_fault_observed = 0;
      last_fire = expected_fire;
      last_delivered = expected_output;
      consumed = int'(expected_fire[0]) + int'(expected_fire[1]);
      delivered_packets = delivered_packets + consumed;
      for (index = 0; index < 8; index = index + 1) begin
        if (index + consumed < queue_count) expected_queue[index] = expected_queue[index+consumed];
        else expected_queue[index] = '0;
      end
      queue_count = queue_count - consumed;
      request_transfer = request_valid && request_ready;
      response_transfer = response_valid && response_ready;
      was_pending = memory_pending;
      was_error = memory_error;
      old_read = memory_read;
      stale_old = memory_read.stale || redirect;
      if (redirect) begin
        if (offered_valid) offered_read.stale = 1;
        if (memory_pending) memory_read.stale = 1;
      end
      if (request_valid && !offered_valid) begin
        accepted_read = describe_read(reference_pc);
        check(!reference_halt && enable && !redirect && reference_pc[1:0] == 0,
              "fresh request uses an enabled, aligned, nonfaulted PC");
        check(!memory_pending || response_transfer,
              "one outstanding accepted transaction allows only pop/refill overlap");
        check(request_address == accepted_read.address && request_address[2:0] == 0,
              "request is the expected aligned 64-bit row");
        offered_read = accepted_read;
        offered_valid = 1;
        reference_pc = accepted_read.count == 2
                       ? accepted_read.packets.lane1.prediction.next_pc
                       : accepted_read.packets.lane0.prediction.next_pc;
      end
      accepted_read = offered_read;
      if (request_transfer) begin
        check(offered_valid, "every accepted request has captured PC/prediction ownership");
        offered_valid = 0;
        accepted_reads = accepted_reads + 1;
      end
      if (redirect) begin
        reference_pc = redirect_pc;
        reference_halt = 0;
        queue_count = 0;
        for (index = 0; index < 8; index = index + 1) expected_queue[index] = '0;
        accepted_read.stale = 1;
        redirect_events = redirect_events + 1;
      end else if (response_transfer) begin
        check(was_pending, "response consumption has an accepted descriptor");
        if (stale_old) stale_returns = stale_returns + 1;
        else begin
          first = old_read.packets.lane0;
          second = old_read.packets.lane1;
          first.instruction = memory_word(first.pc);
          second.instruction = memory_word(second.pc);
          if (was_error) begin
            first.instruction = 0;
            first.exception.valid = 1;
            first.exception.cause = CAUSE_INSTR_ACCESS;
            first.exception.tval = first.pc;
            reference_halt = 1; fault_tokens = fault_tokens + 1;
            push_expected(first);
          end else begin
            push_expected(first);
            if (old_read.count == 2) push_expected(second);
          end
        end
      end
      if (redirect && response_transfer && was_pending) stale_returns = stale_returns + 1;

      #1; clk = 1; #1; clk = 0;
      if (response_transfer) memory_pending = 0;
      else if (memory_pending && memory_delay > 0) memory_delay = memory_delay - 1;
      if (request_transfer) begin
        memory_pending = 1;
        memory_read = accepted_read;
        memory_delay = next_delay;
        memory_error = next_error;
      end
      reference_ack = redirect;
      #1;
      if (!reference_halt && fault_pending && !redirect) begin
        // Local bad-PC faults have no memory transaction to enqueue a descriptor.
        // Observe their stop transition, then require the exact architectural token.
        check(old_reference_pc[1:0] != 0 && enable && !request_transfer,
              "new local fault corresponds to a misaligned saved PC, not a memory request");
        local_packet = '0;
        local_packet.pc = old_reference_pc;
        local_packet.prediction = local_prediction;
        local_packet.exception.valid = 1;
        local_packet.exception.cause = CAUSE_INSTR_MISALIGNED;
        local_packet.exception.tval = old_reference_pc;
        push_expected(local_packet);
        reference_halt = 1;
        fault_tokens = fault_tokens + 1;
        local_fault_observed = 1;
      end
      check(occupancy == queue_count && fault_pending == reference_halt,
            "post-edge instruction count and fault-stop state match expected effects");
      if (local_fault_observed)
        check(!request_valid, "local instruction-alignment fault issues no read");
      cycle_number = cycle_number + 1;
    end
  endtask

  task automatic neutral_controls;
    begin
      enable = 1; redirect = 0; redirect_pc = 0; predict_enable = 0;
      fetch_ready = 3; allow_request = 1; release_response = 1;
      next_delay = 0; next_error = 0; force_spurious = 0;
      prediction_mode = 0; prediction_anchor = 0; prediction_target = 0; prediction_salt = 0;
    end
  endtask

  task automatic reset_dut;
    begin
      neutral_controls; rst_n = 0; clk = 0;
      offered_read = '0; memory_read = '0; offered_valid = 0; memory_pending = 0;
      memory_error = 0; memory_delay = 0; reference_pc = RESET_PC;
      queue_count = 0; reference_halt = 0; reference_ack = 0;
      for (integer index = 0; index < 8; index = index + 1) expected_queue[index] = '0;
      #2;
      check(fetch_valid == 0 && fetch_fire == 0 && !request_valid && !response_ready,
            "global reset masks all instruction/memory handshakes");
      check(occupancy == 0 && !transaction_pending && !stale_pending && !fault_pending,
            "global reset clears buffered instruction and transaction ownership");
      rst_n = 1; #1;
    end
  endtask

  task automatic redirect_to(input word_t pc);
    begin redirect = 1; redirect_pc = pc; cycle; redirect = 0; end
  endtask

  task automatic wait_output(input word_t pc);
    begin
      fetch_ready = 0; limit = 0;
      while (!fetch_valid[0] && limit < 30) begin cycle; limit = limit + 1; end
      check(limit < 30 && fetch_valid[0] && fetched.lane0.pc == pc,
            "new path eventually presents the requested first instruction");
    end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block09/block09.vcd");
      $dumpvars(0, tb_block09);
    end
    #3000000; $fatal(1, "Block 9 fetch test timed out");
  end

  initial begin
    reset_dut;
    check($bits(fetch_row_t) == 64 && $bits(fetch_count_t) == 3,
          "one read supplies two instructions and occupancy includes four entries");
    cycle; cycle; #1;
    check(fetch_valid == 3 && fetched.lane0.pc == 0 && fetched.lane1.pc == 4
          && fetched.lane0.instruction == memory_word(0)
          && fetched.lane1.instruction == memory_word(4),
          "one 64-bit row becomes two correctly ordered 32-bit instructions");
    request_before = accepted_reads;
    delivered_before = delivered_packets;
    for (n = 0; n < 16; n = n + 1) begin
      cycle;
      check(last_fire == 3, "one-cycle memory sustains two consumed instructions every cycle");
    end
    check(accepted_reads-request_before == 16 && delivered_packets-delivered_before == 32,
          "steady state fetches one row and supplies two instructions per edge");

    fetch_ready = 0;
    for (n = 0; n < 6; n = n + 1) cycle;
    check(occupancy == 4 && !transaction_pending && !request_valid,
          "four-entry buffer stops new reads after all reserved capacity is used");
    held_pair = fetched;
    for (n = 0; n < 4; n = n + 1) begin
      cycle;
      check(fetched == held_pair, "whole-prefix backpressure preserves both visible records");
    end
    fetch_ready = 1; cycle;
    check(fetched.lane0 == held_pair.lane1, "partial acceptance compacts the unconsumed suffix");
    for (n = 0; n < 9; n = n + 1) cycle;
    fetch_ready = 3;
    for (n = 0; n < 8; n = n + 1) cycle;

    // Upper-half entry and wraparound exercise row extraction independently of request alignment.
    for (mode = 0; mode < 2; mode = mode + 1) begin
      reset_dut; enable = 0; cycle;
      if (mode == 0) redirect_to(32'h104);
      else redirect_to(32'hfffffffc);
      enable = 1;
      if (mode == 0) wait_output(32'h104);
      else wait_output(32'hfffffffc);
      check(fetch_valid[0] && fetched.lane0.instruction == memory_word(fetched.lane0.pc),
            "PC at row word 1 selects high instruction bits without inventing a preceding word");
      fetch_ready = 3;
      for (n = 0; n < 10; n = n + 1) cycle;
    end

    // Prediction on word 0 excludes word 1; prediction on word 1 selects the following row.
    for (mode = 0; mode < 2; mode = mode + 1) begin
      reset_dut; predict_enable = 1; prediction_mode = 1; prediction_salt = 32'h35;
      prediction_anchor = mode == 0 ? 0 : 4;
      prediction_target = mode == 0 ? 32'h80 : 32'h100;
      fetch_ready = 0; cycle; cycle;
      check(fetch_valid[0] && fetched.lane0.pc == 0,
            "prediction retains its matching instruction PC");
      if (mode == 0)
        check(occupancy == 1 && fetched.lane0.prediction.next_pc == 32'h80,
              "nonsequential word-0 prediction suppresses the unused second word");
      else check(occupancy == 2 && fetched.lane1.pc == 4
                 && fetched.lane1.prediction.next_pc == 32'h100,
                 "word-1 prediction preserves both row instructions before redirecting next fetch");
      fetch_ready = 3;
      for (n = 0; n < 14; n = n + 1) cycle;
    end

    // Prediction metadata is captured when an unaccepted request first appears.
    reset_dut; predict_enable = 1; prediction_mode = 1;
    prediction_anchor = 0; prediction_target = 32'h200; prediction_salt = 32'h5b;
    allow_request = 0; fetch_ready = 0; cycle;
    retained_prediction = offered_read.packets.lane0.prediction;
    prediction_mode = 0; prediction_salt = 32'h12;
    for (n = 0; n < 4; n = n + 1) cycle;
    allow_request = 1; cycle; enable = 0; cycle;
    check(fetch_valid[0] && occupancy == 1 && fetched.lane0.prediction == retained_prediction,
          "stalled request returns its captured prediction rather than newer table contents");

    // Three redirects cannot make the original delayed response look current again.
    reset_dut; fetch_ready = 0; next_delay = 5; release_response = 0; cycle;
    redirect_to(32'h80); redirect_to(32'h100); redirect_to(32'h180);
    check(stale_pending && transaction_pending, "multiple redirects retain stale ownership");
    next_delay = 0; release_response = 1; wait_output(32'h180);
    check(!fault_pending, "discarded old response does not stop the new path");

    // An unaccepted request survives redirects and pause; only the latest restart PC is used.
    reset_dut; allow_request = 0; fetch_ready = 0; cycle;
    enable = 0; redirect_to(32'h400); redirect_to(32'h600); redirect_to(32'h800);
    check(request_valid && request_address == 0 && stale_pending,
          "held request keeps old address while latest redirect PC is saved separately");
    for (n = 0; n < 3; n = n + 1) cycle;
    allow_request = 1; cycle;
    for (n = 0; n < 4; n = n + 1) cycle;
    check(occupancy == 0 && !transaction_pending && !request_valid,
          "paused fetch drains a stale accepted read without admitting fresh work");
    enable = 1; wait_output(32'h800);

    // A response accepted on the redirect edge must be discarded, not briefly exposed.
    reset_dut; fetch_ready = 0; cycle;
    check(response_valid && response_ready, "old response is ready before the redirect edge");
    redirect_to(32'h240);
    check(occupancy == 0, "redirect wins over simultaneous old response enqueue");
    wait_output(32'h240);

    // Bus error creates one first-PC fault token, including upper-half and stale-error cases.
    for (mode = 0; mode < 2; mode = mode + 1) begin
      reset_dut; enable = 0; cycle;
      if (mode == 1) redirect_to(32'h304);
      next_error = 1; enable = 1; fetch_ready = 0; cycle;
      next_error = 0; enable = 0; cycle;
      check(occupancy == 1 && fault_pending && fetched.lane0.exception.valid
            && fetched.lane0.exception.cause == CAUSE_INSTR_ACCESS
            && fetched.lane0.exception.tval == (mode == 0 ? 0 : 32'h304)
            && fetched.lane0.instruction == 0,
            "memory error produces exactly one precise first-PC access-fault record");
      fetch_ready = 3; enable = 1;
      for (n = 0; n < 5; n = n + 1) cycle;
      check(fault_pending && !request_valid && occupancy == 0,
            "consuming the fault token does not restart fetching before a redirect");
      redirect_to(32'h380); wait_output(32'h380);
    end
    reset_dut; next_error = 1; release_response = 0; cycle;
    next_error = 0; redirect_to(32'h480); release_response = 1;
    wait_output(32'h480);
    check(!fault_pending && !fetched.lane0.exception.valid,
          "stale error responses cannot create traps");

    // Misaligned PCs create local tokens without issuing a memory request.
    for (mode = 0; mode < 3; mode = mode + 1) begin
      reset_dut; enable = 0; cycle; redirect_to(32'h500 + mode + 1);
      request_before = accepted_reads; enable = 1; fetch_ready = 0; cycle;
      check(fault_pending && occupancy == 1 && !request_valid
            && fetched.lane0.exception.cause == CAUSE_INSTR_MISALIGNED
            && fetched.lane0.exception.tval == 32'h500 + mode + 1
            && accepted_reads == request_before,
            "misaligned instruction PC becomes one local fault rather than a BRAM read");
      fetch_ready = 3;
      for (n = 0; n < 4; n = n + 1) cycle;
      check(fault_pending && occupancy == 0,
            "local fault waits for a controller redirect after delivery");
    end
    reset_dut; predict_enable = 1; prediction_mode = 1; prediction_anchor = 4;
    prediction_target = 2; fetch_ready = 0; cycle; cycle;
    check(occupancy == 3 && fault_pending,
          "a misaligned predicted successor queues its fault behind both preceding instructions");

    // A younger predicted-PC fault must not pass an older delayed instruction response.
    reset_dut; predict_enable = 1; prediction_mode = 1; prediction_anchor = 4;
    prediction_target = 2; fetch_ready = 0; release_response = 0; cycle;
    for (n = 0; n < 5; n = n + 1) begin
      cycle;
      check(!fault_pending && occupancy == 0 && memory_pending && !request_valid,
            "delayed older read keeps its younger local fault out of the visible queue");
    end
    release_response = 1; cycle;
    check(occupancy == 3 && fetched.lane0.pc == 0 && fetched.lane1.pc == 4
          && !fetched.lane0.exception.valid && !fetched.lane1.exception.valid,
          "delayed successful pair precedes the predicted-successor fault");
    fetch_ready = 3; cycle;
    check(fetch_valid == 1 && fetched.lane0.pc == 2
          && fetched.lane0.exception.cause == CAUSE_INSTR_MISALIGNED,
          "the younger alignment fault becomes oldest only after the successful pair leaves");
    cycle;

    // An older access fault suppresses a younger local fault at the predicted next PC.
    reset_dut; predict_enable = 1; prediction_mode = 1; prediction_anchor = 4;
    prediction_target = 2; fetch_ready = 0; release_response = 0; next_error = 1; cycle;
    next_error = 0;
    for (n = 0; n < 3; n = n + 1) begin
      cycle;
      check(!fault_pending && occupancy == 0, "no younger fault bypasses the delayed access");
    end
    release_response = 1; cycle;
    check(occupancy == 1 && fetch_valid == 1 && fetched.lane0.pc == 0
          && fetched.lane0.exception.cause == CAUSE_INSTR_ACCESS
          && fetched.lane0.exception.tval == 0,
          "returning access fault wins over the misaligned predicted successor");
    fetch_ready = 3;
    for (n = 0; n < 4; n = n + 1) cycle;
    check(fault_pending && occupancy == 0 && !request_valid,
          "one older access-fault token halts fetch without adding a second local fault");

    // Illegal ready prefix and unowned response are diagnostic-only protocol violations.
    reset_dut; fetch_ready = 0;
    for (n = 0; n < 5; n = n + 1) cycle;
    held_pair = fetched; fetch_ready = 2;
    for (n = 0; n < 3; n = n + 1) begin
      cycle;
      check(last_fire == 0 && fetched == held_pair, "ready mask 10 cannot skip the oldest word");
    end
    reset_dut; enable = 0; force_spurious = 1;
    for (n = 0; n < 2; n = n + 1) cycle;
    check(!response_ready && occupancy == 0 && !fault_pending,
          "unowned response is not mistaken for an instruction or fault");
    force_spurious = 0;

    // Mixed legal stalls, changing predictions, redirects, pause, and response errors.
    reset_dut;
    for (n = 0; n < 2400; n = n + 1) begin
      rng = next_random(rng); allow_request = rng[0] || rng[1];
      release_response = rng[2] || rng[3]; enable = rng[4] || rng[5];
      case (rng[7:6])
        0: fetch_ready = 0;
        1: fetch_ready = 1;
        default: fetch_ready = 3;
      endcase
      predict_enable = rng[8]; prediction_salt = rng;
      prediction_mode = rng[9]; prediction_anchor = prediction_pc0;
      prediction_target = {22'b0, rng[11:2]};
      prediction_target[1:0] = 0;
      next_delay = rng[13:12]; next_error = n % 113 == 0;
      redirect = n % 37 == 0 || (fault_pending && n % 3 == 0);
      redirect_pc = {20'b0, rng[11:2], 2'b0};
      if (n % 211 == 0) begin redirect = 1; redirect_pc[1:0] = 2; end
      cycle;
    end
    redirect = 0; enable = 1; allow_request = 1; release_response = 1; fetch_ready = 3;
    next_delay = 0; next_error = 0; predict_enable = 0;
    redirect_to(32'h1000);
    for (n = 0; n < 15; n = n + 1) cycle;
    check(held_cycles > 10 && stale_returns > 10 && fault_tokens > 5
          && paused_pending > 5 && delayed_cycles > 10 && redirect_events > 10,
          "directed/random traffic exercises every major wait, stale, and fault mechanism");

    // Reset with an accepted delayed response, and again with an unaccepted held offer.
    reset_dut; release_response = 0; next_delay = 7; cycle; reset_dut;
    allow_request = 0; cycle; reset_dut;
    for (n = 0; n < 8; n = n + 1) cycle;
    $display("BLOCK09 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Block 9 fetch tests failed");
    $finish;
  end
endmodule
