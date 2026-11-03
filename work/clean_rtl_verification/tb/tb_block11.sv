// Recovery ownership-barrier tests. Requests are normally pulses offered only while idle.
// Busy requests are deliberately injected below to verify that they cannot replace the target.
`timescale 1ns / 1ps

module tb_block11;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1;
  logic request, quiescent, acknowledged, cancel, apply, busy, done;
  word_t target, redirect_pc, expected_target;
  word_t rng = 32'h7311cafe;
  integer checks = 0, failures = 0, cycle_number = 0;
  integer apply_edges = 0, completed_recoveries = 0;
  integer iteration, delay_cycles, position, choice, before_apply;

  rv32_recovery_controller dut (
    .clk_i(clk), .rst_ni(rst_n), .request_i(request), .target_i(target),
    .units_quiescent_i(quiescent), .cancel_ack_i(acknowledged),
    .cancel_o(cancel), .apply_o(apply), .redirect_pc_o(redirect_pc),
    .busy_o(busy), .done_o(done)
  );

  function automatic word_t next_random(input word_t value);
    next_random = {value[30:0], value[31] ^ value[21] ^ value[1] ^ value[0]};
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 30)
          $display("FAIL: %s cycle=%0d time=%0t", label_text, cycle_number, $time);
      end
    end
  endtask

  task automatic edge_step;
    begin
      #1;
      check(!apply || (cancel && busy), "safe apply retains cancellation throughout its edge");
      check(!done || (!busy && !cancel && !apply), "done is an idle-state completion pulse");
      if (apply) apply_edges = apply_edges + 1;
      #1; clk = 1; #1; clk = 0; #1;
      cycle_number = cycle_number + 1;
    end
  endtask

  task automatic check_idle(input logic completion_pulse);
    begin
      check(!busy && !cancel && !apply && done == completion_pulse,
            "idle exposes neither cancellation nor reusable-state flush");
      check(redirect_pc == expected_target, "target remains retained until another request/reset");
    end
  endtask

  task automatic reset_controller;
    begin
      rst_n = 0; request = 0; target = 32'hdeadbeef;
      quiescent = 0; acknowledged = 0; expected_target = RESET_PC;
      #2;
      check(!cancel && !apply && !busy && !done && redirect_pc == RESET_PC,
            "asynchronous reset clears both active recovery and its saved target");
      rst_n = 1; #1; check_idle(0);
    end
  endtask

  task automatic start_recovery(input word_t destination, input logic already_ready);
    begin
      check(!busy, "caller starts a new ownership recovery only while idle");
      request = 1; target = destination;
      quiescent = already_ready; acknowledged = already_ready;
      #1;
      check(!cancel && !apply && !busy, "new request does not bypass clocked target capture");
      expected_target = destination;
      edge_step;
      request = 0; target = ~destination; #1;
      check(cancel && busy && !apply && !done,
            "every request enters cancellation before apply, even when producers were idle");
      check(redirect_pc == destination, "request edge captures all target bits");
    end
  endtask

  task automatic hold_for_one_edge(input logic have_ack, input logic have_quiet);
    begin
      acknowledged = have_ack; quiescent = have_quiet;
      // A caller violating the idle-only request contract must not corrupt the active owner.
      request = 1; target = rng ^ 32'h55aa0011;
      #1;
      check(cancel && busy && !apply && !done && redirect_pc == expected_target,
            "cancellation holds the captured owner while incomplete barrier inputs wait");
      edge_step;
      check(cancel && busy && !apply && !done && redirect_pc == expected_target,
            "one barrier input alone never authorizes early resource reuse");
    end
  endtask

  task automatic release_and_finish;
    begin
      before_apply = apply_edges;
      request = 1; target = expected_target ^ 32'h11112222;
      quiescent = 1; acknowledged = 1;
      #1;
      check(cancel && !apply && redirect_pc == expected_target,
            "both barrier inputs still require their sampled clock edge");
      edge_step;
      check(cancel && apply && busy && !done && redirect_pc == expected_target,
            "simultaneous quiescence and acknowledgments authorize exactly the apply phase");
      // The barrier was sampled. Dropping it now cannot truncate the one-cycle apply pulse.
      quiescent = 0; acknowledged = 0; #1;
      check(apply && cancel, "apply is retained despite later changes to acknowledgment inputs");
      edge_step;
      request = 0;
      check_idle(1);
      check(apply_edges == before_apply + 1, "each completed recovery has exactly one apply edge");
      completed_recoveries = completed_recoveries + 1;
    end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block11/block11.vcd");
      $dumpvars(0, tb_block11);
    end
    #1000000; $fatal(1, "Recovery-controller test timed out");
  end

  initial begin
    request = 0; target = 0; quiescent = 0; acknowledged = 0;
    expected_target = RESET_PC;
    reset_controller;

    // Neither a single acknowledgment nor historical alternating inputs satisfy the barrier.
    start_recovery(32'h00000100, 0);
    hold_for_one_edge(0, 0);
    hold_for_one_edge(1, 0);
    hold_for_one_edge(0, 1);
    hold_for_one_edge(1, 0);
    hold_for_one_edge(0, 1);
    release_and_finish;
    edge_step; check_idle(0);

    // Idle producers still pass through a full cancellation and apply sequence.
    start_recovery(32'hfffffffc, 1);
    release_and_finish;
    // Start the next request on the first available RUN edge, with no extra idle bubble.
    start_recovery(32'h00000004, 0);
    hold_for_one_edge(0, 1);
    release_and_finish;
    edge_step; check_idle(0);

    // Asynchronous reset may abort either outstanding cancellation or the apply phase.
    start_recovery(32'h10000000, 0);
    hold_for_one_edge(1, 0);
    reset_controller;
    start_recovery(32'h00000080, 1);
    request = 0; edge_step;
    check(apply && busy && cancel, "directed reset fixture reaches its apply interval");
    reset_controller;
    edge_step; check_idle(0);

    // Targets and barrier arrival order vary; the release schedule is independent of the DUT.
    for (iteration = 0; iteration < 400; iteration = iteration + 1) begin
      rng = next_random(rng);
      start_recovery(rng, rng[0]);
      delay_cycles = 1 + int'(rng[6:3]);
      for (position = 0; position < delay_cycles; position = position + 1) begin
        rng = next_random(rng);
        choice = int'(rng[1:0]) % 3;
        case (choice)
          0: hold_for_one_edge(0, 0);
          1: hold_for_one_edge(1, 0);
          default: hold_for_one_edge(0, 1);
        endcase
      end
      release_and_finish;
      if (iteration % 7 == 0) begin
        // Reset the done pulse without offering another request.
        edge_step; check_idle(0);
      end
    end
    edge_step; check_idle(0);

    // Changing idle inputs and targets cannot create a flush or capture a target by itself.
    for (iteration = 0; iteration < 32; iteration = iteration + 1) begin
      rng = next_random(rng); target = rng;
      request = 0; quiescent = rng[0]; acknowledged = rng[1];
      edge_step; check_idle(0);
    end
    check(completed_recoveries == 403, "all nonaborted directed/random recoveries completed");
    $display("BLOCK11 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Recovery-controller ownership barrier failed");
    $finish;
  end
endmodule
