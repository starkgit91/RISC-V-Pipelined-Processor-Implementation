// Block 4 checks rename/control state, not execution, ROB arbitration, or cancellation.
`timescale 1ns / 1ps

module tb_block04;
  import rv32_ooo_pkg::*;

  logic clk = 0;
  logic rst_n = 1;
  logic dispatch_valid;
  logic resources_ready;
  decoded_uop_t decoded;
  rob_tag_t dispatch_tag;
  logic dispatch_ready;
  renamed_uop_t renamed;
  logic src1_ready;
  logic src2_ready;
  logic completion_valid;
  completion_t completion;
  logic retire_valid;
  renamed_uop_t retire_uop;
  logic recover;
  logic recover_done;
  arch_reg_t committed_query;
  phys_reg_t committed_result;
  phys_mask_t free_mask;
  phys_mask_t ready_mask;
  phys_count_t free_count;

  // The reference stores register identities as integers rather than packed RTL arrays.
  integer ref_spec [0:ARCH_REG_COUNT-1];
  integer ref_commit [0:ARCH_REG_COUNT-1];
  logic ref_free [0:PHYS_REG_COUNT-1];
  logic ref_ready [0:PHYS_REG_COUNT-1];
  renamed_uop_t last_dispatch;
  renamed_uop_t first_writer;
  renamed_uop_t second_writer;
  renamed_uop_t saved_uop;
  renamed_uop_t queue [0:63];
  integer queue_head = 0;
  integer queue_tail = 0;
  integer queue_count = 0;
  integer checks = 0;
  integer failures = 0;
  integer sequence_number = 0;
  integer i;
  integer n;
  integer chosen;
  integer before_free;
  integer dispatched;
  integer retired;
  logic recovered;
  word_t rng = 32'hc01df00d;

  rv32_rename dut (
    .clk_i(clk), .rst_ni(rst_n),
    .dispatch_valid_i(dispatch_valid), .dispatch_resources_ready_i(resources_ready),
    .decoded_i(decoded), .dispatch_rob_tag_i(dispatch_tag),
    .dispatch_ready_o(dispatch_ready), .renamed_o(renamed),
    .src1_ready_o(src1_ready), .src2_ready_o(src2_ready),
    .completion_valid_i(completion_valid), .completion_i(completion),
    .retire_valid_i(retire_valid), .retire_uop_i(retire_uop),
    .recover_i(recover), .recover_done_o(recover_done),
    .committed_query_arch_i(committed_query), .committed_query_phys_o(committed_result),
    .free_mask_o(free_mask), .ready_mask_o(ready_mask), .free_count_o(free_count)
  );

  function automatic word_t next_random(input word_t state);
    next_random = {state[30:0], state[31] ^ state[21] ^ state[1] ^ state[0]};
  endfunction

  function automatic integer lowest_free;
    integer r;
    begin
      lowest_free = 0;
      for (r = PHYS_REG_COUNT-1; r >= 1; r = r - 1)
        if (ref_free[r]) lowest_free = r;
    end
  endfunction

  function automatic integer number_free;
    integer r;
    begin
      number_free = 0;
      for (r = 1; r < PHYS_REG_COUNT; r = r + 1)
        if (ref_free[r]) number_free = number_free + 1;
    end
  endfunction

  function automatic logic destination_needed;
    destination_needed = decoded.writes_rd && decoded.rd != 0 && !decoded.fetch.exception.valid;
  endfunction

  function automatic logic expected_ready(input integer physical);
    begin
      expected_ready = physical == 0 || ref_ready[physical];
      if (completion_valid && completion.writes_rd && !completion.exception.valid &&
          completion.pdst != 0 && !ref_free[completion.pdst] && completion.pdst == physical)
        expected_ready = 1;
    end
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 32)
          $display("FAIL: %s time=%0t free=%0d", label_text, $time, free_count);
      end
    end
  endtask

  task automatic neutral;
    begin
      dispatch_valid = 0;
      resources_ready = 1;
      decoded = '0;
      dispatch_tag = '0;
      completion_valid = 0;
      completion = '0;
      retire_valid = 0;
      retire_uop = '0;
      recover = 0;
      committed_query = 0;
    end
  endtask

  task automatic reset_reference;
    integer r;
    begin
      for (r = 0; r < ARCH_REG_COUNT; r = r + 1) begin
        ref_spec[r] = r;
        ref_commit[r] = r;
      end
      for (r = 0; r < PHYS_REG_COUNT; r = r + 1) begin
        ref_free[r] = r >= ARCH_REG_COUNT;
        ref_ready[r] = r < ARCH_REG_COUNT;
      end
    end
  endtask

  task automatic writer(input integer rd, rs1, rs2);
    begin
      decoded = '0;
      decoded.fetch.pc = sequence_number * 4;
      decoded.fetch.prediction.next_pc = decoded.fetch.pc + 4;
      decoded.op_class = UOP_INTEGER;
      decoded.alu_op = ALU_ADD;
      decoded.writes_rd = 1;
      decoded.uses_rs1 = 1;
      decoded.uses_rs2 = 1;
      decoded.rd = rd;
      decoded.rs1 = rs1;
      decoded.rs2 = rs2;
      dispatch_tag.index = sequence_number % ROB_DEPTH;
      dispatch_tag.generation = sequence_number / ROB_DEPTH;
      dispatch_valid = 1;
      sequence_number = sequence_number + 1;
    end
  endtask

  task automatic complete_uop(input renamed_uop_t uop);
    begin
      completion_valid = 1;
      completion = '0;
      completion.rob_tag = uop.rob_tag;
      completion.pdst = uop.pdst;
      completion.writes_rd = uop.decoded.writes_rd;
      completion.control_flow.next_pc = uop.decoded.fetch.pc + 4;
    end
  endtask

  task automatic check_comb;
    integer s1;
    integer s2;
    integer candidate;
    logic want_ready;
    begin
      candidate = lowest_free();
      s1 = decoded.uses_rs1 && !decoded.fetch.exception.valid ? ref_spec[decoded.rs1] : 0;
      s2 = decoded.uses_rs2 && !decoded.fetch.exception.valid ? ref_spec[decoded.rs2] : 0;
      want_ready = rst_n && !recover && resources_ready &&
                   (!destination_needed() || candidate != 0);
      check(dispatch_ready == want_ready, "dispatch readiness includes every allocation gate");
      check(renamed.prs1 == s1 && renamed.prs2 == s2,
            "source mappings are captured before the destination update");
      check(src1_ready == expected_ready(s1) && src2_ready == expected_ready(s2),
            "source readiness includes only valid completion bypass");
      check(renamed.decoded.writes_rd == destination_needed(), "real destination write flag");
      check(renamed.rob_tag == dispatch_tag, "dispatch retains the allocated ROB identity");
      check(renamed.decoded.fetch == decoded.fetch, "PC/prediction/fault metadata pass unchanged");
      if (destination_needed()) begin
        check(renamed.pdst == candidate && renamed.old_pdst == ref_spec[decoded.rd],
              "lowest available destination and replaced mapping");
      end else begin
        check(renamed.pdst == 0 && renamed.old_pdst == 0, "nonwriters allocate no physical tag");
      end
      if (decoded.fetch.exception.valid)
        check(!renamed.decoded.uses_rs1 && !renamed.decoded.uses_rs2,
              "fault token has no register dependencies");
    end
  endtask

  task automatic update_reference;
    integer candidate;
    integer r;
    logic fire;
    begin
      candidate = lowest_free();
      fire = dispatch_valid && rst_n && !recover && resources_ready &&
             (!destination_needed() || candidate != 0);
      if (completion_valid && completion.writes_rd && !completion.exception.valid &&
          completion.pdst != 0 && !ref_free[completion.pdst])
        ref_ready[completion.pdst] = 1;
      if (retire_valid && retire_uop.decoded.writes_rd && retire_uop.decoded.rd != 0 &&
          !retire_uop.decoded.fetch.exception.valid) begin
        check(retire_uop.old_pdst == ref_commit[retire_uop.decoded.rd],
              "test supplies valid in-order retirement history");
        ref_commit[retire_uop.decoded.rd] = retire_uop.pdst;
        if (retire_uop.old_pdst != 0) begin
          ref_free[retire_uop.old_pdst] = 1;
          ref_ready[retire_uop.old_pdst] = 0;
        end
      end
      if (fire && destination_needed()) begin
        ref_spec[decoded.rd] = candidate;
        ref_free[candidate] = 0;
        ref_ready[candidate] = 0;
      end
      if (recover) begin
        for (r = 0; r < PHYS_REG_COUNT; r = r + 1) begin
          ref_free[r] = 1;
          ref_ready[r] = 0;
        end
        for (r = 0; r < ARCH_REG_COUNT; r = r + 1) begin
          ref_spec[r] = ref_commit[r];
          ref_free[ref_commit[r]] = 0;
          ref_ready[ref_commit[r]] = 1;
        end
      end
      ref_spec[0] = 0;
      ref_commit[0] = 0;
      ref_free[0] = 0;
      ref_ready[0] = 1;
    end
  endtask

  task automatic check_state;
    integer r;
    begin
      check(free_count == number_free(), "free count matches available identities");
      for (r = 0; r < PHYS_REG_COUNT; r = r + 1) begin
        check(free_mask[r] == ref_free[r], "free-list bit matches reference ownership");
        check(ready_mask[r] == ref_ready[r], "readiness bit matches reference completion state");
      end
      // Inputs below only query maps: no clock edge occurs during this sweep.
      decoded = '0;
      decoded.uses_rs1 = 1;
      decoded.uses_rs2 = 1;
      for (r = 0; r < ARCH_REG_COUNT; r = r + 1) begin
        committed_query = r;
        decoded.rs1 = r;
        decoded.rs2 = ARCH_REG_COUNT-1-r;
        #1;
        check(committed_result == ref_commit[r], "committed map changes only on retirement");
        check(renamed.prs1 == ref_spec[r] &&
              renamed.prs2 == ref_spec[ARCH_REG_COUNT-1-r], "speculative map query");
      end
      check(!free_mask[0] && ready_mask[0], "p0 is permanently reserved and ready");
    end
  endtask

  task automatic cycle;
    logic recover_this_edge;
    begin
      #1;
      check_comb;
      if (dispatch_valid && dispatch_ready) last_dispatch = renamed;
      recover_this_edge = recover;
      update_reference;
      #2;
      clk = 1;
      #1;
      check(recover_done == recover_this_edge, "recovery acknowledgement follows apply edge");
      neutral;
      clk = 0;
      #1;
      check_state;
    end
  endtask

  task automatic reset_dut;
    begin
      clk = 0;
      neutral;
      rst_n = 0;
      reset_reference;
      #2;
      check(!dispatch_ready && !recover_done, "reset blocks dispatch and clears acknowledgement");
      rst_n = 1;
      #1;
      check_state;
    end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block04/block04.vcd");
      $dumpvars(0, tb_block04);
    end
    #1000000;
    $fatal(1, "Block 4 timed out");
  end

  initial begin
    reset_dut;
    check(free_count == 32, "reset leaves p32 through p63 free");

    // First lesson: two x5 writers get different versions and retain their old mappings.
    neutral; writer(5, 1, 2); cycle; first_writer = last_dispatch;
    check(first_writer.pdst == 32 && first_writer.old_pdst == 5, "first x5 version is p32");
    neutral; writer(5, 3, 4); cycle; second_writer = last_dispatch;
    check(second_writer.pdst == 33 && second_writer.old_pdst == 32,
          "second x5 version is p33, replacing speculative p32");
    neutral; decoded.rs1 = 5; decoded.rs2 = 5; dispatch_valid = 1; #1;
    check(renamed.prs1 == 0 && renamed.prs2 == 0 && src1_ready && src2_ready,
          "unused source fields do not depend on the pending x5 version"); cycle;
    neutral; writer(7, 5, 5); #1;
    check(renamed.prs1 == 33 && renamed.prs2 == 33 && !src1_ready && !src2_ready,
          "younger readers depend on newest x5 version");
    complete_uop(second_writer); #1;
    check(src1_ready && src2_ready && !ready_mask[33],
          "accepted completion wakes both sources before readiness is registered");
    cycle;
    check(ready_mask[33], "completion readiness persists after its valid signal falls");
    complete_uop(first_writer); cycle;
    retire_valid = 1; retire_uop = first_writer; cycle;
    check(ref_commit[5] == 32 && ref_spec[5] == 33,
          "older x5 commit does not overwrite the newer speculative mapping");
    retire_valid = 1; retire_uop = second_writer; cycle;
    check(ref_commit[5] == 33 && free_mask[32], "second retirement releases prior x5 version");

    neutral; writer(7, 7, 7); #1;
    check(renamed.prs1 == ref_spec[7] && renamed.prs2 == ref_spec[7] &&
          renamed.pdst != renamed.prs1, "self read/write captures the old version");
    cycle;
    neutral; writer(9, 5, 7); resources_ready = 0; before_free = number_free(); cycle;
    check(free_count == before_free, "blocked downstream resources prevent all allocation");
    neutral; writer(9, 5, 7); dispatch_valid = 0; cycle;
    check(free_count == before_free, "valid-low candidates do not consume identities");
    neutral; writer(0, 1, 2); cycle;
    check(free_count == before_free, "rd=x0 consumes no physical destination");

    neutral; completion_valid = 1; completion.writes_rd = 1; completion.pdst = 63; cycle;
    check(!ready_mask[63], "completion to a free identity is ignored");
    neutral; complete_uop(last_dispatch); completion.pdst = 34;
    completion.exception.valid = 1; cycle;
    check(!ready_mask[34], "fault completion does not publish readiness");
    neutral; completion_valid = 1; completion.pdst = 34; completion.writes_rd = 0; cycle;
    check(!ready_mask[34], "non-register completion does not publish destination readiness");
    neutral; completion_valid = 1; completion.writes_rd = 1; completion.pdst = 0; cycle;
    check(ready_mask[0] && !free_mask[0], "p0 completion cannot change reservation/readiness");
    neutral; retire_valid = 1; retire_uop.decoded.writes_rd = 1; retire_uop.decoded.rd = 0;
    retire_uop.pdst = 33; retire_uop.old_pdst = 0; before_free = number_free(); cycle;
    check(free_count == before_free && ref_commit[0] == 0,
          "x0 retirement cannot alter committed mappings or free p0");

    // Simultaneous accepted completion, older retirement, and a new destination allocation.
    reset_dut;
    neutral; writer(5, 1, 2); cycle; first_writer = last_dispatch;
    complete_uop(first_writer); cycle;
    neutral; writer(6, 1, 2); cycle; second_writer = last_dispatch;
    neutral; writer(7, 6, 5); complete_uop(second_writer);
    retire_valid = 1; retire_uop = first_writer; cycle;
    check(last_dispatch.pdst == 34 && last_dispatch.prs1 == 33 && free_mask[5],
          "current free pool allocates p34 while retirement releases p5 for next cycle");
    neutral; writer(7, 7, 6); cycle;
    check(last_dispatch.pdst == 5 && last_dispatch.prs1 == 34,
          "released p5 is reused next cycle without changing captured sources");

    // Unit stress exceeds the future 16-entry ROB to exercise a genuinely empty free list.
    // The integrated 64-PRF/16-ROB core cannot naturally reach this physical exhaustion case.
    reset_dut;
    for (n = 0; n < 32; n = n + 1) begin
      neutral; writer(5, 5, 0); cycle;
      if (n == 0) first_writer = last_dispatch;
    end
    check(free_count == 0, "allocator unit can represent an exhausted physical pool");
    neutral; writer(6, 1, 2); #1;
    check(!dispatch_ready, "writer stalls when no destination is free"); cycle;
    neutral; decoded.op_class = UOP_BRANCH; decoded.uses_rs1 = 1; decoded.rs1 = 5;
    dispatch_valid = 1; #1;
    check(dispatch_ready && renamed.pdst == 0, "nonwriter can dispatch with no free destinations");
    cycle;
    neutral; writer(6, 1, 2); decoded.fetch.exception.valid = 1;
    decoded.op_class = UOP_NONE; decoded.fetch.exception.cause = CAUSE_ILLEGAL_INSTR;
    #1;
    check(dispatch_ready && renamed.pdst == 0, "valid fault token needs no destination"); cycle;
    complete_uop(first_writer); cycle;
    neutral; writer(6, 1, 2); retire_valid = 1; retire_uop = first_writer;
    #1; check(!dispatch_ready, "retirement release is not a same-cycle allocator bypass"); cycle;
    check(free_count == 1 && free_mask[5], "retirement release makes one tag available");
    neutral; writer(6, 1, 2); cycle;
    check(last_dispatch.pdst == 5 && free_count == 0, "following cycle consumes released tag");
    recover = 1; cycle;
    check(free_count == 32 && ref_spec[5] == 32 && ref_spec[6] == 6,
          "safe recovery retains committed versions and reclaims speculative versions");

    // A retiring jump's link mapping must survive recovery applied on the same edge.
    reset_dut;
    neutral; writer(1, 0, 0); decoded.op_class = UOP_JUMP;
    decoded.uses_rs1 = 0; decoded.uses_rs2 = 0; cycle; first_writer = last_dispatch;
    complete_uop(first_writer); cycle;
    neutral; writer(1, 3, 4); cycle; second_writer = last_dispatch;
    neutral; writer(2, 1, 1); retire_valid = 1; retire_uop = first_writer; recover = 1;
    #1; check(!dispatch_ready, "recovery blocks a simultaneous younger dispatch"); cycle;
    check(ref_spec[1] == 32 && ref_commit[1] == 32 && free_mask[33] && !free_mask[32],
          "recovery includes same-edge link retirement and cancels younger version");
    neutral; cycle; check(!recover_done, "recovery acknowledgement clears after pulse");

    // Deterministic mixed transactions use a small software ROB solely to supply legal retires.
    reset_dut;
    queue_head = 0; queue_tail = 0; queue_count = 0;
    for (n = 0; n < 768; n = n + 1) begin
      neutral;
      rng = next_random(rng);
      if (queue_count != 0 && rng[0]) begin
        chosen = (queue_head + ((rng >> 4) % queue_count)) % 64;
        saved_uop = queue[chosen];
        complete_uop(saved_uop);
        if (rng[3:1] == 0) completion.exception.valid = 1;
      end
      if (queue_count != 0 && n % 3 != 0) begin
        saved_uop = queue[queue_head];
        if (ref_ready[saved_uop.pdst]) begin retire_valid = 1; retire_uop = saved_uop; end
      end
      rng = next_random(rng);
      if (queue_count < ROB_DEPTH && rng[2:0] != 0) begin
        writer(1 + (rng[9:5] % 31), rng[14:10], rng[19:15]);
        resources_ready = n % 11 != 0;
      end
      if (n % 97 == 96) recover = 1;
      #1;
      dispatched = dispatch_valid && dispatch_ready;
      retired = retire_valid;
      recovered = recover;
      cycle;
      if (retired) begin queue_head = (queue_head + 1) % 64; queue_count = queue_count - 1; end
      if (dispatched) begin
        queue[queue_tail] = last_dispatch;
        queue_tail = (queue_tail + 1) % 64;
        queue_count = queue_count + 1;
      end
      if (recovered) begin queue_head = 0; queue_tail = 0; queue_count = 0; end
    end

    // Asynchronous reset also wins when requests are held active.
    neutral; writer(5, 1, 2); recover = 1; #1; rst_n = 0; reset_reference; #2;
    check(!dispatch_ready && !recover_done, "reset dominates held dispatch and recovery");
    neutral; rst_n = 1; #1; check_state;

    $display("BLOCK04 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Block 4 rename/readiness tests failed");
    $finish;
  end
endmodule
