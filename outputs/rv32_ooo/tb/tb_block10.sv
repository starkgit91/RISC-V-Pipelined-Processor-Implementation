// BTB/PHT verification using full-PC reference tags and integer saturating counters.
`timescale 1ns / 1ps

module tb_block10;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1, enable, invalidate, invalidate_done, protocol_error;
  word_t query_pc0, query_pc1;
  prediction_pair_t prediction;
  logic [1:0] retire_valid, counter0, counter1;
  rob_entry_pair_t retirement;
  logic ref_valid [0:63];
  logic ref_conditional [0:63];
  word_t ref_pc [0:63], ref_target [0:63];
  integer ref_counter [0:127];
  logic ref_ack;
  word_t rng = 32'h10c0ffee;
  integer checks = 0, failures = 0, sequence_id = 0;
  integer i, n, state_value, outcomes, kind, bad, retire_mask;
  rob_entry_t first, second, malformed;
  word_t pc, other_pc, target;
  logic waves;

  rv32_branch_predictor dut (
    .clk_i(clk), .rst_ni(rst_n), .enable_i(enable),
    .query_pc0_i(query_pc0), .query_pc1_i(query_pc1), .prediction_o(prediction),
    .retire_valid_i(retire_valid), .retire_entry_i(retirement),
    .invalidate_i(invalidate), .invalidate_done_o(invalidate_done),
    .query0_counter_o(counter0), .query1_counter_o(counter1), .protocol_error_o(protocol_error)
  );

  function automatic word_t next_random(input word_t value);
    next_random = {value[30:0], value[31] ^ value[21] ^ value[1] ^ value[0]};
  endfunction

  function automatic integer btb_slot(input word_t address);
    btb_slot = (address / 4) % 64;
  endfunction

  function automatic integer pht_slot(input word_t address);
    pht_slot = (address / 4) % 128;
  endfunction

  function automatic rob_entry_t branch_record(
    input word_t address, destination, input logic taken
  );
    rob_entry_t value;
    begin
      value = '0;
      value.valid = 1;
      value.complete = 1;
      value.uop.decoded.op_class = UOP_BRANCH;
      value.uop.decoded.branch_op = BRANCH_NE;
      value.uop.decoded.fetch.pc = address;
      value.uop.decoded.fetch.instruction = 32'hfe009ce3;
      value.uop.decoded.fetch.prediction.next_pc = address + 4;
      value.uop.decoded.fetch.prediction.pht_index = pht_slot(address);
      value.uop.decoded.immediate = destination - address;
      value.control_flow.is_control = 1;
      value.control_flow.is_conditional = 1;
      value.control_flow.taken = taken;
      value.control_flow.mispredicted = taken;
      value.control_flow.next_pc = taken ? destination : address + 4;
      branch_record = value;
    end
  endfunction

  function automatic rob_entry_t jump_record(
    input word_t address, destination, input logic indirect
  );
    rob_entry_t value;
    begin
      value = branch_record(address, destination, 1'b1);
      value.uop.decoded.op_class = UOP_JUMP;
      value.uop.decoded.jump_indirect = indirect;
      value.uop.decoded.writes_rd = 1;
      value.uop.decoded.rd = 1;
      value.uop.pdst = 32;
      value.control_flow.is_conditional = 0;
      jump_record = value;
    end
  endfunction

  function automatic rob_entry_t ordinary_record(input word_t address);
    rob_entry_t value;
    begin
      value = '0;
      value.valid = 1;
      value.complete = 1;
      value.uop.decoded.op_class = UOP_INTEGER;
      value.uop.decoded.alu_op = ALU_ADD;
      value.uop.decoded.fetch.pc = address;
      value.uop.decoded.fetch.instruction = NOP_INSTRUCTION;
      value.uop.decoded.fetch.prediction.pht_index = pht_slot(address);
      value.control_flow.next_pc = address + 4;
      ordinary_record = value;
    end
  endfunction

  // This is the retirement interface contract, not a second instruction decoder.
  function automatic logic record_ok(input rob_entry_t value);
    word_t address, branch_target, expected_pc;
    logic usable;
    begin
      address = value.uop.decoded.fetch.pc;
      branch_target = address + value.uop.decoded.immediate;
      usable = value.valid && value.complete && !value.exception.valid && address % 4 == 0
               && value.uop.decoded.op_class != UOP_NONE;
      if (value.uop.decoded.op_class == UOP_BRANCH) begin
        expected_pc = value.control_flow.taken ? branch_target : address + 4;
        usable = usable && value.control_flow.is_control && value.control_flow.is_conditional
                 && value.uop.decoded.fetch.prediction.pht_index == pht_slot(address)
                 && value.control_flow.next_pc == expected_pc
                 && (!value.control_flow.taken || branch_target % 4 == 0);
      end else if (value.uop.decoded.op_class == UOP_JUMP) begin
        usable = usable && value.control_flow.is_control && !value.control_flow.is_conditional
                 && value.control_flow.taken && value.control_flow.next_pc % 4 == 0;
        if (!value.uop.decoded.jump_indirect)
          usable = usable && value.control_flow.next_pc == branch_target;
      end
      record_ok = usable;
    end
  endfunction

  function automatic logic expected_error;
    begin
      expected_error = 0;
      if (rst_n && !invalidate) begin
        if (retire_valid == 2'b10) expected_error = 1;
        if (retire_valid[0] && !record_ok(retirement.lane0)) expected_error = 1;
        if (retire_valid[1] && !record_ok(retirement.lane1)) expected_error = 1;
      end
    end
  endfunction

  function automatic prediction_t predicted(input word_t address);
    prediction_t result;
    integer slot, counter_index;
    begin
      result = '0;
      result.next_pc = address + 4;
      counter_index = pht_slot(address);
      result.pht_index = counter_index;
      slot = btb_slot(address);
      if (rst_n && enable && !invalidate && address % 4 == 0
          && ref_valid[slot] && ref_pc[slot] == address) begin
        result.btb_hit = 1;
        result.taken = !ref_conditional[slot] || ref_counter[counter_index] >= 2;
        if (result.taken) result.next_pc = ref_target[slot];
      end
      predicted = result;
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

  task automatic evaluate;
    prediction_pair_t expected;
    begin
      expected.lane0 = predicted(query_pc0);
      expected.lane1 = predicted(query_pc1);
      check(prediction == expected, "both asynchronous lookups match independent full-PC model");
      check(counter0 == (rst_n ? ref_counter[pht_slot(query_pc0)] : 1)
            && counter1 == (rst_n ? ref_counter[pht_slot(query_pc1)] : 1),
            "debug counter values show actual registered state even with lookup disabled");
      check(invalidate_done == ref_ack, "invalidation acknowledgement follows its clock edge");
      check(protocol_error == expected_error(), "retirement validity diagnostics match contract");
    end
  endtask

  task automatic clear_reference;
    begin
      for (integer index = 0; index < 64; index++) begin
        ref_valid[index] = 0;
        ref_conditional[index] = 0;
        ref_pc[index] = 0;
        ref_target[index] = 0;
      end
      for (integer index = 0; index < 128; index++) ref_counter[index] = 1;
    end
  endtask

  // Full PC equality and bounded integer arithmetic differ from the RTL's tag slices/counter logic.
  task automatic train_reference(input rob_entry_t value);
    integer slot, counter_index, adjusted;
    word_t address, destination;
    begin
      address = value.uop.decoded.fetch.pc;
      slot = btb_slot(address);
      if (value.uop.decoded.op_class == UOP_BRANCH) begin
        counter_index = value.uop.decoded.fetch.prediction.pht_index;
        adjusted = ref_counter[counter_index] + (value.control_flow.taken ? 1 : -1);
        if (adjusted < 0) adjusted = 0;
        if (adjusted > 3) adjusted = 3;
        ref_counter[counter_index] = adjusted;
        destination = address + value.uop.decoded.immediate;
        if (destination % 4 == 0) begin
          ref_valid[slot] = 1;
          ref_pc[slot] = address;
          ref_target[slot] = destination;
          ref_conditional[slot] = 1;
        end else if (ref_valid[slot] && ref_pc[slot] == address) ref_valid[slot] = 0;
      end else if (value.uop.decoded.op_class == UOP_JUMP) begin
        ref_valid[slot] = 1;
        ref_pc[slot] = address;
        ref_target[slot] = value.control_flow.next_pc;
        ref_conditional[slot] = 0;
      end else if (ref_valid[slot] && ref_pc[slot] == address) ref_valid[slot] = 0;
    end
  endtask

  task automatic cycle;
    begin
      #1; evaluate;
      #1; clk = 1;
      ref_ack = invalidate;
      if (invalidate) clear_reference;
      else if (retire_valid != 2'b10 && retire_valid[0] && record_ok(retirement.lane0)) begin
        train_reference(retirement.lane0);
        if (retire_valid[1] && record_ok(retirement.lane1)) train_reference(retirement.lane1);
      end
      #1; clk = 0; #1; evaluate;
    end
  endtask

  task automatic query(input word_t address0, address1);
    begin query_pc0 = address0; query_pc1 = address1; #1; evaluate; end
  endtask

  task automatic retire_pair(
    input rob_entry_t older, younger, input logic [1:0] mask
  );
    begin
      retirement.lane0 = older; retirement.lane1 = younger; retire_valid = mask;
      cycle;
      retire_valid = 0; retirement = '0; #1; evaluate;
    end
  endtask

  task automatic retire_one(input rob_entry_t value);
    begin retire_pair(value, '0, 2'b01); end
  endtask

  task automatic invalidate_tables;
    begin
      invalidate = 1; cycle;
      invalidate = 0; retire_valid = 0; retirement = '0; #1; evaluate;
    end
  endtask

  task automatic reset_dut;
    begin
      clk = 0; enable = 1; invalidate = 0; retire_valid = 0; retirement = '0;
      query_pc0 = 0; query_pc1 = 4; ref_ack = 0;
      rst_n = 0; clear_reference; #2; evaluate;
      rst_n = 1; #1; evaluate;
    end
  endtask

  task automatic sweep_tables;
    begin
      for (integer index = 0; index < 64; index++)
        query(ref_pc[index], ref_pc[index] ^ 32'h100);
      for (integer index = 0; index < 128; index++)
        query(32'h8000 + index * 4, 32'h10000 + ((index + 47) % 128) * 4);
    end
  endtask

  initial begin #3000000; $fatal(1, "Branch predictor test timed out"); end
  initial begin
    waves = $test$plusargs("waves");
    if (waves) begin
      $dumpfile("build/block10/block10.vcd");
      $dumpvars(0, tb_block10);
    end
    reset_dut;
    check(BTB_ENTRIES == 64 && PHT_ENTRIES == 128,
          "approved BTB and PHT dimensions remain fixed");

    // Leading example: the loop target must remain static even after its not-taken exit.
    pc = 32'h40; target = 32'h20; query(pc, pc + 4);
    check(counter0 == 1 && !prediction.lane0.btb_hit && !prediction.lane0.taken,
          "cold loop branch has weak not-taken state and no cached target");
    retire_one(branch_record(pc, target, 1));
    check(counter0 == 2 && prediction.lane0.taken && prediction.lane0.next_pc == target,
          "first retired taken outcome installs target and weak taken state");
    retire_one(branch_record(pc, target, 1));
    check(counter0 == 3, "second taken outcome strengthens the loop prediction");
    retire_one(branch_record(pc, target, 1));
    check(counter0 == 3, "taken saturation cannot wrap back to not-taken");
    retire_one(branch_record(pc, target, 0));
    check(counter0 == 2 && prediction.lane0.taken && prediction.lane0.next_pc == target,
          "loop exit weakens direction but BTB retains the loop target instead of fall-through");
    retire_one(branch_record(pc, target, 0));
    check(counter0 == 1 && prediction.lane0.btb_hit && !prediction.lane0.taken
          && prediction.lane0.next_pc == pc + 4,
          "second not-taken outcome crosses the direction threshold but retains a BTB hit");
    retire_one(branch_record(pc, target, 0)); retire_one(branch_record(pc, target, 0));
    check(counter0 == 0, "not-taken saturation cannot wrap to taken");

    // Both training records may touch one counter. Saturation makes ordering observable.
    for (state_value = 0; state_value < 4; state_value++) begin
      for (outcomes = 0; outcomes < 4; outcomes++) begin
        invalidate_tables; pc = 32'h80; other_pc = pc + 32'h200; query(pc, other_pc);
        if (state_value == 0) retire_one(branch_record(pc, 32'h60, 0));
        if (state_value >= 2) retire_one(branch_record(pc, 32'h60, 1));
        if (state_value == 3) retire_one(branch_record(pc, 32'h60, 1));
        check(counter0 == state_value, "collision test starts from each counter state");
        first = branch_record(pc, 32'h60, outcomes[1]);
        second = branch_record(other_pc, 32'h300, outcomes[0]);
        retire_pair(first, second, 3);
        check(!prediction.lane0.btb_hit && prediction.lane1.btb_hit,
              "younger control owns an aliased BTB slot after ordered pair training");
      end
    end

    // Exercise every PHT location and both lookup ports, including each BTB replacement index.
    if (waves) $dumpoff;
    invalidate_tables;
    for (i = 0; i < 128; i++) begin
      pc = 32'h1000 + i * 4; query(pc, pc);
      retire_one(branch_record(pc, pc + 32'h100, 1));
      retire_one(branch_record(pc, pc + 32'h100, 1));
      retire_one(branch_record(pc, pc + 32'h100, 0));
      retire_one(branch_record(pc, pc + 32'h100, 0));
      retire_one(branch_record(pc, pc + 32'h100, 0));
      check(counter0 == 0 && counter1 == 0, "each PHT counter reaches both saturation boundaries");
    end
    sweep_tables;

    // A BTB collision need not be a PHT collision: PC+256 changes the seventh PHT index bit.
    invalidate_tables; pc = 32'h44; other_pc = pc + 32'h100; query(pc, other_pc);
    retire_pair(branch_record(pc, 32'h100, 1), branch_record(other_pc, 32'h200, 1), 3);
    check(counter0 == 2 && counter1 == 2 && !prediction.lane0.btb_hit
          && prediction.lane1.next_pc == 32'h200,
          "independent PHT counters coexist while the younger BTB alias replaces the older tag");

    // Ordinary instructions evict only their own stale tag, never another PC's alias.
    retire_one(ordinary_record(pc));
    check(prediction.lane1.btb_hit, "ordinary alias does not evict a different cached branch PC");
    retire_one(ordinary_record(other_pc));
    check(!prediction.lane1.btb_hit && counter1 == 2,
          "ordinary instruction invalidates its own stale BTB entry without training PHT");
    retire_pair(branch_record(pc, 32'h100, 1), ordinary_record(other_pc), 3);
    check(prediction.lane0.btb_hit,
          "younger ordinary alias preserves an older branch installation");
    retire_pair(branch_record(pc, 32'h100, 0), ordinary_record(pc), 3);
    check(!prediction.lane0.btb_hit,
          "younger same-PC ordinary record clears the older branch entry");

    // Unconditional jumps use cached targets even when the direction counter predicts not-taken.
    pc = 32'h90; query(pc, pc + 4);
    retire_one(branch_record(pc, 32'h80, 0));
    retire_one(jump_record(pc, 32'h400, 0));
    check(counter0 == 0 && prediction.lane0.taken && prediction.lane0.next_pc == 32'h400,
          "JAL prediction is unconditional and does not train its PHT counter");
    retire_one(jump_record(pc, 32'hc00, 1));
    check(counter0 == 0 && prediction.lane0.next_pc == 32'hc00,
          "JALR learns its latest resolved target without changing direction state");
    first = ordinary_record(pc); first.uop.decoded.op_class = UOP_SYSTEM;
    first.uop.decoded.system_op = SYS_MRET; first.control_flow.is_control = 1;
    first.control_flow.next_pc = 32'h8000;
    retire_one(first);
    check(!prediction.lane0.btb_hit && counter0 == 0,
          "system return metadata does not turn SYSTEM into a BTB jump");

    // Lookup disable changes prediction only; accepted retirement still trains the tables.
    enable = 0; pc = 32'ha0; query(pc, pc + 4);
    retire_one(branch_record(pc, 32'h700, 1));
    check(counter0 == 2 && !prediction.lane0.btb_hit && prediction.lane0.next_pc == pc + 4,
          "disabled lookup stays sequential while retired outcomes continue to train");
    enable = 1; #1; evaluate;
    check(prediction.lane0.taken && prediction.lane0.next_pc == 32'h700,
          "reenabling lookup exposes retained trained state");

    // Misaligned query PCs never match a cached aligned instruction.
    for (i = 1; i < 4; i++) begin
      query(pc + i, pc);
      check(!prediction.lane0.btb_hit && !prediction.lane0.taken,
            "misaligned lookup forces sequential prediction with no BTB hit");
    end
    pc = 32'hfffffffc; query(pc, pc + 4);
    retire_one(branch_record(pc, 0, 1));
    check(prediction.lane0.next_pc == 0 && prediction.lane0.btb_hit,
          "PC-relative arithmetic wraps at the architectural 32-bit boundary");

    // A legal not-taken branch can have an unusable taken target.
    pc = 32'h200; query(pc, pc + 32'h100);
    retire_one(branch_record(pc, pc + 8, 1));
    retire_one(branch_record(pc, pc + 2, 0));
    check(counter0 == 1 && !prediction.lane0.btb_hit,
          "non-taken misaligned target trains direction but clears only its own unusable target");
    retire_one(jump_record(pc + 32'h100, 32'h800, 0));
    retire_one(branch_record(pc, pc + 2, 0));
    check(prediction.lane1.btb_hit && prediction.lane1.next_pc == 32'h800,
          "uninstallable branch target does not evict a different PC's valid alias");

    // Invalid retirement records cannot pollute either table; older valid training still applies.
    for (bad = 0; bad < 13; bad++) begin
      first = branch_record(32'h300, 32'h280, 1);
      malformed = branch_record(32'h500, 32'h480, 1);
      case (bad)
        0: malformed.valid = 0;
        1: malformed.complete = 0;
        2: malformed.exception.valid = 1;
        3: malformed.uop.decoded.fetch.pc = 32'h502;
        4: malformed.uop.decoded.op_class = UOP_NONE;
        5: malformed.control_flow.is_control = 0;
        6: malformed.control_flow.is_conditional = 0;
        7: malformed.uop.decoded.fetch.prediction.pht_index = 0;
        8: malformed.control_flow.next_pc = 32'h484;
        9: begin
          malformed.uop.decoded.immediate = 2;
          malformed.control_flow.next_pc = 32'h502;
        end
        10: begin malformed = jump_record(32'h500, 32'h480, 0);
                  malformed.control_flow.taken = 0; end
        11: begin malformed = jump_record(32'h500, 32'h482, 1); end
        12: begin malformed = jump_record(32'h500, 32'h480, 0);
                  malformed.control_flow.next_pc = 32'h484; end
      endcase
      query(32'h300, 32'h500);
      retire_pair(malformed, first, 3);
      retire_pair(first, malformed, 3);
      retire_pair('0, first, 2'b10);
      sweep_tables;
    end

    // Invalidation masks lookups immediately and overrides even malformed retirement sidebands.
    if (waves) $dumpon;
    invalidate_tables;
    for (i = 0; i < 64; i++)
      retire_one(jump_record(32'h4000 + i * 4, 32'h8000 + i * 8, 1));
    query(32'h4000, 32'h40fc);
    check(prediction.lane0.btb_hit && prediction.lane1.btb_hit,
          "invalidation fixture fills all cached targets");
    retirement.lane0 = '0; retirement.lane1 = '0; retire_valid = 2'b10; invalidate = 1;
    #1; evaluate;
    check(!prediction.lane0.btb_hit && !protocol_error,
          "invalidation bypass masks predictions before the edge and ignores bad retirement");
    cycle; invalidate = 0; retire_valid = 0; retirement = '0;
    for (i = 0; i < 64; i++) query(32'h4000 + i * 4, 32'h4000 + ((i+19) % 64) * 4);
    sweep_tables;
    check(invalidate_done, "invalidation acknowledgement remains registered until the next edge");
    cycle; check(!invalidate_done, "acknowledgement drops after invalidation deasserts");

    // Random retirement groups preserve program-order folding while both lookup PCs move freely.
    if (waves) $dumpoff;
    for (n = 0; n < 2500; n++) begin
      rng = next_random(rng); pc = {16'h1234, rng[15:2], 2'b0};
      rng = next_random(rng); other_pc = n % 3 == 0 ? pc + 32'h200 : (rng & 32'h0000fffc);
      rng = next_random(rng); target = rng & 32'hffff_fffc;
      kind = rng[2:0];
      if (kind < 4) first = branch_record(pc, target, rng[3]);
      else if (kind < 6) first = jump_record(pc, target, kind[0]);
      else first = ordinary_record(pc);
      rng = next_random(rng);
      if (rng[1:0] < 2) second = branch_record(other_pc, target ^ 32'h400, rng[3]);
      else if (rng[1:0] == 2) second = jump_record(other_pc, target, 1);
      else second = ordinary_record(other_pc);
      if (n % 17 == 0) second.exception.valid = 1;
      if (n % 23 == 0) first.complete = 0;
      enable = n % 13 != 0; invalidate = n % 101 == 100;
      query_pc0 = n % 4 == 0 ? pc + 1 : pc;
      query_pc1 = n % 5 == 0 ? other_pc ^ 32'h100 : other_pc;
      case (rng[5:4])
        0: retire_mask = 0;
        1: retire_mask = 1;
        2: retire_mask = 3;
        default: retire_mask = n % 7 == 0 ? 2 : 3;
      endcase
      retire_pair(first, second, retire_mask);
      if (n % 97 == 0) sweep_tables;
    end
    invalidate = 0; enable = 1; retire_valid = 0; retirement = '0; sweep_tables;

    // Reset must dominate occupied prediction state and outstanding training inputs.
    if (waves) $dumpon;
    retire_one(branch_record(32'hc0, 32'h80, 1));
    for (i = 0; i < 64; i++)
      retire_one(jump_record(32'h6000 + i * 4, 32'ha000 + i * 8, 1));
    query(32'h6000, 32'h60fc);
    retirement.lane0 = branch_record(32'hc0, 32'h40, 1); retire_valid = 1;
    rst_n = 0; ref_ack = 0; clear_reference; #2; evaluate;
    check(!prediction.lane0.btb_hit && counter0 == 1 && !invalidate_done,
          "asynchronous reset discards learned state and pending training");
    rst_n = 1; retire_valid = 0; retirement = '0; #1; evaluate;
    for (i = 0; i < 64; i++) query(32'h6000 + i * 4, 32'h6000 + ((i+13) % 64) * 4);
    sweep_tables;
    $display("BLOCK10 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Branch predictor unit checks failed");
    $finish;
  end
endmodule
