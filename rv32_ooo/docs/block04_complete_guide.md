# Block 4, two-issue revision: rename a pair without losing program order

This is the active two-wide rename block. The previous single-instruction lesson, sources,
and reports remain in `reference/single_issue`.

## 1. Two writers of x5 in the same dispatch group

```asm
div x5, x1, x2       # Lane 0: older
add x5, x3, x4       # Lane 1: younger
```

Initially x5 maps to p5 and p32..p63 are free. Lane 0 allocates p32 and records old_pdst=p5.
Lane 1 observes that update, allocates p33, and records old_pdst=p32. The speculative map ends
at p33. The committed map stays p5 until retirement.

If both retire together in program order, committed x5 first becomes p32 and then p33 in the
same edge. The final mapping is p33; p5 and p32 are released. The retirement trace will still
record both instructions in order. This example concerns naming/state; the divider is not
implemented in this block.

## 2. Real and false dependencies inside the pair

```asm
add x5, x1, x2
add x6, x5, x3
```

Lane 1 must capture lane 0's new physical destination as its x5 source, with readiness false.
It cannot read the old x5 value, or become ready merely because that physical location contains
old bits. This is a true RAW dependency and still delays execution.

For two writers of x5, separate destinations remove WAW. For an older reader followed by a
writer, lane 0 retains its old physical source while lane 1 gets a different destination,
removing WAR. An instruction reading and writing its own architectural name captures sources
before replacing its destination mapping.

## 3. Circuit and data flow

```text
                current speculative map / current free list
                               |
                   lane 0 source and destination lookup
                               |
                    accepted lane 0 mapping bypass
                               |
                   lane 1 source and destination lookup
                               |
                   next speculative map / allocated tags

 two accepted completions -> readiness lookup and registered ready bits
 two ordered retirements  -> committed map, then old-destination releases
 safe recovery APPLY      -> copy final committed map and rebuild availability
```

Both map tables contain 32 six-bit physical identities. The 64-bit free and readiness masks
remain separate from the physical values stored in block 3. Two priority selections allocate
from current free entries, removing lane 0's accepted destination before choosing lane 1's.

## 4. Pair interface and acceptance

The decoded, renamed, completion, retirement, and ROB-tag connections use named lane0/lane1
packed pairs. For dispatch and retirement, lane 0 is older. Dispatch validity and successful
retirement use prefix masks: 00 means none, 01 means lane 0, 11 means both.

| Signal group | Width/type | Source → destination | Meaning |
|---|---|---|---|
| clk_i / rst_ni | 1 each | Common clock/reset → maps and masks | Rising-edge updates; asynchronous active-low reset |
| decoded_i | 528; decoded_pair_t | Two decoders → rename | Two operation records with architectural source/destination fields |
| dispatch_rob_tag_i | 24; two 12-bit tags | ROB allocator → rename | Each tag contains a four-bit slot and eight-bit generation |
| dispatch_valid_i | 2 | Fetch/dispatch holding queue → rename | Instructions offered in program order |
| dispatch_resources_ready_i | 2 | Global dispatch control → rename | Per-lane ROB/issue/memory capacity agreement |
| dispatch_ready_o / dispatch_fire_o | 2 each | Rename → atomic dispatcher/fetch | Available capacity and the accepted ordered prefix |
| renamed_o | 600; renamed_pair_t | Rename → ROB/IQ/LSU | Source tags and distinct new/old destination tags; each physical tag is six bits |
| src1_ready_o / src2_ready_o | 2 each | Rename ready lookup → IQ | Readiness of each lane's actual physical sources, including accepted-completion bypass |
| completion_valid_i / completion_i | 2 / 444; completion_pair_t | Validated writeback → readiness table | Up to two accepted results; mask 10 is legal here |
| retire_valid_i / retire_uop_i | 2 / 600; renamed_pair_t | ROB retirement → committed maps/free list | Successful retirements, oldest first; release replaced physical mappings |
| recover_i | 1 | Recovery APPLY → rename | Restore speculative ownership after cancellation/drain |
| recover_done_o | 1 | Rename → recovery controller | Registered acknowledgment of the restoration edge |
| free_mask_o / ready_mask_o | 64 each; phys_mask_t | Current rename state → status/control | One ownership/readiness bit per physical register |
| free_count_o | 7; phys_count_t | Free-mask count → status/control | Number of available physical identities, range 0–64 |
| committed_query_arch_i | 5; arch_reg_t | Debug/test query → committed map | Select architectural x0 through x31 |
| committed_query_phys_o | 6; phys_reg_t | Committed map → debug/test | Physical mapping for the selected committed register |
| protocol_error_o | 1 | Rename diagnostics → integration checks | Invalid dispatch/retirement prefix or duplicate ready-producing write |

Aggregate widths are packed wire widths in the fixed configuration, not a count of RAM blocks.
The paired record shapes are defined once in the shared package, so every consumer sees the
same source tags, rename history, and ROB ownership metadata.

Lane 1 transfers only if lane 0 transfers. If there is space for only the older instruction,
fire=01. The front end must retain the younger suffix and compact it into lane 0 for the next
cycle; it must not resend the already accepted older instruction.

The older ready signal advertises capacity independently of valid. An invalid 10 offer still
cannot transfer because valid0=0 prevents both fire0 and lane1 readiness. This distinction
avoids a combinational ready/valid loop when the global dispatcher joins several resources.

The candidate renamed records are combinational previews, not independent buffered channels.
Capture a lane only on its fire bit. Candidates may change while stalled as resources change.

## 5. Allocation, completion, and retirement ordering

Unused sources and x0 select p0, which is always ready and never free. A valid fault token
requires a ROB entry but no physical destination or source wait. The decoder's original PC,
prediction, and exception information are retained.

Readiness lookup includes both accepted live non-faulting completions. A destination newly
allocated by lane 0 is explicitly unready when lane 1 looks it up. A duplicate physical
completion is diagnosed and lane 0 has priority; the upstream arbiter must prevent it.

Only accepted validated completions can publish readiness. This module does not independently
check ROB generations, so the future ROB/completion controller must reject stale owners before
asserting completion_valid. A ready bit is not an architectural commit.

Retirement applies lane 0 and then lane 1. The old physical identity for each writer must match
its replaced committed mapping at that point in the prefix. The ROB/controller supplies legal
in-order retirement records. Ordinary retirement must not overwrite the speculative map, which
may already name younger values.

Allocation uses the current free pool rather than retirement releases from the same edge.
Newly released tags can be reused next cycle. With only one free tag, the older writer can
transfer while the younger writer waits. A nonwriter or fault can transfer without a free tag.
The testbench may exhaust the allocator alone; the final 16-entry ROB prevents that 32-writer
exhaustion scenario during normal whole-core operation.

## 6. Recovery

recover_i is a safe APPLY pulse after execution cancellation and result draining. It blocks
new dispatch, applies any simultaneous successful retirement prefix, then restores speculative
mappings from the resulting committed map. Free and ready masks are rebuilt from those mappings.
This preserves a retiring branch's link version, including ordered same-register retirements.

Physical register values are not cleared. Cancellation/ownership validation and safe reuse
are implemented in later controller blocks; a generation counter alone is not the barrier.

## 7. Tests and simulator use

```sh
python3 scripts/run_checks.py --block 4 --waves
```

Tests cover intra-pair RAW/WAW/WAR, self-read/write, partial acceptance and compaction,
simultaneous completion/retirement/allocation, readiness, exhaustion at allocator-unit level,
recovery, protocol diagnostics, and reset. Check the report for exact count and source hashes.
The controlled clock pulses allow independent state queries; they do not measure FPGA timing.

The optional waveform is `build/block04/block04.vcd`. Observe fire masks, both physical
destinations, lane 1's bypassed physical source, source-ready bits, and committed x5. The first
pair must not turn a RAW-dependent lane 1 instruction into an independent operation.

For Vivado XSim after sourcing the Linux installation's settings:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_rename.sv tb/tb_block04.sv
xelab tb_block04 -s block04_sim
xsim block04_sim --runall
```

The actual Vivado fit, timing, and bitstream remain unverified. Icarus may emit its known
always_comb whole-vector sensitivity notices; complete messages are retained in the log.

## 8. Interview questions

**Why is lane 0's mapping visible to lane 1 before the edge?** Program order still determines
which version each instruction reads, even when two instructions rename together.

**Why can't the two instructions simply rename independently?** That would miss within-pair
RAW and WAW relationships and could allocate the same physical destination twice.

**Can lane 1 transfer while lane 0 stalls?** No. Dispatch accepts a program-order prefix.
A leftover younger instruction is compacted into the older position next cycle.

**Why can completion mask10 be valid while retirement mask10 is invalid?** Results may arrive
independently; architectural retirement may not skip an older instruction in the same prefix.

**Do two writes to x5 need only one physical destination?** No. They represent different
versions. Both must remain distinct until ordering permits the old versions to be reclaimed.

**Does dual retirement change the final architectural meaning?** No. It applies two sequential
architectural updates in one edge, with lane 1 observing lane 0's committed update.

The complete active RTL and testbench follow. Historical single-issue code is retained only
as a reference and is not compiled by the active test runner.

## Complete rename, free-list, and readiness module

Source: `rtl/rv32_rename.sv`.

```systemverilog
// Two-wide ID rename: lane 0 is older; lane 1 observes lane 0's accepted mapping update.
// Completion ownership and valid in-order retirement are supplied by the ROB/controller.
`timescale 1ns / 1ps

module rv32_rename (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  input  logic [1:0]                   dispatch_valid_i,
  input  logic [1:0]                   dispatch_resources_ready_i,
  input  rv32_ooo_pkg::decoded_pair_t   decoded_i,
  input  rv32_ooo_pkg::rob_tag_pair_t   dispatch_rob_tag_i,
  output logic [1:0]                   dispatch_ready_o,
  output logic [1:0]                   dispatch_fire_o,
  output rv32_ooo_pkg::renamed_pair_t   renamed_o,
  output logic [1:0]                   src1_ready_o,
  output logic [1:0]                   src2_ready_o,
  input  logic [1:0]                   completion_valid_i,
  input  rv32_ooo_pkg::completion_pair_t completion_i,
  input  logic [1:0]                   retire_valid_i,
  input  rv32_ooo_pkg::renamed_pair_t   retire_uop_i,
  input  logic                         recover_i,
  output logic                         recover_done_o,
  input  rv32_ooo_pkg::arch_reg_t       committed_query_arch_i,
  output rv32_ooo_pkg::phys_reg_t       committed_query_phys_o,
  output rv32_ooo_pkg::phys_mask_t      free_mask_o,
  output rv32_ooo_pkg::phys_mask_t      ready_mask_o,
  output rv32_ooo_pkg::phys_count_t     free_count_o,
  output logic                         protocol_error_o
);
  import rv32_ooo_pkg::*;

  phys_reg_t speculative_q [0:ARCH_REG_COUNT-1];
  phys_reg_t speculative_d [0:ARCH_REG_COUNT-1];
  phys_reg_t after_lane0 [0:ARCH_REG_COUNT-1];
  phys_reg_t committed_q [0:ARCH_REG_COUNT-1];
  phys_reg_t committed_d [0:ARCH_REG_COUNT-1];
  phys_mask_t free_q, free_d, free_after0;
  phys_mask_t ready_q, ready_d, ready_lookup0, ready_lookup1;
  phys_reg_t allocate0, allocate1;
  decoded_uop_t dec0, dec1;
  completion_t wb0, wb1;
  renamed_uop_t retiring0, retiring1, renamed0, renamed1;
  logic need0, need1, complete0, complete1, completion_conflict;
  logic retire_write0, retire_write1;
  integer map_index;
  integer state_index;
  integer reset_index;
  integer count_index;
  integer free_count;

  assign dec0 = decoded_i.lane0;
  assign dec1 = decoded_i.lane1;
  assign wb0 = completion_i.lane0;
  assign wb1 = completion_i.lane1;
  assign retiring0 = retire_uop_i.lane0;
  assign retiring1 = retire_uop_i.lane1;
  assign renamed_o.lane0 = renamed0;
  assign renamed_o.lane1 = renamed1;
  assign free_mask_o = free_q;
  assign ready_mask_o = ready_q;
  assign committed_query_phys_o = committed_q[committed_query_arch_i];

  function automatic phys_reg_t first_free(input phys_mask_t mask);
    integer candidate;
    begin
      first_free = '0;
      for (candidate = PHYS_REG_COUNT-1; candidate >= 1; candidate = candidate - 1)
        if (mask[candidate]) first_free = candidate[PHYS_REG_W-1:0];
    end
  endfunction

  function automatic logic needs_destination(input decoded_uop_t operation);
    needs_destination = operation.writes_rd && operation.rd != 0 &&
                        !operation.fetch.exception.valid;
  endfunction

  always_comb begin
    need0 = needs_destination(dec0);
    need1 = needs_destination(dec1);
    complete0 = completion_valid_i[0] && wb0.writes_rd && !wb0.exception.valid &&
                wb0.pdst != 0 && !free_q[wb0.pdst];
    complete1 = completion_valid_i[1] && wb1.writes_rd && !wb1.exception.valid &&
                wb1.pdst != 0 && !free_q[wb1.pdst];
    completion_conflict = complete0 && complete1 && wb0.pdst == wb1.pdst;
    // A duplicate physical write violates the upstream contract; lane 0 has priority.
    if (completion_conflict) complete1 = 1'b0;
    protocol_error_o = rst_ni && ((dispatch_valid_i == 2'b10) ||
                       (retire_valid_i == 2'b10) || completion_conflict);
    ready_lookup0 = ready_q;
    if (complete0) ready_lookup0[wb0.pdst] = 1'b1;
    if (complete1) ready_lookup0[wb1.pdst] = 1'b1;

    allocate0 = first_free(free_q);
    // Ready advertises capacity independently of valid; invalid 10 still cannot fire lane 1.
    dispatch_ready_o[0] = rst_ni && !recover_i && dispatch_resources_ready_i[0] &&
                          (!need0 || allocate0 != 0);
    dispatch_fire_o[0] = dispatch_valid_i[0] && dispatch_ready_o[0];

    renamed0 = '0;
    renamed0.decoded = dec0;
    renamed0.decoded.writes_rd = need0;
    renamed0.rob_tag = dispatch_rob_tag_i.lane0;
    if (dec0.fetch.exception.valid) begin
      renamed0.decoded.uses_rs1 = 1'b0;
      renamed0.decoded.uses_rs2 = 1'b0;
    end else begin
      if (dec0.uses_rs1) renamed0.prs1 = speculative_q[dec0.rs1];
      if (dec0.uses_rs2) renamed0.prs2 = speculative_q[dec0.rs2];
    end
    if (need0) begin
      renamed0.pdst = allocate0;
      renamed0.old_pdst = speculative_q[dec0.rd];
    end
    src1_ready_o[0] = renamed0.prs1 == 0 || ready_lookup0[renamed0.prs1];
    src2_ready_o[0] = renamed0.prs2 == 0 || ready_lookup0[renamed0.prs2];

    for (map_index = 0; map_index < ARCH_REG_COUNT; map_index = map_index + 1)
      after_lane0[map_index] = speculative_q[map_index];
    free_after0 = free_q;
    ready_lookup1 = ready_lookup0;
    if (dispatch_fire_o[0] && need0) begin
      after_lane0[dec0.rd] = allocate0;
      free_after0[allocate0] = 1'b0;
      ready_lookup1[allocate0] = 1'b0;
    end

    allocate1 = first_free(free_after0);
    // Lane 1 can transfer only with lane 0; the source must compact a stalled suffix next cycle.
    dispatch_ready_o[1] = dispatch_fire_o[0] && dispatch_resources_ready_i[1] &&
                          (!need1 || allocate1 != 0);
    dispatch_fire_o[1] = dispatch_valid_i[1] && dispatch_ready_o[1];
    renamed1 = '0;
    renamed1.decoded = dec1;
    renamed1.decoded.writes_rd = need1;
    renamed1.rob_tag = dispatch_rob_tag_i.lane1;
    if (dec1.fetch.exception.valid) begin
      renamed1.decoded.uses_rs1 = 1'b0;
      renamed1.decoded.uses_rs2 = 1'b0;
    end else begin
      if (dec1.uses_rs1) renamed1.prs1 = after_lane0[dec1.rs1];
      if (dec1.uses_rs2) renamed1.prs2 = after_lane0[dec1.rs2];
    end
    if (need1) begin
      renamed1.pdst = allocate1;
      renamed1.old_pdst = after_lane0[dec1.rd];
    end
    src1_ready_o[1] = renamed1.prs1 == 0 || ready_lookup1[renamed1.prs1];
    src2_ready_o[1] = renamed1.prs2 == 0 || ready_lookup1[renamed1.prs2];

    retire_write0 = retire_valid_i[0] && needs_destination(retiring0.decoded);
    retire_write1 = (&retire_valid_i) && needs_destination(retiring1.decoded);
    free_count = 0;
    for (count_index = 1; count_index < PHYS_REG_COUNT; count_index = count_index + 1)
      if (free_q[count_index]) free_count = free_count + 1;
    free_count_o = free_count[$clog2(PHYS_REG_COUNT+1)-1:0];
  end

  always_comb begin
    for (state_index = 0; state_index < ARCH_REG_COUNT; state_index = state_index + 1) begin
      speculative_d[state_index] = after_lane0[state_index];
      committed_d[state_index] = committed_q[state_index];
    end
    free_d = free_q;
    ready_d = ready_lookup0;
    // Retirement is an ordered prefix. The second writer sees the first committed update.
    if (retire_write0) begin
      committed_d[retiring0.decoded.rd] = retiring0.pdst;
      if (retiring0.old_pdst != 0) begin
        free_d[retiring0.old_pdst] = 1'b1;
        ready_d[retiring0.old_pdst] = 1'b0;
      end
    end
    if (retire_write1) begin
      committed_d[retiring1.decoded.rd] = retiring1.pdst;
      if (retiring1.old_pdst != 0) begin
        free_d[retiring1.old_pdst] = 1'b1;
        ready_d[retiring1.old_pdst] = 1'b0;
      end
    end
    if (dispatch_fire_o[0] && need0) begin
      free_d[allocate0] = 1'b0;
      ready_d[allocate0] = 1'b0;
    end
    if (dispatch_fire_o[1] && need1) begin
      speculative_d[dec1.rd] = allocate1;
      free_d[allocate1] = 1'b0;
      ready_d[allocate1] = 1'b0;
    end
    if (recover_i) begin
      // APPLY only after cancelled execution and completion queues acknowledge quiescence.
      free_d = '1;
      ready_d = '0;
      for (state_index = 0; state_index < ARCH_REG_COUNT; state_index = state_index + 1) begin
        speculative_d[state_index] = committed_d[state_index];
        free_d[committed_d[state_index]] = 1'b0;
        ready_d[committed_d[state_index]] = 1'b1;
      end
    end
    speculative_d[0] = '0;
    committed_d[0] = '0;
    free_d[0] = 1'b0;
    ready_d[0] = 1'b1;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (reset_index = 0; reset_index < ARCH_REG_COUNT; reset_index = reset_index + 1) begin
        speculative_q[reset_index] <= reset_index[PHYS_REG_W-1:0];
        committed_q[reset_index] <= reset_index[PHYS_REG_W-1:0];
      end
      for (reset_index = 0; reset_index < PHYS_REG_COUNT; reset_index = reset_index + 1) begin
        free_q[reset_index] <= reset_index >= ARCH_REG_COUNT;
        ready_q[reset_index] <= reset_index < ARCH_REG_COUNT;
      end
      recover_done_o <= 1'b0;
    end else begin
      for (reset_index = 0; reset_index < ARCH_REG_COUNT; reset_index = reset_index + 1) begin
        speculative_q[reset_index] <= speculative_d[reset_index];
        committed_q[reset_index] <= committed_d[reset_index];
      end
      free_q <= free_d;
      ready_q <= ready_d;
      recover_done_o <= recover_i;
    end
  end
endmodule
```

## Complete self-checking testbench

Source: `tb/tb_block04.sv`.

```systemverilog
// Two-wide rename tests. The driver supplies retirement/completion events.
// The software queue is a test stimulus source, not a hardware ROB.
`timescale 1ns / 1ps

module tb_block04;
  import rv32_ooo_pkg::*;

  logic clk = 0;
  logic rst_n = 1;
  logic [1:0] dispatch_valid, resources_ready, dispatch_ready, dispatch_fire;
  decoded_pair_t decoded;
  rob_tag_pair_t dispatch_tag;
  renamed_pair_t renamed, retire_uop, captured;
  logic [1:0] src1_ready, src2_ready, completion_valid, retire_valid;
  completion_pair_t completion;
  logic recover, recover_done, protocol_error;
  arch_reg_t committed_query;
  phys_reg_t committed_result;
  phys_mask_t free_mask, ready_mask;
  phys_count_t free_count;

  integer ref_spec [0:ARCH_REG_COUNT-1];
  integer ref_commit [0:ARCH_REG_COUNT-1];
  logic ref_free [0:PHYS_REG_COUNT-1];
  logic ref_ready [0:PHYS_REG_COUNT-1];
  integer preview_map [0:ARCH_REG_COUNT-1];
  logic preview_free [0:PHYS_REG_COUNT-1];
  logic preview_ready [0:PHYS_REG_COUNT-1];
  renamed_pair_t expected_pair;
  logic [1:0] expected_ready, expected_fire, expected_src1, expected_src2, captured_fire;
  logic expected_error;
  renamed_uop_t a_uop, b_uop, c_uop, d_uop, saved, temp_uop;
  renamed_uop_t queue [0:127];
  integer queue_head, queue_tail, queue_count;
  integer checks = 0;
  integer failures = 0;
  integer serial = 0;
  integer n, i, j, index, chosen, completed_first, retired_count;
  word_t rng = 32'h53c09a71;
  logic was_recovered;
  decoded_uop_t suffix;

  rv32_rename dut (
    .clk_i(clk), .rst_ni(rst_n),
    .dispatch_valid_i(dispatch_valid), .dispatch_resources_ready_i(resources_ready),
    .decoded_i(decoded), .dispatch_rob_tag_i(dispatch_tag),
    .dispatch_ready_o(dispatch_ready), .dispatch_fire_o(dispatch_fire), .renamed_o(renamed),
    .src1_ready_o(src1_ready), .src2_ready_o(src2_ready),
    .completion_valid_i(completion_valid), .completion_i(completion),
    .retire_valid_i(retire_valid), .retire_uop_i(retire_uop),
    .recover_i(recover), .recover_done_o(recover_done),
    .committed_query_arch_i(committed_query), .committed_query_phys_o(committed_result),
    .free_mask_o(free_mask), .ready_mask_o(ready_mask), .free_count_o(free_count),
    .protocol_error_o(protocol_error)
  );

  function automatic word_t next_random(input word_t state);
    next_random = {state[30:0], state[31] ^ state[21] ^ state[1] ^ state[0]};
  endfunction

  function automatic decoded_uop_t make_writer(input integer rd, rs1, rs2, pc);
    decoded_uop_t value;
    begin
      value = '0;
      value.op_class = UOP_INTEGER;
      value.alu_op = ALU_ADD;
      value.writes_rd = 1;
      value.uses_rs1 = 1;
      value.uses_rs2 = 1;
      value.rd = rd;
      value.rs1 = rs1;
      value.rs2 = rs2;
      value.fetch.pc = pc;
      value.fetch.prediction.next_pc = pc + 4;
      make_writer = value;
    end
  endfunction

  function automatic logic needs_dest(input decoded_uop_t operation);
    needs_dest = operation.writes_rd && operation.rd != 0 && !operation.fetch.exception.valid;
  endfunction

  function automatic integer available_count;
    integer r;
    begin
      available_count = 0;
      for (r = 1; r < PHYS_REG_COUNT; r = r + 1)
        if (ref_free[r]) available_count = available_count + 1;
    end
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 24)
          $display("FAIL: %s time=%0t free=%0d", label_text, $time, free_count);
      end
    end
  endtask

  task automatic neutral;
    begin
      dispatch_valid = 0;
      resources_ready = 2'b11;
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

  task automatic set_lane(input integer lane, input decoded_uop_t operation);
    rob_tag_t tag;
    begin
      tag.index = serial % ROB_DEPTH;
      tag.generation = serial / ROB_DEPTH;
      serial = serial + 1;
      if (lane == 0) begin decoded.lane0 = operation; dispatch_tag.lane0 = tag; end
      else begin decoded.lane1 = operation; dispatch_tag.lane1 = tag; end
      dispatch_valid[lane] = 1;
    end
  endtask

  task automatic complete_lane(input integer lane, input renamed_uop_t operation);
    completion_t result;
    begin
      result = '0;
      result.rob_tag = operation.rob_tag;
      result.pdst = operation.pdst;
      result.writes_rd = operation.decoded.writes_rd;
      result.control_flow.next_pc = operation.decoded.fetch.pc + 4;
      if (lane == 0) completion.lane0 = result;
      else completion.lane1 = result;
      completion_valid[lane] = 1;
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

  // Construct a sequential two-instruction preview using integer maps and boolean tables.
  task automatic predict;
    integer r, lane, candidate;
    decoded_uop_t operation;
    renamed_uop_t result;
    completion_t wb0, wb1;
    logic eligible0, eligible1, collision, need, can_transfer;
    begin
      wb0 = completion.lane0;
      wb1 = completion.lane1;
      eligible0 = completion_valid[0] && wb0.writes_rd && !wb0.exception.valid &&
                  wb0.pdst != 0 && !ref_free[wb0.pdst];
      eligible1 = completion_valid[1] && wb1.writes_rd && !wb1.exception.valid &&
                  wb1.pdst != 0 && !ref_free[wb1.pdst];
      collision = eligible0 && eligible1 && wb0.pdst == wb1.pdst;
      if (collision) eligible1 = 0;
      expected_error = rst_n && (dispatch_valid == 2'b10 || retire_valid == 2'b10 || collision);
      for (r = 0; r < ARCH_REG_COUNT; r = r + 1) preview_map[r] = ref_spec[r];
      for (r = 0; r < PHYS_REG_COUNT; r = r + 1) begin
        preview_free[r] = ref_free[r];
        preview_ready[r] = ref_ready[r];
      end
      if (eligible0) preview_ready[wb0.pdst] = 1;
      if (eligible1) preview_ready[wb1.pdst] = 1;
      expected_pair = '0;
      expected_ready = 0;
      expected_fire = 0;
      for (lane = 0; lane < 2; lane = lane + 1) begin
        operation = lane == 0 ? decoded.lane0 : decoded.lane1;
        need = needs_dest(operation);
        candidate = 0;
        // Reverse priority scan is independent of the RTL's packaged mask helper.
        for (r = PHYS_REG_COUNT-1; r >= 1; r = r - 1)
          if (preview_free[r]) candidate = r;
        can_transfer = lane == 0 ?
          (rst_n && !recover && resources_ready[0]) :
          (expected_fire[0] && resources_ready[1]);
        expected_ready[lane] = can_transfer && (!need || candidate != 0);
        expected_fire[lane] = dispatch_valid[lane] && expected_ready[lane];
        result = '0;
        result.decoded = operation;
        result.decoded.writes_rd = need;
        result.rob_tag = lane == 0 ? dispatch_tag.lane0 : dispatch_tag.lane1;
        if (operation.fetch.exception.valid) begin
          result.decoded.uses_rs1 = 0;
          result.decoded.uses_rs2 = 0;
        end else begin
          if (operation.uses_rs1) result.prs1 = preview_map[operation.rs1];
          if (operation.uses_rs2) result.prs2 = preview_map[operation.rs2];
        end
        if (need) begin
          result.pdst = candidate;
          result.old_pdst = preview_map[operation.rd];
        end
        expected_src1[lane] = result.prs1 == 0 || preview_ready[result.prs1];
        expected_src2[lane] = result.prs2 == 0 || preview_ready[result.prs2];
        if (lane == 0) expected_pair.lane0 = result;
        else expected_pair.lane1 = result;
        if (expected_fire[lane] && need) begin
          preview_map[operation.rd] = candidate;
          preview_free[candidate] = 0;
          preview_ready[candidate] = 0;
        end
      end
    end
  endtask

  task automatic check_comb;
    begin
      predict;
      check(dispatch_ready == expected_ready && dispatch_fire == expected_fire,
            "dispatch transfers an available ordered prefix");
      check(renamed == expected_pair,
            "both renamed records preserve controls and correct mappings");
      check(src1_ready == expected_src1 && src2_ready == expected_src2,
            "both source pairs use completion and intra-pair readiness");
      check(protocol_error == expected_error, "protocol diagnostics match the supplied events");
    end
  endtask

  task automatic update_reference;
    integer r, lane;
    renamed_uop_t value;
    completion_t wb0, wb1;
    logic good0, good1;
    begin
      wb0 = completion.lane0;
      wb1 = completion.lane1;
      good0 = completion_valid[0] && wb0.writes_rd && !wb0.exception.valid &&
              wb0.pdst != 0 && !ref_free[wb0.pdst];
      good1 = completion_valid[1] && wb1.writes_rd && !wb1.exception.valid &&
              wb1.pdst != 0 && !ref_free[wb1.pdst];
      if (good0 && good1 && wb0.pdst == wb1.pdst) good1 = 0;
      if (good0) ref_ready[wb0.pdst] = 1;
      if (good1) ref_ready[wb1.pdst] = 1;
      for (lane = 0; lane < 2; lane = lane + 1) begin
        value = lane == 0 ? retire_uop.lane0 : retire_uop.lane1;
        if (retire_valid[lane] && (lane == 0 || retire_valid[0]) &&
            needs_dest(value.decoded)) begin
          check(value.old_pdst == ref_commit[value.decoded.rd], "ordered retirement history");
          check(ref_ready[value.pdst], "driver retires only a ready physical version");
          ref_commit[value.decoded.rd] = value.pdst;
          if (value.old_pdst != 0) begin
            ref_free[value.old_pdst] = 1;
            ref_ready[value.old_pdst] = 0;
          end
        end
      end
      for (lane = 0; lane < 2; lane = lane + 1) begin
        value = lane == 0 ? expected_pair.lane0 : expected_pair.lane1;
        if (expected_fire[lane] && value.decoded.writes_rd) begin
          ref_spec[value.decoded.rd] = value.pdst;
          ref_free[value.pdst] = 0;
          ref_ready[value.pdst] = 0;
        end
      end
      if (recover) begin
        for (r = 0; r < PHYS_REG_COUNT; r = r + 1) begin ref_free[r] = 1; ref_ready[r] = 0; end
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
      check(free_count == available_count(), "free count matches reference ownership");
      for (r = 0; r < PHYS_REG_COUNT; r = r + 1) begin
        check(free_mask[r] == ref_free[r], "free-list bit");
        check(ready_mask[r] == ref_ready[r], "ready-table bit");
      end
      decoded = '0;
      decoded.lane0.uses_rs1 = 1;
      decoded.lane0.uses_rs2 = 1;
      decoded.lane1.uses_rs1 = 1;
      decoded.lane1.uses_rs2 = 1;
      for (r = 0; r < ARCH_REG_COUNT; r = r + 1) begin
        committed_query = r;
        decoded.lane0.rs1 = r;
        decoded.lane0.rs2 = ARCH_REG_COUNT-1-r;
        decoded.lane1.rs1 = (r+5) % ARCH_REG_COUNT;
        decoded.lane1.rs2 = (r+11) % ARCH_REG_COUNT;
        #1;
        check(committed_result == ref_commit[r], "committed-map query");
        check(renamed.lane0.prs1 == ref_spec[r] &&
              renamed.lane0.prs2 == ref_spec[ARCH_REG_COUNT-1-r], "lane-0 map queries");
        check(renamed.lane1.prs1 == ref_spec[(r+5) % ARCH_REG_COUNT] &&
              renamed.lane1.prs2 == ref_spec[(r+11) % ARCH_REG_COUNT], "lane-1 map queries");
        check(!free_mask[ref_spec[r]] && !free_mask[ref_commit[r]],
              "mapped versions stay reserved");
      end
      check(!free_mask[0] && ready_mask[0], "p0 is reserved and ready");
    end
  endtask

  task automatic cycle;
    logic recovering;
    begin
      #1;
      check_comb;
      captured = renamed;
      captured_fire = dispatch_fire;
      recovering = recover;
      update_reference;
      #2; clk = 1; #1;
      check(recover_done == recovering, "recovery acknowledgement follows the apply edge");
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
      check(!dispatch_fire && !recover_done && !protocol_error, "reset dominates control events");
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
    $fatal(1, "Two-wide block 4 timed out");
  end

  initial begin
    reset_dut;
    check(free_count == 32, "initial spare physical versions");

    // WAW and RAW in one accepted pair: the younger x5 writer reads the new older version.
    neutral;
    set_lane(0, make_writer(5, 1, 2, 0));
    set_lane(1, make_writer(5, 5, 4, 4));
    #1;
    check(renamed.lane0.pdst == 32 && renamed.lane1.pdst == 33 &&
          renamed.lane1.old_pdst == 32, "same-pair x5 writers receive distinct linked versions");
    check(renamed.lane1.prs1 == 32 && !src1_ready[1], "pair RAW waits for lane-0 producer");
    cycle; a_uop = captured.lane0; b_uop = captured.lane1;
    complete_lane(0, a_uop); complete_lane(1, b_uop); cycle;
    retire_valid = 2'b11; retire_uop.lane0 = a_uop; retire_uop.lane1 = b_uop; cycle;
    check(ref_commit[5] == 33 && free_mask[5] && free_mask[32] && !free_mask[33],
          "two ordered x5 retirements leave only the youngest mapping committed");

    // WAR retains an older reader's physical source before a younger writer changes the name.
    neutral;
    set_lane(0, make_writer(6, 5, 1, 8));
    set_lane(1, make_writer(5, 3, 4, 12));
    #1;
    check(renamed.lane0.prs1 == 33 && renamed.lane1.pdst != 33,
          "older reader retains its source across a younger x5 rename"); cycle;
    neutral;
    set_lane(0, make_writer(7, 7, 7, 16));
    set_lane(1, make_writer(7, 7, 7, 20));
    #1;
    check(renamed.lane0.prs1 == 7 && renamed.lane0.prs2 == 7 &&
          renamed.lane1.prs1 == renamed.lane0.pdst && !src1_ready[1],
          "self-read uses old mapping and lane 1 sees the accepted replacement"); cycle;

    // Partial pair acceptance requires compaction of the younger suffix next cycle.
    reset_dut;
    neutral; set_lane(0, make_writer(5, 1, 2, 24));
    suffix = make_writer(6, 5, 3, 28); set_lane(1, suffix); resources_ready = 2'b01;
    #1; check(dispatch_fire == 2'b01, "only the available prefix is accepted"); cycle;
    a_uop = captured.lane0;
    neutral; set_lane(0, suffix); #1;
    check(renamed.lane0.prs1 == a_uop.pdst && !src1_ready[0],
          "compacted suffix retains the true dependency"); cycle;
    b_uop = captured.lane0;
    neutral; set_lane(0, make_writer(8, 1, 2, 32));
    set_lane(1, make_writer(9, 3, 4, 36)); resources_ready = 2'b10;
    #1; check(dispatch_fire == 0, "younger slot cannot bypass unavailable older resources"); cycle;
    neutral; set_lane(1, make_writer(8, 1, 2, 40));
    #1; check(protocol_error && dispatch_fire == 0, "10 dispatch mask is rejected"); cycle;
    neutral; retire_valid = 2'b10; retire_uop.lane1 = b_uop;
    #1; check(protocol_error, "10 retirement mask is diagnosed and ignored"); cycle;

    // Both accepted completion lanes can wake either source of either new instruction.
    reset_dut;
    neutral; set_lane(0, make_writer(5, 1, 2, 44)); set_lane(1, make_writer(6, 3, 4, 48));
    cycle; a_uop = captured.lane0; b_uop = captured.lane1;
    neutral;
    decoded.lane0 = make_writer(0, 5, 6, 52);
    decoded.lane1 = make_writer(0, 6, 5, 56);
    complete_lane(0, a_uop); complete_lane(1, b_uop); #1;
    check(src1_ready == 2'b11 && src2_ready == 2'b11 &&
          !ready_mask[a_uop.pdst] && !ready_mask[b_uop.pdst], "dual readiness bypass before edge");
    cycle;
    neutral; set_lane(0, make_writer(7, 1, 2, 60)); cycle; c_uop = captured.lane0;
    complete_lane(1, c_uop); #1;
    check(!protocol_error, "completion lane 1 alone is legal"); cycle;
    neutral; set_lane(0, make_writer(8, 1, 2, 64)); cycle; d_uop = captured.lane0;
    complete_lane(0, d_uop); completion.lane0.exception.valid = 1; cycle;
    check(!ready_mask[d_uop.pdst], "fault completion does not publish readiness");
    complete_lane(0, d_uop); completion.lane0.writes_rd = 0; cycle;
    check(!ready_mask[d_uop.pdst], "non-register completion is ignored");
    neutral; completion_valid = 2'b11; completion.lane0.writes_rd = 1;
    completion.lane1.writes_rd = 1; completion.lane0.pdst = 63; completion.lane1.pdst = 0;
    cycle; check(!ready_mask[63] && ready_mask[0], "free and p0 result updates are ignored");
    complete_lane(0, d_uop); complete_lane(1, d_uop); #1;
    check(protocol_error, "duplicate physical completion is diagnosed"); cycle;
    check(ready_mask[d_uop.pdst], "lane-0 duplicate priority still marks one version ready");

    // Concurrent two-result, two-retirement, and two-allocation update.
    reset_dut;
    neutral; set_lane(0, make_writer(5, 1, 2, 68)); set_lane(1, make_writer(6, 3, 4, 72));
    cycle; a_uop = captured.lane0; b_uop = captured.lane1;
    complete_lane(0, a_uop); complete_lane(1, b_uop); cycle;
    neutral; set_lane(0, make_writer(7, 1, 2, 76)); set_lane(1, make_writer(8, 3, 4, 80));
    cycle; c_uop = captured.lane0; d_uop = captured.lane1;
    neutral; set_lane(0, make_writer(9, 7, 8, 84)); set_lane(1, make_writer(10, 8, 7, 88));
    complete_lane(0, c_uop); complete_lane(1, d_uop);
    retire_valid = 2'b11; retire_uop.lane0 = a_uop; retire_uop.lane1 = b_uop;
    #1; check(renamed.lane0.pdst == 36 && renamed.lane1.pdst == 37 &&
              src1_ready == 2'b11 && src2_ready == 2'b11,
              "concurrent sources and current-free tags");
    cycle;
    neutral; set_lane(0, make_writer(11, 1, 2, 92)); set_lane(1, make_writer(12, 3, 4, 96));
    #1; check(renamed.lane0.pdst == 5 && renamed.lane1.pdst == 6,
              "two retirement releases become available on the following cycle"); cycle;

    // Exhaustion is unit-only: the future 16-entry ROB would backpressure sooner.
    reset_dut;
    for (n = 0; n < 31; n = n + 1) begin
      neutral; set_lane(0, make_writer(5, 5, 0, 100+n*4)); cycle;
      if (n == 0) a_uop = captured.lane0;
    end
    check(free_count == 1, "one destination remains in allocator stress");
    neutral; set_lane(0, make_writer(6, 1, 2, 228)); set_lane(1, make_writer(7, 3, 4, 232));
    #1; check(dispatch_fire == 2'b01 && renamed.lane0.pdst == 63,
              "one free identity accepts only the first writer"); cycle;
    neutral; set_lane(0, make_writer(7, 3, 4, 232)); #1;
    check(dispatch_fire == 0, "compacted writer waits for a free destination"); cycle;
    neutral; set_lane(0, make_writer(0, 1, 2, 236));
    decoded.lane1 = '0; decoded.lane1.op_class = UOP_BRANCH; dispatch_valid[1] = 1;
    #1; check(dispatch_fire == 2'b11, "two nonwriters can pass a full physical pool"); cycle;
    neutral; set_lane(0, make_writer(8, 1, 2, 240));
    decoded.lane0.fetch.exception.valid = 1;
    decoded.lane0.fetch.exception.cause = CAUSE_ILLEGAL_INSTR;
    decoded.lane0.op_class = UOP_NONE; #1;
    check(dispatch_fire == 2'b01 && renamed.lane0.pdst == 0 && src1_ready[0],
          "valid fault token needs neither a destination nor a source wait"); cycle;
    complete_lane(0, a_uop); cycle;
    neutral; set_lane(0, make_writer(8, 1, 2, 244));
    retire_valid = 2'b01; retire_uop.lane0 = a_uop;
    #1; check(dispatch_fire == 0, "same-edge release is not an allocator bypass"); cycle;
    neutral; set_lane(0, make_writer(8, 1, 2, 244));
    set_lane(1, make_writer(0, 3, 4, 248)); #1;
    check(dispatch_fire == 2'b11 && renamed.lane0.pdst == 5,
          "one writer and one nonwriter fit the newly released identity"); cycle;
    recover = 1; cycle; check(free_count == 32, "safe apply reclaims uncommitted versions");

    // Apply recovery after two simultaneous ordered x5 retirements, including a jump link.
    reset_dut;
    neutral; set_lane(0, make_writer(5, 1, 2, 252)); set_lane(1, make_writer(5, 3, 4, 256));
    decoded.lane1.op_class = UOP_JUMP;
    cycle; a_uop = captured.lane0; b_uop = captured.lane1;
    complete_lane(0, a_uop); complete_lane(1, b_uop); cycle;
    neutral; set_lane(0, make_writer(5, 5, 1, 260)); set_lane(1, make_writer(6, 5, 2, 264));
    cycle;
    neutral; set_lane(0, make_writer(9, 1, 2, 268)); set_lane(1, make_writer(10, 3, 4, 272));
    retire_valid = 2'b11; retire_uop.lane0 = a_uop; retire_uop.lane1 = b_uop;
    recover = 1; #1; check(dispatch_fire == 0, "recovery blocks both dispatch lanes"); cycle;
    check(ref_spec[5] == b_uop.pdst && ref_commit[5] == b_uop.pdst &&
          !free_mask[b_uop.pdst] && free_mask[a_uop.pdst],
          "recovery keeps final retiring link map");
    neutral; cycle; check(!recover_done, "recovery acknowledgement clears after the pulse");

    // Mixed legal traffic uses a small software queue only to drive in-order retirement.
    reset_dut;
    queue_head = 0; queue_tail = 0; queue_count = 0;
    for (n = 0; n < 512; n = n + 1) begin
      neutral;
      completed_first = -1;
      if (queue_count != 0) begin
        rng = next_random(rng);
        chosen = (queue_head + ((rng >> 3) % queue_count)) % 128;
        temp_uop = queue[chosen];
        if (!ref_ready[temp_uop.pdst]) begin
          complete_lane(0, temp_uop); completed_first = chosen;
        end
        for (j = 0; j < queue_count; j = j + 1) begin
          index = (chosen+j+1) % 128;
          // Only choose an entry inside the live circular queue.
          if (index >= queue_head) i = index - queue_head;
          else i = index + 128 - queue_head;
          if (i < queue_count && index != completed_first && !completion_valid[1]) begin
            temp_uop = queue[index];
            if (!ref_ready[temp_uop.pdst]) complete_lane(1, temp_uop);
          end
        end
      end
      if (queue_count != 0 && n % 4 != 0) begin
        temp_uop = queue[queue_head];
        if (ref_ready[temp_uop.pdst]) begin
          retire_valid[0] = 1; retire_uop.lane0 = temp_uop;
          if (queue_count > 1 && n % 3 != 0) begin
            temp_uop = queue[(queue_head+1) % 128];
            if (ref_ready[temp_uop.pdst]) begin
              retire_valid[1] = 1; retire_uop.lane1 = temp_uop;
            end
          end
        end
      end
      rng = next_random(rng);
      if (queue_count < ROB_DEPTH && rng[0]) begin
        set_lane(0, make_writer(1+(rng[9:5] % 31), rng[14:10], rng[19:15], 280+n*8));
        if (queue_count < ROB_DEPTH-1 && rng[1]) begin
          rng = next_random(rng);
          set_lane(1, make_writer(1+(rng[9:5] % 31), rng[14:10], rng[19:15], 284+n*8));
          if (n % 4 == 0) decoded.lane1.rs1 = decoded.lane0.rd;
          if (n % 8 == 0) decoded.lane1.rd = decoded.lane0.rd;
        end
        resources_ready = n % 11 == 0 ? 2'b01 : 2'b11;
      end
      if (n % 79 == 78) recover = 1;
      retired_count = retire_valid[0] + retire_valid[1];
      was_recovered = recover;
      cycle;
      queue_head = (queue_head+retired_count) % 128;
      queue_count = queue_count-retired_count;
      if (captured_fire[0]) begin
        queue[queue_tail] = captured.lane0;
        queue_tail = (queue_tail+1) % 128; queue_count = queue_count+1;
      end
      if (captured_fire[1]) begin
        queue[queue_tail] = captured.lane1;
        queue_tail = (queue_tail+1) % 128; queue_count = queue_count+1;
      end
      if (was_recovered) begin queue_head = 0; queue_tail = 0; queue_count = 0; end
    end

    neutral; set_lane(0, make_writer(5, 1, 2, 5000));
    set_lane(1, make_writer(6, 3, 4, 5004)); recover = 1; #1;
    rst_n = 0; reset_reference; #2;
    check(dispatch_fire == 0 && !recover_done && !protocol_error, "reset wins held pair requests");
    neutral; rst_n = 1; #1; check_state;

    $display("BLOCK04 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Two-wide rename tests failed");
    $finish;
  end
endmodule
```
