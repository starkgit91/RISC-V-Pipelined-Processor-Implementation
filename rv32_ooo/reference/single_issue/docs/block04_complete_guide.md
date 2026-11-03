# Block 4: rename maps, the free list, and readiness

This block gives each register-writing instruction its own physical destination, while keeping
the architectural register names seen by the program. It implements speculative/committed maps,
physical allocation, readiness tracking, retirement updates, and the application of safe recovery.

The ROB controller, instruction scheduler, completion arbiter, and execution cancellation logic
are later blocks. Their validated events are modeled by this block's testbench.

## 1. Start with two writers of x5

Consider:

```asm
div x5, x1, x2
add x5, x3, x4
```

After reset, x5 maps to p5. Initially p32 through p63 are free. The allocator chooses the lowest
currently free physical identity, so the two writers receive p32 and p33 before either retires.

| Event | Speculative x5 map | Committed x5 map | New destination | Replaced physical identity |
|---|---|---|---|---|
| Reset | p5 | p5 | None | None |
| Dispatch DIV | p32 | p5 | p32 | p5 |
| Dispatch ADD | p33 | p5 | p33 | p32 |
| ADD completes first | p33 | p5 | No allocation | No release |
| DIV completes and retires | p33 | p32 | No allocation | p5 becomes free |
| ADD retires | p33 | p33 | No allocation | p32 becomes free |

Completion makes a value available to consumers, but does not change the committed map. The
younger ADD cannot overwrite DIV's p32 value, and retirement preserves the original instruction
order. This removes the WAW false dependency between their shared architectural destination.

Two writers alone demonstrate WAW, not WAR. To see WAR, insert an older reader before a writer:

```asm
add x6, x5, x1       # Captures the current source identity for x5.
add x5, x3, x4       # Gets a different destination; it cannot overwrite that source identity.
```

The older reader keeps its captured physical source even if the speculative x5 map changes.
The writer does not need to wait merely to protect the architectural name.

RAW is different: if a later instruction actually needs p33's value, it must wait until p33
is ready. Renaming preserves real dependencies; it does not remove them.

## 2. Hardware structures and where they connect

```text
                       ID: decoded register names + operation
                                      |
                                      v
                       speculative map: x-register -> p-register
                        |                                |
             capture source tags                  find old destination
                        |                                |
                        +------ free-list allocation ----+
                                      |
                           renamed instruction / ROB tag
                                      |
                         future ROB and issue queue

WB accepted completion ----------> readiness table
future ROB retirement -----------> committed map + old-destination release
safe recovery apply -------------> restore speculative map; rebuild free/readiness state

EX physical register file from block 3 stores the actual values separately.
```

Both maps have 32 entries of six bits each. The free list and readiness table each have one
bit per physical identity. Combinational selection reads mappings and finds a free identity;
clocked state updates apply allocations, completions, retirement, and recovery.

The physical register file is not copied when a mapping changes. The map chooses which stored
version a register name refers to. Shared package types `phys_mask_t` and `phys_count_t` describe
the mask and free-count connections.

## 3. Interface and ownership

| Port/group | Producer | Consumer | Meaning |
|---|---|---|---|
| `decoded_i` | Decoder | Rename | Architectural sources/destination, source-use flags, operation, and fault metadata |
| `dispatch_rob_tag_i` | Future ROB allocator | Renamed record | Identity reserved for this instruction when dispatch fires |
| `dispatch_valid_i` | ID/fetch holding register | Rename | There is a live instruction waiting to dispatch |
| `dispatch_resources_ready_i` | Future dispatch controller | Rename | All required ROB/issue/memory resources are available |
| `dispatch_ready_o` | Rename | Dispatch controller | This instruction can be accepted, including any physical destination |
| `renamed_o` | Rename | Future ROB/issue queue | Original operation, captured physical sources, new destination, and old destination |
| `src1_ready_o`, `src2_ready_o` | Readiness lookup/bypass | New issue entry | Whether the captured sources are available now |
| `completion_valid_i`, `completion_i` | Future completion arbiter | Readiness table | One accepted, live, ownership-validated result |
| `retire_valid_i`, `retire_uop_i` | Future ROB retirement | Committed map/free list | One successfully retiring instruction and its rename history |
| `recover_i` | Future recovery controller | Map/free/readiness state | Apply recovery after cancellation and result draining are complete |
| `recover_done_o` | Rename state register | Recovery controller | The recovery application has reached a clock edge |
| `committed_query_arch_i` / `committed_query_phys_o` | Debug/test query | Committed-map lookup | Read an architectural name's current committed physical identity |
| `free_mask_o`, `ready_mask_o`, `free_count_o` | Current state | Scheduling/control/debug | Registered availability state and its free-entry count |

The interface uses one atomic dispatch decision:

```text
dispatch_fire = dispatch_valid_i AND dispatch_ready_o
```

The controller must use that same event to allocate the instruction's other resources. It must
not allocate a physical destination while the corresponding ROB/issue/memory allocation fails.
No change occurs just because a candidate record is visible at `renamed_o`.

`renamed_o` is a combinational preview, not an independently buffered output channel. Capture
it only on dispatch_fire. Its candidate physical destination can change while dispatch is
stalled, for example when retirement makes a lower-numbered identity available.

## 4. Read sources before replacing the destination

For `addi x5,x5,1`, the source must refer to the version that existed before this instruction.
Both physical source lookups use the current speculative map. Only at an accepted dispatch edge
does the map change to the new destination.

For unused sources the physical tag is p0 and readiness is true. Immediate bits cannot create
false dependencies. x0 always maps to p0 and never allocates a destination. The module defensively
clears the destination-write flag for x0 or a faulted instruction even if its caller supplies it.

A valid fault token still needs a ROB entry for precise trapping, but needs no destination or
source wait. Do not drop it as an empty slot just because its execution controls are disabled.

## 5. Free identities and readiness are different

| Physical identity state | Free bit | Ready bit | Meaning |
|---|---:|---:|---|
| p0 | 0 | 1 | Reserved constant zero |
| Initial p1..p31 | 0 | 1 | Mapped startup values, reset to zero by block 3 |
| Initial p32..p63 | 1 | 0 | Available for allocation |
| Newly allocated destination | 0 | 0 | Owned by a producer whose value is not ready |
| Completed live destination | 0 | 1 | Value is available to consumers |
| Released replaced destination | 1 | 0 | Can be allocated again |

The allocator scans current free bits and chooses the lowest available identity. It does not
reuse an identity released by retirement on the same edge; that identity becomes available on
the next cycle. This removes same-edge recycling ambiguity and makes the handoff easy to inspect.

The free count describes physical storage availability, not ROB capacity. With 64 physical
identities, 32 architectural mappings, and a 16-entry ROB, a normal full-core execution cannot
exhaust all 32 initially free destinations. The standalone allocator test can deliberately
allocate more outstanding writers to exercise exhaustion, while making clear that the full
core's ROB would have stopped dispatch sooner.

## 6. Same-cycle completion and readiness

Completion can make a newly dispatched consumer ready on the same edge:

```text
current map says x5 -> p32; registered ready[p32] is still 0
accepted completion carries p32
consumer source lookup returns p32
src_ready output bypasses to 1 now
on the edge, ready[p32] becomes 1
```

The physical-register bypass from block 3 supplies the matching value. Control readiness and
data forwarding must refer to the same accepted completion. Neither may use an unaccepted
execution-unit result waiting behind another completion.

The module ignores readiness updates for a faulted result, a result with no GPR write, p0,
or a currently free destination. This is not a substitute for ROB identity validation. A stale
completion targeting a newly reallocated live physical identity can only be rejected by the
owning ROB/generation checks and cancellation rules in later blocks.

The normal update order is completion-ready, retirement release, then new allocation clears
its destination's readiness. Source readiness uses current mappings before that destination change.

## 7. Retirement changes the committed map

The retirement input means the instruction has successfully reached its in-order architectural
boundary. The future ROB must enforce this and must not retire a faulting or incomplete instruction.

For a real destination write, the new physical identity becomes committed and the old identity
is released. At this point `old_pdst` must equal that architectural register's current committed
mapping, and `pdst` must be the instruction's allocated nonzero destination. The ROB controller
will check this contract; the rename module does not independently implement a second ROB.

The speculative map is not overwritten by an ordinary retirement. It might already point to
an even younger version. This is why the two writers of x5 can finish and retire correctly
without forgetting the younger mapping.

## 8. Recovery applies the committed state without clearing values

`recover_i` is an apply pulse after the controller has cancelled younger execution, drained held
completions, and made resource reuse safe. This module does not perform that cancellation itself.

On the apply edge:

1. Incorporate any simultaneous successful retirement into the committed map.
2. Copy that updated committed map to the speculative map.
3. Mark all physical identities free except those present in the committed map.
4. Mark committed identities ready and all other identities not ready.
5. Preserve p0 as reserved/ready and acknowledge the application through recover_done_o.

Including retirement first matters for a mispredicted JAL/JALR. Its link register must commit
before younger work is discarded. Restoring yesterday's map would lose the valid link write.

Recovery blocks dispatch. If recover_i is held rather than pulsed, restoration repeats and
recover_done_o remains asserted after each edge; the future controller will use a single pulse.
There is no physical-register reset on recovery, so committed values remain intact.

## 9. Testing and waveform guidance

From the project directory:

```sh
python3 scripts/run_checks.py --block 4
python3 scripts/run_checks.py --block all
```

For an optional waveform, use `--block 4 --waves`. Reports and source hashes are saved in
`reports/block04_results.json` and `reports/block04_simulation.log`; the waveform is under
`build/block04/block04.vcd`.

The tests exercise two writers of x5, self-read/write source ordering, unused sources, readiness
bypass, blocked atomic allocation, exhaustion at allocator-unit level, retirement/freeing,
simultaneous events, safe-recovery application, and reset. Check exact coverage against the
testbench source and the recorded result; this is not whole-core out-of-order verification.

The testbench controls clock pulses explicitly so it can query all mappings between update edges.
This is a sequential-logic functional test, not a 50 MHz timing experiment. The 768 mixed
transactions use a small software queue to supply retirement in valid program order; that queue
is a test driver, not a delivered hardware ROB.

Useful waveform signals are dispatch_valid_i/ready_o, renamed_o's source/destination tags,
free_mask_o/count, ready_mask_o, src1/2_ready_o, retirement inputs, recover_i/done, and the
committed query for x5. During the two-writer example, observe different destination tags,
the newest source mapping, and committed x5 staying unchanged until retirement.

For XSim on Linux after sourcing Vivado's settings file:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_rename.sv tb/tb_block04.sv
xelab tb_block04 -s block04_sim
xsim block04_sim --runall
```

Icarus may report that constant selects in always_comb are sensitized to the whole vector.
This is a simulation sensitivity diagnostic; full messages remain in the log. Vivado resource
mapping, timing, and bitstream results remain unverified until run on your Linux installation.

## 10. Interview questions and model answers

**Does renaming solve every dependency?**
No. It removes false WAR/WAW dependencies caused by reusing register names. RAW dependencies
remain and are enforced using producer identities and readiness.

**Why retain both speculative and committed maps?**
The speculative map follows dispatched instructions; the committed map records architecturally
accepted versions and provides a safe recovery point.

**Why save an instruction's old physical destination?**
It identifies the version replaced at retirement. Freeing it at execution completion would be
too early because older work or committed state may still require it.

**Why not update the speculative map from every retirement?**
It may already contain a younger mapping. Overwriting it would lose that speculative dependency chain.

**Why is a zero value not the same as a ready value?**
The bits might be reset or left over from an earlier owner. Only the producer-completion/control
state establishes that this instruction's value is available.

**Why rebuild the free list from committed mappings on recovery?**
All younger ownership has been cancelled. The committed mappings identify the physical versions
that must remain live; the rest can be reused after the cancellation barrier.

**Can a source and destination use the same architectural name?**
Yes. Read the previous source mapping first, then allocate/update the destination mapping.

**What remains for a real out-of-order demonstration?**
The ROB, issue scheduler, overlapping execution units, completion arbitration, and integrated
recovery must be connected and tested. This block implements the required naming/readiness state.

## 11. References

- [BOOM rename-stage documentation](https://docs.boom-core.org/en/latest/sections/rename-stage.html)
- [BOOM reorder buffer and retirement](https://docs.boom-core.org/en/latest/sections/reorder-buffer.html)
- [lowRISC coding style](https://github.com/lowRISC/style-guides/blob/master/VerilogCodingStyle.md)

This is original project source. Its retirement-based recovery is intentionally conservative;
it does not implement BOOM's per-branch checkpoint mechanism.

## Complete rename, free-list, and readiness module

Source: `rtl/rv32_rename.sv`.

```systemverilog
// ID rename: map program register names to physical versions and track availability.
// The ROB/completion controller supplies validated retirement and completion events.
`timescale 1ns / 1ps

module rv32_rename (
  input  logic                       clk_i,
  input  logic                       rst_ni,
  input  logic                       dispatch_valid_i,
  input  logic                       dispatch_resources_ready_i,
  input  rv32_ooo_pkg::decoded_uop_t  decoded_i,
  input  rv32_ooo_pkg::rob_tag_t      dispatch_rob_tag_i,
  output logic                       dispatch_ready_o,
  output rv32_ooo_pkg::renamed_uop_t  renamed_o,
  output logic                       src1_ready_o,
  output logic                       src2_ready_o,
  input  logic                       completion_valid_i,
  input  rv32_ooo_pkg::completion_t   completion_i,
  input  logic                       retire_valid_i,
  input  rv32_ooo_pkg::renamed_uop_t  retire_uop_i,
  input  logic                       recover_i,
  output logic                       recover_done_o,
  input  rv32_ooo_pkg::arch_reg_t     committed_query_arch_i,
  output rv32_ooo_pkg::phys_reg_t     committed_query_phys_o,
  output rv32_ooo_pkg::phys_mask_t    free_mask_o,
  output rv32_ooo_pkg::phys_mask_t    ready_mask_o,
  output rv32_ooo_pkg::phys_count_t   free_count_o
);
  import rv32_ooo_pkg::*;

  phys_reg_t speculative_q [0:ARCH_REG_COUNT-1];
  phys_reg_t speculative_d [0:ARCH_REG_COUNT-1];
  phys_reg_t committed_q [0:ARCH_REG_COUNT-1];
  phys_reg_t committed_d [0:ARCH_REG_COUNT-1];
  phys_mask_t free_q, free_d;
  phys_mask_t ready_q, ready_d;
  phys_reg_t allocated_phys;
  logic allocation_available;
  logic needs_destination;
  logic dispatch_fire;
  logic completion_sets_ready;
  logic retirement_writes;
  integer allocate_index;
  integer count_index;
  integer state_index;
  integer reset_index;
  integer free_count;

  assign free_mask_o = free_q;
  assign ready_mask_o = ready_q;
  assign committed_query_phys_o = committed_q[committed_query_arch_i];

  always_comb begin
    allocated_phys = '0;
    allocation_available = 1'b0;
    // Allocate from CURRENT free entries. A retirement release is usable next cycle.
    for (allocate_index = 1; allocate_index < PHYS_REG_COUNT; allocate_index = allocate_index + 1)
      if (!allocation_available && free_q[allocate_index]) begin
        allocated_phys = allocate_index[PHYS_REG_W-1:0];
        allocation_available = 1'b1;
      end

    free_count = 0;
    for (count_index = 1; count_index < PHYS_REG_COUNT; count_index = count_index + 1)
      if (free_q[count_index]) free_count = free_count + 1;
    free_count_o = free_count[$clog2(PHYS_REG_COUNT+1)-1:0];
  end

  always_comb begin
    needs_destination = decoded_i.writes_rd && (decoded_i.rd != 0) &&
                        !decoded_i.fetch.exception.valid;
    completion_sets_ready = completion_valid_i && completion_i.writes_rd &&
                            !completion_i.exception.valid && (completion_i.pdst != 0) &&
                            !free_q[completion_i.pdst];
    retirement_writes = retire_valid_i && retire_uop_i.decoded.writes_rd &&
                        (retire_uop_i.decoded.rd != 0) &&
                        !retire_uop_i.decoded.fetch.exception.valid;

    renamed_o = '0;
    renamed_o.decoded = decoded_i;
    renamed_o.decoded.writes_rd = needs_destination;
    renamed_o.rob_tag = dispatch_rob_tag_i;
    if (decoded_i.fetch.exception.valid) begin
      renamed_o.decoded.uses_rs1 = 1'b0;
      renamed_o.decoded.uses_rs2 = 1'b0;
    end else begin
      // Read the old mappings BEFORE replacing the destination, including rd==rs1/rs2.
      if (decoded_i.uses_rs1) renamed_o.prs1 = speculative_q[decoded_i.rs1];
      if (decoded_i.uses_rs2) renamed_o.prs2 = speculative_q[decoded_i.rs2];
    end
    if (needs_destination) begin
      renamed_o.old_pdst = speculative_q[decoded_i.rd];
      if (allocation_available) renamed_o.pdst = allocated_phys;
    end

    src1_ready_o = (renamed_o.prs1 == 0) || ready_q[renamed_o.prs1] ||
                  (completion_sets_ready && completion_i.pdst == renamed_o.prs1);
    src2_ready_o = (renamed_o.prs2 == 0) || ready_q[renamed_o.prs2] ||
                  (completion_sets_ready && completion_i.pdst == renamed_o.prs2);
    dispatch_ready_o = rst_ni && !recover_i && dispatch_resources_ready_i &&
                       (!needs_destination || allocation_available);
    dispatch_fire = dispatch_valid_i && dispatch_ready_o;
  end

  always_comb begin
    for (state_index = 0; state_index < ARCH_REG_COUNT; state_index = state_index + 1) begin
      speculative_d[state_index] = speculative_q[state_index];
      committed_d[state_index] = committed_q[state_index];
    end
    free_d = free_q;
    ready_d = ready_q;

    if (completion_sets_ready) ready_d[completion_i.pdst] = 1'b1;

    if (retirement_writes) begin
      committed_d[retire_uop_i.decoded.rd] = retire_uop_i.pdst;
      // old_pdst must equal the current committed mapping at in-order retirement.
      // The controller checks that contract; p0 is never returned to the free list.
      if (retire_uop_i.old_pdst != 0) begin
        free_d[retire_uop_i.old_pdst] = 1'b1;
        ready_d[retire_uop_i.old_pdst] = 1'b0;
      end
    end

    if (dispatch_fire && needs_destination) begin
      speculative_d[decoded_i.rd] = allocated_phys;
      free_d[allocated_phys] = 1'b0;
      ready_d[allocated_phys] = 1'b0;
    end

    if (recover_i) begin
      // recover_i is an APPLY pulse after cancellation acknowledgements and drained results.
      // committed_d includes a same-cycle retiring branch's link-register mapping.
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
```
