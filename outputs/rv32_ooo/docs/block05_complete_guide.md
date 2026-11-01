# Block 5: two-wide reorder buffer and ordered architectural trace

The ROB lets instructions finish in a different order while preserving program order at
retirement. This block tracks up to 16 instructions, accepts up to two completions, and can
retire an eligible prefix of two instructions per edge.

The testbench supplies controlled completion packets. It does not yet execute a real divider,
run the scheduler, update machine CSRs, or perform AXI transactions.

## 1. Older DIV, younger ADD completes first

```asm
div x5, x1, x2       # Older: may take many execution cycles.
add x6, x3, x4       # Younger and independent.
```

Both can enter the ROB in one dispatch group. ADD may complete first; its entry records the
result, but DIV remains the incomplete head. ADD cannot retire past it.

| Event | Older DIV entry | Younger ADD entry | Architectural retirement |
|---|---|---|---|
| Dispatch pair | Valid, incomplete | Valid, incomplete | None |
| ADD completion accepted | Incomplete | Complete | None |
| DIV completion accepted | Becomes complete | Complete | None on that same completion edge |
| Following eligible retirement edge | Removed | Removed | DIV then ADD, up to two events |

Completion does not directly change the committed map. The retirement outputs provide the
ordered records used by block 4 to advance committed versions and free replaced identities.
The registered trace contains the same architectural sequence even when execution order differs.

## 2. Circuit organization

```text
 ID: renamed pair + proposed ROB tags
                    |
              dispatch prefix (0, 1, or 2)
                    v
       tail -> [16 entries in a circular buffer] <- tagged completions
                              |
                         head / head+1
                              |
            complete + permission + ordering checks
                              |
                   retirement / precise trap prefix
                        |                    |
               committed-map updates    architectural event pair
                                             |
                                      trace recorder
```

Each entry retains its instruction, original PC, rename history, completion state, exception,
actual next PC, and completed memory-access metadata. Head/tail pointers and occupancy track
allocation order; per-slot generations distinguish different uses of a recycled slot.

The array is an RTL storage description with multiple update paths. Its actual FPGA resource
mapping and timing still need Vivado verification. Five logical architectural sections do not
mean every instruction waits exactly five cycles.

## 3. Dispatch and tag ownership

The ROB proposes tags for tail and tail+1. Each includes the slot index and its current
generation. The same tag must be captured by rename, the issue queue, execution, and memory
ownership tracking when that instruction is accepted.

Dispatch accepts only a program-order prefix: 00, 01, or 11. It never accepts lane 1 while
lane 0 stalls. With one slot free, only the older instruction can transfer; the front end
must compact the remaining younger instruction into lane 0 next cycle.

A full buffer does not borrow capacity from a simultaneous retirement on that same edge.
The newly released slots become available next cycle. This is a deliberate simple contract,
not a statement that all processors use the same lookahead policy.

An offered tag that does not match the ROB candidate stalls that lane. An invalid prefix is
diagnosed. Known faults, system serialization, mispredictions, recovery, and external hold
can stop admission while older work continues toward retirement.

Each slot's generation increments at allocation and survives a microarchitectural flush.
Generation alone is not an unlimited guarantee: cancellation and draining must finish before
flushed resources are reused, and global reset must reset the complete system consistently.

## 4. Two completion ports

The completion interface separates consuming a packet from accepting its ownership:

| Output | Meaning |
|---|---|
| completion_ready | The selected packet can be consumed this cycle |
| completion_accepted | It matches a live, incomplete instruction and its destination metadata |
| completion_rejected | It was consumed but must be discarded without architectural effects |

A completion is checked against the full ROB tag, expected physical destination, and decoded
nonzero-register write intent. Old generations, retired entries, wrong destinations, and
already-completed duplicates are rejected. If two selected packets claim the same owner, lane 0
has deterministic priority and the second is rejected/diagnosed.

Only accepted non-faulting register results may write block 3 or mark readiness in block 4.
An accepted fault completion still makes its ROB entry complete-for-trap, but publishes no
usable physical result. A faulting load or JAL retains its original decoded write intent in
the completion; exception.valid separately disables the physical write.

Completing an instruction at the head does not bypass its completion state into same-edge
retirement. The registered complete bit is observed on the following cycle. This makes state
and trace timing explicit.

## 5. Retirement is an ordered prefix

head_valid describes occupied head entries, not necessarily completed entries. head_complete
can show a completed younger instruction while the older one waits.

Head 0 may advance only when valid, complete, and permitted by head_ready[0]. Head 1 may
advance only if head 0 also advances and does not create a fault, misprediction, or serialized
system boundary. System operations retire alone. The controller must supply a prefix readiness
mask; requesting only the younger head is diagnosed and cannot skip the older instruction.

Important cases:

- Two completed ordinary instructions: both can retire in program order.
- Older incomplete, younger complete: neither retires.
- Fault in head 0: only its trap event advances; the younger instruction cannot retire.
- Normal head 0 and an eligible nonsystem fault in head 1: the older instruction retires, then the trap is
  taken as the second ordered event. Its destination/store effects are suppressed.
- Mispredicted head 0: retire its valid effects, including a jump link, then stop younger effects.

retire_fire supplies successful retirement records to the committed map and retirement counter.
trap_fire supplies a precise fault event to the future CSR/trap controller; a fault does not
increment the retired-instruction counter.

## 6. Pending mispredictions, faults, and systems

The ROB scans live entries in program order and reports the oldest completed, non-faulting
misprediction. If a younger branch resolves first and an older branch later mispredicts, the
older entry wins. This stops new dispatch; the scheduler must still allow older work to finish.
Stopping all issue would deadlock an older unresolved producer.

A live fault or system operation also stops younger admission. A system instruction in lane 0
prevents dispatching lane 1 behind it. CSR operations, fences, MRET, and WFI are then executed
at the appropriate serialized head boundary by the later system controller.

After a trap or a retiring recovery boundary, recovery_required latches. It masks head permission
and new dispatch even if head_ready remains high. This prevents a younger completed instruction
or MMIO access from slipping through while cancellation is arranged. Existing completion packets
can still be drained and validated.

FENCE.I and MRET require front-end recovery as well as serialized retirement. Their real CSR,
fetch, predictor, and interrupt effects are implemented in later blocks, not by this queue alone.

## 7. Safe flush and memory effects

flush_i is an APPLY operation after the recovery controller has cancelled younger execution,
drained held results/uncancellable transactions, and made reuse safe. It has priority over
new dispatch, completion acceptance, and head events. It clears ROB entries, pointers, count,
and the recovery lock while retaining generation counters.

A flush is not an architectural retirement event and must not reset trace ordering or clear
committed physical values. The rename module restores committed mappings on its corresponding
safe recovery edge. The controller coordinates these operations; the ROB does not implement
execution-unit cancellation itself.

The future LSU may start an access only for a live, incomplete head, with no recovery lock.
It retains ownership until response and retirement. Once the response completes that head,
the memory engine must not issue the same request again while waiting for retirement.

Completed memory metadata travels through completion into the ROB: address, store data,
byte strobes, and whether an access was a write. A successful retirement can include this
information in the trace. A fault event cannot advertise a successful register or store effect.
This is observation metadata, not an AXI request or proof that a bus transaction happened.

## 8. Trace recorder

The ROB produces event templates only when a head actually advances. rv32_trace_recorder
registers up to two accepted templates and stamps monotonically increasing 64-bit event orders.
For two events, lane 0 gets N and lane 1 gets N+1; the next event starts at N+2. Idle cycles
produce invalid trace records without consuming order numbers.

Trace ordering survives a microarchitectural flush; only global reset resets the recorder.
The later interrupt controller can merge an interrupt event before this recorder at a safe
architectural boundary. Testing an injected interrupt-shaped trace record is not testing
interrupt management. This project trace is not claimed to be the standard RVFI interface.

## 9. Run and inspect the tests

```sh
python3 scripts/run_checks.py --block 5 --waves
```

The optional waveform is `build/block05/block05.vcd`. Results and source hashes are recorded
in `reports/block05_results.json` and the accompanying simulation log. The bench checks
out-of-order completion/in-order retirement, two-wide prefixes, identity rejection, wrap/reuse,
faults, recovery locking, store-trace fields, serialization, and registered event ordering.

The same runner also executes tb_block05_integration, connecting the built renamer, ROB,
physical register file, and trace recorder. It injects the ADD result 19 before the DIV result 6,
checks that storage/readiness can change before committed mappings, observes ordered dual
retirement, and verifies that stale completions are rejected before PRF bypass/write. It also
checks partial-pair compaction and preservation of committed values across safe apply. These
are connected control checks, not real DIV execution or a complete processor simulation.

For the DIV/ADD example, observe both allocated tags, the ADD completion acceptance, the
incomplete older head, and the absence of retirement. After the DIV completion is stored,
observe the accepted retirement prefix and the two trace records in program order.

XSim on Linux, after sourcing Vivado settings:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_reorder_buffer.sv rtl/rv32_trace_recorder.sv tb/tb_block05.sv
xelab tb_block05 -s block05_sim
xsim block05_sim --runall
```

These are functional tests. They do not establish full-core ISA compliance, execution overlap,
AXI effects, FPGA fit, or 50 MHz timing. Full instruction execution and architectural comparison
will be tested after the remaining blocks are connected.

## 10. Interview questions

**Why not commit a younger result as soon as it finishes?** An older instruction might still
fault. In-order commitment preserves precise architectural state while permitting speculative work.

**Why retain an old generation across flush?** A slot may be reused immediately after safe
recovery. Its new instruction must not accept a stale result from its prior owner.

**Why consume rejected completions?** A stale result must be dropped rather than block a
completion unit forever. It must not write a physical register or wake a dependent instruction.

**Can two instructions retire to the same architectural register?** Yes. They have distinct
physical versions; apply their committed-map updates in program order, with the younger last.

**What if the second retirement faults?** The older instruction can commit, then the younger
instruction traps without publishing its destination. No still-younger work can commit.

**Why is completion separate from trace output?** Completion is speculative execution state.
The architectural trace records only accepted retirement/trap events, after ordering checks.

**Is this enough to claim a working two-issue processor?** No. Issue selection, overlapping
execution, memory, prediction, system control, and the integrated front end are still required.

## References

- [BOOM ROB and retirement](https://docs.boom-core.org/en/latest/sections/reorder-buffer.html)
- [BOOM issue unit](https://docs.boom-core.org/en/latest/sections/issue-units.html)
- [RISC-V specifications](https://docs.riscv.org/reference/isa/)

The complete active modules and testbench follow this explanation. They are original project
source; retirement-based recovery is this project's conservative implementation choice.

## Complete two-wide reorder buffer

Source: `rtl/rv32_reorder_buffer.sv`.

```systemverilog
// Two-wide WB/commit: allocate in program order, complete by tag, retire an ordered prefix.
// Safe flush applies only after the controller has cancelled/drained younger work.
`timescale 1ns / 1ps

module rv32_reorder_buffer (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  input  logic                         dispatch_hold_i,
  input  logic [1:0]                   dispatch_valid_i,
  input  rv32_ooo_pkg::renamed_pair_t   dispatch_uop_i,
  output logic [1:0]                   dispatch_ready_o,
  output logic [1:0]                   dispatch_fire_o,
  output rv32_ooo_pkg::rob_tag_pair_t   dispatch_tag_o,
  input  logic [1:0]                   completion_valid_i,
  input  rv32_ooo_pkg::completion_pair_t completion_i,
  output logic [1:0]                   completion_ready_o,
  output logic [1:0]                   completion_accepted_o,
  output logic [1:0]                   completion_rejected_o,
  output logic [1:0]                   head_valid_o,
  output logic [1:0]                   head_complete_o,
  output rv32_ooo_pkg::rob_entry_pair_t head_entry_o,
  input  logic [1:0]                   head_ready_i,
  input  rv32_ooo_pkg::word_t           head_trap_target_i,
  output logic [1:0]                   retire_fire_o,
  output logic [1:0]                   trap_fire_o,
  output rv32_ooo_pkg::retire_trace_pair_t retirement_event_o,
  output logic                         recovery_required_o,
  output logic                         mispredict_pending_o,
  output rv32_ooo_pkg::rob_tag_t        mispredict_tag_o,
  output rv32_ooo_pkg::word_t           mispredict_next_pc_o,
  output logic                         serial_pending_o,
  output logic                         fault_pending_o,
  input  logic                         flush_i,
  output logic                         flush_done_o,
  input  rv32_ooo_pkg::rob_tag_t        query_tag_i,
  output logic                         query_live_o,
  output rv32_ooo_pkg::rob_index_t      query_age_o,
  output rv32_ooo_pkg::rob_index_t      head_index_o,
  output rv32_ooo_pkg::rob_count_t      occupancy_o,
  output logic                         full_o,
  output logic                         empty_o,
  output logic                         protocol_error_o
);
  import rv32_ooo_pkg::*;

  rob_entry_t entries_q [0:ROB_DEPTH-1];
  rob_entry_t entries_d [0:ROB_DEPTH-1];
  logic [ROB_GENERATION_W-1:0] generation_q [0:ROB_DEPTH-1];
  logic [ROB_GENERATION_W-1:0] generation_d [0:ROB_DEPTH-1];
  rob_index_t head_q, head_d, tail_q, tail_d, head1_index, tail1_index;
  rob_count_t count_q, count_d;
  logic recovery_q, recovery_d;
  renamed_uop_t dispatch0, dispatch1;
  completion_t completion0, completion1;
  rob_entry_t head0, head1, owner0, owner1, query_owner, scan_entry;
  rob_entry_t updated0, updated1, new0, new1;
  rob_tag_t tag0, tag1;
  logic [1:0] head_fire;
  logic owner0_matches, owner1_matches, completion_conflict;
  logic head0_serial, head1_serial, allocation_allowed;
  integer scan_offset, scan_index, state_index, reset_index;
  integer push_count, pop_count;

  function automatic rob_index_t advance_index(input rob_index_t base, input integer amount);
    integer next_index;
    begin
      next_index = base + amount;
      if (next_index >= ROB_DEPTH) next_index = next_index - ROB_DEPTH;
      advance_index = next_index[ROB_INDEX_W-1:0];
    end
  endfunction

  // Read a whole entry before selecting its fields for Icarus/Vivado portability.
  assign dispatch0 = dispatch_uop_i.lane0;
  assign dispatch1 = dispatch_uop_i.lane1;
  assign completion0 = completion_i.lane0;
  assign completion1 = completion_i.lane1;
  assign head1_index = advance_index(head_q, 1);
  assign tail1_index = advance_index(tail_q, 1);
  assign head0 = entries_q[head_q];
  assign head1 = entries_q[head1_index];
  assign owner0 = entries_q[completion0.rob_tag.index];
  assign owner1 = entries_q[completion1.rob_tag.index];
  assign query_owner = entries_q[query_tag_i.index];
  assign head_index_o = head_q;
  assign occupancy_o = count_q;
  assign full_o = count_q == ROB_DEPTH;
  assign empty_o = count_q == 0;
  assign recovery_required_o = recovery_q;
  assign dispatch_tag_o.lane0 = tag0;
  assign dispatch_tag_o.lane1 = tag1;

  always_comb begin
    mispredict_pending_o = 1'b0;
    mispredict_tag_o = '0;
    mispredict_next_pc_o = '0;
    serial_pending_o = 1'b0;
    fault_pending_o = 1'b0;
    scan_entry = '0;
    scan_index = 0;
    // Age-order scan permits an older, later-resolving misprediction to take priority.
    for (scan_offset = 0; scan_offset < ROB_DEPTH; scan_offset = scan_offset + 1) begin
      scan_index = head_q + scan_offset;
      if (scan_index >= ROB_DEPTH) scan_index = scan_index - ROB_DEPTH;
      scan_entry = entries_q[scan_index];
      if (scan_entry.valid) begin
        if (scan_entry.uop.decoded.op_class == UOP_SYSTEM) serial_pending_o = 1'b1;
        if (scan_entry.exception.valid) fault_pending_o = 1'b1;
        if (!mispredict_pending_o && scan_entry.complete && !scan_entry.exception.valid &&
            scan_entry.control_flow.mispredicted) begin
          mispredict_pending_o = 1'b1;
          mispredict_tag_o = scan_entry.uop.rob_tag;
          mispredict_next_pc_o = scan_entry.control_flow.next_pc;
        end
      end
    end
    if (!rst_ni || flush_i) begin
      mispredict_pending_o = 1'b0;
      mispredict_tag_o = '0;
      mispredict_next_pc_o = '0;
      serial_pending_o = 1'b0;
      fault_pending_o = 1'b0;
    end
  end

  always_comb begin
    tag0.index = tail_q;
    tag0.generation = generation_q[tail_q];
    tag1.index = tail1_index;
    tag1.generation = generation_q[tail1_index];
    allocation_allowed = rst_ni && !flush_i && !recovery_q && !dispatch_hold_i &&
                         !mispredict_pending_o && !serial_pending_o && !fault_pending_o;
    // Capacity is current occupancy: slots released at this edge are usable next cycle.
    dispatch_ready_o[0] = allocation_allowed && count_q < ROB_DEPTH &&
                          dispatch0.rob_tag == tag0;
    dispatch_fire_o[0] = dispatch_valid_i[0] && dispatch_ready_o[0];
    // Mask 10 cannot transfer: lane 1 requires lane 0's fire. Ready 0 stays valid-independent.
    dispatch_ready_o[1] = dispatch_fire_o[0] && count_q < ROB_DEPTH-1 &&
                          dispatch1.rob_tag == tag1 && !dispatch0.decoded.fetch.exception.valid &&
                          dispatch0.decoded.op_class != UOP_SYSTEM;
    dispatch_fire_o[1] = dispatch_valid_i[1] && dispatch_ready_o[1];

    completion_ready_o = rst_ni && !flush_i ? 2'b11 : 2'b00;
    owner0_matches = owner0.valid && !owner0.complete &&
                     owner0.uop.rob_tag == completion0.rob_tag &&
                     owner0.uop.pdst == completion0.pdst &&
                     owner0.uop.decoded.writes_rd == completion0.writes_rd;
    owner1_matches = owner1.valid && !owner1.complete &&
                     owner1.uop.rob_tag == completion1.rob_tag &&
                     owner1.uop.pdst == completion1.pdst &&
                     owner1.uop.decoded.writes_rd == completion1.writes_rd;
    completion_accepted_o[0] = completion_valid_i[0] && completion_ready_o[0] && owner0_matches;
    completion_accepted_o[1] = completion_valid_i[1] && completion_ready_o[1] && owner1_matches;
    completion_conflict = (&completion_accepted_o) &&
                          ((completion0.rob_tag == completion1.rob_tag) ||
                           (completion0.writes_rd && completion1.writes_rd &&
                            completion0.pdst != 0 && completion0.pdst == completion1.pdst));
    // Invalid packets are consumed but never broadcast; duplicate lane 1 loses to lane 0.
    if (completion_conflict) completion_accepted_o[1] = 1'b0;
    completion_rejected_o = completion_valid_i & completion_ready_o & ~completion_accepted_o;

    head_valid_o = '0;
    if (rst_ni && !flush_i && !recovery_q && count_q != 0 && head0.valid) begin
      head_valid_o[0] = 1'b1;
      if (count_q > 1 && head1.valid) head_valid_o[1] = 1'b1;
    end
    head_complete_o[0] = head_valid_o[0] && head0.complete;
    head_complete_o[1] = head_valid_o[1] && head1.complete;
    head_entry_o = '0;
    if (head_valid_o[0]) head_entry_o.lane0 = head0;
    if (head_valid_o[1]) head_entry_o.lane1 = head1;
    head0_serial = head0.uop.decoded.op_class == UOP_SYSTEM;
    head1_serial = head1.uop.decoded.op_class == UOP_SYSTEM;
    head_fire[0] = head_complete_o[0] && head_ready_i[0];
    head_fire[1] = head_fire[0] && head_complete_o[1] && head_ready_i[1] &&
                   !head0.exception.valid && !head0.control_flow.mispredicted &&
                   !head0_serial && !head1_serial;
    retire_fire_o[0] = head_fire[0] && !head0.exception.valid;
    retire_fire_o[1] = head_fire[1] && !head1.exception.valid;
    trap_fire_o[0] = head_fire[0] && head0.exception.valid;
    trap_fire_o[1] = head_fire[1] && head1.exception.valid;

    query_live_o = rst_ni && !flush_i && query_owner.valid &&
                   query_owner.uop.rob_tag == query_tag_i;
    query_age_o = '0;
    // The fixed power-of-two ring makes subtraction a modulo-16 age offset.
    if (query_live_o) query_age_o = query_tag_i.index - head_q;
    protocol_error_o = rst_ni && ((dispatch_valid_i == 2'b10) ||
                       (head_ready_i == 2'b10) || completion_conflict);
  end

  function automatic retire_trace_t event_template(input rob_entry_t entry, input logic trap);
    retire_trace_t event_record;
    begin
      event_record = '0;
      event_record.valid = 1'b1;
      event_record.pc = entry.uop.decoded.fetch.pc;
      event_record.instruction = entry.uop.decoded.fetch.instruction;
      if (trap) begin
        event_record.trap = 1'b1;
        event_record.cause = entry.exception.cause;
        event_record.tval = entry.exception.tval;
        event_record.next_pc = head_trap_target_i;
      end else begin
        event_record.next_pc = entry.control_flow.next_pc;
        if (entry.uop.decoded.writes_rd && entry.uop.decoded.rd != 0) begin
          event_record.rd = entry.uop.decoded.rd;
          event_record.rd_value = entry.result;
        end
        if (entry.memory_access.valid) begin
          event_record.memory_write = entry.memory_access.write;
          event_record.memory_address = entry.memory_access.address;
          event_record.memory_wdata = entry.memory_access.wdata;
          event_record.memory_wstrb = entry.memory_access.wstrb;
        end
      end
      event_template = event_record;
    end
  endfunction

  always_comb begin
    retirement_event_o = '0;
    if (head_fire[0]) retirement_event_o.lane0 = event_template(head0, trap_fire_o[0]);
    if (head_fire[1]) retirement_event_o.lane1 = event_template(head1, trap_fire_o[1]);
  end

  always_comb begin
    for (state_index = 0; state_index < ROB_DEPTH; state_index = state_index + 1) begin
      entries_d[state_index] = entries_q[state_index];
      generation_d[state_index] = generation_q[state_index];
    end
    head_d = head_q;
    tail_d = tail_q;
    count_d = count_q;
    recovery_d = recovery_q;
    updated0 = owner0;
    updated1 = owner1;
    new0 = '0;
    new1 = '0;
    push_count = 0;
    pop_count = 0;

    if (completion_accepted_o[0]) begin
      updated0.complete = 1'b1;
      updated0.result = completion0.value;
      updated0.exception = completion0.exception;
      updated0.control_flow = completion0.control_flow;
      updated0.memory_access = completion0.memory_access;
      entries_d[completion0.rob_tag.index] = updated0;
    end
    if (completion_accepted_o[1]) begin
      updated1.complete = 1'b1;
      updated1.result = completion1.value;
      updated1.exception = completion1.exception;
      updated1.control_flow = completion1.control_flow;
      updated1.memory_access = completion1.memory_access;
      entries_d[completion1.rob_tag.index] = updated1;
    end

    if (head_fire[0]) begin
      entries_d[head_q] = '0;
      pop_count = pop_count + 1;
      if (trap_fire_o[0] || head0.control_flow.mispredicted ||
          head0.uop.decoded.system_op == SYS_MRET ||
          head0.uop.decoded.system_op == SYS_FENCE_I) recovery_d = 1'b1;
    end
    if (head_fire[1]) begin
      entries_d[head1_index] = '0;
      pop_count = pop_count + 1;
      if (trap_fire_o[1] || head1.control_flow.mispredicted) recovery_d = 1'b1;
    end

    if (dispatch_fire_o[0]) begin
      new0.valid = 1'b1;
      new0.uop = dispatch0;
      new0.exception = dispatch0.decoded.fetch.exception;
      new0.complete = dispatch0.decoded.fetch.exception.valid;
      new0.control_flow.next_pc = dispatch0.decoded.fetch.pc + 32'd4;
      entries_d[tail_q] = new0;
      generation_d[tail_q] = generation_q[tail_q] + 1'b1;
      push_count = push_count + 1;
    end
    if (dispatch_fire_o[1]) begin
      new1.valid = 1'b1;
      new1.uop = dispatch1;
      new1.exception = dispatch1.decoded.fetch.exception;
      new1.complete = dispatch1.decoded.fetch.exception.valid;
      new1.control_flow.next_pc = dispatch1.decoded.fetch.pc + 32'd4;
      entries_d[tail1_index] = new1;
      generation_d[tail1_index] = generation_q[tail1_index] + 1'b1;
      push_count = push_count + 1;
    end
    head_d = advance_index(head_q, pop_count);
    tail_d = advance_index(tail_q, push_count);
    count_d = count_q + push_count - pop_count;

    if (flush_i) begin
      // Dominant safe APPLY: no retirement event or new packet is accepted on this edge.
      // Generation counters survive so the next occupant differs from a flushed owner.
      for (state_index = 0; state_index < ROB_DEPTH; state_index = state_index + 1)
        entries_d[state_index] = '0;
      head_d = '0;
      tail_d = '0;
      count_d = '0;
      recovery_d = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (reset_index = 0; reset_index < ROB_DEPTH; reset_index = reset_index + 1) begin
        entries_q[reset_index] <= '0;
        generation_q[reset_index] <= '0;
      end
      head_q <= '0;
      tail_q <= '0;
      count_q <= '0;
      recovery_q <= 1'b0;
      flush_done_o <= 1'b0;
    end else begin
      for (reset_index = 0; reset_index < ROB_DEPTH; reset_index = reset_index + 1) begin
        entries_q[reset_index] <= entries_d[reset_index];
        generation_q[reset_index] <= generation_d[reset_index];
      end
      head_q <= head_d;
      tail_q <= tail_d;
      count_q <= count_d;
      recovery_q <= recovery_d;
      flush_done_o <= flush_i;
    end
  end
endmodule
```

## Complete architectural trace recorder

Source: `rtl/rv32_trace_recorder.sv`.

```systemverilog
// Register an ordered prefix of up to two architectural events and stamp event order.
// A later trap/interrupt controller may merge its events before this recorder.
`timescale 1ns / 1ps

module rv32_trace_recorder (
  input  logic                            clk_i,
  input  logic                            rst_ni,
  input  rv32_ooo_pkg::retire_trace_pair_t  event_i,
  output rv32_ooo_pkg::retire_trace_pair_t  trace_o,
  output logic                            protocol_error_o
);
  import rv32_ooo_pkg::*;
  logic [63:0] next_order_q, next_order_d;
  retire_trace_t event0, event1, stamped0, stamped1;
  retire_trace_pair_t next_trace;

  assign event0 = event_i.lane0;
  assign event1 = event_i.lane1;
  assign protocol_error_o = rst_ni && !event0.valid && event1.valid;

  always_comb begin
    stamped0 = event0;
    stamped1 = event1;
    stamped0.order = next_order_q;
    stamped1.order = next_order_q + 64'd1;
    next_trace = '0;
    next_order_d = next_order_q;
    if (!protocol_error_o && event0.valid) begin
      next_trace.lane0 = stamped0;
      next_order_d = next_order_q + 64'd1;
      if (event1.valid) begin
        next_trace.lane1 = stamped1;
        next_order_d = next_order_q + 64'd2;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      trace_o <= '0;
      next_order_q <= '0;
    end else begin
      trace_o <= next_trace;
      next_order_q <= next_order_d;
    end
  end
endmodule
```

## Complete self-checking testbench

Source: `tb/tb_block05.sv`.

```systemverilog
// Block 5 supplies controlled completions; it does not execute DIV/ADD or manage interrupts.
`timescale 1ns / 1ps

module tb_block05;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1, dispatch_hold, head_override, flush;
  logic [1:0] dispatch_valid, dispatch_ready, dispatch_fire;
  logic [1:0] completion_valid, completion_ready, completion_accepted, completion_rejected;
  logic [1:0] head_valid, head_complete, head_ready, retire_fire, trap_fire;
  renamed_pair_t dispatch_uop;
  rob_tag_pair_t dispatch_tag;
  completion_pair_t completion;
  rob_entry_pair_t head_entry;
  retire_trace_pair_t rob_event, injected_event, recorder_event, trace;
  logic recovery_required, mispredict_pending, serial_pending, fault_pending;
  logic flush_done, query_live, full, empty, protocol_error, recorder_error;
  rob_tag_t mispredict_tag, query_tag;
  word_t mispredict_next_pc, trap_target;
  rob_index_t query_age, head_index;
  rob_count_t occupancy;

  // The reference locates completion owners by scanning live records, not indexing by packet tag.
  rob_entry_t ref_entries [0:ROB_DEPTH-1];
  integer ref_generation [0:ROB_DEPTH-1];
  integer ref_head = 0, ref_tail = 0, ref_count = 0;
  logic ref_recovery = 0;
  logic [63:0] next_trace_order = 0;
  logic [1:0] expected_push, expected_accept, expected_reject, expected_pop;
  logic [1:0] expected_retire, expected_trap, expected_head_valid, expected_head_complete;
  logic expected_mispredict, expected_serial, expected_fault, expected_protocol;
  logic expected_completion_conflict;
  rob_tag_t expected_mispredict_tag;
  word_t expected_mispredict_pc;
  retire_trace_pair_t expected_event, expected_trace;
  integer expected_trace_count;
  logic expected_recorder_error;
  renamed_uop_t last0, last1, a_uop, b_uop, c_uop, d_uop, temp_uop;
  renamed_uop_t saved [0:31];
  completion_t temp_completion;
  rob_entry_t temp_entry;
  integer checks = 0, failures = 0;
  integer i, n, selected, serial_id = 0;
  word_t rng = 32'h17a5c0de;

  rv32_reorder_buffer dut (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_hold_i(dispatch_hold),
    .dispatch_valid_i(dispatch_valid), .dispatch_uop_i(dispatch_uop),
    .dispatch_ready_o(dispatch_ready), .dispatch_fire_o(dispatch_fire),
    .dispatch_tag_o(dispatch_tag), .completion_valid_i(completion_valid),
    .completion_i(completion), .completion_ready_o(completion_ready),
    .completion_accepted_o(completion_accepted), .completion_rejected_o(completion_rejected),
    .head_valid_o(head_valid), .head_complete_o(head_complete), .head_entry_o(head_entry),
    .head_ready_i(head_ready), .head_trap_target_i(trap_target),
    .retire_fire_o(retire_fire), .trap_fire_o(trap_fire), .retirement_event_o(rob_event),
    .recovery_required_o(recovery_required), .mispredict_pending_o(mispredict_pending),
    .mispredict_tag_o(mispredict_tag), .mispredict_next_pc_o(mispredict_next_pc),
    .serial_pending_o(serial_pending), .fault_pending_o(fault_pending),
    .flush_i(flush), .flush_done_o(flush_done), .query_tag_i(query_tag),
    .query_live_o(query_live), .query_age_o(query_age), .head_index_o(head_index),
    .occupancy_o(occupancy), .full_o(full), .empty_o(empty), .protocol_error_o(protocol_error)
  );
  assign recorder_event = head_override ? injected_event : rob_event;
  rv32_trace_recorder recorder (
    .clk_i(clk), .rst_ni(rst_n), .event_i(recorder_event),
    .trace_o(trace), .protocol_error_o(recorder_error)
  );

  function automatic word_t next_random(input word_t state);
    next_random = {state[30:0], state[31] ^ state[21] ^ state[1] ^ state[0]};
  endfunction

  function automatic rob_tag_t reference_tag(input integer index);
    rob_tag_t value;
    begin
      value.index = index;
      value.generation = ref_generation[index];
      reference_tag = value;
    end
  endfunction

  function automatic rob_entry_t logical_entry(input integer offset);
    begin
      if (offset < ref_count) logical_entry = ref_entries[(ref_head + offset) % ROB_DEPTH];
      else logical_entry = '0;
    end
  endfunction

  function automatic integer find_owner(input completion_t packet);
    integer index;
    rob_entry_t entry;
    begin
      find_owner = -1;
      for (index = 0; index < ROB_DEPTH; index = index + 1) begin
        entry = ref_entries[index];
        if (entry.valid && !entry.complete && entry.uop.rob_tag == packet.rob_tag &&
            entry.uop.pdst == packet.pdst &&
            entry.uop.decoded.writes_rd == packet.writes_rd) find_owner = index;
      end
    end
  endfunction

  function automatic phys_reg_t unused_physical(input phys_reg_t exclude);
    integer physical, index;
    logic used;
    rob_entry_t entry;
    begin
      unused_physical = 0;
      for (physical = 63; physical >= 32; physical = physical - 1) begin
        used = physical == exclude;
        for (index = 0; index < ROB_DEPTH; index = index + 1) begin
          entry = ref_entries[index];
          if (entry.valid && entry.uop.decoded.writes_rd && entry.uop.pdst == physical) used = 1;
        end
        if (!used) unused_physical = physical;
      end
    end
  endfunction

  function automatic renamed_uop_t writer(input integer id, rd, physical);
    renamed_uop_t value;
    begin
      value = '0;
      value.decoded.fetch.pc = 32'h100 + id * 4;
      value.decoded.fetch.instruction = 32'h00208033 | (rd << 7);
      value.decoded.fetch.prediction.next_pc = value.decoded.fetch.pc + 4;
      value.decoded.op_class = UOP_INTEGER;
      value.decoded.alu_op = ALU_ADD;
      value.decoded.writes_rd = rd != 0;
      value.decoded.rd = rd;
      value.pdst = rd != 0 ? physical : 0;
      value.old_pdst = rd;
      writer = value;
    end
  endfunction

  function automatic completion_t result_for(input renamed_uop_t uop, input word_t value);
    completion_t packet;
    begin
      packet = '0;
      packet.rob_tag = uop.rob_tag;
      packet.pdst = uop.pdst;
      packet.writes_rd = uop.decoded.writes_rd;
      packet.value = value;
      packet.control_flow.next_pc = uop.decoded.fetch.pc + 4;
      result_for = packet;
    end
  endfunction

  function automatic retire_trace_t architectural_event(input rob_entry_t entry);
    retire_trace_t value;
    begin
      value = '0;
      value.valid = 1;
      value.pc = entry.uop.decoded.fetch.pc;
      value.instruction = entry.uop.decoded.fetch.instruction;
      if (entry.exception.valid) begin
        value.trap = 1;
        value.cause = entry.exception.cause;
        value.tval = entry.exception.tval;
        value.next_pc = trap_target;
      end else begin
        value.next_pc = entry.control_flow.next_pc;
        if (entry.uop.decoded.writes_rd && entry.uop.decoded.rd != 0) begin
          value.rd = entry.uop.decoded.rd;
          value.rd_value = entry.result;
        end
        if (entry.memory_access.valid) begin
          value.memory_write = entry.memory_access.write;
          value.memory_address = entry.memory_access.address;
          value.memory_wdata = entry.memory_access.wdata;
          value.memory_wstrb = entry.memory_access.wstrb;
        end
      end
      architectural_event = value;
    end
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 30)
          $display("FAIL: %s time=%0t occupancy=%0d", label_text, $time, occupancy);
      end
    end
  endtask

  task automatic neutral;
    begin
      dispatch_hold = 0; dispatch_valid = 0; dispatch_uop = '0;
      completion_valid = 0; completion = '0; head_ready = 0;
      flush = 0; query_tag = '0; trap_target = 32'h800;
      head_override = 0; injected_event = '0;
    end
  endtask

  task automatic offer(input renamed_uop_t older, younger, input logic [1:0] mask);
    begin
      dispatch_uop.lane0 = older;
      dispatch_uop.lane1 = younger;
      dispatch_uop.lane0.rob_tag = reference_tag(ref_tail);
      dispatch_uop.lane1.rob_tag = reference_tag((ref_tail + 1) % ROB_DEPTH);
      dispatch_valid = mask;
    end
  endtask

  task automatic reference_pending;
    integer offset;
    rob_entry_t entry;
    begin
      expected_mispredict = 0; expected_serial = 0; expected_fault = 0;
      expected_mispredict_tag = '0; expected_mispredict_pc = 0;
      for (offset = 0; offset < ref_count; offset = offset + 1) begin
        entry = logical_entry(offset);
        if (entry.uop.decoded.op_class == UOP_SYSTEM) expected_serial = 1;
        if (entry.exception.valid) expected_fault = 1;
        if (!expected_mispredict && entry.complete && !entry.exception.valid &&
            entry.control_flow.mispredicted) begin
          expected_mispredict = 1;
          expected_mispredict_tag = entry.uop.rob_tag;
          expected_mispredict_pc = entry.control_flow.next_pc;
        end
      end
    end
  endtask

  task automatic evaluate;
    rob_entry_t first, second;
    completion_t c0, c1;
    renamed_uop_t u0, u1;
    logic allowed, ready0, ready1;
    logic serial0, serial1;
    retire_trace_pair_t source;
    begin
      first = logical_entry(0); second = logical_entry(1);
      c0 = completion.lane0; c1 = completion.lane1;
      u0 = dispatch_uop.lane0; u1 = dispatch_uop.lane1;
      reference_pending;
      allowed = rst_n && !flush && !ref_recovery && !dispatch_hold &&
                !expected_mispredict && !expected_serial && !expected_fault;
      ready0 = allowed && ref_count < ROB_DEPTH && u0.rob_tag == reference_tag(ref_tail);
      expected_push[0] = dispatch_valid[0] && ready0;
      ready1 = expected_push[0] && ref_count < ROB_DEPTH-1 &&
               u1.rob_tag == reference_tag((ref_tail + 1) % ROB_DEPTH) &&
               !u0.decoded.fetch.exception.valid && u0.decoded.op_class != UOP_SYSTEM;
      expected_push[1] = dispatch_valid[1] && ready1;
      check(dispatch_tag.lane0 == reference_tag(ref_tail) &&
            dispatch_tag.lane1 == reference_tag((ref_tail + 1) % ROB_DEPTH),
            "candidate identities");
      check(dispatch_ready == {ready1, ready0} && dispatch_fire == expected_push,
            "dispatch accepts only the legal capacity-limited prefix");
      expected_accept[0] = completion_valid[0] && rst_n && !flush && find_owner(c0) >= 0;
      expected_accept[1] = completion_valid[1] && rst_n && !flush && find_owner(c1) >= 0;
      expected_completion_conflict = (&expected_accept) &&
        (c0.rob_tag == c1.rob_tag || (c0.writes_rd && c1.writes_rd &&
         c0.pdst != 0 && c0.pdst == c1.pdst));
      if (expected_completion_conflict) expected_accept[1] = 0;
      expected_reject = completion_valid & {2{rst_n && !flush}} & ~expected_accept;
      check(completion_ready == (rst_n && !flush ? 2'b11 : 2'b00), "raw completion consumption");
      check(completion_accepted == expected_accept && completion_rejected == expected_reject,
            "only live matching unfinished owners accept results");
      expected_head_valid = 0;
      if (rst_n && !flush && !ref_recovery && ref_count > 0) begin
        expected_head_valid[0] = 1;
        expected_head_valid[1] = ref_count > 1;
      end
      expected_head_complete[0] = expected_head_valid[0] && first.complete;
      expected_head_complete[1] = expected_head_valid[1] && second.complete;
      check(head_valid == expected_head_valid && head_complete == expected_head_complete,
            "head visibility and independent completion state");
      check(head_entry.lane0 == (expected_head_valid[0] ? first : '0), "oldest record contents");
      check(head_entry.lane1 == (expected_head_valid[1] ? second : '0), "second record contents");
      serial0 = first.uop.decoded.op_class == UOP_SYSTEM;
      serial1 = second.uop.decoded.op_class == UOP_SYSTEM;
      expected_pop[0] = expected_head_complete[0] && head_ready[0];
      expected_pop[1] = expected_pop[0] && expected_head_complete[1] && head_ready[1] &&
                        !first.exception.valid && !first.control_flow.mispredicted &&
                        !serial0 && !serial1;
      expected_retire[0] = expected_pop[0] && !first.exception.valid;
      expected_retire[1] = expected_pop[1] && !second.exception.valid;
      expected_trap[0] = expected_pop[0] && first.exception.valid;
      expected_trap[1] = expected_pop[1] && second.exception.valid;
      check(retire_fire == expected_retire && trap_fire == expected_trap,
            "architectural events preserve the oldest prefix and precise fault boundary");
      expected_event = '0;
      if (expected_pop[0]) expected_event.lane0 = architectural_event(first);
      if (expected_pop[1]) expected_event.lane1 = architectural_event(second);
      check(rob_event == expected_event, "retirement event values and suppressed trap effects");
      expected_protocol = rst_n && (dispatch_valid == 2'b10 || head_ready == 2'b10 ||
                                   expected_completion_conflict);
      check(protocol_error == expected_protocol, "ROB protocol diagnostics");
      check(mispredict_pending == (rst_n && !flush && expected_mispredict) &&
            serial_pending == (rst_n && !flush && expected_serial) &&
            fault_pending == (rst_n && !flush && expected_fault), "pending control scan");
      if (rst_n && !flush && expected_mispredict)
        check(mispredict_tag == expected_mispredict_tag &&
              mispredict_next_pc == expected_mispredict_pc,
              "oldest resolved misprediction has priority");
      source = head_override ? injected_event : expected_event;
      expected_recorder_error = rst_n && !source.lane0.valid && source.lane1.valid;
      check(recorder_error == expected_recorder_error, "trace prefix protocol diagnostic");
      expected_trace = '0;
      expected_trace_count = 0;
      if (!expected_recorder_error && source.lane0.valid) begin
        expected_trace.lane0 = source.lane0;
        expected_trace.lane0.order = next_trace_order;
        expected_trace_count = 1;
        if (source.lane1.valid) begin
          expected_trace.lane1 = source.lane1;
          expected_trace.lane1.order = next_trace_order + 1;
          expected_trace_count = 2;
        end
      end
    end
  endtask

  task automatic update_reference;
    integer lane, owner, index, pops, pushes;
    completion_t packet;
    renamed_uop_t operation;
    rob_entry_t entry, old0, old1;
    begin
      old0 = logical_entry(0); old1 = logical_entry(1);
      if (flush) begin
        for (index = 0; index < ROB_DEPTH; index = index + 1) ref_entries[index] = '0;
        ref_head = 0; ref_tail = 0; ref_count = 0; ref_recovery = 0;
      end else begin
        for (lane = 0; lane < 2; lane = lane + 1) begin
          packet = lane == 0 ? completion.lane0 : completion.lane1;
          if (expected_accept[lane]) begin
            owner = find_owner(packet);
            entry = ref_entries[owner];
            entry.complete = 1;
            entry.result = packet.value;
            entry.exception = packet.exception;
            entry.control_flow = packet.control_flow;
            entry.memory_access = packet.memory_access;
            ref_entries[owner] = entry;
          end
        end
        pops = (expected_pop[0] ? 1 : 0) + (expected_pop[1] ? 1 : 0);
        if (expected_pop[0]) begin
          ref_entries[ref_head] = '0;
          if (old0.exception.valid || old0.control_flow.mispredicted ||
              old0.uop.decoded.system_op == SYS_MRET ||
              old0.uop.decoded.system_op == SYS_FENCE_I) ref_recovery = 1;
        end
        if (expected_pop[1]) begin
          ref_entries[(ref_head+1) % ROB_DEPTH] = '0;
          if (old1.exception.valid || old1.control_flow.mispredicted) ref_recovery = 1;
        end
        ref_head = (ref_head + pops) % ROB_DEPTH;
        pushes = (expected_push[0] ? 1 : 0) + (expected_push[1] ? 1 : 0);
        for (lane = 0; lane < pushes; lane = lane + 1) begin
          operation = lane == 0 ? dispatch_uop.lane0 : dispatch_uop.lane1;
          entry = '0;
          entry.valid = 1;
          entry.uop = operation;
          entry.exception = operation.decoded.fetch.exception;
          entry.complete = entry.exception.valid;
          entry.control_flow.next_pc = operation.decoded.fetch.pc + 4;
          ref_entries[(ref_tail + lane) % ROB_DEPTH] = entry;
          ref_generation[(ref_tail + lane) % ROB_DEPTH] =
            (ref_generation[(ref_tail + lane) % ROB_DEPTH] + 1) % 256;
        end
        ref_tail = (ref_tail + pushes) % ROB_DEPTH;
        ref_count = ref_count + pushes - pops;
      end
      next_trace_order = next_trace_order + expected_trace_count;
    end
  endtask

  task automatic check_state;
    integer index, age;
    rob_entry_t entry;
    begin
      check(occupancy == ref_count && full == (ref_count == ROB_DEPTH) && empty == (ref_count == 0),
            "occupancy and full/empty state");
      check(head_index == ref_head && recovery_required == ref_recovery, "head and recovery state");
      for (index = 0; index < ROB_DEPTH; index = index + 1) begin
        entry = ref_entries[index];
        query_tag = entry.valid ? entry.uop.rob_tag : reference_tag(index);
        #1;
        check(query_live == entry.valid, "live identity query");
        age = entry.valid ? (index - ref_head + ROB_DEPTH) % ROB_DEPTH : 0;
        check(query_age == age, "age relative to current head");
      end
    end
  endtask

  task automatic cycle;
    logic apply_flush;
    begin
      #1; evaluate;
      if (expected_push[0]) last0 = dispatch_uop.lane0;
      if (expected_push[1]) last1 = dispatch_uop.lane1;
      apply_flush = flush;
      update_reference;
      #2; clk = 1; #1;
      check(trace == expected_trace,
            "registered trace has accepted fields and monotonic event order");
      check(flush_done == apply_flush, "safe flush acknowledgement follows the apply edge");
      clk = 0; neutral; #1; check_state;
    end
  endtask

  task automatic reset_dut;
    integer index;
    begin
      clk = 0; neutral; rst_n = 0;
      ref_head = 0; ref_tail = 0; ref_count = 0; ref_recovery = 0; next_trace_order = 0;
      for (index = 0; index < ROB_DEPTH; index = index + 1) begin
        ref_entries[index] = '0; ref_generation[index] = 0;
      end
      #2;
      check(trace == '0 && !recovery_required && !flush_done,
            "global reset clears control and trace");
      rst_n = 1; #1; check_state;
    end
  endtask

  task automatic flush_dut;
    begin neutral; flush = 1; cycle; neutral; cycle; end
  endtask

  task automatic queue_pair(input renamed_uop_t older, younger);
    begin neutral; offer(older, younger, 2'b11); cycle; end
  endtask

  task automatic queue_single(input renamed_uop_t operation);
    begin neutral; offer(operation, '0, 2'b01); cycle; end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block05/block05.vcd");
      $dumpvars(0, tb_block05);
    end
    #2000000;
    $fatal(1, "Block 5 timed out");
  end

  initial begin
    reset_dut;

    // Lead example: controlled DIV completion is late; the younger ADD is already finished.
    a_uop = writer(0, 5, 32); a_uop.decoded.op_class = UOP_MULDIV;
    a_uop.decoded.muldiv_op = MD_DIV;
    b_uop = writer(1, 6, 33);
    queue_pair(a_uop, b_uop); a_uop = last0; b_uop = last1;
    completion_valid = 2'b10; completion.lane1 = result_for(b_uop, 19); cycle;
    check(head_complete == 2'b10 && occupancy == 2, "younger ADD completes before older DIV");
    head_ready = 2'b11; #1;
    check(retire_fire == 0 && trap_fire == 0, "completed younger instruction cannot skip DIV");
    cycle;
    completion_valid = 2'b01; completion.lane0 = result_for(a_uop, 6); head_ready = 2'b11;
    #1; check(retire_fire == 0, "no same-edge completion-to-retirement bypass"); cycle;
    head_ready = 2'b11; #1;
    check(retire_fire == 2'b11, "both completed instructions retire in order");
    cycle;
    check(trace.lane0.order == 0 && trace.lane1.order == 1 &&
          trace.lane0.rd_value == 6 && trace.lane1.rd_value == 19,
          "DIV then ADD appear as ordered architectural trace events");
    check(empty, "dual retirement empties the two-instruction ROB");
    cycle;
    check(trace == '0, "idle trace contains no accidental repeat event");

    // Same architectural destination still has two independent physical versions/results.
    a_uop = writer(2, 5, 34); b_uop = writer(3, 5, 35); b_uop.old_pdst = 34;
    queue_pair(a_uop, b_uop); a_uop = last0; b_uop = last1;
    completion_valid = 2'b11;
    completion.lane0 = result_for(b_uop, 200); completion.lane1 = result_for(a_uop, 100); cycle;
    head_ready = 2'b11; cycle;
    check(trace.lane0.order == 2 && trace.lane1.order == 3 &&
          trace.lane0.rd == 5 && trace.lane1.rd == 5 &&
          trace.lane0.rd_value == 100 && trace.lane1.rd_value == 200,
          "WAW versions retire in instruction order rather than completion-lane order");

    // Mismatched and duplicate packets are consumed without changing live results.
    reset_dut;
    queue_pair(writer(0, 5, 32), writer(1, 6, 33)); a_uop = last0; b_uop = last1;
    completion_valid = 2'b01; completion.lane0 = result_for(a_uop, 11);
    completion.lane0.rob_tag.generation = a_uop.rob_tag.generation + 1;
    #1; check(completion_rejected == 1, "wrong generation is rejected"); cycle;
    completion_valid = 1; completion.lane0 = result_for(a_uop, 11); completion.lane0.pdst = 50;
    cycle;
    completion_valid = 1; completion.lane0 = result_for(a_uop, 11); completion.lane0.writes_rd = 0;
    cycle;
    completion_valid = 2'b11; completion.lane0 = result_for(a_uop, 11);
    completion.lane1 = result_for(a_uop, 99); #1;
    check(completion_accepted == 1 && completion_rejected == 2 && protocol_error,
          "duplicate completion owner gives lane 0 priority"); cycle;
    completion_valid = 1; completion.lane0 = result_for(a_uop, 77); cycle;
    completion_valid = 2'b10; completion.lane1 = result_for(b_uop, 22); cycle;
    head_ready = 2'b11; cycle;
    check(trace.lane0.rd_value == 11 && trace.lane1.rd_value == 22,
          "rejected duplicates do not overwrite completed results");
    completion_valid = 1; completion.lane0 = result_for(a_uop, 88); cycle;
    check(empty, "completion after retirement cannot resurrect an owner");

    // Deliberately violate rename uniqueness to check the ROB's second-write protection.
    reset_dut;
    queue_pair(writer(0, 5, 32), writer(1, 6, 32)); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 11);
    completion.lane1 = result_for(b_uop, 22); #1;
    check(completion_accepted == 1 && completion_rejected == 2 && protocol_error,
          "distinct owners cannot publish two physical writes to the same destination");
    cycle; flush_dut;

    // Explicit protocol violations cannot create a younger-only architectural transfer.
    offer(writer(2, 7, 34), writer(3, 8, 35), 2'b10); #1;
    check(dispatch_fire == 0 && protocol_error, "younger-only dispatch is rejected"); cycle;
    offer(writer(2, 7, 34), '0, 2'b01); dispatch_uop.lane0.rob_tag.generation = 99;
    #1; check(!dispatch_ready[0], "dispatch identity must match the offered ROB slot"); cycle;
    queue_pair(writer(2, 7, 34), writer(3, 8, 35)); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 7);
    completion.lane1 = result_for(b_uop, 8); cycle;
    head_ready = 2'b10; #1;
    check(retire_fire == 0 && protocol_error, "head ready cannot skip lane 0");
    cycle;
    dispatch_hold = 1; offer(writer(4, 9, 36), '0, 1); #1;
    check(dispatch_fire == 0, "external dispatch hold does not consume a ROB slot");
    head_ready = 1; cycle;
    check(occupancy == 1, "one-ready mask retires only the older instruction");
    head_ready = 3; cycle;

    // Capacity and generation state at ring wrap; no same-edge reuse when currently full.
    reset_dut;
    for (n = 0; n < 8; n = n + 1) begin
      queue_pair(writer(n*2, 1+n*2, 32+n*2), writer(n*2+1, 2+n*2, 33+n*2));
      saved[n*2] = last0; saved[n*2+1] = last1;
    end
    check(full && occupancy == 16, "sixteen live instructions fill the ROB");
    completion_valid = 3; completion.lane0 = result_for(saved[0], 1);
    completion.lane1 = result_for(saved[1], 2); cycle;
    offer(writer(16, 20, 48), writer(17, 21, 49), 3); head_ready = 3;
    #1; check(dispatch_fire == 0 && retire_fire == 3, "full ROB does not borrow retiring slots");
    cycle;
    check(occupancy == 14, "two retired entries release capacity for the following cycle");
    queue_pair(writer(16, 20, 48), writer(17, 21, 49)); c_uop = last0;
    temp_uop = saved[0];
    check(c_uop.rob_tag.index == 0 && c_uop.rob_tag.generation != temp_uop.rob_tag.generation,
          "slot reuse advances its allocation generation");
    completion_valid = 1; completion.lane0 = result_for(saved[0], 123); cycle;
    check(full, "late old-generation result does not consume or change a new owner");
    flush_dut;
    queue_single(writer(18, 22, 50)); a_uop = last0;
    check(a_uop.rob_tag.generation > c_uop.rob_tag.generation,
          "safe flush retains per-slot generation history");
    completion_valid = 1; completion.lane0 = result_for(c_uop, 123); cycle;
    flush_dut;

    // Fifteen occupied entries allow an older-only dispatch prefix and later compaction.
    reset_dut;
    for (n = 0; n < 7; n = n + 1) begin
      queue_pair(writer(n*2, 1+n*2, 32+n*2), writer(n*2+1, 2+n*2, 33+n*2));
      saved[n*2] = last0; saved[n*2+1] = last1;
    end
    queue_single(writer(14, 15, 46));
    offer(writer(15, 16, 47), writer(16, 17, 48), 3); #1;
    check(dispatch_fire == 1, "one free entry accepts only the older dispatch lane"); cycle;
    completion_valid = 1; completion.lane0 = result_for(saved[0], 1); cycle;
    head_ready = 1; cycle;
    queue_single(writer(16, 17, 48));
    check(last0.decoded.fetch.pc == 32'h140, "stalled suffix can be compacted and accepted once");
    flush_dut;

    // Three independent update paths in the same clock edge.
    queue_pair(writer(0, 5, 32), writer(1, 6, 33)); a_uop = last0; b_uop = last1;
    queue_pair(writer(2, 7, 34), writer(3, 8, 35)); c_uop = last0; d_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 1);
    completion.lane1 = result_for(b_uop, 2); cycle;
    offer(writer(4, 9, 36), writer(5, 10, 37), 3);
    completion_valid = 3; completion.lane0 = result_for(c_uop, 3);
    completion.lane1 = result_for(d_uop, 4); head_ready = 3; cycle;
    check(occupancy == 4 && trace.lane0.rd_value == 1 && trace.lane1.rd_value == 2,
          "two pushes, completions, and retirements coexist without slot conflicts");
    flush_dut;

    // A fault at the oldest instruction suppresses every younger architectural effect.
    reset_dut;
    queue_pair(writer(0, 5, 32), writer(1, 6, 33)); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 77);
    completion.lane0.exception.valid = 1; completion.lane0.exception.cause = CAUSE_LOAD_ACCESS;
    completion.lane0.exception.tval = 32'hbad00000;
    completion.lane0.memory_access.valid = 1; completion.lane0.memory_access.write = 1;
    completion.lane1 = result_for(b_uop, 88); cycle;
    check(fault_pending && completion_accepted == 0,
          "registered fault is pending after acceptance");
    head_ready = 3; #1;
    check(trap_fire == 1 && retire_fire == 0, "oldest fault prevents younger retirement"); cycle;
    check(trace.lane0.trap && trace.lane0.rd == 0 && !trace.lane0.memory_write &&
          trace.lane0.next_pc == 32'h800 && !trace.lane1.valid,
          "fault trace reports target and suppresses GPR/store effects");
    head_ready = 3; offer(writer(2, 7, 34), '0, 1); #1;
    check(head_valid == 0 && dispatch_fire == 0 && recovery_required,
          "held readiness cannot retire younger work before safe recovery"); cycle;
    flush_dut;

    // The older success can retire before a second-slot precise fault in the same edge.
    reset_dut;
    queue_pair(writer(0, 5, 32), writer(1, 6, 33)); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 42);
    completion.lane1 = result_for(b_uop, 99); completion.lane1.exception.valid = 1;
    completion.lane1.exception.cause = CAUSE_ILLEGAL_INSTR;
    completion.lane1.exception.tval = b_uop.decoded.fetch.instruction; cycle;
    head_ready = 3; #1;
    check(retire_fire == 1 && trap_fire == 2, "second-slot fault preserves older retirement");
    cycle;
    check(trace.lane0.rd_value == 42 && !trace.lane0.trap && trace.lane1.trap &&
          trace.lane1.order == trace.lane0.order+1,
          "retirement then trap have distinct ordered events");
    flush_dut;

    // A valid decode fault needs no execution completion, and stops the pair at that instruction.
    reset_dut;
    a_uop = writer(0, 0, 0); a_uop.decoded.op_class = UOP_NONE;
    a_uop.decoded.fetch.exception.valid = 1;
    a_uop.decoded.fetch.exception.cause = CAUSE_ILLEGAL_INSTR;
    a_uop.decoded.fetch.exception.tval = 32'hffffffff;
    offer(a_uop, writer(1, 5, 32), 3); #1;
    check(dispatch_fire == 1, "decode fault truncates a dispatch pair"); cycle;
    check(head_complete == 1 && fault_pending, "decode fault is complete-for-trap, not a bubble");
    head_ready = 3; cycle; check(trace.lane0.trap, "decode fault reaches architectural trap event");
    flush_dut;

    // An older late-resolving misprediction overrides a younger pending branch.
    reset_dut;
    a_uop = writer(0, 0, 0); a_uop.decoded.op_class = UOP_BRANCH;
    b_uop = writer(1, 0, 0); b_uop.decoded.op_class = UOP_BRANCH;
    queue_pair(a_uop, b_uop); a_uop = last0; b_uop = last1;
    queue_single(writer(2, 5, 32)); c_uop = last0;
    completion_valid = 2; completion.lane1 = result_for(b_uop, 0);
    completion.lane1.control_flow.mispredicted = 1;
    completion.lane1.control_flow.next_pc = 32'h500; cycle;
    check(mispredict_tag == b_uop.rob_tag, "younger resolved misprediction is initially pending");
    offer(writer(3, 6, 33), '0, 1); #1;
    check(dispatch_fire == 0, "known misprediction freezes allocation");
    completion_valid = 1; completion.lane0 = result_for(a_uop, 0);
    completion.lane0.control_flow.mispredicted = 1;
    completion.lane0.control_flow.next_pc = 32'h400; cycle;
    check(mispredict_tag == a_uop.rob_tag && mispredict_next_pc == 32'h400,
          "older branch takes recovery priority when it resolves later");
    head_ready = 3; #1; check(retire_fire == 1, "mispredicted first slot prevents younger commit");
    cycle;
    completion_valid = 1; completion.lane0 = result_for(c_uop, 7); #1;
    check(completion_accepted == 1 && head_valid == 0,
          "held results may drain while architectural retirement is recovery-blocked"); cycle;
    flush_dut;
    completion_valid = 1; completion.lane0 = result_for(c_uop, 7); cycle;

    // System instructions serialize dispatch and retire alone; MRET/FENCE.I require front flush.
    reset_dut;
    a_uop = writer(0, 0, 0); a_uop.decoded.op_class = UOP_SYSTEM;
    a_uop.decoded.system_op = SYS_FENCE;
    offer(a_uop, writer(1, 5, 32), 3); #1;
    check(dispatch_fire == 1, "system instruction in lane 0 truncates dispatch"); cycle;
    a_uop = last0;
    offer(writer(1, 5, 32), '0, 1); #1;
    check(serial_pending && dispatch_fire == 0, "queued system instruction serializes admission");
    completion_valid = 1; completion.lane0 = result_for(a_uop, 0); cycle;
    head_ready = 3; cycle;
    check(!recovery_required, "ordinary FENCE retires without forced recovery");
    queue_pair(writer(2, 5, 32), a_uop); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 10);
    completion.lane1 = result_for(b_uop, 0); cycle;
    head_ready = 3; #1;
    check(retire_fire == 1, "a system instruction cannot retire in slot 1"); cycle;
    head_ready = 3; cycle;
    for (n = 0; n < 2; n = n + 1) begin
      a_uop = writer(4+n, 0, 0); a_uop.decoded.op_class = UOP_SYSTEM;
      if (n == 0) a_uop.decoded.system_op = SYS_FENCE_I;
      else a_uop.decoded.system_op = SYS_MRET;
      queue_single(a_uop); a_uop = last0;
      completion_valid = 1; completion.lane0 = result_for(a_uop, 0); cycle;
      head_ready = 3; cycle;
      check(recovery_required, "FENCE.I/MRET retirement requires a safe front-end flush");
      flush_dut;
    end

    // Successful stores trace the acknowledged access; fault events suppress that effect.
    reset_dut;
    a_uop = writer(0, 0, 0); a_uop.decoded.op_class = UOP_STORE;
    queue_single(a_uop); a_uop = last0;
    completion_valid = 1; completion.lane0 = result_for(a_uop, 0);
    completion.lane0.memory_access.valid = 1; completion.lane0.memory_access.write = 1;
    completion.lane0.memory_access.address = 32'h100;
    completion.lane0.memory_access.wdata = 32'h00ab0000;
    completion.lane0.memory_access.wstrb = 4'b0100; cycle;
    head_ready = 3; cycle;
    check(trace.lane0.memory_write && trace.lane0.memory_address == 32'h100 &&
          trace.lane0.memory_wdata == 32'h00ab0000 && trace.lane0.memory_wstrb == 4'b0100,
          "store trace preserves byte-address, shifted data, and byte strobes");
    a_uop = writer(1, 0, 0); a_uop.decoded.op_class = UOP_STORE;
    queue_single(a_uop); a_uop = last0;
    completion_valid = 1; completion.lane0 = result_for(a_uop, 0);
    completion.lane0.exception.valid = 1; completion.lane0.exception.cause = CAUSE_STORE_ACCESS;
    completion.lane0.exception.tval = 32'h100;
    completion.lane0.memory_access.valid = 1; completion.lane0.memory_access.write = 1; cycle;
    head_ready = 3; cycle;
    check(trace.lane0.trap && !trace.lane0.memory_write && trace.lane0.memory_wstrb == 0,
          "faulted store produces a trap, not a successful-store trace");
    flush_dut;

    // Safe flush dominates traffic and does not reset architectural event ordering.
    queue_pair(writer(2, 5, 32), writer(3, 6, 33)); a_uop = last0; b_uop = last1;
    completion_valid = 3; completion.lane0 = result_for(a_uop, 3);
    completion.lane1 = result_for(b_uop, 4); cycle;
    offer(writer(4, 7, 34), writer(5, 8, 35), 3); completion_valid = 3;
    completion.lane0 = result_for(a_uop, 8); completion.lane1 = result_for(b_uop, 9);
    head_ready = 3; flush = 1; #1;
    check(dispatch_fire == 0 && completion_ready == 0 && retire_fire == 0 && trap_fire == 0,
          "safe flush dominates every instruction handshake"); cycle;
    check(empty && !recovery_required, "safe apply empties all speculative state");
    head_override = 1; injected_event.lane1.valid = 1; #1;
    check(recorder_error, "trace recorder rejects a younger-only event prefix"); cycle;
    head_override = 1; injected_event.lane0.valid = 1;
    injected_event.lane0.trap = 1; injected_event.lane0.interrupt = 1;
    injected_event.lane0.pc = 32'h44; injected_event.lane0.next_pc = 32'h800;
    injected_event.lane0.cause = CAUSE_IRQ_EXTERNAL;
    injected_event.lane0.order = 64'hffffffff; cycle;
    check(trace.lane0.interrupt && trace.lane0.cause == CAUSE_IRQ_EXTERNAL,
          "recorder can stamp an externally selected IRQ-format event without an IRQ controller");

    // Mixed traffic uses the independent live-record model; no ISA execution is being inferred.
    reset_dut; serial_id = 0;
    for (n = 0; n < 256; n = n + 1) begin
      neutral;
      rng = next_random(rng);
      if (ref_count != 0 && rng[0]) begin
        selected = (rng >> 4) % ref_count;
        temp_entry = logical_entry(selected);
        completion_valid[0] = 1;
        completion.lane0 = result_for(temp_entry.uop, rng);
      end
      rng = next_random(rng);
      if (ref_count != 0 && rng[0]) begin
        selected = (rng >> 4) % ref_count;
        temp_entry = logical_entry(selected);
        completion_valid[1] = 1;
        completion.lane1 = result_for(temp_entry.uop, rng);
      end
      if (n % 4 == 0) head_ready = 0;
      else if (n % 4 == 1) head_ready = 1;
      else head_ready = 3;
      rng = next_random(rng);
      if (rng[2:0] != 0) begin
        a_uop = writer(serial_id, 1 + serial_id % 31, unused_physical(0));
        serial_id = serial_id + 1;
        b_uop = writer(serial_id, 1 + serial_id % 31, unused_physical(a_uop.pdst));
        serial_id = serial_id + 1;
        offer(a_uop, b_uop, n % 3 == 0 ? 2'b01 : 2'b11);
      end
      dispatch_hold = n % 17 == 0;
      if (n % 71 == 70) flush = 1;
      cycle;
    end

    // Reset wins over held requests and clears the event-order sequence.
    neutral; offer(writer(900, 5, 32), writer(901, 6, 33), 3);
    head_ready = 3; flush = 1; rst_n = 0; #2;
    check(dispatch_fire == 0 && completion_ready == 0 && retire_fire == 0 && trace == '0,
          "asynchronous global reset dominates active requests");

    $display("BLOCK05 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Block 5 ROB/retirement/trace tests failed");
    $finish;
  end
endmodule
```

## Complete connected control/storage test

Source: `tb/tb_block05_integration.sv`.

```systemverilog
// Couple the built control/storage blocks; results are injected, not executed by a CPU.
`timescale 1ns / 1ps
module tb_block05_integration;
  import rv32_ooo_pkg::*;
  logic clk = 0, rst_n = 1;
  logic [1:0] front_valid, external_resources;
  decoded_pair_t decoded;
  renamed_pair_t renamed, retiring, saved;
  rob_tag_pair_t tags;
  logic [1:0] rename_ready, rename_fire, rob_ready, rob_fire, source1_ready, source2_ready;
  logic [1:0] wb_valid, wb_ready, wb_accepted, wb_rejected, head_valid, head_complete;
  logic [1:0] retire_fire, trap_fire, write_enable;
  completion_pair_t wb;
  rob_entry_pair_t heads;
  retire_trace_pair_t events, trace;
  logic apply, rename_done, rob_done, rename_error, rob_error, trace_error, write_error;
  logic recovery, mispredict, serial_pending, fault_pending, query_live, full, empty;
  rob_tag_t mispredict_tag;
  word_t mispredict_pc;
  rob_index_t query_age, head_index;
  rob_count_t occupancy;
  phys_mask_t free_mask, ready_mask;
  phys_count_t free_count;
  arch_reg_t committed_query;
  phys_reg_t committed_phys;
  word_t value0, value1, value2, value3;
  phys_reg_t read0, read1, read2, read3;
  integer checks = 0, failures = 0;

  rv32_rename rename (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_valid_i(front_valid),
    .dispatch_resources_ready_i(rob_ready & external_resources), .decoded_i(decoded),
    .dispatch_rob_tag_i(tags), .dispatch_ready_o(rename_ready), .dispatch_fire_o(rename_fire),
    .renamed_o(renamed), .src1_ready_o(source1_ready), .src2_ready_o(source2_ready),
    .completion_valid_i(wb_accepted), .completion_i(wb),
    .retire_valid_i(retire_fire), .retire_uop_i(retiring),
    .recover_i(apply), .recover_done_o(rename_done),
    .committed_query_arch_i(committed_query), .committed_query_phys_o(committed_phys),
    .free_mask_o(free_mask), .ready_mask_o(ready_mask), .free_count_o(free_count),
    .protocol_error_o(rename_error)
  );
  assign retiring.lane0 = heads.lane0.uop;
  assign retiring.lane1 = heads.lane1.uop;

  rv32_reorder_buffer rob (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_hold_i(1'b0),
    .dispatch_valid_i(rename_fire), .dispatch_uop_i(renamed),
    .dispatch_ready_o(rob_ready), .dispatch_fire_o(rob_fire), .dispatch_tag_o(tags),
    .completion_valid_i(wb_valid), .completion_i(wb), .completion_ready_o(wb_ready),
    .completion_accepted_o(wb_accepted), .completion_rejected_o(wb_rejected),
    .head_valid_o(head_valid), .head_complete_o(head_complete), .head_entry_o(heads),
    .head_ready_i(2'b11), .head_trap_target_i(32'h100),
    .retire_fire_o(retire_fire), .trap_fire_o(trap_fire), .retirement_event_o(events),
    .recovery_required_o(recovery), .mispredict_pending_o(mispredict),
    .mispredict_tag_o(mispredict_tag), .mispredict_next_pc_o(mispredict_pc),
    .serial_pending_o(serial_pending), .fault_pending_o(fault_pending),
    .flush_i(apply), .flush_done_o(rob_done), .query_tag_i(wb.lane0.rob_tag),
    .query_live_o(query_live), .query_age_o(query_age), .head_index_o(head_index),
    .occupancy_o(occupancy), .full_o(full), .empty_o(empty), .protocol_error_o(rob_error)
  );
  assign write_enable[0] = wb_accepted[0] && wb.lane0.writes_rd && !wb.lane0.exception.valid;
  assign write_enable[1] = wb_accepted[1] && wb.lane1.writes_rd && !wb.lane1.exception.valid;
  rv32_physical_regfile prf (
    .clk_i(clk), .rst_ni(rst_n),
    .read0_rs1_i(read0), .read0_rs2_i(read1), .read1_rs1_i(read2), .read1_rs2_i(read3),
    .read0_data1_o(value0), .read0_data2_o(value1),
    .read1_data1_o(value2), .read1_data2_o(value3),
    .write_enable_i(write_enable), .write0_addr_i(wb.lane0.pdst),
    .write1_addr_i(wb.lane1.pdst), .write0_data_i(wb.lane0.value),
    .write1_data_i(wb.lane1.value), .write_conflict_o(write_error)
  );
  rv32_trace_recorder recorder (
    .clk_i(clk), .rst_ni(rst_n), .event_i(events), .trace_o(trace),
    .protocol_error_o(trace_error)
  );

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        $display("FAIL: %s time=%0t", label_text, $time);
      end
    end
  endtask
  task automatic edge_step;
    begin
      #1;
      check(rename_fire == rob_fire, "atomic dispatch agrees between rename and ROB");
      check(!rename_error && !rob_error && !trace_error && !write_error,
            "legal connected traffic has no protocol violations");
      #1; clk = 1; #1; clk = 0; #1;
    end
  endtask
  function automatic decoded_uop_t operation(input integer pc, rd, rs1, rs2);
    decoded_uop_t result;
    begin
      result = '0;
      result.fetch.pc = pc;
      result.fetch.instruction = (rs2 << 20) | (rs1 << 15) | (rd << 7) | 7'h33;
      result.fetch.prediction.next_pc = pc + 4;
      result.op_class = UOP_INTEGER;
      result.alu_op = ALU_ADD;
      result.rd = rd;
      result.rs1 = rs1;
      result.rs2 = rs2;
      result.uses_rs1 = 1;
      result.uses_rs2 = 1;
      result.writes_rd = rd != 0;
      operation = result;
    end
  endfunction
  function automatic completion_t completed(input renamed_uop_t instruction, input word_t value);
    completion_t result;
    begin
      result = '0;
      result.rob_tag = instruction.rob_tag;
      result.pdst = instruction.pdst;
      result.writes_rd = instruction.decoded.writes_rd;
      result.value = value;
      result.control_flow.next_pc = instruction.decoded.fetch.pc + 4;
      completed = result;
    end
  endfunction

  initial begin #10000; $fatal(1, "Connected control test timed out"); end
  initial begin
    front_valid = 0; external_resources = 2'b11; decoded = '0;
    wb_valid = 0; wb = '0; apply = 0; committed_query = 5;
    read0 = 32; read1 = 33; read2 = 32; read3 = 33;
    #1; rst_n = 0; #2; rst_n = 1; #1;
    check(empty && free_count == 32 && committed_phys == 5, "coupled startup state");
    decoded.lane0 = operation(0, 5, 1, 2);
    decoded.lane0.op_class = UOP_MULDIV;
    decoded.lane0.muldiv_op = MD_DIV;
    decoded.lane0.fetch.instruction = 32'h0220c2b3;
    decoded.lane1 = operation(4, 6, 3, 4);
    front_valid = 2'b11; #1; saved = renamed;
    check(rename_fire == 2'b11 && saved.lane0.pdst == 32 && saved.lane1.pdst == 33,
          "two distinct physical destinations dispatch in one edge");
    edge_step; front_valid = 0; #1;
    check(occupancy == 2 && free_count == 30 && head_complete == 0,
          "both instructions are tracked without premature retirement");

    wb.lane0 = completed(saved.lane1, 19); wb_valid = 2'b01; #1;
    check(wb_accepted == 2'b01 && value1 == 19 && value3 == 19,
          "younger accepted result bypasses to both instruction lanes");
    edge_step; wb_valid = 0; #1;
    check(ready_mask[33] && !ready_mask[32] && retire_fire == 0,
          "younger result ready while older DIV blocks commitment");
    committed_query = 6; #1;
    check(committed_phys == 6 && value1 == 19, "speculative data does not change committed map");

    wb.lane0 = completed(saved.lane0, 6); wb_valid = 2'b01; #1;
    check(retire_fire == 0 && value0 == 6, "head completion does not bypass retirement state");
    edge_step; wb_valid = 0; #1;
    check(retire_fire == 2'b11, "completed older and younger instructions form retirement pair");
    edge_step;
    check(empty && free_count == 32, "dual retirement releases both replaced versions");
    check(trace.lane0.valid && trace.lane1.valid &&
          trace.lane0.order == 0 && trace.lane1.order == 1,
          "registered architectural trace preserves program order");
    check(trace.lane0.rd == 5 && trace.lane0.rd_value == 6 &&
          trace.lane1.rd == 6 && trace.lane1.rd_value == 19, "trace contains committed values");
    committed_query = 5; #1; check(committed_phys == 32, "x5 committed mapping advanced");
    committed_query = 6; #1; check(committed_phys == 33, "x6 committed mapping advanced");

    wb.lane1 = completed(saved.lane1, 999); wb_valid = 2'b10; #1;
    check(wb_rejected == 2'b10 && write_enable == 0 && value1 == 19,
          "retired-owner result is dropped before PRF bypass or write");
    edge_step; wb_valid = 0; #1;
    check(value1 == 19 && value3 == 19, "stale completion cannot corrupt stored value");

    apply = 1; edge_step; apply = 0; #1;
    check(empty && rename_done && rob_done && value0 == 6 && value1 == 19,
          "safe apply preserves committed data and clears ROB control");
    check(tags.lane0.generation == 1, "micro-flush retains the next allocation generation");

    // One backend slot permits only the older lane; the caller compacts the suffix next.
    decoded.lane0 = operation(8, 7, 5, 6);
    decoded.lane1 = operation(12, 8, 7, 6);
    front_valid = 2'b11; external_resources = 2'b01; #1;
    check(rename_fire == 2'b01 && rob_fire == 2'b01, "partial pair is accepted atomically");
    edge_step;
    decoded.lane0 = decoded.lane1; decoded.lane1 = '0;
    front_valid = 2'b01; external_resources = 2'b11; #1;
    check(renamed.lane0.prs1 == 5 && !source1_ready[0],
          "compacted suffix sees its older producer's new physical destination");
    edge_step; front_valid = 0;
    apply = 1; edge_step; apply = 0; #1;
    check(empty && free_count == 32 && value0 == 6 && value1 == 19,
          "cancelled unexecuted pair leaves committed values intact");

    $display("BLOCK05_INTEGRATION SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Connected two-wide control/storage test failed");
    $finish;
  end
endmodule
```
