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

### ROB and trace interface

These widths follow the active package: a ROB identity is 12 bits, comprising a four-bit slot
and an eight-bit generation. Paired payloads retain two complete records; their control masks
are separate wires. Lane 0 is older for allocation, head permission, retirement, and events.

| Signal group | Width/type | Source → destination | Meaning |
|---|---|---|---|
| clk_i / rst_ni | 1 each | Common clock/reset → ROB and recorder | Rising-edge state updates; asynchronous active-low reset |
| dispatch_hold_i | 1 | Admission control → ROB | Prevent new allocation while older work progresses |
| dispatch_valid_i / dispatch_ready_o / dispatch_fire_o | 2 each | Dispatcher ↔ ROB | Offered, eligible, and accepted ordered allocation prefix |
| dispatch_uop_i | 600; renamed_pair_t | Rename → ROB | Instructions, physical sources/destinations, previous mappings, and owner tags |
| dispatch_tag_o | 24; rob_tag_pair_t | ROB tail candidates → rename/dispatcher | Full identities proposed for the two possible allocations |
| completion_valid_i / completion_ready_o | 2 each | Validated writeback ↔ ROB | Completion packets offered and consumed |
| completion_i | 444; completion_pair_t | Writeback → ROB | Result/exception/control/memory metadata plus full ownership |
| completion_accepted_o / completion_rejected_o | 2 each | ROB → writeback acceptance/wakeup checks | Distinguish a usable live owner from a consumed stale/malformed packet |
| head_valid_o / head_complete_o | 2 each | ROB heads → retirement/LSU/system control | Occupied and completed state; younger completeness cannot bypass an older wait |
| head_entry_o | 1,010; rob_entry_pair_t | ROB → retirement/LSU/system/predictor | Two oldest records, including retained completion and exception state |
| head_ready_i | 2 | Retirement control → ROB | Permission to advance an eligible ordered head prefix |
| head_trap_target_i | 32; word_t | CSR trap selection → ROB | Actual trap next PC recorded in a fault event |
| retire_fire_o / trap_fire_o | 2 each | ROB → committed maps/counters or trap controller | Successful retirement versus precise synchronous exception |
| retirement_event_o | 666; retire_trace_pair_t | ROB → core event merger/recorder | Two 333-bit architectural event templates; order is stamped later |
| recovery_required_o | 1 | ROB recovery lock → core control | A retired recovery boundary blocks younger advancement until safe flush |
| mispredict_pending_o / mispredict_tag_o / mispredict_next_pc_o | 1 / 12 / 32 | Age-order ROB scan → scheduler/core control | Oldest known completed misprediction and actual restart PC |
| serial_pending_o / fault_pending_o | 1 each | ROB live-entry scan → admission control | A serialized operation or recorded fault restricts younger admission |
| flush_i / flush_done_o | 1 each | Recovery APPLY ↔ ROB | Safely discard speculative entries and acknowledge that edge |
| query_tag_i / query_live_o / query_age_o | 12 / 1 / 4 | WB/integration query ↔ ROB | Check a full live identity and its circular distance from head |
| head_index_o / occupancy_o | 4 / 5 | ROB state → integration/status | Oldest slot and occupied-entry count, from 0 through 16 |
| full_o / empty_o / protocol_error_o | 1 each | ROB → dispatch/control/diagnostics | Capacity boundary or an invalid ownership/prefix contract |
| Recorder event_i | 666; retire_trace_pair_t | ROB/core event merger → trace recorder | Accepted ordered normal/trap/interrupt templates |
| Recorder trace_o | 666; retire_trace_pair_t | Registered recorder → external debug/test | Architectural event pair stamped with 64-bit order values |
| Recorder protocol_error_o | 1 | Recorder → integration checks | An invalid younger-only event prefix was presented |

An accepted completion changes speculative ROB state at the edge. A retirement event describes
an eligible head advancing at an edge; the trace recorder then registers that event. Neither
the packed metadata width nor trace recording adds another execution or memory transaction.

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
