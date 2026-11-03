# Block 11: cancellation barrier and production recovery

The recovery controller converts a precise restart decision into safe resource reuse. It does
not decide which instruction is architecturally oldest; the ROB/core selects the fault,
retired misprediction, MRET, FENCE.I, or drained interrupt boundary and supplies its target.
The controller then cancels producers before applying queue/map restoration and fetch redirect.

The production controller is implemented and tested. Its standalone test passes 19,625 checks;
block 15 also exercises the same controller inside the actual processor.

## 1. Why flushing valid bits alone is insufficient

Suppose an older DIV is still running while younger instructions have already written physical
registers. A redirect makes that younger work invalid, but the divider or a held completion
could still produce a result later. If a flushed ROB slot or physical identity were reused too
early, the delayed result could corrupt a new instruction's state.

ROB generations reject stale ownership, but finite generations are not an unlimited substitute
for draining/cancellation. This design removes executing and buffered owners before making
recycled identities available again.

## 2. Connections to the core

```mermaid
flowchart LR
  DECIDE["ROB/core: precise restart event and target"] --> LATCH["Capture target in RUN"]
  LATCH --> CANCEL["CANCEL: stop admission/issue and cancel producers"]
  CANCEL --> EX["Integer / multiply-divide units"]
  CANCEL --> WB["Writeback result buffers"]
  CANCEL --> SYS["Serialized system unit"]
  EX --> BARRIER["All cancellation acknowledgments AND all units quiescent"]
  WB --> BARRIER
  SYS --> BARRIER
  MEM["LSU: no active transaction or held result"] --> BARRIER
  BARRIER --> APPLY["APPLY: flush ROB / IQ / memory queue, restore rename map"]
  APPLY --> IF["Fetch redirect; old reads become stale"]
  APPLY --> RUN["Return to RUN with done pulse"]
```

The complete core blocks new admission while recovery is active and blocks new issue through
its recovery controls. Previously known misprediction handling still allows older work to
reach the retirement boundary before requesting this barrier.

## 3. Signals and contracts

| Signal | Width | Source → destination | Meaning |
|---|---:|---|---|
| clk_i, rst_ni | 1 each | Common clock/reset → controller | Rising-edge FSM; active-low asynchronous reset |
| request_i | 1 | Core/ROB boundary → controller | One restart pulse offered while busy_o is zero |
| target_i | 32 | Core → target register | Trap vector, resolved next PC, MRET target, or fence restart PC |
| units_quiescent_i | 1 | Combined producer/LSU state → barrier | No executing owner or retained result remains to corrupt reuse |
| cancel_ack_i | 1 | Combined producer acknowledgments → barrier | Cancellation was applied by every participating producer |
| cancel_o | 1 | Controller → producers/control | Remains asserted throughout CANCEL and APPLY |
| apply_o | 1 | Controller → queues/maps/fetch | One full clock interval authorizing safe restoration/redirect |
| redirect_pc_o | 32 | Saved target → fetch | Stable captured destination, retained after completion |
| busy_o | 1 | FSM → admission/control | Recovery owns the machine outside RUN |
| done_o | 1 | FSM → control/status | One-cycle pulse after the APPLY edge returns to RUN |

This is a pulse request interface, not an arbitrary request queue. Requests while busy cannot
replace the saved target. The core already ensures that only the selected precise boundary
starts recovery; the controller need not merge several speculative redirect candidates itself.

## 4. Circuit and state-machine behavior

The RTL infers a small RUN/CANCEL/APPLY state register, a 32-bit target register, and a done
flip-flop. Output decoding exposes cancellation/apply state. There is no combinational path
that turns a new request directly into a flush.

| State before edge | Condition at edge | State afterward | Visible consequence |
|---|---|---|---|
| RUN | No request | RUN | No cancellation or apply |
| RUN | Restart request | CANCEL | Target captured; cancellation asserted |
| CANCEL | Only acknowledgment, only quiescence, or neither | CANCEL | Target and cancellation remain held |
| CANCEL | Acknowledgment and quiescence together | APPLY | Apply asserted while cancellation stays active |
| APPLY | Following clock edge | RUN | Queues/maps/fetch apply recovery; done pulses |

The two barrier inputs must be true together at a sampled edge. An acknowledgment in one cycle
followed by quiescence in another is not enough. Even if all producers were initially idle,
a request first enters CANCEL rather than bypassing the cancellation phase.

A typical delayed sequence is:

| Edge | Inputs/state event | Result |
|---|---|---|
| E0 | Request target T in RUN | Capture T, enter CANCEL |
| E1 | Producers quiet but an acknowledgment missing | Keep cancellation; do not apply |
| E2 | Acknowledgments present but a producer not quiet | Keep cancellation; do not apply |
| E3 | Both combined barrier conditions true | Enter APPLY, target still T |
| E4 | Apply edge | Restore/flush/restart, enter RUN, done=1 |
| E5 | No next request | done clears; target remains retained |

The cancellation duration is variable. Only the apply interval is exactly one controller
cycle. A new request can start on the first available RUN edge after completion.

## 5. What is cancelled and what is preserved

Execution cancellation clears multicycle arithmetic progress and every unit's held completion.
Writeback cancellation clears both retained result slots. CSR-unit cancellation clears pending
uncommitted system ownership; it does not erase architectural CSRs or counters.

The LSU reports quiescence only when its request/response/completion engine is idle. Unissued
queue entries can still exist then and are safe to flush because they have made no bus request.
An accepted store or read is allowed to finish; it is not withdrawn from AXI. Restricting actual
data requests to the ROB head prevents younger wrong-path transactions from escaping.

After the barrier, rename restores speculative mappings from committed mappings and rebuilds
free identities. PRF values belonging to committed mappings are preserved. ROB generations
are also preserved across a microarchitectural flush.

Instruction reads need different handling: they are read-only transactions with PC metadata.
Fetch clears buffered instructions at apply and marks any old held/outstanding read stale.
The old response may drain after controller completion; it cannot enter decode as current work.
Thus fetch redirect acknowledgment is not an assertion that every old read has disappeared.

## 6. Precise selection and integration

The ROB/core establishes ordering before invoking this controller. A younger known
misprediction cannot outrank an older fault that reaches the head. A mispredicted JAL/JALR can
commit its valid link-register effect before younger mappings are restored. FENCE.I and MRET
retire as serialized system boundaries and request a restart even if no prediction mismatch
is otherwise present.

Interrupt entry first stops admission, drains admitted work, and rechecks eligibility. Only
then does it update machine trap state and request recovery to mtvec. The controller does not
save a speculative fetch PC as mepc.

## 7. Tests and waveforms

```sh
python3 scripts/run_checks.py --block 11
python3 scripts/run_checks.py --block 11 --waves
```

reports/block11_results.json records 19,625 passing checks and tested source hashes. Directed
and 400 randomized recovery sequences cover both missing-barrier conditions, alternating
individual acknowledgments/quiescence, stable targets, busy requests, a single apply edge,
done pulses, immediate next recovery, idle changes, and reset during CANCEL or APPLY.

In build/block11/block11.vcd, begin with request/target, cancel, both barrier inputs, apply,
busy, done, and redirect_pc. Confirm that cancellation precedes apply, that a stalled barrier
never changes the target, and that cancel remains high during the actual apply edge.

The block-15 trap and arithmetic tests additionally exercise production recovery with running
DIV, completed younger results, wrong-path memory entries, competing branch resolutions,
precise exceptions, interrupts, MRET, and FENCE.I. Their source-hashed reports distinguish those
composition checks from this standalone FSM test. No exhaustive formal proof or official ACT4
certification is claimed.

## 8. Interview questions

**Why is acknowledgment alone insufficient?** A module can acknowledge a command before all
owned work or buffered output is gone. Reuse also requires actual quiescence.

**Why keep cancellation high in APPLY?** Producers must remain unable to publish old work on
the same edge that identities and queues are restored.

**Why not reset the physical register file?** Recovery must preserve committed values. It
restores ownership/mappings and discards speculative identities instead.

**Why retain the target during a long wait?** Upstream PCs can change while cancellation
drains. The restart must use the destination selected at the precise boundary.

**Can an old instruction-memory response survive recovery?** Yes, provided fetch retains its
old ownership and discards it as stale before admitting it to decode.

**Who gives an older fault priority over a younger branch?** The ROB/core's ordered boundary
selection. This controller executes that selected decision safely.
