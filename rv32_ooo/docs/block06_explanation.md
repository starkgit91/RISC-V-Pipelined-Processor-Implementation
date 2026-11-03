# Block 6: two-issue queue, wakeup, and oldest-ready scheduling

This block connects ID/rename to the execution paths in EX. It stores renamed instructions
until their actual source values and a suitable execution path are available. Up to two
instructions leave per clock edge. Their position in this queue does not determine retirement
order: the ROB from block 5 still controls that independently.

The execution engines are introduced in block 7. Here the tests model their availability and
supply controlled completion packets. This is a tested scheduler, not yet a complete processor.

## 1. Your example: a younger ready instruction passes an older operand wait

```asm
div x5, x1, x2       # Already issued; its physical destination p32 is not ready.
add x6, x5, x3       # Older queued instruction: needs p32, so cannot execute yet.
xor x7, x4, x8       # Younger and independent: its source values are ready.
add x9, x10, x11     # Another younger independent instruction.
```

Rename has already replaced the architectural source names with physical identities. The
waiting ADD remembers p32; another instruction writing x5 later cannot change that dependency.
The scheduler can send XOR and the independent ADD to the two integer paths while keeping the
waiting ADD in its slot. No architectural register map changes because of this choice.

When DIV's completion is accepted by the ROB, its p32 tag wakes the waiting ADD. If an integer
path is available, the ADD can issue on that same completion edge. The physical register file
from block 3 bypasses the accepted p32 value onto its read port before the edge; the execution
input captures that value. A rejected/stale completion must never be connected to this broadcast.

| Interval before edge | Accepted broadcast | Scheduler decision at edge | Waiting ADD afterward |
|---|---|---|---|
| ADD waits; XOR and independent ADD ready | None | Issue both independent instructions | Remains queued |
| DIV still running | None | No instruction issues if nothing else is ready | Remains queued |
| DIV result available; integer path ready | Nonfaulting p32 result | Issue dependent ADD with bypassed p32 value | Removed |

The ADD can execute before some older instruction retires. Completion, physical writeback, and
architectural retirement remain different events.

## 2. Connections and stored state

```text
 ID / rename                   accepted ROB completion broadcasts (up to 2)
 renamed pair + source-ready              | physical tags
          |                               v
          v                     compare with each waiting source
  [8 independent slots: valid, renamed instruction, two ready bits]
          |                    |
     ROB-relative age      operand-ready + available resource
          \                    /
             oldest eligible selection
               + compatible partner
                  /           \
              issue port 0   issue port 1
              control/ALU/   ALU/multiply-divide
              address/SYS       |
                  |             |
              two PRF tags   two PRF tags
                  \             /
                  four PRF read values
                           |
            execution input packets captured at issue edge
```

Each slot retains its own validity, the complete renamed instruction, and source readiness.
There is no instruction-value storage in this queue: operands come from the PRF at issue.
The queue uses available slots rather than shifting all entries after every issue. It still
chooses by program age, rather than giving a permanently low slot number higher priority.

With the current package, an issue entry is 303 bits: 300 bits of renamed instruction plus
valid and two readiness bits. Eight entries describe 2,424 state bits before implementation
optimization. The RTL also describes comparison networks, priority/matching logic, and output
multiplexers. Actual FPGA resources and the 50 MHz target still need Vivado verification.

## 3. Signals, sources, and destinations

All signals use the common processor clock. Packed record widths come from block 1.

| Signal | Width | Source → destination | Meaning |
|---|---:|---|---|
| clk_i, rst_ni | 1 each | Clock/reset system → queue | Rising-edge state; active-low asynchronous reset |
| dispatch_uop_i | 2 × 300 | Rename → queue | Renamed instruction, physical tags, full ROB identity |
| dispatch_valid_i | 2 | Atomic dispatch → queue | Offered program-order prefix: 00, 01, or 11 |
| dispatch_src1_ready_i, dispatch_src2_ready_i | 2 each | Rename → queue | Initial availability of each physical source |
| dispatch_ready_o, dispatch_fire_o | 2 each | Queue → dispatcher | Capacity/legality preview and actual accepted prefix |
| dispatch_hold_i | 1 | Admission/recovery control → queue | Stop new admission; permit existing work to issue |
| issue_hold_i | 1 | Recovery control → queue | Stop both new admission and issue |
| completion_i, completion_valid_i | 2 × 222, 2 | Accepted ROB completions → queue | Wake ready values; fault results do not wake |
| rob_head_index_i | 4 | ROB → scheduler | Origin for modular program-age comparisons |
| rob_head_valid_i, rob_head_tag_i | 1, 12 | ROB → scheduler | Live window and full identity for serialized system issue |
| mispredict_pending_i, mispredict_tag_i | 1, 12 | ROB → scheduler | Stop admission and allow only strictly older work to issue |
| alu0_ready_i, alu1_ready_i | 1 each | Integer sinks → scheduler | Corresponding integer/control path can accept this edge |
| memory_ready_i | 1 | Address-preparation sink → scheduler | Port 0 can prepare one load/store; not a data-bus permission |
| muldiv_ready_i | 1 | Multiply/divide engine → scheduler | Port 1 can accept one multiply/divide instruction |
| system_ready_i | 1 | Serialized system sink → scheduler | Port 0 system path can accept the matching ROB head |
| read0_rs1_o, read0_rs2_o, read1_rs1_o, read1_rs2_o | 6 each | Scheduler → PRF | Four selected physical source identities |
| read0_data1_i, read0_data2_i, read1_data1_i, read1_data2_i | 32 each | PRF → issue packets | Values, including accepted completion bypass |
| issue_packet_o | 2 × 364 | Queue/PRF → execution inputs | Selected renamed instruction and two raw register values |
| issue_fire_o | 2 | Scheduler → execution inputs | Capture this port's packet at this edge |
| flush_i, flush_done_o | 1 each | Recovery control ↔ queue | Safe clear application and registered acknowledgment |
| occupancy_o, occupied_mask_o | 4, 8 | Queue → status/tests | Current occupied slot count and valid mask |
| protocol_error_o | 1 | Queue → integration diagnostics | Malformed dispatch prefix, NONE operation, or duplicate owner |

The two issue outputs identify **execution ports**, not an older/younger instruction pair.
For example, port 1 can contain an older ADD while port 0 contains a younger branch. ROB tags
preserve their program order, so completion and retirement do not depend on port numbering.

## 4. Wakeup and operand correctness

For each source, readiness is true when any of these conditions holds:

- Rename or a previous cycle already marked the source ready.
- The decoded operation does not use that source, or its physical identity is p0.
- Either accepted completion writes that nonzero physical identity without an exception.

Both completion buses are compared with both sources of every occupied entry. The same checks
apply to an incoming instruction so it cannot miss a broadcast on its dispatch edge. Readiness
persists in the entry until it issues or is flushed. A newly enqueued instruction first becomes
eligible in the next cycle; there is no dispatch-to-execution bypass in this block.

The scheduler does not independently revalidate completion ownership. The wiring contract is:
ROB completion_accepted → nonfaulting PRF write / rename readiness / IQ wakeup. Feeding raw unit
results directly into wakeup would bypass the generation and ownership protection from block 5.

Physical-source comparison is sufficient only while allocation and recovery preserve unique
live physical destinations. A generation field does not replace cancellation: flushed units
and completion buffers must become quiescent before resources are reused.

## 5. Selecting two instructions that fit the available hardware

| Operation | Port 0 eligibility | Port 1 eligibility |
|---|---|---|
| Integer ALU | alu0_ready_i | alu1_ready_i |
| Conditional branch or jump | alu0_ready_i | None |
| Multiply/divide | None | muldiv_ready_i |
| Load/store address preparation | memory_ready_i | None |
| System operation | system_ready_i and full ROB-head tag match | None |

The ready signals advertise acceptance capacity before selection. A selected instruction is
accepted on that edge; the receiver must not withdraw acceptance after seeing issue_fire_o.
This is a pulse-based issue interface, not a stalled valid packet interface. When a sink is
unavailable, no packet is offered to it and the instruction stays queued. Selection may change
between cycles as operands or resource availability change.

Each issue port shares its two PRF read paths with that port's alternate execution resources;
the queue uses four PRF reads in total. A branch and an integer operation can issue together,
while two branches cannot. A multiply/divide instruction and one integer operation can issue together;
two integer operations can also issue together. An already-busy divider leaves the integer
paths usable. System operations issue alone at the ROB head.

The selection policy is:

1. Form the operand-ready, resource-compatible candidate set.
2. Include its oldest instruction.
3. Add the oldest different instruction that can occupy the other port.
4. If necessary, move a flexible integer instruction to port 1 to leave port 0 for a branch,
   address operation, or jump. Prefer placing the older integer on port 0 when either placement fits.

For example, older ADD + younger BEQ issue as ADD on port 1 and BEQ on port 0. Assigning ADD to
port 0 immediately would unnecessarily leave BEQ waiting. The policy gives the oldest eligible
instruction priority even when choosing two younger instructions could otherwise fill both ports.

Age is `(instruction ROB index - current head index) modulo 16`. With head index 14, indices
14, 15, 0, 1 have ages 0, 1, 2, 3. All queue entries must belong to the current live ROB window;
full tags travel onward to execution, while the index distance supplies age within that window.
The queue does not allocate or retire ROB entries.

## 6. Admission, capacity, and exactly-once issue

The dispatcher must coordinate rename, ROB, IQ, and eventually the memory queue atomically.
Every required participant accepts the same program-order prefix. Queue ready[0] previews
capacity independently of dispatch_valid_i so this coordination need not form a ready/fire loop.
Lane 1 requires an actual accepted lane 0 and sufficient remaining capacity.

Capacity comes from entries that are free at the beginning of the cycle. An entry issued on
this edge becomes dispatch capacity next cycle. This simplifies allocation and avoids using
a just-selected entry twice. With one free slot, an ordinary pair accepts only lane 0; the
front end retains and compacts lane 1. A full queue therefore introduces a refill bubble.

A dispatch fault token still enters the ROB but consumes no IQ slot: it cannot execute. A
fault or SYSTEM in lane 0 blocks lane 1, matching the ROB's serialization boundary. A fault
in lane 1 may be accepted after an ordinary lane 0 without consuming a second IQ slot. A
nonfaulting UOP_NONE is malformed and is rejected/diagnosed. Reoffering a tag that is still in
this queue, or offering the same owner twice in a pair, is also rejected/diagnosed.

An instruction is removed only when its selected path can accept it. It cannot issue twice
because the valid bit is cleared at that edge. Instructions not selected retain all instruction
metadata and readiness, including when unrelated instructions issue or arrive concurrently.

## 7. Recovery and system ordering

A known branch misprediction stops new admission. The scheduler permits only instructions
strictly older than the oldest known misprediction to newly issue. The branch itself has
already executed. Older operand waits and long operations can still make progress toward its
retirement; a pending redirect must not freeze work needed to reach the recovery point.

issue_hold_i blocks all issue during the recovery barrier. flush_i clears every queue entry
and suppresses handshakes on its application edge; flush_done_o acknowledges that edge. This
module does not cancel instructions already inside an execution unit. The later recovery
controller must coordinate those cancellation acknowledgments before applying safe reuse.

Load/store issue here prepares addresses and store operands only. Actual memory requests still
wait for the live incomplete ROB head in the later memory engine. This distinction prevents
a younger issued operation from performing a wrong-path peripheral read or store.

The full ROB tag check for system issue prevents an index-reuse alias. Execution of CSR,
fence, trap, return, and WFI behavior remains the job of the later machine-control block.

## 8. Verification and waveform reading

The standalone testbench models queue contents independently. Its selection oracle enumerates
legal port assignments rather than duplicating the RTL's greedy matching algorithm. Directed
and generated cases exercise operand waits, dual broadcasts, resource conflicts, age wrap,
capacity, prefix admission, system ordering, recovery filtering, and retained instruction data.
The connected test wires rename, ROB, PRF, this queue, and the trace recorder, with modeled
execution/completion.
It checks that source readiness and the captured register values agree on the issue edge.

Run from the project directory:

```sh
python3 scripts/run_checks.py --block 6
python3 scripts/run_checks.py --block 6 --waves
```

Current Icarus evidence is **34,054 standalone checks + 43 connected checks = 34,097 checks,
zero failures**. The report records separate case counts and source hashes. The optional VCDs are in
build/block06. Start with dispatch_fire_o, occupancy_o, completion_valid_i, issue_fire_o,
read*_rs*_o, issue_packet_o, and the ROB head. A broadcast can change combinational issue
selection before the clock edge; removal and PRF persistence happen at the edge. Observe
rob retirement separately to see that issue order does not change architectural order.

For Vivado XSim, the standalone scheduler source list is:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_issue_queue.sv tb/tb_block06.sv
xelab tb_block06 -s block06_sim
xsim block06_sim --runall
```

The connected case also needs rv32_rename.sv, rv32_physical_regfile.sv, and
rv32_reorder_buffer.sv and rv32_trace_recorder.sv before tb_block06_integration.sv. XSim and synthesis have not been run
here. Passing these tests does not prove execution-engine correctness, whole-core ISA
compliance, AXI behavior, or complete recovery coordination.

## 9. Interview questions

**Why can a younger ADD issue before an older instruction?** Its own physical sources and
execution resource are ready. The ROB still prevents it from retiring ahead of older work.

**Does renaming remove RAW dependencies?** No. Renaming removes WAR/WAW name dependencies;
RAW waits remain attached to physical-source identities and wake when their values are ready.

**Why keep ready bits as well as physical tags?** The tag identifies the producer's value;
the ready bit records whether that value is available. Comparing completion tags wakes only
instructions waiting for that specific physical result.

**Why does same-cycle wakeup need PRF bypass?** The storage write happens at the upcoming edge.
Without bypass, an asynchronous PRF read might still expose the previous value while the
scheduler already treats the new result as ready.

**Why can port 1 contain the older instruction?** The ports represent hardware routes.
Moving a flexible integer instruction can let a port-0-only operation execute alongside it.
The instruction's ROB identity preserves ordering independently of its route.

**Why not execute a younger MMIO load as soon as its address is ready?** Peripheral reads
can have side effects. This design prepares the address early but starts the transaction only
at the ROB head, after older instructions have committed safely.

**Why does misprediction recovery allow older instructions to issue?** Recovery occurs when
the mispredicted instruction retires. Freezing its older producers would prevent that boundary
from ever being reached.

## 10. References and implementation ownership

[BOOM's issue-unit documentation](https://docs.boom-core.org/en/latest/sections/issue-units.html)
explains operand readiness, compatible execution ports, availability, and age-based priority.
This custom design uses one stationary queue and explicit ROB-relative age matching, rather
than BOOM's split queues or collapsing age-ordered queue. No third-party RTL is copied here.
Comments and naming follow the intent of the
[lowRISC style guide](https://github.com/lowRISC/style-guides/blob/master/VerilogCodingStyle.md).
