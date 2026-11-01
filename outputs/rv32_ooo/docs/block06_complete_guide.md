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

## Complete two-issue queue and scheduler

Source: `rtl/rv32_issue_queue.sv`.

```systemverilog
// Eight-entry, two-issue scheduler. Program age comes from the live ROB window.
// Ready inputs advertise execution acceptance THIS edge, not future capacity.
`timescale 1ns / 1ps

module rv32_issue_queue
  import rv32_ooo_pkg::*;
(
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  logic                 dispatch_hold_i,
  input  logic                 issue_hold_i,
  input  logic [1:0]           dispatch_valid_i,
  input  renamed_pair_t        dispatch_uop_i,
  input  logic [1:0]           dispatch_src1_ready_i,
  input  logic [1:0]           dispatch_src2_ready_i,
  output logic [1:0]           dispatch_ready_o,
  output logic [1:0]           dispatch_fire_o,

  // Broadcast only completions accepted by the ROB ownership checks.
  input  logic [1:0]           completion_valid_i,
  input  completion_pair_t     completion_i,
  input  logic                 rob_head_valid_i,
  input  rob_index_t           rob_head_index_i,
  input  rob_tag_t             rob_head_tag_i,
  input  logic                 mispredict_pending_i,
  input  rob_tag_t             mispredict_tag_i,

  // Port 0: integer/control/address/system. Port 1: integer or multiply/divide.
  input  logic                 alu0_ready_i,
  input  logic                 alu1_ready_i,
  input  logic                 muldiv_ready_i,
  input  logic                 memory_ready_i,
  input  logic                 system_ready_i,
  output logic [1:0]           issue_fire_o,
  output execute_packet_pair_t issue_packet_o,
  output phys_reg_t            read0_rs1_o,
  output phys_reg_t            read0_rs2_o,
  output phys_reg_t            read1_rs1_o,
  output phys_reg_t            read1_rs2_o,
  input  word_t                read0_data1_i,
  input  word_t                read0_data2_i,
  input  word_t                read1_data1_i,
  input  word_t                read1_data2_i,

  // Apply only after recovery has cancelled/drained all executing owners.
  input  logic                 flush_i,
  output logic                 flush_done_o,
  output issue_count_t         occupancy_o,
  output logic [ISSUE_DEPTH-1:0] occupied_mask_o,
  output logic                 protocol_error_o
);

  issue_entry_t entries_q [ISSUE_DEPTH];
  issue_entry_t entries_d [ISSUE_DEPTH];
  renamed_pair_t selected_uop;

  function automatic logic source_ready(
    input logic initial_ready,
    input logic used,
    input phys_reg_t source_tag,
    input logic [1:0] wb_valid,
    input completion_pair_t wb
  );
    logic hit0, hit1;
    begin
      hit0 = wb_valid[0] && wb.lane0.writes_rd && !wb.lane0.exception.valid
             && wb.lane0.pdst != 0 && wb.lane0.pdst == source_tag;
      hit1 = wb_valid[1] && wb.lane1.writes_rd && !wb.lane1.exception.valid
             && wb.lane1.pdst != 0 && wb.lane1.pdst == source_tag;
      source_ready = initial_ready || !used || source_tag == 0 || hit0 || hit1;
    end
  endfunction

  function automatic int program_age(input rob_index_t index, input rob_index_t head);
    rob_index_t distance;
    begin
      distance = index - head;
      program_age = int'(distance);
    end
  endfunction

  // Selection does not depend on PRF data. This avoids a combinational cycle
  // through the read multiplexers; a separate path attaches the selected values.
  always_comb begin : schedule
    issue_entry_t entry;
    renamed_uop_t incoming0, incoming1;
    logic [ISSUE_DEPTH-1:0] eligible0, eligible1, is_system;
    int age [ISSUE_DEPTH];
    int free0, free1, free_count;
    int oldest, partner, select0, select1, boundary_age;
    logic duplicate0, duplicate1, malformed0, malformed1;
    logic skip0, skip1, allow_dispatch;

    incoming0 = dispatch_uop_i.lane0;
    incoming1 = dispatch_uop_i.lane1;
    eligible0 = '0;
    eligible1 = '0;
    is_system = '0;
    occupied_mask_o = '0;
    occupancy_o = '0;
    free0 = -1;
    free1 = -1;
    free_count = 0;
    duplicate0 = 1'b0;
    duplicate1 = 1'b0;
    boundary_age = program_age(mispredict_tag_i.index, rob_head_index_i);

    for (int i = 0; i < ISSUE_DEPTH; i++) begin
      entry = entries_q[i];
      age[i] = program_age(entry.uop.rob_tag.index, rob_head_index_i);
      occupied_mask_o[i] = entry.valid;
      if (entry.valid) begin
        occupancy_o = occupancy_o + issue_count_t'(1);
        duplicate0 = duplicate0 || entry.uop.rob_tag == incoming0.rob_tag;
        duplicate1 = duplicate1 || entry.uop.rob_tag == incoming1.rob_tag;
      end else begin
        if (free0 < 0) free0 = i;
        else if (free1 < 0) free1 = i;
        free_count = free_count + 1;
      end
      entry.src1_ready = source_ready(entry.src1_ready, entry.uop.decoded.uses_rs1,
                                     entry.uop.prs1, completion_valid_i, completion_i);
      entry.src2_ready = source_ready(entry.src2_ready, entry.uop.decoded.uses_rs2,
                                     entry.uop.prs2, completion_valid_i, completion_i);
      entries_d[i] = entry;
      is_system[i] = entry.uop.decoded.op_class == UOP_SYSTEM;

      if (rst_ni && !flush_i && !issue_hold_i && rob_head_valid_i && entry.valid
          && entry.src1_ready && entry.src2_ready
          && (!mispredict_pending_i || age[i] < boundary_age)) begin
        case (entry.uop.decoded.op_class)
          UOP_INTEGER: begin
            eligible0[i] = alu0_ready_i;
            eligible1[i] = alu1_ready_i;
          end
          UOP_BRANCH, UOP_JUMP: eligible0[i] = alu0_ready_i;
          UOP_MULDIV: eligible1[i] = muldiv_ready_i;
          UOP_LOAD, UOP_STORE: eligible0[i] = memory_ready_i;
          UOP_SYSTEM: eligible0[i] = system_ready_i
                                        && entry.uop.rob_tag == rob_head_tag_i;
          default: begin end
        endcase
      end
    end

    // Include the oldest eligible operation, then the oldest compatible partner.
    // Flexible INTEGER placement preserves a port-0-only partner's opportunity.
    oldest = -1;
    partner = -1;
    select0 = -1;
    select1 = -1;
    for (int i = 0; i < ISSUE_DEPTH; i++) begin
      if ((eligible0[i] || eligible1[i]) && (oldest < 0 || age[i] < age[oldest]))
        oldest = i;
    end
    if (oldest >= 0) begin
      if (is_system[oldest]) begin
        select0 = oldest;
      end else begin
        for (int i = 0; i < ISSUE_DEPTH; i++) begin
          if (i != oldest && !is_system[i]
              && ((eligible0[oldest] && eligible1[i])
                  || (eligible1[oldest] && eligible0[i]))
              && (partner < 0 || age[i] < age[partner])) partner = i;
        end
        if (partner < 0) begin
          if (eligible0[oldest]) select0 = oldest;
          else select1 = oldest;
        end else if (eligible0[oldest] && eligible1[partner]) begin
          select0 = oldest;
          select1 = partner;
        end else begin
          select1 = oldest;
          select0 = partner;
        end
      end
    end

    issue_fire_o = '0;
    selected_uop = '0;
    read0_rs1_o = '0;
    read0_rs2_o = '0;
    read1_rs1_o = '0;
    read1_rs2_o = '0;
    if (select0 >= 0) begin
      entry = entries_d[select0];
      issue_fire_o[0] = 1'b1;
      selected_uop.lane0 = entry.uop;
      if (entry.uop.decoded.uses_rs1) read0_rs1_o = entry.uop.prs1;
      if (entry.uop.decoded.uses_rs2) read0_rs2_o = entry.uop.prs2;
      entry.valid = 1'b0;
      entries_d[select0] = entry;
    end
    if (select1 >= 0) begin
      entry = entries_d[select1];
      issue_fire_o[1] = 1'b1;
      selected_uop.lane1 = entry.uop;
      if (entry.uop.decoded.uses_rs1) read1_rs1_o = entry.uop.prs1;
      if (entry.uop.decoded.uses_rs2) read1_rs2_o = entry.uop.prs2;
      entry.valid = 1'b0;
      entries_d[select1] = entry;
    end

    // Preview readiness must not depend on valid[0]: atomic dispatch may derive
    // each participant's valid from the combined readiness of all participants.
    skip0 = incoming0.decoded.fetch.exception.valid;
    skip1 = incoming1.decoded.fetch.exception.valid;
    malformed0 = incoming0.decoded.op_class == UOP_NONE && !skip0;
    malformed1 = incoming1.decoded.op_class == UOP_NONE && !skip1;
    allow_dispatch = rst_ni && !flush_i && !dispatch_hold_i && !issue_hold_i
                     && !mispredict_pending_i;
    dispatch_ready_o = '0;
    dispatch_fire_o = '0;
    dispatch_ready_o[0] = allow_dispatch && !duplicate0 && !malformed0
                          && (skip0 || free_count >= 1);
    dispatch_fire_o[0] = dispatch_valid_i[0] && dispatch_ready_o[0];
    dispatch_ready_o[1] = dispatch_fire_o[0] && !skip0
                          && incoming0.decoded.op_class != UOP_SYSTEM
                          && !duplicate1 && !malformed1
                          && incoming1.rob_tag != incoming0.rob_tag
                          && (skip1 || free_count >= 2);
    dispatch_fire_o[1] = dispatch_valid_i[1] && dispatch_ready_o[1];

    if (dispatch_fire_o[0] && !skip0) begin
      entry = '0;
      entry.valid = 1'b1;
      entry.uop = incoming0;
      entry.src1_ready = source_ready(dispatch_src1_ready_i[0],
                        incoming0.decoded.uses_rs1, incoming0.prs1,
                        completion_valid_i, completion_i);
      entry.src2_ready = source_ready(dispatch_src2_ready_i[0],
                        incoming0.decoded.uses_rs2, incoming0.prs2,
                        completion_valid_i, completion_i);
      entries_d[free0] = entry;
    end
    if (dispatch_fire_o[1] && !skip1) begin
      entry = '0;
      entry.valid = 1'b1;
      entry.uop = incoming1;
      entry.src1_ready = source_ready(dispatch_src1_ready_i[1],
                        incoming1.decoded.uses_rs1, incoming1.prs1,
                        completion_valid_i, completion_i);
      entry.src2_ready = source_ready(dispatch_src2_ready_i[1],
                        incoming1.decoded.uses_rs2, incoming1.prs2,
                        completion_valid_i, completion_i);
      entries_d[free1] = entry;
    end

    protocol_error_o = 1'b0;
    if (rst_ni && !flush_i) begin
      protocol_error_o = dispatch_valid_i == 2'b10
                        || (dispatch_valid_i[0] && (duplicate0 || malformed0))
                        || (dispatch_valid_i[1] && (duplicate1 || malformed1))
                        || (dispatch_valid_i == 2'b11
                            && incoming0.rob_tag == incoming1.rob_tag);
    end
  end

  // The selected tags drive the four PRF read muxes. Accepted completion bypass
  // in the PRF makes a newly awakened value available on this same issue edge.
  always_comb begin
    issue_packet_o = '0;
    if (issue_fire_o[0]) begin
      issue_packet_o.lane0.uop = selected_uop.lane0;
      issue_packet_o.lane0.rs1_value = read0_data1_i;
      issue_packet_o.lane0.rs2_value = read0_data2_i;
    end
    if (issue_fire_o[1]) begin
      issue_packet_o.lane1.uop = selected_uop.lane1;
      issue_packet_o.lane1.rs1_value = read1_data1_i;
      issue_packet_o.lane1.rs2_value = read1_data2_i;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < ISSUE_DEPTH; i++) entries_q[i] <= '0;
      flush_done_o <= 1'b0;
    end else begin
      flush_done_o <= flush_i;
      for (int i = 0; i < ISSUE_DEPTH; i++) begin
        if (flush_i) entries_q[i] <= '0;
        else entries_q[i] <= entries_d[i];
      end
    end
  end

endmodule
```

## Complete self-checking scheduler test

Source: `tb/tb_block06.sv`.

```systemverilog
// Standalone issue-queue tests. PRF values and accepted ROB completions are test drivers.
`timescale 1ns / 1ps

module tb_block06;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1, dispatch_hold, issue_hold, flush, flush_done;
  logic [1:0] dispatch_valid, dispatch_ready, dispatch_fire;
  renamed_pair_t dispatch_uop;
  logic [1:0] dispatch_src1_ready, dispatch_src2_ready;
  logic [1:0] completion_valid;
  completion_pair_t completion;
  logic rob_head_valid, mispredict_pending;
  rob_index_t rob_head_index;
  rob_tag_t rob_head_tag, mispredict_tag;
  logic alu0_ready, alu1_ready, muldiv_ready, memory_ready, system_ready;
  logic [1:0] issue_fire;
  execute_packet_pair_t issue_packet;
  phys_reg_t read0_rs1, read0_rs2, read1_rs1, read1_rs2;
  word_t read0_data1, read0_data2, read1_data1, read1_data2;
  issue_count_t occupancy;
  logic [ISSUE_DEPTH-1:0] occupied_mask;
  logic protocol_error;

  issue_entry_t ref_entries [0:ISSUE_DEPTH-1];
  word_t ref_values [0:PHYS_REG_COUNT-1];
  logic [ISSUE_DEPTH-1:0] expected_mask;
  logic [1:0] expected_dispatch, expected_issue;
  execute_packet_pair_t expected_packets;
  integer selected0, selected1, free0, free1;
  logic expected_protocol;
  renamed_uop_t a_uop, b_uop, c_uop, d_uop;
  integer checks = 0, failures = 0;
  integer n, kind0, kind1, resources, head_case, batch;
  word_t rng = 32'h43c0ffee;

  rv32_issue_queue dut (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_hold_i(dispatch_hold), .issue_hold_i(issue_hold),
    .dispatch_valid_i(dispatch_valid), .dispatch_uop_i(dispatch_uop),
    .dispatch_src1_ready_i(dispatch_src1_ready), .dispatch_src2_ready_i(dispatch_src2_ready),
    .dispatch_ready_o(dispatch_ready), .dispatch_fire_o(dispatch_fire),
    .completion_valid_i(completion_valid), .completion_i(completion),
    .rob_head_valid_i(rob_head_valid), .rob_head_index_i(rob_head_index),
    .rob_head_tag_i(rob_head_tag), .mispredict_pending_i(mispredict_pending),
    .mispredict_tag_i(mispredict_tag), .alu0_ready_i(alu0_ready), .alu1_ready_i(alu1_ready),
    .muldiv_ready_i(muldiv_ready), .memory_ready_i(memory_ready), .system_ready_i(system_ready),
    .issue_fire_o(issue_fire), .issue_packet_o(issue_packet),
    .read0_rs1_o(read0_rs1), .read0_rs2_o(read0_rs2),
    .read1_rs1_o(read1_rs1), .read1_rs2_o(read1_rs2),
    .read0_data1_i(read0_data1), .read0_data2_i(read0_data2),
    .read1_data1_i(read1_data1), .read1_data2_i(read1_data2),
    .flush_i(flush), .flush_done_o(flush_done), .occupancy_o(occupancy),
    .occupied_mask_o(occupied_mask), .protocol_error_o(protocol_error)
  );

  function automatic word_t next_random(input word_t state);
    next_random = {state[30:0], state[31] ^ state[21] ^ state[1] ^ state[0]};
  endfunction

  function automatic word_t source_value(input phys_reg_t physical);
    completion_t first, second;
    begin
      first = completion.lane0; second = completion.lane1;
      if (!rst_n || physical == 0) source_value = 0;
      else if (completion_valid[0] && first.writes_rd && !first.exception.valid &&
               first.pdst == physical) source_value = first.value;
      else if (completion_valid[1] && second.writes_rd && !second.exception.valid &&
               second.pdst == physical) source_value = second.value;
      else source_value = ref_values[physical];
    end
  endfunction

  // Test-only PRF/bypass model. Selection is independent of these returned data values.
  always_comb begin
    read0_data1 = source_value(read0_rs1);
    read0_data2 = source_value(read0_rs2);
    read1_data1 = source_value(read1_rs1);
    read1_data2 = source_value(read1_rs2);
  end

  function automatic logic awakened(input logic already, used, input phys_reg_t tag);
    completion_t packet;
    logic result;
    integer lane;
    begin
      result = already || !used || tag == 0;
      for (lane = 0; lane < 2; lane = lane + 1) begin
        packet = lane == 0 ? completion.lane0 : completion.lane1;
        if (completion_valid[lane] && packet.writes_rd && !packet.exception.valid &&
            packet.pdst != 0 && packet.pdst == tag) result = 1;
      end
      awakened = result;
    end
  endfunction

  function automatic integer age_of(input rob_tag_t tag);
    integer index_value;
    begin
      index_value = tag.index;
      age_of = (index_value - rob_head_index + ROB_DEPTH) % ROB_DEPTH;
    end
  endfunction

  function automatic logic duplicate_tag(input rob_tag_t tag);
    issue_entry_t entry;
    integer slot;
    begin
      duplicate_tag = 0;
      for (slot = 0; slot < ISSUE_DEPTH; slot = slot + 1) begin
        entry = ref_entries[slot];
        if (entry.valid && entry.uop.rob_tag == tag) duplicate_tag = 1;
      end
    end
  endfunction

  function automatic logic eligible(input integer slot, port_number);
    issue_entry_t entry;
    logic values_ready, control_allows, supports;
    begin
      entry = ref_entries[slot];
      values_ready = awakened(entry.src1_ready, entry.uop.decoded.uses_rs1, entry.uop.prs1) &&
                     awakened(entry.src2_ready, entry.uop.decoded.uses_rs2, entry.uop.prs2);
      control_allows = rst_n && !flush && !issue_hold && rob_head_valid && entry.valid &&
                       (!mispredict_pending || age_of(entry.uop.rob_tag) < age_of(mispredict_tag));
      supports = 0;
      case (entry.uop.decoded.op_class)
        UOP_INTEGER: supports = port_number == 0 ? alu0_ready : alu1_ready;
        UOP_BRANCH, UOP_JUMP: supports = port_number == 0 && alu0_ready;
        UOP_MULDIV: supports = port_number == 1 && muldiv_ready;
        UOP_LOAD, UOP_STORE: supports = port_number == 0 && memory_ready;
        UOP_SYSTEM: supports = port_number == 0 && system_ready &&
                               entry.uop.rob_tag == rob_head_tag;
        default: supports = 0;
      endcase
      eligible = values_ready && control_allows && supports;
    end
  endfunction

  function automatic renamed_uop_t operation(input integer id, kind, tag_index);
    renamed_uop_t value;
    begin
      value = '0;
      value.decoded.fetch.pc = 32'h100 + id * 4;
      value.decoded.fetch.instruction = 32'h002080b3;
      value.decoded.fetch.prediction.next_pc = value.decoded.fetch.pc + 4;
      // Distinct nonzero metadata detects truncation or accidental field reconstruction.
      value.decoded.fetch.prediction.taken = id % 2;
      value.decoded.fetch.prediction.btb_hit = (id / 2) % 2;
      value.decoded.fetch.prediction.pht_index = id * 7 + 3;
      value.decoded.fetch.exception.cause = 32'hc0000000 + id;
      value.decoded.fetch.exception.tval = 32'hd0000000 + id * 9;
      if (id % 2 == 0) begin
        value.decoded.src_a = SRC_A_PC;
        value.decoded.src_b = SRC_B_IMMEDIATE;
        value.decoded.branch_op = BRANCH_LT;
        value.decoded.mem_size = MEM_HALF;
        value.decoded.csr_op = CSR_RS;
      end else begin
        value.decoded.src_a = SRC_A_ZERO;
        value.decoded.src_b = SRC_B_REGISTER;
        value.decoded.branch_op = BRANCH_GEU;
        value.decoded.mem_size = MEM_WORD;
        value.decoded.csr_op = CSR_RCI;
      end
      value.decoded.load_unsigned = id % 2;
      value.decoded.jump_indirect = (id / 2) % 2;
      value.decoded.rs1 = 1 + (id * 3) % 31;
      value.decoded.rs2 = 1 + (id * 5) % 31;
      value.decoded.immediate = 32'h81230000 + id * 33;
      value.decoded.csr_address = 12'h340 + id % 16;
      value.decoded.csr_immediate = id * 3 + 1;
      value.old_pdst = 1 + (id * 9) % 63;
      value.decoded.op_class = UOP_INTEGER;
      if (id % 2 == 0) value.decoded.alu_op = ALU_AND;
      else value.decoded.alu_op = ALU_XOR;
      value.decoded.uses_rs1 = 1;
      value.decoded.uses_rs2 = 1;
      value.decoded.writes_rd = 1;
      value.decoded.rd = 1 + id % 31;
      value.prs1 = 1 + id % 31;
      value.prs2 = 1 + (id + 7) % 31;
      value.pdst = 32 + id % 32;
      value.rob_tag.index = tag_index;
      value.rob_tag.generation = 9;
      case (kind)
        1: begin
          value.decoded.op_class = UOP_MULDIV;
          if (id % 2 == 0) value.decoded.muldiv_op = MD_DIV;
          else value.decoded.muldiv_op = MD_REMU;
        end
        2: begin
          value.decoded.op_class = UOP_BRANCH;
          value.decoded.writes_rd = 0; value.pdst = 0;
        end
        3: begin value.decoded.op_class = UOP_JUMP; value.decoded.uses_rs2 = 0; end
        4: begin value.decoded.op_class = UOP_LOAD; value.decoded.uses_rs2 = 0; end
        5: begin
          value.decoded.op_class = UOP_STORE;
          value.decoded.writes_rd = 0; value.pdst = 0;
        end
        6: begin
          value.decoded.op_class = UOP_SYSTEM; value.decoded.system_op = SYS_CSR;
          value.decoded.uses_rs2 = 0;
        end
        default: begin end
      endcase
      operation = value;
    end
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 30)
          $display("FAIL: %s time=%0t count=%0d", label_text, $time, occupancy);
      end
    end
  endtask

  task automatic resources_for(input integer mask);
    begin
      alu0_ready = (mask & 1) != 0;
      alu1_ready = (mask & 2) != 0;
      muldiv_ready = (mask & 4) != 0;
      memory_ready = (mask & 8) != 0;
      system_ready = (mask & 16) != 0;
    end
  endtask

  task automatic neutral;
    begin
      dispatch_hold = 0; issue_hold = 0; flush = 0;
      dispatch_valid = 0; dispatch_uop = '0;
      dispatch_src1_ready = 0; dispatch_src2_ready = 0;
      completion_valid = 0; completion = '0;
      rob_head_valid = 1; rob_head_index = 0;
      rob_head_tag = '0; rob_head_tag.generation = 9;
      mispredict_pending = 0; mispredict_tag = '0;
      resources_for(0);
    end
  endtask

  task automatic offer(input renamed_uop_t older, younger, input logic [1:0] mask);
    begin
      dispatch_uop.lane0 = older; dispatch_uop.lane1 = younger;
      dispatch_valid = mask;
      dispatch_src1_ready = 3; dispatch_src2_ready = 3;
    end
  endtask

  // Enumerate every legal assignment, rather than reproduce the DUT's greedy selection.
  // Rank: oldest selected age, two before one, oldest partner, then older on port 0.
  task automatic select_reference;
    integer p0, p1, number, first_age, second_age, a0, a1, tie;
    integer best_first, best_count, best_second, best_tie;
    issue_entry_t e0, e1;
    logic legal, better;
    begin
      selected0 = -1; selected1 = -1;
      best_first = 100; best_count = 0; best_second = 100; best_tie = 100;
      for (p0 = -1; p0 < ISSUE_DEPTH; p0 = p0 + 1) begin
        for (p1 = -1; p1 < ISSUE_DEPTH; p1 = p1 + 1) begin
          legal = (p0 >= 0 || p1 >= 0) && (p0 < 0 || p1 < 0 || p0 != p1);
          e0 = '0; e1 = '0;
          if (p0 >= 0) begin e0 = ref_entries[p0]; legal = legal && eligible(p0, 0); end
          if (p1 >= 0) begin e1 = ref_entries[p1]; legal = legal && eligible(p1, 1); end
          if (p0 >= 0 && p1 >= 0 &&
              (e0.uop.decoded.op_class == UOP_SYSTEM || e1.uop.decoded.op_class == UOP_SYSTEM))
            legal = 0;
          if (legal) begin
            number = (p0 >= 0 ? 1 : 0) + (p1 >= 0 ? 1 : 0);
            a0 = p0 >= 0 ? age_of(e0.uop.rob_tag) : 100;
            a1 = p1 >= 0 ? age_of(e1.uop.rob_tag) : 100;
            first_age = a0 < a1 ? a0 : a1;
            second_age = number == 2 ? (a0 > a1 ? a0 : a1) : 100;
            tie = a0 <= a1 ? 0 : 1;
            better = first_age < best_first ||
                     (first_age == best_first && number > best_count) ||
                     (first_age == best_first && number == best_count &&
                      second_age < best_second) ||
                     (first_age == best_first && number == best_count &&
                      second_age == best_second && tie < best_tie);
            if (better) begin
              selected0 = p0; selected1 = p1;
              best_first = first_age; best_count = number;
              best_second = second_age; best_tie = tie;
            end
          end
        end
      end
    end
  endtask

  task automatic evaluate;
    integer index, free_count, expected_count;
    issue_entry_t entry;
    renamed_uop_t u0, u1;
    phys_reg_t p01, p02, p11, p12;
    logic ready0, ready1, skip0, skip1, bad0, bad1, duplicate0, duplicate1, allow;
    begin
      select_reference;
      expected_issue = 0;
      expected_packets = '0;
      p01 = 0; p02 = 0; p11 = 0; p12 = 0;
      if (selected0 >= 0) begin
        entry = ref_entries[selected0];
        expected_issue[0] = 1;
        expected_packets.lane0.uop = entry.uop;
        if (entry.uop.decoded.uses_rs1) p01 = entry.uop.prs1;
        if (entry.uop.decoded.uses_rs2) p02 = entry.uop.prs2;
        expected_packets.lane0.rs1_value = source_value(p01);
        expected_packets.lane0.rs2_value = source_value(p02);
      end
      if (selected1 >= 0) begin
        entry = ref_entries[selected1];
        expected_issue[1] = 1;
        expected_packets.lane1.uop = entry.uop;
        if (entry.uop.decoded.uses_rs1) p11 = entry.uop.prs1;
        if (entry.uop.decoded.uses_rs2) p12 = entry.uop.prs2;
        expected_packets.lane1.rs1_value = source_value(p11);
        expected_packets.lane1.rs2_value = source_value(p12);
      end
      check(issue_fire == expected_issue, "issue ports match exhaustive legal-assignment oracle");
      check(issue_packet == expected_packets, "selected owners and captured operand values");
      check(read0_rs1 == p01 && read0_rs2 == p02 && read1_rs1 == p11 && read1_rs2 == p12,
            "four PRF selectors belong to the selected execution ports");
      free0 = -1; free1 = -1; free_count = 0; expected_count = 0; expected_mask = 0;
      for (index = 0; index < ISSUE_DEPTH; index = index + 1) begin
        entry = ref_entries[index];
        if (entry.valid) begin expected_count = expected_count + 1; expected_mask[index] = 1; end
        else begin
          if (free0 < 0) free0 = index;
          else if (free1 < 0) free1 = index;
          free_count = free_count + 1;
        end
      end
      check(occupancy == expected_count && occupied_mask == expected_mask,
            "current occupied slots and count match reference");
      u0 = dispatch_uop.lane0; u1 = dispatch_uop.lane1;
      duplicate0 = duplicate_tag(u0.rob_tag); duplicate1 = duplicate_tag(u1.rob_tag);
      skip0 = u0.decoded.fetch.exception.valid; skip1 = u1.decoded.fetch.exception.valid;
      bad0 = u0.decoded.op_class == UOP_NONE && !skip0;
      bad1 = u1.decoded.op_class == UOP_NONE && !skip1;
      allow = rst_n && !flush && !dispatch_hold && !issue_hold && !mispredict_pending;
      ready0 = allow && !duplicate0 && !bad0 && (skip0 || free_count >= 1);
      expected_dispatch[0] = dispatch_valid[0] && ready0;
      ready1 = expected_dispatch[0] && !skip0 && u0.decoded.op_class != UOP_SYSTEM &&
               !duplicate1 && !bad1 && u1.rob_tag != u0.rob_tag && (skip1 || free_count >= 2);
      expected_dispatch[1] = dispatch_valid[1] && ready1;
      check(dispatch_ready == {ready1, ready0} && dispatch_fire == expected_dispatch,
            "admission uses current free capacity and accepts a valid prefix");
      expected_protocol = rst_n && !flush && (dispatch_valid == 2'b10 ||
                          (dispatch_valid[0] && (duplicate0 || bad0)) ||
                          (dispatch_valid[1] && (duplicate1 || bad1)) ||
                          (dispatch_valid == 3 && u0.rob_tag == u1.rob_tag));
      check(protocol_error == expected_protocol, "malformed or duplicate offers are diagnosed");
    end
  endtask

  task automatic update_reference;
    integer index, lane;
    issue_entry_t entry;
    renamed_uop_t value;
    completion_t wb;
    begin
      if (flush) begin
        for (index = 0; index < ISSUE_DEPTH; index = index + 1) ref_entries[index] = '0;
      end else begin
        for (index = 0; index < ISSUE_DEPTH; index = index + 1) begin
          entry = ref_entries[index];
          entry.src1_ready = awakened(entry.src1_ready, entry.uop.decoded.uses_rs1, entry.uop.prs1);
          entry.src2_ready = awakened(entry.src2_ready, entry.uop.decoded.uses_rs2, entry.uop.prs2);
          ref_entries[index] = entry;
        end
        if (selected0 >= 0) begin
          entry = ref_entries[selected0]; entry.valid = 0; ref_entries[selected0] = entry;
        end
        if (selected1 >= 0) begin
          entry = ref_entries[selected1]; entry.valid = 0; ref_entries[selected1] = entry;
        end
        for (lane = 0; lane < 2; lane = lane + 1) begin
          value = lane == 0 ? dispatch_uop.lane0 : dispatch_uop.lane1;
          if (expected_dispatch[lane] && !value.decoded.fetch.exception.valid) begin
            entry = '0; entry.valid = 1; entry.uop = value;
            entry.src1_ready = awakened(dispatch_src1_ready[lane],
                               value.decoded.uses_rs1, value.prs1);
            entry.src2_ready = awakened(dispatch_src2_ready[lane],
                               value.decoded.uses_rs2, value.prs2);
            ref_entries[lane == 0 ? free0 : free1] = entry;
          end
        end
      end
      // Same-edge accepted writes persist in the test-only PRF model; lane 0 wins a collision.
      for (lane = 1; lane >= 0; lane = lane - 1) begin
        wb = lane == 0 ? completion.lane0 : completion.lane1;
        if (completion_valid[lane] && wb.writes_rd && !wb.exception.valid && wb.pdst != 0)
          ref_values[wb.pdst] = wb.value;
      end
    end
  endtask

  task automatic cycle;
    logic applied;
    begin
      #1; evaluate;
      applied = flush;
      update_reference;
      #2; clk = 1; #1;
      check(flush_done == applied, "flush acknowledgement follows the application edge");
      clk = 0; neutral; #1; evaluate;
    end
  endtask

  task automatic reset_dut;
    integer index;
    begin
      clk = 0; neutral; rst_n = 0;
      for (index = 0; index < ISSUE_DEPTH; index = index + 1) ref_entries[index] = '0;
      for (index = 0; index < PHYS_REG_COUNT; index = index + 1)
        ref_values[index] = index == 0 ? 0 : 32'h11000000 + index * 32'h10201;
      #2; check(issue_fire == 0 && dispatch_fire == 0 && !flush_done, "reset dominates activity");
      rst_n = 1; #1; evaluate;
    end
  endtask

  task automatic flush_queue;
    begin neutral; flush = 1; cycle; neutral; cycle; end
  endtask

  task automatic queue_pair(input renamed_uop_t older, younger);
    begin neutral; offer(older, younger, 3); cycle; end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block06/block06.vcd");
      $dumpvars(0, tb_block06);
    end
    #3000000; $fatal(1, "Block 6 timed out");
  end

  initial begin
    reset_dut;
    check($bits(issue_count_t) == 4, "eight-entry occupancy includes full count");
    check($bits(execute_packet_pair_t) == 2 * $bits(execute_packet_t),
          "issue pair carries two complete owner/value records");

    // Oldest INTEGER may use port 1 so a younger port-0-only branch can issue alongside it.
    queue_pair(operation(0, 0, 0), operation(1, 2, 1));
    resources_for(31); #1; evaluate;
    check(issue_fire == 3 && issue_packet.lane0.uop.decoded.op_class == UOP_BRANCH &&
          issue_packet.lane1.uop.rob_tag.index == 0, "flexible ALU moves for branch-only partner");
    cycle;
    check(occupancy == 0, "both accepted issue ports remove their separate owners");

    // An older RAW-blocked instruction does not prevent an independent younger instruction.
    a_uop = operation(2, 0, 0); a_uop.prs1 = 37; a_uop.prs2 = 38;
    b_uop = operation(3, 0, 1);
    offer(a_uop, b_uop, 3); dispatch_src1_ready = 2; dispatch_src2_ready = 2; cycle;
    resources_for(31); #1;
    check(issue_fire == 1 && issue_packet.lane0.uop.rob_tag == b_uop.rob_tag,
          "younger ready operation bypasses an older RAW dependency"); cycle;
    resources_for(31); #1; check(issue_fire == 0, "unfinished source values prevent issue");
    completion_valid = 3;
    completion.lane0.writes_rd = 1; completion.lane0.pdst = 37;
    completion.lane0.value = 99;
    completion.lane1.writes_rd = 1; completion.lane1.pdst = 38;
    completion.lane1.value = 123; #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.rs1_value == 99 &&
          issue_packet.lane0.rs2_value == 123,
          "dual same-cycle wakeup forwards new operand values");
    cycle;

    // Each broadcast can wake several consumers, including both selected instructions.
    a_uop = operation(83, 0, 0); a_uop.prs1 = 42; a_uop.prs2 = 43;
    b_uop = operation(84, 0, 1); b_uop.prs1 = 43; b_uop.prs2 = 42;
    offer(a_uop, b_uop, 3); dispatch_src1_ready = 0; dispatch_src2_ready = 0; cycle;
    completion_valid = 3;
    completion.lane0.writes_rd = 1; completion.lane0.pdst = 42;
    completion.lane0.value = 32'h12345678;
    completion.lane1.writes_rd = 1; completion.lane1.pdst = 43;
    completion.lane1.value = 32'h89abcdef;
    resources_for(31); #1; evaluate;
    check(issue_fire == 3 && issue_packet.lane0.rs1_value == 32'h12345678 &&
          issue_packet.lane0.rs2_value == 32'h89abcdef &&
          issue_packet.lane1.rs1_value == 32'h89abcdef &&
          issue_packet.lane1.rs2_value == 32'h12345678,
          "dual completion wakes both consumers and forwards all four source values"); cycle;

    // Wakeups are sticky even if issue is held; bad/nonwriter/zero completions do not wake.
    a_uop = operation(4, 0, 0); a_uop.prs1 = 40; a_uop.prs2 = 41;
    offer(a_uop, '0, 1); dispatch_src1_ready = 0; dispatch_src2_ready = 0; cycle;
    completion_valid = 1; completion.lane0.writes_rd = 1; completion.lane0.pdst = 40;
    completion.lane0.exception.valid = 1; resources_for(31); #1;
    check(issue_fire == 0, "fault completion does not wake a dependency"); cycle;
    completion_valid = 1; completion.lane0.pdst = 40; completion.lane0.writes_rd = 0; cycle;
    completion_valid = 1; completion.lane0.pdst = 0; completion.lane0.writes_rd = 1; cycle;
    issue_hold = 1; completion_valid = 3;
    completion.lane0.writes_rd = 1; completion.lane0.pdst = 40; completion.lane0.value = 444;
    completion.lane1.writes_rd = 1; completion.lane1.pdst = 41; completion.lane1.value = 555;
    resources_for(31); #1; check(issue_fire == 0, "issue hold blocks awakened transfer"); cycle;
    resources_for(31); #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.rs1_value == 444 &&
          issue_packet.lane0.rs2_value == 555,
          "stored readiness and values survive broadcast removal");
    cycle;

    // Unused/p0 operands are ready even when the incoming ready hints are false.
    a_uop = operation(5, 0, 0); a_uop.decoded.uses_rs1 = 0; a_uop.prs1 = 60;
    a_uop.prs2 = 0;
    offer(a_uop, '0, 1); dispatch_src1_ready = 0; dispatch_src2_ready = 0; cycle;
    resources_for(31); #1; evaluate;
    check(issue_fire == 1 && read0_rs1 == 0 && read0_rs2 == 0,
          "unused field and p0 create no false dependencies"); cycle;

    // A busy iterative engine does not block an eligible operation on the other execution port.
    queue_pair(operation(6, 1, 0), operation(7, 0, 1));
    resources_for(3); #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.uop.decoded.op_class == UOP_INTEGER,
          "busy mul/div leaves younger integer work eligible"); cycle;
    resources_for(4); #1; evaluate;
    check(issue_fire == 2 && issue_packet.lane1.uop.decoded.op_class == UOP_MULDIV,
          "mul/div issues on its port when its own engine becomes available"); cycle;

    // Compare every class pair against all 32 resource masks, including ROB age wrap.
    for (head_case = 0; head_case < 2; head_case = head_case + 1) begin
      for (kind0 = 0; kind0 < 7; kind0 = kind0 + 1) begin
        for (kind1 = 0; kind1 < 7; kind1 = kind1 + 1) begin
          flush_queue;
          rob_head_index = head_case == 0 ? 0 : 15;
          rob_head_tag.index = rob_head_index; rob_head_tag.generation = 9;
          a_uop = operation(10+kind0, kind0, rob_head_index);
          b_uop = operation(20+kind1, kind1, (rob_head_index + 1) % ROB_DEPTH);
          offer(a_uop, b_uop, 3); cycle;
          for (resources = 0; resources < 32; resources = resources + 1) begin
            rob_head_index = head_case == 0 ? 0 : 15;
            rob_head_tag.index = rob_head_index; rob_head_tag.generation = 9;
            resources_for(resources); #1; evaluate;
          end
        end
      end
    end
    flush_queue;

    // SYSTEM needs the full ROB head identity, and it issues alone.
    queue_pair(operation(30, 6, 0), operation(31, 0, 1));
    resources_for(31); rob_head_tag.generation = 10; #1; evaluate;
    check(issue_fire == 0, "wrong-generation system owner cannot issue as the head");
    rob_head_tag.generation = 9; #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.uop.decoded.op_class == UOP_SYSTEM,
          "matching head system instruction takes one exclusive issue slot"); cycle;
    check(occupancy == 0, "lane-0 system admission blocked its younger dispatch partner");
    queue_pair(operation(32, 0, 0), operation(33, 6, 1));
    resources_for(31); #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.uop.decoded.op_class == UOP_INTEGER,
          "younger system instruction waits for actual head ownership"); cycle;
    offer(operation(85, 0, 2), '0, 1); cycle;
    rob_head_index = 1; rob_head_tag.index = 1; resources_for(31); #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.uop.decoded.op_class == UOP_SYSTEM,
          "matching-head system issues alone despite a ready younger integer"); cycle;
    resources_for(31); cycle;

    // Known misprediction drains strict older ages only, including across 15->0 wrap.
    flush_queue;
    rob_head_index = 14; rob_head_tag.index = 14;
    offer(operation(34, 0, 14), operation(35, 1, 15), 3); cycle;
    rob_head_index = 14; rob_head_tag.index = 14;
    offer(operation(36, 0, 0), operation(37, 0, 1), 3); cycle;
    rob_head_index = 14; rob_head_tag.index = 14;
    mispredict_pending = 1; mispredict_tag.index = 0; mispredict_tag.generation = 9;
    resources_for(31); #1; evaluate;
    check(issue_fire == 3 && issue_packet.lane0.uop.rob_tag.index == 14 &&
          issue_packet.lane1.uop.rob_tag.index == 15, "strict older work drains across ROB wrap");
    offer(operation(38, 0, 2), '0, 1); #1;
    check(dispatch_fire == 0, "pending recovery blocks new admission"); cycle;
    rob_head_index = 14; rob_head_tag.index = 14;
    mispredict_pending = 1; mispredict_tag.index = 0; resources_for(31); #1; evaluate;
    check(issue_fire == 0 && occupancy == 2,
          "branch boundary and younger work wait for cancellation");
    flush_queue;

    // A current full queue cannot allocate slots that are being issued on this edge.
    for (n = 0; n < 4; n = n + 1)
      queue_pair(operation(40+n*2, 0, n*2), operation(41+n*2, 0, n*2+1));
    offer(operation(48, 0, 8), operation(49, 0, 9), 3); resources_for(31); #1; evaluate;
    check(issue_fire == 3 && dispatch_fire == 0,
          "full queue uses current capacity, not issue lookahead");
    cycle;
    offer(operation(48, 0, 8), operation(49, 0, 9), 3); #1;
    check(dispatch_fire == 3, "released issue entries accept a pair on the following cycle"); cycle;
    flush_queue;

    // One free entry accepts an older prefix; a fault consumes no issue entry.
    for (n = 0; n < 3; n = n + 1)
      queue_pair(operation(50+n*2, 0, n*2), operation(51+n*2, 0, n*2+1));
    offer(operation(56, 0, 6), '0, 1); cycle;
    offer(operation(57, 0, 7), operation(58, 0, 8), 3); #1;
    check(dispatch_fire == 1, "one free entry truncates a two-instruction offer"); cycle;
    check(occupancy == ISSUE_DEPTH, "accepted prefix fills the final entry");
    a_uop = operation(59, 0, 8); a_uop.decoded.op_class = UOP_NONE;
    a_uop.decoded.fetch.exception.valid = 1;
    offer(a_uop, operation(60, 0, 9), 3); #1; evaluate;
    check(dispatch_fire == 1 && !protocol_error,
          "valid fault is accepted without an issue slot and blocks its younger partner"); cycle;
    check(occupancy == ISSUE_DEPTH, "fault admission does not consume queue storage");
    flush_queue;

    // Fault in lane 1 can follow a real operation with only one physical queue slot needed.
    for (n = 0; n < 3; n = n + 1)
      queue_pair(operation(61+n*2, 0, n*2), operation(62+n*2, 0, n*2+1));
    offer(operation(67, 0, 6), '0, 1); cycle;
    a_uop = operation(68, 0, 7); b_uop = operation(69, 0, 8);
    b_uop.decoded.op_class = UOP_NONE; b_uop.decoded.fetch.exception.valid = 1;
    offer(a_uop, b_uop, 3); #1; evaluate;
    check(dispatch_fire == 3, "younger fault needs no second queue slot"); cycle;
    check(occupancy == ISSUE_DEPTH, "only the real instruction enters the queue"); flush_queue;

    // Duplicate owners and malformed NONE are diagnostics, while raw valid10 never transfers.
    queue_pair(operation(70, 0, 0), operation(71, 0, 1));
    a_uop = operation(72, 0, 0); offer(a_uop, '0, 1); #1; evaluate;
    check(protocol_error && dispatch_fire == 0,
          "existing full tag blocks duplicate admission"); cycle;
    a_uop = operation(73, 0, 2); b_uop = a_uop; offer(a_uop, b_uop, 3); #1;
    check(dispatch_fire == 1 && protocol_error, "duplicate within pair accepts only older owner");
    cycle;
    a_uop = operation(74, 0, 3); a_uop.decoded.op_class = UOP_NONE;
    offer(a_uop, '0, 1); #1;
    check(dispatch_fire == 0 && protocol_error,
          "nonfault NONE is not silently stored forever"); cycle;
    offer(operation(75, 0, 3), operation(76, 0, 4), 2'b10); #1;
    check(dispatch_fire == 0 && protocol_error,
          "younger-only offer never skips an older lane"); cycle;

    // Admission and issue holds, missing ROB context, and flush have explicit boundaries.
    offer(operation(77, 0, 3), '0, 1); dispatch_hold = 1; resources_for(31); #1; evaluate;
    check(dispatch_fire == 0 && issue_fire != 0,
          "admission hold does not stop existing ready work");
    cycle;
    queue_pair(operation(78, 0, 4), operation(79, 0, 5));
    offer(operation(80, 0, 6), '0, 1); issue_hold = 1; resources_for(31); #1;
    check(dispatch_fire == 0 && issue_fire == 0,
          "issue hold freezes both issue and admission"); cycle;
    resources_for(31); rob_head_valid = 0; #1;
    check(issue_fire == 0, "missing live ROB head context prevents age-based issue"); cycle;
    resources_for(31); flush = 1; offer(operation(81, 0, 6), '0, 1); #1;
    check(issue_fire == 0 && dispatch_fire == 0, "safe flush dominates current transfers"); cycle;
    check(occupancy == 0, "flush clears waiting owners");

    // New dispatch cannot issue immediately, but can capture a concurrent accepted wakeup.
    a_uop = operation(82, 0, 0); a_uop.prs1 = 45; a_uop.prs2 = 0;
    offer(a_uop, '0, 1); dispatch_src1_ready = 0; dispatch_src2_ready = 0;
    completion_valid = 2; completion.lane1.writes_rd = 1;
    completion.lane1.pdst = 45; completion.lane1.value = 32'habcdef01;
    resources_for(31); #1;
    check(issue_fire == 0, "no dispatch-to-issue combinational bypass"); cycle;
    resources_for(31); #1; evaluate;
    check(issue_fire == 1 && issue_packet.lane0.rs1_value == 32'habcdef01,
          "incoming operand readiness captures the same accepted completion"); cycle;

    // Four-entry cases make the independent pair oracle choose among multiple partners.
    for (batch = 0; batch < 32; batch = batch + 1) begin
      flush_queue;
      a_uop = operation(100+batch*4, batch % 6, 14);
      b_uop = operation(101+batch*4, (batch+1) % 6, 15);
      c_uop = operation(102+batch*4, (batch+2) % 6, 0);
      d_uop = operation(103+batch*4, (batch+3) % 6, 1);
      offer(a_uop, b_uop, 3); cycle;
      offer(c_uop, d_uop, 3); cycle;
      for (resources = 0; resources < 32; resources = resources + 1) begin
        rob_head_index = 14; rob_head_tag.index = 14; resources_for(resources);
        #1; evaluate;
      end
    end
    flush_queue;

    // Mixed resource/backpressure/wakeup traffic checks the physical slot bookkeeping.
    for (n = 0; n < 192; n = n + 1) begin
      neutral;
      rng = next_random(rng); resources_for(rng[4:0]);
      issue_hold = n % 19 == 0; dispatch_hold = n % 13 == 0;
      if (n % 5 != 0) begin
        a_uop = operation(300+n*2, n % 6, n % 16);
        b_uop = operation(301+n*2, (n+1) % 6, (n+1) % 16);
        a_uop.rob_tag.generation = 20+n/16;
        b_uop.rob_tag.generation = 20+(n+1)/16;
        offer(a_uop, b_uop, n % 3 == 0 ? 2'b01 : 2'b11);
        dispatch_src1_ready = n % 4 == 0 ? 0 : 3;
        dispatch_src2_ready = n % 7 == 0 ? 0 : 3;
      end
      completion_valid = 3;
      rng = next_random(rng); completion.lane0.writes_rd = 1;
      completion.lane0.pdst = 1 + rng % 31; completion.lane0.value = rng;
      rng = next_random(rng); completion.lane1.writes_rd = 1;
      completion.lane1.pdst = 1 + rng % 31; completion.lane1.value = rng;
      if (n % 47 == 46) flush = 1;
      cycle;
    end
    flush_queue;

    neutral; resources_for(31); offer(operation(900, 0, 0), operation(901, 0, 1), 3);
    flush = 1; rst_n = 0; #2;
    check(issue_fire == 0 && dispatch_fire == 0 && occupancy == 0 && !flush_done,
          "global reset wins held traffic and flush");

    $display("BLOCK06 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Block 6 issue-queue tests failed");
    $finish;
  end
endmodule
```

## Complete connected scheduling test

Source: `tb/tb_block06_integration.sv`.

```systemverilog
// Connected rename/ROB/PRF/IQ test. Execution values are deliberately injected.
`timescale 1ns / 1ps
module tb_block06_integration;
  import rv32_ooo_pkg::*;
  logic clk = 0, rst_n = 1;
  logic [1:0] front_valid, rename_ready, rename_fire, rob_ready, rob_fire;
  logic [1:0] iq_ready, iq_fire, src1_ready, src2_ready, wb_valid;
  logic [1:0] wb_ready, wb_accepted, wb_rejected, head_valid, head_complete;
  logic [1:0] head_ready, retire_fire, trap_fire, write_enable, issue_fire;
  decoded_pair_t decoded;
  renamed_pair_t renamed, retiring, producer_pair, independent_pair;
  rob_tag_pair_t tags;
  completion_pair_t wb;
  rob_entry_pair_t heads;
  retire_trace_pair_t events, trace;
  execute_packet_pair_t issued;
  phys_reg_t r00, r01, r10, r11, committed_phys;
  word_t v00, v01, v10, v11, mispredict_pc;
  phys_mask_t free_mask, ready_mask;
  phys_count_t free_count;
  rob_index_t head_index, query_age;
  rob_count_t rob_occupancy;
  issue_count_t iq_occupancy;
  logic [ISSUE_DEPTH-1:0] occupied;
  rob_tag_t mispredict_tag;
  logic alu0_ready, alu1_ready, md_ready, apply;
  logic rename_done, rob_done, iq_done, rename_error, rob_error, iq_error;
  logic trace_error, write_error, recovery, mispredict, serial_pending, fault_pending;
  logic query_live, full, empty;
  integer checks = 0, failures = 0;

  rv32_rename rename (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_valid_i(front_valid),
    .dispatch_resources_ready_i(rob_ready & iq_ready), .decoded_i(decoded),
    .dispatch_rob_tag_i(tags), .dispatch_ready_o(rename_ready), .dispatch_fire_o(rename_fire),
    .renamed_o(renamed), .src1_ready_o(src1_ready), .src2_ready_o(src2_ready),
    .completion_valid_i(wb_accepted), .completion_i(wb),
    .retire_valid_i(retire_fire), .retire_uop_i(retiring),
    .recover_i(apply), .recover_done_o(rename_done),
    .committed_query_arch_i(5'd5), .committed_query_phys_o(committed_phys),
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
    .head_ready_i(head_ready), .head_trap_target_i(32'h100),
    .retire_fire_o(retire_fire), .trap_fire_o(trap_fire), .retirement_event_o(events),
    .recovery_required_o(recovery), .mispredict_pending_o(mispredict),
    .mispredict_tag_o(mispredict_tag), .mispredict_next_pc_o(mispredict_pc),
    .serial_pending_o(serial_pending), .fault_pending_o(fault_pending),
    .flush_i(apply), .flush_done_o(rob_done), .query_tag_i(wb.lane0.rob_tag),
    .query_live_o(query_live), .query_age_o(query_age), .head_index_o(head_index),
    .occupancy_o(rob_occupancy), .full_o(full), .empty_o(empty), .protocol_error_o(rob_error)
  );
  assign write_enable[0] = wb_accepted[0] && wb.lane0.writes_rd && !wb.lane0.exception.valid;
  assign write_enable[1] = wb_accepted[1] && wb.lane1.writes_rd && !wb.lane1.exception.valid;
  rv32_physical_regfile prf (
    .clk_i(clk), .rst_ni(rst_n),
    .read0_rs1_i(r00), .read0_rs2_i(r01), .read1_rs1_i(r10), .read1_rs2_i(r11),
    .read0_data1_o(v00), .read0_data2_o(v01),
    .read1_data1_o(v10), .read1_data2_o(v11),
    .write_enable_i(write_enable), .write0_addr_i(wb.lane0.pdst),
    .write1_addr_i(wb.lane1.pdst), .write0_data_i(wb.lane0.value),
    .write1_data_i(wb.lane1.value), .write_conflict_o(write_error)
  );
  rv32_issue_queue iq (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_hold_i(1'b0), .issue_hold_i(recovery),
    .dispatch_valid_i(rename_fire), .dispatch_uop_i(renamed),
    .dispatch_src1_ready_i(src1_ready), .dispatch_src2_ready_i(src2_ready),
    .dispatch_ready_o(iq_ready), .dispatch_fire_o(iq_fire),
    .completion_valid_i(wb_accepted), .completion_i(wb),
    .rob_head_valid_i(head_valid[0]), .rob_head_index_i(head_index),
    .rob_head_tag_i(heads.lane0.uop.rob_tag),
    .mispredict_pending_i(mispredict), .mispredict_tag_i(mispredict_tag),
    .alu0_ready_i(alu0_ready), .alu1_ready_i(alu1_ready), .muldiv_ready_i(md_ready),
    .memory_ready_i(1'b0), .system_ready_i(1'b0),
    .issue_fire_o(issue_fire), .issue_packet_o(issued),
    .read0_rs1_o(r00), .read0_rs2_o(r01), .read1_rs1_o(r10), .read1_rs2_o(r11),
    .read0_data1_i(v00), .read0_data2_i(v01),
    .read1_data1_i(v10), .read1_data2_i(v11),
    .flush_i(apply), .flush_done_o(iq_done), .occupancy_o(iq_occupancy),
    .occupied_mask_o(occupied), .protocol_error_o(iq_error)
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
      check(rename_fire == rob_fire && rename_fire == iq_fire,
            "atomic dispatch agrees across rename, ROB, and IQ");
      check(!rename_error && !rob_error && !iq_error && !trace_error && !write_error,
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

  initial begin #10000; $fatal(1, "Connected scheduler test timed out"); end
  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block06/block06_integration.vcd");
      $dumpvars(0, tb_block06_integration);
    end
    front_valid = 0; decoded = '0; wb_valid = 0; wb = '0; apply = 0;
    head_ready = 0; alu0_ready = 0; alu1_ready = 0; md_ready = 0;
    #1; rst_n = 0; #2; rst_n = 1; #1;
    check(empty && iq_occupancy == 0 && free_count == 32, "coupled startup state");

    decoded.lane0 = operation(0, 5, 1, 2);
    decoded.lane0.op_class = UOP_MULDIV;
    decoded.lane0.muldiv_op = MD_DIV;
    decoded.lane0.fetch.instruction = 32'h0220c2b3;
    decoded.lane1 = operation(4, 7, 5, 1);
    front_valid = 2'b11; #1; producer_pair = renamed;
    check(rename_fire == 2'b11 && !src1_ready[1],
          "intra-pair RAW captures the producer's new unready physical tag");
    edge_step; front_valid = 0; md_ready = 1; #1;
    check(issue_fire == 2'b10 && issued.lane1.uop == producer_pair.lane0,
          "older DIV launches on the multiply/divide path");
    edge_step; md_ready = 0; #1;
    check(iq_occupancy == 1 && issue_fire == 0,
          "dependent older ADD waits after DIV leaves the issue queue");

    decoded.lane0 = operation(8, 6, 3, 4);
    decoded.lane1 = operation(12, 8, 3, 4);
    front_valid = 2'b11; #1; independent_pair = renamed;
    check(rename_fire == 2'b11, "two independent younger instructions dispatch together");
    edge_step; front_valid = 0; alu0_ready = 1; alu1_ready = 1; #1;
    check(issue_fire == 2'b11 && issued.lane0.uop == independent_pair.lane0
          && issued.lane1.uop == independent_pair.lane1,
          "two younger ready ADDs bypass the older waiting ADD");
    check(issued.lane0.rs1_value == 0 && issued.lane1.rs2_value == 0,
          "issued packet carries PRF operand values as well as identities");
    edge_step; alu0_ready = 0; alu1_ready = 0; #1;
    check(iq_occupancy == 1 && rob_occupancy == 4,
          "issue removes queue entries without removing ROB owners");

    wb.lane0 = completed(independent_pair.lane0, 7);
    wb.lane1 = completed(independent_pair.lane1, 9); wb_valid = 2'b11; #1;
    check(wb_accepted == 2'b11 && issue_fire == 0,
          "both younger results are accepted while the RAW consumer still waits");
    edge_step; wb_valid = 0; #1;
    check(head_complete == 0 && retire_fire == 0 && committed_phys == 5,
          "younger completion cannot commit past incomplete DIV and dependent ADD");

    wb.lane0 = completed(producer_pair.lane0, 999);
    wb.lane0.rob_tag.generation = wb.lane0.rob_tag.generation + 1'b1;
    wb_valid = 2'b01; alu0_ready = 1; alu1_ready = 1; #1;
    check(wb_rejected == 2'b01 && write_enable == 0 && issue_fire == 0,
          "stale generation cannot write PRF or wake the waiting physical source");
    edge_step; wb_valid = 0; #1;
    check(iq_occupancy == 1 && !ready_mask[producer_pair.lane0.pdst],
          "rejected completion leaves readiness and queue ownership intact");

    wb.lane0 = completed(producer_pair.lane0, 40); wb_valid = 2'b01; #1;
    check(wb_accepted == 2'b01 && issue_fire == 2'b01,
          "accepted producer completion wakes the dependent in the same cycle");
    check(issued.lane0.uop == producer_pair.lane1
          && issued.lane0.rs1_value == 40 && issued.lane0.rs2_value == 0,
          "four-port PRF bypass supplies the newly awakened source before the edge");
    edge_step; wb_valid = 0; alu0_ready = 0; alu1_ready = 0; #1;
    check(iq_occupancy == 0 && rob_occupancy == 4 && head_complete == 2'b01,
          "completion and issue remain distinct from retirement");

    wb.lane0 = completed(producer_pair.lane1, 40); wb_valid = 2'b01;
    edge_step; wb_valid = 0; head_ready = 2'b11; #1;
    check(retire_fire == 2'b11 && events.lane0.pc == 0 && events.lane1.pc == 4,
          "first retirement pair is DIV then its consumer despite younger execution");
    edge_step;
    check(trace.lane0.order == 0 && trace.lane1.order == 1
          && trace.lane0.rd == 5 && trace.lane1.rd == 7,
          "trace records ordered architectural results for the first pair");
    check(retire_fire == 2'b11 && events.lane0.pc == 8 && events.lane1.pc == 12,
          "independent younger pair retires only after older pair");
    edge_step;
    check(empty && iq_occupancy == 0 && free_count == 32 && committed_phys == 32,
          "all owners retire and old physical mappings return to the free list");
    check(trace.lane0.order == 2 && trace.lane1.order == 3
          && trace.lane0.rd_value == 7 && trace.lane1.rd_value == 9,
          "younger results retain their values and retirement order");

    apply = 1; edge_step; apply = 0; #1;
    check(rename_done && rob_done && iq_done && empty && iq_occupancy == 0,
          "one safe recovery application clears all built speculative structures");
    $display("BLOCK06_INTEGRATION SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Connected scheduler test failed");
    $finish;
  end
endmodule
```
