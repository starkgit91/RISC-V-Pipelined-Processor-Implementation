# Block 1: the common language between processor blocks

This block defines the formats used to carry instructions and results through the processor.
The package and its testbench are complete for this block. They are not a complete processor.
Later blocks will implement the circuits that allocate, store, inspect, and update these records.

## 1. Where this block operates

Our design has five logical architectural sections, not a fixed five-clock instruction latency.

```text
 IF                 ID                    EX                 MEM              WB
 Fetch       Decode -> Rename       Schedule -> Execute   Ordered access   Complete -> Commit
   |                  |                      |                  |                 |
 fetch_packet_t -> decoded_uop_t -> renamed_uop_t -> execute_packet_t       completion_t
                        |                    |                  |                 |
                        +---------- ROB keeps program order ----+----> retire_trace_t
```

These arrows show record ownership and data flow, not a complete wiring diagram. A load/store
uses the memory path; an ADD does not issue a memory transaction. Completion informs both the
physical register file and ROB. Retirement is a separate decision made at ROB head.

There are no runtime input or output pins on a SystemVerilog package. A module imports the
definitions when compiled. Later module ports carry records of these types and separate
valid/ready signals indicate when a record is meaningful and when it transfers.

## 2. Configuration and hardware dimensions

The IF interface now includes a 64-bit fetch_row_t, a three-bit fetch_count_t for the four-entry
instruction buffer, and paired prediction records. Shared RAM contains 8,192 64-bit instruction
rows; the native row address is 13 bits. These types are implemented/tested in block 9.

Five held completion sources (two integer, multiply/divide, later memory, later system) now
share two registered writeback slots. Their packed completion_sources_t bundle uses independent
valid/ready bits and does not change dispatch/retirement width.

The active configuration is two-wide: fetch, dispatch, issue, completion, and retirement each
support up to two instructions/results per cycle. Named lane0/lane1 packed pairs preserve
per-instruction fields; lane 0 is older for dispatch and retirement. The execution-packet pair
uses lane numbers for execution ports instead: a flexible older integer operation may use
port 1 while a younger branch uses port 0. Validity is a separate
prefix mask (00, 01, or 11). The fetch buffer holds four instructions, or two full groups.

| Name | Value | What later hardware uses it for |
|---|---:|---|
| XLEN | 32 | Operand, result, address, and instruction width |
| ARCH_REG_COUNT | 32 | Program-visible x0 through x31 |
| PHYS_REG_COUNT | 64 | Storage for committed and speculative register versions |
| ROB_DEPTH | 16 | Maximum instructions tracked in the reorder buffer |
| ISSUE_DEPTH | 8 | Instructions waiting to issue |
| MEMORY_DEPTH | 4 | Load/store records waiting for preparation or completion |
| FETCH_DEPTH | 4 | Buffer four fetched instructions, or two full two-instruction groups |
| BTB_ENTRIES | 64 | Remember branch/jump PCs and target addresses |
| PHT_ENTRIES | 128 | Two-bit branch-direction counters |

The source derives register tag widths with `$clog2`. Naming 32 architectural registers takes
5 bits; naming 64 physical registers takes 6 bits. A 16-entry ROB slot index takes 4 bits.

The configuration does not instantiate RAMs or registers. For example, a later declaration of
an array of 64 `word_t` elements will describe the physical register storage. Constants can set
its size without describing the read multiplexers or write logic, which belong to block 3.

The 64 KiB RAM contains 16,384 words of four bytes. Requests remain byte-addressed. The memory
adapter will translate the address to the BRAM's word location; callers must not pre-divide
the address and then accidentally divide it again.

## 3. Architectural registers, physical registers, and instruction identity

An architectural register is a name in the program. A physical register is storage for a
particular version of a value. A ROB tag identifies an instruction, not a register.

Consider two instructions that both write x5:

```asm
div x5, x1, x2
add x5, x3, x4
```

Illustrative mappings, assuming p37 and p38 are available:

| Instruction | Architectural destination | New physical destination | Previous mapping |
|---|---|---|---|
| DIV | x5 | p37 | p5 |
| ADD | x5 | p38 | p37 |

The DIV and ADD keep separate values. A later instruction reading x5 after the ADD is renamed
to read p38. An instruction between the two writers would retain p37 as its source. This is
how renaming removes false dependencies without changing program semantics.

The numbers p37/p38 are examples, not a required allocator sequence. The testbench constructs
these records directly; it does not yet implement allocation or execute either instruction.

`writes_rd` means a real write to a nonzero architectural register. The decoder will clear it
for rd=x0; renaming must never allocate a physical destination for x0. The register file will
also independently protect physical p0 so correctness does not depend on one control signal.

### A conceptual timeline

| Event | Meaning |
|---|---|
| Rename DIV | Capture x1/x2 source mappings; assign x5 a new destination p37 |
| Rename ADD | Capture x3/x4 mappings; replace x5's speculative mapping with p38 |
| Issue DIV | Divider starts working on its captured operand values |
| Issue ADD | Independent integer work can proceed while DIV is busy |
| Complete ADD | p38 becomes ready; ADD's ROB entry becomes complete |
| DIV still running | ADD cannot retire past the older incomplete DIV |
| Complete and retire DIV | Committed x5 becomes p37; old committed p5 is released |
| Retire ADD | Committed x5 becomes p38; old committed p37 is released |

These are intended behavior examples for future blocks, not measured cycle counts from this
block's declaration-only simulation. The distinction between readiness and commitment is central:
ready means a value can feed dependent instructions; committed means it belongs to architectural state.

## 4. Field map: which block produces and consumes each record

| Record / fields | Width or shape | Produced by | Consumed by | Meaning |
|---|---|---|---|---|
| `word_t` | 32 bits | Datapaths and memories | Datapaths and memories | Raw data/address/instruction bits; signedness is selected by the operation |
| `arch_reg_t` | 5 bits | Decoder | Rename and retirement | Program register name |
| `phys_reg_t` | 6 bits | Rename | Scheduler, register file, completion | Internal storage identifier |
| `rob_tag_t.index` | 4 bits | ROB allocator | Queues and completion | Slot occupied by this instruction |
| `rob_tag_t.generation` | 8 bits | ROB allocator | Queues and completion | Distinguish recycled uses of a slot |
| `exception_t` | valid + 32-bit cause + 32-bit tval | Fetch/decode/execute/memory | ROB and trap control | Fault ownership and details |
| `prediction_t` | next PC + flags + 7-bit PHT index | Fetch predictor | Execution and retirement | What was predicted, and which predictor counter was used |
| `fetch_packet_t` | PC + instruction + prediction + exception | IF | Decoder | One fetched instruction or fetch fault |
| `decoded_uop_t` | Fetch metadata + controls + architectural sources/destination | Decoder | Rename and execution paths | Internal operation description |
| `renamed_uop_t` | Decoded operation + ROB tag + four physical tags | Rename | Scheduler, execution, ROB | Captured sources, new destination, and old destination |
| `issue_entry_t` | valid + renamed operation + readiness flags | Dispatch/wakeup | Scheduler | Instruction waiting for a suitable execution engine |
| `execute_packet_t` | Renamed operation + two 32-bit values | Register read | Execution engine | Resolved operand values, not only their tags |
| `resolved_control_t` | Classification, outcome, mismatch, next PC | Execution | ROB/recovery/predictor update | Actual control-flow result |
| `completion_t` | ROB/physical identity + value + exception/control result | Execution or memory | WB arbiter, PRF, ROB | Speculative completion; not an architectural commit |
| `rob_entry_t` | Valid/completion state + original operation + result/fault/control | Dispatch and completion | Retirement/recovery | Instruction state in program order |
| `memory_entry_t` | Identity + prepared address/data + access/control metadata | Dispatch/address generation | Memory engine | Load/store preparation and response ownership |
| `data_request_t` | write + 32-bit address/data + four strobes | LSU | Native/AXI adapter | One aligned word transaction with selected write bytes |
| `memory_response_t` | 32-bit data + error | Memory adapter | Fetch/LSU | Completion of a request, including errors |
| `retire_trace_t` | Architectural event, operands/results, optional store information | Retirement/trap controller | Testbench/debug | What actually became architecturally visible |

The ROB stores the original renamed operation so retirement has the instruction PC, architectural
destination, and old/new mappings. It also has a current exception field: initialize that from
the incoming fetch/decode fault, then update it if execution discovers a fault. The nested incoming
metadata remains the original record; the current ROB exception is authoritative at retirement.

Every completion must supply the actual successor PC. For an ordinary arithmetic, multiply/divide,
or memory instruction this is pc+4. Leaving it zero would corrupt committed-PC tracking and
interrupt return addresses. A cleared record is an empty default, not a valid completion result.

Memory responses have no transaction ID because each port permits only one outstanding request.
The requesting engine keeps that request's tag and access metadata until it consumes the response.
This restriction must be enforced by the later memory engine and adapters.

## 5. Important contracts the later circuits must enforce

### Validity is not the instruction value

A zeroed queue entry has `valid=0`. Its other bits must not cause execution, allocation, or
retirement. Do not infer validity from instruction bits. An actual fetched all-zero instruction
is illegal; an invalid queue entry is simply empty. Canonical NOP is `0x00000013`.

Handshake payloads do not contain another `valid` bit. Their interface has a separate valid signal.
Stored queue entries do contain validity because the owning queue must remember occupied slots.

### Tags are not values

`prs1=37` means read p37, not use decimal 37 as an operand. `rs1_value` is the actual 32-bit operand
captured for execution. Keeping these names different makes waveforms easier to read.

### Source-use flags matter

For ADDI, instruction bits [24:20] are part of the immediate, not a second register source. The
decoder sets `uses_rs2=0`, so the scheduler must not wait for a nonexistent second dependency.
Similarly, immediate CSR operations use `csr_immediate`, not the contents of an integer register.

### Completion is not commitment

An execution result may wake younger instructions while an older instruction still blocks retirement.
Only the committed rename map defines architectural register contents. A testbench comparing the
program's x5 must consult committed state or retirement events, not an arbitrary speculative PRF entry.

### Generation matching is necessary but not sufficient

A generation field eventually wraps. Recovery must cancel or drain old operations and their held
completions before reusing their ROB/physical resources. A generation counter is not a substitute
for this cancellation guarantee. Block 11 will implement the recovery logic.

### Memory acceptance is not completion

`data_request_t` has aligned byte addresses and byte strobes. A byte store at lane 2 can carry
`wstrb=0100` and its byte in `wdata[23:16]`. This represents a transfer; it does not yet implement
alignment, extraction, or AXI handshaking. The LSU and AXI adapter implement those later.

The original effective address remains in the memory entry for byte extraction and trap reporting.
Memory requests will start only at ROB head. A store cannot retire just because its request was accepted.

### Internal operation codes are not instruction encodings

`ALU_ADD` is a compact internal selector, not the 32-bit encoding of ADD. Most enum encodings are
private implementation details. M-extension and CSR-operation enum values intentionally match their
funct3 fields after the decoder validates the full instruction. Unsupported encodings still trap.

## 6. What exists at circuit level after this block

The package describes widths and layouts. It does not instantiate a clocked register, memory,
scheduler, divider, or comparator. A later queue module will use these types to declare storage
and write `always_ff` logic. A packed record can travel over a group of wires or be stored in
flip-flops; `packed` makes the bit layout definite, rather than choosing an implementation technology.

Use one shared package instead of independently redefining ports in each module. A mismatch in
destination width or tag ordering could otherwise connect the wrong result to a register.

Clock targets, CSR addresses, and exception causes are also definitions, not implementations.
For example, declaring `CSR_MTVEC` does not create a trap-vector register or enforce its write rules.

## 7. Verification and waveform expectations

Run `python3 scripts/run_checks.py --block 1` from the project directory. The testbench checks:

- Agreed dimensions, derived tag widths, clock constants, and memory windows.
- Every ROB index across several generations.
- Distinct physical destinations for two modeled writers of x5.
- Full packed-record round trips, including nested fields.
- Preservation of exception and completion identity information.
- Invalid/empty queue representation and byte-strobe representation.

These checks establish representation contracts. They do not prove a scheduler, register allocator,
or CPU correct. The records are filled directly by the testbench. Simulation ends at time zero,
because no clocked logic exists in this block. A cycle waveform is therefore not useful yet;
later module tests will generate clocked waveforms and measured instruction traces.

The README includes equivalent `xvlog`, `xelab`, and `xsim` commands for your Linux Vivado install.
Icarus compilation/simulation is verified here; Vivado has not been run here.

With 64 physical registers and a 16-entry ROB, normal full-core operation cannot use up all
32 initially free physical registers: at most 16 unretired destinations are allocated. Later
free-list exhaustion tests will therefore exercise the allocator unit with controlled availability
or a smaller test configuration, rather than claim an unreachable full-core scenario was tested.

## 8. Interview questions and model answers

**Why have more physical registers than architectural registers?**
To keep separate in-flight versions of a program register, removing WAR and WAW false dependencies.

**Does renaming remove RAW dependencies?**
No. A consumer still needs its producer's actual value. The physical source tag identifies that producer.

**Why save `old_pdst`?**
The old mapping remains needed by older instructions and committed state. It can be released when
the replacement instruction retires, rather than when the replacement merely completes.

**Why not identify an instruction only by ROB slot number?**
Slots are reused. A tag includes a generation, and cancellation rules prevent late old results from
being accepted for a new occupant.

**Why is a completed instruction not necessarily retired?**
An older instruction might still be running or might fault. In-order retirement preserves precise
architectural state while allowing execution to occur out of order.

**Does a SystemVerilog struct automatically create hardware storage?**
No. It defines a bit layout. Storage and combinational behavior come from declarations and logic
that use the type inside actual modules.

**Can this block generate a processor bitstream by itself?**
No. This is the shared foundation. Subsequent lessons implement the processor modules;
FPGA integration is outside the revised project scope.

## 9. References and coding style

This is original project source. The following sources informed its conventions and explanations:

- [lowRISC Verilog coding style](https://github.com/lowRISC/style-guides/blob/master/VerilogCodingStyle.md):
  readable names, consistent formatting, and explicit combinational/sequential intent.
- [BOOM rename-stage documentation](https://docs.boom-core.org/en/latest/sections/rename-stage.html):
  physical versions, maps, readiness, old destinations, and committed-map recovery.
- [RISC-V ISA specifications](https://docs.riscv.org/reference/isa/): architectural names and encodings.

The custom core will use conservative retirement-based branch recovery. This is a project design
choice, not a claim that it implements BOOM's checkpointed branch-recovery mechanism.
