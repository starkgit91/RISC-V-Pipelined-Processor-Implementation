# Review of the supplied five-stage processor RTL

The RTL has the structure of a five-stage, single-issue, in-order processor, but it does not implement the full functionality requested and has a reproducible correctness bug even within its small instruction subset. It is best described as an educational RV32I-subset prototype.

This review covers the exact 469-line file supplied. The source was not modified. An unchanged copy, a separate simulation testbench, and the simulation results accompany this report.

## Verification performed

The supplied RTL compiled with Icarus Verilog 13.0 using SystemVerilog mode. The embedded demonstration program produced the expected register and memory values, one load-use stall, and one taken branch. Additional directed simulations reproduced incorrect results, missing instructions, unnecessary stalls, and an undriven control signal.

The testbench contains 36 checks: 18 passed and 18 failed. Several failures deliberately exercise features the prototype does not implement; these counts are not an ISA compliance score or exhaustive verification. Synthesis, timing closure, FPGA operation, formal verification, and full architectural compliance were not tested.

## Requested capabilities

| Capability | Finding |
|---|---|
| IF, ID, EX, MEM, WB | Present. Branch decisions are made in EX. |
| IF/ID, ID/EX, EX/MEM, MEM/WB registers | Present. |
| Fetch one instruction every cycle | Possible only while the front end is not stalled or redirected. |
| CPI = 1 | Ideal steady-state throughput is possible; CPI = 1 is not guaranteed. |
| RAW handling | Partial: forwarding and a load-use interlock exist, but WB/ID same-edge dependencies fail. |
| WAR and WAW handling | Prevented by this fixed, in-order register-read/write schedule; separate detection is unnecessary. |
| Forwarding | EX/MEM and MEM/WB values feed EX operands, including branch comparisons and store data. Newer EX/MEM results have priority. |
| Stalling | A one-cycle load-use stall holds PC and IF/ID and inserts an ID/EX bubble. Detection produces false stalls. |
| Control hazards | Taken BEQ redirects fetch and flushes the two younger instructions. Other branch conditions and jumps are missing. |
| Structural hazards | Separate instruction/data memories and a two-read/one-write register file avoid the principal conflicts for the supported operations. There is no general arbitration or resource-busy handling. |
| Register renaming | Absent and unnecessary for this microarchitecture. Renaming does not remove RAW dependencies. |
| Dynamic branch prediction | Absent. Fetch behaves like static predict-not-taken. |
| BTB / PHT / branch-history registers | Absent. |
| Dedicated loop predictor / return-address stack | Absent; optional enhancements rather than base ISA requirements. |

## Instruction support

The intended subset is ADD, SUB, ADDI, LW, SW, and BEQ. These have datapaths and control, subject to the correctness limitations below. Canonical NOP, encoded as ADDI x0,x0,0, works. The I-, S-, and B-immediate layouts in the immediate generator are correct for these uses.

| Instruction or category | Finding |
|---|---|
| ADD, SUB, ADDI | Implemented, subject to the WB/ID RAW bug. |
| SUBI | No standard RISC-V machine instruction with this name. Use ADDI with a negated immediate when encodable, or an instruction sequence. |
| MUL, DIV | Missing; their encodings currently execute ADD. |
| LW, SW | Implemented for aligned 32-bit words in the modeled memory. |
| LD, SD | Not implemented. Standard integer LD/SD are RV64I instructions; this is a 32-bit datapath. |
| LB, LBU, LH, LHU, SB, SH | Missing; load/store funct3 is ignored, so these encodings incorrectly access full words. |
| SLT, SLTU, SLTI, SLTIU | Missing. |
| SLL, SRL, SRA, SLLI, SRLI, SRAI | Missing. |
| AND, OR, XOR, ANDI, ORI, XORI | Missing. NOT can be expressed with XORI -1 once XORI is implemented. |
| BEQ / BEQZ | BEQ exists; BEQZ is assembler shorthand for BEQ rs,x0,target. |
| BNE / BNEZ | Missing. BNEZ is the usual RISC-V spelling, rather than BNEQZ. All branch encodings currently behave as BEQ. |
| BLT, BGE, BLTU, BGEU | Missing. |
| JAL, JALR | Missing, including PC+4 link writeback and indirect-jump target handling. |
| J / JMP | Standard RISC-V uses J, an alias for JAL x0,target; the required JAL implementation is missing. |
| LUI, AUIPC | Missing. |
| IF/ELSE, FOR, WHILE, LOOP | High-level constructs or labels, not dedicated base instructions. They are implemented using arithmetic, conditional branches, and jumps. Some can be manually encoded using the existing subset, but general compiler output is not supported. |
| FENCE | No explicit decode. With these simple, blocking, in-order memories, no extra barrier datapath may be needed; ordering must be specified and verified before claiming compliance. |
| ECALL, EBREAK | Missing; no environment-trap implementation. |

RISC-V and MIPS32 are different ISAs. A selection of similar operation names does not make a processor compatible with MIPS32 binaries.

## Concrete defects and limitations

### 1. Same-edge WB/ID RAW correctness bug

The register file writes on the rising edge using a nonblocking assignment, while ID/EX captures the register-file read values on that same rising edge. A consumer in ID therefore captures the previous value when its producer is in WB. During the consumer's following EX cycle, that producer has already left WB, so the existing EX forwarding cannot repair the stale value.

Reproducer:

```asm
addi x1, x0, 7
nop
nop
addi x2, x1, 1
```

Expected x2 = 8; simulation produces x2 = 1. Separate tests show stale store data and incorrect BEQ decisions at the same producer-consumer spacing.

Source locations: ID/EX operand capture at original lines 154–155; register reads/writes at 407–414. A WB-to-ID read bypass for both register operands is a direct remedy. A deliberately designed alternative register-file clocking arrangement can also work, but should not be assumed from this RTL.

### 2. Instruction decoding accepts unsupported operations with incorrect semantics

The control unit never uses funct3, and R-type operation selection only checks funct7[5]. Consequently:

- AND, OR, XOR, SLT, SLL, MUL, and DIV commonly execute ADD.
- SRL executes ADD and SRA executes SUB.
- All immediate arithmetic/logical encodings execute ADDI behavior.
- BNE and signed/unsigned relational branches execute equality comparisons.
- Byte/halfword loads and stores execute word accesses.

Directed simulation with operands 3 and 1 produced MUL = 4 rather than 3, AND = 4 rather than 1, and SLL = 4 rather than 6. Source: original lines 344–395. Decode must distinguish each supported encoding and provide defined handling for unsupported instructions.

### 3. Load-use interlock creates false dependencies

Original line 306 compares both instruction fields against ex_rd without checking whether the decoded instruction actually reads each source. Immediate bits [24:20] are not rs2 for ADDI. The detector also fails to exclude a load destination of x0.

Both examples below inserted an unnecessary stall in simulation:

```asm
lw   x1, 0(x0)
addi x2, x0, 1       # Immediate happens to resemble rs2=x1.
```

```asm
lw   x0, 0(x0)
addi x2, x0, 1       # x0 has no register-result dependency.
```

Use decoded source-use information and ex_rd != 0. A load to x0 must still perform any memory access side effects or faults defined by the execution environment.

### 4. Undriven flush_id

flush_id is declared at original line 45 and used at line 78 but is never driven. It floats in simulation. The demonstration still flushes taken branches through pc_src, but the unused floating input should be removed or explicitly connected.

Other declared signals are unused or undriven, including id_jump. This declaration does not implement jumps; the unused ex_alu_src/ex_alu_op wires also do not invalidate the actual ALU path, which directly uses the ID/EX registers.

### 5. Memory addressing silently discards alignment bits

Both memory models index with addr[31:2]. A misaligned LW/SW therefore accesses a rounded-down word rather than being correctly emulated or rejected. Instruction fetch similarly ignores the low address bits. RV32I without compressed instructions needs four-byte instruction alignment; a taken branch target offset by two bytes needs defined fault handling.

Each memory contains only 64 words, or 256 bytes. There are no bounds checks, access-fault responses, or memory-protection checks. Out-of-range reads may become unknown in simulation. Data memory starts uninitialized unless written; that is a normal modeling choice, not an ISA violation by itself.

### 6. Zero is an internal bubble here, not an architectural NOP

IF/ID flush writes instruction 0. The current default decoder makes this act inert, but 0x00000000 is not a legal base RISC-V NOP. Use per-stage valid bits to distinguish bubbles from real instructions, or consistently inject the canonical NOP 0x00000013 where appropriate. Valid bits are particularly valuable when adding traps, prediction, memory handshakes, and retirement tracing; they are not the only possible correct bubble implementation.

## CPI and branch prediction

For N useful instructions on a hazard-free path, an ideal five-stage pipeline takes approximately N+4 cycles from first fetch through last writeback. Its throughput approaches one instruction per cycle after filling.

In this implementation an immediately dependent load consumer incurs one bubble; a taken BEQ resolving in EX discards two younger instructions. False interlocks add more bubbles. Fetching instructions each cycle does not imply retiring useful instructions each cycle.

The next-PC mux chooses PC+4 until ex_branch and ex_zero indicate a taken branch. This is static predict-not-taken behavior; there is no learned prediction and no history of loop iterations or previous if/else outcomes.

To meet the requested dynamic-prediction scope, add:

1. A fetch-time prediction path selecting sequential PC or predicted target.
2. A BTB with valid bits, branch-PC identification/tags, and targets.
3. A direction predictor/PHT, for example two-bit saturating counters. Global/local history is optional, depending on predictor choice.
4. Prediction metadata carried with each in-flight branch, including enough information to recover its predicted next PC and training index.
5. EX resolution that compares actual versus predicted next PC, redirects on a mismatch, and squashes younger instructions.
6. Defined predictor update/reset behavior and handling of stalls, bubbles, and squashed work.

A basic two-bit predictor can learn recurring branch behavior but does not understand source-level loops or guarantee correct prediction of loop exits or arbitrary if/else conditions. Prediction is optional for architectural correctness and cannot guarantee CPI = 1 for every program.

## Remaining implementation scope

For a practical small core matching the requested functionality:

1. Fix the WB/ID bug, false interlocks, and undriven signals.
2. Specify the ISA precisely. RV32IM is a natural target for 32-bit integer code with hardware multiplication/division. If literal 64-bit LD/SD are required, choose a 64-bit ISA/datapath such as RV64IM instead.
3. Complete decoding and the missing integer operations, branches, jumps, U/J immediates, byte/halfword accesses, and load extension/store byte enables.
4. Add multiply/divide hardware and control. A multicycle unit needs readiness/busy handling and stalls or another explicit dependency/completion scheme. Full RV32M includes MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU. Signed division, zero divisors, and signed overflow require the specified results; RISC-V integer division by zero does not require an arithmetic trap.
5. Define exceptions and the execution environment: unsupported/illegal instructions, instruction/data alignment, access errors, ECALL, EBREAK, fault PC, squash behavior, and suppression of younger side effects. If implementing machine-mode trap handling, add the associated CSRs and return/control logic. CSR instructions belong to Zicsr, not automatically to RV32I.
6. Add memory request/response handshakes and whole-pipeline backpressure if memories can take variable time. The supplied memories are asynchronous-read arrays, so the design cannot simply be attached to a synchronous FPGA block RAM, cache, or external bus without revisiting timing and control. A shared instruction/data port would additionally need arbitration or structural stalls.
7. Add the requested BTB/PHT prediction and recovery machinery.
8. Provide program-loading and system integration appropriate to the target: reset/boot address, a memory map, and bus or memory ports. The current top level exposes only clk/reset and embeds its program; it is not a reusable system-facing core interface.
9. Verify the expanded design with directed hazard/control tests, ISA/reference-model comparison, assertions, and synthesis/timing checks. A retirement/commit trace helps measure CPI correctly and compare against a reference model.

Interrupts/timers, performance counters, debug support, return prediction, caches, store buffers, MMUs/TLBs, privilege levels, compressed instructions, atomics, floating point, vectors, superscalar issue, and out-of-order execution are additional choices. They are not all requirements for a basic five-stage core. A small in-order core also does not need hardware register renaming, a reorder buffer, or a special stack/loop/if instruction.

## References

- [RISC-V RV32I base ISA](https://docs.riscv.org/reference/isa/v20260120/unpriv/rv32.html): integer instructions, branches, jumps, memory access, FENCE, ECALL, EBREAK.
- [RISC-V M extension](https://docs.riscv.org/reference/isa/v20260120/unpriv/m-st-ext.html): integer multiply/divide instructions and exceptional arithmetic inputs.
- [RISC-V RV64I](https://docs.riscv.org/reference/isa/v20260120/unpriv/rv64.html): integer LD/SD and 64-bit behavior.
- [RISC-V assembly programmer's manual](https://github.com/riscv-non-isa/riscv-asm-manual/blob/main/src/asm-manual.adoc): pseudoinstructions including BEQZ, BNEZ, J, and RET.
