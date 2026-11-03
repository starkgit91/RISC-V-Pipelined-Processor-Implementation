# Block 3, two-issue revision: four reads, two writes, and completion bypass

This is the active two-issue register file. The earlier two-read/one-write lesson and verified
source are preserved under `reference/single_issue`.

## 1. Start with the same-cycle example

Suppose p37 stores 7 and p38 stores 8. Two accepted completions supply p37=99 and p38=123.
Instruction lane 0 reads p37 and p38; instruction lane 1 can read the same pair or two other tags.
Before the rising edge, every matching read port selects its corresponding completion value.
At the edge, both distinct physical destinations update. After write enable falls, the stored
99 and 123 remain available.

This gives two two-source instructions enough operand bandwidth in the same cycle. It does
not make dependent instructions independent: the scheduler must still wait for producer readiness.

## 2. Ports and connections

| Ports | Width | Source/destination | Role |
|---|---:|---|---|
| clk_i / rst_ni | 1 each | Common clock/global reset | Edge writes and asynchronous clear |
| read0_rs1_i / read0_rs2_i | 6 each | First issued instruction | Physical source tags |
| read1_rs1_i / read1_rs2_i | 6 each | Second issued instruction | Physical source tags |
| read0_data1_o / read0_data2_o | 32 each | First execution operand path | Values, including completion bypass |
| read1_data1_o / read1_data2_o | 32 each | Second execution operand path | Values, including completion bypass |
| write_enable_i | 2 | Accepted completion arbiter outputs | One independent enable per result |
| write0_addr_i / write1_addr_i | 6 each | Accepted completions | Physical destinations |
| write0_data_i / write1_data_i | 32 each | Accepted completions | New values |
| write_conflict_o | 1 | Debug/protocol checking | Both enabled writes target the same nonzero physical tag |

The write lanes carry accepted, live, non-faulting results. A waiting execution result must
not assert a write until the arbiter and ROB identity checks accept it. There is no write-ready
port because the file can accept both distinct writes on each enabled edge.

## 3. Circuit view

```text
                    63 stored 32-bit words (p1..p63)
                           |  |  |  |
                    four address-selected reads
                           |  |  |  |
 write0 tag/value -------> match and bypass selection for each read
 write1 tag/value -------> match and bypass selection for each read
                           |  |  |  |
                    reset/p0 zero selection
                           |  |  |  |
                   lane0 operands   lane1 operands
```

p0 has no writable storage and always reads zero. The stored state remains 2,016 bits; wider
ports increase selection, decode, and bypass logic rather than requiring a second architectural
copy of every value. The exact FPGA cell counts and timing require Vivado reports.

Each stored word has two decoded write enables. Different destinations update simultaneously.
Four independent read selectors can request any combination of tags, including four reads of
the same register. Reset clears all stored values and overrides bypass on all read ports.

## 4. Write conflicts and priority

Legal live completions have different physical destinations even if their architectural
instructions both write x5. Renaming gives those instructions different physical versions.

If a caller violates this contract and sends two enabled writes to the same nonzero tag,
write_conflict_o asserts and lane 0 wins consistently for both storage and bypass. This is a
protocol diagnostic, not an ISA exception or a normal scheduling policy. The completion arbiter
must prevent such a conflict during legal operation. Two attempted writes to p0 are ignored.

Read priority is: reset zero, p0 zero, write-lane-0 match, write-lane-1 match, stored value.
This makes the result independent of simulator nonblocking-assignment ordering.

## 5. Out-of-order state remains separate

The register file stores speculative and committed physical values together. It does not own
readiness, rename allocation, ROB generations, or the committed map. Completion can update a
physical version before that instruction retires.

Branch recovery must preserve this array and restore mappings after safe cancellation; it
must not pulse the file's global reset. Readiness cannot be inferred from a value of zero.

The asynchronous reads and reset describe register-based storage. A synchronous block RAM is
not a drop-in replacement: it would change operand latency. The larger program/data RAM will
have a separate synchronous memory interface.

## 6. Tests and waveforms

From the project directory:

```sh
python3 scripts/run_checks.py --block 3 --waves
```

The clocked test compares four reads and two writes against an independent reference array.
It checks distinct simultaneous writes, all read ports, pre-edge bypass, persisted values,
withdrawn/disabled writes, reset, p0, conflicts, and randomized cycles. The report identifies
exact counts and source hashes; it is not a complete processor or timing test.

The optional waveform is `build/block03/block03.vcd`. Add both write enables/addresses/data,
the four read tags/values, clock, reset, and write_conflict. Look for bypass values before an
edge and persistence after write enable falls. In a legal two-result example, conflict stays low.

For Vivado XSim on Linux after sourcing your installation's settings file:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_physical_regfile.sv tb/tb_block03.sv
xelab tb_block03 -s block03_sim
xsim block03_sim --runall
```

Vivado execution and the 50 MHz target remain unverified until the actual Linux build runs.

## 7. Interview questions

**Why are four reads useful for two-issue?** Two issued integer instructions can each require
two independent source values in the same cycle.

**Why do two writes to x5 not collide here?** The instructions have different renamed physical
destinations. Architectural naming and physical storage addressing are different layers.

**Does two-issue guarantee two instructions execute every cycle?** No. Dependencies, resource
availability, prediction, and memory latency determine which instructions can issue.

**Why bypass both write lanes to every read port?** Either source of either instruction may
need a newly accepted completion, so the match cannot be limited to one fixed pairing.

**Does a write mean architectural commitment?** No. Only in-order retirement updates the
committed register map.

The module and testbench source below are the files actually compiled. Historical single-issue
results remain in the reference checkpoint and are not evidence for this revised interface.
