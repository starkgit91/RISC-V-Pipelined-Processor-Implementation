# Custom RV32 out-of-order learning project

This project is being implemented sequentially, with a user learning checkpoint before each block.
The intended final processor is RV32IM_Zicsr_Zifencei, organized as five logical sections:
IF, ID/rename, EX/scheduling, MEM, and WB/commit.

**Current state: blocks 1 through 4 are implemented.** These include shared definitions, an
RV32IM/system instruction decoder, integer ALU, branch comparator, physical register file with
bypass, and rename/free-list/readiness state. This is not yet a working processor. Execution
engines/buffers, scheduling, ROB control, AXI, Vivado integration, firmware, and bitstream generation
remain to be implemented.

## Read and run block 4

Start with `docs/block04_complete_guide.md`. It follows two writers of x5 through separate
physical destinations, explains speculative versus committed maps, and includes the complete
rename module and self-checking testbench.

```sh
python3 scripts/run_checks.py --block 4 --waves
```

Results and source hashes are in `reports/block04_results.json`; the log is
`reports/block04_simulation.log`. The optional waveform is `build/block04/block04.vcd`.
Tests cover allocation, readiness, source-before-destination lookup, stalls, retirement/freeing,
unit-level exhaustion, simultaneous events, recovery application, and reset.

The ROB and completion controller must still validate ownership and successful retirement.
`recover_i` applies mappings only after external cancellation/draining has made reuse safe;
this block does not implement the cancellation controller. Its testbench uses controlled clock
pulses for state queries and does not measure FPGA timing.

For Vivado XSim on Linux:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_rename.sv tb/tb_block04.sv
xelab tb_block04 -s block04_sim
xsim block04_sim --runall
```

## Read and run block 3

Start with `docs/block03_complete_guide.md`. It begins with p37 changing from 7 to 99 through
same-cycle bypass, explains the two read ports and one write port, and includes the complete
physical register-file module and clocked testbench.

```sh
python3 scripts/run_checks.py --block 3 --waves
```

The optional VCD is `build/block03/block03.vcd`. The test covers all read-address pairs,
clocked writes, pre-edge bypass, withdrawn/disabled writes, p0 protection, asynchronous reset,
walking data bits, and deterministic random cycles. Source hashes and results are in
`reports/block03_results.json`; the log is `reports/block03_simulation.log`.

The register file stores values. It does not track readiness, validate ROB identity, allocate
physical destinations, or decide what is committed. In particular, branch recovery must preserve
its committed values instead of resetting the array.

To run block 3 in Vivado XSim on Linux:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_physical_regfile.sv tb/tb_block03.sv
xelab tb_block03 -s block03_sim
xsim block03_sim --runall
```

## Read and run block 2

Start with `docs/block02_complete_guide.md`. It follows register ADD from instruction bits
through decoded source names and ALU input values, explains the future rename connection, and
includes the complete decoder, ALU, branch comparator, and testbench source.

```sh
python3 scripts/run_checks.py --block all
python3 scripts/build_guide.py
```

Use `--block 2` for just the decoder/datapath tests. The block-2 tests cover exhaustive R-type
selector and shift-upper-field legality, all signed 12-bit I/S immediates, branch/jump boundaries,
CSR/system/fence forms, x0 behavior, fetch-fault priority, ALU edge/random cases, and branches.
See `reports/block02_results.json` and `reports/block02_simulation.log` for exact evidence.

M instructions and system instructions are recognized but do not execute here. CSR permissions,
memory effects, traps, branch targets/link writeback, and OoO behavior belong to later blocks.

For the same tests in Vivado's XSim on Linux, compile the package before the other files:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv rtl/rv32_decoder.sv rtl/rv32_alu.sv rtl/rv32_branch_compare.sv tb/tb_block02.sv
xelab tb_block02 -s block02_sim
xsim block02_sim --runall
```

These Vivado commands are provided for your run; they have not been executed in this workspace.

## Read and run block 1

- Start with `docs/block01_complete_guide.md`: explanation, signal map, worked example,
  interview questions, and the complete package/testbench source.
- `rtl/rv32_ooo_pkg.sv` is the source of the shared types and constants.
- `tb/tb_block01.sv` verifies configuration and record representation, not processor behavior.
- `reports/block01_results.json` and `reports/block01_simulation.log` record the latest checks.
- `STATUS.md` tracks the full implementation sequence and the current checkpoint.

From this project directory, with Python 3 and Icarus Verilog available:

```sh
python3 scripts/run_checks.py --block 1
python3 scripts/build_guide.py
```

The runner returns a nonzero exit code on compile failure, simulation failure, or timeout.
No external Python packages are required. A successful record test does not establish ISA or
out-of-order execution correctness; block-2 tests do not establish whole-core correctness.

## Run the same block-1 test in Vivado 2026.1 on Linux

After sourcing your installation's `settings64.sh`, use these commands from this directory:

```sh
xvlog --sv rtl/rv32_ooo_pkg.sv tb/tb_block01.sv
xelab tb_block01 -s block01_sim
xsim block01_sim --runall
```

Expected result: `BLOCK01 SUMMARY` with zero failures. Vivado/XSim execution has not been
verified in this workspace. This test has no clock because it checks declarations, not sequential
hardware. Clocked tests will accompany the actual modules when they are implemented.

## Fixed architecture and FPGA target

| Item | Target |
|---|---|
| ISA | RV32IM_Zicsr_Zifencei, machine mode only |
| Widths | One dispatch, issue, completion broadcast, and retirement per cycle |
| Registers | 32 architectural; 64 physical |
| Queues | 16 ROB, 8 issue, 4 ordered memory, 2 fetch |
| Prediction | 64 BTB, 128 two-bit PHT |
| Memory | 64 KiB shared executable dual-port BRAM |
| FPGA | xc7a35tcpg236-1, Basys-3 reference pinout |
| Tools | Vivado 2026.1 on Linux; portable simulation with Icarus |
| Clock | 100 MHz reference, 50 MHz system target |

Out-of-order execution does not imply out-of-order architectural state changes. Results write
the physical register file on completion; committed mappings advance only at in-order retirement.
Memory requests will be issued only at ROB head. Branch recovery will restore the committed map
after the mispredicting branch retires, cancel younger work, and rebuild the free list.

The 50 MHz target and FPGA fit have not been verified. Actual synthesis, implementation,
constraints, generated IP, firmware initialization, and bitstream checks belong to later blocks.
