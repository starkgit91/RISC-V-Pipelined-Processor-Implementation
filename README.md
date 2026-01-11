5-Stage Pipelined RISC-V Processor

This project implements a 32-bit RISC-V processor (RV32I subset) using a 5-stage pipelined architecture. It is written in Verilog and includes a Python-based behavioral simulator for visualization.

Features

5-Stage Pipeline: Fetch (IF), Decode (ID), Execute (EX), Memory (MEM), Writeback (WB).

Hazard Handling:

Data Hazards: Full forwarding unit (EX-to-EX, MEM-to-EX) to resolve Read-After-Write (RAW) dependencies without stalling when possible.

Load-Use Hazards: Detection unit stalls the pipeline for one cycle if a dependent instruction follows a Load immediately.

Control Hazards: Branch prediction assumes "branch not taken". If a branch is taken, the pipeline flushes instructions in the IF and ID stages.

Instruction Set: Supports a subset of RV32I:

Arithmetic: ADD, SUB, ADDI

Memory: LW, SW

Control Flow: BEQ

File Structure

riscv_pipeline.v: The synthesized Verilog source code containing the Top Module, Datapath, Control Unit, ALU, Hazard Detection, Forwarding Unit, and Memories.

pipeline_sim.py: A cycle-accurate Python simulator that visualizes the pipeline stages, register values, and hazard handling textually.

How to Run

1. Verilog Simulation (RTL)

You can simulate the hardware design using any standard Verilog simulator (e.g., ModelSim, Vivado, Verilator, Icarus Verilog).

Using Icarus Verilog:

iverilog -o riscv_sim riscv_pipeline.v
vvp riscv_sim


Expected Output:
The testbench (tb_riscv) monitors the Program Counter (PC), fetched instructions, and Writeback data.

2. Python Behavioral Simulation

For a quick visualization of how instructions move through the pipeline stages without needing EDA tools:

python3 pipeline_sim.py


This will print a table showing the state of every pipeline stage (IF, ID, EX, MEM, WB) per clock cycle, highlighting stalls, bubbles, and forwarding.

Architecture Overview

The design follows the classic RISC pipeline structure:

Instruction Fetch (IF): Updates PC, fetches instruction from IMEM.

Instruction Decode (ID): Decodes opcode, reads Register File, generates immediate values.

Execute (EX): Performs ALU operations. The Forwarding Unit muxes operands here to bypass stale register data.

Memory (MEM): Accesses Data Memory for Load/Store operations.

Writeback (WB): Writes results back to the Register File.

Test Program

Both the Verilog testbench and Python simulator run the following hardcoded assembly program to demonstrate pipeline features:

0x00: addi x1, x0, 10   # Initialize x1 = 10
0x04: addi x2, x0, 5    # Initialize x2 = 5
0x08: sub  x3, x1, x2   # x3 = x1 - x2 = 5 (Hazard: Forwarding used)
0x0C: sw   x3, 0(x0)    # Store 5 to address 0
0x10: lw   x4, 0(x0)    # Load 5 into x4
0x14: beq  x3, x4, +8   # Branch to 0x1C if x3 == x4 (Taken)
0x18: addi x5, x0, 99   # Flushed instruction (due to branch)
0x1C: addi x6, x0, 100  # Target of branch
