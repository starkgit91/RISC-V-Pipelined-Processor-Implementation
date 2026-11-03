`timescale 1ns / 1ps

// ==============================================================================
// 5-Stage Pipelined RISC-V Processor (RV32I Subset)
// Stages: Fetch (IF), Decode (ID), Execute (EX), Memory (MEM), Writeback (WB)
// Features: Forwarding, Load-Use Hazard Detection, Branch Flushing
// ==============================================================================

module riscv_top (
    input clk,
    input reset
);
    // Wires for Inter-stage Registers
    
    // --- IF Stage Signals ---
    wire [31:0] if_pc, if_next_pc, if_instr;
    
    // --- ID Stage Signals ---
    wire [31:0] id_pc, id_instr;
    wire [31:0] id_rd1, id_rd2, id_imm;
    wire [4:0]  id_rs1, id_rs2, id_rd;
    wire        id_reg_write, id_mem_read, id_mem_write, id_alu_src, id_branch, id_jump;
    wire [2:0]  id_alu_op;

    // --- EX Stage Signals ---
    wire [31:0] ex_pc, ex_rs1_data, ex_rs2_data, ex_imm;
    wire [4:0]  ex_rs1, ex_rs2, ex_rd;
    wire        ex_reg_write, ex_mem_read, ex_mem_write, ex_alu_src, ex_branch;
    wire [2:0]  ex_alu_op;
    wire [31:0] ex_alu_res, ex_write_data;
    wire        ex_zero;
    
    // --- MEM Stage Signals ---
    wire [31:0] mem_alu_res, mem_write_data;
    wire [4:0]  mem_rd;
    wire        mem_reg_write, mem_mem_read, mem_mem_write, mem_branch, mem_zero;
    wire [31:0] mem_read_data;
    
    // --- WB Stage Signals ---
    wire [31:0] wb_alu_res, wb_read_data, wb_final_data;
    wire [4:0]  wb_rd;
    wire        wb_reg_write, wb_mem_to_reg;

    // --- Hazard & Control Signals ---
    wire        stall_if, stall_id, flush_id, flush_ex;
    wire [1:0]  forward_a, forward_b;
    wire        pc_src; // 1 if branch taken

    // ==========================================================================
    // 1. INSTRUCTION FETCH (IF)
    // ==========================================================================
    
    reg [31:0] pc;
    
    // PC Mux Logic (Branch taken ?)
    assign pc_src = ex_branch & ex_zero;
    assign if_next_pc = pc_src ? (ex_pc + ex_imm) : (pc + 4);

    always @(posedge clk or posedge reset) begin
        if (reset) 
            pc <= 0;
        else if (!stall_if)
            pc <= if_next_pc;
    end
    
    assign if_pc = pc;

    // Instruction Memory Instance
    imem instruction_memory (
        .addr(pc),
        .instr(if_instr)
    );

    // IF/ID Pipeline Register
    reg [31:0] if_id_pc, if_id_instr;
    
    always @(posedge clk or posedge reset) begin
        if (reset || flush_id || pc_src) begin // Flush on branch taken
            if_id_pc <= 0;
            if_id_instr <= 0; // NOP
        end else if (!stall_id) begin
            if_id_pc <= if_pc;
            if_id_instr <= if_instr;
        end
    end

    // ==========================================================================
    // 2. INSTRUCTION DECODE (ID)
    // ==========================================================================
    
    assign id_pc = if_id_pc;
    assign id_instr = if_id_instr;
    assign id_rs1 = id_instr[19:15];
    assign id_rs2 = id_instr[24:20];
    assign id_rd  = id_instr[11:7];

    // Control Unit
    control_unit ctrl (
        .opcode(id_instr[6:0]),
        .funct3(id_instr[14:12]),
        .funct7(id_instr[31:25]),
        .reg_write(id_reg_write),
        .mem_read(id_mem_read),
        .mem_write(id_mem_write),
        .alu_src(id_alu_src),
        .branch(id_branch),
        .alu_op(id_alu_op)
    );

    // Register File
    reg_file rf (
        .clk(clk),
        .reset(reset),
        .rs1(id_rs1),
        .rs2(id_rs2),
        .rd(wb_rd),
        .w_data(wb_final_data),
        .we(wb_reg_write),
        .rd1(id_rd1),
        .rd2(id_rd2)
    );

    // Immediate Generator
    imm_gen ig (
        .instr(id_instr),
        .imm(id_imm)
    );

    // Hazard Detection Unit (Load-Use)
    hazard_detection hu (
        .id_rs1(id_rs1),
        .id_rs2(id_rs2),
        .ex_rd(ex_rd),
        .ex_mem_read(ex_mem_read),
        .stall_if(stall_if),
        .stall_id(stall_id),
        .flush_ex(flush_ex)
    );

    // ID/EX Pipeline Register
    reg [31:0] id_ex_pc, id_ex_rs1_data, id_ex_rs2_data, id_ex_imm;
    reg [4:0]  id_ex_rs1, id_ex_rs2, id_ex_rd;
    reg [2:0]  id_ex_alu_op;
    reg        id_ex_reg_write, id_ex_mem_read, id_ex_mem_write, id_ex_alu_src, id_ex_branch;

    always @(posedge clk or posedge reset) begin
        if (reset || flush_ex || pc_src) begin
            id_ex_pc <= 0; id_ex_rs1_data <= 0; id_ex_rs2_data <= 0; id_ex_imm <= 0;
            id_ex_rs1 <= 0; id_ex_rs2 <= 0; id_ex_rd <= 0;
            id_ex_reg_write <= 0; id_ex_mem_read <= 0; id_ex_mem_write <= 0; 
            id_ex_alu_src <= 0; id_ex_branch <= 0; id_ex_alu_op <= 0;
        end else begin
            id_ex_pc <= id_pc;
            id_ex_rs1_data <= id_rd1;
            id_ex_rs2_data <= id_rd2;
            id_ex_imm <= id_imm;
            id_ex_rs1 <= id_rs1;
            id_ex_rs2 <= id_rs2;
            id_ex_rd <= id_rd;
            id_ex_alu_op <= id_alu_op;
            id_ex_reg_write <= id_reg_write;
            id_ex_mem_read <= id_mem_read;
            id_ex_mem_write <= id_mem_write;
            id_ex_alu_src <= id_alu_src;
            id_ex_branch <= id_branch;
        end
    end

    // ==========================================================================
    // 3. EXECUTE (EX)
    // ==========================================================================
    
    assign ex_pc = id_ex_pc;
    assign ex_rs1 = id_ex_rs1;
    assign ex_rs2 = id_ex_rs2;
    assign ex_rd  = id_ex_rd;
    assign ex_imm = id_ex_imm;
    assign ex_reg_write = id_ex_reg_write;
    assign ex_mem_read = id_ex_mem_read;
    assign ex_mem_write = id_ex_mem_write;
    assign ex_branch = id_ex_branch;

    // Forwarding Unit
    forwarding_unit fu (
        .ex_rs1(ex_rs1),
        .ex_rs2(ex_rs2),
        .mem_rd(mem_rd),
        .wb_rd(wb_rd),
        .mem_reg_write(mem_reg_write),
        .wb_reg_write(wb_reg_write),
        .forward_a(forward_a),
        .forward_b(forward_b)
    );

    // ALU Input Muxes (Forwarding Logic)
    reg [31:0] alu_in_a, alu_in_b_temp, alu_in_b;

    always @(*) begin
        case (forward_a)
            2'b00: alu_in_a = id_ex_rs1_data;
            2'b10: alu_in_a = mem_alu_res;    // Forward from MEM
            2'b01: alu_in_a = wb_final_data;  // Forward from WB
            default: alu_in_a = id_ex_rs1_data;
        endcase

        case (forward_b)
            2'b00: alu_in_b_temp = id_ex_rs2_data;
            2'b10: alu_in_b_temp = mem_alu_res;
            2'b01: alu_in_b_temp = wb_final_data;
            default: alu_in_b_temp = id_ex_rs2_data;
        endcase

        // ALU Source Mux (Register vs Immediate)
        alu_in_b = id_ex_alu_src ? id_ex_imm : alu_in_b_temp;
    end
    
    assign ex_write_data = alu_in_b_temp; // Data to store in memory (for SW)

    // ALU Instance
    alu main_alu (
        .a(alu_in_a),
        .b(alu_in_b),
        .op(id_ex_alu_op),
        .res(ex_alu_res),
        .zero(ex_zero)
    );

    // EX/MEM Pipeline Register
    reg [31:0] ex_mem_alu_res, ex_mem_write_data;
    reg [4:0]  ex_mem_rd;
    reg        ex_mem_reg_write, ex_mem_mem_read, ex_mem_mem_write;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ex_mem_alu_res <= 0; ex_mem_write_data <= 0; ex_mem_rd <= 0;
            ex_mem_reg_write <= 0; ex_mem_mem_read <= 0; ex_mem_mem_write <= 0;
        end else begin
            ex_mem_alu_res <= ex_alu_res;
            ex_mem_write_data <= ex_write_data;
            ex_mem_rd <= ex_rd;
            ex_mem_reg_write <= ex_reg_write;
            ex_mem_mem_read <= ex_mem_read;
            ex_mem_mem_write <= ex_mem_write;
        end
    end

    // ==========================================================================
    // 4. MEMORY (MEM)
    // ==========================================================================
    
    assign mem_alu_res = ex_mem_alu_res;
    assign mem_write_data = ex_mem_write_data;
    assign mem_rd = ex_mem_rd;
    assign mem_reg_write = ex_mem_reg_write;
    assign mem_mem_read = ex_mem_mem_read;

    // Data Memory Instance
    dmem data_memory (
        .clk(clk),
        .mem_read(mem_mem_read),
        .mem_write(ex_mem_mem_write),
        .addr(mem_alu_res),
        .w_data(mem_write_data),
        .r_data(mem_read_data)
    );

    // MEM/WB Pipeline Register
    reg [31:0] mem_wb_alu_res, mem_wb_read_data;
    reg [4:0]  mem_wb_rd;
    reg        mem_wb_reg_write, mem_wb_mem_to_reg;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_wb_alu_res <= 0; mem_wb_read_data <= 0; mem_wb_rd <= 0;
            mem_wb_reg_write <= 0; mem_wb_mem_to_reg <= 0;
        end else begin
            mem_wb_alu_res <= mem_alu_res;
            mem_wb_read_data <= mem_read_data;
            mem_wb_rd <= mem_rd;
            mem_wb_reg_write <= mem_reg_write;
            mem_wb_mem_to_reg <= mem_mem_read; // If it was a read, we write back loaded data
        end
    end

    // ==========================================================================
    // 5. WRITEBACK (WB)
    // ==========================================================================
    
    assign wb_reg_write = mem_wb_reg_write;
    assign wb_rd = mem_wb_rd;
    assign wb_final_data = mem_wb_mem_to_reg ? mem_wb_read_data : mem_wb_alu_res;

endmodule

// ==============================================================================
// Sub-Modules
// ==============================================================================

module hazard_detection(
    input [4:0] id_rs1, id_rs2, ex_rd,
    input ex_mem_read,
    output reg stall_if, stall_id, flush_ex
);
    always @(*) begin
        // Load-Use Hazard: If ID tries to read a register that EX is loading from mem
        if (ex_mem_read && ((ex_rd == id_rs1) || (ex_rd == id_rs2))) begin
            stall_if = 1;
            stall_id = 1;
            flush_ex = 1; // Insert Bubble in EX
        end else begin
            stall_if = 0;
            stall_id = 0;
            flush_ex = 0;
        end
    end
endmodule

module forwarding_unit(
    input [4:0] ex_rs1, ex_rs2, mem_rd, wb_rd,
    input mem_reg_write, wb_reg_write,
    output reg [1:0] forward_a, forward_b
);
    always @(*) begin
        forward_a = 2'b00;
        forward_b = 2'b00;

        // EX Hazard (Forward from MEM stage)
        if (mem_reg_write && (mem_rd != 0) && (mem_rd == ex_rs1))
            forward_a = 2'b10;
        if (mem_reg_write && (mem_rd != 0) && (mem_rd == ex_rs2))
            forward_b = 2'b10;

        // MEM Hazard (Forward from WB stage)
        // Only forward if EX hazard condition isn't met
        if (wb_reg_write && (wb_rd != 0) && (wb_rd == ex_rs1) && 
            !(mem_reg_write && (mem_rd != 0) && (mem_rd == ex_rs1)))
            forward_a = 2'b01;
        if (wb_reg_write && (wb_rd != 0) && (wb_rd == ex_rs2) && 
            !(mem_reg_write && (mem_rd != 0) && (mem_rd == ex_rs2)))
            forward_b = 2'b01;
    end
endmodule

module control_unit(
    input [6:0] opcode,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg reg_write, mem_read, mem_write, alu_src, branch,
    output reg [2:0] alu_op
);
    always @(*) begin
        {reg_write, mem_read, mem_write, alu_src, branch, alu_op} = 0;
        case(opcode)
            7'b0110011: begin // R-type (add, sub, etc.)
                reg_write = 1;
                alu_op = (funct7[5]) ? 3'b001 : 3'b000; // 001 for SUB, 000 for ADD
            end
            7'b0010011: begin // I-type (addi)
                reg_write = 1;
                alu_src = 1;
                alu_op = 3'b000; // ADD
            end
            7'b0000011: begin // Load (lw)
                reg_write = 1;
                mem_read = 1;
                alu_src = 1;
                alu_op = 3'b000; // ADD (calc address)
            end
            7'b0100011: begin // Store (sw)
                mem_write = 1;
                alu_src = 1;
                alu_op = 3'b000; // ADD (calc address)
            end
            7'b1100011: begin // Branch (beq)
                branch = 1;
                alu_op = 3'b001; // SUB (compare)
            end
        endcase
    end
endmodule

module alu(
    input [31:0] a, b,
    input [2:0] op,
    output reg [31:0] res,
    output zero
);
    always @(*) begin
        case(op)
            3'b000: res = a + b;
            3'b001: res = a - b;
            default: res = 0;
        endcase
    end
    assign zero = (res == 0);
endmodule

module reg_file(
    input clk, reset, we,
    input [4:0] rs1, rs2, rd,
    input [31:0] w_data,
    output [31:0] rd1, rd2
);
    reg [31:0] regs [0:31];
    integer i;

    assign rd1 = (rs1 == 0) ? 0 : regs[rs1];
    assign rd2 = (rs2 == 0) ? 0 : regs[rs2];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i=0; i<32; i=i+1) regs[i] <= 0;
        end else if (we && rd != 0) begin
            regs[rd] <= w_data;
        end
    end
endmodule

module imm_gen(input [31:0] instr, output reg [31:0] imm);
    always @(*) begin
        case(instr[6:0])
            7'b0010011, 7'b0000011: imm = {{20{instr[31]}}, instr[31:20]}; // I-type
            7'b0100011: imm = {{20{instr[31]}}, instr[31:25], instr[11:7]}; // S-type
            7'b1100011: imm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; // B-type
            default: imm = 0;
        endcase
    end
endmodule

module imem(input [31:0] addr, output [31:0] instr);
    reg [31:0] mem [0:63]; // Small IMEM
    
    initial begin
        // Program: 
        // 0: addi x1, x0, 10
        // 4: addi x2, x0, 5
        // 8: sub  x3, x1, x2  (x3 = 5)
        // C: sw   x3, 0(x0)
        // 10: lw   x4, 0(x0)  (Load-use hazard likely)
        // 14: beq  x3, x4, +8 (Branch taken)
        // 18: addi x5, x0, 99 (Should be flushed)
        // 1C: addi x6, x0, 100 (Branch target)
        
        mem[0] = 32'h00a00093; // addi x1, x0, 10
        mem[1] = 32'h00500113; // addi x2, x0, 5
        mem[2] = 32'h402081b3; // sub x3, x1, x2
        mem[3] = 32'h00302023; // sw x3, 0(x0)
        mem[4] = 32'h00002203; // lw x4, 0(x0)
        mem[5] = 32'h00418463; // beq x3, x4, 8 (offset 8)
        mem[6] = 32'h06300293; // addi x5, x0, 99
        mem[7] = 32'h06400313; // addi x6, x0, 100
        // Fill rest with NOPs
        for (integer k=8; k<64; k=k+1) mem[k] = 32'h00000013; // nop (addi x0, x0, 0)
    end
    
    assign instr = mem[addr[31:2]];
endmodule

module dmem(
    input clk, mem_read, mem_write,
    input [31:0] addr, w_data,
    output [31:0] r_data
);
    reg [31:0] mem [0:63];
    always @(posedge clk) begin
        if (mem_write) mem[addr[31:2]] <= w_data;
    end
    assign r_data = (mem_read) ? mem[addr[31:2]] : 0;
endmodule

// ==============================================================================
// Testbench
// ==============================================================================
module tb_riscv();
    reg clk, reset;
    
    riscv_top dut (
        .clk(clk),
        .reset(reset)
    );
    
    always #5 clk = ~clk;
    
    initial begin
        clk = 0; reset = 1;
        #10 reset = 0;
        
        #200;
        $finish;
    end
    
    initial begin
        $monitor("Time=%0t | PC=%h | IF_Instr=%h | WB_Data=%d | WB_Reg=%d", 
                 $time, dut.pc, dut.if_instr, dut.wb_final_data, dut.wb_rd);
    end
endmodule
