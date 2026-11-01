`timescale 1ns/1ps
module rtl_audit_tb;
  reg clk = 0;
  reg reset = 0;
  always #5 clk = ~clk;
  riscv_top dut(.clk(clk), .reset(reset));
  integer checks = 0, failures = 0, stalls = 0, redirects = 0;
  integer i;

  always @(posedge clk) begin
    if (!reset) begin
      if (dut.stall_if === 1'b1) stalls = stalls + 1;
      if (dut.pc_src === 1'b1) redirects = redirects + 1;
    end
  end

  function [31:0] enc_i(input integer imm, rs1, f3, rd, opcode);
    enc_i = ((imm & 4095) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | opcode;
  endfunction
  function [31:0] enc_r(input integer f7, rs2, rs1, f3, rd);
    enc_r = (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | 7'h33;
  endfunction
  function [31:0] enc_b(input integer imm, rs2, rs1, f3);
    enc_b = (((imm >> 12) & 1) << 31) | (((imm >> 5) & 63) << 25) |
            (rs2 << 20) | (rs1 << 15) | (f3 << 12) |
            (((imm >> 1) & 15) << 8) | (((imm >> 11) & 1) << 7) | 7'h63;
  endfunction
  function [31:0] enc_s(input integer imm, rs2, rs1, f3);
    enc_s = (((imm >> 5) & 127) << 25) | (rs2 << 20) | (rs1 << 15) |
            (f3 << 12) | ((imm & 31) << 7) | 7'h23;
  endfunction

  task prepare;
    begin
      @(negedge clk); reset = 1; #1;
      for (i = 0; i < 64; i = i + 1) begin
        dut.instruction_memory.mem[i] = 32'h00000013;
        dut.data_memory.mem[i] = 0;
      end
      stalls = 0; redirects = 0;
      @(negedge clk); reset = 0;
    end
  endtask
  task run_cycles(input integer n);
    begin repeat (n) begin @(posedge clk); #1; end end
  endtask
  task check(input string label, input [31:0] got, expected);
    begin
      checks = checks + 1;
      if (got !== expected) begin
        failures = failures + 1;
        $display("FAIL %-45s got=%h expected=%h", label, got, expected);
      end else $display("PASS %-45s value=%h", label, got);
    end
  endtask

  initial begin
    // Run the pasted, unmodified demonstration program first.
    @(negedge clk); reset = 1; #1;
    stalls = 0; redirects = 0;
    @(negedge clk); reset = 0;
    run_cycles(18);
    check("demo x1", dut.rf.regs[1], 10);
    check("demo x2", dut.rf.regs[2], 5);
    check("demo x3", dut.rf.regs[3], 5);
    check("demo x4", dut.rf.regs[4], 5);
    check("demo x5 flushed", dut.rf.regs[5], 0);
    check("demo x6 branch target", dut.rf.regs[6], 100);
    check("demo stored word", dut.data_memory.mem[0], 5);
    check("demo load-use stalls", stalls, 1);
    check("demo taken branches", redirects, 1);
    check("flush_id is driven", (dut.flush_id === 1'b0), 1);

    prepare;
    dut.instruction_memory.mem[0] = enc_i(7, 0, 0, 1, 7'h13);
    dut.instruction_memory.mem[1] = enc_i(5, 1, 0, 1, 7'h13);
    dut.instruction_memory.mem[2] = enc_r(0, 1, 1, 0, 2);
    run_cycles(12);
    check("consecutive dependent ADDI", dut.rf.regs[1], 12);
    check("newest MEM forwarding has priority", dut.rf.regs[2], 24);

    prepare;
    dut.instruction_memory.mem[0] = enc_i(7, 0, 0, 1, 7'h13);
    dut.instruction_memory.mem[3] = enc_i(1, 1, 0, 2, 7'h13);
    run_cycles(12);
    check("WB/ID same-edge RAW", dut.rf.regs[2], 8);

    prepare;
    dut.instruction_memory.mem[0] = enc_i(7, 0, 0, 1, 7'h13);
    dut.instruction_memory.mem[3] = enc_s(0, 1, 0, 2);
    run_cycles(12);
    check("WB/ID same-edge store data", dut.data_memory.mem[0], 7);

    prepare;
    dut.instruction_memory.mem[0] = enc_i(7, 0, 0, 1, 7'h13);
    dut.instruction_memory.mem[3] = enc_b(8, 0, 1, 0);
    dut.instruction_memory.mem[4] = enc_i(1, 0, 0, 2, 7'h13);
    run_cycles(14);
    check("WB/ID same-edge BEQ must not take", dut.rf.regs[2], 1);

    prepare;
    dut.data_memory.mem[0] = 41;
    dut.instruction_memory.mem[0] = enc_i(0, 0, 2, 1, 7'h03);
    dut.instruction_memory.mem[1] = enc_i(1, 1, 0, 2, 7'h13);
    run_cycles(12);
    check("adjacent load-use result", dut.rf.regs[2], 42);
    check("adjacent load-use one stall", stalls, 1);

    prepare;
    dut.instruction_memory.mem[0] = enc_i(0, 0, 2, 1, 7'h03);
    dut.instruction_memory.mem[1] = enc_i(1, 0, 0, 2, 7'h13);
    run_cycles(12);
    check("ADDI immediate bits must not be rs2", stalls, 0);
    check("false-stall ADDI result", dut.rf.regs[2], 1);

    prepare;
    dut.instruction_memory.mem[0] = enc_i(0, 0, 2, 0, 7'h03);
    dut.instruction_memory.mem[1] = enc_i(1, 0, 0, 2, 7'h13);
    run_cycles(12);
    check("load to x0 must not cause RAW stall", stalls, 0);
    check("x0 remains zero", dut.rf.regs[0], 0);

    prepare;
    dut.instruction_memory.mem[0] = enc_i(1, 0, 0, 1, 7'h13);
    dut.instruction_memory.mem[1] = enc_b(8, 0, 1, 1);
    dut.instruction_memory.mem[2] = enc_i(99, 0, 0, 2, 7'h13);
    run_cycles(12);
    check("BNE must skip next instruction", dut.rf.regs[2], 0);

    prepare;
    dut.instruction_memory.mem[0] = 32'h008000ef; // jal x1,+8
    dut.instruction_memory.mem[1] = enc_i(99, 0, 0, 2, 7'h13);
    run_cycles(12);
    check("JAL link value", dut.rf.regs[1], 4);
    check("JAL redirect", dut.rf.regs[2], 0);

    prepare;
    dut.instruction_memory.mem[0] = enc_i(3, 0, 0, 1, 7'h13);
    dut.instruction_memory.mem[1] = enc_i(1, 0, 0, 2, 7'h13);
    // Wait for both operands to commit so missing ALU operations are tested
    // independently of the WB/ID timing bug.
    dut.instruction_memory.mem[6] = enc_r(0, 2, 1, 7, 3); // and
    dut.instruction_memory.mem[7] = enc_r(0, 2, 1, 1, 4); // sll
    dut.instruction_memory.mem[8] = enc_r(0, 2, 1, 2, 5); // slt
    dut.instruction_memory.mem[9] = enc_r(1, 2, 1, 0, 6); // mul
    dut.instruction_memory.mem[10] = enc_r(1, 2, 1, 4, 7); // div
    dut.instruction_memory.mem[11] = enc_i(1, 1, 7, 8, 7'h13); // andi
    run_cycles(20);
    check("AND", dut.rf.regs[3], 1);
    check("SLL", dut.rf.regs[4], 6);
    check("SLT", dut.rf.regs[5], 0);
    check("MUL", dut.rf.regs[6], 3);
    check("DIV", dut.rf.regs[7], 3);
    check("ANDI", dut.rf.regs[8], 1);

    prepare;
    dut.data_memory.mem[0] = 32'h12345680;
    dut.instruction_memory.mem[0] = enc_i(0, 0, 0, 1, 7'h03); // lb
    dut.instruction_memory.mem[1] = enc_i(0, 0, 4, 2, 7'h03); // lbu
    run_cycles(12);
    check("LB sign extension", dut.rf.regs[1], 32'hffffff80);
    check("LBU zero extension", dut.rf.regs[2], 32'h80);

    prepare;
    dut.data_memory.mem[0] = 32'h12345678;
    dut.instruction_memory.mem[0] = enc_i(127, 0, 0, 1, 7'h13);
    dut.instruction_memory.mem[1] = enc_s(0, 1, 0, 0); // sb
    run_cycles(12);
    check("SB preserves other bytes", dut.data_memory.mem[0], 32'h1234567f);

    prepare;
    dut.instruction_memory.mem[0] = enc_b(0, 0, 0, 0); // beq x0,x0,0
    dut.instruction_memory.mem[1] = enc_s(0, 0, 0, 2);
    dut.instruction_memory.mem[2] = enc_i(99, 0, 0, 1, 7'h13);
    dut.data_memory.mem[0] = 32'habcdef01;
    run_cycles(14);
    check("taken BEQ flushes wrong-path store", dut.data_memory.mem[0], 32'habcdef01);
    check("taken BEQ flushes wrong-path reg write", dut.rf.regs[1], 0);

    prepare;
    dut.instruction_memory.mem[0] = enc_i(-1, 0, 0, 1, 7'h13);
    dut.instruction_memory.mem[1] = enc_i(-2, 1, 0, 2, 7'h13);
    run_cycles(12);
    check("negative ADDI immediate", dut.rf.regs[2], 32'hfffffffd);

    $display("AUDIT SUMMARY: %0d checks, %0d pass, %0d fail", checks, checks-failures, failures);
    $finish;
  end
endmodule
