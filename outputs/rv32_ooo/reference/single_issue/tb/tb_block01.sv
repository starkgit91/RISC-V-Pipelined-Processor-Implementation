`timescale 1ns / 1ps

module tb_block01;
  import rv32_ooo_pkg::*;

  integer checks = 0;
  integer failures = 0;
  integer i;
  integer j;
  rob_tag_t tag_a;
  rob_tag_t tag_b;
  decoded_uop_t decoded;
  renamed_uop_t older;
  renamed_uop_t younger;
  renamed_uop_t copied;
  logic [$bits(renamed_uop_t)-1:0] packed_uop;
  completion_t completion;
  completion_t copied_completion;
  logic [$bits(completion_t)-1:0] packed_completion;
  issue_entry_t issue_entry;
  rob_entry_t rob_entry;
  memory_entry_t memory_entry;
  data_request_t request;
  retire_trace_t trace_event;

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        $display("FAIL: %s", label_text);
      end else begin
        $display("PASS: %s", label_text);
      end
    end
  endtask

  initial begin
    check(XLEN == 32 && ARCH_REG_COUNT == 32, "architectural width and register count");
    check(PHYS_REG_COUNT == 64 && ROB_DEPTH == 16, "physical register and ROB capacity");
    check(ISSUE_DEPTH == 8 && MEMORY_DEPTH == 4, "issue and memory queue capacity");
    check(ARCH_REG_W == 5 && PHYS_REG_W == 6, "register tags cover their complete ranges");
    check(ROB_INDEX_W == 4 && $bits(rob_tag_t) == 12, "ROB identity includes generation");
    check(BTB_ENTRIES == 64 && PHT_ENTRIES == 128, "predictor dimensions");
    check(RAM_BYTES == 65536 && RAM_WORDS == 16384, "byte-addressed RAM capacity");
    check(REFERENCE_HZ == 100000000 && SYSTEM_HZ == 50000000, "selected clock targets");
    check(RESET_PC == 0 && NOP_INSTRUCTION == 32'h00000013, "reset and architectural NOP");
    check(MISA_VALUE == 32'h40001100, "RV32 I/M misa constant");
    check(TIMER_BASE >= RAM_BASE + RAM_BYTES, "RAM and timer windows do not overlap");
    check(UART_BASE >= TIMER_BASE + MMIO_BYTES, "timer and UART windows do not overlap");
    check(GPIO_BASE >= UART_BASE + MMIO_BYTES, "UART and GPIO windows do not overlap");
    check(MTIME_OFFSET + 8 <= MMIO_BYTES, "64-bit timer fits its MMIO window");
    check(CSR_MEPC == 12'h341 && CSR_MCAUSE == 12'h342, "machine trap CSR addresses");
    check(CAUSE_ECALL_M == 11 && CAUSE_IRQ_TIMER == 32'h80000007,
          "exceptions and interrupts have distinct cause encodings");

    // Exhaust every slot through several generations; do not merely test slot zero.
    for (i = 0; i < ROB_DEPTH; i = i + 1) begin
      for (j = 0; j < 3; j = j + 1) begin
        tag_a.index = i;
        tag_a.generation = j;
        tag_b = tag_a;
        check(tag_a.index == i && tag_a.generation == j, "ROB identity retains its fields");
        tag_b.generation = j + 1;
        check(tag_a != tag_b, "same slot in another generation is a different instruction");
      end
    end

    decoded = '0;
    decoded.fetch.pc = 32'h00000020;
    decoded.fetch.instruction = 32'h0220c2b3; // div x5,x1,x2; decoder comes in block 2.
    decoded.fetch.prediction.next_pc = 32'h00000024;
    decoded.op_class = UOP_MULDIV;
    decoded.muldiv_op = MD_DIV;
    decoded.uses_rs1 = 1;
    decoded.uses_rs2 = 1;
    decoded.writes_rd = 1;
    decoded.rs1 = 1;
    decoded.rs2 = 2;
    decoded.rd = 5;
    older = '0;
    older.decoded = decoded;
    older.rob_tag.index = 14;
    older.rob_tag.generation = 9;
    older.prs1 = 1;
    older.prs2 = 2;
    older.pdst = 37;
    older.old_pdst = 5;

    // Represent a younger writer without implementing rename allocation here.
    younger = older;
    younger.decoded.fetch.pc = 32'h00000024;
    younger.decoded.fetch.instruction = 32'h004182b3; // add x5,x3,x4
    younger.decoded.op_class = UOP_INTEGER;
    younger.decoded.alu_op = ALU_ADD;
    younger.decoded.rs1 = 3;
    younger.decoded.rs2 = 4;
    younger.prs1 = 3;
    younger.prs2 = 4;
    younger.pdst = 38;
    younger.old_pdst = 37;
    younger.rob_tag.index = 15;
    check(older.decoded.rd == younger.decoded.rd && older.pdst != younger.pdst,
          "two versions of x5 can have different physical destinations");
    check(older.pdst == younger.old_pdst, "replacement retains the previous mapping");
    check(older.rob_tag != younger.rob_tag, "writers retain independent instruction identities");
    packed_uop = younger;
    copied = packed_uop;
    check(copied == younger, "full renamed record survives packed-wire transport");
    check(copied.decoded.fetch.pc == 32'h24 && copied.pdst == 38,
          "nested PC and destination fields are preserved");

    completion = '0;
    completion.rob_tag = younger.rob_tag;
    completion.pdst = younger.pdst;
    completion.writes_rd = 1;
    completion.value = 32'hfeedcafe;
    completion.control_flow.next_pc = 32'h28;
    packed_completion = completion;
    copied_completion = packed_completion;
    check(copied_completion == completion, "completion transport preserves identity and result");
    check(copied_completion.rob_tag == younger.rob_tag && copied_completion.pdst == 38,
          "writeback carries the owning ROB tag and physical destination");
    completion.exception.valid = 1;
    completion.exception.cause = CAUSE_LOAD_ACCESS;
    completion.exception.tval = 32'hf0000000;
    check(completion.exception.valid && completion.exception.tval == 32'hf0000000,
          "fault completion retains cause and fault address");

    issue_entry = '0;
    rob_entry = '0;
    memory_entry = '0;
    trace_event = '0;
    check(!issue_entry.valid && !rob_entry.valid && !memory_entry.valid && !trace_event.valid,
          "cleared validity distinguishes empty storage from an instruction");
    issue_entry.valid = 1;
    issue_entry.uop = older;
    check(issue_entry.uop.decoded.op_class == UOP_MULDIV,
          "issue queue record carries execution classification");
    rob_entry.valid = 1;
    rob_entry.uop = older;
    rob_entry.complete = 0;
    check(rob_entry.uop.old_pdst == 5 && !rob_entry.complete,
          "ROB retains rename history independently of completion");

    request = '0;
    request.write = 1;
    request.address = 32'h100;
    request.wdata = 32'h00ab0000;
    request.wstrb = 4'b0100;
    check(request.wstrb == 4'b0100 && request.wdata[23:16] == 8'hab,
          "data request can represent an isolated byte write");
    check(MD_MUL == 0 && MD_DIV == 4 && MD_REMU == 7, "M operations preserve funct3 encoding");
    check(CSR_RWI == 5 && CSR_RCI == 7, "CSR immediate operations preserve funct3 encoding");

    $display("BLOCK01 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures != 0) $fatal(1, "Block 1 record/configuration checks failed");
    $finish;
  end
endmodule
