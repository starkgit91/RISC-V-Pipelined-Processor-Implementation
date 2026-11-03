// Machine CSR/system-unit checks: completion is speculative, retirement changes state.
`timescale 1ns / 1ps
module tb_block13;
  import rv32_ooo_pkg::*;
  logic clk = 0, rst_n = 1, request_valid, request_ready, completion_valid, completion_ready;
  execute_packet_t request_packet;
  completion_t completion_packet, saved_result;
  renamed_uop_t saved_owner;
  logic [1:0] retire_valid;
  rob_entry_pair_t retire_entry;
  logic external_irq, software_irq, timer_irq, interrupt_pending;
  word_t interrupt_cause, trap_pc, trap_cause, trap_tval, trap_target;
  logic trap_enter, waiting, cancel, cancel_done, quiet, error, force_nonzero_rs1;
  integer checks = 0, failures = 0, sequence_id = 1;
  word_t value, before_value, after_value;
  logic [63:0] counter_before;

  rv32_csr_file dut (
    .clk_i(clk), .rst_ni(rst_n), .request_valid_i(request_valid),
    .request_ready_o(request_ready), .request_i(request_packet),
    .completion_valid_o(completion_valid), .completion_ready_i(completion_ready),
    .completion_o(completion_packet), .retire_valid_i(retire_valid), .retire_entry_i(retire_entry),
    .irq_external_i(external_irq), .irq_software_i(software_irq), .irq_timer_i(timer_irq),
    .interrupt_pending_o(interrupt_pending), .interrupt_cause_o(interrupt_cause),
    .trap_enter_i(trap_enter), .trap_pc_i(trap_pc), .trap_cause_i(trap_cause),
    .trap_tval_i(trap_tval), .trap_target_o(trap_target), .wfi_wait_o(waiting),
    .cancel_i(cancel), .cancel_done_o(cancel_done), .quiescent_o(quiet), .protocol_error_o(error)
  );

  task automatic check(input logic condition, input string description);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        $display("FAIL: %s at %0t", description, $time);
      end
    end
  endtask
  task automatic step;
    begin #1; clk = 1; #1; clk = 0; #1; end
  endtask

  task automatic issue(
    input system_op_e system_kind, input csr_addr_t address,
    input csr_op_e operation, input word_t operand_value, input logic [4:0] immediate
  );
    begin
      request_packet = '0;
      request_packet.uop.rob_tag.index = sequence_id % ROB_DEPTH;
      request_packet.uop.rob_tag.generation = sequence_id / ROB_DEPTH;
      request_packet.uop.pdst = 32 + sequence_id % 32;
      request_packet.uop.decoded.op_class = UOP_SYSTEM;
      request_packet.uop.decoded.system_op = system_kind;
      request_packet.uop.decoded.csr_address = address;
      request_packet.uop.decoded.csr_op = operation;
      request_packet.uop.decoded.csr_immediate = immediate;
      request_packet.uop.decoded.writes_rd = system_kind == SYS_CSR;
      request_packet.uop.decoded.fetch.pc = 32'h100 + sequence_id * 4;
      request_packet.uop.decoded.fetch.instruction = 32'h00000073 | (address << 20);
      request_packet.uop.decoded.fetch.prediction.next_pc =
        request_packet.uop.decoded.fetch.pc + 4;
      request_packet.rs1_value = operand_value;
      request_packet.uop.decoded.rs1 = (operand_value != 0 || force_nonzero_rs1) ? 1 : 0;
      sequence_id = sequence_id + 1;
      request_valid = 1; #1;
      check(request_ready, "idle system unit accepts one owner");
      step; request_valid = 0; #1;
      check(completion_valid && !request_ready && !quiet, "completion retains serialized owner");
      check(completion_packet.rob_tag == request_packet.uop.rob_tag
            && completion_packet.pdst == request_packet.uop.pdst,
            "completion preserves full identity and physical destination");
      saved_owner = request_packet.uop;
      saved_result = completion_packet;
    end
  endtask

  task automatic consume;
    begin
      completion_ready = 1; step; completion_ready = 0; #1;
      check(!completion_valid && !request_ready && !quiet,
            "completion consumption retains intent until actual retirement");
    end
  endtask
  task automatic commit;
    begin
      retire_entry = '0;
      retire_entry.lane0.valid = 1;
      retire_entry.lane0.complete = 1;
      retire_entry.lane0.uop = saved_owner;
      retire_entry.lane0.result = saved_result.value;
      retire_entry.lane0.control_flow = saved_result.control_flow;
      retire_valid = 1; #1;
      check(!error, "matching completed retirement has no protocol diagnostic");
      step; retire_valid = 0; retire_entry = '0; #1;
      check(quiet && !completion_valid, "retirement releases serialized intent");
    end
  endtask
  task automatic read_csr(input csr_addr_t address, output word_t result);
    begin
      issue(SYS_CSR, address, CSR_RS, 0, 0);
      check(!completion_packet.exception.valid, "implemented CSR reads normally");
      result = completion_packet.value;
      consume; commit;
    end
  endtask
  task automatic write_csr(input csr_addr_t address, input word_t data);
    begin
      issue(SYS_CSR, address, CSR_RW, data, 0);
      check(!completion_packet.exception.valid, "writable or fixed WARL CSR write is legal");
      consume; commit;
    end
  endtask
  task automatic modify(
    input csr_op_e operation, input word_t data, input logic [4:0] immediate,
    input word_t expected_old
  );
    begin
      issue(SYS_CSR, CSR_MSCRATCH, operation, data, immediate);
      check(!completion_packet.exception.valid && completion_packet.value == expected_old,
            "CSR operation returns the old value");
      consume; commit;
    end
  endtask
  task automatic clear_speculation;
    begin
      cancel = 1; #1;
      check(!completion_valid && !request_ready, "cancel masks completion and request immediately");
      step; cancel = 0; #1;
      check(cancel_done && quiet, "cancel acknowledges removal of pending system owner");
      step;
    end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block13/block13.vcd");
      $dumpvars(0, tb_block13);
    end
    #100000; $fatal(1, "CSR/system test timed out");
  end
  initial begin
    request_valid = 0; request_packet = '0; completion_ready = 0;
    retire_valid = 0; retire_entry = '0; external_irq = 0; software_irq = 0; timer_irq = 0;
    trap_enter = 0; trap_pc = 0; trap_cause = 0; trap_tval = 0; cancel = 0;
    force_nonzero_rs1 = 0;
    #1; rst_n = 0; #2;
    check(!completion_valid && !request_ready && quiet && !waiting,
          "global reset clears architectural and speculative system state");
    rst_n = 1; #1;
    read_csr(CSR_MSTATUS, value); check(value == 32'h1800, "MPP is fixed machine mode");

    // Completion, including a consumed completion, cannot commit a CSR write.
    issue(SYS_CSR, CSR_MSCRATCH, CSR_RW, 32'hdeadbeef, 0);
    check(completion_packet.value == 0, "first scratch write sees reset old value");
    for (integer i = 0; i < 3; i++) begin
      step; check(completion_valid && completion_packet == saved_result,
                  "held completion remains stable while counters continue");
    end
    consume; clear_speculation;
    read_csr(CSR_MSCRATCH, value);
    check(value == 0, "cancel after completion prevents CSR mutation");
    modify(CSR_RW, 32'h1234, 0, 0);
    modify(CSR_RS, 32'h40, 0, 32'h1234);
    modify(CSR_RC, 32'h30, 0, 32'h1274);
    modify(CSR_RWI, 0, 3, 32'h1244);
    modify(CSR_RSI, 0, 16, 3);
    modify(CSR_RCI, 0, 1, 19);
    read_csr(CSR_MSCRATCH, value); check(value == 18, "all six CSR operations update as specified");

    // Consuming the result is insufficient: only the exact allocated owner may commit.
    issue(SYS_CSR, CSR_MSCRATCH, CSR_RW, 32'hbad0cafe, 0); consume;
    retire_entry = '0; retire_entry.lane0.valid = 1; retire_entry.lane0.complete = 1;
    retire_entry.lane0.uop = saved_owner;
    retire_entry.lane0.uop.rob_tag.generation = saved_owner.rob_tag.generation + 1'b1;
    retire_valid = 1; #1; check(!error, "another retirement generation cannot claim this intent");
    step; retire_valid = 0;
    check(!quiet && !request_ready, "stale retirement leaves the serialized owner pending");
    retire_entry.lane0.uop = saved_owner;
    retire_entry.lane0.uop.pdst = saved_owner.pdst == 63 ? 62 : 63;
    retire_valid = 1; #1; check(error, "matching tag with a wrong destination cannot commit");
    step; retire_valid = 0; retire_entry = '0;
    check(!quiet, "malformed matching retirement retains intent for recovery");
    clear_speculation; read_csr(CSR_MSCRATCH, value);
    check(value == 18, "rejected retirement packets cannot mutate committed scratch");

    read_csr(CSR_MVENDORID, value); check(value == 0, "identification CSR is readable zero");
    issue(SYS_CSR, CSR_MVENDORID, CSR_RW, 0, 0);
    check(completion_packet.exception.valid && completion_packet.exception.cause == 2,
          "CSRRW writes even operand zero and faults on read-only CSR");
    consume; clear_speculation;
    force_nonzero_rs1 = 1;
    issue(SYS_CSR, CSR_MVENDORID, CSR_RS, 0, 0);
    check(completion_packet.exception.valid && completion_packet.exception.cause == 2,
          "CSRRS with a nonzero source register attempts a write even if its value is zero");
    clear_speculation; force_nonzero_rs1 = 0;
    issue(SYS_CSR, CSR_MVENDORID, CSR_RSI, 0, 0);
    check(!completion_packet.exception.valid, "CSRRSI zimm zero is a legal read-only read");
    consume; commit;
    issue(SYS_CSR, 12'h777, CSR_RS, 0, 0);
    check(completion_packet.exception.valid && completion_packet.exception.cause == 2
          && completion_packet.exception.tval == request_packet.uop.decoded.fetch.instruction,
          "unsupported CSR read is an illegal instruction");
    clear_speculation;
    write_csr(CSR_MISA, 0); read_csr(CSR_MISA, value);
    check(value == MISA_VALUE, "misa is fixed RV32 I/M WARL state");
    write_csr(CSR_MSTATUSH, 32'hffffffff); read_csr(CSR_MSTATUSH, value);
    check(value == 0, "machine status high half is fixed zero");

    // Freeze both counters, verify half writes, then resume and count a retire pair.
    write_csr(CSR_MCOUNTINHIBIT, 5);
    read_csr(CSR_MCYCLE, before_value);
    repeat (5) step;
    read_csr(CSR_MCYCLE, after_value);
    check(before_value == after_value, "mcountinhibit freezes cycle increments");
    write_csr(CSR_MCYCLEH, 32'h1234); write_csr(CSR_MCYCLE, 32'hfffffffe);
    read_csr(CSR_MCYCLE, value); check(value == 32'hfffffffe, "cycle low-half write");
    read_csr(CSR_MCYCLEH, value); check(value == 32'h1234, "cycle high-half write");
    write_csr(CSR_MINSTRETH, 32'habcd); write_csr(CSR_MINSTRET, 32'h42);
    read_csr(CSR_MINSTRET, value); check(value == 32'h42, "instret low half write");
    read_csr(CSR_MINSTRETH, value); check(value == 32'habcd, "instret high half write");
    write_csr(CSR_MCOUNTINHIBIT, 0); repeat (4) step;
    read_csr(CSR_MCYCLEH, value);
    check(value == 32'h1235, "64-bit cycle counter carries across halves");
    read_csr(CSR_MINSTRET, before_value);
    retire_entry = '0;
    retire_entry.lane0.valid = 1; retire_entry.lane0.complete = 1;
    retire_entry.lane1.valid = 1; retire_entry.lane1.complete = 1;
    retire_valid = 3; step; retire_valid = 0; retire_entry = '0;
    read_csr(CSR_MINSTRET, after_value);
    check(after_value == before_value + 3,
          "instret counts read retirement plus two ordinary retirements");

    // Counter writes replace the whole implicit increment, including its carry.
    // Observe pre/post-edge state without forcing or modifying any DUT register.
    write_csr(CSR_MCOUNTINHIBIT, 4);
    write_csr(CSR_MINSTRETH, 32'h12345678);
    write_csr(CSR_MINSTRET, 32'hffffffff);
    write_csr(CSR_MCOUNTINHIBIT, 0);
    issue(SYS_CSR, CSR_MINSTRETH, CSR_RW, 32'habcdef01, 0); consume;
    counter_before = dut.instret_q;
    check(counter_before[31:0] == 32'hffffffff,
          "instret high-write fixture starts at low-half overflow boundary");
    commit;
    check(dut.instret_q == {32'habcdef01, counter_before[31:0]},
          "instret high-half write preserves low half and does not count itself");
    write_csr(CSR_MINSTRET, 32'hffffffff);
    issue(SYS_CSR, CSR_MINSTRET, CSR_RW, 32'h55, 0); consume;
    counter_before = dut.instret_q;
    check(counter_before[31:0] == 32'hffffffff,
          "instret low-write fixture independently starts at the carry boundary");
    commit;
    check(dut.instret_q == {counter_before[63:32], 32'h55},
          "instret low-half write suppresses carry into the untouched high half");

    write_csr(CSR_MCYCLEH, 32'h10203040);
    write_csr(CSR_MCYCLE, 32'hfffffffd);
    issue(SYS_CSR, CSR_MCYCLE, CSR_RW, 32'h66, 0); consume;
    counter_before = dut.cycle_q;
    check(counter_before[31:0] == 32'hffffffff,
          "cycle low-write fixture reaches overflow just before retirement");
    commit;
    check(dut.cycle_q == {counter_before[63:32], 32'h66},
          "cycle low-half write suppresses implicit high-half carry");
    issue(SYS_CSR, CSR_MCYCLEH, CSR_RW, 32'h50607080, 0); consume;
    counter_before = dut.cycle_q;
    commit;
    check(dut.cycle_q == {32'h50607080, counter_before[31:0]},
          "cycle high-half write preserves the pre-increment low half");

    write_csr(CSR_MIE, 32'hffffffff); read_csr(CSR_MIE, value);
    check(value == 32'h888, "mie exposes only machine external/software/timer enables");
    write_csr(CSR_MSTATUS, 8);
    external_irq = 1; software_irq = 1; timer_irq = 1; #1;
    check(interrupt_pending && interrupt_cause == CAUSE_IRQ_EXTERNAL,
          "external interrupt priority");
    external_irq = 0; #1;
    check(interrupt_cause == CAUSE_IRQ_SOFTWARE, "software interrupt priority over timer");
    software_irq = 0; #1;
    check(interrupt_cause == CAUSE_IRQ_TIMER, "timer interrupt selected when alone");
    write_csr(CSR_MIP, 32'hffffffff); read_csr(CSR_MIP, value);
    check(value == 32'h80, "mip reflects hardware inputs and ignores CSR writes");
    timer_irq = 0; #1; check(!interrupt_pending, "interrupt pending clears with source");

    write_csr(CSR_MTVEC, 32'h201);
    trap_cause = CAUSE_IRQ_EXTERNAL; #1;
    check(trap_target == 32'h22c, "vectored interrupt adds four times cause code");
    trap_cause = CAUSE_ILLEGAL_INSTR; #1;
    check(trap_target == 32'h200, "synchronous exception uses mtvec base in vectored mode");
    trap_pc = 32'h333; trap_cause = CAUSE_IRQ_EXTERNAL; trap_tval = 32'hbeef;
    trap_enter = 1; step; trap_enter = 0;
    read_csr(CSR_MEPC, value); check(value == 32'h330, "trap captures aligned mepc");
    read_csr(CSR_MCAUSE, value); check(value == CAUSE_IRQ_EXTERNAL, "trap captures mcause");
    read_csr(CSR_MTVAL, value); check(value == 32'hbeef, "trap captures mtval");
    read_csr(CSR_MSTATUS, value);
    check(value == 32'h1880, "trap saves MIE into MPIE and masks MIE");
    issue(SYS_MRET, 0, CSR_NONE, 0, 0);
    check(completion_packet.control_flow.next_pc == 32'h330, "MRET completion redirects to mepc");
    consume; commit;
    read_csr(CSR_MSTATUS, value); check(value == 32'h1888, "MRET restores MIE and sets MPIE");
    write_csr(CSR_MTVEC, 32'h103); read_csr(CSR_MTVEC, value);
    check(value == 32'h100, "unsupported mtvec mode coerces to direct");

    write_csr(CSR_MSTATUS, 0);
    issue(SYS_WFI, 0, CSR_NONE, 0, 0); consume; commit;
    check(waiting && !request_ready && !interrupt_pending, "WFI waits without a gated clock");
    timer_irq = 1; step;
    check(!waiting && request_ready && !interrupt_pending,
          "locally enabled IRQ wakes WFI even when global MIE is clear");
    timer_irq = 0;
    issue(SYS_FENCE, 0, CSR_NONE, 0, 0); consume; commit;
    issue(SYS_FENCE_I, 0, CSR_NONE, 0, 0); consume; commit;
    issue(SYS_ECALL, 0, CSR_NONE, 0, 0);
    check(completion_packet.exception.valid && completion_packet.exception.cause == 11,
          "ECALL produces precise machine environment-call cause");
    clear_speculation;
    issue(SYS_EBREAK, 0, CSR_NONE, 0, 0);
    check(completion_packet.exception.valid && completion_packet.exception.cause == 3,
          "EBREAK produces breakpoint cause");
    clear_speculation;

    retire_valid = 2; #1; check(error, "nonprefix retirement is diagnosed");
    step; retire_valid = 0;
    read_csr(CSR_MSCRATCH, value);
    check(value == 18, "trap/cancel preserve committed scratch state");
    $display("BLOCK13 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "CSR/system checks failed");
    $finish;
  end
endmodule
