// Precise-trap/interrupt composition through the real core and native instruction adapter.
// The architectural reference below is independent of physical tags and recovery internals.
`timescale 1ns / 1ps
module tb_core_traps;
  import rv32_ooo_pkg::*;

  logic clk = 0, rst_n = 1, halt = 0;
  logic i_valid, i_ready, i_rsp_valid, i_rsp_ready, i_error;
  word_t i_address;
  fetch_row_t i_data, bram_data = 0;
  logic bram_enable, adapter_pending, adapter_error;
  logic [BRAM_FETCH_ADDR_W-1:0] bram_address;
  logic d_valid, d_ready, d_rsp_valid, d_rsp_ready, d_pending = 0;
  data_request_t d_request;
  memory_response_t d_response, saved_response;
  logic irq_external = 0, irq_software = 0, irq_timer = 0;
  retire_trace_pair_t trace;
  logic empty, protocol_error;
  word_t memory [0:RAM_WORDS-1];
  word_t reference_regs [0:31];
  word_t reference_pc, ref_status, ref_mie, ref_mtvec, ref_mepc, ref_mcause, ref_mtval;
  word_t expected_sync_pc, expected_sync_cause, expected_sync_tval, finish_pc;
  logic sync_expected, allow_interrupt, finished, injected;
  logic saw_younger_redirect, saw_younger_div_busy, saw_younger_div_result;
  integer d_delay = 0, data_latency = 5;
  integer checks = 0, failures = 0, total_cycles = 0, case_number = 0;
  integer events_seen, normal_seen, sync_seen, irq_seen, memory_requests;
  integer mmio_reads, mmio_writes, maximum_memory_queue, wfi_seen;
  integer n, mode, watchdog, irq_profile, expected_mmio_reads, expected_mmio_writes;
  logic captured_request;
  data_request_t accepted_request;

  rv32_ooo_core dut (
    .clk_i(clk), .rst_ni(rst_n), .predict_enable_i(1'b1), .halt_i(halt),
    .instruction_request_valid_o(i_valid), .instruction_request_ready_i(i_ready),
    .instruction_request_address_o(i_address), .instruction_response_valid_i(i_rsp_valid),
    .instruction_response_ready_o(i_rsp_ready), .instruction_response_data_i(i_data),
    .instruction_response_error_i(i_error), .data_request_valid_o(d_valid),
    .data_request_ready_i(d_ready), .data_request_o(d_request),
    .data_response_valid_i(d_rsp_valid), .data_response_ready_o(d_rsp_ready),
    .data_response_i(d_response), .irq_external_i(irq_external),
    .irq_software_i(irq_software), .irq_timer_i(irq_timer), .trace_o(trace),
    .empty_o(empty), .protocol_error_o(protocol_error)
  );
  rv32_instruction_adapter adapter (
    .clk_i(clk), .rst_ni(rst_n), .request_valid_i(i_valid), .request_ready_o(i_ready),
    .request_address_i(i_address), .response_valid_o(i_rsp_valid),
    .response_ready_i(i_rsp_ready), .response_data_o(i_data), .response_error_o(i_error),
    .bram_block_i(1'b0), .bram_enable_o(bram_enable), .bram_address_o(bram_address),
    .bram_data_i(bram_data), .transaction_pending_o(adapter_pending),
    .protocol_error_o(adapter_error)
  );
  always_ff @(posedge clk) begin
    if (bram_enable) bram_data <= {memory[{bram_address, 1'b1}],
                                  memory[{bram_address, 1'b0}]};
  end
  assign d_ready = rst_n && !d_pending;
  assign d_rsp_valid = d_pending && d_delay == 0;
  assign d_response = saved_response;

  function automatic word_t op_i(input integer imm, rs1, funct3, rd, opcode);
    op_i = ((imm & 4095) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode;
  endfunction
  function automatic word_t op_r(input integer funct7, rs2, rs1, funct3, rd);
    op_r = (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | 7'h33;
  endfunction
  function automatic word_t op_s(input integer imm, rs2, rs1, funct3);
    op_s = (((imm >> 5) & 127) << 25) | (rs2 << 20) | (rs1 << 15)
           | (funct3 << 12) | ((imm & 31) << 7) | 7'h23;
  endfunction
  function automatic word_t op_b(input integer imm, rs2, rs1, funct3);
    op_b = (((imm >> 12) & 1) << 31) | (((imm >> 5) & 63) << 25)
           | (rs2 << 20) | (rs1 << 15) | (funct3 << 12)
           | (((imm >> 1) & 15) << 8) | (((imm >> 11) & 1) << 7) | 7'h63;
  endfunction
  function automatic word_t op_j(input integer imm, rd);
    op_j = (((imm >> 20) & 1) << 31) | (((imm >> 1) & 1023) << 21)
           | (((imm >> 11) & 1) << 20) | (((imm >> 12) & 255) << 12)
           | (rd << 7) | 7'h6f;
  endfunction
  function automatic word_t csr(input integer address, funct3, source, rd);
    csr = (address << 20) | (source << 15) | (funct3 << 12) | (rd << 7) | 7'h73;
  endfunction
  function automatic word_t architectural_register(input integer index);
    phys_reg_t physical;
    begin
      physical = dut.rename.committed_q[index];
      architectural_register = physical == 0 ? 0 : dut.prf.values_q[physical];
    end
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 45)
          $display("FAIL case%0d cycle%0d pc=%h: %s", case_number,
                   total_cycles, reference_pc, label_text);
      end
    end
  endtask

  function automatic word_t read_reference_csr(input logic [11:0] address);
    case (address)
      CSR_MSTATUS: read_reference_csr = ref_status;
      CSR_MIE: read_reference_csr = ref_mie;
      CSR_MTVEC: read_reference_csr = ref_mtvec;
      CSR_MEPC: read_reference_csr = ref_mepc;
      CSR_MCAUSE: read_reference_csr = ref_mcause;
      CSR_MTVAL: read_reference_csr = ref_mtval;
      CSR_MISA: read_reference_csr = MISA_VALUE;
      CSR_MIP: read_reference_csr = (irq_external ? 32'h800 : 0)
                                  | (irq_software ? 32'h8 : 0) | (irq_timer ? 32'h80 : 0);
      default: read_reference_csr = 0;
    endcase
  endfunction

  task automatic write_reference_csr(input logic [11:0] address, input word_t value);
    begin
      case (address)
        CSR_MSTATUS: ref_status = 32'h1800 | (value & 32'h88);
        CSR_MIE: ref_mie = value & 32'h888;
        CSR_MTVEC: ref_mtvec = (value & 32'hfffffffc) | (value[1:0] == 1 ? 1 : 0);
        CSR_MEPC: ref_mepc = value & 32'hfffffffc;
        CSR_MCAUSE: ref_mcause = value;
        CSR_MTVAL: ref_mtval = value;
        default: begin end
      endcase
    end
  endtask

  task automatic observe(input retire_trace_t event_record);
    word_t instruction, result, next_pc, immediate, operand, old_csr, value;
    word_t expected_cause, expected_tval, destination, enabled_pending;
    integer opcode, funct3, rd, rs1, rs2;
    logic writes, csr_write;
    begin
      if (event_record.valid) begin
        check(!finished, "no architectural event follows the serialized final WFI");
        check(event_record.order == events_seen,
              "architectural trace order has no gaps or repeats");
        check(event_record.pc == reference_pc, "precise event uses the next architectural PC");
        events_seen = events_seen + 1;
        if (event_record.trap) begin
          check(event_record.rd == 0 && !event_record.memory_write,
                "a trap does not write a destination or publish a store");
          if (event_record.interrupt) begin
            check(allow_interrupt, "this scenario permits an interrupt after admitted work drains");
            check(!sync_expected, "an older synchronous fault precedes a pending interrupt");
            enabled_pending = ref_mie & ((irq_external ? 32'h800 : 0)
                              | (irq_software ? 32'h8 : 0) | (irq_timer ? 32'h80 : 0));
            check(ref_status[3] && enabled_pending != 0,
                  "interrupt eligibility is rechecked at entry");
            if (enabled_pending[11]) expected_cause = CAUSE_IRQ_EXTERNAL;
            else if (enabled_pending[3]) expected_cause = CAUSE_IRQ_SOFTWARE;
            else expected_cause = CAUSE_IRQ_TIMER;
            expected_tval = 0;
            irq_seen = irq_seen + 1;
            check(dut.rob_occupancy == 0 && dut.cluster_quiescent && dut.lsu_quiescent
                  && dut.writeback_quiescent && dut.csr_quiescent && !d_pending,
                  "interrupt entry occurs only after admitted execution and data traffic drain");
          end else begin
            check(sync_expected && event_record.pc == expected_sync_pc,
                  "only the intended faulting instruction creates a synchronous trap");
            expected_cause = expected_sync_cause;
            expected_tval = expected_sync_tval;
            sync_expected = 0;
            sync_seen = sync_seen + 1;
          end
          check(event_record.cause == expected_cause && event_record.tval == expected_tval,
                "trap cause and fault address/instruction match the architectural reference");
          destination = ref_mtvec & 32'hfffffffc;
          if (event_record.interrupt && ref_mtvec[1:0] == 1)
            destination = destination + ((expected_cause & 32'h7fffffff) << 2);
          check(event_record.next_pc == destination,
                "direct/vectored mtvec selects the exact target");
          ref_mepc = reference_pc;
          ref_mcause = expected_cause;
          ref_mtval = expected_tval;
          ref_status[7] = ref_status[3];
          ref_status[3] = 0;
          reference_pc = destination;
          if (event_record.interrupt) begin
            irq_external = 0; irq_software = 0; irq_timer = 0;
          end
        end else begin
          instruction = event_record.instruction;
          check(reference_pc < RAM_BYTES && instruction == memory[reference_pc >> 2],
                "normal retirement matches the RAM instruction at the reference PC");
          opcode = instruction[6:0]; funct3 = instruction[14:12];
          rd = instruction[11:7]; rs1 = instruction[19:15]; rs2 = instruction[24:20];
          result = 0; writes = 0; next_pc = reference_pc + 4;
          immediate = {{20{instruction[31]}}, instruction[31:20]};
          case (opcode)
            7'h13: begin
              check(funct3 == 0, "trap-suite integer fixture uses ADDI");
              result = reference_regs[rs1] + immediate; writes = 1;
            end
            7'h37: begin result = {instruction[31:12], 12'b0}; writes = 1; end
            7'h33: begin
              check(instruction[31:25] == 1 && funct3 == 4, "long-operation fixture uses DIV");
              result = $signed(reference_regs[rs1]) / $signed(reference_regs[rs2]); writes = 1;
            end
            7'h03: begin
              value = reference_regs[rs1] + immediate;
              check(funct3 == 2 && value[1:0] == 0 && value < RAM_BYTES,
                    "successful memory fixture is an aligned LW");
              result = memory[value >> 2]; writes = 1;
            end
            7'h23: begin
              immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
              value = reference_regs[rs1] + immediate;
              check(funct3 == 2 && event_record.memory_write
                    && event_record.memory_address == value
                    && event_record.memory_wstrb == 4'b1111
                    && event_record.memory_wdata == reference_regs[rs2],
                    "successful store trace publishes the correct address/data exactly once");
            end
            7'h6f: begin
              immediate = {{11{instruction[31]}}, instruction[31], instruction[19:12],
                           instruction[20], instruction[30:21], 1'b0};
              result = reference_pc + 4; writes = 1; next_pc = reference_pc + immediate;
            end
            7'h73: begin
              if (instruction == 32'h30200073) begin
                next_pc = ref_mepc;
                ref_status[3] = ref_status[7]; ref_status[7] = 1;
              end else if (instruction == 32'h10500073) begin
                wfi_seen = wfi_seen + 1;
              end else begin
                check(funct3 != 0, "normal system fixture is CSR, MRET, or WFI");
                old_csr = read_reference_csr(instruction[31:20]);
                operand = funct3[2] ? word_t'(rs1) : reference_regs[rs1];
                csr_write = 0; value = old_csr;
                case (funct3)
                  1, 5: begin csr_write = 1; value = operand; end
                  2, 6: begin csr_write = rs1 != 0; value = old_csr | operand; end
                  3, 7: begin csr_write = rs1 != 0; value = old_csr & ~operand; end
                  default: check(0, "fixture has a supported CSR operation");
                endcase
                result = old_csr; writes = 1;
                if (csr_write) write_reference_csr(instruction[31:20], value);
              end
            end
            default: check(0, "no unexpected instruction retires in trap suite");
          endcase
          check(event_record.next_pc == next_pc, "normal control flow and MRET target are correct");
          if (writes && rd != 0) begin
            check(event_record.rd == rd && event_record.rd_value == result,
                  "retired GPR value matches the architectural reference");
            reference_regs[rd] = result;
          end else check(event_record.rd == 0,
                         "nonwriters and x0 do not alter an architectural GPR");
          reference_regs[0] = 0;
          normal_seen = normal_seen + 1;
          reference_pc = next_pc;
          if (event_record.pc == finish_pc && instruction == 32'h10500073) begin
            finished = 1; halt = 1;
          end
        end
      end
    end
  endtask

  task automatic step;
    logic response_transfer;
    logic [3:0] byte_enable;
    word_t store_data;
    begin
      #1;
      check(!protocol_error && !adapter_error, "legal core/adapter traffic has no protocol error");
      if (case_number == 33 || case_number == 34) begin
        if (d_pending && d_delay > 0 && dut.mispredict) saw_younger_redirect = 1;
        if (d_pending && dut.mispredict && dut.cluster_busy[2]) saw_younger_div_busy = 1;
        if (dut.md_valid) saw_younger_div_result = 1;
      end
      if (dut.lsu_occupancy > maximum_memory_queue) maximum_memory_queue = dut.lsu_occupancy;
      captured_request = d_valid && d_ready;
      accepted_request = d_request; // Capture before the edge can advance the owner's state.
      byte_enable = accepted_request.wstrb;
      store_data = accepted_request.wdata;
      response_transfer = d_rsp_valid && d_rsp_ready;
      #1; clk = 1; #1; clk = 0;
      if (response_transfer) d_pending = 0;
      else if (d_pending && d_delay > 0) d_delay = d_delay - 1;
      if (captured_request) begin
        memory_requests = memory_requests + 1;
        saved_response = '0;
        if (accepted_request.address >= RAM_BYTES) begin
          saved_response.error = 1;
          if (accepted_request.write) mmio_writes = mmio_writes + 1;
          else mmio_reads = mmio_reads + 1;
        end else begin
          saved_response.rdata = memory[accepted_request.address >> 2];
          if (accepted_request.write) begin
            for (integer lane = 0; lane < 4; lane++) begin
              if (byte_enable[lane])
                memory[accepted_request.address >> 2][lane*8 +: 8]
                  = store_data[lane*8 +: 8];
            end
          end
        end
        d_pending = 1; d_delay = data_latency;
      end
      #1;
      observe(trace.lane0); observe(trace.lane1);
      total_cycles = total_cycles + 1;
    end
  endtask

  task automatic reset_case;
    begin
      halt = 0; rst_n = 0; clk = 0;
      irq_external = 0; irq_software = 0; irq_timer = 0;
      d_pending = 0; d_delay = 0; saved_response = '0; bram_data = 0;
      reference_pc = 0; ref_status = 32'h1800; ref_mie = 0; ref_mtvec = 0;
      ref_mepc = 0; ref_mcause = 0; ref_mtval = 0;
      events_seen = 0; normal_seen = 0; sync_seen = 0; irq_seen = 0;
      memory_requests = 0; mmio_reads = 0; mmio_writes = 0;
      maximum_memory_queue = 0; wfi_seen = 0; finished = 0; injected = 0;
      saw_younger_redirect = 0; saw_younger_div_busy = 0; saw_younger_div_result = 0;
      sync_expected = 0; allow_interrupt = 0;
      finish_pc = 32'hffffffff;
      expected_mmio_reads = 0; expected_mmio_writes = 0;
      for (integer i = 0; i < 32; i++) reference_regs[i] = 0;
      for (integer i = 0; i < RAM_WORDS; i++) memory[i] = NOP_INSTRUCTION;
      #2; rst_n = 1; #1;
    end
  endtask

  task automatic install_handler(input logic synchronous_fault);
    begin
      // Direct and each supported vectored entry all lead to the same readable body.
      memory['h100 >> 2] = op_j('h180 - 'h100, 0);
      memory['h10c >> 2] = op_j('h180 - 'h10c, 0);
      memory['h11c >> 2] = op_j('h180 - 'h11c, 0);
      memory['h12c >> 2] = op_j('h180 - 'h12c, 0);
      memory['h180 >> 2] = csr(CSR_MCAUSE, 2, 0, 15);
      memory['h184 >> 2] = csr(CSR_MEPC, 2, 0, 16);
      memory['h188 >> 2] = csr(CSR_MTVAL, 2, 0, 17);
      memory['h18c >> 2] = csr(CSR_MSTATUS, 2, 0, 18);
      if (synchronous_fault) begin
        memory['h190 >> 2] = op_i(24, 0, 0, 19, 'h13);
        memory['h194 >> 2] = csr(CSR_MEPC, 1, 19, 0);
        memory['h198 >> 2] = 32'h30200073;
      end else memory['h190 >> 2] = 32'h30200073;
    end
  endtask

  task automatic run_until_finished(input integer limit_cycles);
    begin
      watchdog = 0;
      while (!finished && watchdog < limit_cycles) begin step; watchdog = watchdog + 1; end
      check(finished, "program returns through MRET and reaches its serialized WFI terminator");
      for (integer i = 0; i < 5; i++) step;
      check(empty && !d_pending, "terminated program drains all admitted work and transactions");
      for (integer i = 0; i < 32; i++)
        check(architectural_register(i) == reference_regs[i],
              "committed physical mapping retains every architectural GPR");
    end
  endtask

  task automatic setup_interrupt_program(input integer vector_mode, profile);
    begin
      install_handler(0);
      memory[0] = op_i('h100 + vector_mode, 0, 0, 1, 'h13);
      memory[1] = csr(CSR_MTVEC, 1, 1, 0);
      memory[2] = op_i(-1912, 0, 0, 2, 'h13); // Low supported bits are 0x888.
      memory[3] = csr(CSR_MIE, 1, 2, 0);
      memory[4] = csr(CSR_MSTATUS, 6, 8, 0);
      if (profile == 1) begin
        memory[5] = op_i('h700, 0, 0, 3, 'h13);
        memory[6] = op_i(0, 3, 2, 5, 'h03);
        memory[7] = op_i(1, 5, 0, 6, 'h13);
        memory[8] = op_s(4, 6, 3, 2);
        memory['h700 >> 2] = 32'h12345678;
      end else begin
        memory[5] = op_i(100, 0, 0, 3, 'h13);
        memory[6] = op_i(4, 0, 0, 4, 'h13);
        memory[7] = op_r(1, 4, 3, 4, 5);
        memory[8] = op_i(1, 5, 0, 6, 'h13);
      end
      for (integer pc = 36; pc < 'h8c; pc = pc + 4)
        memory[pc >> 2] = op_i(pc, 0, 0, 7, 'h13);
      memory['h8c >> 2] = csr(CSR_MIE, 5, 0, 0);
      memory['h90 >> 2] = 32'h10500073;
      finish_pc = 'h90;
      allow_interrupt = 1;
    end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/core_traps/core_traps.vcd");
      $dumpvars(0, tb_core_traps);
    end
    #1000000; $fatal(1, "Core trap/interrupt test timed out");
  end

  initial begin
    // Every handler reads actual trap CSRs through instructions, then resumes via MRET.
    for (mode = 0; mode < 11; mode++) begin
      case_number = mode; reset_case; install_handler(1);
      memory[0] = op_i('h100, 0, 0, 1, 'h13);
      memory[1] = csr(CSR_MTVEC, 1, 1, 0);
      memory[2] = 32'h10000537; // LUI x10,0x10000: a visible wrong-path MMIO base.
      expected_sync_pc = 12; expected_sync_cause = CAUSE_ILLEGAL_INSTR;
      expected_sync_tval = 32'hffffffff;
      case (mode)
        0: memory[3] = 32'hffffffff;
        1: begin memory[3] = 32'h00000073; expected_sync_cause = CAUSE_ECALL_M;
                 expected_sync_tval = 0; end
        2: begin memory[3] = 32'h00100073; expected_sync_cause = CAUSE_BREAKPOINT;
                 expected_sync_tval = 12; end
        3: begin memory[3] = op_i(1, 0, 2, 9, 'h03);
                 expected_sync_cause = CAUSE_LOAD_MISALIGNED; expected_sync_tval = 1; end
        4: begin memory[3] = op_s(2, 0, 0, 2);
                 expected_sync_cause = CAUSE_STORE_MISALIGNED; expected_sync_tval = 2; end
        5: begin memory[3] = csr('h777, 2, 0, 9); expected_sync_tval = memory[3]; end
        6: begin memory[3] = op_j(2, 9); expected_sync_cause = CAUSE_INSTR_MISALIGNED;
                 expected_sync_tval = 14; end
        7: begin memory[3] = op_j('h10000 - 12, 0); expected_sync_pc = 'h10000;
                 expected_sync_cause = CAUSE_INSTR_ACCESS; expected_sync_tval = 'h10000; end
        8: begin memory[3] = op_i(0, 10, 2, 9, 'h03);
                 expected_sync_cause = CAUSE_LOAD_ACCESS; expected_sync_tval = 'h10000000;
                 expected_mmio_reads = 1; end
        9: begin memory[3] = op_s(0, 0, 10, 2);
                 expected_sync_cause = CAUSE_STORE_ACCESS; expected_sync_tval = 'h10000000;
                 expected_mmio_writes = 1; end
        10: begin
          // A nonzero encoded rs1 is still a CSR write when its stored value is zero.
          memory[3] = csr(CSR_MVENDORID, 2, 7, 9);
          expected_sync_tval = memory[3];
        end
      endcase
      memory[4] = op_s(0, 0, 10, 2);      // Wrong-path write must never reach the data endpoint.
      memory[5] = op_i(0, 10, 2, 11, 'h03); // Wrong-path MMIO read must also stay speculative.
      memory[6] = 32'h10500073;
      finish_pc = 24; sync_expected = 1;
      run_until_finished(900);
      check(sync_seen == 1 && irq_seen == 0 && !sync_expected,
            "exactly one synchronous trap is handled and returned");
      check(mmio_reads == expected_mmio_reads && mmio_writes == expected_mmio_writes,
            "only the intended faulting access reaches MMIO; all younger accesses are suppressed");
      check(reference_regs[9] == 0 && reference_regs[11] == 0,
            "fault destinations and wrong-path load destinations remain uncommitted");
    end

    // Interrupts asserted during a long operation must drain all admitted work first.
    for (irq_profile = 0; irq_profile < 8; irq_profile++) begin
      case_number = 20 + irq_profile; reset_case;
      setup_interrupt_program(irq_profile == 1 || irq_profile == 3
                              || irq_profile == 4 || irq_profile == 5 || irq_profile == 7,
                              irq_profile == 3 ? 1 : 0);
      data_latency = irq_profile == 3 ? 14 : 5;
      watchdog = 0;
      while (!injected && watchdog < 300) begin
        if ((irq_profile != 3 && dut.cluster_busy[2])
            || (irq_profile == 3 && d_pending && d_delay > 0)) begin
          irq_external = irq_profile == 0 || irq_profile == 4;
          irq_software = irq_profile == 0 || irq_profile == 1 || irq_profile == 3
                         || irq_profile == 6 || irq_profile == 7;
          irq_timer = irq_profile == 0 || irq_profile == 2 || irq_profile == 5 || irq_profile == 7;
          injected = 1;
        end
        step; watchdog = watchdog + 1;
      end
      check(injected, "test asserted an IRQ during the intended outstanding operation");
      run_until_finished(1400);
      check(irq_seen == 1 && sync_seen == 0,
            "one enabled prioritized interrupt returns without skipped or duplicate instructions");
      check(mmio_reads == 0 && mmio_writes == 0, "IRQ recovery creates no wrong-path MMIO access");
    end

    // Eligibility can disappear while an admitted CSR instruction drains behind DIV.
    case_number = 30; reset_case; setup_interrupt_program(0, 0);
    memory[8] = csr(CSR_MSTATUS, 7, 8, 0);
    memory[9] = op_i(1, 5, 0, 6, 'h13);
    memory[10] = csr(CSR_MIE, 5, 0, 0);
    memory[11] = 32'h10500073; finish_pc = 44; allow_interrupt = 0;
    watchdog = 0;
    while (!injected && watchdog < 300) begin
      if (dut.cluster_busy[2] && dut.serial_pending) begin irq_software = 1; injected = 1; end
      step; watchdog = watchdog + 1;
    end
    check(injected, "global-disable CSR was admitted before interrupt-drain started");
    run_until_finished(1000);
    check(irq_seen == 0 && !ref_status[3] && reference_regs[6] == 26,
          "eligibility is rechecked after CSR drain; a withdrawn interrupt is not taken");

    // A synchronous head fault wins first; a still-pending IRQ is reconsidered after MRET.
    case_number = 32; reset_case; install_handler(1);
    memory[0] = op_i('h100, 0, 0, 1, 'h13);
    memory[1] = csr(CSR_MTVEC, 1, 1, 0);
    memory[2] = op_i(8, 0, 0, 2, 'h13);
    memory[3] = csr(CSR_MIE, 1, 2, 0);
    memory[4] = csr(CSR_MSTATUS, 6, 8, 0);
    memory[5] = 32'h10000537;
    memory[6] = 32'h00000073;
    memory[7] = op_s(0, 0, 10, 2);
    memory[8] = op_i(0, 10, 2, 11, 'h03);
    memory[10] = 32'h10500073;
    memory['h190 >> 2] = op_i(40, 0, 0, 19, 'h13);
    finish_pc = 40; sync_expected = 1; allow_interrupt = 1;
    expected_sync_pc = 24; expected_sync_cause = CAUSE_ECALL_M; expected_sync_tval = 0;
    watchdog = 0;
    while (!injected && watchdog < 300) begin
      if (dut.machine.pending_q && dut.machine.owner_q.decoded.system_op == SYS_ECALL) begin
        irq_software = 1; injected = 1;
      end
      step; watchdog = watchdog + 1;
    end
    check(injected, "IRQ becomes pending after the synchronous fault owner has issued");
    run_until_finished(1400);
    check(sync_seen == 1 && irq_seen == 1 && mmio_reads == 0 && mmio_writes == 0,
          "fault entry wins, MRET rechecks pending IRQ, and neither path admits wrong MMIO");

    // A younger known redirect must never outrank an older faulting memory instruction.
    for (mode = 0; mode < 2; mode++) begin
      case_number = 33 + mode; reset_case; install_handler(1);
      memory[0] = op_i('h100, 0, 0, 1, 'h13);
      memory[1] = csr(CSR_MTVEC, 1, 1, 0);
      memory[2] = 32'h10000537;
      memory[3] = op_i(100, 0, 0, 3, 'h13);
      memory[4] = op_i(3, 0, 0, 4, 'h13);
      memory[5] = op_i(0, 10, 2, 9, 'h03);
      memory[6] = op_b(16, 0, 0, 0); // Cold taken branch resolves before older load returns.
      memory[7] = op_r(1, 4, 3, 4, 5);
      memory[8] = op_s(0, 0, 10, 2);
      memory[9] = op_i(0, 10, 2, 11, 'h03);
      memory[10] = op_s(0, 0, 10, 2); // Even the branch's real target is still younger.
      memory[11] = op_i(0, 10, 2, 11, 'h03);
      memory[12] = 32'h10500073;
      memory['h190 >> 2] = op_i(48, 0, 0, 19, 'h13);
      finish_pc = 48; sync_expected = 1;
      expected_sync_pc = 20; expected_sync_cause = CAUSE_LOAD_ACCESS;
      expected_sync_tval = 'h10000000;
      data_latency = mode == 0 ? 16 : 44;
      run_until_finished(1400);
      check(saw_younger_redirect && saw_younger_div_busy,
            "younger branch misprediction and active DIV coexist before the older fault returns");
      check(saw_younger_div_result == (mode == 1),
            "delay variants cover cancelling running DIV and discarding a completed younger DIV");
      check(sync_seen == 1 && irq_seen == 0 && mmio_reads == 1 && mmio_writes == 0,
            "older load faults precisely and no younger path reaches the peripheral endpoint");
      check(maximum_memory_queue >= 2,
            "younger wrong-path memory entries were actually admitted behind the faulting load");
      check(reference_regs[5] == 0 && reference_regs[9] == 0 && reference_regs[11] == 0,
            "neither fault destination nor cancelled/completed younger results commit");
      check(dut.predictor.pht_q[6] == 2'b01 && dut.predictor.btb_q[6] == '0,
            "a squashed younger branch never trains prediction at retirement");
    end

    // WFI may wake on locally enabled pending IRQ even when global MIE disables trap entry.
    for (mode = 0; mode < 2; mode++) begin
      case_number = 40 + mode; reset_case; install_handler(0);
      memory[0] = op_i('h100, 0, 0, 1, 'h13);
      memory[1] = csr(CSR_MTVEC, 1, 1, 0);
      memory[2] = op_i(8, 0, 0, 2, 'h13);
      memory[3] = csr(CSR_MIE, 1, 2, 0);
      if (mode == 1) begin
        memory[4] = csr(CSR_MSTATUS, 6, 8, 0);
        memory[5] = 32'h10500073;
        memory[6] = op_i(55, 0, 0, 5, 'h13);
        memory[7] = csr(CSR_MIE, 5, 0, 0);
        memory[8] = 32'h10500073; finish_pc = 32;
      end else begin
        memory[4] = 32'h10500073;
        memory[5] = op_i(55, 0, 0, 5, 'h13);
        memory[6] = csr(CSR_MIE, 5, 0, 0);
        memory[7] = 32'h10500073; finish_pc = 28;
      end
      allow_interrupt = mode == 1;
      watchdog = 0;
      while (!dut.wfi_wait && watchdog < 300) begin step; watchdog = watchdog + 1; end
      check(watchdog < 300 && wfi_seen == 1, "first WFI commits and waits with an ungated clock");
      for (n = 0; n < 5; n++) step;
      check(wfi_seen == 1 && reference_regs[5] == 0,
            "WFI admits no younger instruction before wake");
      irq_software = 1; injected = 1;
      if (mode == 0) begin
        watchdog = 0;
        while (reference_regs[5] != 55 && watchdog < 300) begin step; watchdog = watchdog + 1; end
        irq_software = 0;
      end
      run_until_finished(1000);
      check(irq_seen == mode && reference_regs[5] == 55 && wfi_seen == 2,
            "WFI wake respects global trap enable and resumes exactly the next instruction");
    end

    $display("CORE_TRAPS COVERAGE: 11 faults, 8 IRQ profiles, fault/redirect priority, WFI");
    $display("CORE_TRAPS SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Full-core trap/interrupt verification failed");
    $finish;
  end
endmodule
