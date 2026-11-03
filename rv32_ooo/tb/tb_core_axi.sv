// Portable full-core AXI integration. The slave below is an independent test model.
`timescale 1ns / 1ps
module tb_core_axi;
  import rv32_ooo_pkg::*;
  logic clk = 0, rst_n = 1, halt, empty, protocol_error;
  logic bram_enable, bram_block;
  logic [BRAM_FETCH_ADDR_W-1:0] bram_address;
  fetch_row_t bram_data = 0;
  word_t awaddr, wdata, araddr, rdata;
  logic [2:0] awprot, arprot;
  logic [3:0] wstrb;
  logic awvalid, awready, wvalid, wready, bvalid, bready;
  logic arvalid, arready, rvalid, rready;
  logic [1:0] bresp, rresp;
  logic timer_write, timer_read, gate_aw, gate_w, gate_ar, gate_b, gate_r;
  logic timer_awready, timer_wready, timer_bvalid, timer_arready, timer_rvalid;
  logic timer_irq, software_irq;
  word_t timer_rdata;
  logic [1:0] timer_bresp, timer_rresp;
  integer timer_writes, timer_reads, expected_timer_writes, expected_timer_reads;
  retire_trace_pair_t trace;
  word_t memory [0:RAM_WORDS-1];
  logic aw_seen, w_seen, b_pending, r_pending;
  word_t saved_awaddr, saved_wdata, saved_rdata;
  logic [3:0] saved_wstrb;
  logic [1:0] saved_bresp, saved_rresp;
  integer write_delay, read_delay, cycles, aw_wait, w_wait, ar_wait;
  integer writes, reads, fault_writes, mmio_accesses, aw_first, w_first;
  integer aw_stalls, w_stalls, ar_stalls, b_holds, r_holds;
  logic previous_aw_stall, previous_w_stall, previous_ar_stall;
  word_t held_awaddr, held_wdata, held_araddr;
  logic [3:0] held_wstrb;
  logic old_instruction_read;
  word_t expected_pc [0:63], expected_instruction [0:63], expected_value [0:63];
  word_t expected_next_pc [0:63], expected_cause [0:63], expected_tval [0:63];
  arch_reg_t expected_rd [0:63];
  logic expected_trap [0:63], expected_interrupt [0:63];
  integer expected_count, observed_count, checks = 0, failures = 0;
  integer case_number, watchdog, expected_writes, expected_reads, expected_fault_writes;
  localparam word_t FAULT_ADDRESS = 32'h0000_0600;

  rv32_ooo_axi_top dut (
    .clk_i(clk), .rst_ni(rst_n), .predict_enable_i(1'b1), .halt_i(halt),
    .irq_external_i(1'b0), .irq_software_i(software_irq), .irq_timer_i(timer_irq),
    .bram_block_i(bram_block), .bram_enable_o(bram_enable),
    .bram_address_o(bram_address), .bram_data_i(bram_data),
    .m_axi_awaddr_o(awaddr), .m_axi_awprot_o(awprot),
    .m_axi_awvalid_o(awvalid), .m_axi_awready_i(awready), .m_axi_wdata_o(wdata),
    .m_axi_wstrb_o(wstrb), .m_axi_wvalid_o(wvalid), .m_axi_wready_i(wready),
    .m_axi_bresp_i(bresp), .m_axi_bvalid_i(bvalid), .m_axi_bready_o(bready),
    .m_axi_araddr_o(araddr), .m_axi_arprot_o(arprot),
    .m_axi_arvalid_o(arvalid), .m_axi_arready_i(arready),
    .m_axi_rdata_i(rdata), .m_axi_rresp_i(rresp), .m_axi_rvalid_i(rvalid),
    .m_axi_rready_o(rready), .trace_o(trace), .empty_o(empty),
    .protocol_error_o(protocol_error)
  );

  // This test fabric has one outstanding master; its saved AW/AR address stays stable
  // until the response, so data/response routing remains fixed even when W arrives first.
  assign timer_write = awaddr[31:16] == 16'h0200;
  assign timer_read = araddr[31:16] == 16'h0200;
  assign gate_aw = aw_wait >= ((writes + fault_writes) % 2 == 0 ? 1 : 4);
  assign gate_w = w_wait >= ((writes + fault_writes) % 2 == 0 ? 4 : 1);
  assign gate_ar = ar_wait >= 2;
  assign gate_b = cycles % 7 >= 3;
  assign gate_r = cycles % 5 >= 2;
  assign awready = rst_n && gate_aw && (timer_write ? timer_awready : !aw_seen && !b_pending);
  assign wready = rst_n && gate_w && (timer_write ? timer_wready : !w_seen && !b_pending);
  assign arready = rst_n && gate_ar && (timer_read ? timer_arready : !r_pending);
  assign bvalid = rst_n && gate_b && (timer_write ? timer_bvalid : b_pending);
  assign bresp = timer_write ? timer_bresp : saved_bresp;
  assign rvalid = rst_n && gate_r
                  && (timer_read ? timer_rvalid : r_pending && read_delay == 0);
  assign rdata = timer_read ? timer_rdata : saved_rdata;
  assign rresp = timer_read ? timer_rresp : saved_rresp;
  rv32_machine_timer timer (
    .clk_i(clk), .rst_ni(rst_n), .s_axi_awaddr_i(awaddr), .s_axi_awprot_i(awprot),
    .s_axi_awvalid_i(awvalid && timer_write && gate_aw), .s_axi_awready_o(timer_awready),
    .s_axi_wdata_i(wdata), .s_axi_wstrb_i(wstrb),
    .s_axi_wvalid_i(wvalid && timer_write && gate_w), .s_axi_wready_o(timer_wready),
    .s_axi_bresp_o(timer_bresp), .s_axi_bvalid_o(timer_bvalid),
    .s_axi_bready_i(bready && timer_write && gate_b),
    .s_axi_araddr_i(araddr), .s_axi_arprot_i(arprot),
    .s_axi_arvalid_i(arvalid && timer_read && gate_ar), .s_axi_arready_o(timer_arready),
    .s_axi_rdata_o(timer_rdata), .s_axi_rresp_o(timer_rresp), .s_axi_rvalid_o(timer_rvalid),
    .s_axi_rready_i(rready && timer_read && gate_r),
    .timer_irq_o(timer_irq), .software_irq_o(software_irq)
  );
  // RAM read/write collisions are handled outside the portable core adapter.
  assign bram_block = aw_seen && w_seen && write_delay == 0
    && saved_awaddr < RAM_BYTES && saved_awaddr != FAULT_ADDRESS
    && saved_awaddr[15:3] == bram_address;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      aw_seen <= 0; w_seen <= 0; b_pending <= 0; r_pending <= 0;
      saved_awaddr <= 0; saved_wdata <= 0; saved_wstrb <= 0;
      saved_bresp <= 0; saved_rdata <= 0; saved_rresp <= 0;
      write_delay <= -1; read_delay <= 0; cycles <= 0; aw_wait <= 0; w_wait <= 0; ar_wait <= 0;
      writes <= 0; reads <= 0; fault_writes <= 0; mmio_accesses <= 0;
      aw_first <= 0; w_first <= 0; old_instruction_read <= 0;
      timer_writes <= 0; timer_reads <= 0;
    end else begin
      cycles <= cycles + 1;
      if (awvalid && !awready) aw_wait <= aw_wait + 1;
      if (wvalid && !wready) w_wait <= w_wait + 1;
      if (arvalid && !arready) ar_wait <= ar_wait + 1;
      if (bram_enable) begin
        bram_data <= {memory[int'(bram_address)*2+1], memory[int'(bram_address)*2]};
        if (case_number == 4 && bram_address == 3 && memory[6] == 32'h00100613)
          old_instruction_read <= 1;
      end
      if (awvalid && awready) begin
        aw_wait <= 0;
        if (!timer_write) begin
          saved_awaddr <= awaddr; aw_seen <= 1;
          if (!w_seen && !(wvalid && wready)) aw_first <= aw_first + 1;
        end
      end
      if (wvalid && wready) begin
        w_wait <= 0;
        if (!timer_write) begin
          saved_wdata <= wdata; saved_wstrb <= wstrb; w_seen <= 1;
          if (!aw_seen && !(awvalid && awready)) w_first <= w_first + 1;
        end
      end
      if (aw_seen && w_seen && !b_pending) begin
        if (write_delay < 0) write_delay <= 3;
        else if (write_delay > 0) write_delay <= write_delay - 1;
        else begin
          saved_bresp <= 0;
          if (saved_awaddr == FAULT_ADDRESS) begin
            saved_bresp <= 2'b10; fault_writes <= fault_writes + 1;
          end else if (saved_awaddr >= RAM_BYTES || |saved_awaddr[1:0]) begin
            saved_bresp <= 2'b11; mmio_accesses <= mmio_accesses + 1;
          end else begin
            for (integer lane = 0; lane < 4; lane++)
              if (saved_wstrb[lane])
                memory[saved_awaddr >> 2][lane*8 +: 8] <= saved_wdata[lane*8 +: 8];
            writes <= writes + 1;
          end
          b_pending <= 1; aw_seen <= 0; w_seen <= 0; write_delay <= -1;
        end
      end
      if (bvalid && bready && !timer_write) b_pending <= 0;
      if (bvalid && bready && timer_write) timer_writes <= timer_writes + 1;
      if (arvalid && arready) ar_wait <= 0;
      if (arvalid && arready && timer_read) timer_reads <= timer_reads + 1;
      if (arvalid && arready && !timer_read) begin
        r_pending <= 1; read_delay <= 3; reads <= reads + 1;
        saved_rresp <= 0;
        if (araddr == FAULT_ADDRESS) begin saved_rresp <= 2'b10; saved_rdata <= 32'hbad0cafe; end
        else if (araddr >= RAM_BYTES || |araddr[1:0]) begin
          saved_rresp <= 2'b11; saved_rdata <= 0; mmio_accesses <= mmio_accesses + 1;
        end else saved_rdata <= memory[araddr >> 2];
      end
      if (r_pending && read_delay > 0) read_delay <= read_delay - 1;
      if (rvalid && rready && !timer_read) r_pending <= 0;
    end
  end

  function automatic word_t immediate_op(input integer rd, rs1, immediate, funct3);
    immediate_op = ((immediate & 4095) << 20) | (rs1 << 15) | (funct3 << 12)
                   | (rd << 7) | 7'h13;
  endfunction
  function automatic word_t load_op(input integer rd, rs1, immediate, funct3);
    load_op = ((immediate & 4095) << 20) | (rs1 << 15) | (funct3 << 12)
              | (rd << 7) | 7'h03;
  endfunction
  function automatic word_t store_op(input integer rs2, rs1, immediate, funct3);
    store_op = (((immediate >> 5) & 127) << 25) | (rs2 << 20) | (rs1 << 15)
               | (funct3 << 12) | ((immediate & 31) << 7) | 7'h23;
  endfunction
  function automatic word_t branch_op(input integer rs1, rs2, immediate, funct3);
    branch_op = (((immediate >> 12) & 1) << 31) | (((immediate >> 5) & 63) << 25)
                | (rs2 << 20) | (rs1 << 15) | (funct3 << 12)
                | (((immediate >> 1) & 15) << 8) | (((immediate >> 11) & 1) << 7) | 7'h63;
  endfunction
  function automatic word_t csr_op(input integer rd, rs1, address, funct3);
    csr_op = (address << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | 7'h73;
  endfunction

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures < 35)
          $display("FAIL: %s case=%0d cycle=%0d time=%0t", label_text, case_number, cycles, $time);
      end
    end
  endtask
  task automatic expected(input word_t pc, input integer rd, input word_t value, next_pc);
    begin
      expected_pc[expected_count] = pc;
      expected_instruction[expected_count] = memory[pc >> 2];
      expected_rd[expected_count] = rd;
      expected_value[expected_count] = value;
      expected_next_pc[expected_count] = next_pc;
      expected_trap[expected_count] = 0; expected_interrupt[expected_count] = 0;
      expected_cause[expected_count] = 0;
      expected_tval[expected_count] = 0;
      expected_count = expected_count + 1;
    end
  endtask
  task automatic trap_expected(input word_t pc, cause, address);
    begin
      expected(pc, 0, 0, 256);
      expected_trap[expected_count-1] = 1;
      expected_cause[expected_count-1] = cause;
      expected_tval[expected_count-1] = address;
    end
  endtask
  task automatic interrupt_expected(input word_t pc, cause);
    begin
      expected(pc, 0, 0, 256);
      expected_instruction[expected_count-1] = 0;
      expected_trap[expected_count-1] = 1;
      expected_interrupt[expected_count-1] = 1;
      expected_cause[expected_count-1] = cause;
    end
  endtask
  task automatic observe_event(input retire_trace_t event_record);
    integer index;
    begin
      index = observed_count;
      check(index < expected_count, "retirement belongs to the test program");
      if (index < expected_count) begin
        check(event_record.order == index && event_record.pc == expected_pc[index],
              "architectural order and PC match expected control flow");
        check(event_record.instruction == expected_instruction[index],
              "instruction bits match the expected committed program");
        check(event_record.rd == expected_rd[index]
              && event_record.rd_value == expected_value[index],
              "architectural result matches expected arithmetic/load/CSR value");
        check(event_record.next_pc == expected_next_pc[index]
              && event_record.trap == expected_trap[index]
              && event_record.interrupt == expected_interrupt[index],
              "next PC and precise trap event match reference");
        if (case_number == 0 && event_record.pc == 12)
          check(event_record.memory_write && event_record.memory_address == 512
                && event_record.memory_wstrb == 4'b1111
                && event_record.memory_wdata == 32'h80ff7f01, "word-store retirement trace");
        else if (case_number == 0 && event_record.pc == 44)
          check(event_record.memory_write && event_record.memory_address == 513
                && event_record.memory_wstrb == 4'b0010
                && event_record.memory_wdata == 32'h00005500, "byte-store retirement trace");
        else if (case_number == 0 && event_record.pc == 48)
          check(event_record.memory_write && event_record.memory_address == 514
                && event_record.memory_wstrb == 4'b1100
                && event_record.memory_wdata == 32'h00550000, "halfword-store retirement trace");
        else if (case_number == 4 && event_record.pc == 12)
          check(event_record.memory_write && event_record.memory_address == 24
                && event_record.memory_wstrb == 4'b1111
                && event_record.memory_wdata == 32'h02a00613, "executable-memory store trace");
        else if (case_number == 5 && (event_record.pc == 28 || event_record.pc == 256))
          check(event_record.memory_write && event_record.memory_address == 32'h02000000
                && event_record.memory_wstrb == 4'b1111
                && event_record.memory_wdata == (event_record.pc == 28 ? 1 : 0),
                "MSIP store trace matches peripheral write");
        else if (case_number == 6 && (event_record.pc == 24 || event_record.pc == 28
                                     || event_record.pc == 260 || event_record.pc == 264))
          check(event_record.memory_write && event_record.memory_wstrb == 4'b1111
                && event_record.memory_address ==
                  ((event_record.pc == 28 || event_record.pc == 264) ? 32'h02004004 : 32'h02004000)
                && event_record.memory_wdata == (event_record.pc >= 260 ? 32'hffffffff : 0),
                "mtimecmp store trace matches selected low/high word");
        else check(!event_record.memory_write, "nonstore retirement has no store side effect");
        if (expected_trap[index]) begin
          check(event_record.cause == expected_cause[index]
                && event_record.tval == expected_tval[index] && !event_record.memory_write,
                "AXI error becomes a precise fault without store trace effects");
        end
      end
      observed_count = observed_count + 1;
    end
  endtask
  task automatic step;
    begin
      #1;
      check(!protocol_error, "composed core/adapter has no protocol violation");
      if (previous_aw_stall)
        check(awvalid && awaddr == held_awaddr,
              "AW payload holds through independent backpressure");
      if (previous_w_stall)
        check(wvalid && wdata == held_wdata && wstrb == held_wstrb,
              "W payload holds through independent backpressure");
      if (previous_ar_stall)
        check(arvalid && araddr == held_araddr, "AR payload holds through backpressure");
      previous_aw_stall = awvalid && !awready;
      previous_w_stall = wvalid && !wready;
      previous_ar_stall = arvalid && !arready;
      held_awaddr = awaddr; held_wdata = wdata; held_wstrb = wstrb; held_araddr = araddr;
      if (previous_aw_stall) aw_stalls = aw_stalls + 1;
      if (previous_w_stall) w_stalls = w_stalls + 1;
      if (previous_ar_stall) ar_stalls = ar_stalls + 1;
      if (b_pending && !bvalid) b_holds = b_holds + 1;
      if (r_pending && read_delay == 0 && !rvalid) r_holds = r_holds + 1;
      if (awvalid && awready) check(awprot == 3'b001, "AW is a privileged data request");
      if (arvalid && arready) check(arprot == 3'b001, "AR is a privileged data request");
      #1; clk = 1; #1;
      if (trace.lane0.valid) observe_event(trace.lane0);
      if (trace.lane1.valid) observe_event(trace.lane1);
      if (observed_count >= expected_count) halt = 1;
      clk = 0; #1;
    end
  endtask
  task automatic start_case;
    begin
      halt = 1; clk = 0; rst_n = 0; #2;
      expected_count = 0; observed_count = 0;
      expected_timer_writes = 0; expected_timer_reads = 0;
      previous_aw_stall = 0; previous_w_stall = 0; previous_ar_stall = 0;
      aw_stalls = 0; w_stalls = 0; ar_stalls = 0; b_holds = 0; r_holds = 0;
      for (integer i = 0; i < RAM_WORDS; i++) memory[i] = NOP_INSTRUCTION;
    end
  endtask
  task automatic run_case;
    begin
      rst_n = 1; halt = 0; #1; watchdog = 0;
      while (observed_count < expected_count && watchdog < 3000) begin
        step; watchdog = watchdog + 1;
      end
      check(observed_count == expected_count && watchdog < 3000, "all expected events retire");
      halt = 1;
      for (integer i = 0; i < 8; i++) step;
      check(empty && !b_pending && !r_pending && !aw_seen && !w_seen
            && !timer_bvalid && !timer_rvalid,
            "backend and AXI ownership drain after program completion");
      check(writes == expected_writes && reads == expected_reads
            && fault_writes == expected_fault_writes && mmio_accesses == 0
            && timer_writes == expected_timer_writes && timer_reads == expected_timer_reads,
            "AXI transactions occur exactly once and no wrong-path MMIO reaches the bus");
      $display("CORE_AXI CASE %0d: %0d events, %0d cycles, RAM W/R=%0d/%0d, timer W/R=%0d/%0d",
               case_number, observed_count, cycles, writes, reads, timer_writes, timer_reads);
    end
  endtask

  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/core_axi/core_axi.vcd");
      $dumpvars(0, tb_core_axi);
    end
    #200000; $fatal(1, "Core AXI test timed out");
  end
  initial begin
    case_number = 0; start_case;
    memory[0] = immediate_op(1, 0, 512, 0);
    memory[1] = (32'h80ff8 << 12) | (2 << 7) | 7'h37;
    memory[2] = immediate_op(2, 2, -255, 0);
    memory[3] = store_op(2, 1, 0, 2);
    memory[4] = load_op(3, 1, 0, 0); memory[5] = load_op(4, 1, 2, 0);
    memory[6] = load_op(5, 1, 3, 4); memory[7] = load_op(6, 1, 2, 1);
    memory[8] = load_op(7, 1, 2, 5); memory[9] = load_op(8, 1, 0, 2);
    memory[10] = immediate_op(9, 0, 85, 0);
    memory[11] = store_op(9, 1, 1, 0); memory[12] = store_op(9, 1, 2, 1);
    memory[13] = load_op(10, 1, 0, 2);
    memory[14] = (9 << 20) | (10 << 15) | (11 << 7) | 7'h33;
    memory[15] = 32'h10500073;
    expected(0, 1, 512, 4); expected(4, 2, 32'h80ff8000, 8);
    expected(8, 2, 32'h80ff7f01, 12); expected(12, 0, 0, 16);
    expected(16, 3, 1, 20); expected(20, 4, 32'hffffffff, 24);
    expected(24, 5, 128, 28); expected(28, 6, 32'hffff80ff, 32);
    expected(32, 7, 32'h80ff, 36); expected(36, 8, 32'h80ff7f01, 40);
    expected(40, 9, 85, 44); expected(44, 0, 0, 48); expected(48, 0, 0, 52);
    expected(52, 10, 32'h00555501, 56); expected(56, 11, 32'h00555556, 60);
    expected(60, 0, 0, 64);
    expected_writes = 3; expected_reads = 7; expected_fault_writes = 0; run_case;
    check(memory[128] == 32'h00555501, "byte/halfword strobes preserve unselected RAM bytes");
    check(aw_stalls > 0 && w_stalls > 0 && ar_stalls > 0 && aw_first > 0 && w_first > 0,
          "AW-first, W-first, and every request-channel stall occurred");

    case_number = 1; start_case;
    memory[0] = (32'h10000 << 12) | (2 << 7) | 7'h37;
    memory[1] = immediate_op(1, 0, 1, 0);
    memory[2] = branch_op(1, 1, 12, 0);
    memory[3] = store_op(1, 2, 0, 2); memory[4] = immediate_op(3, 0, 99, 0);
    memory[5] = immediate_op(3, 0, 7, 0); memory[6] = 32'h10500073;
    expected(0, 2, 32'h10000000, 4); expected(4, 1, 1, 8); expected(8, 0, 0, 20);
    expected(20, 3, 7, 24); expected(24, 0, 0, 28);
    expected_writes = 0; expected_reads = 0; run_case;

    for (case_number = 2; case_number < 4; case_number++) begin
      start_case;
      memory[0] = immediate_op(1, 0, 256, 0); memory[1] = csr_op(0, 1, CSR_MTVEC, 1);
      memory[2] = (32'h10000 << 12) | (4 << 7) | 7'h37;
      memory[3] = immediate_op(2, 0, FAULT_ADDRESS, 0);
      memory[4] = case_number == 2 ? load_op(3, 2, 0, 2) : store_op(1, 2, 0, 2);
      memory[5] = store_op(1, 4, 0, 2);
      memory[64] = csr_op(5, 0, CSR_MCAUSE, 2);
      memory[65] = csr_op(6, 0, CSR_MTVAL, 2);
      memory[66] = csr_op(7, 0, CSR_MEPC, 2); memory[67] = 32'h10500073;
      expected(0, 1, 256, 4); expected(4, 0, 0, 8);
      expected(8, 4, 32'h10000000, 12); expected(12, 2, FAULT_ADDRESS, 16);
      trap_expected(16, case_number == 2 ? CAUSE_LOAD_ACCESS : CAUSE_STORE_ACCESS, FAULT_ADDRESS);
      expected(256, 5, case_number == 2 ? CAUSE_LOAD_ACCESS : CAUSE_STORE_ACCESS, 260);
      expected(260, 6, FAULT_ADDRESS, 264); expected(264, 7, 16, 268);
      expected(268, 0, 0, 272);
      expected_writes = 0; expected_reads = case_number == 2 ? 1 : 0;
      expected_fault_writes = case_number == 3 ? 1 : 0; run_case;
      check(memory[FAULT_ADDRESS >> 2] == NOP_INSTRUCTION,
            "failed AXI write leaves memory unchanged");
    end

    case_number = 4; start_case;
    memory[0] = immediate_op(1, 0, 24, 0);
    memory[1] = (32'h02a00 << 12) | (2 << 7) | 7'h37;
    memory[2] = immediate_op(2, 2, 12'h613, 0); memory[3] = store_op(2, 1, 0, 2);
    memory[4] = 32'h0000100f; memory[5] = immediate_op(13, 0, 7, 0);
    memory[6] = immediate_op(12, 0, 1, 0); memory[7] = 32'h10500073;
    expected(0, 1, 24, 4); expected(4, 2, 32'h02a00000, 8);
    expected(8, 2, 32'h02a00613, 12); expected(12, 0, 0, 16);
    expected(16, 0, 0, 20); expected(20, 13, 7, 24);
    expected(24, 12, 42, 28); expected_instruction[6] = 32'h02a00613;
    expected(28, 0, 0, 32);
    expected_writes = 1; expected_reads = 0; expected_fault_writes = 0; run_case;
    check(old_instruction_read, "old target instruction was prefetched before FENCE.I refetch");
    check(memory[6] == 32'h02a00613, "FENCE.I test modified executable shared RAM");

    // Program the actual AXI timer slave, take a precise interrupt, clear it in
    // the handler, and return to a peripheral read before the final WFI.
    for (case_number = 5; case_number < 7; case_number++) begin
      start_case;
      memory[0] = immediate_op(1, 0, 256, 0); memory[1] = csr_op(0, 1, CSR_MTVEC, 1);
      memory[2] = ((case_number == 5 ? 32'h02000 : 32'h02004) << 12) | (2 << 7) | 7'h37;
      memory[3] = immediate_op(3, 0, case_number == 5 ? 8 : 128, 0);
      memory[4] = csr_op(0, 3, CSR_MIE, 1); memory[5] = csr_op(0, 8, CSR_MSTATUS, 6);
      if (case_number == 5) begin
        memory[6] = immediate_op(4, 0, 1, 0); memory[7] = store_op(4, 2, 0, 2);
      end else begin
        memory[6] = store_op(0, 2, 0, 2); memory[7] = store_op(0, 2, 4, 2);
      end
      memory[8] = 32'h10500073; memory[9] = load_op(9, 2, 0, 2);
      memory[10] = immediate_op(5, 0, 77, 0); memory[11] = 32'h10500073;
      if (case_number == 5) begin
        memory[64] = store_op(0, 2, 0, 2); memory[65] = csr_op(6, 0, CSR_MCAUSE, 2);
        memory[66] = csr_op(7, 0, CSR_MEPC, 2); memory[67] = 32'h30200073;
      end else begin
        memory[64] = immediate_op(4, 0, -1, 0); memory[65] = store_op(4, 2, 0, 2);
        memory[66] = store_op(4, 2, 4, 2); memory[67] = csr_op(6, 0, CSR_MCAUSE, 2);
        memory[68] = csr_op(7, 0, CSR_MEPC, 2); memory[69] = 32'h30200073;
      end
      expected(0, 1, 256, 4); expected(4, 0, 0, 8);
      expected(8, 2, case_number == 5 ? 32'h02000000 : 32'h02004000, 12);
      expected(12, 3, case_number == 5 ? 8 : 128, 16);
      expected(16, 0, 0, 20); expected(20, 0, 0, 24);
      expected(24, case_number == 5 ? 4 : 0, case_number == 5 ? 1 : 0, 28);
      expected(28, 0, 0, 32); expected(32, 0, 0, 36);
      interrupt_expected(36, case_number == 5 ? CAUSE_IRQ_SOFTWARE : CAUSE_IRQ_TIMER);
      if (case_number == 5) begin
        expected(256, 0, 0, 260); expected(260, 6, CAUSE_IRQ_SOFTWARE, 264);
        expected(264, 7, 36, 268); expected(268, 0, 0, 36);
        expected_timer_writes = 2;
      end else begin
        expected(256, 4, 32'hffffffff, 260); expected(260, 0, 0, 264);
        expected(264, 0, 0, 268); expected(268, 6, CAUSE_IRQ_TIMER, 272);
        expected(272, 7, 36, 276); expected(276, 0, 0, 36);
        expected_timer_writes = 4;
      end
      expected(36, 9, case_number == 5 ? 0 : 32'hffffffff, 40);
      expected(40, 5, 77, 44); expected(44, 0, 0, 48);
      expected_timer_reads = 1;
      expected_writes = 0; expected_reads = 0; expected_fault_writes = 0; run_case;
      check(!software_irq && !timer_irq, "handler clears the actual peripheral pending interrupt");
    end

    $display("CORE_AXI SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Core AXI integration failed");
    $finish;
  end
endmodule
