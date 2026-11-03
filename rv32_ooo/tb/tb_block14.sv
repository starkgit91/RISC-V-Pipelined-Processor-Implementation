// AXI master/timer tests with independent channel delays and a direct slave probe.
`timescale 1ns / 1ps
module tb_block14;
  import rv32_ooo_pkg::*;
  logic clk = 0, rst_n = 1;
  logic req_valid, req_ready, rsp_valid, rsp_ready, busy, master_error;
  data_request_t request;
  memory_response_t response;
  word_t awaddr, wdata, araddr, rdata;
  logic [2:0] awprot, arprot;
  logic [3:0] wstrb;
  logic awvalid, awready, wvalid, wready, bvalid, bready;
  logic arvalid, arready, rvalid, rready;
  logic [1:0] bresp, rresp;
  logic gate_aw, gate_w, gate_b, gate_ar, gate_r, direct;
  logic inject_b, inject_r;
  logic [1:0] injected_bresp, injected_rresp;
  word_t t_awaddr, t_wdata, t_araddr, t_rdata;
  logic [2:0] t_awprot, t_arprot;
  logic [3:0] t_wstrb;
  logic t_awvalid, t_awready, t_wvalid, t_wready, t_bvalid, t_bready;
  logic t_arvalid, t_arready, t_rvalid, t_rready;
  logic [1:0] t_bresp, t_rresp;
  word_t d_awaddr, d_wdata, d_araddr;
  logic [3:0] d_wstrb;
  logic d_awvalid, d_wvalid, d_bready, d_arvalid, d_rready;
  logic timer_irq, software_irq;
  integer checks = 0, failures = 0, n, watchdog;
  word_t result, earlier, later;
  logic [1:0] result_error;

  rv32_axi_lite_master master (
    .clk_i(clk), .rst_ni(rst_n), .request_valid_i(req_valid), .request_ready_o(req_ready),
    .request_i(request), .response_valid_o(rsp_valid), .response_ready_i(rsp_ready),
    .response_o(response), .m_axi_awaddr_o(awaddr), .m_axi_awprot_o(awprot),
    .m_axi_awvalid_o(awvalid), .m_axi_awready_i(awready), .m_axi_wdata_o(wdata),
    .m_axi_wstrb_o(wstrb), .m_axi_wvalid_o(wvalid), .m_axi_wready_i(wready),
    .m_axi_bresp_i(bresp), .m_axi_bvalid_i(bvalid), .m_axi_bready_o(bready),
    .m_axi_araddr_o(araddr), .m_axi_arprot_o(arprot), .m_axi_arvalid_o(arvalid),
    .m_axi_arready_i(arready), .m_axi_rdata_i(rdata), .m_axi_rresp_i(rresp),
    .m_axi_rvalid_i(rvalid), .m_axi_rready_o(rready), .busy_o(busy),
    .protocol_error_o(master_error)
  );
  rv32_machine_timer timer (
    .clk_i(clk), .rst_ni(rst_n), .s_axi_awaddr_i(t_awaddr), .s_axi_awprot_i(t_awprot),
    .s_axi_awvalid_i(t_awvalid), .s_axi_awready_o(t_awready), .s_axi_wdata_i(t_wdata),
    .s_axi_wstrb_i(t_wstrb), .s_axi_wvalid_i(t_wvalid), .s_axi_wready_o(t_wready),
    .s_axi_bresp_o(t_bresp), .s_axi_bvalid_o(t_bvalid), .s_axi_bready_i(t_bready),
    .s_axi_araddr_i(t_araddr), .s_axi_arprot_i(t_arprot), .s_axi_arvalid_i(t_arvalid),
    .s_axi_arready_o(t_arready), .s_axi_rdata_o(t_rdata), .s_axi_rresp_o(t_rresp),
    .s_axi_rvalid_o(t_rvalid), .s_axi_rready_i(t_rready),
    .timer_irq_o(timer_irq), .software_irq_o(software_irq)
  );

  // Mask both valid and ready at each inserted delay, preserving one real handshake.
  assign t_awaddr = direct ? d_awaddr : awaddr;
  assign t_awprot = direct ? 3'b001 : awprot;
  assign t_awvalid = direct ? d_awvalid : awvalid && gate_aw;
  assign awready = !direct && gate_aw && t_awready;
  assign t_wdata = direct ? d_wdata : wdata;
  assign t_wstrb = direct ? d_wstrb : wstrb;
  assign t_wvalid = direct ? d_wvalid : wvalid && gate_w;
  assign wready = !direct && gate_w && t_wready;
  assign t_bready = direct ? d_bready : bready && gate_b;
  assign bvalid = !direct && gate_b && t_bvalid;
  assign bresp = inject_b ? injected_bresp : t_bresp;
  assign t_araddr = direct ? d_araddr : araddr;
  assign t_arprot = direct ? 3'b001 : arprot;
  assign t_arvalid = direct ? d_arvalid : arvalid && gate_ar;
  assign arready = !direct && gate_ar && t_arready;
  assign t_rready = direct ? d_rready : rready && gate_r;
  assign rvalid = !direct && gate_r && t_rvalid;
  assign rdata = t_rdata;
  assign rresp = inject_r ? injected_rresp : t_rresp;

  task automatic check(input logic condition, input string label_text);
    begin
      checks = checks + 1;
      if (condition !== 1'b1) begin
        failures = failures + 1;
        if (failures <= 30) $display("FAIL: %s time=%0t", label_text, $time);
      end
    end
  endtask
  task automatic edge_step;
    begin #1; clk = 1; #1; clk = 0; #1; end
  endtask
  task automatic controls;
    begin
      req_valid = 0; rsp_ready = 0; request = '0;
      gate_aw = 1; gate_w = 1; gate_b = 1; gate_ar = 1; gate_r = 1;
      direct = 0; inject_b = 0; inject_r = 0; injected_bresp = 0; injected_rresp = 0;
      d_awaddr = 0; d_wdata = 0; d_wstrb = 0; d_araddr = 0;
      d_awvalid = 0; d_wvalid = 0; d_bready = 0; d_arvalid = 0; d_rready = 0;
    end
  endtask
  task automatic reset_all;
    begin
      controls; rst_n = 0; #2;
      check(!req_ready && !rsp_valid && !awvalid && !wvalid && !arvalid && !rready && !bready,
            "reset masks every master transfer");
      check(!t_bvalid && !t_rvalid && !timer_irq && !software_irq,
            "reset clears slave responses and interrupts");
      rst_n = 1; #1;
      check(req_ready && !busy, "master returns to idle after reset");
    end
  endtask
  task automatic core_request(input logic write_access, input word_t address, value,
                              input logic [3:0] strobes);
    begin
      request.write = write_access; request.address = address;
      request.wdata = value; request.wstrb = strobes; req_valid = 1;
      #1; check(req_ready, "idle bridge accepts a core request");
      edge_step; req_valid = 0;
      check(busy && !req_ready, "one outstanding transaction excludes new core acceptance");
    end
  endtask
  task automatic finish_core(input logic expected_error);
    memory_response_t saved;
    begin
      watchdog = 0;
      while (!rsp_valid && watchdog < 30) begin edge_step; watchdog = watchdog + 1; end
      check(rsp_valid && watchdog < 30, "core response arrives within timeout");
      check(response.error == expected_error,
            "AXI response code propagates as architectural error");
      saved = response; result = response.rdata;
      for (integer hold = 0; hold < 3; hold++) begin
        edge_step;
        check(rsp_valid && response == saved && !req_ready,
              "core result remains stable under response backpressure");
      end
      rsp_ready = 1; edge_step; rsp_ready = 0;
      check(req_ready && !rsp_valid && !busy, "response consumed once before next transaction");
      check(!master_error, "legal AXI traffic produces no bridge diagnostic");
    end
  endtask
  task automatic write_core(input word_t address, value, input logic [3:0] strobes,
                            input integer ordering, input logic expected_error);
    begin
      gate_aw = 0; gate_w = 0; gate_b = 0;
      core_request(1, address, value, strobes);
      for (integer hold = 0; hold < 3; hold++) begin
        check(awvalid && wvalid && awaddr == address && wdata == value && wstrb == strobes,
              "write address/data/strobes remain stable before independent acceptance");
        check(!rsp_valid, "a write is not complete before its B response"); edge_step;
      end
      if (ordering == 0) begin
        gate_aw = 1; edge_step;
        check(!awvalid && wvalid, "AW may transfer before W");
        repeat (2) edge_step;
        gate_w = 1; edge_step;
      end else if (ordering == 1) begin
        gate_w = 1; edge_step;
        check(awvalid && !wvalid, "W may transfer before AW");
        repeat (2) edge_step;
        gate_aw = 1; edge_step;
      end else begin gate_aw = 1; gate_w = 1; edge_step; end
      repeat (3) begin
        edge_step; check(!rsp_valid, "delayed B response delays core completion");
      end
      gate_b = 1; finish_core(expected_error);
    end
  endtask
  task automatic read_core(input word_t address, input logic expected_error);
    word_t snapshot;
    begin
      gate_ar = 0; gate_r = 0;
      core_request(0, address, 0, 0);
      repeat (3) begin
        check(arvalid && araddr == address && !rsp_valid,
              "read address is stable while AR is blocked"); edge_step;
      end
      gate_ar = 1; edge_step;
      check(!arvalid && t_rvalid && !rsp_valid, "AR acceptance creates a held slave snapshot");
      snapshot = t_rdata;
      repeat (3) begin
        edge_step;
        check(t_rvalid && t_rdata == snapshot && !rsp_valid,
              "timer read snapshot remains stable while its counter continues");
      end
      gate_r = 1; finish_core(expected_error);
      check(result == snapshot, "read response preserves the exact accepted slave snapshot");
    end
  endtask
  task automatic direct_write(input word_t address, value, input logic [3:0] strobes,
                              input logic [1:0] expected_response);
    begin
      direct = 1; d_awaddr = address; d_wdata = value; d_wstrb = strobes;
      d_awvalid = 1; d_wvalid = 0;
      #1; check(t_awready, "direct slave probe accepts address independently");
      edge_step; d_awvalid = 0;
      repeat (2) edge_step;
      check(!t_bvalid, "address alone cannot cause a write response");
      d_wvalid = 1; #1; check(t_wready, "late write data is independently accepted");
      edge_step; d_wvalid = 0;
      watchdog = 0;
      while (!t_bvalid && watchdog < 10) begin edge_step; watchdog = watchdog + 1; end
      check(t_bvalid && t_bresp == expected_response, "slave reports expected write status");
      repeat (3) begin edge_step; check(t_bvalid && t_bresp == expected_response,
                                      "B payload remains stable until BREADY"); end
      d_bready = 1; edge_step; d_bready = 0; direct = 0;
    end
  endtask
  task automatic direct_read(input word_t address, input logic [1:0] expected_response);
    word_t snapshot;
    begin
      direct = 1; d_araddr = address; d_arvalid = 1;
      #1; check(t_arready, "direct read address accepts while read slot is empty");
      edge_step; d_arvalid = 0; snapshot = t_rdata;
      check(t_rvalid && t_rresp == expected_response, "slave reports expected read status");
      repeat (3) begin edge_step; check(t_rvalid && t_rdata == snapshot
          && t_rresp == expected_response, "R status/data stay stable until RREADY"); end
      result = t_rdata;
      d_rready = 1; edge_step; d_rready = 0; direct = 0;
    end
  endtask

  initial begin #100000; $fatal(1, "AXI/timer verification timed out"); end
  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block14/block14.vcd"); $dumpvars(0, tb_block14);
    end
    reset_all;
    read_core(TIMER_BASE, 0); check(result == 0, "MSIP resets clear");
    read_core(TIMER_BASE + 32'h4000, 0); check(result == 32'hffffffff, "compare low resets max");
    read_core(TIMER_BASE + 32'h4004, 0); check(result == 32'hffffffff, "compare high resets max");
    check(!timer_irq && awprot == 1 && arprot == 1, "reset timer is masked by max compare");

    write_core(TIMER_BASE, 1, 1, 0, 0); check(software_irq, "MSIP sets software interrupt");
    write_core(TIMER_BASE, 0, 2, 1, 0); check(software_irq, "other byte lanes preserve MSIP");
    read_core(TIMER_BASE, 0); check(result == 1, "MSIP exposes only bit zero");
    write_core(TIMER_BASE, 0, 1, 2, 0); check(!software_irq, "MSIP clears on enabled byte zero");
    write_core(TIMER_BASE + 32'h4000, 32'h11223344, 15, 0, 0);
    write_core(TIMER_BASE + 32'h4004, 32'haabbccdd, 15, 1, 0);
    read_core(TIMER_BASE + 32'h4000, 0); check(result == 32'h11223344, "compare low readback");
    read_core(TIMER_BASE + 32'h4004, 0); check(result == 32'haabbccdd, "compare high readback");
    write_core(TIMER_BASE + 32'h4000, 32'h99ffbbff, 5, 2, 0);
    read_core(TIMER_BASE + 32'h4000, 0);
    check(result == 32'h11ff33ff, "byte strobes modify only selected compare bytes");
    write_core(TIMER_BASE + 32'h4004, 0, 0, 0, 0);
    read_core(TIMER_BASE + 32'h4004, 0); check(result == 32'haabbccdd, "zero strobes are no-op");

    write_core(TIMER_BASE + 32'h1234, 32'hdeadbeef, 15, 1, 1);
    read_core(TIMER_BASE + 32'h1234, 1); check(result == 0, "unmapped read returns zero/error");
    direct_write(TIMER_BASE + 32'h4002, 0, 15, 2);
    direct_read(TIMER_BASE + 32'hbff9, 2); check(result == 0, "unaligned read returns zero");
    read_core(TIMER_BASE + 32'h4000, 0);
    check(result == 32'h11ff33ff, "invalid writes cannot alter a mapped register");

    for (n = 1; n < 4; n++) begin
      inject_b = 1; injected_bresp = n; write_core(TIMER_BASE, 0, 1, n % 3, 1); inject_b = 0;
      inject_r = 1; injected_rresp = n; read_core(TIMER_BASE, 1); inject_r = 0;
    end

    write_core(TIMER_BASE + 32'hbffc, 32'h12345678, 15, 0, 0);
    write_core(TIMER_BASE + 32'hbff8, 32'hffff0000, 15, 1, 0);
    read_core(TIMER_BASE + 32'hbffc, 0); check(result == 32'h12345678, "mtime high half writes");
    read_core(TIMER_BASE + 32'hbff8, 0); earlier = result;
    repeat (40) edge_step;
    read_core(TIMER_BASE + 32'hbff8, 0); later = result;
    check(later > earlier && later - earlier >= 40, "mtime advances independently of bus stalls");
    write_core(TIMER_BASE + 32'hbffc, 32'hab000000, 8, 2, 0);
    read_core(TIMER_BASE + 32'hbffc, 0); check(result == 32'hab345678, "mtime high byte strobe");
    write_core(TIMER_BASE + 32'hbff8, 32'haa000000, 8, 0, 0);
    read_core(TIMER_BASE + 32'hbff8, 0);
    check((result & 32'hff000000) == 32'haa000000, "mtime low byte strobes preserve other bytes");

    write_core(TIMER_BASE + 32'hbffc, 32'h13572468, 15, 1, 0);
    write_core(TIMER_BASE + 32'hbff8, 32'hfffffff0, 15, 0, 0);
    repeat (40) edge_step;
    read_core(TIMER_BASE + 32'hbffc, 0);
    check(result == 32'h13572469, "mtime low-word overflow carries into the high half");

    write_core(TIMER_BASE + 32'h4004, 0, 15, 1, 0);
    write_core(TIMER_BASE + 32'h4000, 0, 15, 0, 0);
    check(timer_irq, "mtime >= mtimecmp asserts timer interrupt");
    write_core(TIMER_BASE + 32'h4000, 32'hffffffff, 15, 1, 0);
    write_core(TIMER_BASE + 32'h4004, 32'hffffffff, 15, 0, 0);
    check(!timer_irq, "future compare value clears pending timer interrupt");
    write_core(TIMER_BASE + 32'hbffc, 0, 15, 1, 0);
    write_core(TIMER_BASE + 32'hbff8, 0, 15, 0, 0);
    write_core(TIMER_BASE + 32'h4004, 0, 15, 1, 0);
    write_core(TIMER_BASE + 32'h4000, 200, 15, 0, 0);
    check(!timer_irq, "future near compare starts with interrupt low");
    watchdog = 0;
    while (!timer_irq && watchdog < 250) begin edge_step; watchdog = watchdog + 1; end
    check(timer_irq && watchdog < 250, "timer reaches the programmed threshold");
    read_core(TIMER_BASE + 32'hbff8, 0); check(result >= 200, "interrupt threshold matches mtime");

    // Illegal core word address is rejected locally without issuing AXI.
    request = '0; request.address = TIMER_BASE + 2; req_valid = 1; #1;
    check(req_ready && master_error, "unaligned core request is diagnosed before AXI");
    edge_step; req_valid = 0;
    check(rsp_valid && response.error && !arvalid && !awvalid && !wvalid,
          "unaligned request creates one local error response");
    rsp_ready = 1; edge_step; rsp_ready = 0;

    // Reset aborts a globally reset partial bus transaction and both retained slave channels.
    gate_aw = 1; gate_w = 0; core_request(1, TIMER_BASE, 1, 1); edge_step;
    reset_all;
    direct = 1; d_awvalid = 1; d_wvalid = 1; d_awaddr = TIMER_BASE;
    d_wdata = 1; d_wstrb = 1; edge_step; d_awvalid = 0; d_wvalid = 0; edge_step;
    d_arvalid = 1; d_araddr = TIMER_BASE; edge_step; d_arvalid = 0;
    check(t_bvalid && t_rvalid, "slave can independently retain B and R responses");
    reset_all;
    $display("BLOCK14 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "AXI/timer checks failed");
    $finish;
  end
endmodule
