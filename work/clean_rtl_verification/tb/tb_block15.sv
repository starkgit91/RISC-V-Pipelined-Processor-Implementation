// Full core verification. Instructions enter through memory, not decoded injection.
`timescale 1ns / 1ps
module tb_block15;
  import rv32_ooo_pkg::*;
  logic clk = 0, rst_n = 1, predict_enable = 1, halt = 0;
  logic i_valid, i_ready, i_rsp_valid, i_rsp_ready, i_error;
  word_t i_address;
  fetch_row_t i_data;
  logic d_valid, d_ready, d_rsp_valid, d_rsp_ready;
  data_request_t d_request;
  memory_response_t d_response;
  logic irq_external = 0, irq_software = 0, irq_timer = 0;
  retire_trace_pair_t trace;
  logic empty, protocol_error;
  word_t memory [0:RAM_WORDS-1];
  word_t reference_memory [0:RAM_WORDS-1];
  word_t reference_csrs [0:4095];
  retire_trace_t baseline [0:63];
  logic i_pending = 0, d_pending = 0;
  integer i_delay = 0, d_delay = 0, instruction_latency = 0, data_latency = 2;
  fetch_row_t i_saved;
  logic i_saved_error;
  memory_response_t d_saved;
  logic allow_i = 1, allow_d = 1;
  integer checks = 0, failures = 0, cycles = 0, retired = 0;
  integer requests = 0, stores = 0, mmio_reads = 0, mmio_writes = 0;
  integer recovery_count = 0, trap_count = 0, interrupt_count = 0;
  integer current_case = 0, watchdog, n;
  integer expected_requests = 0, expected_stores = 0;
  integer expected_mmio_reads = 0, expected_mmio_writes = 0;
  integer cursor, selector, round_index, rd_index, rs1_index, rs2_index, compare_mode = 0;
  integer mode, funct, operand_index;
  integer case_count = 0;
  integer max_rob = 0, max_iq = 0, max_memory = 0;
  logic random_stalls = 0;
  logic younger_redirect_seen = 0, older_redirect_seen = 0;
  word_t registers [0:31];
  word_t expected_pc = 0, finish_pc = 32'hffffffff;
  logic finished = 0;
  word_t rng = 32'hc001face;

  rv32_ooo_core dut (
    .clk_i(clk), .rst_ni(rst_n), .predict_enable_i(predict_enable), .halt_i(halt),
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
  assign i_rsp_valid = i_pending && i_delay == 0;
  assign i_ready = rst_n && allow_i && (!i_pending || (i_rsp_valid && i_rsp_ready));
  assign i_data = i_saved;
  assign i_error = i_saved_error;
  assign d_rsp_valid = d_pending && d_delay == 0;
  assign d_ready = rst_n && allow_d && !d_pending;
  assign d_response = d_saved;

  function automatic word_t op_i(input integer imm, rs1, funct3, rd, opcode);
    op_i = ((imm & 4095) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode;
  endfunction
  function automatic word_t op_r(input integer funct7, rs2, rs1, funct3, rd);
    op_r = (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | 7'h33;
  endfunction
  function automatic word_t op_b(input integer imm, rs2, rs1, funct3);
    op_b = (((imm >> 12) & 1) << 31) | (((imm >> 5) & 63) << 25) | (rs2 << 20)
      | (rs1 << 15) | (funct3 << 12) | (((imm >> 1) & 15) << 8)
      | (((imm >> 11) & 1) << 7) | 7'h63;
  endfunction
  function automatic word_t op_s(input integer imm, rs2, rs1, funct3);
    op_s = (((imm >> 5) & 127) << 25) | (rs2 << 20) | (rs1 << 15)
      | (funct3 << 12) | ((imm & 31) << 7) | 7'h23;
  endfunction
  function automatic word_t op_j(input integer imm, rd);
    op_j = (((imm >> 20) & 1) << 31) | (((imm >> 1) & 1023) << 21)
      | (((imm >> 11) & 1) << 20) | (((imm >> 12) & 255) << 12) | (rd << 7) | 7'h6f;
  endfunction
  task automatic check(input logic condition, input string text_label);
    begin
      checks++;
      if (condition !== 1'b1) begin
        failures++;
        if (failures < 40) $display("FAIL case%0d cycle%0d: %s", current_case, cycles, text_label);
      end
    end
  endtask

  // Initial minimal architectural interpreter. Later cases extend CSR/trap checking.
  task automatic observe(input retire_trace_t event_record);
    word_t instruction, a, b, immediate, result, next_pc, address, raw, store_data;
    word_t csr_old, csr_operand, csr_value;
    logic [3:0] store_strobes;
    integer rd, rs1, rs2, funct3, opcode, address_index, shift, csr_address;
    logic writes, taken, csr_write;
    logic signed [63:0] sa, sb, product;
    logic [63:0] unsigned_product;
    begin
      if (event_record.valid) begin
        check(!finished, "no event follows the program terminator");
        check(event_record.order == retired, "trace order is consecutive");
        check(event_record.pc == expected_pc, "retirement PC follows architectural control flow");
        instruction = reference_memory[expected_pc >> 2];
        check(event_record.instruction == instruction, "retired instruction matches program");
        if (compare_mode == 1 && retired < 64) baseline[retired] = event_record;
        if (compare_mode == 2 && retired < 64)
          check(event_record == baseline[retired],
                "prediction preserves the full retirement trace");
        opcode = instruction[6:0]; funct3 = instruction[14:12];
        rd = instruction[11:7]; rs1 = instruction[19:15]; rs2 = instruction[24:20];
        a = registers[rs1]; b = registers[rs2];
        immediate = {{20{instruction[31]}},instruction[31:20]};
        result = 0; next_pc = expected_pc + 4; writes = 0; taken = 0;
        address = 0; raw = 0; store_data = 0; store_strobes = 0;
        csr_old = 0; csr_operand = 0; csr_value = 0; csr_write = 0;
        address_index = 0; shift = 0; csr_address = instruction[31:20];
        check(!event_record.trap && !event_record.interrupt,
              "arithmetic case has no trap/interrupt");
        case (opcode)
          7'h13: begin
            writes = 1;
            case (funct3)
              0: result = a + immediate;
              2: result = $signed(a) < $signed(immediate);
              3: result = a < immediate;
              4: result = a ^ immediate;
              6: result = a | immediate;
              7: result = a & immediate;
              1: result = a << instruction[24:20];
              5: begin
                if (instruction[30]) result = $signed(a) >>> instruction[24:20];
                else result = a >> instruction[24:20];
              end
            endcase
          end
          7'h33: begin
            writes = 1; sa = {{32{a[31]}},a}; sb = {{32{b[31]}},b};
            product = sa * sb; unsigned_product = {32'b0,a} * {32'b0,b};
            if (instruction[31:25] == 1) begin
              case (funct3)
                0: result = unsigned_product[31:0];
                1: result = product[63:32];
                2: begin sb = {32'b0,b}; product = sa * sb; result = product[63:32]; end
                3: result = unsigned_product[63:32];
                4: begin
                  if (b == 0) result = 32'hffffffff;
                  else begin sa = sa / sb; result = sa[31:0]; end
                end
                5: result = b == 0 ? 32'hffffffff : a / b;
                6: begin
                  if (b == 0) result = a;
                  else begin sa = sa % sb; result = sa[31:0]; end
                end
                7: result = b == 0 ? a : a % b;
              endcase
            end else case (funct3)
              0: result = instruction[30] ? a - b : a + b;
              1: result = a << b[4:0];
              2: result = $signed(a) < $signed(b);
              3: result = a < b;
              4: result = a ^ b;
              5: begin
                if (instruction[30]) result = $signed(a) >>> b[4:0];
                else result = a >> b[4:0];
              end
              6: result = a | b;
              7: result = a & b;
            endcase
          end
          7'h37: begin writes = 1; result = {instruction[31:12],12'b0}; end
          7'h17: begin writes = 1; result = expected_pc + {instruction[31:12],12'b0}; end
          7'h63: begin
            immediate = {{19{instruction[31]}},instruction[31],instruction[7],
                         instruction[30:25],instruction[11:8],1'b0};
            case (funct3)
              0: taken = a == b;
              1: taken = a != b;
              4: taken = $signed(a) < $signed(b);
              5: taken = $signed(a) >= $signed(b);
              6: taken = a < b;
              7: taken = a >= b;
            endcase
            if (taken) next_pc = expected_pc + immediate;
          end
          7'h6f: begin
            writes = 1; result = next_pc;
            immediate = {{11{instruction[31]}},instruction[31],instruction[19:12],
                         instruction[20],instruction[30:21],1'b0};
            next_pc = expected_pc + immediate;
          end
          7'h67: begin writes = 1; result = next_pc; next_pc = (a + immediate) & 32'hfffffffe; end
          7'h03: begin
            address = a + immediate; shift = (address % 4) * 8;
            check(address < RAM_BYTES || address == 32'h10000000,
                  "nonfaulting load uses a mapped address");
            raw = address < RAM_BYTES ? reference_memory[address >> 2] : 32'h12345678;
            raw = raw >> shift; writes = 1;
            case (funct3)
              0: result = {{24{raw[7]}},raw[7:0]};
              1: result = {{16{raw[15]}},raw[15:0]};
              2: result = raw;
              4: result = {24'b0,raw[7:0]};
              5: result = {16'b0,raw[15:0]};
              default: check(0, "legal load size");
            endcase
            check(!event_record.memory_write && event_record.memory_address == address,
                  "load completion reports its effective byte address");
            expected_requests++;
            if (address == 32'h10000000) expected_mmio_reads++;
          end
          7'h23: begin
            immediate = {{20{instruction[31]}},instruction[31:25],instruction[11:7]};
            address = a + immediate; shift = (address % 4) * 8;
            store_data = b << shift;
            case (funct3)
              0: store_strobes = 4'b0001 << (address % 4);
              1: store_strobes = 4'b0011 << (address % 4);
              2: store_strobes = 4'b1111;
              default: check(0, "legal store size");
            endcase
            check(event_record.memory_write && event_record.memory_address == address
                  && event_record.memory_wdata == store_data
                  && event_record.memory_wstrb == store_strobes,
                  "store address, lanes, and data match independent architectural values");
            expected_requests++;
            if (address < RAM_BYTES) begin
              expected_stores++;
              for (integer byte_lane = 0; byte_lane < 4; byte_lane++)
                if (store_strobes[byte_lane]) reference_memory[address >> 2][byte_lane*8 +: 8]
                    = store_data[byte_lane*8 +: 8];
            end else begin
              check(address == 32'h10000000, "nonfaulting store uses mapped address");
              expected_mmio_writes++;
            end
          end
          7'h0f: check(funct3 == 0 || funct3 == 1, "FENCE/FENCE.I encoding is supported");
          7'h73: begin
            if (funct3 == 0) check(instruction == 32'h10500073, "case terminates with WFI");
            else begin
              csr_old = reference_csrs[csr_address];
              if (csr_address == CSR_MISA) csr_old = MISA_VALUE;
              if (csr_address >= CSR_MVENDORID) csr_old = 0;
              csr_operand = funct3[2] ? rs1 : a;
              csr_value = csr_old; csr_write = 1; writes = 1; result = csr_old;
              case (funct3)
                1,5: csr_value = csr_operand;
                2,6: begin csr_value = csr_old | csr_operand; csr_write = rs1 != 0; end
                3,7: begin csr_value = csr_old & ~csr_operand; csr_write = rs1 != 0; end
                default: check(0, "supported CSR operation");
              endcase
              if (csr_write) begin
                case (csr_address)
                  CSR_MSTATUS: reference_csrs[csr_address] = 32'h1800 | (csr_value & 32'h88);
                  CSR_MIE: reference_csrs[csr_address] = csr_value & 32'h888;
                  CSR_MTVEC: reference_csrs[csr_address] = (csr_value & 32'hfffffffc)
                    | ((csr_value & 3) == 1 ? 1 : 0);
                  CSR_MEPC: reference_csrs[csr_address] = csr_value & 32'hfffffffc;
                  CSR_MCOUNTINHIBIT: reference_csrs[csr_address] = csr_value & 5;
                  CSR_MISA,CSR_MIP,CSR_MSTATUSH: begin end
                  default: reference_csrs[csr_address] = csr_value;
                endcase
              end
            end
          end
          default: check(0, "interpreter supports every instruction in this case");
        endcase
        if (writes && rd != 0) begin
          check(event_record.rd == rd && event_record.rd_value == result,
                "destination/value match independent instruction semantics");
          registers[rd] = result;
        end else check(event_record.rd == 0, "nonwriter cannot publish a destination");
        check(event_record.next_pc == next_pc, "resolved next PC matches reference");
        if (opcode != 7'h23) check(!event_record.memory_write, "only stores trace memory writes");
        expected_pc = next_pc;
        retired++;
        if (event_record.pc == finish_pc) begin finished = 1; halt = 1; end
      end
    end
  endtask

  task automatic step;
    logic ih, ir, dh, dr;
    data_request_t request;
    logic [3:0] store_strobes;
    word_t store_data;
    word_t accepted_instruction_address;
    begin
      #1;
      ih = i_valid && i_ready; ir = i_rsp_valid && i_rsp_ready;
      accepted_instruction_address = i_address;
      dh = d_valid && d_ready; dr = d_rsp_valid && d_rsp_ready; request = d_request;
      store_strobes = request.wstrb;
      store_data = request.wdata;
      check(!protocol_error, "all composed protocol diagnostics remain clear");
      if (dut.rob_occupancy > max_rob) max_rob = dut.rob_occupancy;
      if (dut.iq_occupancy > max_iq) max_iq = dut.iq_occupancy;
      if (dut.lsu_occupancy > max_memory) max_memory = dut.lsu_occupancy;
      if (dh) check(dut.head_valid[0] && !dut.head_complete[0]
                    && (dut.heads.lane0.uop.decoded.op_class == UOP_LOAD
                        || dut.heads.lane0.uop.decoded.op_class == UOP_STORE),
                    "every data request belongs to the oldest incomplete instruction");
      if (dut.apply) recovery_count++;
      if (dut.mispredict && dut.mispredict_pc == 24) younger_redirect_seen = 1;
      if (dut.mispredict && dut.mispredict_pc == 32) older_redirect_seen = 1;
      #1; clk = 1; #1;
      if (ir) i_pending = 0; else if (i_pending && i_delay > 0) i_delay--;
      if (ih) begin
        i_pending = 1; i_delay = instruction_latency;
        i_saved_error = accepted_instruction_address >= RAM_BYTES;
        i_saved = i_saved_error ? 64'b0
          : {memory[(accepted_instruction_address >> 2)+1],
             memory[accepted_instruction_address >> 2]};
      end
      if (dr) d_pending = 0; else if (d_pending && d_delay > 0) d_delay--;
      if (dh) begin
        requests++; d_pending = 1; d_delay = data_latency; d_saved = '0;
        if (request.address < RAM_BYTES) begin
          d_saved.rdata = memory[request.address >> 2];
          if (request.write) begin
            stores++;
            for (integer lane = 0; lane < 4; lane++)
              if (store_strobes[lane]) memory[request.address >> 2][lane*8 +: 8]
                  = store_data[lane*8 +: 8];
          end
        end else if (request.address == 32'h10000000) begin
          if (request.write) mmio_writes++; else mmio_reads++;
          d_saved.rdata = 32'h12345678;
        end else begin d_saved.error = 1; d_saved.rdata = 0; end
      end
      observe(trace.lane0); observe(trace.lane1);
      clk = 0; cycles++;
      if (random_stalls) begin
        rng = {rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
        allow_i = rng[0] || rng[1]; allow_d = rng[2] || rng[3];
        instruction_latency = rng[5:4]; data_latency = rng[7:6];
      end
    end
  endtask
  task automatic reset_case(input integer id);
    begin
      halt = 1; rst_n = 0; current_case = id; finished = 0; cycles = 0;
      retired = 0; expected_pc = 0; requests = 0; stores = 0; mmio_reads = 0; mmio_writes = 0;
      trap_count = 0; interrupt_count = 0; recovery_count = 0;
      expected_requests = 0; expected_stores = 0;
      expected_mmio_reads = 0; expected_mmio_writes = 0;
      max_rob = 0; max_iq = 0; max_memory = 0;
      younger_redirect_seen = 0; older_redirect_seen = 0;
      irq_external = 0; irq_software = 0; irq_timer = 0;
      i_pending = 0; d_pending = 0; i_delay = 0; d_delay = 0;
      allow_i = 1; allow_d = 1; instruction_latency = 0; data_latency = 2;
      for (integer i = 0; i < RAM_WORDS; i++) memory[i] = NOP_INSTRUCTION;
      for (integer i = 0; i < 32; i++) registers[i] = 0;
      for (integer i = 0; i < 4096; i++) reference_csrs[i] = 0;
      reference_csrs[CSR_MSTATUS] = 32'h1800;
      #2;
    end
  endtask
  task automatic run_case;
    begin
      for (integer i = 0; i < RAM_WORDS; i++) reference_memory[i] = memory[i];
      rst_n = 1; halt = 0; watchdog = 0;
      while (!finished && watchdog < 20000) begin step; watchdog++; end
      check(finished, "program reaches its terminator within timeout");
      check(requests == expected_requests && stores == expected_stores
            && mmio_reads == expected_mmio_reads && mmio_writes == expected_mmio_writes,
            "every data-side transaction matches a retired architectural access exactly once");
      check(failures == 0, "program has no architectural mismatch");
      case_count++;
      $display("BLOCK15 CASE %0d: %0d events, %0d cycles, %0d requests, %0d recoveries",
               current_case,retired,cycles,requests,recovery_count);
    end
  endtask
  task automatic emit(input word_t instruction);
    begin memory[cursor] = instruction; cursor++; end
  endtask
  task automatic end_program;
    begin finish_pc = cursor * 4; emit(32'h10500073); run_case; end
  endtask
  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block15/block15.vcd"); $dumpvars(0,tb_block15);
    end
    #3000000; $fatal(1,"Full core timed out");
  end
  initial begin
    for (mode = 0; mode < 2; mode++) begin
      reset_case(mode + 1); predict_enable = mode; compare_mode = mode == 0 ? 1 : 2;
      memory[0] = op_i(4,0,0,1,7'h13);
      memory[1] = op_i(0,0,0,2,7'h13);
      memory[2] = op_i(3,2,0,2,7'h13);
      memory[3] = op_i(-1,1,0,1,7'h13);
      memory[4] = op_b(-8,0,1,1);
      memory[5] = op_i(1,2,0,3,7'h13);
      memory[6] = 32'h10500073;
      finish_pc = 24; run_case;
      check(registers[1] == 0 && registers[2] == 12 && registers[3] == 13,
            "production recovery executes the loop and preserves architectural values");
    end
    compare_mode = 0; predict_enable = 1;

    // All RV32M operations, repeated versions of destinations, and immediate consumers.
    reset_case(3); cursor = 0;
    emit(32'h800000b7); // LUI x1,0x80000: minimum signed integer.
    emit(op_i(-1,0,0,2,7'h13));
    emit(op_i(0,0,0,3,7'h13));
    emit(op_i(33,0,0,4,7'h13));
    for (selector = 0; selector < 8; selector++) begin
      emit(op_r(1,2,1,selector,5));
      emit(op_r(0,5,4,0,6));
      emit(op_r(1,3,1,selector,5));
      emit(op_r(0,5,6,4,7));
      emit(op_r(1,4,2,selector,5));
      emit(op_r(0,5,7,0,7));
    end
    end_program;

    // Six branch comparisons with signed/unsigned boundary operands, taken and not-taken.
    reset_case(4); cursor = 0;
    emit(op_i(-1,0,0,1,7'h13)); emit(op_i(1,0,0,2,7'h13));
    emit(32'h800001b7); emit(32'h80000237); emit(op_i(-1,4,0,4,7'h13));
    for (selector = 0; selector < 6; selector++) begin
      case (selector)
        0: funct = 0; 1: funct = 1; 2: funct = 4;
        3: funct = 5; 4: funct = 6; default: funct = 7;
      endcase
      for (operand_index = 0; operand_index < 5; operand_index++) begin
        rs1_index = operand_index % 4 + 1;
        rs2_index = (operand_index + selector) % 4 + 1;
        emit(op_b(8,rs2_index,rs1_index,funct));
        emit(op_i(1,7,0,7,7'h13));
        emit(op_i(2,8,0,8,7'h13));
      end
    end
    end_program;

    // Direct/indirect link effects survive recovery, including rd == rs1 for JALR.
    reset_case(5);
    memory[0] = op_j(8,5); memory[1] = op_i(999,0,0,6,7'h13);
    memory[2] = op_i(1,5,0,6,7'h13); memory[3] = op_i(25,0,0,7,7'h13);
    memory[4] = op_i(0,7,0,7,7'h67); memory[5] = op_i(999,0,0,6,7'h13);
    memory[6] = op_i(1,7,0,9,7'h13); memory[7] = 32'h00001517;
    memory[8] = 32'h800005b7; memory[9] = 32'h10500073;
    finish_pc = 36; run_case;

    // Head-only memory transactions with byte strobes, sign extension, and RAW dependencies.
    reset_case(6); cursor = 0; random_stalls = 1;
    emit(32'h00006fb7); // x31=0x6000 data base, separate from the program.
    emit(32'h807f00b7); emit(op_i(-128,1,0,1,7'h13));
    emit(op_s(0,1,31,2));
    for (operand_index = 0; operand_index < 4; operand_index++) begin
      emit(op_i(operand_index,31,0,2,7'h03));
      emit(op_i(operand_index,31,4,3,7'h03));
      emit(op_r(0,3,2,0,4));
    end
    emit(op_i(0,31,1,5,7'h03)); emit(op_i(0,31,5,6,7'h03));
    emit(op_i(2,31,1,7,7'h03)); emit(op_i(2,31,5,8,7'h03));
    emit(op_s(5,1,31,0)); emit(op_s(6,1,31,1));
    emit(op_i(4,31,2,9,7'h03));
    emit(32'h0000000f); end_program;

    // A younger wrong-path MMIO load/store must not issue while an older DIV delays recovery.
    reset_case(7); cursor = 0;
    emit(op_i(100,0,0,1,7'h13)); emit(op_i(4,0,0,2,7'h13)); emit(32'h100001b7);
    emit(op_r(1,2,1,4,5)); emit(op_b(12,0,0,0));
    emit(op_s(0,1,3,2)); emit(op_i(0,3,2,6,7'h03));
    emit(op_i(1,5,0,7,7'h13)); end_program;
    check(mmio_reads == 0 && mmio_writes == 0, "wrong-path peripherals have no side effects");

    // Full-core CSR dependencies and all six forms, without relying on unit-level injection.
    reset_case(8); cursor = 0;
    emit(op_i(32'h55,0,0,1,7'h13)); emit(op_i(CSR_MSCRATCH,1,1,2,7'h73));
    emit(op_i(CSR_MSCRATCH,2,6,3,7'h73)); emit(op_i(CSR_MSCRATCH,1,7,4,7'h73));
    emit(op_i(CSR_MSCRATCH,5,5,5,7'h73)); emit(op_i(CSR_MSCRATCH,1,2,6,7'h73));
    emit(op_i(CSR_MSCRATCH,1,3,7,7'h73)); emit(op_i(CSR_MSCRATCH,0,2,8,7'h73));
    emit(op_i(CSR_MISA,0,2,9,7'h73)); emit(op_i(CSR_MVENDORID,0,2,10,7'h73));
    emit(32'h0000100f); end_program;

    // Random register versions/RAW/WAR/WAW and mixed integer/M/memory instructions.
    for (round_index = 0; round_index < 4; round_index++) begin
      reset_case(9 + round_index); cursor = 0; predict_enable = round_index[0];
      emit(32'h00006fb7);
      for (rd_index = 1; rd_index < 16; rd_index++) begin
        rng = {rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
        emit((rng & 32'hfffff000) | (rd_index << 7) | 7'h37);
        emit(op_i(rng[11:0],rd_index,0,rd_index,7'h13));
      end
      for (n = 0; n < 240; n++) begin
        rng = {rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
        rd_index = rng[4:0] == 31 ? 0 : rng[4:0];
        rs1_index = rng[9:5]; rs2_index = rng[14:10]; selector = rng[17:15];
        case (rng[21:18] % 6)
          0: emit(op_r(0,rs2_index,rs1_index,selector,rd_index));
          1: emit(op_r(1,rs2_index,rs1_index,selector,rd_index));
          2: begin
            if (selector == 1 || selector == 5)
              emit(op_i(rng[4:0],rs1_index,selector,rd_index,7'h13));
            else emit(op_i(rng[11:0],rs1_index,selector,rd_index,7'h13));
          end
          3: emit(op_s((rng[5:2] * 4),rs2_index,31,2));
          4: emit(op_i((rng[5:2] * 4),31,2,rd_index,7'h03));
          5: emit(op_r(selector[0] ? 32 : 0,rs2_index,rs1_index,0,rd_index));
        endcase
      end
      end_program;
    end
    random_stalls = 0;
    reset_case(13); cursor = 0;
    emit(32'h800000b7); emit(op_i(-1,0,0,2,7'h13));
    emit(op_i(33,0,0,3,7'h13));
    for (selector = 0; selector < 8; selector++) begin
      emit(op_r(0,3,1,selector,4));
      if (selector == 0 || selector == 5) emit(op_r(32,3,1,selector,5));
      if (selector == 1 || selector == 5) begin
        emit(op_i(31,1,selector,6,7'h13));
        if (selector == 5) emit(op_i(32'h41f,1,selector,7,7'h13));
      end else emit(op_i(-2048,1,selector,6,7'h13));
      emit(op_r(0,6,4,0,4));
    end
    emit(op_i(2047,0,0,8,7'h13)); emit(op_i(1,0,0,0,7'h13));
    emit(op_r(0,0,0,0,9)); end_program;
    check(registers[0] == 0 && registers[9] == 0, "x0 remains constant through real execution");

    // Fill ROB/IQ behind an older DIV, then fill the memory queue without issuing early.
    reset_case(14); cursor = 0;
    emit(op_i(100,0,0,1,7'h13)); emit(op_i(3,0,0,2,7'h13));
    emit(op_r(1,2,1,4,5));
    for (selector = 0; selector < 7; selector++) emit(op_i(selector,5,0,6+selector,7'h13));
    for (selector = 0; selector < 12; selector++) emit(op_i(selector,20,0,20,7'h13));
    end_program;
    check(max_rob == ROB_DEPTH && max_iq == ISSUE_DEPTH,
          "full-core ROB and issue-queue exhaustion recover without lost instructions");
    reset_case(15); cursor = 0;
    emit(32'h00006fb7); emit(op_i(100,0,0,1,7'h13)); emit(op_i(3,0,0,2,7'h13));
    emit(op_r(1,2,1,4,5));
    for (selector = 0; selector < 8; selector++) emit(op_i(selector*4,31,2,6+selector,7'h03));
    end_program;
    check(max_memory == MEMORY_DEPTH, "full memory queue stalls admission and drains in order");
    reset_case(16);
    memory[0] = op_i(100,0,0,1,7'h13); memory[1] = op_i(4,0,0,2,7'h13);
    memory[2] = op_r(1,2,1,4,5);
    memory[3] = op_b(20,0,5,1); // Older branch waits on DIV and targets PC32.
    memory[4] = op_b(8,0,0,0); // Younger branch resolves first and targets PC24.
    memory[5] = op_i(999,0,0,6,7'h13); memory[6] = op_i(999,0,0,7,7'h13);
    memory[7] = op_j(12,0); memory[8] = op_i(7,0,0,6,7'h13);
    memory[9] = op_i(1,5,0,7,7'h13); memory[10] = 32'h10500073;
    finish_pc = 40; run_case;
    check(younger_redirect_seen && older_redirect_seen && registers[6] == 7 && registers[7] == 26,
          "later-resolving older misprediction replaces the younger recovery boundary");
    $display("BLOCK15 PROGRAMS: %0d",case_count);
    $display("BLOCK15 SUMMARY: %0d checks, %0d failures",checks,failures);
    if (failures) $fatal(1,"Full core failed");
    $finish;
  end
endmodule
