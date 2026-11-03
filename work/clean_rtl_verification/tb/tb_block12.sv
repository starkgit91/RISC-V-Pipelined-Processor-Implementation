// Ordered LSU tests: only the prepared ROB head may send a data transaction.
`timescale 1ns / 1ps
module tb_block12;
  import rv32_ooo_pkg::*;
  logic clk = 0, rst_n = 1;
  logic [1:0] dispatch_valid, dispatch_ready, dispatch_fire;
  renamed_pair_t dispatch_uop;
  logic prepare_valid, prepare_ready, head_valid, head_complete;
  memory_entry_t preparation, prepared;
  rob_tag_t head_tag;
  logic request_valid, request_ready, response_valid, response_ready;
  data_request_t request;
  memory_response_t response;
  logic completion_valid, completion_ready, flush, flush_done, quiet, protocol_error;
  completion_t completion, saved_completion;
  logic [2:0] occupancy;
  renamed_uop_t a, b, c, d, e, owner;
  integer checks = 0, failures = 0, serial_number = 0, accepted_requests = 0;
  integer size_index, lane, unsigned_load, n, limit, before_requests;

  rv32_load_store_unit dut (
    .clk_i(clk), .rst_ni(rst_n), .dispatch_valid_i(dispatch_valid),
    .dispatch_uop_i(dispatch_uop), .dispatch_ready_o(dispatch_ready),
    .dispatch_fire_o(dispatch_fire), .prepare_valid_i(prepare_valid),
    .prepare_ready_o(prepare_ready), .prepare_i(preparation),
    .head_valid_i(head_valid), .head_complete_i(head_complete), .head_tag_i(head_tag),
    .data_request_valid_o(request_valid), .data_request_ready_i(request_ready),
    .data_request_o(request), .data_response_valid_i(response_valid),
    .data_response_ready_o(response_ready), .data_response_i(response),
    .completion_valid_o(completion_valid), .completion_ready_i(completion_ready),
    .completion_o(completion), .flush_i(flush), .flush_done_o(flush_done),
    .quiescent_o(quiet), .occupancy_o(occupancy), .protocol_error_o(protocol_error)
  );
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
    begin
      #1;
      if (request_valid && request_ready) accepted_requests = accepted_requests + 1;
      #1; clk = 1; #1; clk = 0; #1;
    end
  endtask
  task automatic controls;
    begin
      dispatch_valid = 0; dispatch_uop = '0; prepare_valid = 0; preparation = '0;
      head_valid = 0; head_complete = 0; head_tag = '0;
      request_ready = 0; response_valid = 0; response = '0; completion_ready = 0; flush = 0;
    end
  endtask
  function automatic mem_size_e access_size(input integer selector);
    case (selector)
      0: access_size = MEM_BYTE;
      1: access_size = MEM_HALF;
      default: access_size = MEM_WORD;
    endcase
  endfunction
  function automatic renamed_uop_t instruction(
    input integer id, size_selector, input logic store_access, unsigned_access
  );
    renamed_uop_t result;
    begin
      result = '0;
      result.rob_tag.index = id % 16; result.rob_tag.generation = 20 + id / 16;
      result.decoded.fetch.pc = 32'h200 + id * 4;
      result.decoded.fetch.prediction.next_pc = result.decoded.fetch.pc + 4;
      if (store_access) result.decoded.op_class = UOP_STORE;
      else result.decoded.op_class = UOP_LOAD;
      result.decoded.mem_size = access_size(size_selector);
      result.decoded.load_unsigned = unsigned_access;
      result.decoded.writes_rd = !store_access;
      result.decoded.rd = store_access ? 0 : 5;
      result.pdst = store_access ? 0 : 32 + id % 32;
      instruction = result;
    end
  endfunction
  function automatic memory_entry_t address_record(
    input renamed_uop_t operation, input word_t address, value
  );
    memory_entry_t result;
    begin
      result = '0; result.valid = 1; result.prepared = 1;
      result.rob_tag = operation.rob_tag;
      result.is_store = operation.decoded.op_class == UOP_STORE;
      result.size = operation.decoded.mem_size;
      result.load_unsigned = operation.decoded.load_unsigned;
      result.pdst = operation.pdst; result.writes_rd = operation.decoded.writes_rd;
      result.address = address; result.store_value = value;
      result.control_flow.next_pc = operation.decoded.fetch.pc + 4;
      result.control_flow.mispredicted = operation.rob_tag.index[0];
      address_record = result;
    end
  endfunction
  function automatic word_t loaded_value(
    input word_t raw, input integer byte_offset, size_selector, input logic is_unsigned
  );
    word_t selected;
    begin
      selected = raw >> (byte_offset * 8);
      if (size_selector == 0) begin
        selected = selected & 255;
        if (!is_unsigned && selected >= 128) selected = selected | 32'hffffff00;
      end else if (size_selector == 1) begin
        selected = selected & 65535;
        if (!is_unsigned && selected >= 32768) selected = selected | 32'hffff0000;
      end else selected = raw;
      loaded_value = selected;
    end
  endfunction
  task automatic offer(input renamed_uop_t older, younger, input logic [1:0] mask);
    begin dispatch_uop.lane0 = older; dispatch_uop.lane1 = younger; dispatch_valid = mask; end
  endtask
  task automatic allocate_one(input renamed_uop_t operation);
    begin
      offer(operation, '0, 1); #1; check(dispatch_fire == 1, "memory owner allocates atomically");
      edge_step; dispatch_valid = 0;
    end
  endtask
  task automatic prepare_one(input memory_entry_t value);
    begin
      preparation = value; prepare_valid = 1; #1;
      check(prepare_ready && !protocol_error, "registered owner accepts one matching preparation");
      edge_step; prepare_valid = 0;
    end
  endtask
  task automatic idle_flush;
    begin
      head_valid = 0; flush = 1; #1;
      check(quiet && dispatch_fire == 0, "safe flush blocks admission of new queue owners");
      edge_step; flush = 0;
      check(flush_done && occupancy == 0 && quiet, "safe idle flush discards only unissued work");
      edge_step;
    end
  endtask

  task automatic access_case(
    input logic is_store, input integer size_selector, byte_offset,
    input logic is_unsigned, bus_error, inherited_fault, input word_t raw
  );
    logic misaligned;
    word_t original_address, store_data, expected_result;
    logic [3:0] expected_strobes;
    begin
      serial_number = serial_number + 1;
      owner = instruction(serial_number, size_selector, is_store, is_unsigned);
      original_address = 32'h1000 + byte_offset;
      store_data = 32'h89abcdef;
      misaligned = (size_selector == 1 && byte_offset % 2 != 0)
                   || (size_selector == 2 && byte_offset != 0);
      allocate_one(owner);
      prepared = address_record(owner, original_address, store_data);
      if (inherited_fault) begin
        prepared.exception.valid = 1; prepared.exception.cause = CAUSE_ILLEGAL_INSTR;
        prepared.exception.tval = 32'hbad0bad0;
      end
      prepare_one(prepared);
      before_requests = accepted_requests;
      head_valid = 1; head_tag = owner.rob_tag; head_complete = 1;
      repeat (2) begin edge_step; check(!request_valid && !completion_valid,
                                     "completed ROB head cannot reissue a memory access"); end
      head_complete = 0; head_tag.generation = head_tag.generation + 1'b1;
      repeat (2) begin edge_step; check(!request_valid && !completion_valid,
                                     "wrong-generation ROB head cannot authorize memory"); end
      head_tag = owner.rob_tag;
      limit = 0;
      while (!request_valid && !completion_valid && limit < 15) begin edge_step; limit++; end
      check(limit < 15, "prepared actual ROB head makes bounded progress");
      if (!misaligned && !inherited_fault) begin
        expected_strobes = size_selector == 0 ? (1 << byte_offset)
                           : size_selector == 1 ? (3 << byte_offset) : 15;
        check(request_valid && request.write == is_store && request.address == 32'h1000,
              "data bus sees the correct direction and aligned word address");
        if (is_store)
          check(request.wdata == (store_data << (byte_offset * 8))
                && request.wstrb == expected_strobes, "store byte lanes/data match byte address");
        else check(request.wstrb == 0, "loads never enable write strobes");
        for (integer hold = 0; hold < 3; hold++) begin
          edge_step;
          check(request_valid && request.address == 32'h1000 && !completion_valid,
                "request remains stable under data-port backpressure");
        end
        flush = 1; #1;
        check(protocol_error && request_valid && !flush_done,
              "unsafe flush cannot withdraw an offered memory transaction");
        edge_step; flush = 0; request_ready = 1; edge_step; request_ready = 0;
        check(response_ready && !request_valid, "one request transitions to response ownership");
        repeat (2) edge_step;
        flush = 1; #1;
        check(protocol_error && response_ready, "unsafe flush does not cancel response ownership");
        edge_step; flush = 0;
        response.rdata = raw; response.error = bus_error; response_valid = 1;
        edge_step; response_valid = 0;
        check(accepted_requests == before_requests + 1, "access sends exactly one bus request");
      end else begin
        check(!request_valid && completion_valid && accepted_requests == before_requests,
              "local fault completes at the head without any bus side effect");
      end
      #1;
      check(completion_valid && completion.rob_tag == owner.rob_tag
            && completion.pdst == owner.pdst && completion.writes_rd == owner.decoded.writes_rd
            && completion.control_flow == prepared.control_flow,
            "completion preserves full owner, destination intent, and next-PC metadata");
      if (misaligned || inherited_fault || bus_error) begin
        check(completion.exception.valid && !completion.memory_access.valid,
              "faulting memory operation publishes no successful-access trace");
        if (inherited_fault)
          check(completion.exception == prepared.exception, "inherited exception takes precedence");
        else if (misaligned)
          check(completion.exception.cause == (is_store ? CAUSE_STORE_MISALIGNED
                                                       : CAUSE_LOAD_MISALIGNED)
                && completion.exception.tval == original_address,
                "misalignment trap describes original byte address");
        else check(completion.exception.cause == (is_store ? CAUSE_STORE_ACCESS : CAUSE_LOAD_ACCESS)
                   && completion.exception.tval == original_address,
                   "bus error becomes load/store access fault at original address");
      end else begin
        expected_result = is_store ? 0 : loaded_value(raw, byte_offset, size_selector, is_unsigned);
        check(!completion.exception.valid && completion.value == expected_result,
              "load extraction/sign extension or store result matches independent value oracle");
        check(completion.memory_access.valid && completion.memory_access.write == is_store
              && completion.memory_access.address == original_address,
              "successful access trace retains original address and direction");
        if (is_store)
          check(completion.memory_access.wdata == (store_data << (byte_offset * 8))
                && completion.memory_access.wstrb == expected_strobes,
                "store trace reports exactly the bus lanes/data that completed");
        else check(completion.memory_access.wstrb == 0 && completion.memory_access.wdata == 0,
                   "load trace has no write-lane effects");
      end
      saved_completion = completion;
      flush = 1;
      for (integer hold = 0; hold < 3; hold++) begin
        #1; check(protocol_error && completion_valid && completion == saved_completion,
                  "held completion survives backpressure and rejected active flush");
        edge_step; check(!flush_done, "active flush never acknowledges safe cancellation");
      end
      flush = 0; completion_ready = 1; edge_step; completion_ready = 0; head_valid = 0;
      check(occupancy == 0 && quiet && !completion_valid,
            "accepted completion removes queue owner exactly once");
    end
  endtask

  initial begin #300000; $fatal(1, "Ordered LSU test timed out"); end
  initial begin
    if ($test$plusargs("waves")) begin
      $dumpfile("build/block12/block12.vcd"); $dumpvars(0, tb_block12);
    end
    controls; rst_n = 0; #2;
    check(occupancy == 0 && !request_valid && !completion_valid, "reset clears queue and engine");
    rst_n = 1; #1;

    a = instruction(1, 2, 0, 0); b = instruction(2, 2, 1, 0);
    c = instruction(3, 2, 0, 0); d = instruction(4, 0, 1, 0);
    offer(a, b, 3); #1; check(dispatch_fire == 3, "two memory operations allocate together");
    edge_step; offer(c, d, 3); edge_step; dispatch_valid = 0;
    check(occupancy == 4, "four-entry memory-order queue fills");
    offer(instruction(5, 0, 0, 0), instruction(6, 0, 1, 0), 3); #1;
    check(dispatch_fire == 0, "full queue stalls memory dispatch"); dispatch_valid = 0;
    owner = a; owner.decoded.op_class = UOP_INTEGER;
    offer(owner, owner, 3); #1; check(dispatch_fire == 3, "nonmemory pair consumes no queue slots");
    edge_step; dispatch_valid = 0;
    owner.decoded.op_class = UOP_SYSTEM; offer(owner, a, 3); #1;
    check(dispatch_fire == 1, "system boundary blocks younger dispatch"); edge_step;
    dispatch_valid = 0;
    prepared = address_record(d, UART_BASE, 32'h12345678); prepare_one(prepared);
    head_valid = 1; head_tag = d.rob_tag;
    repeat (4) begin edge_step; check(!request_valid,
      "prepared younger MMIO store cannot bypass older memory queue owners"); end
    idle_flush;

    // Ownership includes the decoded extension mode, not just destination and size.
    owner = instruction(9, 0, 0, 0);
    allocate_one(owner);
    preparation = address_record(owner, 32'h1000, 0);
    preparation.load_unsigned = 1;
    prepare_valid = 1; #1;
    check(prepare_ready && protocol_error, "mismatched load-extension metadata is rejected");
    edge_step; prepare_valid = 0;
    head_valid = 1; head_tag = owner.rob_tag;
    repeat (2) edge_step;
    check(!request_valid && !completion_valid,
          "invalid preparation cannot make its instruction executable");
    idle_flush;

    // Preparation requires a live unprepared owner, matching destination intent and access kind.
    allocate_one(a); prepared = address_record(a, 32'h1000, 0);
    for (n = 0; n < 6; n++) begin
      preparation = prepared;
      case (n)
        0: preparation.rob_tag.generation = preparation.rob_tag.generation + 1'b1;
        1: preparation.pdst = preparation.pdst + 1'b1;
        2: preparation.writes_rd = 0;
        3: preparation.is_store = 1;
        4: preparation.valid = 0;
        5: preparation.prepared = 0;
      endcase
      prepare_valid = 1; #1; check(prepare_ready && protocol_error,
                                 "malformed/stale preparation is consumed and diagnosed");
      edge_step; prepare_valid = 0;
    end
    prepare_one(prepared); prepare_valid = 1; preparation = prepared; #1;
    check(protocol_error, "duplicate preparation cannot overwrite prepared operands");
    edge_step; prepare_valid = 0; idle_flush;

    // Full-queue capacity does not borrow a same-edge completion release.
    offer(a, b, 3); edge_step; offer(c, d, 3); edge_step; dispatch_valid = 0;
    prepare_one(address_record(a, 32'h1000, 0)); head_valid = 1; head_tag = a.rob_tag;
    limit = 0; while (!request_valid && limit < 10) begin edge_step; limit++; end
    request_ready = 1; edge_step; request_ready = 0;
    response_valid = 1; response.rdata = 0; edge_step; response_valid = 0;
    e = instruction(7, 2, 0, 0); offer(e, '0, 1); completion_ready = 1; #1;
    check(completion_valid && dispatch_fire == 0,
          "full queue cannot borrow completion-edge capacity");
    edge_step; completion_ready = 0; head_valid = 0; #1;
    check(occupancy == 3 && dispatch_fire == 1, "released queue slot is usable next cycle");
    edge_step; dispatch_valid = 0;
    check(occupancy == 4, "retained producer allocates once space opens");
    idle_flush;

    for (size_index = 0; size_index < 3; size_index++)
      for (unsigned_load = 0; unsigned_load < 2; unsigned_load++)
        for (lane = 0; lane < 4; lane++)
          access_case(0, size_index, lane, unsigned_load, 0, 0,
                      lane[0] ? 32'h7fff0080 : 32'h807fff80);
    for (size_index = 0; size_index < 3; size_index++)
      for (lane = 0; lane < 4; lane++)
        access_case(1, size_index, lane, 0, 0, 0, 0);
    for (size_index = 0; size_index < 3; size_index++) begin
      access_case(0, size_index, 0, 0, 1, 0, 32'hffffffff);
      access_case(1, size_index, 0, 0, 1, 0, 0);
    end
    access_case(0, 2, 0, 0, 0, 1, 0);
    access_case(1, 2, 0, 0, 0, 1, 0);

    dispatch_valid = 2; #1; check(protocol_error && dispatch_fire == 0,
                                 "invalid dispatch prefix cannot allocate only lane one");
    dispatch_valid = 0; response_valid = 1; #1;
    check(protocol_error && !response_ready, "unsolicited bus response has no live owner");
    response_valid = 0;
    controls; rst_n = 0; #2;
    check(occupancy == 0 && quiet && !completion_valid && !request_valid,
          "global reset clears all speculative memory ownership");
    $display("BLOCK12 SUMMARY: %0d checks, %0d failures", checks, failures);
    if (failures) $fatal(1, "Ordered LSU checks failed");
    $finish;
  end
endmodule
