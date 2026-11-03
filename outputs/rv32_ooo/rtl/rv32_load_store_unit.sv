// Ordered memory queue. Preparation may be speculative; bus requests require the ROB head.
`timescale 1ns / 1ps
module rv32_load_store_unit
  import rv32_ooo_pkg::*;
(
  input logic clk_i, rst_ni,
  input logic [1:0] dispatch_valid_i,
  input renamed_pair_t dispatch_uop_i,
  output logic [1:0] dispatch_ready_o, dispatch_fire_o,
  input logic prepare_valid_i,
  output logic prepare_ready_o,
  input memory_entry_t prepare_i,
  input logic head_valid_i, head_complete_i,
  input rob_tag_t head_tag_i,
  output logic data_request_valid_o,
  input logic data_request_ready_i,
  output data_request_t data_request_o,
  input logic data_response_valid_i,
  output logic data_response_ready_o,
  input memory_response_t data_response_i,
  output logic completion_valid_o,
  input logic completion_ready_i,
  output completion_t completion_o,
  input logic flush_i,
  output logic flush_done_o, quiescent_o,
  output logic [2:0] occupancy_o,
  output logic protocol_error_o
);
  typedef enum logic [1:0] {IDLE, REQUEST, RESPONSE, COMPLETE} state_e;
  state_e state_q, state_d;
  memory_entry_t queue_q [MEMORY_DEPTH], queue_d [MEMORY_DEPTH];
  logic [1:0] head_q, head_d, tail_q, tail_d;
  logic [2:0] count_q, count_d;
  data_request_t request_q, request_d;
  completion_t completion_q, completion_d;

  function automatic logic needs_memory(input renamed_uop_t uop);
    needs_memory = !uop.decoded.fetch.exception.valid
      && (uop.decoded.op_class == UOP_LOAD || uop.decoded.op_class == UOP_STORE);
  endfunction
  function automatic memory_entry_t allocate(input renamed_uop_t uop);
    memory_entry_t entry;
    begin
      entry = '0;
      entry.valid = 1'b1;
      entry.rob_tag = uop.rob_tag;
      entry.is_store = uop.decoded.op_class == UOP_STORE;
      entry.size = uop.decoded.mem_size;
      entry.load_unsigned = uop.decoded.load_unsigned;
      entry.pdst = uop.pdst;
      entry.writes_rd = uop.decoded.writes_rd;
      allocate = entry;
    end
  endfunction

  assign occupancy_o = count_q;
  // Unissued queue entries are cancellable; an accepted bus transaction is not.
  assign quiescent_o = state_q == IDLE;
  assign prepare_ready_o = rst_ni && !flush_i;
  assign data_request_valid_o = rst_ni && state_q == REQUEST;
  assign data_request_o = request_q;
  assign data_response_ready_o = rst_ni && state_q == RESPONSE;
  assign completion_valid_o = rst_ni && state_q == COMPLETE;
  assign completion_o = completion_valid_o ? completion_q : completion_t'('0);

  always_comb begin : queue_and_engine
    memory_entry_t entry, current;
    renamed_uop_t incoming0, incoming1;
    logic mem0, mem1, matched, bad_alignment;
    int pushed, popped, free_count;
    logic [1:0] allocation_index;
    logic [4:0] shift;
    word_t extracted;

    for (int i = 0; i < MEMORY_DEPTH; i++) queue_d[i] = queue_q[i];
    state_d = state_q;
    head_d = head_q;
    tail_d = tail_q;
    count_d = count_q;
    request_d = request_q;
    completion_d = completion_q;
    incoming0 = dispatch_uop_i.lane0;
    incoming1 = dispatch_uop_i.lane1;
    mem0 = needs_memory(incoming0);
    mem1 = needs_memory(incoming1);
    free_count = MEMORY_DEPTH - int'(count_q);
    pushed = 0;
    popped = 0;
    allocation_index = tail_q;
    matched = 1'b0;
    entry = '0;
    current = queue_q[head_q];
    shift = {current.address[1:0], 3'b000};
    extracted = data_response_i.rdata >> shift;
    bad_alignment = (current.size == MEM_HALF && current.address[0])
                    || (current.size == MEM_WORD && |current.address[1:0]);
    dispatch_ready_o = '0;
    dispatch_fire_o = '0;
    protocol_error_o = rst_ni && (dispatch_valid_i == 2'b10
      || (flush_i && state_q != IDLE)
      || (data_response_valid_i && state_q != RESPONSE));

    if (rst_ni && !flush_i) begin
      dispatch_ready_o[0] = !mem0 || free_count >= 1;
      dispatch_fire_o[0] = dispatch_valid_i[0] && dispatch_ready_o[0];
      dispatch_ready_o[1] = dispatch_fire_o[0] && !incoming0.decoded.fetch.exception.valid
        && incoming0.decoded.op_class != UOP_SYSTEM
        && free_count >= int'(mem0) + int'(mem1);
      dispatch_fire_o[1] = dispatch_valid_i[1] && dispatch_ready_o[1];
      if (dispatch_fire_o[0] && mem0) begin
        queue_d[allocation_index] = allocate(incoming0);
        allocation_index = allocation_index + 2'd1;
        pushed = pushed + 1;
      end
      if (dispatch_fire_o[1] && mem1) begin
        queue_d[allocation_index] = allocate(incoming1);
        allocation_index = allocation_index + 2'd1;
        pushed = pushed + 1;
      end
      if (prepare_valid_i) begin
        for (int i = 0; i < MEMORY_DEPTH; i++) begin
          entry = queue_q[i];
          if (entry.valid && !entry.prepared && entry.rob_tag == prepare_i.rob_tag
              && entry.pdst == prepare_i.pdst && entry.writes_rd == prepare_i.writes_rd
              && entry.is_store == prepare_i.is_store && entry.size == prepare_i.size
              && entry.load_unsigned == prepare_i.load_unsigned
              && prepare_i.valid && prepare_i.prepared) begin
            queue_d[i] = prepare_i;
            matched = 1'b1;
          end
        end
        if (!matched) protocol_error_o = 1'b1;
      end
    end

    case (state_q)
      IDLE: if (!flush_i && count_q != 0 && current.valid && current.prepared
                && head_valid_i && !head_complete_i && current.rob_tag == head_tag_i) begin
        completion_d = '0;
        completion_d.rob_tag = current.rob_tag;
        completion_d.pdst = current.pdst;
        completion_d.writes_rd = current.writes_rd;
        completion_d.control_flow = current.control_flow;
        completion_d.exception = current.exception;
        request_d = '0;
        request_d.write = current.is_store;
        request_d.address = {current.address[31:2], 2'b00};
        request_d.wdata = current.store_value << shift;
        case (current.size)
          MEM_BYTE: request_d.wstrb = 4'b0001 << current.address[1:0];
          MEM_HALF: request_d.wstrb = 4'b0011 << current.address[1:0];
          default: request_d.wstrb = 4'b1111;
        endcase
        if (!current.is_store) request_d.wstrb = '0;
        if (bad_alignment && !current.exception.valid) begin
          completion_d.exception.valid = 1'b1;
          completion_d.exception.cause = current.is_store
            ? CAUSE_STORE_MISALIGNED : CAUSE_LOAD_MISALIGNED;
          completion_d.exception.tval = current.address;
        end
        if (completion_d.exception.valid) state_d = COMPLETE;
        else state_d = REQUEST;
      end
      REQUEST: if (data_request_valid_o && data_request_ready_i) state_d = RESPONSE;
      RESPONSE: if (data_response_valid_i && data_response_ready_o) begin
        if (data_response_i.error) begin
          completion_d.exception.valid = 1'b1;
          completion_d.exception.cause = current.is_store ? CAUSE_STORE_ACCESS : CAUSE_LOAD_ACCESS;
          completion_d.exception.tval = current.address;
          completion_d.value = '0;
        end else begin
          if (!current.is_store) begin
            case (current.size)
              MEM_BYTE: completion_d.value = current.load_unsigned
                ? {24'b0, extracted[7:0]} : {{24{extracted[7]}}, extracted[7:0]};
              MEM_HALF: completion_d.value = current.load_unsigned
                ? {16'b0, extracted[15:0]} : {{16{extracted[15]}}, extracted[15:0]};
              default: completion_d.value = data_response_i.rdata;
            endcase
          end
          completion_d.memory_access.valid = 1'b1;
          completion_d.memory_access.write = current.is_store;
          completion_d.memory_access.address = current.address;
          if (current.is_store) begin
            completion_d.memory_access.wdata = request_q.wdata;
            completion_d.memory_access.wstrb = request_q.wstrb;
          end
        end
        state_d = COMPLETE;
      end
      COMPLETE: if (completion_valid_o && completion_ready_i) begin
        queue_d[head_q] = '0;
        head_d = head_q + 2'd1;
        popped = 1;
        state_d = IDLE;
      end
      default: state_d = IDLE;
    endcase
    tail_d = allocation_index;
    count_d = 3'(int'(count_q) + pushed - popped);
    if (flush_i && state_q == IDLE) begin
      for (int i = 0; i < MEMORY_DEPTH; i++) queue_d[i] = '0;
      head_d = '0;
      tail_d = '0;
      count_d = '0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < MEMORY_DEPTH; i++) queue_q[i] <= '0;
      state_q <= IDLE;
      head_q <= '0;
      tail_q <= '0;
      count_q <= '0;
      request_q <= '0;
      completion_q <= '0;
      flush_done_o <= 1'b0;
    end else begin
      for (int i = 0; i < MEMORY_DEPTH; i++) queue_q[i] <= queue_d[i];
      state_q <= state_d;
      head_q <= head_d;
      tail_q <= tail_d;
      count_q <= count_d;
      request_q <= request_d;
      completion_q <= completion_d;
      flush_done_o <= flush_i && state_q == IDLE;
    end
  end
endmodule
