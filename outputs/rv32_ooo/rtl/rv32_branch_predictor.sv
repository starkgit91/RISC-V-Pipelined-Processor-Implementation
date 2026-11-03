// IF lookup / WB retirement training: tagged BTB and two-bit bimodal direction table.
// Speculative execution has no training port. Only a committed retirement prefix trains state.
`timescale 1ns / 1ps

module rv32_branch_predictor
  import rv32_ooo_pkg::*;
(
  input  logic             clk_i,
  input  logic             rst_ni,
  input  logic             enable_i,
  input  word_t            query_pc0_i,
  input  word_t            query_pc1_i,
  output prediction_pair_t prediction_o,
  input  logic [1:0]       retire_valid_i,
  input  rob_entry_pair_t  retire_entry_i,
  input  logic             invalidate_i,
  output logic             invalidate_done_o,
  output logic [1:0]       query0_counter_o,
  output logic [1:0]       query1_counter_o,
  output logic             protocol_error_o
);
  localparam int BTB_INDEX_W = $clog2(BTB_ENTRIES);

  typedef struct packed {
    logic valid;
    logic conditional;
    logic [XLEN-1:BTB_INDEX_W+2] tag;
    word_t target;
  } btb_entry_t;

  btb_entry_t btb_q [BTB_ENTRIES];
  btb_entry_t btb_d [BTB_ENTRIES];
  logic [1:0] pht_q [PHT_ENTRIES];
  logic [1:0] pht_d [PHT_ENTRIES];

  function automatic logic [1:0] train_counter(
    input logic [1:0] counter, input logic taken
  );
    begin
      train_counter = counter;
      if (taken && counter != 2'b11) train_counter = counter + 2'd1;
      else if (!taken && counter != 2'b00) train_counter = counter - 2'd1;
    end
  endfunction

  function automatic prediction_t lookup(
    input word_t pc, input btb_entry_t entry,
    input logic [1:0] counter, input logic allowed
  );
    prediction_t result;
    begin
      result = '0;
      result.next_pc = pc + 32'd4;
      result.pht_index = pc[PHT_INDEX_W+1:2];
      if (allowed && pc[1:0] == 0 && entry.valid
          && entry.tag == pc[XLEN-1:BTB_INDEX_W+2]) begin
        result.btb_hit = 1'b1;
        result.taken = !entry.conditional || counter[1];
        if (result.taken) result.next_pc = entry.target;
      end
      lookup = result;
    end
  endfunction

  function automatic logic retirement_well_formed(input rob_entry_t entry);
    decoded_uop_t decoded;
    word_t target, actual_next;
    logic good;
    begin
      decoded = entry.uop.decoded;
      target = decoded.fetch.pc + decoded.immediate;
      actual_next = entry.control_flow.next_pc;
      good = entry.valid && entry.complete && !entry.exception.valid
             && decoded.fetch.pc[1:0] == 0 && decoded.op_class != UOP_NONE;
      if (decoded.op_class == UOP_BRANCH) begin
        good = good && entry.control_flow.is_control && entry.control_flow.is_conditional
               && decoded.fetch.prediction.pht_index == decoded.fetch.pc[PHT_INDEX_W+1:2]
               && actual_next == (entry.control_flow.taken ? target : decoded.fetch.pc + 32'd4)
               && (!entry.control_flow.taken || target[1:0] == 0);
      end else if (decoded.op_class == UOP_JUMP) begin
        good = good && entry.control_flow.is_control && !entry.control_flow.is_conditional
               && entry.control_flow.taken && actual_next[1:0] == 0
               && (decoded.jump_indirect || actual_next == target);
      end
      // Nonbranch SYSTEM instructions may later redirect via MRET. Their next-PC
      // metadata must not be constrained to the ordinary sequential value here.
      retirement_well_formed = good;
    end
  endfunction

  always_comb begin : query_tables
    btb_entry_t first, second;
    logic allowed;
    first = btb_q[query_pc0_i[BTB_INDEX_W+1:2]];
    second = btb_q[query_pc1_i[BTB_INDEX_W+1:2]];
    query0_counter_o = rst_ni ? pht_q[query_pc0_i[PHT_INDEX_W+1:2]] : 2'b01;
    query1_counter_o = rst_ni ? pht_q[query_pc1_i[PHT_INDEX_W+1:2]] : 2'b01;
    allowed = rst_ni && enable_i && !invalidate_i;
    prediction_o.lane0 = lookup(query_pc0_i, first, query0_counter_o, allowed);
    prediction_o.lane1 = lookup(query_pc1_i, second, query1_counter_o, allowed);
  end

  always_comb begin : retirement_training
    rob_entry_t retired;
    decoded_uop_t decoded;
    btb_entry_t updated;
    word_t target;
    int index;
    pht_index_t counter_index;
    logic older_good, good;

    for (int i = 0; i < BTB_ENTRIES; i++) btb_d[i] = btb_q[i];
    for (int i = 0; i < PHT_ENTRIES; i++) pht_d[i] = pht_q[i];
    protocol_error_o = 1'b0;
    retired = '0;
    decoded = '0;
    updated = '0;
    target = '0;
    index = 0;
    counter_index = '0;
    older_good = 1'b1;
    good = 1'b0;

    if (invalidate_i) begin
      for (int i = 0; i < BTB_ENTRIES; i++) btb_d[i] = '0;
      for (int i = 0; i < PHT_ENTRIES; i++) pht_d[i] = 2'b01;
    end else if (rst_ni) begin
      if (retire_valid_i == 2'b10) begin
        protocol_error_o = 1'b1;
      end else begin
        // Fold into NEXT state in architectural order. Two same-index outcomes
        // must both affect saturation; the younger BTB replacement wins an alias.
        for (int lane = 0; lane < RETIRE_WIDTH; lane++) begin
          retired = lane == 0 ? retire_entry_i.lane0 : retire_entry_i.lane1;
          if (retire_valid_i[lane]) begin
            good = older_good && retirement_well_formed(retired);
            if (!good) begin
              protocol_error_o = 1'b1;
              if (lane == 0) older_good = 1'b0;
            end else begin
              decoded = retired.uop.decoded;
              index = int'(decoded.fetch.pc[BTB_INDEX_W+1:2]);
              updated = btb_d[index];
              if (decoded.op_class == UOP_BRANCH || decoded.op_class == UOP_JUMP) begin
                target = decoded.op_class == UOP_BRANCH
                         ? decoded.fetch.pc + decoded.immediate : retired.control_flow.next_pc;
                if (decoded.op_class == UOP_BRANCH) begin
                  counter_index = decoded.fetch.prediction.pht_index;
                  pht_d[counter_index] = train_counter(pht_d[counter_index],
                                                       retired.control_flow.taken);
                end
                if (target[1:0] == 0) begin
                  updated.valid = 1'b1;
                  updated.conditional = decoded.op_class == UOP_BRANCH;
                  updated.tag = decoded.fetch.pc[XLEN-1:BTB_INDEX_W+2];
                  updated.target = target;
                end else if (updated.valid
                             && updated.tag == decoded.fetch.pc[XLEN-1:BTB_INDEX_W+2]) begin
                  // A legal not-taken branch may encode a misaligned unused target.
                  // Learn its N outcome, but do not install an unusable target.
                  updated.valid = 1'b0;
                end
              end else if (updated.valid
                           && updated.tag == decoded.fetch.pc[XLEN-1:BTB_INDEX_W+2]) begin
                // Retired noncontrol code removes only its own stale BTB prediction.
                // An unrelated PC sharing this index must not evict the cached branch.
                updated.valid = 1'b0;
              end
              btb_d[index] = updated;
            end
          end
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < BTB_ENTRIES; i++) btb_q[i] <= '0;
      for (int i = 0; i < PHT_ENTRIES; i++) pht_q[i] <= 2'b01;
      invalidate_done_o <= 1'b0;
    end else begin
      for (int i = 0; i < BTB_ENTRIES; i++) btb_q[i] <= btb_d[i];
      for (int i = 0; i < PHT_ENTRIES; i++) pht_q[i] <= pht_d[i];
      invalidate_done_o <= invalidate_i;
    end
  end
endmodule
