// Eight-entry, two-issue scheduler. Program age comes from the live ROB window.
// Ready inputs advertise execution acceptance THIS edge, not future capacity.
`timescale 1ns / 1ps

module rv32_issue_queue
  import rv32_ooo_pkg::*;
(
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  logic                 dispatch_hold_i,
  input  logic                 issue_hold_i,
  input  logic [1:0]           dispatch_valid_i,
  input  renamed_pair_t        dispatch_uop_i,
  input  logic [1:0]           dispatch_src1_ready_i,
  input  logic [1:0]           dispatch_src2_ready_i,
  output logic [1:0]           dispatch_ready_o,
  output logic [1:0]           dispatch_fire_o,

  // Broadcast only completions accepted by the ROB ownership checks.
  input  logic [1:0]           completion_valid_i,
  input  completion_pair_t     completion_i,
  input  logic                 rob_head_valid_i,
  input  rob_index_t           rob_head_index_i,
  input  rob_tag_t             rob_head_tag_i,
  input  logic                 mispredict_pending_i,
  input  rob_tag_t             mispredict_tag_i,

  // Port 0: integer/control/address/system. Port 1: integer or multiply/divide.
  input  logic                 alu0_ready_i,
  input  logic                 alu1_ready_i,
  input  logic                 muldiv_ready_i,
  input  logic                 memory_ready_i,
  input  logic                 system_ready_i,
  output logic [1:0]           issue_fire_o,
  output execute_packet_pair_t issue_packet_o,
  output phys_reg_t            read0_rs1_o,
  output phys_reg_t            read0_rs2_o,
  output phys_reg_t            read1_rs1_o,
  output phys_reg_t            read1_rs2_o,
  input  word_t                read0_data1_i,
  input  word_t                read0_data2_i,
  input  word_t                read1_data1_i,
  input  word_t                read1_data2_i,

  // Apply only after recovery has cancelled/drained all executing owners.
  input  logic                 flush_i,
  output logic                 flush_done_o,
  output issue_count_t         occupancy_o,
  output logic [ISSUE_DEPTH-1:0] occupied_mask_o,
  output logic                 protocol_error_o
);

  issue_entry_t entries_q [ISSUE_DEPTH];
  issue_entry_t entries_d [ISSUE_DEPTH];
  renamed_pair_t selected_uop;

  function automatic logic source_ready(
    input logic initial_ready,
    input logic used,
    input phys_reg_t source_tag,
    input logic [1:0] wb_valid,
    input completion_pair_t wb
  );
    logic hit0, hit1;
    begin
      hit0 = wb_valid[0] && wb.lane0.writes_rd && !wb.lane0.exception.valid
             && wb.lane0.pdst != 0 && wb.lane0.pdst == source_tag;
      hit1 = wb_valid[1] && wb.lane1.writes_rd && !wb.lane1.exception.valid
             && wb.lane1.pdst != 0 && wb.lane1.pdst == source_tag;
      source_ready = initial_ready || !used || source_tag == 0 || hit0 || hit1;
    end
  endfunction

  function automatic int program_age(input rob_index_t index, input rob_index_t head);
    rob_index_t distance;
    begin
      distance = index - head;
      program_age = int'(distance);
    end
  endfunction

  // Selection does not depend on PRF data. This avoids a combinational cycle
  // through the read multiplexers; a separate path attaches the selected values.
  always_comb begin : schedule
    issue_entry_t entry;
    renamed_uop_t incoming0, incoming1;
    logic [ISSUE_DEPTH-1:0] eligible0, eligible1, is_system;
    int age [ISSUE_DEPTH];
    int free0, free1, free_count;
    int oldest, partner, select0, select1, boundary_age;
    logic duplicate0, duplicate1, malformed0, malformed1;
    logic skip0, skip1, allow_dispatch;

    incoming0 = dispatch_uop_i.lane0;
    incoming1 = dispatch_uop_i.lane1;
    eligible0 = '0;
    eligible1 = '0;
    is_system = '0;
    occupied_mask_o = '0;
    occupancy_o = '0;
    free0 = -1;
    free1 = -1;
    free_count = 0;
    duplicate0 = 1'b0;
    duplicate1 = 1'b0;
    boundary_age = program_age(mispredict_tag_i.index, rob_head_index_i);

    for (int i = 0; i < ISSUE_DEPTH; i++) begin
      entry = entries_q[i];
      age[i] = program_age(entry.uop.rob_tag.index, rob_head_index_i);
      occupied_mask_o[i] = entry.valid;
      if (entry.valid) begin
        occupancy_o = occupancy_o + issue_count_t'(1);
        duplicate0 = duplicate0 || entry.uop.rob_tag == incoming0.rob_tag;
        duplicate1 = duplicate1 || entry.uop.rob_tag == incoming1.rob_tag;
      end else begin
        if (free0 < 0) free0 = i;
        else if (free1 < 0) free1 = i;
        free_count = free_count + 1;
      end
      entry.src1_ready = source_ready(entry.src1_ready, entry.uop.decoded.uses_rs1,
                                     entry.uop.prs1, completion_valid_i, completion_i);
      entry.src2_ready = source_ready(entry.src2_ready, entry.uop.decoded.uses_rs2,
                                     entry.uop.prs2, completion_valid_i, completion_i);
      entries_d[i] = entry;
      is_system[i] = entry.uop.decoded.op_class == UOP_SYSTEM;

      if (rst_ni && !flush_i && !issue_hold_i && rob_head_valid_i && entry.valid
          && entry.src1_ready && entry.src2_ready
          && (!mispredict_pending_i || age[i] < boundary_age)) begin
        case (entry.uop.decoded.op_class)
          UOP_INTEGER: begin
            eligible0[i] = alu0_ready_i;
            eligible1[i] = alu1_ready_i;
          end
          UOP_BRANCH, UOP_JUMP: eligible0[i] = alu0_ready_i;
          UOP_MULDIV: eligible1[i] = muldiv_ready_i;
          UOP_LOAD, UOP_STORE: eligible0[i] = memory_ready_i;
          UOP_SYSTEM: eligible0[i] = system_ready_i
                                        && entry.uop.rob_tag == rob_head_tag_i;
          default: begin end
        endcase
      end
    end

    // Include the oldest eligible operation, then the oldest compatible partner.
    // Flexible INTEGER placement preserves a port-0-only partner's opportunity.
    oldest = -1;
    partner = -1;
    select0 = -1;
    select1 = -1;
    for (int i = 0; i < ISSUE_DEPTH; i++) begin
      if ((eligible0[i] || eligible1[i]) && (oldest < 0 || age[i] < age[oldest]))
        oldest = i;
    end
    if (oldest >= 0) begin
      if (is_system[oldest]) begin
        select0 = oldest;
      end else begin
        for (int i = 0; i < ISSUE_DEPTH; i++) begin
          if (i != oldest && !is_system[i]
              && ((eligible0[oldest] && eligible1[i])
                  || (eligible1[oldest] && eligible0[i]))
              && (partner < 0 || age[i] < age[partner])) partner = i;
        end
        if (partner < 0) begin
          if (eligible0[oldest]) select0 = oldest;
          else select1 = oldest;
        end else if (eligible0[oldest] && eligible1[partner]) begin
          select0 = oldest;
          select1 = partner;
        end else begin
          select1 = oldest;
          select0 = partner;
        end
      end
    end

    issue_fire_o = '0;
    selected_uop = '0;
    read0_rs1_o = '0;
    read0_rs2_o = '0;
    read1_rs1_o = '0;
    read1_rs2_o = '0;
    if (select0 >= 0) begin
      entry = entries_d[select0];
      issue_fire_o[0] = 1'b1;
      selected_uop.lane0 = entry.uop;
      if (entry.uop.decoded.uses_rs1) read0_rs1_o = entry.uop.prs1;
      if (entry.uop.decoded.uses_rs2) read0_rs2_o = entry.uop.prs2;
      entry.valid = 1'b0;
      entries_d[select0] = entry;
    end
    if (select1 >= 0) begin
      entry = entries_d[select1];
      issue_fire_o[1] = 1'b1;
      selected_uop.lane1 = entry.uop;
      if (entry.uop.decoded.uses_rs1) read1_rs1_o = entry.uop.prs1;
      if (entry.uop.decoded.uses_rs2) read1_rs2_o = entry.uop.prs2;
      entry.valid = 1'b0;
      entries_d[select1] = entry;
    end

    // Preview readiness must not depend on valid[0]: atomic dispatch may derive
    // each participant's valid from the combined readiness of all participants.
    skip0 = incoming0.decoded.fetch.exception.valid;
    skip1 = incoming1.decoded.fetch.exception.valid;
    malformed0 = incoming0.decoded.op_class == UOP_NONE && !skip0;
    malformed1 = incoming1.decoded.op_class == UOP_NONE && !skip1;
    allow_dispatch = rst_ni && !flush_i && !dispatch_hold_i && !issue_hold_i
                     && !mispredict_pending_i;
    dispatch_ready_o = '0;
    dispatch_fire_o = '0;
    dispatch_ready_o[0] = allow_dispatch && !duplicate0 && !malformed0
                          && (skip0 || free_count >= 1);
    dispatch_fire_o[0] = dispatch_valid_i[0] && dispatch_ready_o[0];
    dispatch_ready_o[1] = dispatch_fire_o[0] && !skip0
                          && incoming0.decoded.op_class != UOP_SYSTEM
                          && !duplicate1 && !malformed1
                          && incoming1.rob_tag != incoming0.rob_tag
                          && (skip1 || free_count >= 2);
    dispatch_fire_o[1] = dispatch_valid_i[1] && dispatch_ready_o[1];

    if (dispatch_fire_o[0] && !skip0) begin
      entry = '0;
      entry.valid = 1'b1;
      entry.uop = incoming0;
      entry.src1_ready = source_ready(dispatch_src1_ready_i[0],
                        incoming0.decoded.uses_rs1, incoming0.prs1,
                        completion_valid_i, completion_i);
      entry.src2_ready = source_ready(dispatch_src2_ready_i[0],
                        incoming0.decoded.uses_rs2, incoming0.prs2,
                        completion_valid_i, completion_i);
      entries_d[free0] = entry;
    end
    if (dispatch_fire_o[1] && !skip1) begin
      entry = '0;
      entry.valid = 1'b1;
      entry.uop = incoming1;
      entry.src1_ready = source_ready(dispatch_src1_ready_i[1],
                        incoming1.decoded.uses_rs1, incoming1.prs1,
                        completion_valid_i, completion_i);
      entry.src2_ready = source_ready(dispatch_src2_ready_i[1],
                        incoming1.decoded.uses_rs2, incoming1.prs2,
                        completion_valid_i, completion_i);
      entries_d[free1] = entry;
    end

    protocol_error_o = 1'b0;
    if (rst_ni && !flush_i) begin
      protocol_error_o = dispatch_valid_i == 2'b10
                        || (dispatch_valid_i[0] && (duplicate0 || malformed0))
                        || (dispatch_valid_i[1] && (duplicate1 || malformed1))
                        || (dispatch_valid_i == 2'b11
                            && incoming0.rob_tag == incoming1.rob_tag);
    end
  end

  // The selected tags drive the four PRF read muxes. Accepted completion bypass
  // in the PRF makes a newly awakened value available on this same issue edge.
  always_comb begin
    issue_packet_o = '0;
    if (issue_fire_o[0]) begin
      issue_packet_o.lane0.uop = selected_uop.lane0;
      issue_packet_o.lane0.rs1_value = read0_data1_i;
      issue_packet_o.lane0.rs2_value = read0_data2_i;
    end
    if (issue_fire_o[1]) begin
      issue_packet_o.lane1.uop = selected_uop.lane1;
      issue_packet_o.lane1.rs1_value = read1_data1_i;
      issue_packet_o.lane1.rs2_value = read1_data2_i;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < ISSUE_DEPTH; i++) entries_q[i] <= '0;
      flush_done_o <= 1'b0;
    end else begin
      flush_done_o <= flush_i;
      for (int i = 0; i < ISSUE_DEPTH; i++) begin
        if (flush_i) entries_q[i] <= '0;
        else entries_q[i] <= entries_d[i];
      end
    end
  end

endmodule
