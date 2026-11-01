// Two-wide ID rename: lane 0 is older; lane 1 observes lane 0's accepted mapping update.
// Completion ownership and valid in-order retirement are supplied by the ROB/controller.
`timescale 1ns / 1ps

module rv32_rename (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  input  logic [1:0]                   dispatch_valid_i,
  input  logic [1:0]                   dispatch_resources_ready_i,
  input  rv32_ooo_pkg::decoded_pair_t   decoded_i,
  input  rv32_ooo_pkg::rob_tag_pair_t   dispatch_rob_tag_i,
  output logic [1:0]                   dispatch_ready_o,
  output logic [1:0]                   dispatch_fire_o,
  output rv32_ooo_pkg::renamed_pair_t   renamed_o,
  output logic [1:0]                   src1_ready_o,
  output logic [1:0]                   src2_ready_o,
  input  logic [1:0]                   completion_valid_i,
  input  rv32_ooo_pkg::completion_pair_t completion_i,
  input  logic [1:0]                   retire_valid_i,
  input  rv32_ooo_pkg::renamed_pair_t   retire_uop_i,
  input  logic                         recover_i,
  output logic                         recover_done_o,
  input  rv32_ooo_pkg::arch_reg_t       committed_query_arch_i,
  output rv32_ooo_pkg::phys_reg_t       committed_query_phys_o,
  output rv32_ooo_pkg::phys_mask_t      free_mask_o,
  output rv32_ooo_pkg::phys_mask_t      ready_mask_o,
  output rv32_ooo_pkg::phys_count_t     free_count_o,
  output logic                         protocol_error_o
);
  import rv32_ooo_pkg::*;

  phys_reg_t speculative_q [0:ARCH_REG_COUNT-1];
  phys_reg_t speculative_d [0:ARCH_REG_COUNT-1];
  phys_reg_t after_lane0 [0:ARCH_REG_COUNT-1];
  phys_reg_t committed_q [0:ARCH_REG_COUNT-1];
  phys_reg_t committed_d [0:ARCH_REG_COUNT-1];
  phys_mask_t free_q, free_d, free_after0;
  phys_mask_t ready_q, ready_d, ready_lookup0, ready_lookup1;
  phys_reg_t allocate0, allocate1;
  decoded_uop_t dec0, dec1;
  completion_t wb0, wb1;
  renamed_uop_t retiring0, retiring1, renamed0, renamed1;
  logic need0, need1, complete0, complete1, completion_conflict;
  logic retire_write0, retire_write1;
  integer map_index;
  integer state_index;
  integer reset_index;
  integer count_index;
  integer free_count;

  assign dec0 = decoded_i.lane0;
  assign dec1 = decoded_i.lane1;
  assign wb0 = completion_i.lane0;
  assign wb1 = completion_i.lane1;
  assign retiring0 = retire_uop_i.lane0;
  assign retiring1 = retire_uop_i.lane1;
  assign renamed_o.lane0 = renamed0;
  assign renamed_o.lane1 = renamed1;
  assign free_mask_o = free_q;
  assign ready_mask_o = ready_q;
  assign committed_query_phys_o = committed_q[committed_query_arch_i];

  function automatic phys_reg_t first_free(input phys_mask_t mask);
    integer candidate;
    begin
      first_free = '0;
      for (candidate = PHYS_REG_COUNT-1; candidate >= 1; candidate = candidate - 1)
        if (mask[candidate]) first_free = candidate[PHYS_REG_W-1:0];
    end
  endfunction

  function automatic logic needs_destination(input decoded_uop_t operation);
    needs_destination = operation.writes_rd && operation.rd != 0 &&
                        !operation.fetch.exception.valid;
  endfunction

  always_comb begin
    need0 = needs_destination(dec0);
    need1 = needs_destination(dec1);
    complete0 = completion_valid_i[0] && wb0.writes_rd && !wb0.exception.valid &&
                wb0.pdst != 0 && !free_q[wb0.pdst];
    complete1 = completion_valid_i[1] && wb1.writes_rd && !wb1.exception.valid &&
                wb1.pdst != 0 && !free_q[wb1.pdst];
    completion_conflict = complete0 && complete1 && wb0.pdst == wb1.pdst;
    // A duplicate physical write violates the upstream contract; lane 0 has priority.
    if (completion_conflict) complete1 = 1'b0;
    protocol_error_o = rst_ni && ((dispatch_valid_i == 2'b10) ||
                       (retire_valid_i == 2'b10) || completion_conflict);
    ready_lookup0 = ready_q;
    if (complete0) ready_lookup0[wb0.pdst] = 1'b1;
    if (complete1) ready_lookup0[wb1.pdst] = 1'b1;

    allocate0 = first_free(free_q);
    // Ready advertises capacity independently of valid; invalid 10 still cannot fire lane 1.
    dispatch_ready_o[0] = rst_ni && !recover_i && dispatch_resources_ready_i[0] &&
                          (!need0 || allocate0 != 0);
    dispatch_fire_o[0] = dispatch_valid_i[0] && dispatch_ready_o[0];

    renamed0 = '0;
    renamed0.decoded = dec0;
    renamed0.decoded.writes_rd = need0;
    renamed0.rob_tag = dispatch_rob_tag_i.lane0;
    if (dec0.fetch.exception.valid) begin
      renamed0.decoded.uses_rs1 = 1'b0;
      renamed0.decoded.uses_rs2 = 1'b0;
    end else begin
      if (dec0.uses_rs1) renamed0.prs1 = speculative_q[dec0.rs1];
      if (dec0.uses_rs2) renamed0.prs2 = speculative_q[dec0.rs2];
    end
    if (need0) begin
      renamed0.pdst = allocate0;
      renamed0.old_pdst = speculative_q[dec0.rd];
    end
    src1_ready_o[0] = renamed0.prs1 == 0 || ready_lookup0[renamed0.prs1];
    src2_ready_o[0] = renamed0.prs2 == 0 || ready_lookup0[renamed0.prs2];

    for (map_index = 0; map_index < ARCH_REG_COUNT; map_index = map_index + 1)
      after_lane0[map_index] = speculative_q[map_index];
    free_after0 = free_q;
    ready_lookup1 = ready_lookup0;
    if (dispatch_fire_o[0] && need0) begin
      after_lane0[dec0.rd] = allocate0;
      free_after0[allocate0] = 1'b0;
      ready_lookup1[allocate0] = 1'b0;
    end

    allocate1 = first_free(free_after0);
    // Lane 1 can transfer only with lane 0; the source must compact a stalled suffix next cycle.
    dispatch_ready_o[1] = dispatch_fire_o[0] && dispatch_resources_ready_i[1] &&
                          (!need1 || allocate1 != 0);
    dispatch_fire_o[1] = dispatch_valid_i[1] && dispatch_ready_o[1];
    renamed1 = '0;
    renamed1.decoded = dec1;
    renamed1.decoded.writes_rd = need1;
    renamed1.rob_tag = dispatch_rob_tag_i.lane1;
    if (dec1.fetch.exception.valid) begin
      renamed1.decoded.uses_rs1 = 1'b0;
      renamed1.decoded.uses_rs2 = 1'b0;
    end else begin
      if (dec1.uses_rs1) renamed1.prs1 = after_lane0[dec1.rs1];
      if (dec1.uses_rs2) renamed1.prs2 = after_lane0[dec1.rs2];
    end
    if (need1) begin
      renamed1.pdst = allocate1;
      renamed1.old_pdst = after_lane0[dec1.rd];
    end
    src1_ready_o[1] = renamed1.prs1 == 0 || ready_lookup1[renamed1.prs1];
    src2_ready_o[1] = renamed1.prs2 == 0 || ready_lookup1[renamed1.prs2];

    retire_write0 = retire_valid_i[0] && needs_destination(retiring0.decoded);
    retire_write1 = (&retire_valid_i) && needs_destination(retiring1.decoded);
    free_count = 0;
    for (count_index = 1; count_index < PHYS_REG_COUNT; count_index = count_index + 1)
      if (free_q[count_index]) free_count = free_count + 1;
    free_count_o = free_count[$clog2(PHYS_REG_COUNT+1)-1:0];
  end

  always_comb begin
    for (state_index = 0; state_index < ARCH_REG_COUNT; state_index = state_index + 1) begin
      speculative_d[state_index] = after_lane0[state_index];
      committed_d[state_index] = committed_q[state_index];
    end
    free_d = free_q;
    ready_d = ready_lookup0;
    // Retirement is an ordered prefix. The second writer sees the first committed update.
    if (retire_write0) begin
      committed_d[retiring0.decoded.rd] = retiring0.pdst;
      if (retiring0.old_pdst != 0) begin
        free_d[retiring0.old_pdst] = 1'b1;
        ready_d[retiring0.old_pdst] = 1'b0;
      end
    end
    if (retire_write1) begin
      committed_d[retiring1.decoded.rd] = retiring1.pdst;
      if (retiring1.old_pdst != 0) begin
        free_d[retiring1.old_pdst] = 1'b1;
        ready_d[retiring1.old_pdst] = 1'b0;
      end
    end
    if (dispatch_fire_o[0] && need0) begin
      free_d[allocate0] = 1'b0;
      ready_d[allocate0] = 1'b0;
    end
    if (dispatch_fire_o[1] && need1) begin
      speculative_d[dec1.rd] = allocate1;
      free_d[allocate1] = 1'b0;
      ready_d[allocate1] = 1'b0;
    end
    if (recover_i) begin
      // APPLY only after cancelled execution and completion queues acknowledge quiescence.
      free_d = '1;
      ready_d = '0;
      for (state_index = 0; state_index < ARCH_REG_COUNT; state_index = state_index + 1) begin
        speculative_d[state_index] = committed_d[state_index];
        free_d[committed_d[state_index]] = 1'b0;
        ready_d[committed_d[state_index]] = 1'b1;
      end
    end
    speculative_d[0] = '0;
    committed_d[0] = '0;
    free_d[0] = 1'b0;
    ready_d[0] = 1'b1;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (reset_index = 0; reset_index < ARCH_REG_COUNT; reset_index = reset_index + 1) begin
        speculative_q[reset_index] <= reset_index[PHYS_REG_W-1:0];
        committed_q[reset_index] <= reset_index[PHYS_REG_W-1:0];
      end
      for (reset_index = 0; reset_index < PHYS_REG_COUNT; reset_index = reset_index + 1) begin
        free_q[reset_index] <= reset_index >= ARCH_REG_COUNT;
        ready_q[reset_index] <= reset_index < ARCH_REG_COUNT;
      end
      recover_done_o <= 1'b0;
    end else begin
      for (reset_index = 0; reset_index < ARCH_REG_COUNT; reset_index = reset_index + 1) begin
        speculative_q[reset_index] <= speculative_d[reset_index];
        committed_q[reset_index] <= committed_d[reset_index];
      end
      free_q <= free_d;
      ready_q <= ready_d;
      recover_done_o <= recover_i;
    end
  end
endmodule
