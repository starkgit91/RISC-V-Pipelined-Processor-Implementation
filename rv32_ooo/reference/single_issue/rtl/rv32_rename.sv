// ID rename: map program register names to physical versions and track availability.
// The ROB/completion controller supplies validated retirement and completion events.
`timescale 1ns / 1ps

module rv32_rename (
  input  logic                       clk_i,
  input  logic                       rst_ni,
  input  logic                       dispatch_valid_i,
  input  logic                       dispatch_resources_ready_i,
  input  rv32_ooo_pkg::decoded_uop_t  decoded_i,
  input  rv32_ooo_pkg::rob_tag_t      dispatch_rob_tag_i,
  output logic                       dispatch_ready_o,
  output rv32_ooo_pkg::renamed_uop_t  renamed_o,
  output logic                       src1_ready_o,
  output logic                       src2_ready_o,
  input  logic                       completion_valid_i,
  input  rv32_ooo_pkg::completion_t   completion_i,
  input  logic                       retire_valid_i,
  input  rv32_ooo_pkg::renamed_uop_t  retire_uop_i,
  input  logic                       recover_i,
  output logic                       recover_done_o,
  input  rv32_ooo_pkg::arch_reg_t     committed_query_arch_i,
  output rv32_ooo_pkg::phys_reg_t     committed_query_phys_o,
  output rv32_ooo_pkg::phys_mask_t    free_mask_o,
  output rv32_ooo_pkg::phys_mask_t    ready_mask_o,
  output rv32_ooo_pkg::phys_count_t   free_count_o
);
  import rv32_ooo_pkg::*;

  phys_reg_t speculative_q [0:ARCH_REG_COUNT-1];
  phys_reg_t speculative_d [0:ARCH_REG_COUNT-1];
  phys_reg_t committed_q [0:ARCH_REG_COUNT-1];
  phys_reg_t committed_d [0:ARCH_REG_COUNT-1];
  phys_mask_t free_q, free_d;
  phys_mask_t ready_q, ready_d;
  phys_reg_t allocated_phys;
  logic allocation_available;
  logic needs_destination;
  logic dispatch_fire;
  logic completion_sets_ready;
  logic retirement_writes;
  integer allocate_index;
  integer count_index;
  integer state_index;
  integer reset_index;
  integer free_count;

  assign free_mask_o = free_q;
  assign ready_mask_o = ready_q;
  assign committed_query_phys_o = committed_q[committed_query_arch_i];

  always_comb begin
    allocated_phys = '0;
    allocation_available = 1'b0;
    // Allocate from CURRENT free entries. A retirement release is usable next cycle.
    for (allocate_index = 1; allocate_index < PHYS_REG_COUNT; allocate_index = allocate_index + 1)
      if (!allocation_available && free_q[allocate_index]) begin
        allocated_phys = allocate_index[PHYS_REG_W-1:0];
        allocation_available = 1'b1;
      end

    free_count = 0;
    for (count_index = 1; count_index < PHYS_REG_COUNT; count_index = count_index + 1)
      if (free_q[count_index]) free_count = free_count + 1;
    free_count_o = free_count[$clog2(PHYS_REG_COUNT+1)-1:0];
  end

  always_comb begin
    needs_destination = decoded_i.writes_rd && (decoded_i.rd != 0) &&
                        !decoded_i.fetch.exception.valid;
    completion_sets_ready = completion_valid_i && completion_i.writes_rd &&
                            !completion_i.exception.valid && (completion_i.pdst != 0) &&
                            !free_q[completion_i.pdst];
    retirement_writes = retire_valid_i && retire_uop_i.decoded.writes_rd &&
                        (retire_uop_i.decoded.rd != 0) &&
                        !retire_uop_i.decoded.fetch.exception.valid;

    renamed_o = '0;
    renamed_o.decoded = decoded_i;
    renamed_o.decoded.writes_rd = needs_destination;
    renamed_o.rob_tag = dispatch_rob_tag_i;
    if (decoded_i.fetch.exception.valid) begin
      renamed_o.decoded.uses_rs1 = 1'b0;
      renamed_o.decoded.uses_rs2 = 1'b0;
    end else begin
      // Read the old mappings BEFORE replacing the destination, including rd==rs1/rs2.
      if (decoded_i.uses_rs1) renamed_o.prs1 = speculative_q[decoded_i.rs1];
      if (decoded_i.uses_rs2) renamed_o.prs2 = speculative_q[decoded_i.rs2];
    end
    if (needs_destination) begin
      renamed_o.old_pdst = speculative_q[decoded_i.rd];
      if (allocation_available) renamed_o.pdst = allocated_phys;
    end

    src1_ready_o = (renamed_o.prs1 == 0) || ready_q[renamed_o.prs1] ||
                  (completion_sets_ready && completion_i.pdst == renamed_o.prs1);
    src2_ready_o = (renamed_o.prs2 == 0) || ready_q[renamed_o.prs2] ||
                  (completion_sets_ready && completion_i.pdst == renamed_o.prs2);
    dispatch_ready_o = rst_ni && !recover_i && dispatch_resources_ready_i &&
                       (!needs_destination || allocation_available);
    dispatch_fire = dispatch_valid_i && dispatch_ready_o;
  end

  always_comb begin
    for (state_index = 0; state_index < ARCH_REG_COUNT; state_index = state_index + 1) begin
      speculative_d[state_index] = speculative_q[state_index];
      committed_d[state_index] = committed_q[state_index];
    end
    free_d = free_q;
    ready_d = ready_q;

    if (completion_sets_ready) ready_d[completion_i.pdst] = 1'b1;

    if (retirement_writes) begin
      committed_d[retire_uop_i.decoded.rd] = retire_uop_i.pdst;
      // old_pdst must equal the current committed mapping at in-order retirement.
      // The controller checks that contract; p0 is never returned to the free list.
      if (retire_uop_i.old_pdst != 0) begin
        free_d[retire_uop_i.old_pdst] = 1'b1;
        ready_d[retire_uop_i.old_pdst] = 1'b0;
      end
    end

    if (dispatch_fire && needs_destination) begin
      speculative_d[decoded_i.rd] = allocated_phys;
      free_d[allocated_phys] = 1'b0;
      ready_d[allocated_phys] = 1'b0;
    end

    if (recover_i) begin
      // recover_i is an APPLY pulse after cancellation acknowledgements and drained results.
      // committed_d includes a same-cycle retiring branch's link-register mapping.
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
