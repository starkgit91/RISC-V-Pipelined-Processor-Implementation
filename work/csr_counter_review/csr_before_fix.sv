// Serialized machine-mode system unit. CSR values change at retirement, never completion.
`timescale 1ns / 1ps
module rv32_csr_file
  import rv32_ooo_pkg::*;
(
  input logic clk_i, rst_ni,
  input logic request_valid_i,
  output logic request_ready_o,
  input execute_packet_t request_i,
  output logic completion_valid_o,
  input logic completion_ready_i,
  output completion_t completion_o,
  input logic [1:0] retire_valid_i,
  input rob_entry_pair_t retire_entry_i,
  input logic irq_external_i, irq_software_i, irq_timer_i,
  output logic interrupt_pending_o,
  output word_t interrupt_cause_o,
  input logic trap_enter_i,
  input word_t trap_pc_i, trap_cause_i, trap_tval_i,
  output word_t trap_target_o,
  output logic wfi_wait_o,
  input logic cancel_i,
  output logic cancel_done_o, quiescent_o, protocol_error_o
);
  localparam word_t IRQ_MASK = 32'h0000_0888;
  logic mie_global_q, mpie_q;
  word_t mie_q, mtvec_q, mscratch_q, mepc_q, mcause_q, mtval_q;
  logic [63:0] cycle_q, instret_q;
  word_t inhibit_q;
  logic waiting_q, pending_q, response_valid_q;
  renamed_uop_t owner_q;
  completion_t result_q, incoming_result;
  csr_addr_t address_q;
  word_t write_value_q, incoming_write_value;
  logic write_q, incoming_write;
  word_t mip, local_pending, csr_read, operand;
  logic csr_known, csr_readonly, illegal, commit_owner;
  logic [1:0] retired_count;

  assign mip = (irq_external_i ? 32'h800 : 0) | (irq_software_i ? 32'h8 : 0)
               | (irq_timer_i ? 32'h80 : 0);
  assign local_pending = mip & mie_q;
  assign interrupt_pending_o = mie_global_q && |local_pending;
  always_comb begin
    interrupt_cause_o = '0;
    if (interrupt_pending_o) begin
      if (local_pending[11]) interrupt_cause_o = CAUSE_IRQ_EXTERNAL;
      else if (local_pending[3]) interrupt_cause_o = CAUSE_IRQ_SOFTWARE;
      else interrupt_cause_o = CAUSE_IRQ_TIMER;
    end
    trap_target_o = {mtvec_q[31:2], 2'b00};
    if (trap_cause_i[31] && mtvec_q[1:0] == 2'b01)
      trap_target_o = trap_target_o + {trap_cause_i[29:0], 2'b00};
  end

  always_comb begin : read_and_prepare
    decoded_uop_t decoded;
    decoded = request_i.uop.decoded;
    csr_known = 1'b1;
    csr_readonly = 1'b0;
    csr_read = '0;
    case (decoded.csr_address)
      CSR_MSTATUS: csr_read = 32'h1800 | (mie_global_q ? 32'h8 : 0)
                             | (mpie_q ? 32'h80 : 0);
      CSR_MISA: csr_read = MISA_VALUE;
      CSR_MSTATUSH: csr_read = '0;
      CSR_MIE: csr_read = mie_q;
      CSR_MIP: csr_read = mip;
      CSR_MTVEC: csr_read = mtvec_q;
      CSR_MSCRATCH: csr_read = mscratch_q;
      CSR_MEPC: csr_read = mepc_q;
      CSR_MCAUSE: csr_read = mcause_q;
      CSR_MTVAL: csr_read = mtval_q;
      CSR_MCOUNTINHIBIT: csr_read = inhibit_q;
      CSR_MCYCLE: csr_read = cycle_q[31:0];
      CSR_MCYCLEH: csr_read = cycle_q[63:32];
      CSR_MINSTRET: csr_read = instret_q[31:0];
      CSR_MINSTRETH: csr_read = instret_q[63:32];
      CSR_MVENDORID, CSR_MARCHID, CSR_MIMPID, CSR_MHARTID, CSR_MCONFIGPTR: begin
        csr_read = '0;
        csr_readonly = 1'b1;
      end
      default: csr_known = 1'b0;
    endcase
    operand = request_i.rs1_value;
    if (decoded.csr_op == CSR_RWI || decoded.csr_op == CSR_RSI || decoded.csr_op == CSR_RCI)
      operand = {27'b0, decoded.csr_immediate};
    incoming_write = 1'b0;
    incoming_write_value = csr_read;
    illegal = decoded.op_class != UOP_SYSTEM;
    incoming_result = '0;
    incoming_result.rob_tag = request_i.uop.rob_tag;
    incoming_result.pdst = request_i.uop.pdst;
    incoming_result.writes_rd = decoded.writes_rd;
    incoming_result.exception = decoded.fetch.exception;
    incoming_result.control_flow.next_pc = decoded.fetch.pc + 32'd4;
    case (decoded.system_op)
      SYS_CSR: begin
        incoming_result.value = csr_read;
        case (decoded.csr_op)
          CSR_RW, CSR_RWI: begin
            incoming_write = 1'b1;
            incoming_write_value = operand;
          end
          CSR_RS, CSR_RSI: begin
            // Register forms suppress by encoded x0, not by a zero register VALUE.
            incoming_write = decoded.csr_op == CSR_RS ? decoded.rs1 != 0 : operand != 0;
            incoming_write_value = csr_read | operand;
          end
          CSR_RC, CSR_RCI: begin
            incoming_write = decoded.csr_op == CSR_RC ? decoded.rs1 != 0 : operand != 0;
            incoming_write_value = csr_read & ~operand;
          end
          default: illegal = 1'b1;
        endcase
        illegal = illegal || !csr_known || (csr_readonly && incoming_write);
      end
      SYS_ECALL: begin
        if (!incoming_result.exception.valid) begin
          incoming_result.exception.valid = 1'b1;
          incoming_result.exception.cause = CAUSE_ECALL_M;
          incoming_result.exception.tval = '0;
        end
      end
      SYS_EBREAK: begin
        if (!incoming_result.exception.valid) begin
          incoming_result.exception.valid = 1'b1;
          incoming_result.exception.cause = CAUSE_BREAKPOINT;
          incoming_result.exception.tval = decoded.fetch.pc;
        end
      end
      SYS_MRET: incoming_result.control_flow.next_pc = mepc_q;
      SYS_WFI, SYS_FENCE, SYS_FENCE_I: begin end
      default: illegal = 1'b1;
    endcase
    if (illegal && !incoming_result.exception.valid) begin
      incoming_result.exception.valid = 1'b1;
      incoming_result.exception.cause = CAUSE_ILLEGAL_INSTR;
      incoming_result.exception.tval = decoded.fetch.instruction;
    end
    if (incoming_result.exception.valid) begin
      incoming_result.value = '0;
      incoming_write = 1'b0;
    end
    incoming_result.control_flow.mispredicted = !incoming_result.exception.valid
      && incoming_result.control_flow.next_pc != decoded.fetch.prediction.next_pc;
  end

  always_comb begin : retirement_match
    rob_entry_t entry;
    logic prefix_ok, older_good, entry_ok, owner_matches;
    retired_count = '0;
    commit_owner = 1'b0;
    protocol_error_o = rst_ni && request_valid_i && !cancel_i && !trap_enter_i
                       && (!request_ready_o || request_i.uop.decoded.op_class != UOP_SYSTEM
                           || request_i.uop.decoded.system_op == SYS_NONE);
    prefix_ok = retire_valid_i != 2'b10;
    older_good = 1'b1;
    if (rst_ni && !prefix_ok) protocol_error_o = 1'b1;
    for (int lane = 0; lane < 2; lane++) begin
      entry = lane == 0 ? retire_entry_i.lane0 : retire_entry_i.lane1;
      entry_ok = older_good && entry.valid && entry.complete && !entry.exception.valid;
      owner_matches = pending_q && entry.uop.rob_tag == owner_q.rob_tag;
      if (rst_ni && prefix_ok && retire_valid_i[lane]) begin
        if (!entry_ok) begin
          protocol_error_o = 1'b1;
          older_good = 1'b0;
        end else begin
          retired_count = retired_count + 2'd1;
          if (owner_matches) begin
            if (response_valid_q || result_q.exception.valid || entry.uop.pdst != owner_q.pdst
                || entry.uop.decoded.writes_rd != owner_q.decoded.writes_rd
                || entry.uop.decoded.system_op != owner_q.decoded.system_op
                || entry.uop.decoded.csr_address != owner_q.decoded.csr_address
                || entry.uop.decoded.csr_op != owner_q.decoded.csr_op) protocol_error_o = 1'b1;
            else commit_owner = 1'b1;
          end
        end
      end
    end
  end

  assign request_ready_o = rst_ni && !cancel_i && !trap_enter_i && !pending_q && !waiting_q;
  assign completion_valid_o = rst_ni && !cancel_i && !trap_enter_i && response_valid_q;
  assign completion_o = completion_valid_o ? result_q : completion_t'('0);
  assign quiescent_o = !pending_q && !response_valid_q;
  assign wfi_wait_o = waiting_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mie_global_q <= 1'b0;
      mpie_q <= 1'b0;
      mie_q <= '0;
      mtvec_q <= '0;
      mscratch_q <= '0;
      mepc_q <= '0;
      mcause_q <= '0;
      mtval_q <= '0;
      inhibit_q <= '0;
      cycle_q <= '0;
      instret_q <= '0;
      waiting_q <= 1'b0;
      pending_q <= 1'b0;
      response_valid_q <= 1'b0;
      owner_q <= '0;
      result_q <= '0;
      address_q <= '0;
      write_value_q <= '0;
      write_q <= 1'b0;
      cancel_done_o <= 1'b0;
    end else begin
      cancel_done_o <= cancel_i;
      if (!inhibit_q[0]) cycle_q <= cycle_q + 64'd1;
      if (!inhibit_q[2]) instret_q <= instret_q + {62'b0, retired_count};
      if (completion_valid_o && completion_ready_i) response_valid_q <= 1'b0;
      if (commit_owner) begin
        pending_q <= 1'b0;
        if (owner_q.decoded.system_op == SYS_CSR && write_q) begin
          case (address_q)
            CSR_MSTATUS: begin mie_global_q <= write_value_q[3]; mpie_q <= write_value_q[7]; end
            CSR_MIE: mie_q <= write_value_q & IRQ_MASK;
            CSR_MTVEC: mtvec_q <= {write_value_q[31:2],
                                  (write_value_q[1:0] == 1 ? 2'b01 : 2'b00)};
            CSR_MSCRATCH: mscratch_q <= write_value_q;
            CSR_MEPC: mepc_q <= {write_value_q[31:2], 2'b00};
            CSR_MCAUSE: mcause_q <= write_value_q;
            CSR_MTVAL: mtval_q <= write_value_q;
            CSR_MCOUNTINHIBIT: inhibit_q <= write_value_q & 32'h5;
            // Either-half writes replace the counter's implicit update. Preserve
            // the other PRE-INCREMENT half, including on a low-half carry edge.
            CSR_MCYCLE: cycle_q[31:0] <= write_value_q;
            CSR_MCYCLEH: cycle_q[63:32] <= write_value_q;
            CSR_MINSTRET: instret_q[31:0] <= write_value_q;
            CSR_MINSTRETH: instret_q[63:32] <= write_value_q;
            // Fixed WARL values and hardware-only pending bits ignore writes.
            default: begin end
          endcase
        end
        if (owner_q.decoded.system_op == SYS_MRET) begin
          mie_global_q <= mpie_q;
          mpie_q <= 1'b1;
        end
        if (owner_q.decoded.system_op == SYS_WFI) waiting_q <= 1'b1;
      end
      if (|local_pending) waiting_q <= 1'b0;
      if (request_valid_i && request_ready_o) begin
        owner_q <= request_i.uop;
        result_q <= incoming_result;
        pending_q <= 1'b1;
        response_valid_q <= 1'b1;
        address_q <= request_i.uop.decoded.csr_address;
        write_value_q <= incoming_write_value;
        write_q <= incoming_write;
      end
      if (cancel_i || trap_enter_i) begin
        pending_q <= 1'b0;
        response_valid_q <= 1'b0;
        owner_q <= '0;
        result_q <= '0;
        write_q <= 1'b0;
      end
      if (trap_enter_i) begin
        mepc_q <= {trap_pc_i[31:2], 2'b00};
        mcause_q <= trap_cause_i;
        mtval_q <= trap_tval_i;
        mpie_q <= mie_global_q;
        mie_global_q <= 1'b0;
        waiting_q <= 1'b0;
      end
    end
  end
endmodule
