`timescale 1ns/1ps
module tb_rename_smoke;
import rv32_ooo_pkg::*;
logic clk=0, rst=1, valid=0;
logic ready, sr1, sr2, done;
decoded_uop_t dec;
rob_tag_t tag;
renamed_uop_t renamed, retired;
completion_t comp;
phys_reg_t committed;
phys_mask_t freebits, readybits;
phys_count_t count;
always #10 clk=~clk;
rv32_rename dut(.clk_i(clk),.rst_ni(rst),.dispatch_valid_i(valid),
.dispatch_resources_ready_i(1'b1),.decoded_i(dec),.dispatch_rob_tag_i(tag),
.dispatch_ready_o(ready),.renamed_o(renamed),.src1_ready_o(sr1),.src2_ready_o(sr2),
.completion_valid_i(1'b0),.completion_i(comp),.retire_valid_i(1'b0),.retire_uop_i(retired),
.recover_i(1'b0),.recover_done_o(done),.committed_query_arch_i(5'd5),
.committed_query_phys_o(committed),.free_mask_o(freebits),.ready_mask_o(readybits),
.free_count_o(count));
initial begin
 dec='0;tag='0;comp='0;retired='0;
 #1;rst=0;#1;
 if(count!==32)$fatal(1,"initial free count");
 @(negedge clk);rst=1;dec.op_class=UOP_INTEGER;dec.writes_rd=1;dec.rd=5;
 dec.uses_rs1=1;dec.rs1=5;valid=1;
 #1;if(renamed.pdst!==32||renamed.prs1!==5)$fatal(1,"first preview");
 @(posedge clk);#1;
 if(renamed.pdst!==33||renamed.prs1!==32||count!==31)$fatal(1,"first allocation");
 @(posedge clk);#1;valid=0;#1;
 if(count!==30||committed!==5)$fatal(1,"second allocation");
 $display("RENAME SMOKE PASS");$finish;
end
initial begin #200;$fatal(1,"timeout");end
endmodule
