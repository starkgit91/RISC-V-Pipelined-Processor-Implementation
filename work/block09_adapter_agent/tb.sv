`timescale 1ns/1ps
module tb_adapter_smoke;
 import rv32_ooo_pkg::*;
 logic clk=0,rst=1,reqv,reqr,rspv,rspr,rspe,block_read,en,pending,error;
 word_t addr;
 fetch_row_t data,bram_data=0;
 logic[BRAM_FETCH_ADDR_W-1:0] ba;
 logic ref_valid=0,ref_error=0;
 fetch_row_t ref_data=0;
 word_t rng=32'h31415927;
 integer checks=0,failures=0,accepts=0,responses=0;
 logic expected_ready,fire,valid_address;
 rv32_instruction_adapter dut(.clk_i(clk),.rst_ni(rst),.request_valid_i(reqv),
 .request_ready_o(reqr),.request_address_i(addr),.response_valid_o(rspv),
 .response_ready_i(rspr),.response_data_o(data),.response_error_o(rspe),
 .bram_block_i(block_read),.bram_enable_o(en),.bram_address_o(ba),.bram_data_i(bram_data),
 .transaction_pending_o(pending),.protocol_error_o(error));
 function automatic fetch_row_t pattern(input integer line_index);
  word_t lo,hi;
  begin lo=32'h10000000+line_index*2;hi=lo+1;pattern={hi,lo};end
 endfunction
 always_ff @(posedge clk) if(en) bram_data<=pattern(ba);
 task automatic check(input logic condition,input string label_text);
  begin checks++; if(condition!==1'b1)begin failures++;$display("FAIL %s",label_text);end end
 endtask
 task automatic observe;
  begin
   expected_ready=rst&&!block_read&&(!ref_valid||rspr);
   valid_address=addr<65536&&addr%8==0;
   fire=reqv&&expected_ready;
   check(reqr==expected_ready,"request capacity");
   check(rspv==(rst&&ref_valid)&&pending==ref_valid,"response owner");
   check(rspe==(rst&&ref_valid&&ref_error),"response error");
   check(data==((rst&&ref_valid&&!ref_error)?ref_data:64'b0),"matching stable payload");
   check(en==(fire&&valid_address),"native read enable");
   check(ba==((addr/8)%8192),"byte-to-row address visible before enable");
   check(error==(fire&&addr%8!=0),"misalignment diagnostic");
  end
 endtask
 task automatic step;
  begin
   #1;observe;
   if(ref_valid&&rspr)begin ref_valid=0;responses++;end
   if(fire)begin ref_valid=1;ref_error=!valid_address;ref_data=pattern(addr/8);accepts++;end
   #1;clk=1;#1;clk=0;#1;observe;
  end
 endtask
 initial begin
  reqv=0;rspr=0;block_read=0;addr=0;#1;rst=0;#2;observe;rst=1;
  reqv=1;addr=0;step;reqv=0;
  repeat(3) step;
  block_read=1;rspr=1;reqv=1;addr=8;step;
  check(!rspv&&!pending&&!reqr,"blocked refill does not block old response drain");
  block_read=0;step;
  for(integer i=0;i<40;i++)begin addr=(i+2)*8;step;end
  reqv=0;step;
  addr=32'h0000fff8;reqv=1;step;
  addr=32'h00010000;step;
  addr=32'h0000fffc;step;
  addr=32'hffffffff;step;
  reqv=0;step;
  for(integer i=0;i<2000;i++)begin
   rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
   if(!(reqv&&!reqr))begin
    reqv=rng[0];
    case(rng[4:2])
     0: addr=rng&32'h0000fff8;
     1: addr=32'h0000fff8;
     2: addr=32'h00010000;
     3: addr=32'hffffffff;
     4: addr=32'h00000004;
     default: addr=rng&32'h0000fff8;
    endcase
   end
   block_read=rng[5]&&rng[6];rspr=rng[7];step;
  end
  reqv=0;rspr=1;block_read=0;step;
  reqv=1;addr=32;rspr=0;step;
  rst=0;ref_valid=0;ref_error=0;#2;observe;
  rst=1;reqv=0;step;
  $display("ADAPTER SMOKE: %0d checks, %0d failures; %0d accepts/%0d responses",
    checks,failures,accepts,responses);
  if(failures)$fatal(1,"adapter smoke failed");
  $finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
