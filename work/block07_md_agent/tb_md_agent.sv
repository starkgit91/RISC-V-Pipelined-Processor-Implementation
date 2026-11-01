`timescale 1ns / 1ps
module tb_md_agent;
  import rv32_ooo_pkg::*;
  logic clk=0,rst_n=1,req_valid=0,req_ready,resp_valid,resp_ready=0;
  logic cancel=0,cancel_done,busy,quiescent,error;
  execute_packet_t req;
  completion_t resp,saved;
  word_t edge_values[0:9];
  integer tests=0;
  rv32_muldiv_execute dut(.clk_i(clk),.rst_ni(rst_n),.request_valid_i(req_valid),
    .request_ready_o(req_ready),.request_i(req),.completion_valid_o(resp_valid),
    .completion_ready_i(resp_ready),.completion_o(resp),.cancel_i(cancel),
    .cancel_done_o(cancel_done),.busy_o(busy),.quiescent_o(quiescent),.protocol_error_o(error));
  task automatic tick; begin #1;clk=1;#1;clk=0;#1;end endtask
  task automatic check(input logic cond,input string s);
    if(cond!==1'b1) $fatal(1,"%s at %0t op=%0d A=%h B=%h result=%h",s,$time,
                          req.uop.decoded.muldiv_op,req.rs1_value,req.rs2_value,resp.value);
  endtask
  function automatic muldiv_op_e operation(input integer n);
    case(n)
      0:operation=MD_MUL;1:operation=MD_MULH;2:operation=MD_MULHSU;3:operation=MD_MULHU;
      4:operation=MD_DIV;5:operation=MD_DIVU;6:operation=MD_REM;default:operation=MD_REMU;
    endcase
  endfunction
  function automatic word_t reference(input integer op,input word_t a,b);
    logic[63:0] unsigned_product;
    logic signed[63:0] sa,sb,sp,sq,sr;
    begin
      unsigned_product={32'b0,a}*{32'b0,b};
      sa={{32{a[31]}},a};sb={{32{b[31]}},b};
      if(op==2) sb={32'b0,b};
      sp=sa*sb;
      sq=0;sr=0;if(b!=0) begin sq=sa/sb;sr=sa%sb;end
      case(op)
        0:reference=unsigned_product[31:0];
        1,2:reference=sp[63:32];
        3:reference=unsigned_product[63:32];
        4:reference=b==0 ? 32'hffffffff :
                    (a==32'h80000000 && b==32'hffffffff ? a : sq[31:0]);
        5:reference=b==0 ? 32'hffffffff : a/b;
        6:reference=b==0 ? a :
                    (a==32'h80000000 && b==32'hffffffff ? 0 : sr[31:0]);
        7:reference=b==0 ? a : a%b;
      endcase
    end
  endfunction
  task automatic run(input integer op,input word_t a,b);
    word_t expected;
    execute_packet_t owner;
    begin
      check(req_ready && quiescent,"idle capacity");
      req='0;req.uop.decoded.op_class=UOP_MULDIV;req.uop.decoded.muldiv_op=operation(op);
      req.uop.decoded.writes_rd=1;req.uop.pdst=37;req.uop.rob_tag.index=11;
      req.uop.rob_tag.generation=7;req.uop.decoded.fetch.pc=32'hfffffff8;
      req.uop.decoded.fetch.prediction.next_pc=32'h12345678;
      req.rs1_value=a;req.rs2_value=b;owner=req;expected=reference(op,a,b);
      req_valid=1;tick;req_valid=0;req='0;
      check(busy&&!quiescent&&!req_ready&&!resp_valid,"request captured, no immediate result");
      for(integer j=0;j<31;j++) begin tick;check(!resp_valid,"no early result");end
      tick;check(resp_valid&&busy&&!req_ready&&!quiescent,"response after exactly32 steps");
      if(resp.value!==expected) $display("MISMATCH op=%0d a=%h b=%h expect=%h actual=%h",op,a,b,expected,resp.value);
      check(resp.value==expected,"behavioral arithmetic reference");
      check(resp.rob_tag==owner.uop.rob_tag && resp.pdst==37 && resp.writes_rd,
            "captured owner metadata");
      check(resp.control_flow.next_pc==32'hfffffffc && resp.control_flow.mispredicted
            && !resp.control_flow.is_control && !resp.exception.valid,"nextPC metadata");
      saved=resp;
      repeat(3) begin tick;check(resp_valid&&resp==saved,"held completion stability");end
      resp_ready=1;tick;resp_ready=0;check(!resp_valid&&req_ready&&quiescent,"response consumed");
      tests=tests+1;
    end
  endtask
  initial begin
    req='0;rst_n=0;#2;rst_n=1;#1;
    edge_values[0]=0;edge_values[1]=1;edge_values[2]=2;edge_values[3]=32'hffffffff;
    edge_values[4]=32'h80000000;edge_values[5]=32'h7fffffff;edge_values[6]=32'h80000001;
    edge_values[7]=32'hffff0000;edge_values[8]=32'h00010001;edge_values[9]=32'h55555555;
    for(integer op=0;op<8;op++) begin
      for(integer a=0;a<10;a++)
        for(integer b=0;b<10;b++) run(op,edge_values[a],edge_values[b]);
      for(integer n=0;n<300;n++) run(op,$random,$random);
    end
    req='0;req.uop.decoded.op_class=UOP_MULDIV;req.uop.decoded.muldiv_op=MD_DIV;
    req.uop.rob_tag.index=3;req.uop.pdst=44;req.uop.decoded.writes_rd=1;
    req.rs1_value=100;req.rs2_value=7;req_valid=1;tick;
    req='0;req.uop.decoded.op_class=UOP_INTEGER;#1;
    check(error&&!req_ready,"busy pulse diagnosed");tick;req_valid=0;
    repeat(31) tick;
    check(resp_valid&&resp.value==14&&resp.rob_tag.index==3&&resp.pdst==44,
          "busy pulse cannot overwrite current owner");
    resp_ready=1;tick;resp_ready=0;
    for(integer progress=0;progress<34;progress++) begin
      req='0;req.uop.decoded.op_class=UOP_MULDIV;req.uop.decoded.muldiv_op=MD_DIV;
      req.rs1_value=32'hffff8000;req.rs2_value=3;req_valid=1;tick;req_valid=0;
      repeat(progress) tick;
      cancel=1;#1;check(!resp_valid&&!req_ready,"cancel masks handshakes");tick;
      check(cancel_done&&quiescent&&!busy&&!resp_valid,"cancel clears running or held owner");
      cancel=0;repeat(35) begin tick;check(!resp_valid,"no delayed canceled response");end
    end
    req='0;req.uop.decoded.op_class=UOP_INTEGER;req.uop.decoded.fetch.instruction=32'h55;
    req.uop.decoded.writes_rd=1;req.uop.pdst=42;req_valid=1;#1;check(error,"bad class diagnosed");
    tick;req_valid=0;check(resp_valid&&resp.exception.valid&&resp.exception.cause==2
      &&resp.exception.tval==32'h55&&resp.writes_rd&&resp.pdst==42,"bad class traps owned token");
    resp_ready=1;tick;resp_ready=0;
    req='0;req.uop.decoded.op_class=UOP_MULDIV;req.uop.decoded.fetch.exception.valid=1;
    req.uop.decoded.fetch.exception.cause=1;req.uop.decoded.fetch.exception.tval=32'h8899;
    req_valid=1;tick;req_valid=0;check(resp_valid&&resp.exception.cause==1
      &&resp.exception.tval==32'h8899&&!resp.control_flow.mispredicted,"inherited exception");
    $display("MD_AGENT: %0d arithmetic cases plus 34 cancellation boundaries PASS",tests);
    $finish;
  end
  initial begin #10000000;$fatal(1,"timeout");end
endmodule
