`timescale 1ns/1ps
module encoder_case #(parameter N=1, IL=0, IR=0)(output reg done);
reg clk=0,rst_n=0,cl=0,cr=0;
always #10 clk=~clk;
reg [1:0] l=0,r=0;
wire signed [31:0] pl,pr;
wire signed [1:0] dl,dr;
wire vl,vr,fl,fr;
wire [31:0] el,er;
dual_ab_quadrature_encoder #(.FILTER_CYCLES(N),.INVERT_L(IL),.INVERT_R(IR)) dut(
.clk(clk),.rst_n(rst_n),.encoder_a_l(l[1]),.encoder_b_l(l[0]),.encoder_a_r(r[1]),.encoder_b_r(r[0]),
.clear_fault_l(cl),.clear_fault_r(cr),.position_l(pl),.position_r(pr),.delta_count_l(dl),.delta_count_r(dr),
.direction_l(vl),.direction_r(vr),.illegal_transition_l(el),.illegal_transition_r(er),.encoder_fault_l(fl),.encoder_fault_r(fr));
reg [1:0] q0[0:1],q1[0:1],last[0:1],prev[0:1];
integer age,runs[0:1],c,diff,d,checks,seed,i,j,k,ed[0:1];
reg ready[0:1],md[0:1],mf[0:1];
reg signed [31:0] mp[0:1];
reg [31:0] me[0:1];
reg [1:0] sample;
reg clr;
function integer phase;
input [1:0] ab;
begin case(ab) 0:phase=0;2:phase=1;3:phase=2;1:phase=3;endcase end
endfunction
function [1:0] fwd;
input [1:0] ab;
begin case(ab) 0:fwd=2;2:fwd=3;3:fwd=1;1:fwd=0;endcase end
endfunction
function [1:0] rev;
input [1:0] ab;
begin case(ab) 0:rev=1;1:rev=3;3:rev=2;2:rev=0;endcase end
endfunction
task fail;
input [255:0] reason;
begin $display("FAIL N=%0d t=%0t %0s L=%0d expected=%0d errors=%0d expected=%0d",N,$time,reason,pl,mp[0],el,me[0]);$fatal(1);end
endtask
// Reference uses run lengths and phase arithmetic rather than the RTL transition table.
always @(posedge clk or negedge rst_n) begin
if(!rst_n) begin
age=0;
for(c=0;c<2;c=c+1) begin
q0[c]=0;q1[c]=0;last[c]=0;prev[c]=0;runs[c]=0;ready[c]=0;md[c]=0;mf[c]=0;mp[c]=0;me[c]=0;ed[c]=0;
end
end else begin
for(c=0;c<2;c=c+1) begin
ed[c]=0;clr=(c==0)?cl:cr;
if(clr) begin me[c]=0;mf[c]=0;end
sample=q1[c];
if(age>=2) begin
if(runs[c]==0 || sample!=last[c]) runs[c]=1;
else if(runs[c]<N) runs[c]=runs[c]+1;
last[c]=sample;
if(runs[c]>=N) begin
if(!ready[c]) ready[c]=1;
else begin
diff=(phase(sample)-phase(prev[c])+4)%4;d=0;
if(diff==1)d=1;if(diff==3)d=-1;
if((c==0 && IL!=0)||(c==1 && IR!=0))d=-d;
if(diff==2)begin mf[c]=1;if(me[c]!=32'hffffffff)me[c]=me[c]+1'b1;end
if(d!=0)begin mp[c]=mp[c]+d;md[c]=(d>0);ed[c]=d;end
end
prev[c]=sample;
end
end
q1[c]=q0[c];q0[c]=(c==0)?l:r;
end
age=age+1;
end
#1;
if(pl!==mp[0] || pr!==mp[1])fail("position");
if($signed(dl)!==ed[0] || $signed(dr)!==ed[1])fail("delta");
if(vl!==md[0] || vr!==md[1])fail("direction");
if(el!==me[0] || er!==me[1])fail("illegal count");
if(fl!==mf[0] || fr!==mf[1])fail("fault");
checks=checks+10;
end
task settle;
begin repeat(N+5)@(negedge clk);end
endtask
task reset_at;
input [1:0] a,b;
begin
@(negedge clk);#3;rst_n=0;l=a;r=b;cl=0;cr=0;
#2;if(pl!==0 || pr!==0 || fl!==0 || fr!==0)fail("async reset");
repeat(3)@(negedge clk);#3;rst_n=1;settle;
if(pl!=0 || pr!=0 || el!=0 || er!=0)fail("startup phantom");
end
endtask
initial begin
done=0;checks=0;seed=32'h417381+N+IL*4+IR*8;
#2;rst_n=1;#2;rst_n=0;
for(i=0;i<4;i=i+1)reset_at(i,3-i);
for(i=0;i<4;i=i+1)for(j=0;j<4;j=j+1)begin reset_at(i,i);#7;l=j;r=j;settle;end
reset_at(0,0);
for(i=0;i<24;i=i+1)begin #3;l=fwd(l);r=rev(r);settle;end
for(i=0;i<24;i=i+1)begin #7;l=rev(l);r=fwd(r);settle;end
repeat(20)@(negedge clk);
for(i=0;i<8;i=i+1)begin l=fwd(l);settle;end
for(i=0;i<8;i=i+1)begin r=rev(r);settle;end
reset_at(0,0);l=3;settle;
if(!fl || fr)fail("fault isolation");
l=fwd(l);settle;cl=1;settle;cl=0;settle;
if(fl || el!=0)fail("clear");
// Input changes just after negedge: acceptance occurs on rising edge N+2.
l=l^3;repeat(N+1)@(negedge clk);cl=1;@(negedge clk);cl=0;settle;
if(el!=1 || !fl)fail("clear collision");
@(negedge clk);dut.left_channel.illegal_transition=32'hfffffffe;me[0]=32'hfffffffe;
l=l^3;settle;l=l^3;settle;
if(el!=32'hffffffff)fail("saturation");
reset_at(0,0);
@(negedge clk);dut.left_channel.position=32'h7fffffff;mp[0]=32'h7fffffff;
l=IL?rev(l):fwd(l);settle;if(pl!==32'h80000000)fail("positive wrap");
l=IL?fwd(l):rev(l);settle;if(pl!==32'h7fffffff)fail("negative wrap");
reset_at(3,1);l=fwd(l);#7;rst_n=0;#3;if(pl!==0 || pr!==0)fail("moving reset");
reset_at(0,0);
if(N>1)begin
l=2;repeat(N-1)@(negedge clk);l=0;settle;if(pl!=0)fail("short glitch");
l=2;repeat(N)@(negedge clk);l=0;settle;if(pl!=0 || el!=0)fail("threshold");
end
@(negedge clk);#2;l=2;#3;l=0;settle;if(pl!=0)fail("subclock glitch");
for(i=0;i<500;i=i+1)begin
k=$random(seed);k=k&32'h7fffffff;
#(1+k%17);l=k%4;r=(k/4)%4;cl=(k%31==0);cr=(k%43==0);
repeat(1+k%(N+5))@(negedge clk);
end
cl=0;cr=0;settle;reset_at(0,0);
for(i=0;i<1456;i=i+1)begin l=fwd(l);r=rev(r);settle;end
if(pl!=(IL?-1456:1456)||pr!=(IR?1456:-1456))fail("1456 counts/rev");
repeat(30)@(negedge clk);
$display("PASS N=%0d IL=%0d IR=%0d checks=%0d end=%0t",N,IL,IR,checks,$time);done=1;
end
endmodule
module tb_dual_ab_quadrature_encoder;
wire d1,d2,d3,d4;
reg completed=0;
encoder_case #(.N(1))t1(d1);
encoder_case #(.N(2),.IL(1))t2(d2);
encoder_case #(.N(3),.IR(1))t3(d3);
encoder_case #(.N(4),.IL(1),.IR(1))t4(d4);
initial begin wait(d1&&d2&&d3&&d4);completed=1;$display("ALL ENCODER TESTS PASSED");$finish;end
initial begin #10000000;$display("FAIL timeout");$fatal(1);end
endmodule
