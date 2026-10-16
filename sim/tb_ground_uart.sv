`timescale 1ns/1ps
module tb_ground_uart;
 reg clk=0,rst=0,completed=0;
 always #10 clk=~clk;
 reg [703:0] values=0;
 wire tx;
 closed_loop_uart_telemetry #(.REPORT_CYCLES(1000000),.FIELD_COUNT(22)) dut(clk,rst,values,tx);
 string received,expected;
 task read_line;
 reg [7:0] b;integer i;
 begin
  received="";
  do begin
   @(negedge tx);repeat(217) @(posedge clk);
   b=0;for(i=0;i<8;i=i+1) begin repeat(434) @(posedge clk);#1;b[i]=tx;end
   repeat(434) @(posedge clk);#1;if(tx!==1) $fatal(1,"stop bit");
   received={received,b};
  end while(b!=10);
 end endtask
 initial begin
  values[0+:32]=32'h80000000;values[32+:32]=32'h7fffffff;
  values[64+:32]=10;values[128+:32]=-12;values[192+:32]=-3;
  values[256+:32]=-625;values[320+:32]=1;values[384+:32]=1;values[416+:32]=1;
  values[448+:32]=32'hffffffff;values[480+:32]=42;
  values[512+:32]=1;values[576+:32]=1;values[608+:32]=3;values[640+:32]=1;values[672+:32]=1;
  repeat(5) @(negedge clk);rst=1;
  fork
   read_line();
   begin wait(dut.state==7);values=0;end
  join
  expected={"L=-2147483648 R=2147483647 TL=10 TR=0 VL=-12 VR=0 SFL=-3 SFR=0 PL=-625 PR=0 SL=1 SR=0 EN=1 ST=1 IL=4294967295 IR=42 FL=1 FR=0 FAULT=1 FC=3 HB=1 WD=1",8'd13,8'd10};
  if(received!=expected) $fatal(1,"atomic fields: %s",received);
  read_line();
  expected={"L=0 R=0 TL=0 TR=0 VL=0 VR=0 SFL=0 SFR=0 PL=0 PR=0 SL=0 SR=0 EN=0 ST=0 IL=0 IR=0 FL=0 FR=0 FAULT=0 FC=0 HB=0 WD=0",8'd13,8'd10};
  if(received!=expected) $fatal(1,"next snapshot: %s",received);
  completed=1;$display("PASS closed loop UART all fields, signed boundaries, CRLF and atomic snapshots");$finish;
 end
 initial begin #100000000;$fatal(1,"timeout");end
endmodule
