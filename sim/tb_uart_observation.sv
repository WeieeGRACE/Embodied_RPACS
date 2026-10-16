`timescale 1ns/1ps
module tb_uart_observation;
 reg clk=0,rst=0; always #10 clk=~clk;
 reg completed=0;
 reg [7:0] data=0; reg valid=0; wire ready,tx,done;
 uart_tx dut(clk,rst,data,valid,ready,tx,done);
 integer checked_cycles=0;
 always @(negedge clk) if(rst && dut.busy) begin
  if(dut.cycles>=434 || tx!==dut.frame[dut.bit_index]) $fatal(1,"bit cycle invariant");
  checked_cycles=checked_cycles+1;
 end
 task check_byte(input [7:0] expected);
 integer i;
 begin
  wait(!tx); repeat(217) @(posedge clk); #1;
  if(tx!==0) $fatal(1,"start");
  for(i=0;i<8;i=i+1) begin repeat(434) @(posedge clk); #1;
   if(tx!==expected[i]) $fatal(1,"data bit %0d",i); end
  repeat(434) @(posedge clk); #1; if(tx!==1) $fatal(1,"stop");
 end endtask
 wire tele_tx; reg signed [31:0] l=-2147483648,r=2147483647;
 reg [31:0] il=32'hffffffff,ir=0;
 encoder_uart_telemetry #(.REPORT_CYCLES(1000000)) tele(clk,rst,l,r,1'b1,1'b0,il,ir,1'b1,1'b0,tele_tx);
 string received,expected;
 task read_line;
 reg [7:0] b; integer i;
 begin received="";
  do begin
   @(negedge tele_tx); repeat(217) @(posedge clk);
   b=0; for(i=0;i<8;i=i+1) begin repeat(434) @(posedge clk); #1; b[i]=tele_tx; end
   repeat(434) @(posedge clk); #1; if(tele_tx!==1) $fatal(1,"tele stop");
   received={received,b};
  end while(b!=10);
 end endtask
 initial begin
  #1; if(tx!==1) $fatal(1,"reset idle"); #20; rst=1;
  fork
   begin check_byte(8'hA5); check_byte(8'h00); check_byte(8'hFF); end
   begin
    @(negedge clk); valid=1;data=8'hA5;
    @(negedge clk);valid=0;data=8'h00; wait(ready);
    @(negedge clk);valid=1; @(negedge clk);valid=0;data=8'hFF;wait(ready);
    @(negedge clk);valid=1;@(negedge clk);valid=0;
   end
  join
  wait(ready); @(negedge clk); valid=1; @(negedge clk);valid=0;
  repeat(20) @(posedge clk); rst=0; #1; if(tx!==1 || !ready) $fatal(1,"midbyte reset");
  @(negedge clk);rst=1;
  fork
   read_line();
   begin wait(tele.state==7); l=123;r=-456;il=0;ir=42; end
  join
  expected={"L=-2147483648 R=2147483647 DL=1 DR=0 IL=4294967295 IR=0 FL=1 FR=0",8'd13,8'd10};
  if(received!=expected) $fatal(1,"snapshot: %s",received);
  read_line(); expected={"L=123 R=-456 DL=1 DR=0 IL=0 IR=42 FL=1 FR=0",8'd13,8'd10};
  if(received!=expected) $fatal(1,"decimal: %s",received);
  completed=1; $display("PASS UART framing, reset, signed boundaries, snapshot and telemetry"); $finish;
 end
 initial begin #100000000; $fatal(1,"timeout"); end
endmodule

