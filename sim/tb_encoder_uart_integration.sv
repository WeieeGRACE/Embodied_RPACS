`timescale 1ns/1ps
module tb_encoder_uart_integration;
 reg clk=0,rst=0; always #10 clk=~clk;
 reg completed=0;
 reg [1:0] left=0,right=0; wire tx;
 robot_encoder_test_top #(.REPORT_CYCLES(1000000)) dut(
 .clk_50m(clk),.reset_n(rst),.encoder_a_l(left[1]),.encoder_b_l(left[0]),
 .encoder_a_r(right[1]),.encoder_b_r(right[0]),.uart_tx(tx),
 .led_left_dir(),.led_right_dir(),.led_left_fault(),.led_right_fault(),.test_left_fault(),.test_right_fault());
 task step(input [1:0] l,input [1:0] r);
 begin @(negedge clk); left=l;right=r;repeat(10) @(posedge clk); end endtask
 string received,expected;
 task read_line;
 reg [7:0] b; integer i;
 begin received="";
  do begin @(negedge tx);repeat(217) @(posedge clk);
   for(i=0;i<8;i=i+1) begin repeat(434) @(posedge clk);#1;b[i]=tx;end
   repeat(434) @(posedge clk);#1;if(tx!==1) $fatal(1,"stop");received={received,b};
  end while(b!=10);
 end endtask
 initial begin
  #30;rst=1;repeat(20) @(posedge clk);
  // Mirrored raw phases: both wheels moving chassis-forward count positive.
  step(1,2);step(3,3);step(2,1);step(0,0);
  read_line();expected={"L=4 R=4 DL=1 DR=1 IL=0 IR=0 FL=0 FR=0",8'd13,8'd10};
  if(received!=expected) $fatal(1,"quadrature: %s",received);
  step(2,1);step(3,3);step(1,2);step(0,0);
  step(3,3); // Both channels make an illegal 00 -> 11 transition.
  read_line();expected={"L=0 R=0 DL=0 DR=0 IL=1 IR=1 FL=1 FR=1",8'd13,8'd10};
  if(received!=expected) $fatal(1,"fault: %s",received);
  @(negedge clk);rst=0;#1;if(tx!==1) $fatal(1,"reset");
  repeat(3) @(posedge clk); @(negedge clk);rst=1;
  read_line();expected={"L=0 R=0 DL=0 DR=0 IL=0 IR=0 FL=0 FR=0",8'd13,8'd10};
  if(received!=expected) $fatal(1,"clear: %s",received);
  completed=1;$display("PASS encoder AB -> signed UART counts, reverse, faults and reset");$finish;
 end
 initial begin #100000000;$fatal(1,"timeout");end
endmodule
