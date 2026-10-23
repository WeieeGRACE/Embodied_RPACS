`timescale 1ns/1ps
module tb_control_fifo;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,cv=0,step=0,fault=0,iv=0;
 reg [7:0] command=0,injected=0;
 wire [7:0] status,response,last_command;wire [1:0] gear;wire ev,pair;
 wire [31:0] debug_errors;
 asr_test_control control(.clk(clk),.rst_n(rst_n),.command_valid(cv),.command(command),
  .step(step),.fault(fault),.inject_valid(iv),.inject_status(injected),.status(status),
  .gear(gear),.event_valid(ev),.event_pair(pair),.response(response),
  .last_command(last_command),.debug_error_count(debug_errors));
 task check(input bit ok,input string msg);if(!ok)$fatal(1,"FAIL %s",msg);endtask
 task cmd(input [7:0] c,input [7:0] expected_ack,input [7:0] expected_state);
  @(negedge clk);command=c;cv=1;@(negedge clk);cv=0;
  check(ev && pair && response==expected_ack && status==expected_state,"command semantics");
 endtask
 task inject(input [7:0] s);@(negedge clk);iv=1;injected=s;@(negedge clk);iv=0;endtask
 // Independent small FIFO fixture, physically decoded TX.
 reg fev=0,fpair=0;reg [7:0] ack=0,state_code=0;
 wire tx;wire [31:0] overflow;
 asr_feedback #(.CLK_HZ(1000000),.BAUD(100000),.DEPTH(2)) feedback(
  .clk(clk),.rst_n(rst_n),.event_valid(fev),.event_pair(fpair),
  .response(ack),.status(state_code),.tx(tx),.overflow_count(overflow));
 reg [7:0] bytes_out[0:1023];integer received=0;reg [7:0] b;
 initial forever begin
  @(negedge tx);repeat(15) @(posedge clk);#1;
  for(integer i=0;i<8;i++) begin b[i]=tx;repeat(10) @(posedge clk);#1;end
  check(tx==1,"feedback stop bit");bytes_out[received]=b;received++;
 end
 task push(input bit p,input [7:0] a,input [7:0] s);
  @(negedge clk);fev=1;fpair=p;ack=a;state_code=s;@(negedge clk);fev=0;
 endtask
 task expect_frame(input integer offset,input [7:0] t,input [7:0] c);
  check(bytes_out[offset]==8'hAA && bytes_out[offset+1]==8'h55 && bytes_out[offset+2]==t &&
   bytes_out[offset+3]==c && bytes_out[offset+4]==(t+c) && bytes_out[offset+5]==13,"physical frame bytes/order");
 endtask
 reg completed=0;integer old;
 initial begin
  repeat(5) @(negedge clk);rst_n=1;repeat(5) @(negedge clk);
  check(status==0 && gear==1 && !ev,"reset idle medium gear, no report");
  cmd(1,0,1);cmd(1,1,1);cmd('h10,1,1);cmd(3,0,1);
  cmd('h14,0,1);check(gear==2,"speed up high");cmd('h14,0,1);check(gear==2,"high saturation");
  cmd('h15,0,1);cmd('h15,0,1);cmd('h15,0,1);check(gear==0,"low saturation");
  @(negedge clk);step=1;@(negedge clk);step=0;check(status==2 && ev && !pair,"step to approach");
  @(negedge clk);step=1;@(negedge clk);step=0;check(status==3,"injected arrival");
  for(integer c=16;c<=19;c++) begin cmd(c,0,'h10);cmd(c,1,'h10);cmd(2,0,4);end
  cmd('h10,0,'h10);@(negedge clk);step=1;@(negedge clk);step=0;check(status=='h11,"manual completion by key");
  for(integer s=0;s<=6;s++) begin inject(s);check(status==s,"all base status codes");end
  inject(0);check(status==6 && debug_errors==1,"fault cannot be cleared by injected state");
  cmd(1,2,6);cmd('h10,2,6);cmd('h14,2,6);cmd(2,0,6);cmd(3,0,6);
  @(negedge clk);rst_n=0;repeat(3) @(negedge clk);rst_n=1;
  inject('h10);inject('h11);check(status=='h11,"extended status injection");
  inject('hFF);check(status=='h11 && debug_errors==1,"unknown status rejected");
  // Fault wins over a simultaneous start, retaining the command response pair.
  @(negedge clk);fault=1;cv=1;command=1;@(negedge clk);fault=0;cv=0;
  check(status==6 && response==2 && ev && pair,"fault priority");
  push(1,0,1);wait(received==12);repeat(20) @(negedge clk);
  expect_frame(0,2,0);expect_frame(6,3,1);
  push(0,0,4);wait(received==18);repeat(20) @(negedge clk);expect_frame(12,3,4);
  old=received;
  // Active pair + two queued entries; remaining entries are dropped whole.
  push(1,0,'h10);push(1,1,2);push(0,0,4);
  for(integer i=0;i<5;i++) push(1,2,6);
  check(overflow==5,"FIFO full counts dropped events");
  wait(received==old+30);repeat(200) @(negedge clk);
  check(received==old+30,"dropped events not partially transmitted");
  expect_frame(old,2,0);expect_frame(old+6,3,'h10);
  expect_frame(old+12,2,1);expect_frame(old+18,3,2);expect_frame(old+24,3,4);
  // Reset aborts an in-flight response and flushes all pending events.
  push(1,0,1);repeat(8) @(negedge clk);rst_n=0;repeat(5) @(negedge clk);
  check(tx==1 && overflow==0,"reset clears busy TX and counters");
  $display("PASS control_fifo: command semantics, nine states, fault priority, atomic FIFO overflow and physical TX");
  completed=1;$finish;
 end
 initial begin #10000000;$fatal(1,"control_fifo watchdog");end
endmodule
