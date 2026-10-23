`timescale 1ns/1ps
module tb_asr_units;
 reg clk=0,rst=0,rx=1;
 always #5 clk=~clk;
 wire cv;wire [7:0] command;
 reg completed=0;
 asr_uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) dut(.clk(clk),.rst_n(rst),.rx(rx),.command_valid(cv),.command(command));
 integer received=0;reg [7:0] last;
 always @(posedge clk)if(cv)begin received=received+1;last=command;end
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task send(input [7:0] b,input bit stop=1);begin
 @(negedge clk);rx=0;cycles(10);
 for(integer i=0;i<8;i++)begin @(negedge clk);rx=b[i];cycles(10);end
 @(negedge clk);rx=stop;cycles(10);@(negedge clk);rx=1;cycles(5);
 end endtask
 task frame(input [7:0] c);begin send('haa);send('h55);send(1);send(c);send(c+1);send(13);cycles(10);end endtask
 task expect_count(input integer n);if(received!=n)$fatal(1,"RX expected %0d got %0d last=%h",n,received,last);endtask
 reg signed [1:0] delta=0;
 wire signed [31:0] position;
 asr_distance distance_dut(.clk(clk),.rst_n(rst),.delta(delta),.position_um(position));
 reg update=0;reg signed [31:0] wrapped=0;wire signed [31:0] yaw;wire valid;
 asr_yaw_unwrap unwrap(.clk(clk),.rst_n(rst),.update_valid(update),.wrapped(wrapped),.unwrapped(yaw),.valid(valid));
 wire signed [31:0] mounted_yaw;wire mounted_valid;
 asr_yaw_unwrap #(.YAW_SIGN(-1)) mounted_unwrap(.clk(clk),.rst_n(rst),.update_valid(update),.wrapped(wrapped),.unwrapped(mounted_yaw),.valid(mounted_valid));
 task angle(input integer v);begin @(negedge clk);wrapped=v;update=1;cycles(1);@(negedge clk);update=0;cycles(1);end endtask
 reg pair=0,event_valid=0;reg [7:0] response=0,pair_status=0,event_status=0;
 wire tx,overflow;
 asr_packet_tx #(.CLK_HZ(1000000),.BAUD(100000)) packets(
 .clk(clk),.rst_n(rst),.pair_valid(pair),.response(response),.pair_status(pair_status),
 .event_valid(event_valid),.event_status(event_status),.tx(tx),.overflow(overflow));
 reg overflow_reported=0;
 integer frames=0,index=0;reg [7:0] bytes[0:5],b;
 always @(negedge tx)begin
 if(rst)begin
 repeat(15)@(negedge clk);
 for(integer i=0;i<8;i++)begin b[i]=tx;repeat(10)@(negedge clk);end
 if(tx!==1)$fatal(1,"TX bad stop");
 bytes[index]=b;
 if(index==5)begin
 if(bytes[0]!='haa || bytes[1]!='h55 || bytes[4]!=((bytes[2]+bytes[3])&255) || b!=13)$fatal(1,"TX framing");
 if(frames==0 && {bytes[2],bytes[3]}!=16'h0200)$fatal(1,"response ordering");
 if(frames==1 && {bytes[2],bytes[3]}!=16'h0310)$fatal(1,"pair status ordering");
 if(frames==2 && {bytes[2],bytes[3]}!=16'h0311)$fatal(1,"event ordering");
 if({bytes[2],bytes[3]}==16'h0306)overflow_reported=1;
 frames++;index=0;
 end else index++;
 end
 end
 initial begin
 cycles(4);rst=1;cycles(5);
 frame('h10);expect_count(1);
 send('haa);frame('h12);expect_count(2); // overlapping AA
 send('h77);send('h55);frame('h13);expect_count(3);
 send('haa);send('h55);send(1);send('h10);send('hff);send(13);expect_count(3);
 send('haa);send('h55);send(1);send('h10);send('h11);send(12);expect_count(3);
 send('haa);send('h55);cycles(20001);send(1);send('h10);send('h11);send(13);expect_count(3);
 send('haa);send('h55);send(1);send('h10);frame('h02);expect_count(4); // embedded header
 send('haa,0);send('h55);send(1);send(3);send(4);send(13);expect_count(4);
 frame(3);frame('h11);frame('h14);frame('h15);frame(1);expect_count(9);
 frame('h16);expect_count(9);
 @(negedge clk);delta=1;cycles(1456);@(negedge clk);delta=0;cycles(2);
 if(position<207344 || position>207346)$fatal(1,"distance fractional accumulation %0d",position);
 @(negedge clk);delta=-1;cycles(1456);@(negedge clk);delta=0;cycles(2);
 if(position!=0)$fatal(1,"reverse distance residual");
 angle(170000);angle(179000);angle(-179000);angle(-100000);
 if(yaw!=90000)$fatal(1,"positive wrap %0d",yaw);
 if(mounted_yaw!= -90000)$fatal(1,"mounted right wrap %0d",mounted_yaw);
 angle(-179000);angle(179000);angle(170000);
 if(yaw!=0)$fatal(1,"negative wrap %0d",yaw);
 if(mounted_yaw!=0)$fatal(1,"mounted left wrap %0d",mounted_yaw);
 cycles(20);if(valid || mounted_valid)$fatal(1,"valid held high on stale yaw");
 @(negedge clk);pair=1;pair_status='h10;event_valid=1;event_status='h11;
 cycles(1);@(negedge clk);pair=0;event_valid=0;cycles(2100);
 if(frames!=3 || overflow)$fatal(1,"pair plus event queue");
 // Producer overload must become a latched safety fault, never silent wrap.
 @(negedge clk);pair=1;cycles(40);@(negedge clk);pair=0;cycles(10);
 if(!overflow)$fatal(1,"queue overflow not latched");
 cycles(50000);if(!overflow_reported)$fatal(1,"overflow fault not eventually reported");
 $display("PASS protocol framing/recovery/timeout, Q16 distance, both yaw wraps, TX framing/concurrency/overflow");
 completed=1;$finish;
 end
 initial begin #20000000;$fatal(1,"unit watchdog");end
endmodule
