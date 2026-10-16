`timescale 1ns/1ps
module tb_ground_watchdog;
 reg clk=0,rst=0,key=1,estop=1,rx=1;always #10 clk=~clk;
 wire a,a1,a2,b,b1,b2,st;reg completed=0;
 robot_closed_loop_test_top #(.TEST_BOTH(1),.WATCHDOG_ENABLE(1),.WATCHDOG_CYCLES(5000),.UART_BIT_CYCLES(20),
 .DEBOUNCE_CYCLES(8),.STARTUP_CYCLES(10),.RUN_CYCLES(30000),.WINDOW_CYCLES(100),.REPORT_CYCLES(100000)) dut(
 .clk_50m(clk),.reset_n(rst),.key_enable_n(key),.key_estop_n(estop),.uart_rx(rx),
 .encoder_a_l(1'b0),.encoder_b_l(1'b0),.encoder_a_r(1'b0),.encoder_b_r(1'b0),
 .PWMA(a),.AIN1(a1),.AIN2(a2),.PWMB(b),.BIN1(b1),.BIN2(b2),.STBY(st),.uart_tx(),
 .led_enable(),.led_estop(),.led_fault(),.led_saturated());
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task byte_send(input [7:0] data,input stop);integer i;begin
 @(negedge clk);rx=0;cycles(20);for(i=0;i<8;i=i+1)begin @(negedge clk);rx=data[i];cycles(20);end
 @(negedge clk);rx=stop;cycles(20);@(negedge clk);rx=1;cycles(30);end endtask
 task off;begin if({a,a1,a2,b,b1,b2,st}!==0)$fatal(1,"unsafe outputs");end endtask
 integer n;
 initial begin
 cycles(5);rst=1;cycles(90);key=0;cycles(50);off;
 byte_send(8'h00,1);if(dut.heartbeat_seen)$fatal(1,"wrong byte accepted");
 byte_send(8'hA5,0);if(dut.heartbeat_seen)$fatal(1,"bad stop accepted");
 byte_send(8'hA5,1);cycles(30);off;if(dut.run_request)$fatal(1,"held key started");
 key=1;cycles(20);key=0;cycles(30);if(!st)$fatal(1,"no start");
 repeat(3)begin cycles(1000);byte_send(8'hA5,1);if(!st || dut.watchdog_expired)$fatal(1,"healthy heartbeat");end
 wait(dut.wd.watchdog.age==100);byte_send(8'h00,1);
 n=0;while(!dut.watchdog_expired)begin cycles(1);n=n+1;if(n>5000)$fatal(1,"timeout absent");end
 if(dut.wd.watchdog.age!=4999)$fatal(1,"timeout boundary");cycles(5);off;
 if(dut.fault_code!=5)$fatal(1,"watchdog fault code");
 byte_send(8'hA5,1);key=1;cycles(20);key=0;cycles(30);off;
 rst=0;cycles(5);rst=1;cycles(80);byte_send(8'hA5,1);cycles(30);off;
 key=1;cycles(20);key=0;cycles(30);if(!st)$fatal(1,"reset rearm failed");
 key=1;#1;off;cycles(20);key=0;cycles(30);estop=0;#1;off;cycles(5);estop=1;cycles(20);off;
 completed=1;$display("PASS UART heartbeat framing, no-heartbeat inhibit, timeout boundary, both outputs, fault=5, latch, reset/rearm, disable and estop");$finish;
 end
 initial begin #2000000;$fatal(1,"test timeout");end
endmodule
