`timescale 1ns/1ps
module tb_closed_loop_test #(parameter integer TEST_TARGET=10);
 reg clk=0,rst=0,key=0,estop=1;always #10 clk=~clk;
 reg [1:0] ab=0;wire a,a1,a2,b,b1,b2,stby,tx;
 reg completed=0;
 robot_closed_loop_test_top #(.TARGET(TEST_TARGET),.WINDOW_CYCLES(10000),.FILTER_LENGTH(4),.DEBOUNCE_CYCLES(8),
 .RUN_CYCLES(500000),.STARTUP_CYCLES(10),.REPORT_CYCLES(1000000)) dut(
 .clk_50m(clk),.reset_n(rst),.key_enable_n(key),.key_estop_n(estop),
 .encoder_a_l(ab[1]),.encoder_b_l(ab[0]),.encoder_a_r(1'b0),.encoder_b_r(1'b0),
 .PWMA(a),.AIN1(a1),.AIN2(a2),.PWMB(b),.BIN1(b1),.BIN2(b2),.STBY(stby),.uart_tx(tx),
 .led_enable(),.led_estop(),.led_fault(),.led_saturated());
 task tick;begin @(posedge clk);#1;end endtask
 task off;begin #1;if({a,a1,a2,b,b1,b2,stby}!==0) $fatal(1,"safety off");end endtask
 task reset;
 begin @(negedge clk);rst=0;key=1;estop=1;ab=0;repeat(5) tick;rst=1;repeat(20) tick;end endtask
 task start;
 begin @(negedge clk);key=0;wait(dut.run_request);wait(stby);@(negedge clk);key=1;end endtask
 integer cycles=0,high_count=0,phase_count=0;
 integer previous_ramp=0,period_duty=0,pwm_periods=0;
 reg was_enabled=0,period_complete=0,period_drive=0;
 // Check full 2500-clock PWM periods at the physical output. Skip periods
 // containing direction deadtime, but still enforce the 625-clock ceiling.
 always @(negedge clk) begin
  if(dut.control_enable && was_enabled &&
     ($signed(dut.ramp_l)-previous_ramp>1 || previous_ramp-$signed(dut.ramp_l)>1))
   $fatal(1,"ramp exceeds one tick per update");
  previous_ramp=$signed(dut.ramp_l);was_enabled=dut.control_enable;
  if(!stby) begin period_complete=0;high_count=0;end
  else begin
   if(dut.driver.phase==0) begin
    if(period_complete) begin
     if(high_count>625) $fatal(1,"physical PWM above 25 percent");
     if(period_drive && high_count!=period_duty) $fatal(1,"PWM pulse width");
     pwm_periods=pwm_periods+1;
    end
    period_complete=1;high_count=0;
    period_duty=$signed(dut.driver.command_l);
    if(period_duty<0) period_duty=-period_duty;
    period_drive=1;
   end
   if(dut.driver.left_channel.state!=1) period_drive=0;
   if(a) high_count=high_count+1;
  end
 end
 reg signed [15:0] old_control;reg sample;
 reg signed [31:0] sample_speed;
 always @(posedge clk) begin
  if(rst && dut.rst_n) begin
   old_control=dut.control_l;sample=dut.valid_l && dut.control_enable;sample_speed=dut.speed_l;
   #1;
   if({b,b1,b2}!==0 || dut.control_r!==0 || dut.ramp_r!==0) $fatal(1,"right channel enabled");
   if(dut.ramp_l>625 || dut.ramp_l< -625 || dut.safe_l>625 || dut.safe_l< -625) $fatal(1,"limit");
   if(sample && sample_speed==0 && old_control==0 && dut.control_l!=33*TEST_TARGET) $fatal(1,"PI response or feedback order");
   if(a && (a1!==!dut.driver.command_l[15] || a2!==dut.driver.command_l[15])) $fatal(1,"direction pins disagree with signed command");
  end
 end
 // Simple deterministic plant: each 16 duty ticks produces one count/window,
 // with a 1-window first-order lag. Drive actual AB inputs, not feedback nets.
 reg model_on=0;integer velocity=0,desired=0,model_phase=0,edge_budget=0,model_cycles=0;
 always @(negedge clk) if(model_on) begin
  if(model_cycles==0) begin
   desired=$signed(dut.ramp_l)/16;
   velocity=(velocity+desired)/2;
   edge_budget=velocity<0 ? -velocity:velocity;
  end
  if(edge_budget>0 && model_cycles%500==0) begin
   if(velocity>=0) case(ab) 0:ab=1;1:ab=3;3:ab=2;2:ab=0;endcase
   else case(ab) 0:ab=2;2:ab=3;3:ab=1;1:ab=0;endcase
   edge_budget=edge_budget-1;
  end
  model_cycles=(model_cycles+1)%10000;
 end
 integer initial_control;
 initial begin
  #1;rst=0;repeat(5) tick;rst=1;repeat(50) tick;
  if(dut.run_request!==0) $fatal(1,"boot held must not run");
  @(negedge clk);key=1;repeat(20) tick;
  // Press bounce must not arm a run.
  key=0;repeat(2) tick;key=1;repeat(20) tick;
  if(dut.run_request) $fatal(1,"bounce");
  start();wait(dut.control_l==33*TEST_TARGET);
  if(dut.saturate16(32'sd40000)!==16'sh7fff || dut.saturate16(-32'sd40000)!==16'sh8000) $fatal(1,"saturation conversion");
  initial_control=dut.control_l;model_on=1;
  // PI accumulates while the deliberately slow ramp catches up: allow most
  // of the run interval, then verify reduced speed error rather than assuming
  // an integral controller must always output less than its first command.
  repeat(450000) tick;
  if((TEST_TARGET>0 && (dut.pos_l<=0 || dut.speed_l<5 || dut.speed_l>=15)) ||
     (TEST_TARGET<0 && (dut.pos_l>=0 || dut.speed_l> -5 || dut.speed_l<= -15)))
   $fatal(1,"plant did not reduce speed error: speed=%0d command=%0d",dut.speed_l,dut.control_l);
  model_on=0;
  if(!dut.run_request || !stby) $fatal(1,"release stopped action");
  while(dut.run_request) tick();off();tick();
  if(dut.control_l!==0 || dut.ramp_l!==0) $fatal(1,"disable clears state");
  repeat(20) tick;start();
  wait(dut.control_l!=0);@(negedge clk);#2;estop=0;off();#2;estop=1;
  repeat(20) tick;if(!dut.local_fault || stby) $fatal(1,"short estop not latched");
  reset();start();
  // Reverse raw phases must generate a negative signed speed.
  repeat(3) begin
   @(negedge clk);ab=2;repeat(120) tick;@(negedge clk);ab=3;repeat(120) tick;
   @(negedge clk);ab=1;repeat(120) tick;@(negedge clk);ab=0;repeat(120) tick;
  end
  wait(dut.valid_l);#1;if(dut.speed_l>=0) $fatal(1,"signed delta");
  // Illegal jump must disable driver as soon as decoder raises fault.
  @(negedge clk);ab=3;fork begin wait(dut.enc_fault_l); end #5000; join_any disable fork;off();repeat(10) tick;
  if(!dut.local_fault || stby) $fatal(1,"encoder fault latch");
  reset();start();cycles=0;
  while(dut.run_request) begin tick();cycles=cycles+1;end
  if(cycles>500000 || cycles<499980) $fatal(1,"run duration %0d",cycles);
  off();repeat(100) tick;if(dut.run_request) $fatal(1,"held retrigger");
  repeat(20) tick;start();if(!stby) $fatal(1,"rearm");
  @(negedge clk);rst=0;off();
  if(pwm_periods<10) $fatal(1,"insufficient PWM coverage");
  completed=1;$display("PASS closed loop plant, 4-window filter, signed feedback, physical PWM width/limit, ramp, unused wheel, bounce, estop, fault, timeout and reset");$finish;
 end
 initial begin #30000000;$fatal(1,"timeout");end
endmodule
