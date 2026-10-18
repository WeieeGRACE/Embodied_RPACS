`timescale 1ns/1ps
module tb_ground_motion #(parameter integer LEFT_TARGET=-10,RIGHT_TARGET=10);
 reg clk=0,rst=0,key=1,estop=1; always #10 clk=~clk;
 reg [1:0] abl=0,abr=0; wire a,a1,a2,b,b1,b2,st,tx;
 reg completed=0,model_on=0;
 robot_closed_loop_test_top #(.TARGET_L(LEFT_TARGET),.TARGET_R(RIGHT_TARGET),.TEST_BOTH(1),.WINDOW_CYCLES(10000),.DEBOUNCE_CYCLES(8),
 .RUN_CYCLES(500000),.STARTUP_CYCLES(10),.REPORT_CYCLES(1000000)) dut(
 .uart_rx(1'b1),.clk_50m(clk),.reset_n(rst),.key_enable_n(key),.key_estop_n(estop),
 .encoder_a_l(abl[1]),.encoder_b_l(abl[0]),.encoder_a_r(abr[1]),.encoder_b_r(abr[0]),
 .PWMA(a),.AIN1(a1),.AIN2(a2),.PWMB(b),.BIN1(b1),.BIN2(b2),.STBY(st),.uart_tx(tx),
 .led_enable(),.led_estop(),.led_fault(),.led_saturated());
 task tick; begin @(posedge clk);#1;end endtask
 task off; input [255:0] stage; begin #1;if({a,a1,a2,b,b1,b2,st}!==0) $fatal(1,"dual safety off stage=%0s outputs=%b control=%b safety=%b key=%b estop=%b fault=%b enc=%b/%b",stage,{a,a1,a2,b,b1,b2,st},dut.control_enable,dut.safety_active,key,estop,dut.fault_latched,dut.enc_fault_l,dut.enc_fault_r);end endtask
 task reset;begin @(negedge clk);rst=0;key=1;estop=1;abl=0;abr=0;repeat(5)tick;rst=1;repeat(80)tick;end endtask
 task start;begin @(negedge clk);key=0;wait(dut.run_request);wait(st);end endtask
 integer mc=0,vl=0,vr=0,el=0,er=0;
 always @(negedge clk) if(model_on) begin
  if(mc==0) begin
   vl=(vl+$signed(dut.ramp_l)/16)/2;vr=(vr+$signed(dut.ramp_r)/20)/2;
   el=vl<0 ? -vl:vl;er=vr<0 ? -vr:vr;
  end
  if(mc%500==0) begin
   if(el>0) begin
    if(vl>=0) case(abl) 0:abl=1;1:abl=3;3:abl=2;2:abl=0;endcase
    else case(abl) 0:abl=2;2:abl=3;3:abl=1;1:abl=0;endcase
    el=el-1;
   end
   if(er>0) begin
    if(vr>=0) case(abr) 0:abr=2;2:abr=3;3:abr=1;1:abr=0;endcase
    else case(abr) 0:abr=1;1:abr=3;3:abr=2;2:abr=0;endcase
    er=er-1;
   end
  end
  mc=(mc+1)%10000;
 end
 integer hl=0,hr=0,dl=0,dr=0,periods=0;reg full=0,drive_l=0,drive_r=0;
 always @(negedge clk) begin
  if(dut.ramp_l>625 || dut.ramp_l< -625 || dut.ramp_r>625 || dut.ramp_r< -625) $fatal(1,"dual limit");
  if(!st) begin full=0;hl=0;hr=0;end
  else begin
   if(dut.driver.phase==0) begin
    if(full) begin
     if(hl>625 || hr>625 || (drive_l && hl!=dl) || (drive_r && hr!=dr)) $fatal(1,"dual physical PWM width");
     periods=periods+1;
    end
    full=1;hl=0;hr=0;dl=$signed(dut.driver.command_l);dr=$signed(dut.driver.command_r);
    if(dl<0)dl=-dl;if(dr<0)dr=-dr;drive_l=1;drive_r=1;
   end
   if(dut.driver.left_channel.state!=1)drive_l=0;
   if(dut.driver.right_channel.state!=1)drive_r=0;
   if(a)begin hl=hl+1;if(a1!==!dut.driver.command_l[15] || a2!==dut.driver.command_l[15])$fatal(1,"left direction");end
   if(b)begin hr=hr+1;if(b1!==!dut.driver.command_r[15] || b2!==dut.driver.command_r[15])$fatal(1,"right direction");end
  end
 end
 integer count;
 initial begin
  repeat(5)tick;rst=1;repeat(30)tick;if(dut.run_request)$fatal(1,"held boot");
  key=0;repeat(10)tick;if(!dut.run_request)$fatal(1,"single press did not trigger");
  key=1;model_on=1;repeat(5000)tick;if(!dut.run_request || !st)$fatal(1,"release stopped action");
  key=0;repeat(20)tick;if(!dut.run_request || dut.run_count<5000)$fatal(1,"re-press changed active action");key=1;
  repeat(445000)tick;
  if((LEFT_TARGET>0 && dut.pos_l<=0) || (LEFT_TARGET<0 && dut.pos_l>=0) ||
     (RIGHT_TARGET>0 && dut.pos_r<=0) || (RIGHT_TARGET<0 && dut.pos_r>=0) ||
     $signed(dut.speed_l)<LEFT_TARGET-3 || $signed(dut.speed_l)>LEFT_TARGET+3 ||
     $signed(dut.speed_r)<RIGHT_TARGET-3 || $signed(dut.speed_r)>RIGHT_TARGET+3)
   $fatal(1,"ground feedback %d %d",dut.speed_l,dut.speed_r);
  if(dut.control_l==dut.control_r)$fatal(1,"independent plant control");
  if(!dut.run_request || !st)$fatal(1,"action stopped before timer");
  count=0;while(dut.run_request)begin tick();count=count+1;end model_on=0;
  if(count>50000 || count<40000)$fatal(1,"auto-stop remainder %0d",count);
  #1;off("8-second auto stop");tick();
  if(dut.control_l!==0 || dut.control_r!==0 || dut.ramp_l!==0 || dut.ramp_r!==0)$fatal(1,"dual clear");
  repeat(20)tick;if(dut.run_request)$fatal(1,"action rearmed while key held");
  key=0;repeat(10)tick;if(!dut.run_request)$fatal(1,"second press did not rearm");key=1;#2;estop=0;off("estop");#2;estop=1;repeat(20)tick;
  if(!dut.local_fault || st)$fatal(1,"dual estop latch");
  reset();key=0;repeat(10)tick;if(!dut.run_request)$fatal(1,"encoder test trigger");key=1;model_on=0;@(negedge clk);abl=3;repeat(120)tick;
  if(!dut.enc_fault_l)$fatal(1,"left encoder fault injection not detected: pin=%b sync=%b candidate=%b stable=%0d previous=%b",abl,dut.enc.left_channel.ab_sync,dut.enc.left_channel.candidate,dut.enc.left_channel.stable_count,dut.enc.left_channel.previous);
  #1;off("left encoder fault");repeat(120)tick;
  if(!dut.local_fault)$fatal(1,"left fault latch");
  reset();key=0;repeat(10)tick;if(!dut.run_request)$fatal(1,"encoder test trigger");key=1;model_on=0;@(negedge clk);abr=3;repeat(120)tick;
  if(!dut.enc_fault_r)$fatal(1,"right encoder fault injection not detected: pin=%b sync=%b candidate=%b stable=%0d previous=%b",abr,dut.enc.right_channel.ab_sync,dut.enc.right_channel.candidate,dut.enc.right_channel.stable_count,dut.enc.right_channel.previous);
  #1;off("right encoder fault");repeat(120)tick;
  if(!dut.local_fault)$fatal(1,"right fault latch");
  reset();key=0;repeat(10)tick;if(!dut.run_request)$fatal(1,"timer test trigger");key=1;count=0;while(dut.run_request)begin tick();count=count+1;end off("timed stop");
  if(count>500000 || count<499980)$fatal(1,"duration %0d",count);
  repeat(30)tick;if(dut.run_request)$fatal(1,"held rearm");
  key=0;repeat(10)tick;if(!dut.run_request)$fatal(1,"release and repress did not rearm");rst=0;off("reset");
  if(periods<100)$fatal(1,"PWM coverage");
  completed=1;$display("PASS dual target=%0d independent plants, signed feedback, physical PWM, direction, disable, both encoder faults, estop, timeout and rearm",LEFT_TARGET);$finish;
 end
 initial begin #40000000;$fatal(1,"timeout");end
endmodule
