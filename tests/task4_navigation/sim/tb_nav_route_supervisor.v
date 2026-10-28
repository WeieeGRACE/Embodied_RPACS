`timescale 1ns/1ps
module tb_nav_route_supervisor;
    reg clk=0,rst_n=0,start=0,stop=0,estop=0,cal=0,geom=0,imu=0,enc=0;
    reg [2:0] route=1,tv=0,tu=0; reg [15:0] tf=0,tl=0,tr=0;
    reg signed [31:0] x=0,y=0,yaw=0;
    wire signed [31:0] vl,vr; wire [4:0] state; wire [7:0] fault; wire [2:0] accepted;
    wire [15:0] af,al,ar; wire done,blocked,rfault,ifault,drive;
    nav_route_supervisor #(.RANGE_AGE_LIMIT(100),.STATIONARY_TICKS(2),.ACTION_TIMEOUT(20)) dut(
      .clk(clk),.rst_n(rst_n),.start(start),.stop(stop),.estop(estop),.route_id(route),
      .calibration_done(cal),.geometry_ok(geom),.imu_valid(imu),.encoder_valid(enc),
      .tof_valid(tv),.tof_update(tu),.tof_front_mm(tf),.tof_left_mm(tl),.tof_right_mm(tr),
      .x_mm(x),.y_mm(y),.yaw_mdeg(yaw),.velocity_l_mm_s(vl),.velocity_r_mm_s(vr),
      .state(state),.fault_code(fault),.accepted_tof(accepted),.tof_age_front(af),.tof_age_left(al),.tof_age_right(ar),
      .done(done),.blocked(blocked),.range_fault(rfault),.input_fault(ifault),.drive_enable(drive));
    always #5 clk=~clk;
    task pulse_start; begin @(negedge clk); start=1; @(negedge clk); start=0; end endtask
    task range_ok; begin @(negedge clk); tv=3'b111;tu=3'b111;tf=500;tl=400;tr=400;@(negedge clk);tu=0;end endtask
    initial begin
      repeat(2) @(negedge clk); rst_n=1;
      pulse_start(); if(state!==7 || fault!==5) $fatal(1,"geometry gate failed state=%0d fault=%0d",state,fault);
      rst_n=0; @(negedge clk); rst_n=1; cal=1;geom=1;imu=1;enc=1; range_ok(); pulse_start();
      repeat(3) @(negedge clk); if(state!==2 || !drive) $fatal(1,"route did not enter drive state=%0d",state);
      tu=3'b001;tf=20;tv=3'b001; @(negedge clk); tu=0; @(negedge clk);
      if (accepted[0] || af==0 || vl===0 || vr===0) $fatal(1,"20 sample refreshed or unexpectedly stopped");
      tu=3'b001;tf=100;tv=3'b001; @(negedge clk); tu=0; @(negedge clk);
      if (vl!==0 || vr!==0) $fatal(1,"valid near sample did not stop");
      rst_n=0; @(negedge clk); rst_n=1; cal=1;geom=1;imu=1;enc=1; range_ok(); pulse_start();
      repeat(5) @(negedge clk); estop=1; @(negedge clk); estop=0;
      @(negedge clk); if (drive || !ifault || fault!==2) $fatal(1,"estop not latched drive=%b ifault=%b fault=%0d state=%0d",drive,ifault,fault,state);
      $display("PASS: geometry gate, calibration/input gate, 20 discard age, near stop, estop latch"); $finish;
    end
endmodule
