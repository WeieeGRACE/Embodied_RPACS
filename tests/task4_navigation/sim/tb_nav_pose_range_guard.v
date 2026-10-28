`timescale 1ns/1ps
module tb_nav_pose_range_guard;
    reg clk=0, rst_n=0, tick=0, pose_valid=1;
    reg signed [23:0] l=0, r=0, gyro=0, enc_yaw=0;
    reg [2:0] upd=0;
    reg [15:0] rf=0, rl=0, rr=0;
    wire signed [31:0] x,y,yaw;
    wire [15:0] af,al,ar,agef,agel,ager;
    wire [2:0] fresh; wire fault;
    nav_pose_range_guard #(.AGE_LIMIT_TICKS(3)) dut(
        .clk(clk),.rst_n(rst_n),.tick_100hz(tick),.pose_valid(pose_valid),
        .left_um(l),.right_um(r),.gyro_yaw_delta_mdeg(gyro),.encoder_yaw_delta_mdeg(enc_yaw),
        .range_update(upd),.range_front_mm(rf),.range_left_mm(rl),.range_right_mm(rr),
        .x_um(x),.y_um(y),.yaw_mdeg(yaw),.accepted_front_mm(af),.accepted_left_mm(al),
        .accepted_right_mm(ar),.age_front(agef),.age_left(agel),.age_right(ager),
        .range_fresh(fresh),.pose_fault(fault));
    always #5 clk=~clk;
    task step; begin @(negedge clk); tick=1; @(negedge clk); tick=0; end endtask
    initial begin
        repeat(2) @(negedge clk); rst_n=1;
        upd=3'b001; rf=16'd540; @(negedge clk); upd=0;
        if (af!==540 || agef!==0 || !fresh[0]) $fatal(1,"valid range not accepted");
        step(); if (agef!==1) $fatal(1,"age did not advance");
        upd=3'b001; rf=20; @(negedge clk); upd=0;
        if (af!==540 || agef!==1 || fresh[0]) $fatal(1,"20 mm refreshed stale range");
        l=100000; r=100000; gyro=1000; enc_yaw=0; step();
        // Q15 maps unity to 32767, so an arithmetic shift can lose up to one
        // Q15 LSB. At a 100 mm increment the expected quantization is < 4 um.
        if ((y<99995 || y>100005) || yaw!==750)
            $fatal(1,"pose convention/fusion mismatch y=%0d yaw=%0d fault=%b ds=%0d fused=%0d",y,yaw,fault,dut.ds_um,dut.fused_yaw);
        pose_valid=0; step(); if (!fault) $fatal(1,"invalid pose was not latched");
        $display("PASS: range 20 discard, freshness aging, signed straight odometry, yaw fusion, fault latch");
        $finish;
    end
endmodule

