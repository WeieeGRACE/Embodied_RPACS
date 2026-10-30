`timescale 1ns/1ns
module tb_nav_closed_loop;
    reg clk=0,reset_n=0,key_start_n=1,key_stop_n=1,key_estop_n=1;
    reg calibration_done=1,geometry_ok=1,imu_valid=1,encoder_valid=1;
    reg signed [23:0] enc_l_um=0,enc_r_um=0,imu_yaw_delta=0,speed_l=0,speed_r=0;
    reg speed_valid=1;
    reg [2:0] tof_valid=0,tof_update=0;
    reg [15:0] tof_front=0,tof_left=0,tof_right=0;
    reg [2:0] route_id=6;
    wire signed [31:0] pose_x_um,pose_y_um,pose_yaw;
    wire signed [31:0] cmd_l,cmd_r;
    wire [4:0] nav_state;wire [3:0] segment_id;wire [7:0] fault_code,wall_event_id;
    wire [2:0] accepted_tof,tof_fresh;
    wire [15:0] age_f,age_l,age_r;
    wire record_valid;wire [3:0] record_segment;wire [31:0] record_distance;
    wire signed [31:0] record_start_yaw,record_end_yaw;
    wire [7:0] record_wall,record_fault;
    wire nav_done,nav_blocked,nav_range_fault,nav_input_fault,drive_enable;

    real true_x=2700.0,true_y=300.0,true_yaw=0.0;
    real motor_l=0.0,motor_r=0.0;
    real frac_l_um=0.0,frac_r_um=0.0,gyro_frac=0.0;
    real cmd_l_d1=0.0,cmd_r_d1=0.0,cmd_l_d2=0.0,cmd_r_d2=0.0;
    real old_x,old_y,old_yaw,dl,dr,ds,dtheta,mid_yaw,rng_f,rng_l,rng_r;
    integer clock_count=0,plant_steps=0,log_fd;
    integer noise_state=32'h5441534B;
    integer sample_front,sample_left,sample_right;
    integer wheel_l_um,wheel_r_um,gyro_mdeg;

    task4_event_navigation_top #(.CLK_HZ(1000)) dut(
      .clk_50m(clk),.reset_n(reset_n),.key_start_n(key_start_n),.key_stop_n(key_stop_n),
      .key_estop_n(key_estop_n),.calibration_done(calibration_done),.geometry_ok(geometry_ok),
      .range_stale_external(1'b0),
      .imu_valid(imu_valid),.encoder_valid(encoder_valid),.encoder_left_um(enc_l_um),
      .encoder_right_um(enc_r_um),.imu_yaw_delta_mdeg(imu_yaw_delta),
      .wheels_stationary(speed_valid && speed_l<=5 && speed_l>=-5 &&
                         speed_r<=5 && speed_r>=-5),
      .tof_valid(tof_valid),.tof_update(tof_update),.tof_front_mm(tof_front),
      .tof_left_mm(tof_left),.tof_right_mm(tof_right),.route_id(route_id),
      .pose_x_um(pose_x_um),.pose_y_um(pose_y_um),.pose_yaw_mdeg(pose_yaw),
      .velocity_l_mm_s(cmd_l),.velocity_r_mm_s(cmd_r),.nav_state(nav_state),.segment_id(segment_id),
      .fault_code(fault_code),.wall_event_id(wall_event_id),.accepted_tof(accepted_tof),
      .tof_fresh(tof_fresh),.tof_age_front(age_f),.tof_age_left(age_l),.tof_age_right(age_r),
      .record_valid(record_valid),.record_segment_id(record_segment),.record_distance_mm(record_distance),
      .record_start_yaw_mdeg(record_start_yaw),.record_end_yaw_mdeg(record_end_yaw),
      .record_wall_event_id(record_wall),.record_fault_code(record_fault),.nav_done(nav_done),
      .nav_blocked(nav_blocked),.nav_range_fault(nav_range_fault),.nav_input_fault(nav_input_fault),
      .drive_enable(drive_enable));

    function real raycast(input real cx,input real cy,input real yaw,input integer direction);
        real fx,fy,sx,sy,ox,oy,dx,dy,ax,ay,bx,by,wx,wy,den,t,u,best;
        integer k;
        begin
            fx=-$sin(yaw);fy=$cos(yaw);sx=0.0;sy=0.0;
            if(direction==0) begin ox=cx+320.0*fx;oy=cy+320.0*fy;dx=fx;dy=fy;end
            else begin
                if(direction==1) begin sx=-$cos(yaw);sy=-$sin(yaw);end
                else begin sx=$cos(yaw);sy=$sin(yaw);end
                ox=cx+160.0*fx+120.0*sx;oy=cy+160.0*fy+120.0*sy;dx=sx;dy=sy;
            end
            best=99999.0;
            for(k=1;k<=14;k=k+1) begin
                case(k)
                    1:begin ax=0;ay=0;bx=3000;by=0;end
                    2:begin ax=3000;ay=0;bx=3000;by=3000;end
                    3:begin ax=3000;ay=3000;bx=0;by=3000;end
                    4:begin ax=0;ay=3000;bx=0;by=0;end
                    5:begin ax=0;ay=1200;bx=600;by=1200;end
                    6:begin ax=600;ay=600;bx=600;by=1200;end
                    7:begin ax=445.736;ay=2574.264;bx=1294.264;by=1725.736;end
                    8:begin ax=445.736;ay=1725.736;bx=1294.264;by=2574.264;end
                    9:begin ax=1200;ay=0;bx=1200;by=600;end
                    10:begin ax=1200;ay=1200;bx=3000;by=1200;end
                    11:begin ax=1800;ay=600;bx=1800;by=2400;end
                    12:begin ax=2400;ay=1800;bx=2400;by=2400;end
                    13:begin ax=1800;ay=1800;bx=2400;by=1800;end
                    default:begin ax=2400;ay=0;bx=2400;by=600;end
                endcase
                wx=bx-ax;wy=by-ay;den=dx*wy-dy*wx;
                if((den>0.000001)||(den< -0.000001)) begin
                    t=((ax-ox)*wy-(ay-oy)*wx)/den;
                    u=((ax-ox)*dy-(ay-oy)*dx)/den;
                    if(t>=0.0 && u>=-0.000001 && u<=1.000001 && t<best) best=t;
                end
            end
            if(best>2500.0) raycast=2500.0;
            else if(best>1.5) raycast=best-1.5;
            else raycast=0.0;
        end
    endfunction

    function integer rounded(input real v);
        begin rounded=$rtoi(v+0.5);end
    endfunction
    function real approach(input real current,input real target,input real step);
        begin
            if(current<target) approach=(target-current<=step)?target:current+step;
            else approach=(current-target<=step)?target:current-step;
        end
    endfunction

    always #500000 clk=~clk; // 1 kHz, consistent with CLK_HZ=1000 and 100 Hz nav ticks.
    always @(posedge clk) begin
        clock_count=clock_count+1;
        tof_update<=0;
        if(clock_count%10==0) begin
            plant_steps=plant_steps+1;
            old_x=true_x;old_y=true_y;old_yaw=true_yaw;
            cmd_l_d2=cmd_l_d1;cmd_r_d2=cmd_r_d1;
            cmd_l_d1=$itor(cmd_l);cmd_r_d1=$itor(cmd_r);
            // 20 ms actuator latency, bounded acceleration, and slight wheel mismatch.
            motor_l=approach(motor_l,cmd_l_d2,2.0);
            motor_r=approach(motor_r,cmd_r_d2,2.0);
            dl=motor_l*0.99*0.01;dr=motor_r*1.01*0.01;
            ds=(dl+dr)/2.0;dtheta=(dr-dl)/240.0;mid_yaw=true_yaw+dtheta/2.0;
            true_x=true_x-ds*$sin(mid_yaw);true_y=true_y+ds*$cos(mid_yaw);true_yaw=true_yaw+dtheta;
            frac_l_um=frac_l_um+dl*1000.0;wheel_l_um=$rtoi(frac_l_um);frac_l_um=frac_l_um-wheel_l_um;
            frac_r_um=frac_r_um+dr*1000.0;wheel_r_um=$rtoi(frac_r_um);frac_r_um=frac_r_um-wheel_r_um;
            enc_l_um<=wheel_l_um;enc_r_um<=wheel_r_um;
            speed_l<=$rtoi(motor_l*0.99);speed_r<=$rtoi(motor_r*1.01);
            gyro_frac=gyro_frac+dtheta*57295.7795 + 0.1;
            gyro_mdeg=$rtoi(gyro_frac);gyro_frac=gyro_frac-gyro_mdeg;
            imu_yaw_delta<=gyro_mdeg;

            // Sensor observations are ray casts from true vehicle pose to the
            // map's actual segments, sampled every 100 ms with deterministic noise.
            if(plant_steps%10==0) begin
                rng_f=raycast(true_x,true_y,true_yaw,0);
                rng_l=raycast(true_x,true_y,true_yaw,1);
                rng_r=raycast(true_x,true_y,true_yaw,2);
                noise_state=(noise_state*1103515245+12345)&32'h7fffffff;
                // This TB drives the event top directly, i.e. after the board-level
                // ToF calibration, so it must feed already-calibrated distances.
                sample_front=rounded(rng_f+((noise_state%5)-2));
                noise_state=(noise_state*1103515245+12345)&32'h7fffffff;
                sample_left=rounded(rng_l+((noise_state%5)-2));
                noise_state=(noise_state*1103515245+12345)&32'h7fffffff;
                sample_right=rounded(rng_r+((noise_state%5)-2));
                tof_front<=sample_front;tof_left<=sample_left;tof_right<=sample_right;
                tof_valid<=3'b111;tof_update<=3'b111;
            end
            if(log_fd!=0)
                $fwrite(log_fd,"%0d,%.3f,%.3f,%.5f,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                    plant_steps,true_x,true_y,true_yaw*57.2957795,$signed(pose_x_um)/1000.0,
                    $signed(pose_y_um)/1000.0,$signed(pose_yaw)/1000,cmd_l,cmd_r,
                    sample_front,sample_left,sample_right,nav_state,dut.odometry_distance_mm,
                    dut.segment_distance_i,dut.map_match,dut.wall_age,wall_event_id,record_valid);
        end
    end

    integer watchdog;
    real position_error,yaw_error;
    initial begin
        log_fd=$fopen("records/closed_loop_route6.csv","w");
        if(log_fd==0) $fatal(1,"could not open closed loop log");
        $fwrite(log_fd,"step,true_x_mm,true_y_mm,true_yaw_deg,estimated_x_mm,estimated_y_mm,estimated_yaw_deg,cmd_l_mm_s,cmd_r_mm_s,tof_front_mm,tof_left_mm,tof_right_mm,state,odom_mm,segment_mm,map_match,range_age,event_id,record\n");
        repeat(20) @(posedge clk);reset_n=1;
        repeat(120) @(posedge clk); // allow three actual ToF reads before start.
        if(dut.map_assoc.ray_wall_id!==8'd10 || !dut.map_assoc.associated)
            $fatal(1,"map association did not identify MID_H: wall=%0d range=%0d measured=%0d",
                dut.map_assoc.ray_wall_id,dut.map_assoc.predicted_range_mm,tof_front);
        key_start_n=0;repeat(10) @(posedge clk);key_start_n=1;
        watchdog=0;
        while(!nav_done && !nav_blocked && watchdog<40000) begin @(posedge clk);watchdog=watchdog+1;end
        position_error=$sqrt((true_x-2700.0)*(true_x-2700.0)+(true_y-750.0)*(true_y-750.0));
        yaw_error=true_yaw*57.2957795;
        if(!nav_done || nav_blocked || fault_code!=0)
            $fatal(1,"closed-loop single leg failed state=%0d fault=%0d x=%.2f y=%.2f dist=%0d wall=%0d",
                nav_state,fault_code,true_x,true_y,record_distance,record_wall);
        if(position_error>100.0 || yaw_error>5.0)
            $fatal(1,"closed-loop target error too large pos=%.2f yaw=%.3f",position_error,yaw_error);
        if(record_segment!=0 || record_wall!=10 || record_distance<400 || record_distance>450)
            $fatal(1,"unexpected segment record seg=%0d wall=%0d distance=%0d",record_segment,record_wall,record_distance);
        $fclose(log_fd);
        $display("PASS: physical wheel/IMU/ToF closed loop, mapped wall=10, stop error=%.2f mm yaw=%.3f deg steps=%0d",
            position_error,yaw_error,plant_steps);
        $finish;
    end
endmodule
