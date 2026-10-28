`timescale 1ns/1ps
module tb_nav_event_segment;
    reg clk=0,rst_n=0,start=0,stop=0,estop=0,cal=0,geom=0,imu=0,enc=0;
    reg tick=1,map_match=1,range_ready=0,stationary=1;
    reg [2:0] route=1;
    reg [31:0] odom=0;
    reg signed [31:0] yaw=0,pose_x=2700,pose_y=300;
    reg front_valid=1;reg [15:0] front_mm=1000;
    reg wall_sample_update=0,wall_sample_valid=1;reg [15:0] wall_mm=0;
    reg range_fresh=0;
    wire clear_event;wire signed [31:0] vl,vr;wire [4:0] state;wire [3:0] seg;
    wire [7:0] accepted_wall,fault;wire [31:0] seg_dist;
    wire [15:0] min_mm,max_mm,hard_mm,front_stop,evt_min,evt_max,speed;
    wire [7:0] expected_wall;wire [1:0] tof_dir,turn_kind;
    wire signed [31:0] target_yaw,target_x,target_y;
    wire done,blocked,rfault,ifault,drive;
    wire record_valid;wire [3:0] record_segment_id;wire [31:0] record_distance;
    wire signed [31:0] record_start_yaw,record_end_yaw;
    wire [7:0] record_wall,record_fault;
    wire [15:0] wall_age,wall_accepted;wire wall_valid,wall_pulse,wall_latched,wall_stale,discard20;
    wire [7:0] wall_event_id;
    reg record_seen=0,record_fault_seen=0;
    always #5 clk=~clk;
    always @(posedge clk) if(record_valid) begin
        record_seen<=1;
        if(record_fault!=0) record_fault_seen<=1;
    end

    nav_segment_table table_i(.route_id(route),.segment_id(seg),.valid(),.final_segment(),
      .distance_min_mm(min_mm),.distance_max_mm(max_mm),.hard_stop_mm(hard_mm),
      .forward_stop_mm(front_stop),.wall_id(expected_wall),.tof_dir(tof_dir),
      .event_range_min_mm(evt_min),.event_range_max_mm(evt_max),.turn_kind(turn_kind),
      .speed_mm_s(speed),.target_yaw_mdeg(target_yaw),.target_x_mm(target_x),.target_y_mm(target_y));

    nav_tof_wall_event #(.CONSECUTIVE_REQUIRED(3),.RANGE_AGE_LIMIT(50)) event_i(
      .clk(clk),.rst_n(rst_n),.clear_event(clear_event),.enable(state==1),.age_tick(tick),
      .expected_wall_id(expected_wall),.map_match(map_match),.sample_update(wall_sample_update),
      .sample_valid(wall_sample_valid),.sample_mm(wall_mm),.event_range_min_mm(evt_min),
      .event_range_max_mm(evt_max),.segment_distance_mm(seg_dist),.distance_min_mm(min_mm),
      .distance_max_mm(max_mm),.age(wall_age),.accepted_mm(wall_accepted),.accepted_valid(wall_valid),
      .accepted_pulse(),.wall_event_id(wall_event_id),.event_pulse(wall_pulse),
      .event_latched(wall_latched),.stale_fault(wall_stale),.sample_discarded_20(discard20));

    nav_event_segment_controller #(.STATIONARY_TICKS(2),.TURN_TIMEOUT_TICKS(20),
      .NO_PROGRESS_TICKS(50),.ACTION_TIMEOUT_TICKS(1000)) dut(
      .clk(clk),.rst_n(rst_n),.control_tick(tick),.start(start),.stop(stop),.estop(estop),
      .route_id(route),.calibration_done(cal),.geometry_ok(geom),.imu_valid(imu),.encoder_valid(enc),
      .range_ready(range_ready),.pose_x_mm(pose_x),.pose_y_mm(pose_y),.odometry_distance_mm(odom),
      .yaw_mdeg(yaw),.wheels_stationary(stationary),.front_valid(front_valid),.front_mm(front_mm),
      .range_stale_fault(wall_stale),.wall_event_pulse(wall_pulse),.wall_event_id(wall_event_id),
      .range_fresh_after_turn(range_fresh),.clear_wall_event(clear_event),
      .velocity_l_mm_s(vl),.velocity_r_mm_s(vr),.state(state),.segment_id(seg),
      .accepted_wall_event(accepted_wall),.segment_distance_mm(seg_dist),
      .expected_min_mm(),.expected_max_mm(),.hard_stop_mm(),.forward_stop_mm(),
      .expected_wall_id(),.expected_tof_dir(),.turn_kind(),.segment_speed_mm_s(),
      .target_yaw_mdeg(),.target_x_mm(),.target_y_mm(),.done(done),.blocked(blocked),
      .range_fault(rfault),.input_fault(ifault),.fault_code(fault),.record_valid(record_valid),
      .record_segment_id(record_segment_id),.record_distance_mm(record_distance),
      .record_start_yaw_mdeg(record_start_yaw),.record_end_yaw_mdeg(record_end_yaw),
      .record_wall_event_id(record_wall),.record_fault_code(record_fault),.drive_enable(drive));

    task pulse_start;begin @(negedge clk);start=1;@(negedge clk);start=0;end endtask
    task sample(input [15:0] v);begin @(negedge clk);wall_mm=v;wall_sample_update=1;@(negedge clk);wall_sample_update=0;end endtask
    task fresh_range;begin @(negedge clk);range_fresh=1;@(negedge clk);range_fresh=0;end endtask
    task wait_cycles(input integer n);begin repeat(n) @(negedge clk);end endtask

    initial begin
        repeat(2) @(negedge clk);rst_n=1;
        pulse_start();@(negedge clk);
        if(fault!=5 || !blocked) $fatal(1,"geometry/calibration gate did not block");

        // Reset, establish a fresh non-20 range, then start a configured segment.
        rst_n=0;@(negedge clk);rst_n=1;cal=1;geom=1;imu=1;enc=1;
        sample(1000);sample(600);range_ready=1;
        pulse_start();@(negedge clk);
        if(state!==1 || !drive) $fatal(1,"did not enter segment drive");

        // Invalidated observations, including exact 20, break the consecutive
        // wall-event streak without refreshing the last accepted range age.
        odom=500;pose_y=800;sample(1000);sample(600);sample(600);sample(20);
        sample(600);sample(600);wait_cycles(2);
        if(state!==1 || wall_pulse) $fatal(1,"20 mm did not break the event confirmation streak");
        sample(600);
        wait_cycles(5);
        if(state!==3 || !record_seen || record_segment_id!=0 || record_distance<450 || record_distance>900)
            $fatal(1,"event was not recorded before route turn: state=%0d seg=%0d dist=%0d",state,record_segment_id,record_distance);
        yaw=90000;wait_cycles(3);
        if(state!==4 || drive) $fatal(1,"turn did not stop and wait for fresh range state=%0d",state);
        sample(600);fresh_range();wait_cycles(3);
        if(seg!==1 || state!==1 || !drive) $fatal(1,"fresh post-turn sample did not advance");

        // A geometrically similar sample with no map association cannot count as a wall event.
        map_match=0;odom=1000;sample(1000);sample(600);sample(600);sample(600);wait_cycles(2);
        if(state!==1 || done || wall_pulse) $fatal(1,"unassociated wall advanced the route");
        map_match=1;

        // Exact 20 is discarded and cannot refresh the accepted distance/age.
        begin : discard_check
            reg [15:0] prior_age,prior_mm;
            prior_age=wall_age;prior_mm=wall_accepted;
            @(negedge clk);wall_mm=20;wall_sample_update=1;@(negedge clk);
            if(!discard20 || wall_accepted!=prior_mm || wall_age<=prior_age)
                $fatal(1,"20 mm refreshed or replaced accepted range");
            wall_sample_update=0;
        end

        // Front obstacle stop is not a wall event and must never report DONE.
        front_mm=100;wait_cycles(5);
        if(state!==6 || done || fault!=9 || !blocked || !record_fault_seen || record_fault!=9)
            $fatal(1,"front obstacle was treated as route completion state=%0d fault=%0d done=%b",state,fault,done);

        // A run of 20 mm values ages out the last normal reading and locks RANGE_FAULT.
        rst_n=0;@(negedge clk);rst_n=1;cal=1;geom=1;imu=1;enc=1;front_mm=1000;odom=0;range_ready=0;
        sample(600);range_ready=1;pulse_start();
        repeat(55) begin @(negedge clk);wall_sample_update=1;wall_mm=20;@(negedge clk);wall_sample_update=0;end
        if(!wall_stale || !rfault || fault!=3) $fatal(1,"continuous 20 did not age out range");

        // Hard segment mileage limit brakes then reports BLOCKED, never DONE.
        rst_n=0;@(negedge clk);rst_n=1;cal=1;geom=1;imu=1;enc=1;front_mm=1000;odom=0;range_ready=0;
        sample(600);range_ready=1;pulse_start();odom=1101;wait_cycles(5);
        if(state!==6 || done || fault!=9) $fatal(1,"hard distance limit did not fail closed");

        $display("PASS: map-linked sample debounce, separate range/distance windows, 20 discard/age, post-turn freshness, obstacle and hard-stop no-DONE");
        $finish;
    end
endmodule
