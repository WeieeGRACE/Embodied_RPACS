`timescale 1ns/1ps
// Route/map association ROM. Distances are navigation odometry mm, not ToF mm.
// These values are simulation baselines; geometry_ok must remain low on hardware
// until a swept-path check signs off the same route/map version.
module nav_segment_table (
    input wire [2:0] route_id,
    input wire [3:0] segment_id,
    output reg valid,
    output reg final_segment,
    output reg [15:0] distance_min_mm,
    output reg [15:0] distance_max_mm,
    output reg [15:0] hard_stop_mm,
    output reg [15:0] forward_stop_mm,
    output reg [7:0] wall_id,
    output reg [1:0] tof_dir,
    output reg [15:0] event_range_min_mm,
    output reg [15:0] event_range_max_mm,
    output reg [1:0] turn_kind, // 0=in-place at proven clear nodes, 1=continuous arc
    output reg [15:0] speed_mm_s,
    output reg signed [31:0] target_yaw_mdeg,
    output reg signed [31:0] target_x_mm,
    output reg signed [31:0] target_y_mm
);
    always @* begin
        valid=0; final_segment=0; distance_min_mm=0; distance_max_mm=0;
        hard_stop_mm=0; forward_stop_mm=120; wall_id=0; tof_dir=0;
        event_range_min_mm=25; event_range_max_mm=2500;
        turn_kind=1; speed_mm_s=0; target_yaw_mdeg=0; target_x_mm=0; target_y_mm=0;
        case ({route_id,segment_id})
            7'b0010000: begin valid=1;distance_min_mm=450;distance_max_mm=900;hard_stop_mm=1100;wall_id=10;tof_dir=0;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=0;target_x_mm=300;target_y_mm=1000;end
            7'b0010001: begin valid=1;distance_min_mm=500;distance_max_mm=1300;hard_stop_mm=1500;wall_id=10;tof_dir=2;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=90000;target_x_mm=300;target_y_mm=1000;end
            7'b0010010: begin valid=1;final_segment=1;distance_min_mm=250;distance_max_mm=800;hard_stop_mm=1000;wall_id=5;tof_dir=1;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=40;target_yaw_mdeg=90000;target_x_mm=300;target_y_mm=1000;end
            7'b0100000: begin valid=1;distance_min_mm=450;distance_max_mm=900;hard_stop_mm=1100;wall_id=1;tof_dir=1;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=0;target_x_mm=300;target_y_mm=2400;end
            7'b0100001: begin valid=1;distance_min_mm=900;distance_max_mm=1800;hard_stop_mm=2000;wall_id=4;tof_dir=1;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=180000;target_x_mm=300;target_y_mm=2400;end
            7'b0100010: begin valid=1;final_segment=1;distance_min_mm=250;distance_max_mm=800;hard_stop_mm=1000;wall_id=5;tof_dir=0;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=40;target_yaw_mdeg=180000;target_x_mm=300;target_y_mm=2400;end
            7'b0110000: begin valid=1;distance_min_mm=450;distance_max_mm=900;hard_stop_mm=1100;wall_id=1;tof_dir=1;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=0;target_x_mm=2100;target_y_mm=2400;end
            7'b0110001: begin valid=1;distance_min_mm=900;distance_max_mm=1900;hard_stop_mm=2100;wall_id=6;tof_dir=2;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=0;target_x_mm=2100;target_y_mm=2400;end
            7'b0110010: begin valid=1;final_segment=1;distance_min_mm=250;distance_max_mm=800;hard_stop_mm=1000;wall_id=7;tof_dir=0;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=40;target_yaw_mdeg=0;target_x_mm=2100;target_y_mm=2400;end
            7'b1000000: begin valid=1;distance_min_mm=450;distance_max_mm=900;hard_stop_mm=1100;wall_id=1;tof_dir=1;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=0;target_x_mm=2100;target_y_mm=1650;end
            7'b1000001: begin valid=1;distance_min_mm=500;distance_max_mm=1300;hard_stop_mm=1500;wall_id=6;tof_dir=2;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=180000;target_x_mm=2100;target_y_mm=1650;end
            7'b1000010: begin valid=1;final_segment=1;distance_min_mm=250;distance_max_mm=800;hard_stop_mm=1000;wall_id=8;tof_dir=0;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=40;target_yaw_mdeg=180000;target_x_mm=2100;target_y_mm=1650;end
            7'b1010000: begin valid=1;distance_min_mm=450;distance_max_mm=900;hard_stop_mm=1100;wall_id=1;tof_dir=1;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=0;target_x_mm=2100;target_y_mm=1650;end
            7'b1010001: begin valid=1;distance_min_mm=900;distance_max_mm=1800;hard_stop_mm=2000;wall_id=4;tof_dir=1;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=180000;target_x_mm=2100;target_y_mm=1650;end
            7'b1010010: begin valid=1;distance_min_mm=1200;distance_max_mm=2200;hard_stop_mm=2400;wall_id=6;tof_dir=2;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=0;target_x_mm=2100;target_y_mm=1650;end
            7'b1010011: begin valid=1;final_segment=1;distance_min_mm=450;distance_max_mm=1000;hard_stop_mm=1200;wall_id=7;tof_dir=0;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=40;target_yaw_mdeg=180000;target_x_mm=2100;target_y_mm=1650;end
            // Restricted single-leg open-ground test: A to the first safe
            // waypoint below MID_H. This is not a B/C/D/E route release.
            7'b1100000: begin valid=1;final_segment=1;distance_min_mm=400;distance_max_mm=430;hard_stop_mm=520;wall_id=10;tof_dir=0;event_range_min_mm=100;event_range_max_mm=700;speed_mm_s=50;target_yaw_mdeg=0;target_x_mm=2700;target_y_mm=750;end
            default: begin end
        endcase
    end
endmodule
