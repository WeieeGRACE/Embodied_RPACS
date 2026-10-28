`timescale 1ns/1ps
// The route supplies only its expected wall ID. Association is accepted only
// when that mapped wall's predicted range agrees with the fresh ToF sample;
// the observed wall ID is never injected by the caller or testbench.
// Wall IDs: 1..4 outer frame (bottom/right/top/left), 5 B_TOP, 6 B_SIDE,
// 7..8 diagonal X, 9 SOUTH_STUB, 10 MID_H, 11 MID_V, 12 D_RIGHT,
// 13 D_BOTTOM, 14 A_SIDE. Coordinates use mm; thickness is handled as a
// conservative 2 mm centerline-to-front-face allowance and uncertainty remains external.
module nav_map_wall_association #(
    parameter integer RANGE_TOLERANCE_MM=50,
    parameter integer FRONT_SENSOR_OFFSET_MM=320,
    parameter integer SIDE_SENSOR_FORWARD_MM=160,
    parameter integer SIDE_SENSOR_LATERAL_MM=120
) (
    input wire signed [31:0] pose_x_mm,
    input wire signed [31:0] pose_y_mm,
    input wire signed [31:0] yaw_mdeg,
    input wire [1:0] tof_dir,
    input wire [7:0] expected_wall_id,
    input wire range_valid,
    input wire [15:0] measured_range_mm,
    output reg [7:0] ray_wall_id,
    output reg [15:0] predicted_range_mm,
    output reg associated
);
    function automatic signed [15:0] sin_q15(input integer angle);
        integer a,q,v;
        begin
            a=angle%360000;if(a<0)a=a+360000;q=a/90000;
            case(q)
                0:v=(a*32767)/90000;
                1:v=32767-((a-90000)*32767)/90000;
                2:v=-((a-180000)*32767)/90000;
                default:v=-32767+((a-270000)*32767)/90000;
            endcase
            sin_q15=v;
        end
    endfunction
    function automatic signed [15:0] cos_q15(input integer angle);
        begin cos_q15=sin_q15(angle+90000);end
    endfunction
    function automatic integer wall_ax(input integer id);
        begin case(id)
            1:wall_ax=0;2:wall_ax=3000;3:wall_ax=3000;4:wall_ax=0;
            5:wall_ax=0;6:wall_ax=600;7:wall_ax=446;8:wall_ax=446;
            9:wall_ax=1200;10:wall_ax=1200;11:wall_ax=1800;12:wall_ax=2400;
            13:wall_ax=1800;14:wall_ax=2400;default:wall_ax=0;
        endcase end
    endfunction
    function automatic integer wall_ay(input integer id);
        begin case(id)
            1:wall_ay=0;2:wall_ay=0;3:wall_ay=3000;4:wall_ay=3000;
            5:wall_ay=1200;6:wall_ay=600;7:wall_ay=2574;8:wall_ay=1726;
            9:wall_ay=0;10:wall_ay=1200;11:wall_ay=600;12:wall_ay=1800;
            13:wall_ay=1800;14:wall_ay=0;default:wall_ay=0;
        endcase end
    endfunction
    function automatic integer wall_bx(input integer id);
        begin case(id)
            1:wall_bx=3000;2:wall_bx=3000;3:wall_bx=0;4:wall_bx=0;
            5:wall_bx=600;6:wall_bx=600;7:wall_bx=1294;8:wall_bx=1294;
            9:wall_bx=1200;10:wall_bx=3000;11:wall_bx=1800;12:wall_bx=2400;
            13:wall_bx=2400;14:wall_bx=2400;default:wall_bx=0;
        endcase end
    endfunction
    function automatic integer wall_by(input integer id);
        begin case(id)
            1:wall_by=0;2:wall_by=3000;3:wall_by=3000;4:wall_by=0;
            5:wall_by=1200;6:wall_by=1200;7:wall_by=1726;8:wall_by=2574;
            9:wall_by=600;10:wall_by=1200;11:wall_by=2400;12:wall_by=2400;
            13:wall_by=1800;14:wall_by=600;default:wall_by=0;
        endcase end
    endfunction

    integer fwd_x,fwd_y,side_x,side_y,offset_fwd,offset_side;
    integer origin_x,origin_y,ray_x,ray_y;
    integer wa_x,wa_y,wb_x,wb_y,sx,sy;
    reg signed [63:0] denom,t_num,u_num,t_value,best_value;
    reg signed [63:0] range_error_numerator,range_limit_numerator,range_target_numerator;
    reg intersection_valid;
    always @* begin
        fwd_x=-sin_q15(yaw_mdeg);fwd_y=cos_q15(yaw_mdeg);
        side_x=0;side_y=0;offset_fwd=0;offset_side=0;
        if(tof_dir==0) begin
            offset_fwd=FRONT_SENSOR_OFFSET_MM;
            ray_x=fwd_x;ray_y=fwd_y;
        end else begin
            offset_fwd=SIDE_SENSOR_FORWARD_MM;
            if(tof_dir==1) begin side_x=-cos_q15(yaw_mdeg);side_y=-sin_q15(yaw_mdeg);end
            else begin side_x=cos_q15(yaw_mdeg);side_y=sin_q15(yaw_mdeg);end
            offset_side=SIDE_SENSOR_LATERAL_MM;
            ray_x=side_x;ray_y=side_y;
        end
        origin_x=pose_x_mm+(fwd_x*offset_fwd+side_x*offset_side)/32767;
        origin_y=pose_y_mm+(fwd_y*offset_fwd+side_y*offset_side)/32767;
        ray_wall_id=0;predicted_range_mm=16'hffff;best_value=0;intersection_valid=0;
        wa_x=wall_ax(expected_wall_id);wa_y=wall_ay(expected_wall_id);
        wb_x=wall_bx(expected_wall_id);wb_y=wall_by(expected_wall_id);
        sx=wb_x-wa_x;sy=wb_y-wa_y;
        denom=$signed(ray_x)*$signed(sy)-$signed(ray_y)*$signed(sx);
        t_num=(($signed(wa_x)-$signed(origin_x))*$signed(sy)-
               ($signed(wa_y)-$signed(origin_y))*$signed(sx))*64'sd32767;
        u_num=($signed(wa_x)-$signed(origin_x))*$signed(ray_y)-
              ($signed(wa_y)-$signed(origin_y))*$signed(ray_x);
        if(denom<0) begin denom=-denom;t_num=-t_num;u_num=-u_num;end
        if(denom!=0 && t_num>=0 && u_num>=0 && u_num<=denom) begin
            intersection_valid=1;best_value=t_num/denom;ray_wall_id=expected_wall_id;
            if(best_value>64'sd65537) predicted_range_mm=16'hffff;
            else if(best_value>2) predicted_range_mm=best_value[15:0]-16'd2;
            else predicted_range_mm=0;
        end
        range_target_numerator=($signed({1'b0,measured_range_mm})+64'sd2)*denom;
        range_error_numerator=t_num-range_target_numerator;
        if(range_error_numerator<0) range_error_numerator=-range_error_numerator;
        range_limit_numerator=RANGE_TOLERANCE_MM*denom;
        associated=range_valid && measured_range_mm!=16'd20 && expected_wall_id!=0 &&
                   intersection_valid && ray_wall_id==expected_wall_id && predicted_range_mm!=16'hffff &&
                   range_error_numerator<=range_limit_numerator;
    end
endmodule
