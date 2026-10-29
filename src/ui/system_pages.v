`timescale 1ns/1ps
// Obstacle-demo pages: root menu, MANUAL actions (incl. SPIN 360), SETTINGS,
// the OBSTACLE AVOID panel and the IMU ATTITUDE page required by the task.
// The other DISPLAY diagnostic pages stay removed so the ToF text page and the
// encoder/speed formatting logic are not synthesized.
module system_pages(
 input wire clk,rst_n,
 input wire [7:0] start_diag,route_reason,
 input wire [2:0] group_id,input wire [3:0] item,input wire page,
 input wire avoid_mode,auto_run,waiting,input wire [4:0] route_state,input wire [47:0] distances,input wire [2:0] tof_valid,
 input wire [2:0] tof_reused,
 input wire attitude_valid,input wire signed [31:0] roll_mdeg,pitch_mdeg,yaw_wrapped,
 input wire voice_enable,voice_candidate,input wire [1:0] gear,gear_candidate,
 input wire ready,busy,fault,
 input wire [7:0] imu_error,sensor_error,motion_fault,command,response,
 output reg [671:0] text);
 function [167:0] row;
 input [167:0] value;integer k;
 begin for(k=0;k<21;k=k+1)row[k*8+:8]=value[(20-k)*8+:8];end
 endfunction
 function [167:0] label;
 input [2:0] group_value;input [3:0] index;
 begin label="                     ";case(group_value)
 0:case(index)
 0:label="MANUAL               ";
 1:label="SETTINGS             ";
 2:label="OBSTACLE AVOID       ";
 3:label="IMU ATTITUDE         ";
 default:label="                     ";endcase
 2:case(index)
 0:label="FORWARD 20CM         ";
 1:label="BACKWARD 20CM        ";
 2:label="LEFT 90 DEG          ";
 3:label="RIGHT 90 DEG         ";
 4:label="STOP                 ";
 5:label="SPIN 360 DEG         ";
 default:label="                     ";endcase
 6:label="IMU ATTITUDE         ";
 4:case(index)
 0:label="VOICE ENABLE         ";
 1:label="SPEED GEAR           ";
 default:label="                     ";endcase
 5:label="OBSTACLE AVOID       ";
 default:label="                     ";endcase end
 endfunction
 function [7:0] hexchar;
 input [3:0] x;begin hexchar=x<10 ? 8'd48+x:8'd55+x;end
 endfunction
 // Sample attitude once, then let one sequential decimal formatter serve roll,
 // pitch and yaw back to back (same scheme as the original DISPLAY page).
 reg signed [31:0] roll_q,pitch_q,yaw_q,att_value;
 reg [1:0] att_conv;
 wire att_valid,an;wire [3:0] ah,at,ao,af;
 attitude_angle_to_digits_pipeline ad(clk,rst_n,att_value,an,ah,at,ao,af,att_valid);
 function [47:0] angle;
 input n;input [3:0] h,t,o,f;
 begin angle={n ? 8'h2d:8'h2b,8'd48+h,8'd48+t,8'd48+o,8'h2e,8'd48+f};end
 endfunction
 reg [47:0] roll_text,pitch_text,yaw_text;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
 roll_q<=0;pitch_q<=0;yaw_q<=0;att_value<=0;att_conv<=0;
 roll_text<=0;pitch_text<=0;yaw_text<=0;
 end else begin
 roll_q<=roll_mdeg;pitch_q<=pitch_mdeg;yaw_q<=yaw_wrapped;
 if(att_valid)case(att_conv)
 0:begin roll_text<=angle(an,ah,at,ao,af);att_value<=pitch_q;att_conv<=1;end
 1:begin pitch_text<=angle(an,ah,at,ao,af);att_value<=yaw_q;att_conv<=2;end
 default:begin yaw_text<=angle(an,ah,at,ao,af);att_value<=roll_q;att_conv<=0;end
 endcase
 end
 reg [167:0] r0,r1,r2,r3,current_label;
 reg [3:0] last_item;
 always @* begin
 r0="                     ";r1=r0;r2=r0;r3=r0;current_label=label(group_id,item);
 case(group_id)0:last_item=3;2:last_item=5;4:last_item=1;5:last_item=0;default:last_item=0;endcase
 if(!page)begin
 // The next-item preview row is gone (r2 stays blank); only the current
 // selection is drawn, which keeps the menu readable and saves text muxes.
 r0=group_id==0 ? "RPACS MENU           ":
 group_id==2 ? "MANUAL               ":
 group_id==4 ? "SETTINGS             ":
 group_id==6 ? "IMU ATTITUDE         ":"OBSTACLE AVOID       ";
 r1={8'h3e,current_label[167:8]};
 r3=group_id==0 ? "K0 RST K1 NEXT K2 OK ":"K0 BACK K1 NEXT K2 OK";
 end else begin
 r0=current_label;
 r3=fault ? "FAULT K0 BACK/RESET  ":busy ? "RUNNING K0 STOP/BACK ":ready ? "READY K0 BACK K2 RUN ":"WAIT IMU CALIBRATION ";
 case(group_id)
 2:begin r1="K2 EXECUTE ONCE      ";r2=item==4 ? "STOP ANY ACTION      ":item<2 ? "GOAL 20CM            ":(item==5 ? "GOAL 360 DEG         ":"GOAL 90 DEG          ");end
 4:begin
 r3="K0 BACK K1 SEL K2 SET";
 if(item==0)begin
 r1=voice_enable ? "CURRENT ON           ":"CURRENT OFF          ";
 r2=voice_candidate ? "SELECT ON            ":"SELECT OFF           ";
 end else begin
 case(gear)0:r1="CURRENT LOW          ";1:r1="CURRENT MID          ";default:r1="CURRENT HIGH         ";endcase
 case(gear_candidate)0:r2="SELECT LOW  0.05 M/S ";1:r2="SELECT MID  0.10 M/S ";default:r2="SELECT HIGH 0.15 M/S ";endcase
 end
 end
 5:begin
 // On a fault the panel shows the four diagnostic bytes directly: route
 // reason, ToF sensor fault word, motion fault code and IMU error code.
 r1=fault ? {"F:",hexchar(route_reason[7:4]),hexchar(route_reason[3:0]),
 " S:",hexchar(sensor_error[7:4]),hexchar(sensor_error[3:0]),
 " M:",hexchar(motion_fault[7:4]),hexchar(motion_fault[3:0]),
 " I:",hexchar(imu_error[7:4]),hexchar(imu_error[3:0]),"  "} :
 waiting ? "SPEAKING PLEASE WAIT ":auto_run ? (route_state==1 || route_state==2 ? "CRUISE               ":"AVOIDING             "):
 !ready ? "WAIT IMU / CHASSIS   ":!(&tof_valid) ? "TOF NOT READY        ":
 busy ? "WAIT MOTION STOP     ":start_diag==7 ? "AUDIO TIMEOUT RETRY  ":
 start_diag==8 ? "TOF LOST RETRY       ":start_diag==1 ? "VOICE OFF K2 START   ":
 // Paused is the normal safe-retry endpoint: show the route reason, state and
 // the ToF lane masks (V=valid, U=reusing an old distance after exact-20) so a
 // field failure gives us the exact cause without a serial cable.
 avoid_mode ? {"PAUSED R:",hexchar(route_reason[7:4]),hexchar(route_reason[3:0]),
 " S",hexchar({1'b0,route_state[4]}),hexchar(route_state[3:0]),
 " V",hexchar({1'b0,tof_valid})," U",hexchar({1'b0,tof_reused})}:"READY TO START       ";
 if(auto_run || (!waiting && ready && &tof_valid && !fault && start_diag==0 && !avoid_mode))r1[39:0]=gear==0 ? " LOW ":gear==1 ? " MID ":" HIGH";
 r2={"F:",(tof_valid[0] ? {hexchar(distances[15:12]),hexchar(distances[11:8]),hexchar(distances[7:4]),hexchar(distances[3:0])}:32'h2d2d2d2d),
 " L:",(tof_valid[1] ? {hexchar(distances[31:28]),hexchar(distances[27:24]),hexchar(distances[23:20]),hexchar(distances[19:16])}:32'h2d2d2d2d),
 " R:",(tof_valid[2] ? {hexchar(distances[47:44]),hexchar(distances[43:40]),hexchar(distances[39:36]),hexchar(distances[35:32])}:32'h2d2d2d2d)," "};
 r3=auto_run ? "K0 EXIT K2 PAUSE     ":(avoid_mode ? "K0 EXIT K2 RETRY     ":"K0 EXIT K2 START     ");
 end
 6:begin
 // Task requirement: show the IMU attitude in the menu. attitude_valid gates
 // the digits so a calibrating IMU shows a prompt instead of stale angles.
 r1={"ROLL  ",roll_text," DEG     "};
 r2={"PITCH ",pitch_text," DEG     "};
 r3=attitude_valid ? {"YAW   ",yaw_text," DEG     "}:"CALIBRATING K0 BACK  ";
 end
 default:begin end
 endcase
 end
 text={row(r3),row(r2),row(r1),row(r0)};
 end
endmodule
