`timescale 1ns/1ps
module system_pages(
 input wire clk,rst_n,
 input wire [2:0] group_id,input wire [3:0] item,input wire page,
 input wire voice_enable,voice_candidate,input wire [1:0] gear,gear_candidate,
 input wire ready,busy,fault,imu_ready,calibrating,attitude_valid,
 input wire [7:0] status,imu_error,sensor_error,motion_fault,command,response,
 input wire signed [31:0] roll_mdeg,pitch_mdeg,yaw_mdeg,pos_l,pos_r,speed_l,speed_r,
 input wire [671:0] tof_text,output reg [671:0] text);
 function [167:0] row;
 input [167:0] value;integer k;
 begin for(k=0;k<21;k=k+1)row[k*8+:8]=value[(20-k)*8+:8];end
 endfunction
 function [167:0] label;
 input [2:0] group_value;input [3:0] index;
 begin label="                     ";case(group_value)
 0:case(index)
 0:label="DISPLAY              ";
 1:label="MANUAL               ";
 2:label="MOTOR TEST           ";
 3:label="SETTINGS             ";
 default:label="                     ";endcase
 1:case(index)
 0:label="SYSTEM STATUS        ";
 1:label="IMU ATTITUDE         ";
 2:label="TOF DISTANCE         ";
 3:label="ENCODER / SPEED      ";
 4:label="VOICE / MOTION       ";
 default:label="                     ";endcase
 2:case(index)
 0:label="FORWARD 20CM         ";
 1:label="BACKWARD 20CM        ";
 2:label="LEFT 90 DEG          ";
 3:label="RIGHT 90 DEG         ";
 4:label="STOP                 ";
 default:label="                     ";endcase
 3:case(index)
 0:label="LEFT FORWARD         ";
 1:label="LEFT REVERSE         ";
 2:label="RIGHT FORWARD        ";
 3:label="RIGHT REVERSE        ";
 4:label="BOTH FORWARD         ";
 5:label="BOTH REVERSE         ";
 6:label="ARC LEFT             ";
 7:label="ARC RIGHT            ";
 default:label="                     ";endcase
 4:case(index)
 0:label="VOICE ENABLE         ";
 1:label="SPEED GEAR           ";
 default:label="                     ";endcase
 default:label="                     ";endcase end
 endfunction
 function [7:0] hexchar;
 input [3:0] x;begin hexchar=x<10 ? 8'd48+x:8'd55+x;end
 endfunction
 function [63:0] hex32;
 input [31:0] x;integer k;
 begin for(k=0;k<8;k=k+1)hex32[(7-k)*8+:8]=hexchar(x[(7-k)*4+:4]);end
 endfunction
 function [47:0] signed_speed;
 input signed [31:0] value;reg [15:0] mag;reg [19:0] bcd;integer k,j;
 begin
 mag=value[31] ? (~value[15:0]+16'd1):value[15:0];bcd=0;
 for(k=15;k>=0;k=k-1)begin
 for(j=0;j<5;j=j+1)if(bcd[j*4+:4]>=5)bcd[j*4+:4]=bcd[j*4+:4]+4'd3;
 bcd={bcd[18:0],mag[k]};end
 signed_speed[47:40]=value<0 ? "-":"+";
 for(k=0;k<5;k=k+1)signed_speed[(4-k)*8+:8]=8'd48+bcd[(4-k)*4+:4];
 end endfunction
 wire [3:0] rh,rt,ro,rf,ph,pt,po,pf,yh,yt,yo,yf;
 wire rn,pn,yn;
 attitude_angle_to_digits rd(roll_mdeg,rn,rh,rt,ro,rf);
 attitude_angle_to_digits pd(pitch_mdeg,pn,ph,pt,po,pf);
 attitude_angle_to_digits yd(yaw_mdeg,yn,yh,yt,yo,yf);
 function [47:0] angle;
 input n;input [3:0] h,t,o,f;
 begin angle={n ? 8'h2d:8'h2b,8'd48+h,8'd48+t,8'd48+o,8'h2e,8'd48+f};end
 endfunction
 reg [47:0] roll_text,pitch_text,yaw_text,left_speed_text,right_speed_text;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
 roll_text<=0;pitch_text<=0;yaw_text<=0;left_speed_text<=0;right_speed_text<=0;
 end else begin
 roll_text<=angle(rn,rh,rt,ro,rf);pitch_text<=angle(pn,ph,pt,po,pf);yaw_text<=angle(yn,yh,yt,yo,yf);
 left_speed_text<=signed_speed(speed_l);right_speed_text<=signed_speed(speed_r);
 end
 reg [167:0] r0,r1,r2,r3,current_label;
 reg [3:0] next_item,last_item;
 always @* begin
 r0="                     ";r1=r0;r2=r0;r3=r0;current_label=label(group_id,item);
 case(group_id)0:last_item=3;1:last_item=4;2:last_item=4;3:last_item=7;default:last_item=1;endcase
 next_item=item==last_item ? 0:item+4'd1;
 if(!page)begin
 r0=group_id==0 ? "RPACS MENU           ":label(3'd0,{1'b0,group_id}-4'd1);
 r1=label(group_id,item);r1[167:160]=">";
 // Leave column zero for the cursor rather than overwriting the label.
 r1={8'h3e,current_label[167:8]};
 r2=label(group_id,next_item);
 r3=group_id==0 ? "K0 RST K1 NEXT K2 OK ":"K0 BACK K1 NEXT K2 OK";
 end else begin
 r0=label(group_id,item);
 r3=fault ? "FAULT K0 BACK/RESET  ":busy ? "RUNNING K0 STOP/BACK ":ready ? "READY K0 BACK K2 RUN ":"WAIT IMU CALIBRATION ";
 case(group_id)
 1:case(item)
 0:begin
 r1=fault ? "SYSTEM FAULT         ":ready ? "SYSTEM READY         ":"SYSTEM INITIALIZING  ";
 r2={"IMU:",hexchar(imu_error[7:4]),hexchar(imu_error[3:0])," SENSOR:",hexchar(sensor_error[7:4]),hexchar(sensor_error[3:0]),"     "};
 r3={"MOTION:",hexchar(motion_fault[7:4]),hexchar(motion_fault[3:0])," K0 BACK    "};
 end
 1:begin
 r1={"ROLL  ",roll_text," DEG     "};
 r2={"PITCH ",pitch_text," DEG     "};
 r3=attitude_valid ? {"YAW   ",yaw_text," DEG     "}:"CALIBRATING K0 BACK  ";
 end
 3:begin
 r0="HEX COUNTS V C/10MS  ";
 r1={"L:",hex32(pos_l)," V:",left_speed_text,"  "};
 r2={"R:",hex32(pos_r)," V:",right_speed_text,"  "};
 r3="K0 BACK              ";
 end
 4:begin
 r1=voice_enable ? "VOICE ON             ":"VOICE OFF            ";
 r2={"STATE:",hexchar(status[7:4]),hexchar(status[3:0])," CMD:",hexchar(command[7:4]),hexchar(command[3:0]),"      "};
 r3={"REPLY:",hexchar(response[7:4]),hexchar(response[3:0])," K0 BACK     "};
 end
 default:begin end
 endcase
 2:begin r1="K2 EXECUTE ONCE      ";r2=item==4 ? "STOP ANY ACTION      ":item<2 ? "GOAL 20CM            ":"GOAL 90 DEG          ";end
 3:begin r1="K2 START 8 SECOND    ";r2="FIXED BENCH SPEED    ";end
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
 default:begin end
 endcase
 end
 text={row(r3),row(r2),row(r1),row(r0)};
 if(page && group_id==1 && item==2)text=tof_text;
 end
endmodule
