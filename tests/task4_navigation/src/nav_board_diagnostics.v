`timescale 1ns/1ps
// Fixed-length hexadecimal diagnostics, snapshotted once per second.
// RST=1 means the synchronized internal reset has been released.
module nav_board_diagnostics #(
 parameter integer CLK_HZ=50000000,BAUD=115200,
 parameter [31:0] BUILD_ID=32'h26102702
)(input wire clk,rst_n,key_enable_n,key_estop_n,estop_latched,
 input wire [31:0] frame_count,
 input wire [47:0] raw_mm,accepted_mm,ages,
 input wire [23:0] errors,input wire [11:0] statuses,
 input wire [2:0] valid,seen,input wire [7:0] nav_fault,
 input wire pose_fault,odometry_fault,imu_valid,encoder_valid,
 output reg [671:0] oled_text,output wire tx);
 function [7:0] hx;input [3:0] n;begin hx=n<10 ? 48+n:55+n;end endfunction
 integer i,j;
 reg [167:0] row;
 always @* begin
   oled_text={84{8'h20}};
   row="T4 DIAG 26102702      ";
   for(j=0;j<21;j=j+1) oled_text[j*8+:8]=row[167-j*8-:8];
   for(j=0;j<8;j=j+1) oled_text[(8+j)*8+:8]=hx(BUILD_ID[(7-j)*4+:4]);
   for(i=0;i<3;i=i+1) begin
     row=i==0 ? "F R0000 E00 A0000    ":i==1 ? "L R0000 E00 A0000    ":"R R0000 E00 A0000    ";
     for(j=0;j<21;j=j+1) oled_text[((i+1)*21+j)*8+:8]=row[167-j*8-:8];
     for(j=0;j<4;j=j+1) begin
       oled_text[((i+1)*21+3+j)*8+:8]=hx(raw_mm[i*16+(3-j)*4+:4]);
       oled_text[((i+1)*21+12+j)*8+:8]=hx(ages[i*16+(3-j)*4+:4]);
     end
     oled_text[((i+1)*21+9)*8+:8]=hx(errors[i*8+4+:4]);
     oled_text[((i+1)*21+10)*8+:8]=hx(errors[i*8+:4]);
     oled_text[((i+1)*21+17)*8+:8]=hx({3'b0,valid[i]});
   end
   oled_text[(21+19)*8+:8]=hx({2'b0,key_enable_n,key_estop_n});
   oled_text[(42+19)*8+:8]=hx({3'b0,estop_latched});
   oled_text[(63+19)*8+:8]=hx(frame_count[3:0]);
 end
 reg [31:0] timer;
 reg pending,active;
 reg [7:0] pos;
 reg [2047:0] snapshot;
 reg [2047:0] message;
 wire ready,done;
 wire send=active && ready;
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) uart(
   .clk(clk),.rst_n(rst_n),.data(snapshot[pos*8+:8]),.valid(send),.ready(ready),.tx(tx),.done(done));
 always @* begin
   message={256{8'h20}};
   message[0+:8]=8'd84;
   message[8+:8]=8'd52;
   message[16+:8]=8'd68;
   message[24+:8]=8'd32;
   message[32+:8]=8'd66;
   message[40+:8]=8'd85;
   message[48+:8]=8'd73;
   message[56+:8]=8'd76;
   message[64+:8]=8'd68;
   message[72+:8]=8'd61;
   message[80+:8]=hx((BUILD_ID) >> 28);
   message[88+:8]=hx((BUILD_ID) >> 24);
   message[96+:8]=hx((BUILD_ID) >> 20);
   message[104+:8]=hx((BUILD_ID) >> 16);
   message[112+:8]=hx((BUILD_ID) >> 12);
   message[120+:8]=hx((BUILD_ID) >> 8);
   message[128+:8]=hx((BUILD_ID) >> 4);
   message[136+:8]=hx((BUILD_ID) >> 0);
   message[144+:8]=8'd32;
   message[152+:8]=8'd82;
   message[160+:8]=8'd83;
   message[168+:8]=8'd84;
   message[176+:8]=8'd61;
   message[184+:8]=8'd49;
   message[192+:8]=8'd32;
   message[200+:8]=8'd75;
   message[208+:8]=8'd69;
   message[216+:8]=8'd89;
   message[224+:8]=8'd61;
   message[232+:8]=hx(({2'b0,key_enable_n,key_estop_n}) >> 0);
   message[240+:8]=8'd32;
   message[248+:8]=8'd69;
   message[256+:8]=8'd83;
   message[264+:8]=8'd84;
   message[272+:8]=8'd61;
   message[280+:8]=hx(({3'b0,estop_latched}) >> 0);
   message[288+:8]=8'd32;
   message[296+:8]=8'd70;
   message[304+:8]=8'd82;
   message[312+:8]=8'd65;
   message[320+:8]=8'd77;
   message[328+:8]=8'd69;
   message[336+:8]=8'd61;
   message[344+:8]=hx((frame_count) >> 28);
   message[352+:8]=hx((frame_count) >> 24);
   message[360+:8]=hx((frame_count) >> 20);
   message[368+:8]=hx((frame_count) >> 16);
   message[376+:8]=hx((frame_count) >> 12);
   message[384+:8]=hx((frame_count) >> 8);
   message[392+:8]=hx((frame_count) >> 4);
   message[400+:8]=hx((frame_count) >> 0);
   message[408+:8]=8'd32;
   message[416+:8]=8'd70;
   message[424+:8]=8'd65;
   message[432+:8]=8'd85;
   message[440+:8]=8'd76;
   message[448+:8]=8'd84;
   message[456+:8]=8'd61;
   message[464+:8]=hx((nav_fault) >> 4);
   message[472+:8]=hx((nav_fault) >> 0);
   message[480+:8]=8'd32;
   message[488+:8]=8'd73;
   message[496+:8]=8'd78;
   message[504+:8]=8'd80;
   message[512+:8]=8'd85;
   message[520+:8]=8'd84;
   message[528+:8]=8'd61;
   message[536+:8]=hx(({pose_fault,odometry_fault,imu_valid,encoder_valid}) >> 0);
   message[544+:8]=8'd32;
   message[552+:8]=8'd86;
   message[560+:8]=8'd65;
   message[568+:8]=8'd76;
   message[576+:8]=8'd73;
   message[584+:8]=8'd68;
   message[592+:8]=8'd61;
   message[600+:8]=hx(({1'b0,valid}) >> 0);
   message[608+:8]=8'd32;
   message[616+:8]=8'd83;
   message[624+:8]=8'd69;
   message[632+:8]=8'd69;
   message[640+:8]=8'd78;
   message[648+:8]=8'd61;
   message[656+:8]=hx(({1'b0,seen}) >> 0);
   message[664+:8]=8'd32;
   message[672+:8]=8'd70;
   message[680+:8]=8'd58;
   message[688+:8]=8'd82;
   message[696+:8]=8'd65;
   message[704+:8]=8'd87;
   message[712+:8]=8'd61;
   message[720+:8]=hx((raw_mm[0+:16]) >> 12);
   message[728+:8]=hx((raw_mm[0+:16]) >> 8);
   message[736+:8]=hx((raw_mm[0+:16]) >> 4);
   message[744+:8]=hx((raw_mm[0+:16]) >> 0);
   message[752+:8]=8'd47;
   message[760+:8]=8'd65;
   message[768+:8]=8'd67;
   message[776+:8]=8'd67;
   message[784+:8]=8'd61;
   message[792+:8]=hx((accepted_mm[0+:16]) >> 12);
   message[800+:8]=hx((accepted_mm[0+:16]) >> 8);
   message[808+:8]=hx((accepted_mm[0+:16]) >> 4);
   message[816+:8]=hx((accepted_mm[0+:16]) >> 0);
   message[824+:8]=8'd47;
   message[832+:8]=8'd65;
   message[840+:8]=8'd71;
   message[848+:8]=8'd69;
   message[856+:8]=8'd61;
   message[864+:8]=hx((ages[0+:16]) >> 12);
   message[872+:8]=hx((ages[0+:16]) >> 8);
   message[880+:8]=hx((ages[0+:16]) >> 4);
   message[888+:8]=hx((ages[0+:16]) >> 0);
   message[896+:8]=8'd47;
   message[904+:8]=8'd69;
   message[912+:8]=8'd82;
   message[920+:8]=8'd82;
   message[928+:8]=8'd61;
   message[936+:8]=hx((errors[0+:8]) >> 4);
   message[944+:8]=hx((errors[0+:8]) >> 0);
   message[952+:8]=8'd47;
   message[960+:8]=8'd83;
   message[968+:8]=8'd84;
   message[976+:8]=8'd61;
   message[984+:8]=hx(({4'b0,statuses[0+:4]}) >> 4);
   message[992+:8]=hx(({4'b0,statuses[0+:4]}) >> 0);
   message[1000+:8]=8'd32;
   message[1008+:8]=8'd76;
   message[1016+:8]=8'd58;
   message[1024+:8]=8'd82;
   message[1032+:8]=8'd65;
   message[1040+:8]=8'd87;
   message[1048+:8]=8'd61;
   message[1056+:8]=hx((raw_mm[16+:16]) >> 12);
   message[1064+:8]=hx((raw_mm[16+:16]) >> 8);
   message[1072+:8]=hx((raw_mm[16+:16]) >> 4);
   message[1080+:8]=hx((raw_mm[16+:16]) >> 0);
   message[1088+:8]=8'd47;
   message[1096+:8]=8'd65;
   message[1104+:8]=8'd67;
   message[1112+:8]=8'd67;
   message[1120+:8]=8'd61;
   message[1128+:8]=hx((accepted_mm[16+:16]) >> 12);
   message[1136+:8]=hx((accepted_mm[16+:16]) >> 8);
   message[1144+:8]=hx((accepted_mm[16+:16]) >> 4);
   message[1152+:8]=hx((accepted_mm[16+:16]) >> 0);
   message[1160+:8]=8'd47;
   message[1168+:8]=8'd65;
   message[1176+:8]=8'd71;
   message[1184+:8]=8'd69;
   message[1192+:8]=8'd61;
   message[1200+:8]=hx((ages[16+:16]) >> 12);
   message[1208+:8]=hx((ages[16+:16]) >> 8);
   message[1216+:8]=hx((ages[16+:16]) >> 4);
   message[1224+:8]=hx((ages[16+:16]) >> 0);
   message[1232+:8]=8'd47;
   message[1240+:8]=8'd69;
   message[1248+:8]=8'd82;
   message[1256+:8]=8'd82;
   message[1264+:8]=8'd61;
   message[1272+:8]=hx((errors[8+:8]) >> 4);
   message[1280+:8]=hx((errors[8+:8]) >> 0);
   message[1288+:8]=8'd47;
   message[1296+:8]=8'd83;
   message[1304+:8]=8'd84;
   message[1312+:8]=8'd61;
   message[1320+:8]=hx(({4'b0,statuses[4+:4]}) >> 4);
   message[1328+:8]=hx(({4'b0,statuses[4+:4]}) >> 0);
   message[1336+:8]=8'd32;
   message[1344+:8]=8'd82;
   message[1352+:8]=8'd58;
   message[1360+:8]=8'd82;
   message[1368+:8]=8'd65;
   message[1376+:8]=8'd87;
   message[1384+:8]=8'd61;
   message[1392+:8]=hx((raw_mm[32+:16]) >> 12);
   message[1400+:8]=hx((raw_mm[32+:16]) >> 8);
   message[1408+:8]=hx((raw_mm[32+:16]) >> 4);
   message[1416+:8]=hx((raw_mm[32+:16]) >> 0);
   message[1424+:8]=8'd47;
   message[1432+:8]=8'd65;
   message[1440+:8]=8'd67;
   message[1448+:8]=8'd67;
   message[1456+:8]=8'd61;
   message[1464+:8]=hx((accepted_mm[32+:16]) >> 12);
   message[1472+:8]=hx((accepted_mm[32+:16]) >> 8);
   message[1480+:8]=hx((accepted_mm[32+:16]) >> 4);
   message[1488+:8]=hx((accepted_mm[32+:16]) >> 0);
   message[1496+:8]=8'd47;
   message[1504+:8]=8'd65;
   message[1512+:8]=8'd71;
   message[1520+:8]=8'd69;
   message[1528+:8]=8'd61;
   message[1536+:8]=hx((ages[32+:16]) >> 12);
   message[1544+:8]=hx((ages[32+:16]) >> 8);
   message[1552+:8]=hx((ages[32+:16]) >> 4);
   message[1560+:8]=hx((ages[32+:16]) >> 0);
   message[1568+:8]=8'd47;
   message[1576+:8]=8'd69;
   message[1584+:8]=8'd82;
   message[1592+:8]=8'd82;
   message[1600+:8]=8'd61;
   message[1608+:8]=hx((errors[16+:8]) >> 4);
   message[1616+:8]=hx((errors[16+:8]) >> 0);
   message[1624+:8]=8'd47;
   message[1632+:8]=8'd83;
   message[1640+:8]=8'd84;
   message[1648+:8]=8'd61;
   message[1656+:8]=hx(({4'b0,statuses[8+:4]}) >> 4);
   message[1664+:8]=hx(({4'b0,statuses[8+:4]}) >> 0);
   message[1672+:8]=13;message[1680+:8]=10;
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin timer<=0;pending<=1;active<=0;pos<=0;snapshot<=0;end
   else begin
     if(timer>=CLK_HZ-1) begin timer<=0;pending<=1;end else timer<=timer+1'b1;
     if(pending && !active && ready) begin snapshot<=message;active<=1;pending<=0;pos<=0;end
     if(send) begin if(pos==8'd210) begin active<=0;pos<=0;end else pos<=pos+1'b1;end
   end
 end
endmodule
