`timescale 1ns/1ps
// Open-drain coupling probe. Never drives a sensor line high. Temporary SRAM test.
module tof_bus_probe_top(input wire clk_50m,reset_n,imu_gnd_reserved,
 input wire tof_front_gpio1,tof_left_gpio1,tof_right_gpio1,
 inout wire tof_front_scl,tof_front_sda,tof_front_xshut_n,
 inout wire tof_left_scl,tof_left_sda,tof_left_xshut_n,
 inout wire tof_right_scl,tof_right_sda,tof_right_xshut_n,
 output wire uart_tx,oled_scl,inout wire oled_sda,
 output wire PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY);
 reg [1:0] rst_pipe=0;
 always @(posedge clk_50m or negedge reset_n)
  if(!reset_n) rst_pipe<=0; else rst_pipe<={rst_pipe[0],1'b1};
 wire rst_n=rst_pipe[1];
 assign {PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY}=7'b0;
 assign oled_scl=1'b1; assign oled_sda=1'bz;
 reg [3:0] phase; reg [21:0] timer;
 wire low_scl=(phase==1 || phase==6);
 wire low_sda=(phase==3 || phase==8);
 wire low_shutdown=(phase<5);
 assign tof_front_scl=low_scl?1'b0:1'bz;
 assign tof_left_scl=low_scl?1'b0:1'bz;
 assign tof_right_scl=low_scl?1'b0:1'bz;
 assign tof_front_sda=low_sda?1'b0:1'bz;
 assign tof_left_sda=low_sda?1'b0:1'bz;
 assign tof_right_sda=low_sda?1'b0:1'bz;
 assign tof_front_xshut_n=low_shutdown?1'b0:1'bz;
 assign tof_left_xshut_n=low_shutdown?1'b0:1'bz;
 assign tof_right_xshut_n=low_shutdown?1'b0:1'bz;
 (* ASYNC_REG="TRUE" *) reg [8:0] pad_meta,pad_sync;
 reg [3:0] snap_phase; reg [8:0] snap_pads;
 reg active; reg [6:0] position;
 wire ready,done; wire [7:0] data=character(position);
 uart_tx serial(.clk(clk_50m),.rst_n(rst_n),.data(data),.valid(active),.ready(ready),.tx(uart_tx),.done(done));
 function [7:0] hx; input [3:0] v; begin hx=v<10?48+v:55+v; end endfunction
 function [7:0] character; input [6:0] pos; begin case(pos)
  0: character=8'h50;
  1: character=8'h52;
  2: character=8'h4f;
  3: character=8'h42;
  4: character=8'h45;
  5: character=8'h20;
  6: character=8'h70;
  7: character=8'h68;
  8: character=8'h61;
  9: character=8'h73;
  10: character=8'h65;
  11: character=8'h3d;
  12: character=hx(snap_phase);
  13: character=8'h20;
  14: character=8'h46;
  15: character=8'h3d;
  16: character=hx({1'b0,snap_pads[2:0]});
  17: character=8'h20;
  18: character=8'h4c;
  19: character=8'h3d;
  20: character=hx({1'b0,snap_pads[5:3]});
  21: character=8'h20;
  22: character=8'h52;
  23: character=8'h3d;
  24: character=hx({1'b0,snap_pads[8:6]});
  25: character=8'h20;
  26: character=8'h28;
  27: character=8'h62;
  28: character=8'h69;
  29: character=8'h74;
  30: character=8'h73;
  31: character=8'h3a;
  32: character=8'h53;
  33: character=8'h43;
  34: character=8'h4c;
  35: character=8'h2c;
  36: character=8'h53;
  37: character=8'h44;
  38: character=8'h41;
  39: character=8'h2c;
  40: character=8'h58;
  41: character=8'h53;
  42: character=8'h48;
  43: character=8'h55;
  44: character=8'h54;
  45: character=8'h29;
  46: character=8'h0d;
  47: character=8'h0a;
 default:character=8'h20; endcase end endfunction
 always @(posedge clk_50m or negedge rst_n) begin
  if(!rst_n) begin
   phase<=0; timer<=0; pad_meta<=0; pad_sync<=0;
   snap_phase<=0; snap_pads<=0; active<=0; position<=0;
  end else begin
   pad_meta<={tof_right_scl,tof_right_sda,tof_right_xshut_n,
              tof_left_scl,tof_left_sda,tof_left_xshut_n,
              tof_front_scl,tof_front_sda,tof_front_xshut_n};
   pad_sync<=pad_meta;
   if(timer==2499999) begin timer<=0; phase<=phase==9?0:phase+1'b1; end
   else timer<=timer+1'b1;
   if(timer==2000000) begin snap_phase<=phase; snap_pads<=pad_sync; active<=1; position<=0; end
   else if(active && ready) begin
    if(position==47) begin active<=0; position<=0; end else position<=position+1'b1;
   end
  end
 end
endmodule
