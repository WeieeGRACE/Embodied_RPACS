`timescale 1ns/1ps
// Fixed eight-byte lines: S/E, source I/M/N/W/L/R/X, hex code, CRLF.
// Each source/code has its own pending bit; no FIFO, decimal conversion or telemetry timer.
module system_event_logger #(parameter integer CLK_HZ=50000000,BAUD=115200,WATCHDOG_PRESENT=1)(
 input wire clk,rst_n,imu_started,imu_failed,
 input wire [7:0] imu_error,input wire [11:0] errors,output wire tx);
 reg [15:0] pending,reported;
 reg imu_boot_seen;
 reg [7:0] imu_boot_code;
 reg busy; reg [3:0] selected; reg [2:0] index;
 reg [7:0] kind,source,code;
 reg [15:0] arrivals;
 integer j; reg found; reg [3:0] choice;
 wire ready,done;
 reg [7:0] data;
 function [7:0] hexchar;input [3:0] n;begin hexchar=n<10 ? 8'd48+n:8'd65+n-10;end endfunction
 always @* begin
  arrivals={errors,4'b0000};
  if(!imu_boot_seen && (imu_started||imu_failed)) arrivals[3]=1;
  found=0;choice=0;
  for(j=0;j<16;j=j+1) if(pending[j] && !found) begin found=1;choice=j;end
  case(index)
   0:data=kind;1:data=8'h20;2:data=source;3:data=8'h20;
   4:data=hexchar(code[7:4]);5:data=hexchar(code[3:0]);6:data=13;default:data=10;
  endcase
 end
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) serial(
 .clk(clk),.rst_n(rst_n),.data(data),.valid(busy),.ready(ready),.tx(tx),.done(done));
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin
  pending<=WATCHDOG_PRESENT ? 16'h0007:16'h0003;reported<=0;imu_boot_seen<=0;imu_boot_code<=0;
  busy<=0;selected<=0;index<=0;kind<=0;source<=0;code<=0;
 end else begin
  pending<=pending|(arrivals & ~reported);
  if(!imu_boot_seen && (imu_started||imu_failed)) begin
   imu_boot_seen<=1;imu_boot_code<=imu_failed ? (imu_error!=0 ? imu_error:8'hff):0;
  end
  if(!busy && ready && found) begin
   selected<=choice;index<=0;busy<=1;
   kind<=choice<4 ? "S":"E";
   source<="I";code<=0;
   case(choice)
    0:source<="M";1:source<="N";2:source<="W";3:code<=imu_boot_code;
    4:code<=4;5:code<=5;6:code<=6;7:code<=7;
    8:code<=8'h0a;9:code<=8'h08;10:code<=8'h09;11:code<=8'hff;
    12:begin source<="W";code<=1;end
    13:begin source<="L";code<=1;end
    14:begin source<="R";code<=1;end
    15:begin source<="X";code<=1;end
   endcase
  end else if(busy && ready) begin
   if(index==7) begin
    busy<=0;reported[selected]<=1;pending[selected]<=0;
   end else index<=index+1'b1;
  end
 end
 end
endmodule
