`timescale 1ns/1ps
// Fixed eight-byte lines: S/E, source I/M/N/W/L/R/X, hex code, CRLF.
// Each source/code has its own pending bit; no FIFO, decimal conversion or telemetry timer.
module system_event_logger #(parameter integer CLK_HZ=50000000,BAUD=115200,WATCHDOG_PRESENT=1)(
 input wire clk,rst_n,imu_started,imu_failed,
 input wire [7:0] imu_error,input wire [11:0] errors,output wire tx,
 input wire [63:0] debug_state,input wire debug_valid);
 reg [15:0] pending,reported;
 reg imu_boot_seen;
 reg [7:0] imu_boot_code;
 reg busy; reg [3:0] selected; reg [4:0] index;
 reg debug_line,debug_overflow;
 reg [63:0] debug_previous,debug_record;
 reg [63:0] debug_fifo[0:31];
 reg [4:0] debug_wr,debug_rd;
 reg [5:0] debug_used;
 reg [31:0] heartbeat;
 reg [7:0] kind,source,code;
 reg [15:0] arrivals;
 integer j; reg found; reg [3:0] choice;
 wire ready,done;
 wire debug_enabled=(^debug_state)!==1'bx;
 wire debug_arrival=debug_enabled && (debug_state!=debug_previous || debug_valid===1'b1 || heartbeat==CLK_HZ-1);
 wire debug_pop=!busy && ready && !found && debug_used!=0;
 wire debug_push=debug_arrival && (debug_used<32 || debug_pop);
 reg [7:0] data;
 function [7:0] hexchar;input [3:0] n;begin hexchar=n<10 ? 8'd48+n:8'd65+n-10;end endfunction
 always @* begin
  arrivals={errors,4'b0000};
  if(!imu_boot_seen && (imu_started||imu_failed)) arrivals[3]=1;
  found=0;choice=0;
  for(j=0;j<16;j=j+1) if(pending[j] && !found) begin found=1;choice=j;end
  if(debug_line)begin
   if(index==0)data=debug_overflow ? "O":"D";
   else if(index==1)data=" ";
   else if(index==18)data=13;
   else if(index==19)data=10;
   else data=hexchar((debug_record >> ((17-index)*4)) & 4'hf);
  end else case(index)
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
  debug_line<=0;debug_overflow<=0;debug_previous<=0;debug_record<=0;heartbeat<=0;
  debug_wr<=0;debug_rd<=0;debug_used<=0;
 end else begin
  if(heartbeat==CLK_HZ-1)heartbeat<=0;else heartbeat<=heartbeat+1'b1;
  debug_previous<=debug_state;
  // A bounded event queue retains short request/ack transitions during TX.
  // Overflow is diagnostic only and never affects motor or ASR arbitration.
  if(debug_push)begin debug_fifo[debug_wr]<=debug_state;debug_wr<=debug_wr+1'b1;end
  if(debug_arrival && !debug_push)debug_overflow<=1;
  if(debug_pop)debug_rd<=debug_rd+1'b1;
  debug_used<=debug_used+debug_push-debug_pop;
  pending<=pending|(arrivals & ~reported);
  if(!imu_boot_seen && (imu_started||imu_failed)) begin
   imu_boot_seen<=1;imu_boot_code<=imu_failed ? (imu_error!=0 ? imu_error:8'hff):0;
  end
  if(!busy && ready && found) begin
   selected<=choice;index<=0;busy<=1;debug_line<=0;
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
  end else if(debug_pop)begin
   debug_record<=debug_fifo[debug_rd];debug_line<=1;index<=0;busy<=1;
  end else if(busy && ready) begin
   if(index==(debug_line ? 19:7)) begin
    busy<=0;
    if(!debug_line)begin reported[selected]<=1;pending[selected]<=0;end
   end else index<=index+1'b1;
  end
 end
 end
endmodule
