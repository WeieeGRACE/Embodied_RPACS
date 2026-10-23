`timescale 1ns/1ps
module tb_protocol;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,rx=1;reg [31:0] tick=0;
 always @(posedge clk) if(!rst_n) tick<=0;else tick<=tick+1;
 wire [7:0] data,command;wire valid,frame_error,command_valid;
 wire [31:0] start_tick,good,bad,timeouts;
 asr_uart_rx receiver(.clk(clk),.rst_n(rst_n),.rx(rx),.tick(tick),.data(data),
  .valid(valid),.frame_error(frame_error),.start_tick(start_tick));
 asr_frame_rx parser(.clk(clk),.rst_n(rst_n),.tick(tick),.byte_start_tick(start_tick),
  .data(data),.valid(valid),.frame_error(frame_error),.command_valid(command_valid),
  .command(command),.good_count(good),.bad_count(bad),.timeout_count(timeouts));
 integer commands=0,bytes_seen=0,uart_errors=0;reg [7:0] last_command;
 always @(posedge clk) begin
  if(command_valid) begin commands=commands+1;last_command=command;end
  if(valid) bytes_seen=bytes_seen+1;
  if(frame_error) uart_errors=uart_errors+1;
 end
 task check(input bit yes,input string message);
  if(!yes) $fatal(1,"FAIL %s t=%0t",message,$time);
 endtask
 task send_byte(input [7:0] b,input bit stop_ok=1);
  @(negedge clk);rx=0;repeat(434) @(negedge clk);
  for(integer i=0;i<8;i++) begin rx=b[i];repeat(434) @(negedge clk);end
  rx=stop_ok;repeat(434) @(negedge clk);
 endtask
 task frame(input [7:0] c,input [7:0] t=1,input [7:0] checksum=8'hFF,input [7:0] tail=8'h0D);
  send_byte(8'hAA);send_byte(8'h55);send_byte(t);send_byte(c);
  send_byte(checksum==8'hFF ? t+c : checksum);send_byte(tail);repeat(5) @(negedge clk);
 endtask
 // Byte-level parser fixture isolates exact deadline and tick-wrap boundaries.
 reg unit_reset=0,uv=0,ue=0;reg [31:0] ut=0,ust=0;reg [7:0] ud=0;
 wire ucv;wire [7:0] uc;wire [31:0] ug,ub,uto;
 asr_frame_rx #(.TIMEOUT_CYCLES(100)) unit_parser(.clk(clk),.rst_n(unit_reset),
  .tick(ut),.byte_start_tick(ust),.data(ud),.valid(uv),.frame_error(ue),
  .command_valid(ucv),.command(uc),.good_count(ug),.bad_count(ub),.timeout_count(uto));
 task unit_byte(input [7:0] b,input [31:0] now_tick,input [31:0] started);
  @(negedge clk);ud=b;ut=now_tick;ust=started;uv=1;
  @(negedge clk);uv=0;
 endtask
 task unit_begin(input [31:0] start);
  @(negedge clk);unit_reset=0;uv=0;ut=start;repeat(2) @(negedge clk);unit_reset=1;
 endtask
 integer before_count,before_bytes;
 reg completed=0;
 initial begin
  repeat(5) @(negedge clk);rst_n=1;
  repeat(100) @(negedge clk);check(commands==0,"idle sends no command");
  // Below-half-bit start glitch must not emit a byte.
  before_bytes=bytes_seen;rx=0;repeat(80) @(negedge clk);rx=1;repeat(500) @(negedge clk);
  check(bytes_seen==before_bytes,"short start glitch rejected");
  for(integer c=1;c<=3;c++) frame(c);
  for(integer c=16;c<=21;c++) frame(c);
  check(commands==9 && good==9 && last_command==8'h15,"all nine command codes, real 50MHz 115200");
  before_count=commands;
  frame(1,1,0);frame(1,1,8'hFF,8'h0A);frame(1,2);frame(8'h7F);
  check(commands==before_count && bad>=4,"invalid checksum tail type and code rejected");
  send_byte(8'h77);send_byte(8'h23);send_byte(8'hAA);frame(3);
  check(commands==before_count+1,"noise and AA AA 55 overlap recovered");
  // A legal header embedded in a corrupt candidate must be retained.
  send_byte(8'hAA);send_byte(8'h55);send_byte(8'h99);frame(2);
  check(commands==before_count+2 && last_command==2,"embedded header recovered");
  send_byte(8'hAA);send_byte(8'h55);send_byte(8'h10,0);
  rx=0;repeat(6000) @(negedge clk);
  check(uart_errors==1,"break produces one framing error, no phantom bytes");
  rx=1;repeat(600) @(negedge clk);frame(3);
  check(commands==before_count+3,"recovery after bad stop and break");
  send_byte(8'hAA);repeat(900000) @(negedge clk);
  send_byte(8'h55);send_byte(1);send_byte(1);send_byte(2);send_byte(13);
  repeat(5) @(negedge clk);check(commands==before_count+4,"sub-20ms split frame accepted");
  send_byte(8'hAA);repeat(1000500) @(negedge clk);
  send_byte(8'h55);send_byte(1);send_byte(1);send_byte(2);send_byte(13);
  repeat(5) @(negedge clk);check(commands==before_count+4 && timeouts>=1,"20ms residual discarded");
  frame(1);check(commands==before_count+5,"good frame after timeout");
  // Deadline counts from START of first byte, not from its completion.
  unit_begin(0);
  unit_byte('hAA,10,0);unit_byte('h55,20,10);unit_byte(1,30,20);
  unit_byte(3,40,30);unit_byte(4,50,40);unit_byte(13,99,90);
  check(ug==1,"deadline minus one accepted");
  unit_begin(0);
  unit_byte('hAA,10,0);unit_byte('h55,20,10);unit_byte(1,30,20);
  unit_byte(3,40,30);unit_byte(4,50,40);unit_byte(13,100,90);
  check(ug==0 && uto==1,"exact deadline rejected");
  unit_begin(32'hFFFFFFF0);
  unit_byte('hAA,32'hFFFFFFFA,32'hFFFFFFF0);unit_byte('h55,4,32'hFFFFFFFA);
  unit_byte(1,14,4);unit_byte(3,24,14);unit_byte(4,34,24);unit_byte(13,83,74);
  check(ug==1,"unsigned tick wrap accepted before deadline");
  unit_begin(0);
  unit_byte('hAA,10,0);unit_byte('hAA,20,10);unit_byte('h55,30,20);
  unit_byte(1,40,30);unit_byte(3,50,40);unit_byte(4,60,50);unit_byte(13,100,90);
  check(ug==0 && uto==1,"overlap retains original deadline");
  // Reset during a residual must remove it.
  @(negedge clk);rst_n=0;repeat(5) @(negedge clk);rst_n=1;
  before_count=commands;send_byte('h55);send_byte(1);send_byte(1);send_byte(2);send_byte(13);
  check(commands==before_count,"reset cannot complete an old frame");
  $display("PASS protocol: physical UART, all codes, faults, resync, deadline boundaries and wrap");
  completed=1;$finish;
 end
 initial begin #100000000; $fatal(1,"protocol watchdog");end
endmodule
