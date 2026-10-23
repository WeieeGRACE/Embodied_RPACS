`timescale 1ns/1ps
// Candidate buffer retains overlapping AA suffixes and the ORIGINAL deadline.
// tick and byte_start_tick are in the same clock domain, unsigned wrap safe.
module asr_frame_rx #(parameter integer TIMEOUT_CYCLES=1000000)(
 input wire clk,rst_n,input wire [31:0] tick,byte_start_tick,
 input wire [7:0] data,input wire valid,frame_error,
 output reg command_valid,output reg [7:0] command,
 output reg [31:0] good_count,bad_count,timeout_count);
 reg [47:0] candidate; reg [2:0] count; reg [31:0] first_tick;
 reg [47:0] assembled; integer j,keep;
 function known_command; input [7:0] c;
  begin known_command=(c==8'h01 || c==8'h02 || c==8'h03 ||
                       (c>=8'h10 && c<=8'h15)); end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin candidate<=0; count<=0; first_tick<=0;
   command_valid<=0; command<=0; good_count<=0; bad_count<=0; timeout_count<=0; end
  else begin
   command_valid<=0;
   if(frame_error) begin count<=0; candidate<=0; bad_count<=bad_count+1'b1; end
   else if(count!=0 && (tick-first_tick)>=TIMEOUT_CYCLES) begin
    count<=0; candidate<=0; timeout_count<=timeout_count+1'b1;
    if(valid && data==8'hAA) begin
     candidate<=48'hAA; count<=1; first_tick<=byte_start_tick;
    end
   end else if(valid) begin
    if(count==0) begin
     if(data==8'hAA) begin candidate<=48'hAA; count<=1; first_tick<=byte_start_tick; end
    end else begin
     assembled={candidate[39:0],data};
     if(count==5) begin
      if(assembled[47:32]==16'hAA55 && assembled[31:24]==8'h01 &&
         known_command(assembled[23:16]) &&
         assembled[15:8]==(8'h01+assembled[23:16]) && assembled[7:0]==8'h0D) begin
       command<=assembled[23:16]; command_valid<=1; good_count<=good_count+1'b1;
       candidate<=0; count<=0;
      end else begin
       bad_count<=bad_count+1'b1;
       // Earliest AA in suffix wins; do not extend first_tick on overlap.
       keep=0;
       for(j=1;j<6;j=j+1)
        if(keep==0 && ((assembled >> ((5-j)*8)) & 48'hFF)==48'hAA) keep=6-j;
       candidate<=assembled; count<=keep;
      end
     end else begin candidate<=assembled; count<=count+1'b1; end
    end
   end
  end
 end
endmodule
