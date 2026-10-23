`timescale 1ns/1ps
// Test adapter ONLY. State completion is injected, never inferred from elapsed time.
module asr_test_control(
 input wire clk,rst_n,command_valid,input wire [7:0] command,
 input wire step,fault,inject_valid,input wire [7:0] inject_status,
 output reg [7:0] status,output reg [1:0] gear,
 output reg event_valid,event_pair,output reg [7:0] response,last_command,
 output reg [31:0] debug_error_count);
 wire busy=(status==8'h01 || status==8'h02 || status==8'h10);
 wire locked=(status==8'h06);
 function known_status; input [7:0] s;
  begin known_status=(s<=8'h06 || s==8'h10 || s==8'h11);end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin status<=0;gear<=1;event_valid<=0;event_pair<=0;
   response<=0;last_command<=0;debug_error_count<=0;end
  else begin
   event_valid<=0;event_pair<=0;
   // All sources are resolved in one place: fault > command > injected state > key.
   if(command_valid) begin
    event_valid<=1;event_pair<=1;response<=0;last_command<=command;
    if(fault || locked) begin
     status<=6;
     if(command!=8'h02 && command!=8'h03) response<=2;
    end else case(command)
     8'h02:status<=4;
     8'h03:;
     8'h14:if(gear<2) gear<=gear+1'b1;
     8'h15:if(gear>0) gear<=gear-1'b1;
     default:if(busy) response<=1;
      else if(command==1) status<=1; else status<=8'h10;
    endcase
   end else if(fault && !locked) begin status<=6;event_valid<=1;end
   else if(inject_valid) begin
    if(known_status(inject_status) && (!locked || inject_status==6)) begin
     if(status!=inject_status) begin status<=inject_status;event_valid<=1;end
    end else debug_error_count<=debug_error_count+1'b1;
   end else if(step && !locked) begin
    case(status)
     1:begin status<=2;event_valid<=1;end
     2:begin status<=3;event_valid<=1;end
     8'h10:begin status<=8'h11;event_valid<=1;end
     default:;
    endcase
   end
   // A coincident state request is explicitly rejected rather than silently queued.
   if(inject_valid && (command_valid || fault)) debug_error_count<=debug_error_count+1'b1;
  end
 end
endmodule
