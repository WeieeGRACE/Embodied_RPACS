`timescale 1ns/1ps
// Encoder glitch tolerance check for the obstacle demo build.
// A direction reversal on a real chassis can couple short EMI spikes into the
// A/B lines; the 2026-10-14 board log recorded two illegal quadrature jumps on
// both channels during one test. The main project now instantiates the decoder
// with ILLEGAL_TOLERANCE=5 / CLEAN_DECAY=256 so isolated glitches are ignored
// while a genuinely broken encoder still latches quickly. Parameter 0 keeps the
// historical immediate-latch behaviour for the legacy tops.
module tb_encoder_glitch_tolerance;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0;reg [1:0] ab=0;
 reg clear_fault=0;
 wire signed [31:0] pos_tol,pos_strict;
 wire signed [1:0] delta_tol;
 wire [31:0] illegal_tol,illegal_strict;
 wire fault_tol,fault_strict;
 reg completed=0;integer checks=0;
 dual_ab_quadrature_encoder #(.FILTER_CYCLES(2),.INVERT_L(0),.INVERT_R(0),
 .ILLEGAL_TOLERANCE(5),.CLEAN_DECAY(64)) tolerant(
 .clk(clk),.rst_n(rst_n),.encoder_a_l(ab[1]),.encoder_b_l(ab[0]),.encoder_a_r(1'b0),.encoder_b_r(1'b0),
 .clear_fault_l(clear_fault),.clear_fault_r(clear_fault),
 .position_l(pos_tol),.position_r(),.delta_count_l(delta_tol),.delta_count_r(),
 .direction_l(),.direction_r(),.illegal_transition_l(illegal_tol),.illegal_transition_r(),
 .encoder_fault_l(fault_tol),.encoder_fault_r());
 dual_ab_quadrature_encoder #(.FILTER_CYCLES(2),.INVERT_L(0),.INVERT_R(0),
 .ILLEGAL_TOLERANCE(0)) strict(
 .clk(clk),.rst_n(rst_n),.encoder_a_l(ab[1]),.encoder_b_l(ab[0]),.encoder_a_r(1'b0),.encoder_b_r(1'b0),
 .clear_fault_l(clear_fault),.clear_fault_r(clear_fault),
 .position_l(pos_strict),.position_r(),.delta_count_l(),.delta_count_r(),
 .direction_l(),.direction_r(),.illegal_transition_l(illegal_strict),.illegal_transition_r(),
 .encoder_fault_l(fault_strict),.encoder_fault_r());
 task check;input condition;input [511:0] msg;
 begin checks=checks+1;if(condition!==1'b1)begin $display("FAIL %0s t=%0t pos=%0d illegal=%0d",msg,$time,pos_tol,illegal_tol);$fatal(1);end end endtask
 task cycles;input integer n;integer q;begin for(q=0;q<n;q=q+1)begin @(posedge clk);#1;end end endtask
 // Present a stable A/B state long enough for the 2-cycle filter to accept it.
 task state;input [1:0] v;begin ab=v;cycles(6);end endtask
 // One legal forward step sequence (00 -> 10 -> 11 -> 01 -> 00 = four counts).
 task fwd;begin state(2'b10);state(2'b11);state(2'b01);state(2'b00);end endtask
 // One illegal 00 -> 11 jump followed by two legal backward steps, i.e. a
 // short true glitch that returns to the 00 bus state.
 task glitch;begin state(2'b11);state(2'b10);state(2'b00);end endtask
 integer i,pos_start;
 initial begin
 repeat(3)@(negedge clk);rst_n=1;cycles(6);
 // Legal quadrature movement never faults either instance.
 for(i=0;i<8;i=i+1)fwd;
 check(!fault_tol && !fault_strict,"legal steps must not fault");
 check(pos_tol==32 && pos_strict==32,"legal position");
 pos_start=pos_tol;
 // One isolated glitch: tolerated, strict historical model latches immediately.
 glitch;
 check(!fault_tol,"isolated glitch tolerated");
 check(fault_strict,"strict instance still latches immediately");
 check(illegal_tol==1 && pos_tol==pos_start-2,"isolated glitch counted, recovery steps applied");
 // With tolerance 5 / decay 64 the first four glitches stay under the limit.
 glitch;glitch;glitch;
 check(!fault_tol && illegal_tol==4,"four glitches still under tolerance");
 glitch;
 check(fault_tol,"fifth glitch reaches tolerance and latches");
 check(illegal_tol==5,"illegal counter accumulates");
 // Clear, then verify the leaky decay: three glitches (3 hits, 6 clean steps)
 // decay to zero after enough clean steps, so later isolated glitches are safe.
 @(negedge clk);clear_fault=1;cycles(2);@(negedge clk);clear_fault=0;cycles(2);
 check(!fault_tol && illegal_tol==0,"clear resets tolerance state");
 glitch;glitch;glitch;
 for(i=0;i<60;i=i+1)fwd;
 check(!fault_tol,"clean steps leak tolerated hits");
 glitch;glitch;glitch;
 check(!fault_tol,"decayed tolerance allows later isolated glitches");
 glitch;glitch;
 check(fault_tol,"repeated glitches still latch after decay");
 // Async reset returns to the safe state.
 rst_n=0;#3;check(!fault_tol && illegal_tol==0 && pos_tol==0,"reset");
 completed=1;
 $display("PASS encoder glitch tolerance: isolated jumps tolerated while repeated jumps latch (checks=%0d)",checks);
 $finish;
 end
 initial begin #500000;$display("FAIL timeout");$fatal(1);end
endmodule
