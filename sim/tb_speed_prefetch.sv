`timescale 1ns/1ps
module speed_prefetch_case #(parameter W=2, L=3, PAD=0)(output reg completed=0);
 reg clk=0, rst=0;
 always #5 clk=~clk;
 reg signed [15:0] delta=0;
 wire signed [31:0] raw_speed,filtered,position;
 wire valid;
 speed_measurement_and_velocity_filter #(.WINDOW_CYCLES(W),.FILTER_LENGTH(L),
 .ZERO_PAD_WARMUP(PAD)) dut(.clk(clk),.rst_n(rst),.delta_count(delta),
 .speed_raw(raw_speed),.speed_filtered(filtered),.position(position),
 .speed_valid(valid),.direction(),.zero_speed());
 integer history[0:L-1];
 integer window_ticks=0,window_total=0,filled=0,pos=0;
 integer expected_raw=0,expected_filtered=0,total,divisor,j,k,epoch;
 always @(posedge clk) begin
  if(!rst) begin
   window_ticks=0;window_total=0;filled=0;pos=0;
   expected_raw=0;expected_filtered=0;
   for(j=0;j<L;j=j+1)history[j]=0;
   #1;if(valid || filtered || raw_speed || position)$fatal(1,"reset W=%0d L=%0d",W,L);
  end else begin
   pos=pos+$signed(delta);window_total=window_total+$signed(delta);
   window_ticks=window_ticks+1;
   if(window_ticks==W) begin
    for(j=L-1;j>0;j=j-1)history[j]=history[j-1];
    history[0]=window_total;
    if(filled<L)filled=filled+1;
    total=0;for(j=0;j<L;j=j+1)total=total+history[j];
    divisor=PAD ? L : filled;
    expected_raw=window_total;expected_filtered=total/divisor;
    window_total=0;window_ticks=0;
    #1;if(!valid)$fatal(1,"missing valid");
   end else begin
    #1;if(valid)$fatal(1,"unexpected valid");
   end
   if(raw_speed!==expected_raw || filtered!==expected_filtered || position!==pos)
    $fatal(1,"filter W=%0d L=%0d PAD=%0d raw=%0d/%0d mean=%0d/%0d position=%0d/%0d",
     W,L,PAD,raw_speed,expected_raw,filtered,expected_filtered,position,pos);
  end
 end
 initial begin
  for(epoch=0;epoch<2;epoch=epoch+1)begin
   @(negedge clk);rst=0;repeat(3)@(negedge clk);rst=1;
   for(k=0;k<200;k=k+1)begin
    // Includes signed extremes, negative rounding and repeated history replacement.
    case(k%7)
     0:delta=-32768;1:delta=32767;2:delta=-3;3:delta=2;
     4:delta=0;default:delta=(k*7919)%65536-32768;
    endcase
    @(negedge clk);
   end
  end
  completed=1;
 end
endmodule
module tb_speed_prefetch;
 wire [5:0] done;
 reg completed=0;
 speed_prefetch_case #(1,1,0) a(done[0]);
 speed_prefetch_case #(1,3,0) b(done[1]);
 speed_prefetch_case #(2,1,0) c(done[2]);
 speed_prefetch_case #(2,3,0) d(done[3]);
 speed_prefetch_case #(4,4,0) e(done[4]);
 speed_prefetch_case #(4,4,1) f(done[5]);
 initial begin
  wait(&done);completed=1;
  $display("PASS speed history prefetch: windows 1/2/4, lengths 1/3/4, both warmup modes, signed extremes/rounding and reset");
  $finish;
 end
 initial begin #100000;$fatal(1,"speed prefetch timeout");end
endmodule
