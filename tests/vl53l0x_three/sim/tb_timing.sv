`timescale 1ns/1ps
// Actual production clock and default parameters, not scaled time constants.
module tb_timing;
 reg clk=0; always #10 clk=~clk;
 reg rst_n=0,trigger=0; tri1 scl,sda,xshut;
 wire busy,done,valid,gpio; wire [7:0] error,model_id,revision,pre,finalp;
 wire [15:0] mm,ambient,signal_count; wire [3:0] status;
 integer shots,polls,blocks,resets; reg completed=0;
 tof_driver dut(.clk(clk),.rst_n(rst_n),.trigger(trigger),.gpio1(1'b1),
  .scl(scl),.sda(sda),.xshut_n(xshut),.gpio_sync(gpio),.busy(busy),.done(done),
  .sample_valid(valid),.error(error),.distance_mm(mm),.ambient(ambient),.signal_count(signal_count),
  .model_id(model_id),.revision(revision),.pre_period(pre),.final_period(finalp),.range_status(status));
 vl53l0x_model model(.scl(scl),.sda(sda),.xshut_n(xshut),.present(1'b1),
  .nack_data(1'b0),.never_ready(1'b0),.distance_mm(16'd1000),.status(4'd3),.ready_after(1),
  .shots(shots),.polls(polls),.read_blocks(blocks),.reset_count(resets));
 time reset_release,xshut_release,boot_finished,last_clock=0,period_value;
 integer clock_checks=0;
 always @(negedge scl) if(rst_n) begin
  // Only count successive data-bit cycles; transaction boundaries are longer.
  if(last_clock!=0) begin
   period_value=$time-last_clock;
   if(period_value<10000) $fatal(1,"I2C faster than 100 kHz: %0t",period_value);
   // Positive SDA hold is included in the low phase; allow transaction boundaries.
   if(period_value<15000) clock_checks++;
  end
  last_clock=$time;
 end
 initial begin
  repeat(4) @(negedge clk); rst_n=1; reset_release=$time;
  @(posedge xshut); xshut_release=$time;
  if(xshut_release-reset_release<999980 || xshut_release-reset_release>1000020)
   $fatal(1,"XSHUT must remain low 1 ms");
  @(negedge busy); boot_finished=$time;
  if(boot_finished-xshut_release!=20000000) $fatal(1,"Boot wait must be 20 ms");
  @(negedge clk); trigger=1; @(negedge clk); trigger=0;
  wait(done); #1;
  if(!valid || error!=0 || mm!=1000 || clock_checks<100)
   $fatal(1,"Production timing result valid=%b error=%h mm=%0d clocks=%0d",valid,error,mm,clock_checks);
  $display("PASS hardware defaults: XSHUT 1ms, boot 20ms, I2C <=100kHz, successful measurement");
  completed=1; $finish;
 end
 initial begin #100000000; $fatal(1,"Production timing watchdog"); end
endmodule
