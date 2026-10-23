`timescale 1ns/1ps
module tb_bus;
 reg clk=0; always #10 clk=~clk;
 reg rst_n=0,start=0; reg present=1,nack_data=0;
 reg hold_scl=0,hold_sda=0; tri1 scl,sda;
 assign scl=hold_scl?1'b0:1'bz; assign sda=hold_sda?1'b0:1'bz;
 wire done,busy; wire [7:0] error; wire [95:0] result;
 wire [15:0] distance_value=16'habcd;
 integer shots,polls,blocks,resets;
 reg completed=0;
 tof_i2c_reg #(.CLK_HZ(100000),.I2C_HZ(5000),.TIMEOUT_CYCLES(80)) dut(
  .clk(clk),.rst_n(rst_n),.start(start),.write_en(1'b0),.reg_addr(8'h14),
  .write_data(8'h01),.read_len(4'd12),.busy(busy),.done(done),.error(error),.rx_data(result),.scl(scl),.sda(sda));
 vl53l0x_model model(.scl(scl),.sda(sda),.xshut_n(rst_n),.present(present),
  .nack_data(nack_data),.never_ready(1'b0),.distance_mm(distance_value),.status(4'd3),
  .ready_after(0),.shots(shots),.polls(polls),.read_blocks(blocks),.reset_count(resets));
 task automatic reset_bus;
  begin
   @(negedge clk); rst_n=0; hold_scl=0; hold_sda=0; start=0;
   repeat(5) @(negedge clk); rst_n=1; repeat(5) @(negedge clk);
  end
 endtask
 task automatic read_expect(input integer expected);
  begin
   @(negedge clk); start=1; @(negedge clk); start=0;
   wait(done); @(negedge clk);
   if(error!=expected) $fatal(1,"I2C error got=%0d expected=%0d",error,expected);
   if(expected==0 && result[95:80]!==16'hcdab) $fatal(1,"Sequential read / last NACK");
  end
 endtask
 initial begin
  reset_bus(); read_expect(0);
  present=0; read_expect(1); present=1;
  nack_data=1; read_expect(1); nack_data=0;
  $display("PASS register transfers, sequential read, address/data NACK");
  reset_bus();
  fork
   read_expect(0);
   begin
    wait(dut.bits.state==7); @(negedge clk); hold_scl=1;
    repeat(40) @(negedge clk); hold_scl=0;
   end
  join
  $display("PASS short clock stretching");
  reset_bus();
  fork
   read_expect(3);
   begin wait(dut.bits.state==7); @(negedge clk); hold_scl=1; end
  join
  if(dut.bits.scl_low || dut.bits.sda_low) $fatal(1,"Timeout must release both outputs");
  reset_bus(); hold_sda=1; read_expect(3);
  if(dut.bits.scl_low || dut.bits.sda_low) $fatal(1,"Stuck bus must release both outputs");
  reset_bus(); read_expect(0);
  $display("PASS clock timeout, stuck SDA and recovery");
  completed=1; $finish;
 end
 initial begin #3000000; $fatal(1,"Bus test watchdog"); end
endmodule
