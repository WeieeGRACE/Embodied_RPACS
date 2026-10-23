`timescale 1ns/1ps
// Sensor-side SCL is delayed relative to SDA. This exposes same-edge SCL/data
// transitions hidden by the original zero-delay model. Delays stay below 1 us.
module tb_edges;
 reg clk=0; always #10 clk=~clk;
 reg rst_n=0,start=0;
 tri1 scl,sda;
 wire sensor_scl;
 assign #(80,120) sensor_scl=scl;
 wire busy,done; wire [7:0] error; wire [95:0] rx;
 reg [7:0] reg_addr=8'hc0; reg [3:0] read_len=1;
 integer shots,polls,blocks,resets,reads=0;
 reg completed=0;
 tof_i2c_reg dut(.clk(clk),.rst_n(rst_n),.start(start),.write_en(1'b0),
  .reg_addr(reg_addr),.write_data(8'h01),.read_len(read_len),
  .busy(busy),.done(done),.error(error),.rx_data(rx),.scl(scl),.sda(sda));
 vl53l0x_model #(.SDA_DELAY_NS(300)) model(.scl(sensor_scl),.sda(sda),
  .xshut_n(rst_n),.present(1'b1),.nack_data(1'b0),.never_ready(1'b0),
  .distance_mm(16'h1234),.status(4'd3),.ready_after(0),
  .shots(shots),.polls(polls),.read_blocks(blocks),.reset_count(resets));
 // Only START/STOP may change SDA while sensor-side SCL is high.
 always @(dut.bits.sda_low) begin
  if(rst_n && busy && (dut.bits.command==2 || dut.bits.command==3) && sensor_scl===1'b1)
   $fatal(1,"Master changes SDA before sensor sees SCL low (missing data hold)");
 end
 task automatic read_reg(input [7:0] addr,input [3:0] count);
  begin
   @(negedge clk); reg_addr=addr; read_len=count; start=1;
   @(negedge clk); start=0;
   wait(done); @(negedge clk);
   if(error!=0) $fatal(1,"Delayed-edge read failed reg=%h error=%h",addr,error);
   reads++;
  end
 endtask
 initial begin
  repeat(5) @(negedge clk); rst_n=1;
  read_reg(8'hc0,1);
  if(rx[7:0]!=8'hee) $fatal(1,"ID read with delayed sensor ACK/data");
  read_reg(8'h14,12);
  if(rx[95:80]!=16'h3412) $fatal(1,"Sequential byte order with delayed sensor data");
  completed=1;
  $display("PASS delayed SCL edges, 300 ns sensor ACK/data, SDA hold, ID and 12-byte read");
  $finish;
 end
 initial begin #5000000; $fatal(1,"Edge model watchdog reads=%0d",reads); end
endmodule
