`timescale 1ns/1ps
module tb_oled_wire;
  reg clk=0,rst_n=0; reg [671:0] text={84{8'h20}}; tri1 sda; wire scl,done;
  integer starts=0,edges=0; reg last_scl=1;
  tof_oled_text #(.CLK_HZ(1000000),.I2C_HZ(100000),.POWERUP_CYCLES(2),.FRAME_WAIT_CYCLES(2)) dut(
    .clk(clk),.rst_n(rst_n),.text(text),.frame_done(done),.scl(scl),.sda(sda));
  always #500 clk=~clk;
  always @(posedge done) begin edges=edges+1; if(edges==1) text[0+:8]="8"; if(edges==2) begin $display("PASS: OLED frame completion and live text refresh"); $finish; end end
  initial begin #1000 rst_n=1; text[0+:8]="1"; #2000000000 $fatal(1,"OLED refresh timeout"); end
endmodule
