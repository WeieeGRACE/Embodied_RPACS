`timescale 1ns/1ps
module tb_system_event_logger;
 reg clk=0,rst=0,failed=0;always #10 clk=~clk;
 reg [11:0] errors=0;wire tx;reg completed=0;
 // Undriven tie keeps the debug parity x exactly like an absent debug source.
 wire [63:0] debug_tie;
 system_event_logger #(.CLK_HZ(1000),.BAUD(50)) dut(
 .clk(clk),.rst_n(rst),.imu_started(1'b0),.imu_failed(failed),
 .imu_error(8'h05),.errors(errors),.tx(tx),
 .debug_state(debug_tie),.debug_valid(1'b0));
 reg [63:0] expected[0:15],frame=0;reg [7:0] value;
 integer index=0,count=0,i;
 initial begin
 expected[0]={"S M 00",8'h0d,8'h0a};expected[1]={"S N 00",8'h0d,8'h0a};expected[2]={"S W 00",8'h0d,8'h0a};
 expected[3]={"S I 05",8'h0d,8'h0a};
 expected[4]={"E I 04",8'h0d,8'h0a};expected[5]={"E I 05",8'h0d,8'h0a};expected[6]={"E I 06",8'h0d,8'h0a};expected[7]={"E I 07",8'h0d,8'h0a};
 expected[8]={"E I 0A",8'h0d,8'h0a};expected[9]={"E I 08",8'h0d,8'h0a};expected[10]={"E I 09",8'h0d,8'h0a};expected[11]={"E I FF",8'h0d,8'h0a};
 expected[12]={"E W 01",8'h0d,8'h0a};expected[13]={"E L 01",8'h0d,8'h0a};expected[14]={"E R 01",8'h0d,8'h0a};expected[15]={"E X 01",8'h0d,8'h0a};
 repeat(5)@(negedge clk);rst=1;
 repeat(5)@(negedge clk);errors=12'hfff;failed=1;
 @(negedge clk);errors=0;failed=0;
 wait(count==16);repeat(100)@(negedge clk);
 errors=12'hfff;failed=1;repeat(5000)@(negedge clk);
 if(count!=16 || dut.pending!=0)$fatal(1,"persistent/repeated error was sent twice");
 completed=1;$display("PASS logger 16 exact lines, one-cycle simultaneous events during TX, startup failure snapshot, hex/CRLF, persistent and reasserted deduplication");$finish;
 end
 always @(negedge tx)begin
 #600;
 for(i=0;i<8;i=i+1)begin value[i]=tx;#400;end
 if(tx!==1)$fatal(1,"stop bit");
 frame={frame[55:0],value};
 if(index==7)begin
 if(count>=16 || frame!==expected[count])$fatal(1,"logger frame %0d actual=%h",count,frame);
 count=count+1;index=0;
 end else index=index+1;
 end
 initial begin #2000000;$fatal(1,"logger timeout");end
endmodule
