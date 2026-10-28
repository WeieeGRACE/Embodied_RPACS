`timescale 1ns/1ps
// Single-master open-drain primitives. Both inputs are synchronized; the high
// clock phase starts only after the actual SCL input rises (clock stretching).
module tof_i2c_bits #(
 parameter integer CLK_HZ=50000000, I2C_HZ=100000,
 parameter integer TIMEOUT_CYCLES=500000
)(input wire clk,rst_n,start,input wire [1:0] op,
 input wire [7:0] tx_data,input wire read_nack,
 output reg done,output reg [7:0] rx_data,output reg [7:0] error,
 inout wire scl,sda,output reg [5:0] bus_debug);
 localparam integer Q=(CLK_HZ/(2*I2C_HZ)<2)?2:CLK_HZ/(2*I2C_HZ);
 // Positive data hold after falling SCL avoids false START/STOP when the
 // clock and data paths have different propagation/level-shifter delays.
 localparam integer HOLD=(Q/4<2)?2:Q/4;
 localparam integer LOW_SETUP=(Q-HOLD<2)?2:Q-HOLD;
 localparam IDLE=0,START_WAIT=1,START_HOLD=2,STOP_LOW=3,
 STOP_WAIT=4,STOP_HOLD=5,STOP_RELEASE=6,BIT_LOW=7,BIT_WAIT=8,BIT_HIGH=9,
 BIT_FALL_HOLD=10,START_LOW=11,STOP_ASSERT=12,START_LOW_HOLD=13;
 reg [3:0] state; reg [31:0] timer,stall;
 reg scl_low,sda_low; reg [1:0] command;
 reg [7:0] data; reg nack; reg [3:0] bit_no;
 (* ASYNC_REG="TRUE" *) reg [1:0] scl_sync,sda_sync;
 assign scl=scl_low?1'b0:1'bz;
 assign sda=sda_low?1'b0:1'bz;
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin scl_sync<=3; sda_sync<=3; end
  else begin scl_sync<={scl_sync[0],scl}; sda_sync<={sda_sync[0],sda}; end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE; timer<=0; stall<=0; done<=0; error<=0; rx_data<=0; bus_debug<=0;
   scl_low<=0; sda_low<=0; command<=0; data<=0; nack<=0; bit_no<=0;
  end else begin
   done<=0;
   if(state==START_WAIT || state==STOP_WAIT || state==BIT_WAIT) begin
    if(stall>=TIMEOUT_CYCLES-1) begin
     error<=3; bus_debug<={state,scl_sync[1],sda_sync[1]};
     done<=1; state<=IDLE; scl_low<=0; sda_low<=0; stall<=0;
    end else begin
     stall<=stall+1'b1;
     if(scl_sync[1] && (state!=START_WAIT || sda_sync[1])) begin
      timer<=0; stall<=0;
      if(state==START_WAIT) state<=START_HOLD;
      else if(state==STOP_WAIT) state<=STOP_HOLD;
      else state<=BIT_HIGH;
     end
    end
   end else if(state==IDLE) begin
    timer<=0; stall<=0;
    if(start) begin
     command<=op; data<=tx_data; nack<=read_nack; error<=0; bus_debug<=0; bit_no<=0;
     case(op)
      0: begin scl_low<=0; sda_low<=0; state<=START_WAIT; end
      1: begin scl_low<=1; state<=STOP_ASSERT; end
      default: begin
       scl_low<=1; sda_low<=(op==2)?~tx_data[7]:1'b0; state<=BIT_LOW;
      end
     endcase
    end
   end else if(timer<((state==BIT_FALL_HOLD)?HOLD:((state==BIT_LOW)?LOW_SETUP:Q))-1) timer<=timer+1'b1;
   else begin
    timer<=0;
    case(state)
     START_HOLD: begin sda_low<=1; state<=START_LOW; end
     START_LOW: begin scl_low<=1; state<=START_LOW_HOLD; end
     START_LOW_HOLD: begin state<=IDLE; done<=1; end
     STOP_ASSERT: begin sda_low<=1; state<=STOP_LOW; end
     STOP_LOW: begin scl_low<=0; state<=STOP_WAIT; end
     STOP_HOLD: begin sda_low<=0; state<=STOP_RELEASE; end
     STOP_RELEASE: begin state<=IDLE; done<=1; end
     BIT_LOW: begin scl_low<=0; state<=BIT_WAIT; end
     BIT_HIGH: begin
      scl_low<=1;
      state<=BIT_FALL_HOLD;
      // Sample while SCL is still high; retain SDA until the low hold ends.
      if(bit_no<8 && command==3) rx_data<={rx_data[6:0],sda_sync[1]};
      if(bit_no==8 && command==2 && sda_sync[1]) begin
       error<=1; bus_debug<={state,scl_sync[1],sda_sync[1]};
      end
     end
     BIT_FALL_HOLD: begin
      if(bit_no==8) begin
       sda_low<=0; state<=IDLE; done<=1;
      end else begin
       bit_no<=bit_no+1'b1;
       if(bit_no==7) sda_low<=(command==3)?~nack:1'b0;
       else sda_low<=(command==2)?~data[6-bit_no]:1'b0;
       state<=BIT_LOW;
      end
     end
     default: begin state<=IDLE; scl_low<=0; sda_low<=0; end
    endcase
   end
  end
 end
endmodule

// Arduino Wire-compatible register access: pointer write + STOP, then read.
// rx_data byte 0 occupies bits [7:0]. Read lengths are 1..12.
module tof_i2c_reg #(
 parameter integer CLK_HZ=50000000,I2C_HZ=100000,TIMEOUT_CYCLES=500000
)(input wire clk,rst_n,start,write_en,input wire [7:0] reg_addr,write_data,
 input wire [3:0] read_len,output reg busy,done,
 output reg [7:0] error,output reg [95:0] rx_data,inout wire scl,sda,
 output reg [15:0] diagnostic);
 reg launch,waiting,wr; reg [3:0] step,index,len;
 reg [7:0] addr,value; reg [1:0] op; reg [7:0] tx;
 wire bit_done; wire [7:0] bit_error,rx; wire [5:0] bit_debug;
 wire last_nack=(index==len-1'b1);
 tof_i2c_bits #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ),.TIMEOUT_CYCLES(TIMEOUT_CYCLES)) bits(
  .clk(clk),.rst_n(rst_n),.start(launch),.op(op),.tx_data(tx),.read_nack(last_nack),
  .done(bit_done),.rx_data(rx),.error(bit_error),.scl(scl),.sda(sda),.bus_debug(bit_debug));
 always @* begin
  op=0; tx=0;
  case(step)
   0,5: op=0;
   1: begin op=2; tx=8'h52; end
   2: begin op=2; tx=addr; end
   3: begin op=2; tx=value; end
   4,8,9: op=1;
   6: begin op=2; tx=8'h53; end
   7: op=3;
   default: op=1;
  endcase
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   launch<=0; waiting<=0; busy<=0; done<=0; error<=0;
   diagnostic<=0; rx_data<=0; step<=0; index<=0; len<=0; addr<=0; value<=0; wr<=0;
  end else begin
   launch<=0; done<=0;
   if(!busy) begin
    if(start) begin
     busy<=1; waiting<=0; step<=0; index<=0; wr<=write_en;
     addr<=reg_addr; value<=write_data; len<=read_len; error<=0; rx_data<=0; diagnostic<=0;
    end
   end else if(!waiting) begin launch<=1; waiting<=1; end
   else if(bit_done) begin
    waiting<=0;
    if(step==9) begin busy<=0; done<=1; end
    else if(bit_error!=0) begin error<=bit_error; diagnostic<={step,index,2'b0,bit_debug}; step<=9; end
    else case(step)
     0: step<=1;
     1: step<=2;
     2: step<=wr?3:4;
     3: step<=8;
     4: step<=5;
     5: step<=6;
     6: step<=7;
     7: begin
      rx_data[index*8+:8]<=rx;
      if(index==len-1'b1) step<=8;
      else index<=index+1'b1;
     end
     8: begin busy<=0; done<=1; end
     default: step<=9;
    endcase
   end
  end
 end
endmodule
