`timescale 1ns/1ps
// Protocol-level model of ONLY the supplied Arduino example. It intentionally
// makes no claim about the silicon's undocumented initialization/calibration.
module vl53l0x_model #(parameter integer SDA_DELAY_NS=0)(input wire scl,inout wire sda,input wire xshut_n,
 input wire present,nack_data,never_ready,input wire [15:0] distance_mm,
 input wire [3:0] status,input integer ready_after,
 output integer shots,polls,read_blocks,reset_count);
 reg drive=0; assign #(SDA_DELAY_NS) sda=drive?1'b0:1'bz;
 reg [7:0] ptr=0,byte_in,address;
 reg measuring=0; integer log_count=0;
 reg [7:0] register_log[0:4095];
 function automatic [7:0] value(input [7:0] a);
  case(a)
   'hc0:value='hee; 'hc2:value='h10; 'h50:value=6; 'h70:value=4;
   'h14:value={1'b0,status,2'b0,(measuring && !never_ready && polls>=ready_after)};
   'h1a:value='h01; 'h1b:value='h23;
   'h1c:value='h04; 'h1d:value='h56;
   'h1e:value=distance_mm[15:8]; 'h1f:value=distance_mm[7:0];
   default:value=0;
  endcase
 endfunction
 task automatic wait_start;
  begin : seek
   forever begin @(negedge sda); if(scl===1'b1) disable seek; end
  end
 endtask
 task automatic wait_stop;
  begin : seek
   forever begin @(posedge sda); if(scl===1'b1) disable seek; end
  end
 endtask
 task automatic receive(output reg [7:0] b);
  integer k;
  begin b=0; for(k=7;k>=0;k=k-1) begin @(posedge scl); b[k]=sda; end end
 endtask
 task automatic ack(input bit yes);
  begin @(negedge scl); drive=yes; @(posedge scl); @(negedge scl); drive=0; end
 endtask
 task automatic send_byte(input reg [7:0] b,output bit more);
  integer k;
  begin
   for(k=7;k>=0;k=k-1) begin drive=!b[k]; @(posedge scl); @(negedge scl); end
   drive=0; @(posedge scl); more=(sda===1'b0); @(negedge scl);
  end
 endtask
 task automatic transaction;
  bit stopped,more; reg [7:0] b; integer count;
  begin
   wait_start(); receive(address);
   if(!present || (address!='h52 && address!='h53)) begin ack(0); wait_stop(); end
   else begin
    ack(1);
    if(address=='h52) begin
     receive(ptr); register_log[log_count]=ptr; log_count++;
     ack(!nack_data);
     if(nack_data) wait_stop();
     else begin
      stopped=0;
      fork : receive_tail
       begin receive(b); end
       begin wait_stop(); stopped=1; end
      join_any
      disable receive_tail;
      if(!stopped) begin
       if(ptr!=0 || b!=1) $fatal(1,"Unexpected sensor write %h=%h",ptr,b);
       shots++; polls=0; measuring=1;
       ack(1); wait_stop();
      end
     end
    end else begin
     if(ptr=='h14) polls++;
     count=0;
     do begin
      b=value(ptr); send_byte(b,more); ptr++; count++;
     end while(more);
     if(count!=1 && count!=12) $fatal(1,"Invalid sensor read length %0d",count);
     if(count==12) read_blocks++;
     wait_stop();
    end
   end
  end
 endtask
 initial begin
  shots=0; polls=0; read_blocks=0; reset_count=0;
  forever begin
   wait(xshut_n===1'b1);
   fork : until_reset
    begin forever transaction(); end
    begin @(negedge xshut_n); end
   join_any
   disable until_reset;
   drive=0; ptr=0; measuring=0; polls=0; log_count=0; reset_count++;
  end
 end
endmodule
