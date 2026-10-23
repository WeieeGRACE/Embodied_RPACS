`timescale 1ns/1ps
// Pure page adapter: does not own or control sensor state. Reusable in menus.
module tof_page(input wire [2:0] seen,valid,input wire [47:0] distances,
 input wire [23:0] errors,input wire [11:0] statuses,output reg [671:0] text);
 function [19:0] bcd16;
  input [15:0] value; reg [19:0] b; integer k,j;
  begin
   b=0;
   for(k=15;k>=0;k=k-1) begin
    for(j=0;j<5;j=j+1) if(b[j*4+:4]>=5) b[j*4+:4]=b[j*4+:4]+4'd3;
    b={b[18:0],value[k]};
   end
   bcd16=b;
  end
 endfunction
 function [7:0] hexchar;
  input [3:0] nib; begin hexchar=(nib<10)?8'd48+nib:8'd65+nib-10; end
 endfunction
 integer r,c; reg [19:0] digits; reg [7:0] ch;
 reg [39:0] label_word; reg [7:0] err;
 always @* begin
  text={84{8'h20}}; digits=0; label_word=0; ch=0; err=0;
  // Low byte is the first visible character.
  text[0*8+:8]="T"; text[1*8+:8]="O"; text[2*8+:8]="F";
  text[4*8+:8]="D"; text[5*8+:8]="I"; text[6*8+:8]="A"; text[7*8+:8]="G";
  for(r=0;r<3;r=r+1) begin
   case(r) 0:label_word="FRONT"; 1:label_word="LEFT "; default:label_word="RIGHT"; endcase
   for(c=0;c<5;c=c+1) text[((r+1)*21+c)*8+:8]=label_word[(4-c)*8+:8];
   text[((r+1)*21+5)*8+:8]=":";
   digits=bcd16(distances[r*16+:16]); err=errors[r*8+:8];
   for(c=0;c<5;c=c+1) begin
    ch=(seen[r] && valid[r]) ? 8'd48+digits[(4-c)*4+:4] : "-";
    text[((r+1)*21+6+c)*8+:8]=ch;
   end
   if(!seen[r]) begin
    text[((r+1)*21+13)*8+:32]={8'h54,8'h49,8'h41,8'h57}; // WAIT
   end else if(valid[r]) begin
    text[((r+1)*21+11)*8+:16]={8'h6d,8'h6d};
    text[((r+1)*21+14)*8+:8]="S"; text[((r+1)*21+15)*8+:8]=":";
    text[((r+1)*21+16)*8+:8]="0";
    text[((r+1)*21+17)*8+:8]=hexchar(statuses[r*4+:4]);
   end else begin
    text[((r+1)*21+13)*8+:8]="E"; text[((r+1)*21+14)*8+:8]=":";
    text[((r+1)*21+15)*8+:8]=hexchar(err[7:4]);
    text[((r+1)*21+16)*8+:8]=hexchar(err[3:0]);
   end
  end
 end
endmodule
