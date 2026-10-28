`timescale 1ns/1ps
module obstacle_page(input wire [4:0] state,input wire [7:0] reason,input wire fault,
 input wire [47:0] distances,input wire [2:0] valid,reused,output reg [671:0] text);
 function [7:0] hexchar;input [3:0] n;begin hexchar=n<10 ? 48+n:55+n;end endfunction
 integer i,j; reg [15:0] mm; reg [167:0] row;
 always @* begin
 text={84{8'h20}};
 // Use glyphs already present in the diagnostic renderer.
 text[0+:8]="S";text[8+:8]="T";text[16+:8]="A";text[24+:8]="T";text[32+:8]="E";
 text[48+:8]=hexchar({3'd0,state[4]});text[56+:8]=hexchar(state[3:0]);
 text[80+:8]="E";text[88+:8]="R";text[96+:8]="R";
 text[112+:8]=hexchar(reason[7:4]);text[120+:8]=hexchar(reason[3:0]);
 text[144+:8]=fault ? "1":"0";
 row=0;mm=0;
 for(i=0;i<3;i=i+1)begin
 row=i==0 ? "FRONT:               ":i==1 ? "LEFT :               ":"RIGHT:               ";
 for(j=0;j<21;j=j+1)text[((i+1)*21+j)*8+:8]=row[167-j*8-:8];
 mm=distances[i*16+:16];
 // V=valid normal reading, H=held during20, X=no usable distance.
 text[((i+1)*21+12)*8+:8]=!valid[i] ? "X":reused[i] ? "H":"V";
 for(j=0;j<5;j=j+1)begin
 text[((i+1)*21+10-j)*8+:8]=valid[i] ? 8'd48+mm%10:8'h2d;mm=mm/10;
 end
 end
 end
endmodule
