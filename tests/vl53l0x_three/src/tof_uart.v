`timescale 1ns/1ps
// Per-channel mailboxes decouple diagnostics from the UART. Hardware round
// interval exceeds the time needed to transmit all three reports.
module tof_uart #(parameter integer CLK_HZ=50000000,BAUD=115200)(
 input wire clk,rst_n,event_valid,input wire [1:0] event_channel,
 input wire [2:0] valid,gpio_sync,input wire [47:0] distances,ambients,signals,
 input wire [23:0] errors,models,revisions,pre_periods,final_periods,
 input wire [11:0] statuses,input wire [95:0] sequences,
 input wire [47:0] diagnostics,input wire [8:0] bus_levels,
 input wire [14:0] read_masks,input wire [287:0] raw_results,output wire tx);
 reg [245:0] mailbox[0:2],active;
 wire [4:0] rd; wire [95:0] raw;
 wire [15:0] diag; wire [2:0] pads;
 reg [2:0] pending; reg [1:0] channel;
 reg [1:0] state; reg [7:0] position;
 wire ready; wire unused_done;
 wire [7:0] data=(state==0)?header_char(position):line_char(position);
 wire send=(state==0 || state==2);
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) serial(
  .clk(clk),.rst_n(rst_n),.data(data),.valid(send),.ready(ready),.tx(tx),.done(unused_done));
 wire [31:0] seq;
 wire [0:0] ok;
 wire [0:0] gpio;
 wire [3:0] status;
 wire [7:0] err;
 wire [7:0] model;
 wire [7:0] rev;
 wire [7:0] pre;
 wire [7:0] finalp;
 wire [15:0] distance_value;
 wire [15:0] amb;
 wire [15:0] sig;
 assign {rd,raw,diag,pads,seq,ok,gpio,status,err,model,rev,pre,finalp,distance_value,amb,sig}=active;
 wire [39:0] label_word=(channel==0)?"FRONT":((channel==1)?"LEFT ":"RIGHT");
 wire [19:0] dist_bcd,amb_bcd,sig_bcd;
 assign dist_bcd=bcd16(distance_value); assign amb_bcd=bcd16(amb); assign sig_bcd=bcd16(sig);
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
 function [7:0] header_char; input [7:0] pos; begin case(pos)
  0: header_char=8'h56;
  1: header_char=8'h4c;
  2: header_char=8'h35;
  3: header_char=8'h33;
  4: header_char=8'h4c;
  5: header_char=8'h30;
  6: header_char=8'h58;
  7: header_char=8'h20;
  8: header_char=8'h44;
  9: header_char=8'h49;
  10: header_char=8'h41;
  11: header_char=8'h47;
  12: header_char=8'h3a;
  13: header_char=8'h20;
  14: header_char=8'h73;
  15: header_char=8'h75;
  16: header_char=8'h70;
  17: header_char=8'h70;
  18: header_char=8'h6c;
  19: header_char=8'h69;
  20: header_char=8'h65;
  21: header_char=8'h64;
  22: header_char=8'h20;
  23: header_char=8'h65;
  24: header_char=8'h78;
  25: header_char=8'h61;
  26: header_char=8'h6d;
  27: header_char=8'h70;
  28: header_char=8'h6c;
  29: header_char=8'h65;
  30: header_char=8'h20;
  31: header_char=8'h6f;
  32: header_char=8'h6e;
  33: header_char=8'h6c;
  34: header_char=8'h79;
  35: header_char=8'h3b;
  36: header_char=8'h20;
  37: header_char=8'h66;
  38: header_char=8'h75;
  39: header_char=8'h6c;
  40: header_char=8'h6c;
  41: header_char=8'h20;
  42: header_char=8'h69;
  43: header_char=8'h6e;
  44: header_char=8'h69;
  45: header_char=8'h74;
  46: header_char=8'h69;
  47: header_char=8'h61;
  48: header_char=8'h6c;
  49: header_char=8'h69;
  50: header_char=8'h7a;
  51: header_char=8'h61;
  52: header_char=8'h74;
  53: header_char=8'h69;
  54: header_char=8'h6f;
  55: header_char=8'h6e;
  56: header_char=8'h2f;
  57: header_char=8'h63;
  58: header_char=8'h61;
  59: header_char=8'h6c;
  60: header_char=8'h69;
  61: header_char=8'h62;
  62: header_char=8'h72;
  63: header_char=8'h61;
  64: header_char=8'h74;
  65: header_char=8'h69;
  66: header_char=8'h6f;
  67: header_char=8'h6e;
  68: header_char=8'h20;
  69: header_char=8'h6d;
  70: header_char=8'h69;
  71: header_char=8'h73;
  72: header_char=8'h73;
  73: header_char=8'h69;
  74: header_char=8'h6e;
  75: header_char=8'h67;
  76: header_char=8'h2e;
  77: header_char=8'h0d;
  78: header_char=8'h0a;
 default:header_char=8'h20; endcase end endfunction
 function [7:0] line_char; input [7:0] pos; begin case(pos)
   0: line_char=label_word[32+:8];
   1: line_char=label_word[24+:8];
   2: line_char=label_word[16+:8];
   3: line_char=label_word[8+:8];
   4: line_char=label_word[0+:8];
   5: line_char=8'h20;
   6: line_char=8'h73;
   7: line_char=8'h65;
   8: line_char=8'h71;
   9: line_char=8'h3d;
   10: line_char=hexchar(seq[28+:4]);
   11: line_char=hexchar(seq[24+:4]);
   12: line_char=hexchar(seq[20+:4]);
   13: line_char=hexchar(seq[16+:4]);
   14: line_char=hexchar(seq[12+:4]);
   15: line_char=hexchar(seq[8+:4]);
   16: line_char=hexchar(seq[4+:4]);
   17: line_char=hexchar(seq[0+:4]);
   18: line_char=8'h20;
   19: line_char=8'h6d;
   20: line_char=8'h6d;
   21: line_char=8'h3d;
   22: line_char=rd[4]?(8'd48+dist_bcd[16+:4]):8'h2d;
   23: line_char=rd[4]?(8'd48+dist_bcd[12+:4]):8'h2d;
   24: line_char=rd[4]?(8'd48+dist_bcd[8+:4]):8'h2d;
   25: line_char=rd[4]?(8'd48+dist_bcd[4+:4]):8'h2d;
   26: line_char=rd[4]?(8'd48+dist_bcd[0+:4]):8'h2d;
   27: line_char=8'h20;
   28: line_char=8'h69;
   29: line_char=8'h64;
   30: line_char=8'h3d;
   31: line_char=rd[1]?(hexchar(model[4+:4])):8'h2d;
   32: line_char=rd[1]?(hexchar(model[0+:4])):8'h2d;
   33: line_char=8'h20;
   34: line_char=8'h72;
   35: line_char=8'h65;
   36: line_char=8'h76;
   37: line_char=8'h3d;
   38: line_char=rd[0]?(hexchar(rev[4+:4])):8'h2d;
   39: line_char=rd[0]?(hexchar(rev[0+:4])):8'h2d;
   40: line_char=8'h20;
   41: line_char=8'h70;
   42: line_char=8'h72;
   43: line_char=8'h65;
   44: line_char=8'h3d;
   45: line_char=rd[2]?(hexchar(pre[4+:4])):8'h2d;
   46: line_char=rd[2]?(hexchar(pre[0+:4])):8'h2d;
   47: line_char=8'h20;
   48: line_char=8'h66;
   49: line_char=8'h69;
   50: line_char=8'h6e;
   51: line_char=8'h61;
   52: line_char=8'h6c;
   53: line_char=8'h3d;
   54: line_char=rd[3]?(hexchar(finalp[4+:4])):8'h2d;
   55: line_char=rd[3]?(hexchar(finalp[0+:4])):8'h2d;
   56: line_char=8'h20;
   57: line_char=8'h73;
   58: line_char=8'h74;
   59: line_char=8'h61;
   60: line_char=8'h74;
   61: line_char=8'h75;
   62: line_char=8'h73;
   63: line_char=8'h3d;
   64: line_char=rd[4]?(8'h30):8'h2d;
   65: line_char=rd[4]?(hexchar(status)):8'h2d;
   66: line_char=8'h20;
   67: line_char=8'h61;
   68: line_char=8'h6d;
   69: line_char=8'h62;
   70: line_char=8'h69;
   71: line_char=8'h65;
   72: line_char=8'h6e;
   73: line_char=8'h74;
   74: line_char=8'h3d;
   75: line_char=rd[4]?(8'd48+amb_bcd[16+:4]):8'h2d;
   76: line_char=rd[4]?(8'd48+amb_bcd[12+:4]):8'h2d;
   77: line_char=rd[4]?(8'd48+amb_bcd[8+:4]):8'h2d;
   78: line_char=rd[4]?(8'd48+amb_bcd[4+:4]):8'h2d;
   79: line_char=rd[4]?(8'd48+amb_bcd[0+:4]):8'h2d;
   80: line_char=8'h20;
   81: line_char=8'h73;
   82: line_char=8'h69;
   83: line_char=8'h67;
   84: line_char=8'h6e;
   85: line_char=8'h61;
   86: line_char=8'h6c;
   87: line_char=8'h3d;
   88: line_char=rd[4]?(8'd48+sig_bcd[16+:4]):8'h2d;
   89: line_char=rd[4]?(8'd48+sig_bcd[12+:4]):8'h2d;
   90: line_char=rd[4]?(8'd48+sig_bcd[8+:4]):8'h2d;
   91: line_char=rd[4]?(8'd48+sig_bcd[4+:4]):8'h2d;
   92: line_char=rd[4]?(8'd48+sig_bcd[0+:4]):8'h2d;
   93: line_char=8'h20;
   94: line_char=8'h67;
   95: line_char=8'h70;
   96: line_char=8'h69;
   97: line_char=8'h6f;
   98: line_char=8'h3d;
   99: line_char=8'd48+gpio;
   100: line_char=8'h20;
   101: line_char=8'h76;
   102: line_char=8'h61;
   103: line_char=8'h6c;
   104: line_char=8'h69;
   105: line_char=8'h64;
   106: line_char=8'h3d;
   107: line_char=8'd48+ok;
   108: line_char=8'h20;
   109: line_char=8'h65;
   110: line_char=8'h72;
   111: line_char=8'h72;
   112: line_char=8'h3d;
   113: line_char=hexchar(err[4+:4]);
   114: line_char=hexchar(err[0+:4]);
   115: line_char=" ";
   116: line_char="d";
   117: line_char="b";
   118: line_char="g";
   119: line_char="=";
   120: line_char=hexchar(diag[15:12]);
   121: line_char=hexchar(diag[11:8]);
   122: line_char=hexchar(diag[7:4]);
   123: line_char=hexchar(diag[3:0]);
   124: line_char=" ";
   125: line_char="p";
   126: line_char="a";
   127: line_char="d";
   128: line_char="=";
   129: line_char=hexchar({1'b0,pads});
   130: line_char=8'h20;
   131: line_char=8'h72;
   132: line_char=8'h64;
   133: line_char=8'h3d;
   134: line_char=hexchar({3'b0,rd[4]});
   135: line_char=hexchar(rd[3:0]);
   136: line_char=8'h20;
   137: line_char=8'h72;
   138: line_char=8'h61;
   139: line_char=8'h77;
   140: line_char=8'h3d;
   141: line_char=rd[4]?(hexchar(raw[4+:4])):8'h2d;
   142: line_char=rd[4]?(hexchar(raw[0+:4])):8'h2d;
   143: line_char=rd[4]?(hexchar(raw[12+:4])):8'h2d;
   144: line_char=rd[4]?(hexchar(raw[8+:4])):8'h2d;
   145: line_char=rd[4]?(hexchar(raw[20+:4])):8'h2d;
   146: line_char=rd[4]?(hexchar(raw[16+:4])):8'h2d;
   147: line_char=rd[4]?(hexchar(raw[28+:4])):8'h2d;
   148: line_char=rd[4]?(hexchar(raw[24+:4])):8'h2d;
   149: line_char=rd[4]?(hexchar(raw[36+:4])):8'h2d;
   150: line_char=rd[4]?(hexchar(raw[32+:4])):8'h2d;
   151: line_char=rd[4]?(hexchar(raw[44+:4])):8'h2d;
   152: line_char=rd[4]?(hexchar(raw[40+:4])):8'h2d;
   153: line_char=rd[4]?(hexchar(raw[52+:4])):8'h2d;
   154: line_char=rd[4]?(hexchar(raw[48+:4])):8'h2d;
   155: line_char=rd[4]?(hexchar(raw[60+:4])):8'h2d;
   156: line_char=rd[4]?(hexchar(raw[56+:4])):8'h2d;
   157: line_char=rd[4]?(hexchar(raw[68+:4])):8'h2d;
   158: line_char=rd[4]?(hexchar(raw[64+:4])):8'h2d;
   159: line_char=rd[4]?(hexchar(raw[76+:4])):8'h2d;
   160: line_char=rd[4]?(hexchar(raw[72+:4])):8'h2d;
   161: line_char=rd[4]?(hexchar(raw[84+:4])):8'h2d;
   162: line_char=rd[4]?(hexchar(raw[80+:4])):8'h2d;
   163: line_char=rd[4]?(hexchar(raw[92+:4])):8'h2d;
   164: line_char=rd[4]?(hexchar(raw[88+:4])):8'h2d;
   165: line_char=8'h0d;
   166: line_char=8'h0a;
 default:line_char=8'h20; endcase end endfunction
 integer i;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   pending<=0; active<=0; channel<=0; state<=0; position<=0;
   for(i=0;i<3;i=i+1) mailbox[i]<=0;
  end else begin
   case(state)
    0: if(ready) begin
     if(position==78) begin position<=0; state<=1; end
     else position<=position+1'b1;
    end
    1: if(pending!=0) begin
     if(pending[0]) begin channel<=0; active<=mailbox[0]; pending[0]<=0; end
     else if(pending[1]) begin channel<=1; active<=mailbox[1]; pending[1]<=0; end
     else begin channel<=2; active<=mailbox[2]; pending[2]<=0; end
     position<=0; state<=3;
    end
    3: state<=2; // allow label/decimal formatting to settle before first byte
    2: if(ready) begin
     if(position==166) begin position<=0; state<=1; end
     else position<=position+1'b1;
    end
   endcase
   if(event_valid) begin
    mailbox[event_channel]<={read_masks[event_channel*5+:5],raw_results[event_channel*96+:96],diagnostics[event_channel*16+:16],bus_levels[event_channel*3+:3],sequences[event_channel*32+:32],valid[event_channel],
     gpio_sync[event_channel],statuses[event_channel*4+:4],errors[event_channel*8+:8],
     models[event_channel*8+:8],revisions[event_channel*8+:8],pre_periods[event_channel*8+:8],
     final_periods[event_channel*8+:8],distances[event_channel*16+:16],
     ambients[event_channel*16+:16],signals[event_channel*16+:16]};
    pending[event_channel]<=1;
   end
  end
 end
endmodule
