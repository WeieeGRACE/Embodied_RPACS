`timescale 1ns/1ps
module tb_oled;
 reg clk=0; always #10 clk=~clk;
 reg rst_n=0; reg [2:0] seen=0,valid=0;
 reg [47:0] distances=0; reg [23:0] errors=0; reg [11:0] statuses=0;
 wire [671:0] text; wire scl,frame_done; tri1 sda; reg ack_low=0;
 assign sda=ack_low?1'b0:1'bz;
 reg completed=0;
 tof_page page(.seen(seen),.valid(valid),.distances(distances),.errors(errors),.statuses(statuses),.text(text));
 tof_oled_text #(.CLK_HZ(10000),.I2C_HZ(1000),.POWERUP_CYCLES(3),.FRAME_WAIT_CYCLES(20)) dut(
  .clk(clk),.rst_n(rst_n),.text(text),.frame_done(frame_done),.scl(scl),.sda(sda));
 reg active=0; integer bit_count=0,byte_count=0,transactions=0;
 reg [7:0] shift=0,address,control,payload;
 reg [7:0] memory[0:7][0:127]; reg [7:0] init_log[0:25];
 integer page_no=0,column=0,init_count=0,frames=0;
 always @(negedge sda) if(scl===1'b1) begin
  active=1; bit_count=0; byte_count=0; shift=0;
 end
 always @(posedge scl) if(active && byte_count<3) begin
  if(bit_count<8) begin
   shift={shift[6:0],sda};
   if(bit_count==7) case(byte_count) 0:address=shift; 1:control=shift; 2:payload=shift; endcase
   bit_count++;
  end else begin bit_count=0; byte_count++; end
 end
 always @(negedge scl) ack_low=(active && bit_count==8 && byte_count<3);
 always @(posedge sda) if(scl===1'b1 && active) begin
  if(byte_count!=3 || address!='h78 || (control!=0 && control!='h40)) $fatal(1,"OLED transaction framing");
  transactions++;
  if(control==0) begin
   if(init_count<26) begin init_log[init_count]=payload; init_count++; end
   else if((payload&'hf8)=='hb0) page_no=payload&7;
   else if((payload&'hf0)==0) column=(column&'hf0)|(payload&15);
   else if((payload&'hf0)=='h10) column=(column&15)|((payload&15)<<4);
  end else begin
   if(column>127) $fatal(1,"OLED column overflow");
   memory[page_no][column]=payload; column++;
  end
  active=0;
 end
 // Golden column bitmap values: independent of the DUT font function.
 function automatic [47:0] glyph(input byte c);
  case(c)
   "0":glyph=48'h3E4549513E00; "1":glyph=48'h00407F420000;
   "2":glyph=48'h465149614200; "3":glyph=48'h314B45412100;
   "4":glyph=48'h107F12141800; "5":glyph=48'h394545452700;
   "6":glyph=48'h3049494A3C00; "7":glyph=48'h030509710100;
   "8":glyph=48'h364949493600; "9":glyph=48'h1E2949490600;
   "A":glyph=48'h7C1211127C00; "D":glyph=48'h1C2241417F00;
   "E":glyph=48'h414949497F00; "F":glyph=48'h010909097F00;
   "G":glyph=48'h7A4949413E00; "H":glyph=48'h7F0808087F00;
   "I":glyph=48'h00417F410000; "L":glyph=48'h404040407F00;
   "N":glyph=48'h7F1008047F00; "O":glyph=48'h3E4141413E00;
   "R":glyph=48'h462919097F00; "S":glyph=48'h324949492600;
   "T":glyph=48'h01017F010100; "W":glyph=48'h3F4038403F00;
   "m":glyph=48'h780418047C00; ":":glyph=48'h000036360000;
   "-":glyph=48'h080808080800; default:glyph=0;
  endcase
 endfunction
 task automatic check_row(input integer row,input string expected);
  integer c; byte ch,want; reg [47:0] bits;
  begin
   for(c=0;c<128;c++) begin
    ch=(c/6<expected.len())?expected.getc(c/6):" "; bits=glyph(ch);
    want=(c<126)?((bits>>((c%6)*8))&'h7f):0;
    if(memory[row*2][c]!==want)
     $fatal(1,"OLED row=%0d col=%0d char=%c got=%h expected=%h",row,c,ch,memory[row*2][c],want);
    if(memory[row*2+1][c]!==0) $fatal(1,"Spacer page not clear");
   end
  end
 endtask
 task automatic next_frame;
  begin @(posedge frame_done); #1; frames++; end
 endtask
 integer i,f; reg [7:0] init_expected[0:25];
 initial begin
  init_expected='{8'hae,8'h00,8'h10,8'h40,8'h81,8'hcf,8'ha1,8'hc8,8'ha6,8'ha8,8'h3f,8'hd3,8'h00,
   8'hd5,8'h80,8'hd9,8'hf1,8'hda,8'h12,8'hdb,8'h30,8'h20,8'h02,8'h8d,8'h14,8'haf};
  for(i=0;i<8;i++) for(f=0;f<128;f++) memory[i][f]='hff;
  repeat(5) @(negedge clk); rst_n=1;
  next_frame();
  for(i=0;i<26;i++) if(init_log[i]!==init_expected[i]) $fatal(1,"OLED initialization command %0d",i);
  check_row(0,"TOF DIAG"); check_row(1,"FRONT:-----  WAIT");
  check_row(2,"LEFT :-----  WAIT"); check_row(3,"RIGHT:-----  WAIT");
  $display("PASS OLED initialization, clear, WAIT text and full bitmap");
  @(negedge clk); seen=7; valid=7; distances={16'd0,16'd65535,16'd12345}; statuses=12'haf3;
  wait(dut.display_text===text);
  // Change the page adapter in mid-frame: the in-flight frame must stay intact.
  wait(dut.page==1 && dut.column>20);
  @(negedge clk); valid=5; errors=24'h000200;
  next_frame();
  check_row(1,"FRONT:12345mm S:03"); check_row(2,"LEFT :65535mm S:0F"); check_row(3,"RIGHT:00000mm S:0A");
  $display("PASS five-digit decimal, raw hex status, frame snapshot coherence");
  next_frame(); check_row(2,"LEFT :-----  E:02");
  check_row(1,"FRONT:12345mm S:03"); check_row(3,"RIGHT:00000mm S:0A");
  $display("PASS fault hides stale distance without disturbing other rows");
  // Recovery, max error code and WAIT replacing a previously valid reading.
  @(negedge clk); valid=3; seen=3; errors=24'hff0000; distances={16'd0,16'd1,16'd2000};
  next_frame();
  check_row(1,"FRONT:02000mm S:03"); check_row(2,"LEFT :00001mm S:0F"); check_row(3,"RIGHT:-----  WAIT");
  @(negedge clk); seen=7;
  next_frame(); check_row(3,"RIGHT:-----  E:FF");
  $display("PASS OLED recovery, WAIT replacement, hexadecimal error code");
  f=$fopen("records/oled_memory.hex","w");
  for(i=0;i<8;i++) for(integer c=0;c<128;c++) $fdisplay(f,"%02h",memory[i][c]);
  $fclose(f); completed=1; $finish;
 end
 initial begin #50000000; $fatal(1,"OLED test watchdog"); end
endmodule
