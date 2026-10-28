`timescale 1ns/1ps
module tb_system_oled;
 reg clk=0; always #10 clk=~clk;
 reg rst_n=0; reg [2:0] seen=0,valid=0;
 reg [47:0] distances=0; reg [23:0] errors=0; reg [11:0] statuses=0;
 wire [671:0] text,tof_text;
 reg [2:0] group_id=0;reg [3:0] item=0;reg page_on=0; wire scl,frame_done; tri1 sda; reg ack_low=0;
 assign sda=ack_low?1'b0:1'bz;
 reg completed=0;reg av=0,ar=0,aw=0,ve=0;reg [4:0] rs=0;
 tof_page page(.seen(seen),.valid(valid),.distances(distances),.errors(errors),.statuses(statuses),.text(tof_text));
 system_pages pages(.start_diag(8'd0),.clk(clk),.rst_n(rst_n),.group_id(group_id),.item(item),.page(page_on),
 .avoid_mode(av),.auto_run(ar),.waiting(aw),.route_state(rs),.distances(distances),.tof_valid(valid),
 .voice_enable(ve),.voice_candidate(1'b1),.gear(2'd1),.gear_candidate(2'd2),
 .ready(1'b1),.busy(1'b0),.fault(1'b0),.imu_ready(1'b1),.calibrating(1'b0),.attitude_valid(1'b1),
 .status(8'h11),.imu_error(8'd0),.sensor_error(8'd0),.motion_fault(8'd0),.command(8'h10),.response(8'd0),
 .roll_mdeg(32'sd12345),.pitch_mdeg(-32'sd12345),.yaw_mdeg(32'sd90000),
 .pos_l(32'sd1456),.pos_r(-32'sd1456),.speed_l(32'sd7),.speed_r(-32'sd7),.tof_text(tof_text),.text(text));
 system_oled_text #(.CLK_HZ(10000),.I2C_HZ(1000),.POWERUP_CYCLES(3),.FRAME_WAIT_CYCLES(20)) dut(
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
   "B":glyph=48'h364949497F00; "C":glyph=48'h224141413E00;
   "K":glyph=48'h412214087F00; "M":glyph=48'h7F020C027F00;
   "P":glyph=48'h060909097F00; "U":glyph=48'h3F4040403F00;
   "V":glyph=48'h1F2040201F00; "X":glyph=48'h631408146300;
   "Y":glyph=48'h070870080700; ">":glyph=48'h00081C224100;
   "/":glyph=48'h030408106000; "+":glyph=48'h08083E080800;
   ".":glyph=48'h000060600000;
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

 task settle_frame;
 begin wait(dut.display_text===text);next_frame();end
 endtask
 integer fd,rr,cc;
 task dump(input string path);
 begin fd=$fopen(path,"w");for(rr=0;rr<8;rr++)for(cc=0;cc<128;cc++)$fdisplay(fd,"%02x",memory[rr][cc]);$fclose(fd);end
 endtask
 initial begin
 repeat(5)@(negedge clk);rst_n=1;next_frame();
 check_row(0,"RPACS MENU");check_row(1,">DISPLAY");check_row(2,"MANUAL");check_row(3,"K0 RST K1 NEXT K2 OK");
 dump("records/main_20261023/system_menu_root.hex");
 $display("PASS main OLED root menu complete bitmap and key legend");
 // The page changes mid-frame. Old frame must remain a complete snapshot.
 wait(dut.page==1 && dut.column>20);@(negedge clk);group_id=2;item=3;page_on=1;
 next_frame();check_row(0,"RPACS MENU");settle_frame();
 check_row(0,"RIGHT 90 DEG");check_row(1,"K2 EXECUTE ONCE");check_row(2,"GOAL 90 DEG");check_row(3,"READY K0 BACK K2 RUN");
 $display("PASS main OLED action page and tear-free page switching");
 group_id=4;item=0;settle_frame();
 check_row(0,"VOICE ENABLE");check_row(1,"CURRENT OFF");check_row(2,"SELECT ON");check_row(3,"K0 BACK K1 SEL K2 SET");
 dump("records/main_20261023/system_menu_voice.hex");
 item=1;settle_frame();check_row(1,"CURRENT MID");check_row(2,"SELECT HIGH 0.15 M/S");
 group_id=1;item=1;settle_frame();check_row(1,"ROLL  +012.3 DEG");check_row(2,"PITCH -012.3 DEG");check_row(3,"YAW   +090.0 DEG");
 item=3;settle_frame();check_row(1,"L:000005B0 V:+00007");check_row(2,"R:FFFFFA50 V:-00007");
 item=2;seen=7;valid=7;distances={16'd3456,16'd2345,16'd1234};statuses=12'h753;settle_frame();
 check_row(0,"TOF DIAG");check_row(1,"FRONT:01234mm S:03");
 valid=5;errors=24'h000200;settle_frame();check_row(2,"LEFT :-----  E:02");
 dump("records/main_20261023/system_menu_tof.hex");
 $display("PASS settings, attitude signs/units, encoder count/speed, ToF bitmap and stale data masking");
 group_id=0;item=3;page_on=0;settle_frame();check_row(1,">OBSTACLE AVOID");
 group_id=5;item=0;page_on=1;settle_frame();check_row(0,"OBSTACLE AVOID");check_row(1,"TOF NOT READY");
 ve=1;av=1;aw=1;valid=7;distances={16'd2000,16'd2000,16'd2000};settle_frame();
 check_row(1,"SPEAKING PLEASE WAIT");check_row(2,"F:07D0 L:07D0 R:07D0");
 aw=0;ar=1;rs=2;settle_frame();check_row(1,"CRUISE           MID");check_row(3,"K0 EXIT K2 PAUSE");
 ar=0;settle_frame();check_row(1,"PAUSED           MID");check_row(3,"K0 EXIT K2 START");
 $display("PASS obstacle OLED entry, voice-off prompt, waiting, cruise, paused, ranges and keys");
 completed=1;$finish;
 end
 initial begin #100000000;$fatal(1,"main OLED watchdog");end
endmodule
