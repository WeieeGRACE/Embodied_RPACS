`timescale 1ns/1ps
module tb_system_obstacle;
 reg clk=0;always #10 clk=~clk;
 reg k0=1,k1=1,k2=1,rx=1,miso=0;
 wire rst=dut.rst_n;
 reg [1:0] abl=0,abr=0;
 wire a,a1,a2,b,b1,b2,st,tx,sck,mosi,cs_n,ready_led,run_led,fault_led,voice_led;
 wire oscl;tri1 osda;tri1 [2:0] scl,sda,xshut;
 reg completed=0,init_bad=0,model_on=0,freeze_l=0,freeze_r=0;
 reg [7:0] model_id=8'h24,model_error=0;
 reg signed [15:0] gyro_model=17;
 reg [2:0] present=7,never_ready=0;reg [47:0] ranges={16'd2000,16'd2000,16'd2000};
 wire [31:0] shots[0:2],polls[0:2],blocks[0:2],resets[0:2];
 robot_system_top #(.CLK_HZ(1000000),.UART_BAUD(100000),.PWM_HZ(1000),
 .ENC_FILTER(2),.DIR_DEADTIME(10),.POR_CYCLES(8),.DEBOUNCE_CYCLES(4),
 .IMU_POWERUP_CYCLES(2),.IMU_CONFIG_WAIT_CYCLES(2),.IMU_RETRY_CYCLES(20000),
 .IMU_SPI_HALF_PERIOD(1),.IMU_HEALTH_CYCLES(20000),.IMU_CALIBRATION_SAMPLES(16),
 .OLED_I2C_HZ(100000),.OLED_POWERUP_CYCLES(3),.OLED_FRAME_WAIT_CYCLES(100),
 .TOF_I2C_HZ(50000),.TOF_RESET_CYCLES(10),.TOF_BOOT_CYCLES(30),
 .TOF_POLL_CYCLES(30),.TOF_MAX_POLLS(3),.TOF_TIMEOUT_CYCLES(300),.TOF_ROUND_CYCLES(50000)) dut(
 .clk_50m(clk),.reset_n(k0),.key_enable_n(k1),.key_estop_n(k2),.asr_rx(rx),
 .encoder_a_l(abl[1]),.encoder_b_l(abl[0]),.encoder_a_r(abr[1]),.encoder_b_r(abr[0]),
 .imu_spi_miso(miso),.imu_int1(1'b0),.imu_int2(1'b0),.imu_gnd_reserved(1'b0),
 .imu_spi_sck(sck),.imu_spi_mosi(mosi),.imu_cs_n(cs_n),
 .PWMA(a),.AIN1(a1),.AIN2(a2),.PWMB(b),.BIN1(b1),.BIN2(b2),.STBY(st),.asr_tx(tx),
 .oled_scl(oscl),.oled_sda(osda),
 .tof_front_gpio1(1'b1),.tof_left_gpio1(1'b0),.tof_right_gpio1(1'b1),
 .tof_front_scl(scl[0]),.tof_front_sda(sda[0]),.tof_front_xshut_n(xshut[0]),
 .tof_left_scl(scl[1]),.tof_left_sda(sda[1]),.tof_left_xshut_n(xshut[1]),
 .tof_right_scl(scl[2]),.tof_right_sda(sda[2]),.tof_right_xshut_n(xshut[2]),
 .led_ready(ready_led),.led_run(run_led),.led_fault(fault_led),.led_heartbeat(voice_led));
 genvar g;
 generate for(g=0;g<3;g++)begin:tof_models
 vl53l0x_model model(.scl(scl[g]),.sda(sda[g]),.xshut_n(xshut[g]),
 .present(present[g]),.nack_data(1'b0),.never_ready(never_ready[g]),
 .distance_mm(ranges[g*16+:16]),.status(4'(3+g*2)),.ready_after(1),
 .shots(shots[g]),.polls(polls[g]),.read_blocks(blocks[g]),.reset_count(resets[g]));
 end endgenerate
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task off;begin #1;if({a,a1,a2,b,b1,b2,st}!==0)$fatal(1,"unsafe motor outputs");end endtask
 task keypress(input integer key_index);begin
 @(negedge clk);case(key_index)0:k0=0;1:k1=0;2:k2=0;endcase
 cycles(12);@(negedge clk);k0=1;k1=1;k2=1;cycles(12);
 end endtask
 task root;begin
 if(dut.page)keypress(0);
 if(dut.group_id!=0)keypress(0);
 if(dut.group_id!=0 || dut.page)$fatal(1,"root navigation");
 end endtask
 function automatic integer menu_index(input integer gr);
  case(gr)2:menu_index=0;4:menu_index=1;5:menu_index=2;default:menu_index=0;endcase
 endfunction
 task choose(input integer gr,input integer it);begin
 root();$display("CHOOSE group=%0d item=%0d page=%b target=%0d",dut.group_id,dut.item,dut.page,gr);
 while(dut.item!=menu_index(gr))keypress(1);keypress(2);
 while(dut.item!=it)begin keypress(1);$display("CHOOSE page item=%0d",dut.item);end
 // Selecting the obstacle entry already opens its panel; a second K2 here
 // would start or pause the mode instead of just opening the page.
 if(gr!=5)keypress(2);
 if(!dut.page || dut.group_id!=gr || dut.item!=it)$fatal(1,"page selection");
 end endtask
 task boot;begin
 model_error=0;model_id='h24;init_bad=0;freeze_l=0;freeze_r=0;present=7;never_ready=0;
 root();keypress(0);wait(ready_led);cycles(1000);off();model_on=1;
 if(voice_led || dut.gear!=1)$fatal(1,"reset defaults");
 end endtask
 task send(input [7:0] data);begin
 @(negedge clk);rx=0;cycles(10);
 for(integer i=0;i<8;i++)begin @(negedge clk);rx=data[i];cycles(10);end
 @(negedge clk);rx=1;cycles(15);
 end endtask
 task frame(input [7:0] code);begin
 send('haa);send('h55);send(1);send(code);send(code+1);send(13);cycles(5);
 end endtask
 integer done_events=0,stop_events=0,fault_events=0,busy_replies=0,reject_replies=0,accepted=0;
 integer audio_requests=0,obstacle_events=0,clear_events=0,exit_events=0;reg [7:0] audio_token=0;
 integer idx=0;reg [7:0] got,line[0:5];
 always @(negedge rst)begin idx=0;done_events=0;stop_events=0;fault_events=0;busy_replies=0;reject_replies=0;accepted=0;end
 always @(negedge tx)begin:capture
 if(rst)begin
 repeat(15)@(negedge clk);
 for(integer i=0;i<8;i++)begin if(!rst)disable capture;got[i]=tx;repeat(10)@(negedge clk);end
 if(!rst)disable capture;
 if(tx!==1)$fatal(1,"UART stop");line[idx]=got;
 if(idx==5)begin
 if(line[0]!='haa || line[1]!='h55 || (line[2]!=2 && line[2]!=3 && line[2]!=4 && line[2]!=6 && line[2]!=7) || line[4]!=((line[2]+line[3])&255) || got!=13)$fatal(1,"bad feedback frame");
 if(line[2]==4)begin audio_token=line[3];audio_requests++;end
 if(line[2]==7)case(line[3])1:obstacle_events++;2:clear_events++;3:exit_events++;endcase
 if(line[2]==2)case(line[3])0:accepted++;1:busy_replies++;2:reject_replies++;default:$fatal(1,"bad response");endcase
 if(line[2]==3)case(line[3])'h11:done_events++;4:stop_events++;6:fault_events++;endcase
 idx=0;end else idx++;
 end end
 // Independent physical plants consume output pin duty/direction, never RTL targets.
 integer pwm_l=0,pwm_r=0,plant_tick=0,velocity_l=0,velocity_r=0;
 integer accum_l=0,accum_r=0;
 real physical_yaw=0.0;
 always @(negedge clk)begin
 if(!rst || !model_on)begin
 pwm_l=0;pwm_r=0;plant_tick=0;velocity_l=0;velocity_r=0;accum_l=0;accum_r=0;gyro_model=17;
 if(!rst)physical_yaw=0;
 end else begin
 if(a && st)pwm_l=pwm_l+(a1 ? 1:-1);
 if(b && st)pwm_r=pwm_r+(b1 ? 1:-1);
 if(plant_tick==999)begin
 velocity_l=(3*velocity_l+pwm_l*8)/4;
 velocity_r=(3*velocity_r+pwm_r*7)/4;
 pwm_l=0;pwm_r=0;plant_tick=0;
 // Measured installation: sensor Z is positive for a physical right turn.
 // Keep physical_yaw left-positive independently of the RTL sign parameter.
 gyro_model=$rtoi((velocity_l-velocity_r)*0.557)+17;
 physical_yaw=physical_yaw+(velocity_r-velocity_l)*0.000034;
 end else plant_tick++;
 if(!freeze_l)begin
 accum_l=accum_l+velocity_l;
 if(accum_l>=1000000)begin
 case(abl)0:abl=1;1:abl=3;3:abl=2;2:abl=0;endcase accum_l-=1000000;
 end
 if(accum_l<=-1000000)begin
 case(abl)0:abl=2;2:abl=3;3:abl=1;1:abl=0;endcase accum_l+=1000000;
 end
 end
 if(!freeze_r)begin
 accum_r=accum_r+velocity_r;
 if(accum_r>=1000000)begin
 case(abr)0:abr=2;2:abr=3;3:abr=1;1:abr=0;endcase accum_r-=1000000;
 end
 if(accum_r<=-1000000)begin
 case(abr)0:abr=1;1:abr=3;3:abr=2;2:abr=0;endcase accum_r+=1000000;
 end
 end
 end
 end
 integer duty_l=0,duty_r=0,phase_count=0;
 always @(negedge clk)begin
 if(!rst || !st)begin duty_l=0;duty_r=0;phase_count=0;end
 else begin
 if(a)duty_l++;if(b)duty_r++;
 if(phase_count==999)begin
 if(duty_l>250 || duty_r>250)$fatal(1,"PWM exceeds 25 percent %0d %0d",duty_l,duty_r);
 duty_l=0;duty_r=0;phase_count=0;
 end else phase_count++;
 end
 end
 always @(posedge clk)if(rst && dut.control.done)begin
 if(st || dut.control.raw_l!=0 || dut.control.raw_r!=0)$fatal(1,"premature completion");
 end
    reg [7:0] tx_shift, rx_shift;
    // The C driver clocks one dummy byte for the SPI turnaround and then
    // clocks the first requested register byte.  Keep both bytes in the
    // model so the DUT's read_buffer indexing is checked against the real
    // transaction.
    reg [55:0] response_shift;
    reg [3:0] bit_count;
    reg [7:0] command;
    integer config_bytes;
    integer write_count;
    reg [7:0] config_first;
    reg [7:0] config_last;
    integer config_uploads=0;
    reg [7:0] expected_config [0:8191];
    initial $readmemh("src/imu/imu660ra_config.hex", expected_config);
    initial begin
        #(64'd10000000000);
        $fatal(1, "timeout waiting for initialization");
    end

    always @(posedge cs_n) if(rst && command==8'h5e && config_bytes!=8192)
        $fatal(1,"bad config stream count=%0d",config_bytes);
    always @(negedge cs_n) begin
        bit_count = 0;
        tx_shift = 0;
        rx_shift = 0;
        response_shift = 0;
        command = 0;
        config_bytes = 0;
        write_count = 0;
        config_first = 0;
        config_last = 0;
        miso = 0;
    end

    always @(posedge sck) begin
        if (!cs_n) begin
            #1;
            tx_shift = {tx_shift[6:0], mosi};
            if (bit_count == 7) begin
                if (command == 0) begin
                    command = tx_shift;
                    if (tx_shift == 8'h5E)
                        config_uploads = config_uploads + 1;
                    if (tx_shift == 8'h80)
                        response_shift = {8'h00,model_id,40'd0};
                    else if (tx_shift == 8'h82)
                        response_shift = {8'h00,model_error,40'd0};
                    else if (tx_shift == 8'hA1)
                        response_shift = (init_bad) ? 56'h00020000000000 : 56'h00010000000000;
                    else if (tx_shift == 8'h8C)
                        response_shift = 56'h00000000000010;
                    else if (tx_shift == 8'h92)
                        response_shift = {8'd0,16'd0,16'd0,gyro_model[7:0],gyro_model[15:8]};
                    else
                        response_shift = 0;
                end else if (command == 8'h5E) begin
                    if (config_bytes >= 8192)
                        $fatal(1, "too many configuration bytes");
                    if (tx_shift !== expected_config[config_bytes])
                        $fatal(1, "configuration mismatch index=%0d actual=%h expected=%h", config_bytes, tx_shift, expected_config[config_bytes]);
                    if (config_bytes == 0)
                        config_first = tx_shift;
                    config_last = tx_shift;
                    config_bytes = config_bytes + 1;
                end else if ((command == 8'h7C) || (command == 8'h59) ||
                             (command == 8'h7D) || (command == 8'h40) ||
                             (command == 8'h42) || (command == 8'h41) ||
                             (command == 8'h43)) begin
                    write_count = write_count + 1;
                end
                bit_count = 0;
            end else begin
                bit_count = bit_count + 1;
            end
        end
    end

    always @(negedge sck) begin
        if (!cs_n) begin
            miso = response_shift[55];
            response_shift = {response_shift[54:0], 1'b0};
        end else begin
            miso = 0;
        end
    end




 task typed_frame(input [7:0] typ,input [7:0] code);begin
 send('haa);send('h55);send(typ);send(code);send(typ+code);send(13);cycles(10);
 end endtask
 task start_avoid;integer old_requests;begin
 if(dut.avoid_mode)begin
 if(!dut.auto_run)keypress(2);
 wait(dut.auto_run);wait(run_led);
 end else begin
 old_requests=audio_requests;frame('h16);wait(dut.waiting);wait(audio_requests>old_requests);
 cycles(3000);off();if(dut.group_id!=5 || !dut.page)$fatal(1,"voice did not navigate to obstacle panel");
 typed_frame(5,audio_token+1);cycles(1000);off();if(dut.auto_run)$fatal(1,"wrong audio token started");
 typed_frame(5,audio_token);wait(dut.auto_run);wait(run_led);
 end
 end endtask
 task pause_avoid;begin keypress(2);wait(!run_led);cycles(2000);off();if(!dut.avoid_mode || dut.auto_run)$fatal(1,"K2 lost mode");end endtask
 task voice_noise;begin
 frame(1);frame(2);frame(3);frame('h10);frame('h11);frame('h12);frame('h13);
 frame('h14);frame('h15);frame('h16);frame('h17);
 end endtask
 task assert_voice_isolated;integer old_accepted,old_busy,old_reject;reg was_auto;begin
 was_auto=dut.auto_run;old_accepted=accepted;old_busy=busy_replies;old_reject=reject_replies;
 // The route legitimately keeps the drive outputs active here, so this check
 // observes control state only; safety cutoffs are asserted by other off() sites.
 voice_noise();cycles(50000);
 if(!dut.avoid_mode || dut.control.manual_active || dut.auto_run!=was_auto || dut.gear!=1 || accepted!=old_accepted || busy_replies!=old_busy || reject_replies!=old_reject)
 $fatal(1,"voice changed avoidance control or received a reply");
 end endtask
 task route_one(input bit right_detour);integer ch,extra_l,extra_r,base_clear,base_obstacle,shift_l,shift_r;
 integer home,return_l,return_r,actual_shift;begin
 ranges={16'd2000,16'(right_detour?650:2000),16'd700};
 wait(dut.distances==ranges);home=dut.control.yaw;base_clear=clear_events;base_obstacle=obstacle_events;
 if(!dut.auto_run)start_avoid();wait(dut.route_state==2);wait(st);
 // Voice speed controls are isolated while the route owns the drivetrain.
 frame('h14);frame('h15);cycles(15000);
 if(dut.gear!=1 || dut.control.motion.target_l!=1000)$fatal(1,"voice changed avoidance cruise gear");
 ranges[15:0]=600;wait(dut.distances[15:0]==600);cycles(15000);
 if(dut.control.motion.target_l!=500)$fatal(1,"near cap must override gear");
 frame('h14);cycles(15000);if(dut.gear!=1 || dut.control.motion.target_l!=500)$fatal(1,"voice changed near-cap route speed");
 ranges[15:0]=130;wait(dut.route_state==4);wait(dut.control.motion.accepted);
 ch=right_detour?1:2;ranges={16'd2000,16'd2000,16'd2000};ranges[ch*16+:16]=400;
 wait(dut.route_state==6);shift_l=dut.control.um_l;shift_r=dut.control.um_r;
 if(!right_detour)assert_voice_isolated();
 wait(((dut.control.um_l-shift_l)/2+(dut.control.um_r-shift_r)/2)>=350000);ranges[ch*16+:16]=900;
 wait(dut.route_state==22);extra_l=dut.control.um_l;extra_r=dut.control.um_r;
 wait(dut.route_state==7);actual_shift=dut.control.route.actual_shift;
 if((dut.control.um_l-extra_l)/2+(dut.control.um_r-extra_r)/2<300000)$fatal(1,"short lateral extra");
 ranges[ch*16+:16]=2000;wait(dut.route_state==9);
 wait(((dut.control.um_l-dut.control.route.pass_l)/2+(dut.control.um_r-dut.control.route.pass_r)/2)>=100000);ranges[ch*16+:16]=400;
 if(!right_detour)assert_voice_isolated();
 wait(dut.control.route.box_seen);
 wait(((dut.control.um_l-dut.control.route.pass_l)/2+(dut.control.um_r-dut.control.route.pass_r)/2)>=400000);ranges[ch*16+:16]=900;
 wait(dut.route_state==11);extra_l=dut.control.um_l;extra_r=dut.control.um_r;
 wait(dut.route_state==12);
 if((dut.control.um_l-extra_l)/2+(dut.control.um_r-extra_r)/2<300000)$fatal(1,"short tail extra");
 ranges={16'd2000,16'd2000,16'd2000};wait(dut.route_state==14);return_l=dut.control.um_l;return_r=dut.control.um_r;
 wait(dut.route_state==15);
 if((dut.control.um_l-return_l)/2+(dut.control.um_r-return_r)/2<actual_shift)$fatal(1,"short return");
 wait(dut.route_state==2);wait(dut.control.route_cap==1000);cycles(15000);
 if(clear_events!=base_clear+1 || obstacle_events!=base_obstacle+1)$fatal(1,"route speech count");
 if(dut.gear!=1 || dut.control.motion.target_l!=1000)$fatal(1,"avoidance gear changed during voice isolation");
 if(dut.control.yaw-home>5000 || dut.control.yaw-home< -5000)$fatal(1,"heading not restored");
 if(fault_led)$fatal(1,"route fault");
 $display("PASS main obstacle full %s route, two300mm, return%0d, speech and isolated voice",right_detour?"right":"left",actual_shift);
 end endtask
 always @(dut.route_state)$display("MAIN route=%0d auto=%b waiting=%b reason=%h time=%0t",dut.route_state,dut.auto_run,dut.waiting,dut.control.route_reason,$time);
 // Diagnose any unexpected auto_run drop or fault latch with full sources.
 reg [4:0] route_q=0;reg auto_q=0;reg fault_q=0;
 always @(posedge clk) begin
 route_q<=dut.control.route_state;auto_q<=dut.control.auto_run;fault_q<=dut.control.fault;
 if(route_q!=dut.control.route_state)
  $display("TRACE route %0d->%0d auto=%b arm=%b diag=%0d fault=%b mflt=%h sflt=%h rflt=%b imubad=%b enc=%b/%b txovf=%b stop=%b cancel=%b hold=%b ev=%b tkv=%b exit=%b pause=%b vcv=%b cv=%b cmd=%h mv=%b mc=%h gate=%b mstop=%b time=%0t",
   route_q,dut.control.route_state,dut.control.auto_run,dut.control.dispatch.key_arming,dut.control.start_diag,
   dut.control.fault,dut.control.motion_fault,dut.control.sensor_fault,dut.control.route.route_fault,
   dut.control.imu_bad,dut.control.enc_fault_l,dut.control.enc_fault_r,dut.control.tx_overflow,
   dut.control.stopping,dut.control.route_cancel,dut.control.dispatch.stop_hold,dut.control.dispatch.stop_event,
   dut.control.dispatch.takeover,dut.control.dispatch.exit_cmd,dut.control.dispatch.pause_cmd,
   dut.control.dispatch.voice_command_valid,dut.control.cv,dut.control.command,
   dut.control.menu_valid,dut.control.menu_command,dut.control.stop_gate,dut.control.dispatch.stop_request,$time);
 if(auto_q&&!dut.control.auto_run)
  $display("TRACE AUTO DROP arm=%b diag=%0d fault=%b mflt=%h sflt=%h rflt=%b imubad=%b enc=%b/%b txovf=%b stop=%b cancel=%b hold=%b ev=%b tkv=%b exit=%b pause=%b vcv=%b cv=%b cmd=%h mv=%b mc=%h gate=%b mstop=%b time=%0t",
   dut.control.dispatch.key_arming,dut.control.start_diag,
   dut.control.fault,dut.control.motion_fault,dut.control.sensor_fault,dut.control.route.route_fault,
   dut.control.imu_bad,dut.control.enc_fault_l,dut.control.enc_fault_r,dut.control.tx_overflow,
   dut.control.stopping,dut.control.route_cancel,dut.control.dispatch.stop_hold,dut.control.dispatch.stop_event,
   dut.control.dispatch.takeover,dut.control.dispatch.exit_cmd,dut.control.dispatch.pause_cmd,
   dut.control.dispatch.voice_command_valid,dut.control.cv,dut.control.command,
   dut.control.menu_valid,dut.control.menu_command,dut.control.stop_gate,dut.control.dispatch.stop_request,$time);
 if(!fault_q&&dut.control.fault)
  $display("TRACE FAULT SET mflt=%h sflt=%h rflt=%b imubad=%b enc=%b/%b txovf=%b imu_err=%h time=%0t",
   dut.control.motion_fault,dut.control.sensor_fault,dut.control.route.route_fault,
   dut.control.imu_bad,dut.control.enc_fault_l,dut.control.enc_fault_r,dut.control.tx_overflow,
   dut.control.imu_error,$time);
end
// One-cycle pulses never survive the route-state traces above; catch them here.
reg se_q=0;reg tk_q=0;
always @(posedge clk) begin
 se_q<=dut.control.dispatch.stop_event;tk_q<=dut.control.dispatch.takeover;
 if(!se_q&&dut.control.dispatch.stop_event)
  $display("PULSE stop_event t=%0t tkv=%b exit=%b pause=%b busyin=%b gate=%b mstop=%b mv=%b mc=%h vcv=%b cv=%b cmd=%h amode=%b auto=%b rstop=%b rstart=%b",
   $time,dut.control.dispatch.takeover,dut.control.dispatch.exit_cmd,dut.control.dispatch.pause_cmd,
   dut.control.busy,dut.control.stop_gate,dut.control.dispatch.stop_request,
   dut.control.menu_valid,dut.control.menu_command,dut.control.dispatch.voice_command_valid,
   dut.control.cv,dut.control.command,dut.control.avoid_mode,dut.control.auto_run,
   dut.control.route_stop,dut.control.route.start);
 if(!tk_q&&dut.control.dispatch.takeover)
  $display("PULSE takeover t=%0t cmd=%h cv=%b amode=%b manual=%b pending=%b auto=%b",
   $time,dut.control.command,dut.control.cv,dut.control.avoid_mode,
   dut.control.manual_active,dut.control.dispatch.pending_manual,dut.control.auto_run);
end
 integer l0,old_done,old_clear,old_requests;
 initial begin
 wait(rst);wait(ready_led);wait(dut.tof_valid==7);cycles(2000);off();model_on=1;
 $display("MAIN boot ready");choose(4,0);keypress(1);keypress(2);root();
 if(!dut.voice_enable)$fatal(1,"voice enable setup failed");
 choose(5,0);keypress(2);cycles(100);wait(dut.auto_run);wait(run_led);
 if(dut.waiting || audio_requests!=0)$fatal(1,"button used gated audio request");
 pause_avoid();
 route_one(0);route_one(1);pause_avoid();
 // Voice commands stay blocked while running and paused, while K2/K0 and
 // playback confirmations retain their independent control paths.
 assert_voice_isolated();keypress(2);wait(dut.auto_run);wait(run_led);
 pause_avoid();keypress(0);wait(!dut.avoid_mode);cycles(2000);off();
 if(dut.group_id!=0)$fatal(1,"K0 did not return to root");
 // The held K0 exit cascades into the documented root soft reset: voice
 // defaults to off, the IMU recalibrates and ready drops until it finishes.
 // Restore voice (without a trailing root() K0, which would cascade again)
 // and wait for the chassis to report ready before the outside-mode section.
 wait(ready_led);cycles(1000);
 choose(4,0);keypress(1);keypress(2);
 if(!dut.voice_enable)$fatal(1,"voice re-enable after K0 cascade reset");
 // Outside obstacle mode, normal voice movement, speed and status replies work.
 ranges={16'd40,16'd40,16'd40};old_done=done_events;
 $display("STEP outside-mode ranges40");cycles(2000);
 frame('h12);
 $display("STEP sent h12 t=%0t ready=%b fault=%b flatch=%b mflt=%h busy=%b pend=%b accman=%b stopping=%b cmdready=%b gear=%d imubad=%b enc=%b/%b txovf=%b",
  $time,dut.control.dispatch.ready,dut.control.fault,dut.control.fault_latched,dut.control.motion_fault,
  dut.control.busy,dut.control.dispatch.pending_manual,dut.control.dispatch.accepted_manual,
  dut.control.stopping,dut.control.motion.cmd_ready,dut.gear,
  dut.control.imu_bad,dut.control.enc_fault_l,dut.control.enc_fault_r,dut.control.tx_overflow);
 wait(dut.control.manual_active);$display("STEP manual active");wait(run_led);$display("STEP run_led");
 frame('h15);$display("STEP sent h15");cycles(15000);if(dut.gear!=0)$fatal(1,"outside-mode gear control regressed");
 wait(!run_led);$display("STEP manual done");cycles(2000);off();
 if(fault_led || dut.avoid_mode || done_events!=old_done+1)$fatal(1,"outside-mode voice motion failed");
 if(accepted==0)$fatal(1,"outside-mode accepted response missing");
 ranges={16'd2000,16'd2000,16'd2000};wait(dut.tof_valid==7);
 // Voice entry still supports playback token validation while mode is off.
 old_requests=audio_requests;frame('h16);wait(dut.waiting);wait(audio_requests>old_requests);
 typed_frame(5,audio_token+1);cycles(1000);if(dut.auto_run)$fatal(1,"wrong token started outside-mode request");
 typed_frame(5,audio_token);wait(dut.auto_run);wait(run_led);
 // Voice exit is ignored in-mode; board K0 always cuts drive and exits.
 frame('h17);cycles(1000);if(!dut.avoid_mode || !dut.auto_run)$fatal(1,"voice exit was not isolated");
 @(negedge clk);k0=0;#1;off();cycles(12);k0=1;cycles(12);wait(!run_led);cycles(2000);off();
 if(dut.avoid_mode || dut.group_id!=0)$fatal(1,"K0 mode exit");
 $display("PASS main obstacle voice isolation, route completion, board pause/exit and outside-mode voice regression");
 completed=1;$finish;
 end
 initial begin #(64'd6000000000);$fatal(1,"obstacle suite watchdog state=%0d reason=%h",dut.route_state,dut.control.route_reason);end
endmodule
