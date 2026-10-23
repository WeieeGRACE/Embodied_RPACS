`timescale 1ns/1ps
module tb_robot_system;
 reg clk=0;always #10 clk=~clk;
 reg k0=1,k1=1,k2=1,rx=1,miso=0;
 wire rst=dut.rst_n;
 reg [1:0] abl=0,abr=0;
 wire a,a1,a2,b,b1,b2,st,tx,sck,mosi,cs_n,ready_led,run_led,fault_led,voice_led;
 wire usb,oscl;tri1 osda;tri1 [2:0] scl,sda,xshut;
 reg completed=0,init_bad=0,model_on=0,freeze_l=0,freeze_r=0;
 reg [7:0] model_id=8'h24,model_error=0;
 reg signed [15:0] gyro_model=17;
 reg [2:0] present=7,never_ready=0;
 wire [31:0] shots[0:2],polls[0:2],blocks[0:2],resets[0:2];
 robot_system_top #(.CLK_HZ(1000000),.UART_BAUD(100000),.PWM_HZ(1000),
 .ENC_FILTER(2),.DIR_DEADTIME(10),.POR_CYCLES(8),.DEBOUNCE_CYCLES(4),
 .IMU_POWERUP_CYCLES(2),.IMU_CONFIG_WAIT_CYCLES(2),.IMU_RETRY_CYCLES(20000),
 .IMU_SPI_HALF_PERIOD(1),.IMU_HEALTH_CYCLES(20000),.IMU_CALIBRATION_SAMPLES(16),
 .OLED_I2C_HZ(100000),.OLED_POWERUP_CYCLES(3),.OLED_FRAME_WAIT_CYCLES(100),
 .TOF_I2C_HZ(50000),.TOF_RESET_CYCLES(10),.TOF_BOOT_CYCLES(30),
 .TOF_POLL_CYCLES(30),.TOF_MAX_POLLS(3),.TOF_TIMEOUT_CYCLES(300),.TOF_ROUND_CYCLES(50000)) dut(
 .clk_50m(clk),.reset_n(k0),.key_enable_n(k1),.key_estop_n(k2),.uart_rx(1'b1),.asr_rx(rx),
 .encoder_a_l(abl[1]),.encoder_b_l(abl[0]),.encoder_a_r(abr[1]),.encoder_b_r(abr[0]),
 .imu_spi_miso(miso),.imu_int1(1'b0),.imu_int2(1'b0),.imu_gnd_reserved(1'b0),
 .imu_spi_sck(sck),.imu_spi_mosi(mosi),.imu_cs_n(cs_n),
 .PWMA(a),.AIN1(a1),.AIN2(a2),.PWMB(b),.BIN1(b1),.BIN2(b2),.STBY(st),.asr_tx(tx),.uart_tx(usb),
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
 .distance_mm(16'(1234+g*1111)),.status(4'(3+g*2)),.ready_after(1),
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
 task choose(input integer gr,input integer it);begin
 root();while(dut.item!=gr-1)keypress(1);keypress(2);
 while(dut.item!=it)keypress(1);keypress(2);
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
 integer idx=0;reg [7:0] got,line[0:5];
 always @(negedge rst)begin idx=0;done_events=0;stop_events=0;fault_events=0;busy_replies=0;reject_replies=0;accepted=0;end
 always @(negedge tx)begin:capture
 if(rst)begin
 repeat(15)@(negedge clk);
 for(integer i=0;i<8;i++)begin if(!rst)disable capture;got[i]=tx;repeat(10)@(negedge clk);end
 if(!rst)disable capture;
 if(tx!==1)$fatal(1,"UART stop");line[idx]=got;
 if(idx==5)begin
 if(line[0]!='haa || line[1]!='h55 || (line[2]!=2 && line[2]!=3) || line[4]!=((line[2]+line[3])&255) || got!=13)$fatal(1,"bad feedback frame");
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



 integer l0,r0,y0,d0,b0,seq0;time t0;real py0;
 task manual_action(input integer it);begin
 choose(2,it);cycles(2000);off();
 l0=dut.control.um_l;r0=dut.control.um_r;y0=dut.control.yaw;d0=done_events;
 keypress(2);wait(run_led);wait(!run_led || fault_led);cycles(2000);off();
 if(fault_led || done_events!=d0+1)$fatal(1,"manual action %d failed code=%d done=%d/%d",it,dut.motion_fault,done_events,d0);
 if(it==0 && (dut.control.um_l-l0<200000 || dut.control.um_r-r0<200000))$fatal(1,"short forward");
 if(it==1 && (l0-dut.control.um_l<200000 || r0-dut.control.um_r<200000))$fatal(1,"short reverse");
 if(it==2 && dut.control.yaw-y0<90000)$fatal(1,"short left");
 if(it==3 && y0-dut.control.yaw<90000)$fatal(1,"short right");
 $display("PASS menu quantitative action %0d distance=%0d/%0d yaw=%0d",it,dut.control.um_l-l0,dut.control.um_r-r0,dut.control.yaw-y0);
 end endtask
 task debug_action(input integer it);begin
 choose(3,it);cycles(1000);off();
 l0=dut.control.pos_l;r0=dut.control.pos_r;d0=done_events;
 keypress(2);wait(run_led);t0=$time;
 wait(!run_led || fault_led);cycles(2000);off();
 if(fault_led || done_events!=d0+1 || $time-t0<160000000)$fatal(1,"8s debug action %d failed fault=%d",it,dut.motion_fault);
 case(it)
 0:if(dut.pos_l<=l0 || dut.pos_r!=r0)$fatal(1,"left forward");
 1:if(dut.pos_l>=l0 || dut.pos_r!=r0)$fatal(1,"left reverse");
 2:if(dut.pos_r<=r0 || dut.pos_l!=l0)$fatal(1,"right forward");
 3:if(dut.pos_r>=r0 || dut.pos_l!=l0)$fatal(1,"right reverse");
 4:if(dut.pos_r<=r0 || dut.pos_l<=l0)$fatal(1,"both forward");
 5:if(dut.pos_r>=r0 || dut.pos_l>=l0)$fatal(1,"both reverse");
 6:if(dut.pos_l<=l0 || dut.pos_r-r0<=dut.pos_l-l0)$fatal(1,"arc left");
 7:if(dut.pos_r<=r0 || dut.pos_l-l0<=dut.pos_r-r0)$fatal(1,"arc right");
 endcase
 $display("PASS full 8s debug action %0d L=%0d R=%0d",it,dut.pos_l-l0,dut.pos_r-r0);
 end endtask
 initial begin
 wait(rst);cycles(100);off();if(voice_led || dut.group_id!=0)$fatal(1,"startup menu");
 // Bounce is ignored, long press selects once and K2 never acts as estop.
 @(negedge clk);k1=0;cycles(1);k1=1;cycles(12);
 if(dut.item!=0)$fatal(1,"bounce accepted");
 @(negedge clk);k2=0;cycles(100);k2=1;cycles(12);
 if(dut.group_id!=1 || dut.page || fault_led)$fatal(1,"held K2 repeated or estop");
 keypress(2); // system page
 @(negedge clk);k0=0;cycles(200);
 if(!rst || dut.group_id!=1 || dut.page)$fatal(1,"held K0 cascaded navigation/reset");
 k0=1;cycles(12);root();
 wait(ready_led);cycles(1500);off();model_on=1;
 frame('h10);cycles(1500);off();if(reject_replies!=1)$fatal(1,"voice default accepted");
 frame('h14);cycles(1500);if(dut.gear!=1)$fatal(1,"disabled voice gear");
 $display("PASS startup defaults, bounce, one-shot keys, K0 context and disabled voice");
 // Settings edits require confirmation.
 choose(4,0);keypress(1);if(voice_led)$fatal(1,"unconfirmed voice");keypress(2);
 if(!voice_led)$fatal(1,"voice on");root();if(!voice_led)$fatal(1,"voice did not persist");
 choose(4,1);keypress(1);if(dut.gear!=1)$fatal(1,"unconfirmed gear");keypress(2);cycles(10);
 if(dut.gear!=2)$fatal(1,"gear commit");keypress(1);keypress(1);keypress(2);cycles(10); // mid
 if(dut.gear!=1)$fatal(1,"gear wrap");root();
 manual_action(0);manual_action(1);manual_action(2);manual_action(3);
 for(integer j=0;j<8;j++)debug_action(j);
 $display("PASS all motions without USB heartbeat");
 choose(2,0);d0=done_events;keypress(2);wait(st);cycles(10000);
 frame('h11);cycles(1500);if(busy_replies==0)$fatal(1,"busy voice not rejected");
 // K0 cuts outputs asynchronously; hold does not resume or reset IMU.
 @(negedge clk);#2;k0=0;#1;off();cycles(1000);
 if(!rst || dut.group_id!=2 || dut.page)$fatal(1,"running K0 reset/navigation");
 k0=1;wait(!run_led);cycles(1500);off();
 if(done_events!=d0 || fault_led)$fatal(1,"stop completion/fault");
 cycles(10000);off();
 $display("PASS running K0 immediate cut, no restart, no false completion");
 // STOP is accepted even if menu also requests movement in that clock.
 choose(2,0);force dut.control.menu_valid=1;force dut.control.menu_command=8'h10;
 force dut.control.command_valid=1;force dut.control.command=8'h02;
 cycles(1);release dut.control.menu_valid;release dut.control.menu_command;
 release dut.control.command_valid;release dut.control.command;cycles(1500);off();
 if(run_led)$fatal(1,"STOP lost arbitration");
 // Menu wins a simultaneous moving voice command, which reports busy.
 force dut.control.menu_valid=1;force dut.control.menu_command=8'h11;
 force dut.control.command_valid=1;force dut.control.command=8'h10;
 cycles(1);release dut.control.menu_valid;release dut.control.menu_command;
 release dut.control.command_valid;release dut.control.command;cycles(50);
 if(!run_led || !dut.control.motion.reverse)$fatal(1,"menu precedence");
 frame(2);wait(!run_led);cycles(2000);off();
 $display("PASS STOP/menu/voice arbitration and voice STOP");
 // Voice toggle off cuts an active move; query and stop still respond.
 choose(4,0);frame('h10);wait(st);keypress(1);keypress(2);
 wait(!run_led);cycles(2000);off();if(voice_led)$fatal(1,"voice off");
 frame(3);frame(2);cycles(2000);off();
 $display("PASS disable voice stops action, disabled query/stop remain available");
 // Three real I2C models keep polling while UI changes and faults do not gate movement.
 wait(dut.tof_seen==7 && dut.tof_valid==7);
 if(dut.distances!={16'd3456,16'd2345,16'd1234})$fatal(1,"ToF distance parse");
 seq0=shots[2];never_ready[0]=1;present[1]=0;
 wait(dut.tof_errors==24'h000102 && dut.tof_valid==4);
 choose(1,2);cycles(1000);
 if(dut.text[27*8+:8]!="-" || dut.text[48*8+:8]!="-")$fatal(1,"stale ToF displayed");
 if(fault_led || !ready_led)$fatal(1,"ToF fault gated motion");
 manual_action(0);
 if(shots[2]<=seq0)$fatal(1,"ToF stopped while navigating");
 never_ready=0;present=7;wait(dut.tof_valid==7 && dut.tof_errors==0);
 $display("PASS integrated ToF normal/NACK/timeout/recovery, hidden stale values, display-only faults");
 // IMU and feedback faults latch, root reset clears and re-calibrates.
 choose(2,0);keypress(2);wait(st);model_error=1;wait(fault_led);cycles(1500);off();
 model_error=0;keypress(2);cycles(30000);off();if(!fault_led)$fatal(1,"fault auto recovered");
 boot();choose(2,0);keypress(2);wait(st);force dut.control.sample_valid=0;
 wait(fault_led);cycles(1500);off();release dut.control.sample_valid;
 boot();choose(2,0);keypress(2);wait(st);force dut.control.valid_l=0;
 wait(fault_led);cycles(1500);off();release dut.control.valid_l;
 if(dut.motion_fault!=2)$fatal(1,"speed freshness");
 boot();choose(2,0);keypress(2);wait(st);model_on=0;
 wait(fault_led);cycles(1500);off();if(dut.motion_fault!=4)$fatal(1,"10s action timeout");
 boot();choose(2,0);keypress(2);wait(st);keypress(0);wait(dut.control.phase==4);
 force dut.control.raw_l=1;wait(fault_led);cycles(1500);off();release dut.control.raw_l;
 if(dut.motion_fault!=5)$fatal(1,"stop timeout");
 boot();choose(2,0);keypress(2);wait(st);repeat(80)frame(3);
 if(!fault_led || !dut.control.tx_overflow)$fatal(1,"feedback overflow");off();
 boot();if(voice_led || run_led || dut.group_id!=0)$fatal(1,"final reset defaults");
 $display("PASS IMU/speed/timeout/still/overflow faults and root reset defaults");
 completed=1;$finish;
 end
 initial begin #(64'd10000000000);$fatal(1,"system suite watchdog");end
endmodule
