`timescale 1ns/1ps
module tb_asr_motion;
 reg clk=0,rst=0,key=1,estop=1,rx=1,miso=0;
 always #10 clk=~clk;
 reg [1:0] abl=0,abr=0;
 wire a,a1,a2,b,b1,b2,st,tx,sck,mosi,cs_n,ready_led,run_led,fault_led,diag_led;
 reg completed=0,init_bad=0,model_on=0,freeze_l=0,freeze_r=0;
 reg [7:0] model_id=8'h24,model_error=0;
 reg signed [15:0] gyro_model=17;
 asr_motion_test_top #(.CLK_HZ(1000000),.BAUD(100000),.PWM_HZ(1000),
 .ENC_FILTER(2),.DIR_DEADTIME(10),.IMU_POWERUP(2),.IMU_WAIT(2),.IMU_RETRY(20000),
 .IMU_SPI_HALF(1),.IMU_HEALTH(20000),.CALIBRATION_SAMPLES(16)) dut(
 .clk_50m(clk),.reset_n(rst),.key_enable_n(key),.key_estop_n(estop),.uart_rx(rx),
 .encoder_a_l(abl[1]),.encoder_b_l(abl[0]),.encoder_a_r(abr[1]),.encoder_b_r(abr[0]),
 .imu_spi_miso(miso),.imu_int1(1'b0),.imu_int2(1'b0),.imu_gnd_reserved(1'b0),
 .imu_spi_sck(sck),.imu_spi_mosi(mosi),.imu_cs_n(cs_n),
 .PWMA(a),.AIN1(a1),.AIN2(a2),.PWMB(b),.BIN1(b1),.BIN2(b2),.STBY(st),.uart_tx(tx),
 .led_ready(ready_led),.led_run(run_led),.led_fault(fault_led),.led_heartbeat(diag_led));
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task off;begin #1;if({a,a1,a2,b,b1,b2,st}!==0)$fatal(1,"unsafe motor outputs");end endtask
 task send(input [7:0] data);begin
 @(negedge clk);rx=0;cycles(10);
 for(integer i=0;i<8;i++)begin @(negedge clk);rx=data[i];cycles(10);end
 @(negedge clk);rx=1;cycles(15);
 end endtask
 task frame(input [7:0] code);begin
 send('haa);send('h55);send(1);send(code);send(code+1);send(13);cycles(5);
 end endtask
 task reset_system;begin
 model_on=0;@(negedge clk);rst=0;key=1;estop=1;rx=1;abl=0;abr=0;
 model_id='h24;model_error=0;init_bad=0;freeze_l=0;freeze_r=0;cycles(20);rst=1;
 end endtask
 task boot;begin reset_system();wait(ready_led);cycles(1000);off();model_on=1;end endtask
 integer done_events=0,stop_events=0,fault_events=0,busy_replies=0,reject_replies=0,accepted=0;
 integer idx=0;reg [7:0] got,line[0:5];
 always @(negedge rst)begin idx=0;done_events=0;stop_events=0;fault_events=0;busy_replies=0;reject_replies=0;accepted=0;end
 always @(negedge tx)begin: capture
 if(rst && !diag_led)begin
 repeat(15)@(negedge clk);
 for(integer i=0;i<8;i++)begin if(!rst)disable capture;got[i]=tx;repeat(10)@(negedge clk);end
 if(!rst)disable capture;
 if(tx!==1)$fatal(1,"UART stop");
 line[idx]=got;
 if(idx==5)begin
 if(line[0]!='haa || line[1]!='h55 || (line[2]!=2 && line[2]!=3) || line[4]!=((line[2]+line[3])&255) || got!=13)$fatal(1,"bad feedback frame");
 if(line[2]==2)case(line[3])0:accepted++;1:busy_replies++;2:reject_replies++;default:$fatal(1,"bad response");endcase
 if(line[2]==3)case(line[3])'h11:done_events++;4:stop_events++;6:fault_events++;endcase
 idx=0;
 end else idx++;
 end
 end
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
 always @(posedge clk)if(rst && dut.done)begin
 if(st || dut.raw_l!=0 || dut.raw_r!=0)$fatal(1,"premature completion");
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
    initial $readmemh("../../src/imu/imu660ra_config.hex", expected_config);
    initial begin
        #1000000000;
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


 integer diagnostic_lines=0,diagnostic_index=0;reg [7:0] diagnostic_byte;
 always @(negedge tx)begin: diag_capture
 if(rst && diag_led)begin
 repeat(15)@(negedge clk);
 for(integer i=0;i<8;i++)begin if(!rst)disable diag_capture;diagnostic_byte[i]=tx;repeat(10)@(negedge clk);end
 if(tx!==1)$fatal(1,"diagnostic UART stop");
 case(diagnostic_index)
 0:if(diagnostic_byte!="L")$fatal(1,"diagnostic label L");
 11:if(diagnostic_byte!="R")$fatal(1,"diagnostic label R");
 22:if(diagnostic_byte!="Y")$fatal(1,"diagnostic label Y");
 1,12,23:if(diagnostic_byte!="=")$fatal(1,"diagnostic equals");
 10,21:if(diagnostic_byte!=32)$fatal(1,"diagnostic separator");
 32:if(diagnostic_byte!=13)$fatal(1,"diagnostic CR");
 33:if(diagnostic_byte!=10)$fatal(1,"diagnostic LF");
 default:if(!((diagnostic_byte>=48 && diagnostic_byte<=57)||(diagnostic_byte>=65 && diagnostic_byte<=70)))$fatal(1,"diagnostic hex");
 endcase
 if(diagnostic_index==33)begin diagnostic_lines++;diagnostic_index=0;end else diagnostic_index++;
 end
 end
 integer l0,r0,y0,d0;real py0;
 task action(input [7:0] code);begin
 l0=dut.um_l;r0=dut.um_r;y0=dut.yaw;py0=physical_yaw;d0=done_events;
 frame(code);wait(run_led);wait(!run_led || fault_led);cycles(1500);
 if(fault_led || done_events!=d0+1)$fatal(1,"action %h failed fc=%d done=%0d/%0d phase=%0d",code,dut.motion_fault,done_events,d0,dut.phase);
 off();
 if(code=='h10 && (dut.um_l-l0<200000 || dut.um_r-r0<200000))$fatal(1,"short forward");
 if(code=='h11 && (l0-dut.um_l<200000 || r0-dut.um_r<200000))$fatal(1,"short reverse");
 if(code=='h12 && dut.yaw-y0<90000)$fatal(1,"short left");
 if(code=='h13 && y0-dut.yaw<90000)$fatal(1,"short right");
 if(code=='h12 && physical_yaw<=py0)$fatal(1,"physical left direction");
 if(code=='h13 && physical_yaw>=py0)$fatal(1,"physical right direction");
 if(code=='h12 || code=='h13)begin
 if(((physical_yaw>py0)?(physical_yaw-py0):(py0-physical_yaw))<85 || ((physical_yaw>py0)?(physical_yaw-py0):(py0-physical_yaw))>100)$fatal(1,"physical angle out of bounds %f",physical_yaw-py0);
 end
 $display("PASS action %h L=%0d um R=%0d um yaw=%0d mdeg physical=%f deg",code,dut.um_l-l0,dut.um_r-r0,dut.yaw-y0,physical_yaw-py0);
 end endtask
 initial begin
 reset_system();wait(dut.calibrating);frame('h10);cycles(1500);
 off();if(reject_replies!=1)$fatal(1,"accepted before calibration");
 wait(ready_led);model_on=1;
 frame(1);cycles(1500);if(reject_replies!=2)$fatal(1,"search accepted");
 frame('h15);frame('h15);frame('h15);cycles(3000);if(dut.gear!=0 || run_led)$fatal(1,"low gear saturation");
 frame('h14);frame('h14);frame('h14);cycles(3000);if(dut.gear!=2 || run_led)$fatal(1,"high gear saturation");
 frame('h15);cycles(2000);
 action('h10);action('h11);action('h12);action('h13);
 // Consecutive left rotations cross the estimator's +180 degree boundary.
 action('h12);action('h12);action('h12);
 frame('h10);wait(run_led);cycles(10000);
 frame('h11);frame(3);frame('h14);cycles(3000);
 if(busy_replies!=1 || dut.motion.vel_l!=1000 || dut.gear!=2)$fatal(1,"busy/query/gear latch");
 frame(2);wait(!run_led);cycles(1500);off();
 if(done_events!=7 || stop_events!=1)$fatal(1,"stop reported done");
 $display("PASS busy refusal/query/next-action gear and ordinary stop");
 frame('h10);wait(st);cycles(10000);
 @(negedge clk);#2;estop=0;#1;off();#1;estop=1;cycles(1500);off();
 if(!fault_led || done_events!=7 || fault_events==0)$fatal(1,"estop latch/report");
 frame('h10);cycles(1500);if(run_led)$fatal(1,"restart after estop");
 boot();frame('h10);wait(st);cycles(2000);model_error=1;wait(fault_led);cycles(1500);off();
 if(done_events!=0 || fault_events==0)$fatal(1,"IMU fault completion");
 model_error=0;cycles(25000);off();if(!fault_led)$fatal(1,"auto recovered fault");
 $display("PASS async estop and sensor error latch/no auto restart");
 boot();frame('h10);wait(st);force dut.sample_valid=0;wait(fault_led);cycles(1500);off();release dut.sample_valid;
 if(done_events!=0)$fatal(1,"sample loss done");
 boot();frame('h12);wait(st);force dut.attitude_update=0;wait(fault_led);cycles(1500);off();release dut.attitude_update;
 $display("PASS sample and attitude feedback age protection");
 boot();model_on=0;frame('h10);wait(st);
 @(negedge clk);abl=3;cycles(20);wait(fault_led);off();
 if(!dut.enc_fault_l)$fatal(1,"encoder illegal transition not latched");
 boot();frame('h10);wait(st);@(negedge clk);rst=0;#1;off();cycles(20);rst=1;
 wait(ready_led);cycles(2000);off();if(run_led)$fatal(1,"auto restart after reset");
 $display("PASS encoder fault and running reset");
 // Frozen wheels keep fresh zero speed samples but cannot reach distance.
 freeze_l=1;freeze_r=1;model_on=1;frame('h10);wait(fault_led);cycles(1500);off();
 if(dut.motion_fault!=4 || done_events!=0)$fatal(1,"action timeout did not fail");
 $display("PASS 10 second action timeout with fresh frozen encoder feedback");
 boot();frame('h10);wait(st);force dut.valid_l=0;wait(fault_led);cycles(1500);off();release dut.valid_l;
 if(dut.motion_fault!=2)$fatal(1,"left speed freshness");
 boot();frame('h10);wait(st);force dut.valid_r=0;wait(fault_led);cycles(1500);off();release dut.valid_r;
 if(dut.motion_fault!=2)$fatal(1,"right speed freshness");
 boot();frame('h10);wait(st);frame(2);wait(dut.phase==4);force dut.raw_l=1;
 wait(fault_led);cycles(1500);off();release dut.raw_l;
 if(dut.motion_fault!=5 || done_events!=0)$fatal(1,"still timeout");
 boot();frame('h10);wait(st);force dut.imu_ready=0;wait(fault_led);cycles(1500);off();release dut.imu_ready;
 if(done_events!=0)$fatal(1,"lost ready completion");
 $display("PASS independent speed stale, stop timeout and ready loss");
 boot();frame('h10);wait(st);
 repeat(80)frame(3);
 if(!fault_led || !dut.tx_overflow)$fatal(1,"protocol overload did not fail closed");
 frame(2);cycles(3000);off();if(done_events!=0)$fatal(1,"overload completion");
 $display("PASS serial overload latches stop; parser still consumes STOP");
 // Passive calibration mode owns TX exclusively and never enables drivers.
 reset_system();@(negedge clk);rst=0;key=0;cycles(20);rst=1;cycles(20);key=1;
 if(!diag_led)$fatal(1,"diagnostic mode not selected");
 frame('h10);cycles(205000);off();
 if(diagnostic_lines!=1)$fatal(1,"diagnostic snapshot missing");
 $display("PASS passive diagnostic mode inhibits commands, hexadecimal count/yaw snapshot decoded");
 completed=1;$finish;
 end
endmodule
