`timescale 1ns/1ps
module tb_system_50mhz;
 reg clk=0,rx=1,miso=0,k0=1,k1=1,k2=1;wire rst=dut.rst_n;
 always #10 clk=~clk;
 reg completed=0,init_bad=0;
 reg [7:0] model_id=8'h24,model_error=0;
 reg signed [15:0] gyro_model=17;
 wire a,a1,a2,b,b1,b2,st,tx,sck,mosi,cs_n,ready_led,run_led,fault_led;
 tri1 [2:0] tscl,tsda,txshut;tri1 osda;wire oscl;
 robot_system_top #(.IMU_POWERUP_CYCLES(2),.IMU_CONFIG_WAIT_CYCLES(2),.IMU_CALIBRATION_SAMPLES(16),.DEBOUNCE_CYCLES(4)) dut(
 .clk_50m(clk),.reset_n(k0),.key_enable_n(k1),.key_estop_n(k2),.uart_rx(1'b1),.asr_rx(rx),
 .encoder_a_l(1'b0),.encoder_b_l(1'b0),.encoder_a_r(1'b0),.encoder_b_r(1'b0),
 .imu_spi_miso(miso),.imu_int1(1'b0),.imu_int2(1'b0),.imu_gnd_reserved(1'b0),
 .imu_spi_sck(sck),.imu_spi_mosi(mosi),.imu_cs_n(cs_n),
 .PWMA(a),.AIN1(a1),.AIN2(a2),.PWMB(b),.BIN1(b1),.BIN2(b2),.STBY(st),.asr_tx(tx),.uart_tx(),.oled_scl(oscl),.oled_sda(osda),
 .tof_front_gpio1(1'b1),.tof_left_gpio1(1'b1),.tof_right_gpio1(1'b1),
 .tof_front_scl(tscl[0]),.tof_front_sda(tsda[0]),.tof_front_xshut_n(txshut[0]),
 .tof_left_scl(tscl[1]),.tof_left_sda(tsda[1]),.tof_left_xshut_n(txshut[1]),
 .tof_right_scl(tscl[2]),.tof_right_sda(tsda[2]),.tof_right_xshut_n(txshut[2]),
 .led_ready(ready_led),.led_run(run_led),.led_fault(fault_led),.led_heartbeat());
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task send(input [7:0] data);begin
 @(negedge clk);rx=0;cycles(434);
 for(integer i=0;i<8;i++)begin @(negedge clk);rx=data[i];cycles(434);end
 @(negedge clk);rx=1;cycles(434);
 end endtask
 task frame(input [7:0] code);begin send('haa);send('h55);send(1);send(code);send(code+1);send(13);end endtask
 integer idx=0,responses=0,stopped=0;reg [7:0] data,bytes[0:5];
 always @(negedge tx)begin
 repeat(651)@(negedge clk);
 for(integer i=0;i<8;i++)begin data[i]=tx;repeat(434)@(negedge clk);end
 if(tx!==1)$fatal(1,"50MHz UART stop");
 bytes[idx]=data;
 if(idx==5)begin
 if(bytes[0]!='haa || bytes[1]!='h55 || bytes[4]!=((bytes[2]+bytes[3])&255) || data!=13)$fatal(1,"50MHz feedback");
 if(bytes[2]==2)responses++;
 if({bytes[2],bytes[3]}==16'h0304)stopped++;
 idx=0;
 end else idx++;
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



 initial begin
 wait(rst);wait(ready_led);
 // Enable voice from the physical menu using the same key pulses as the board.
 repeat(3)begin @(negedge clk);k1=0;cycles(12);k1=1;cycles(12);end
 @(negedge clk);k2=0;cycles(12);k2=1;cycles(12); // settings
 @(negedge clk);k2=0;cycles(12);k2=1;cycles(12); // voice page
 @(negedge clk);k1=0;cycles(12);k1=1;cycles(12); // ON candidate
 @(negedge clk);k2=0;cycles(12);k2=1;cycles(12); // commit
 if(!dut.voice_enable)$fatal(1,"50MHz voice menu");
 if(st || fault_led)$fatal(1,"50MHz boot unsafe");
 frame('h10);wait(a || b);wait(dut.control.target_l==7 && dut.control.target_r==7);cycles(2);
 if(dut.control.target_l!=7 || dut.control.target_r!=7)$fatal(1,"50MHz physical speed conversion");
 frame(2);wait(!run_led);cycles(60000);
 if(st || fault_led || responses!=2 || stopped!=1)$fatal(1,"50MHz stop/response st=%b fault=%b responses=%0d stopped=%0d status=%h",st,fault_led,responses,stopped,dut.status);
 $display("PASS main project menu + 50MHz, 115200 baud, 1MHz SPI, production 10ms speed window/20kHz PWM, calibrated enable and STOP");
 completed=1;$finish;
 end
endmodule
