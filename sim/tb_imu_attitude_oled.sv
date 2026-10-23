`timescale 1ns/1ps
module tb_imu_attitude_oled;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n = 1'b0;
    reg imu_ready = 1'b0;
    reg imu_error = 1'b0;
    reg calibrating = 1'b0;
    reg attitude_valid = 1'b0;
    reg signed [31:0] roll_mdeg = 0, pitch_mdeg = 0, yaw_mdeg = 0;
    wire scl;
    tri1 sda;
    reg completed = 1'b0;

    imu_attitude_oled #(
        .CLK_HZ(10000), .I2C_HZ(1000),
        .POWERUP_CYCLES(1), .FRAME_WAIT_CYCLES(100)
    ) dut (
        .clk(clk), .rst_n(rst_n), .imu_ready(imu_ready), .imu_error(imu_error),
        .calibrating(calibrating), .attitude_valid(attitude_valid),
        .roll_mdeg(roll_mdeg), .pitch_mdeg(pitch_mdeg), .yaw_mdeg(yaw_mdeg),
        .scl(scl), .sda(sda)
    );

    reg bus_active = 1'b0;
    integer bit_count = 0;
    integer byte_count = 0;
    integer transaction_count = 0;
    reg [7:0] bit_shift = 0;
    reg [7:0] address_byte, control_byte, payload_byte;
    reg [7:0] address_log [0:2400];
    reg [7:0] control_log [0:2400];
    reg [7:0] payload_log [0:2400];

    always @(negedge sda) begin
        if (scl === 1'b1) begin
            bus_active = 1'b1;
            bit_count = 0;
            byte_count = 0;
            bit_shift = 0;
            address_byte = 0;
            control_byte = 0;
            payload_byte = 0;
        end
    end

    always @(posedge scl) begin
        if (bus_active) begin
            if (byte_count < 3) begin
                if (bit_count < 8) begin
                    bit_shift = {bit_shift[6:0], sda};
                    if (bit_count == 7) begin
                        case (byte_count)
                            0: address_byte = bit_shift;
                            1: control_byte = bit_shift;
                            2: payload_byte = bit_shift;
                            default: begin end
                        endcase
                    end
                    bit_count = bit_count + 1;
                end else begin
                    bit_count = 0;
                    byte_count = byte_count + 1;
                end
            end
        end
    end

    always @(posedge sda) begin
        if (scl === 1'b1 && bus_active) begin
            if (transaction_count > 2400)
                $fatal(1, "unexpected number of I2C transactions");
            address_log[transaction_count] = address_byte;
            control_log[transaction_count] = control_byte;
            payload_log[transaction_count] = payload_byte;
            transaction_count = transaction_count + 1;
            bus_active = 1'b0;
        end
    end

    reg [7:0] expected_init [0:25];
    integer i, p, c, base;
    initial begin
        expected_init[0]=8'hAE; expected_init[1]=8'h00;
        expected_init[2]=8'h10; expected_init[3]=8'h40;
        expected_init[4]=8'h81; expected_init[5]=8'hCF;
        expected_init[6]=8'hA1; expected_init[7]=8'hC8;
        expected_init[8]=8'hA6; expected_init[9]=8'hA8;
        expected_init[10]=8'h3F; expected_init[11]=8'hD3;
        expected_init[12]=8'h00; expected_init[13]=8'hD5;
        expected_init[14]=8'h80; expected_init[15]=8'hD9;
        expected_init[16]=8'hF1; expected_init[17]=8'hDA;
        expected_init[18]=8'h12; expected_init[19]=8'hDB;
        expected_init[20]=8'h30; expected_init[21]=8'h20;
        expected_init[22]=8'h02; expected_init[23]=8'h8D;
        expected_init[24]=8'h14; expected_init[25]=8'hAF;

        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        imu_ready = 1'b1;
        calibrating = 1'b1;
        roll_mdeg = -32'sd123456;
        pitch_mdeg = 32'sd30555;
        yaw_mdeg = 32'sd180000;
        wait (dut.clear_mode == 0);
        #1;
        if (dut.screen_character(2'd0,5'd4) !== "C" ||
            dut.screen_character(2'd0,5'd5) !== "A" ||
            dut.screen_character(2'd0,5'd6) !== "L")
            $fatal(1,"calibration status must display CAL");
        if (dut.screen_character(2'd1, 5'd5) !== "-" ||
            dut.screen_character(2'd1, 5'd6) !== "1" ||
            dut.screen_character(2'd1, 5'd7) !== "2" ||
            dut.screen_character(2'd1, 5'd8) !== "3" ||
            dut.screen_character(2'd1, 5'd10) !== "5")
            $fatal(1, "OLED roll format failed for -123.5 degrees");
        if (dut.screen_character(2'd2, 5'd6) !== "+" ||
            dut.screen_character(2'd2, 5'd7) !== "0" ||
            dut.screen_character(2'd2, 5'd8) !== "3" ||
            dut.screen_character(2'd2, 5'd9) !== "0" ||
            dut.screen_character(2'd2, 5'd11) !== "6")
            $fatal(1, "OLED pitch format failed for +030.6 degrees");
        if (dut.screen_character(2'd3, 5'd8) !== "+" ||
            dut.screen_character(2'd3, 5'd9) !== "1" ||
            dut.screen_character(2'd3, 5'd10) !== "8" ||
            dut.screen_character(2'd3, 5'd11) !== "0" ||
            dut.screen_character(2'd3, 5'd13) !== "0")
            $fatal(1, "OLED yaw format failed for +180.0 degrees");

        // Live inputs may change while the display sends a frame. Its snapshot
        // must remain consistent until the following frame begins.
        roll_mdeg = 0; pitch_mdeg = 0; yaw_mdeg = 0;
        #1;
        if (dut.screen_character(2'd1,5'd5) !== "-" ||
            dut.screen_character(2'd1,5'd10) !== "5")
            $fatal(1,"display inputs changed inside a frame");
        wait (transaction_count >= 1598);
        for (i = 0; i < 26; i = i + 1) begin
            if (address_log[i] !== 8'h78 || control_log[i] !== 8'h00 ||
                payload_log[i] !== expected_init[i])
                $fatal(1, "OLED init transaction %0d was %02x/%02x/%02x, expected 78/00/%02x",
                       i, address_log[i], control_log[i], payload_log[i], expected_init[i]);
        end
        if (payload_log[26] !== 8'hB0 || payload_log[27] !== 8'h00 ||
            payload_log[28] !== 8'h10 || address_log[29] !== 8'h78 ||
            control_log[29] !== 8'h40 || payload_log[29] !== 0)
            $fatal(1, "OLED page mode setup or initial clear failed");

        for (p=0; p<8; p=p+1) begin
            base=26+p*131;
            if (payload_log[base] !== (8'hB0+p) ||
                payload_log[base+1] !== 0 || payload_log[base+2] !== 8'h10)
                $fatal(1,"clear page %0d address mismatch",p);
            for (c=0; c<128; c=c+1)
                if (address_log[base+3+c] !== 8'h78 ||
                    control_log[base+3+c] !== 8'h40 || payload_log[base+3+c] !== 0)
                    $fatal(1,"clear pixel mismatch at page %0d column %0d",p,c);
        end
        for (p=0; p<4; p=p+1) begin
            base=1074+p*131;
            if (payload_log[base] !== (8'hB0+p*2))
                $fatal(1,"display row %0d page address mismatch",p);
        end
        if (payload_log[1077] !== 0 || payload_log[1078] !== 0 ||
            payload_log[1079] !== 8'h41 || payload_log[1080] !== 8'h7F ||
            payload_log[1081] !== 8'h41 || payload_log[1082] !== 0)
            $fatal(1,"first character I differs from official 6x8 font");

        $display("PASS: SSD1306 init/address, page setup, open-drain write framing, signed OLED text formatting");
        completed = 1'b1;
        $finish;
    end
    initial begin
        #20000000;
        $fatal(1,"OLED test timeout");
    end
endmodule
