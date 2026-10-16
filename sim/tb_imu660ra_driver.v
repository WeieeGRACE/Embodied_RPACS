`timescale 1ns/1ps

module tb_imu660ra_driver #(parameter FAIL_FIRST_INIT=0);
    reg clk = 0;
    reg rst_n = 0;
    reg miso = 0;
    wire sck, mosi, cs_n;
    wire signed [15:0] ax, ay, az, gx, gy, gz;
    wire sample_valid, int1_sync, int2_sync;
    wire [31:0] sample_sequence;
    wire [7:0] chip_id, init_status, error_code;
    wire ready;

    always #10 clk = ~clk;

    imu660ra_driver #(
        .CLK_HZ(1000), .SPI_HALF_PERIOD(1), .POWERUP_CYCLES(2),
        .CONFIG_WAIT_CYCLES(2), .ID_RETRY_CYCLES(2), .RETRY_CYCLES(20),
        .SAMPLE_CYCLES(100)
    ) dut (
        .clk(clk), .rst_n(rst_n), .spi_miso(miso), .imu_int1(1'b0), .imu_int2(1'b0),
        .spi_sclk(sck), .spi_mosi(mosi), .spi_cs_n(cs_n),
        .accel_x(ax), .accel_y(ay), .accel_z(az),
        .gyro_x(gx), .gyro_y(gy), .gyro_z(gz),
        .sample_valid(sample_valid), .sample_sequence(sample_sequence),
        .chip_id(chip_id), .init_status(init_status), .ready(ready),
        .error_code(error_code), .int1_sync(int1_sync), .int2_sync(int2_sync)
        , .power_conf_readback(), .init_ctrl_readback(), .sensor_error()
    );

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
        #20000000;
        $fatal(1, "timeout waiting for initialization");
    end

    always @(negedge cs_n) begin
        if (command == 8'h5E) begin
            if (config_bytes != 8192 || config_first != 8'hC8 || config_last != 8'hC1)
                $fatal(1, "bad config stream count=%0d first=%h last=%h", config_bytes, config_first, config_last);
        end
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
                        response_shift = 56'h00240000000000;
                    else if (tx_shift == 8'hA1)
                        response_shift = (FAIL_FIRST_INIT && config_uploads == 1) ? 56'h00020000000000 : 56'h00010000000000;
                    else if (tx_shift == 8'h8C)
                        response_shift = 56'h003412feff0040;
                    else if (tx_shift == 8'h92)
                        response_shift = 56'h0078560080ff7f;
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
        repeat (4) @(posedge clk);
        rst_n = 1;
        if (FAIL_FIRST_INIT) begin
            wait(error_code != 0);
            #1;
            if(error_code != 5 || ready || init_status != 2)
                $fatal(1,"failure diagnostics incorrect");
            $display("PASS rejected config reports ERR=5 ISTAT=2 READY=0");
            wait(ready);
            if(config_uploads != 2) $fatal(1,"expected one automatic retry");
        end
        wait (ready || error_code != 0);
        #1;
        if (!ready || error_code != 0 || chip_id != 8'h24 || init_status != 8'h01)
            $fatal(1, "IMU init failed: ready=%b err=%h id=%h istat=%h", ready, error_code, chip_id, init_status);
        $display("PASS ready=%b err=%h id=%h istat=%h", ready, error_code, chip_id, init_status);
        wait(sample_valid);
        #1;
        if (sample_sequence != 1 || ax !== 16'h1234 || ay !== 16'hfffe || az !== 16'h4000 ||
            gx !== 16'h5678 || gy !== 16'h8000 || gz !== 16'h7fff)
            $fatal(1,"bad six-axis decoding seq=%0d ax=%h ay=%h az=%h gx=%h gy=%h gz=%h",sample_sequence,ax,ay,az,gx,gy,gz);
        $display("PASS six-axis burst, little-endian signed decoding and atomic sample");
        $finish;
    end
endmodule
