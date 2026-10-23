`timescale 1ns/1ps
module tb_imu_attitude_fixed;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n = 1'b0;
    reg ready_in = 1'b0;
    reg sample_valid = 1'b0;
    reg signed [15:0] accel_x = 0, accel_y = 0, accel_z = 0;
    reg signed [15:0] gyro_x = 0, gyro_y = 0, gyro_z = 0;
    wire calibrating, attitude_valid;
    wire signed [31:0] roll_mdeg, pitch_mdeg, yaw_mdeg;
    wire signed [15:0] gyro_bias_x, gyro_bias_y, gyro_bias_z;

    reg completed = 1'b0;
    imu_attitude_fixed #(.CALIBRATION_SAMPLES(200)) dut (
        .clk(clk), .rst_n(rst_n), .ready_in(ready_in),
        .sample_valid(sample_valid),
        .accel_x(accel_x), .accel_y(accel_y), .accel_z(accel_z),
        .gyro_x(gyro_x), .gyro_y(gyro_y), .gyro_z(gyro_z),
        .calibrating(calibrating), .attitude_valid(attitude_valid),
        .roll_mdeg(roll_mdeg), .pitch_mdeg(pitch_mdeg), .yaw_mdeg(yaw_mdeg),
        .gyro_bias_x(gyro_bias_x), .gyro_bias_y(gyro_bias_y), .gyro_bias_z(gyro_bias_z)
    );

    reg signed [31:0] formatted_angle;
    wire formatted_negative;
    wire [3:0] formatted_hundreds, formatted_tens, formatted_ones, formatted_tenths;
    attitude_angle_to_digits formatter (
        .angle_mdeg(formatted_angle), .negative(formatted_negative),
        .hundreds(formatted_hundreds), .tens(formatted_tens),
        .ones(formatted_ones), .tenths(formatted_tenths)
    );

    task automatic push_sample;
        input signed [15:0] ax, ay, az, gx, gy, gz;
        begin
            @(negedge clk);
            accel_x = ax; accel_y = ay; accel_z = az;
            gyro_x = gx; gyro_y = gy; gyro_z = gz;
            sample_valid = 1'b1;
            @(posedge clk);
            #1 sample_valid = 1'b0;
            repeat (160) @(negedge clk);
        end
    endtask

    task automatic reset_and_calibrate;
        input signed [15:0] bx, by, bz;
        integer sample_index;
        begin
            rst_n = 1'b0;
            ready_in = 1'b0;
            sample_valid = 1'b0;
            repeat (3) @(negedge clk);
            rst_n = 1'b1;
            ready_in = 1'b1;
            for (sample_index = 0; sample_index < 200; sample_index = sample_index + 1) begin
                push_sample(16'sd0, 16'sd0, -16'sd4096, bx, by, bz);
                if (sample_index < 199 && !calibrating)
                    $fatal(1, "calibration completed before 200 samples");
            end
            if (calibrating !== 1'b0)
                $fatal(1, "calibration did not complete after 200 samples");
            if (gyro_bias_x !== bx || gyro_bias_y !== by || gyro_bias_z !== bz)
                $fatal(1, "gyro bias average mismatch: %0d %0d %0d", gyro_bias_x, gyro_bias_y, gyro_bias_z);
        end
    endtask

    integer n;
    initial begin
        formatted_angle = -32'sd123456;
        #1;
        if (formatted_negative !== 1'b1 || formatted_hundreds !== 1 ||
            formatted_tens !== 2 || formatted_ones !== 3 || formatted_tenths !== 5)
            $fatal(1, "negative angle format must be -123.5 degrees");
        formatted_angle = 32'sd180000;
        #1;
        if (formatted_negative !== 1'b0 || formatted_hundreds !== 1 ||
            formatted_tens !== 8 || formatted_ones !== 0 || formatted_tenths !== 0)
            $fatal(1, "positive angle format must be +180.0 degrees");

        reset_and_calibrate(16'sd12, -16'sd8, 16'sd4);
        push_sample(16'sd0, 16'sd0, -16'sd4096, 16'sd12, -16'sd8, 16'sd4);
        if (!attitude_valid || roll_mdeg < -1000 || roll_mdeg > 1000 ||
            pitch_mdeg < -1000 || pitch_mdeg > 1000 || yaw_mdeg != 0)
            $fatal(1, "level initial attitude must be near zero");
        for (n = 0; n < 50; n = n + 1)
            push_sample(16'sd0, 16'sd0, -16'sd4096, 16'sd12, -16'sd8, 16'sd4);
        if (yaw_mdeg != 0)
            $fatal(1, "static calibrated yaw drifted: %0d mdeg", yaw_mdeg);

        for (n = 0; n < 100; n = n + 1)
            push_sample(16'sd0, 16'sd0, -16'sd4096, 16'sd12, -16'sd8, 16'sd104);
        if (yaw_mdeg < 3000 || yaw_mdeg > 3100)
            $fatal(1, "positive Z rate must produce positive relative yaw, got %0d", yaw_mdeg);
        for (n = 0; n < 100; n = n + 1)
            push_sample(16'sd0, 16'sd0, -16'sd4096, 16'sd12, -16'sd8, -16'sd96);
        if (yaw_mdeg < -2 || yaw_mdeg > 2)
            $fatal(1, "positive/negative yaw integration did not cancel: %0d", yaw_mdeg);

        reset_and_calibrate(16'sd0, 16'sd0, 16'sd0);
        push_sample(16'sd0, 16'sd4096, 16'sd0, 16'sd0, 16'sd0, 16'sd0);
        if (roll_mdeg < 85000 || roll_mdeg > 95000)
            $fatal(1, "positive Y acceleration should tilt roll positive, got %0d", roll_mdeg);

        reset_and_calibrate(16'sd0, 16'sd0, 16'sd0);
        push_sample(-16'sd2048, 16'sd0, -16'sd3547, 16'sd0, 16'sd0, 16'sd0);
        if (pitch_mdeg < 28000 || pitch_mdeg > 32000)
            $fatal(1, "negative X acceleration should tilt pitch positive, got %0d", pitch_mdeg);

        $display("PASS: calibration, bias removal, yaw direction/cancellation, roll/pitch, signed angle formatting");
        completed = 1'b1;
        $finish;
    end
    initial begin
        #20000000;
        $fatal(1, "attitude test timeout");
    end
endmodule
