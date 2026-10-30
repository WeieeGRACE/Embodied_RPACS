`timescale 1ns/1ps
// 200 Hz, +/-8 g, +/-2000 dps IMU660RA attitude test estimator.
// One shared iterative CORDIC and square-root unit finish well before the
// next 5 ms sample. Angles are signed millidegrees, wrapped to [-180, 180).
module imu_attitude_fixed #(
    parameter integer CALIBRATION_SAMPLES = 200,
    parameter integer GYRO_WEIGHT = 98,
    parameter integer FILTER_DENOMINATOR = 100
)(
    input wire clk, rst_n, ready_in, sample_valid,
    input wire signed [15:0] accel_x, accel_y, accel_z,
    input wire signed [15:0] gyro_x, gyro_y, gyro_z,
    output wire calibrating,
    output reg update_valid, // One pulse per committed attitude sample.
    output reg attitude_valid,
    output reg signed [31:0] roll_mdeg, pitch_mdeg, yaw_mdeg,
    output reg signed [15:0] gyro_bias_x, gyro_bias_y, gyro_bias_z
);
    localparam integer COUNT_WIDTH = (CALIBRATION_SAMPLES < 2) ? 1 : $clog2(CALIBRATION_SAMPLES + 1);
    // Signed 16-bit samples summed N times need 16+ceil(log2(N)) bits.
    localparam integer SUM_WIDTH = 16 + $clog2(CALIBRATION_SAMPLES);
    localparam [4:0] IDLE=0, ROLL_START=1, ROLL_WAIT=2, SQUARE_Y=3,
        SQUARE_Z=4, SQUARE_X=5, SQRT_WAIT=6, PITCH_WAIT=7, UPDATE=8,
        ROLL_DIV_WAIT=9, PITCH_DIV_WAIT=10, BLEND=11,
        CAL_LOW=12, CAL_HIGH=13, CAL_MEAN=14,
        CAL_X_WAIT=15, CAL_Y_WAIT=16, CAL_Z_WAIT=17;
    reg [4:0] state;
    reg bias_valid;
    reg [COUNT_WIDTH-1:0] calibration_count;
    reg signed [SUM_WIDTH-1:0] gyro_sum_x, gyro_sum_y, gyro_sum_z;
    reg signed [SUM_WIDTH-1:0] calibration_sample_x, calibration_sample_y, calibration_sample_z;
    reg calibration_last;
    reg carry_x, carry_y, carry_z;
    wire [12:0] low_sum_x = {1'b0,gyro_sum_x[11:0]} + {1'b0,calibration_sample_x[11:0]};
    wire [12:0] low_sum_y = {1'b0,gyro_sum_y[11:0]} + {1'b0,calibration_sample_y[11:0]};
    wire [12:0] low_sum_z = {1'b0,gyro_sum_z[11:0]} + {1'b0,calibration_sample_z[11:0]};
    wire [SUM_WIDTH-13:0] high_sum_x = gyro_sum_x[SUM_WIDTH-1:12] + calibration_sample_x[SUM_WIDTH-1:12] + carry_x;
    wire [SUM_WIDTH-13:0] high_sum_y = gyro_sum_y[SUM_WIDTH-1:12] + calibration_sample_y[SUM_WIDTH-1:12] + carry_y;
    wire [SUM_WIDTH-13:0] high_sum_z = gyro_sum_z[SUM_WIDTH-1:12] + calibration_sample_z[SUM_WIDTH-1:12] + carry_z;
    assign calibrating = ready_in && !bias_valid;

    reg bias_division_start;
    reg signed [SUM_WIDTH-1:0] bias_division_input;
    wire signed [15:0] bias_division_quotient;
    wire bias_division_done;
    // Calibration is infrequent; sharing a bit-serial divider avoids a long
    // constant-division path between the sum and the bias registers.
    imu_attitude_bias_divider #(
        .INPUT_WIDTH(SUM_WIDTH), .DIVISOR(CALIBRATION_SAMPLES)
    ) bias_divider (
        .clk(clk), .rst_n(rst_n), .enable(ready_in),
        .start(bias_division_start), .dividend(bias_division_input),
        .done(bias_division_done), .quotient(bias_division_quotient)
    );

    reg signed [15:0] ax, ay, az, square_operand;
    reg signed [16:0] gx, gy, gz;
    reg signed [31:0] gyro_numerator_x, gyro_numerator_y, gyro_numerator_z;
    reg signed [12:0] remainder_x, remainder_y, remainder_z;
    // Keeping the fractional phase removes a per-sample rounding drift.
    // For signed truncating division the remainder equals the low bits with
    // the dividend's sign, so numerator - (numerator/4096)*4096 reduces to
    // the signed low 12 bits and no subtract/multiply network is needed.
    // (Same optimization as the main project's src/imu copy; it shortens the
    //  gyro_numerator -> step/remainder datapath substantially.)
    wire signed [31:0] step_x = gyro_numerator_x / 32'sd4096;
    wire signed [31:0] step_y = gyro_numerator_y / 32'sd4096;
    wire signed [31:0] step_z = gyro_numerator_z / 32'sd4096;
    wire signed [12:0] remainder_next_x = {{2{gyro_numerator_x[11]}},gyro_numerator_x[10:0]};
    wire signed [12:0] remainder_next_y = {{2{gyro_numerator_y[11]}},gyro_numerator_y[10:0]};
    wire signed [12:0] remainder_next_z = {{2{gyro_numerator_z[11]}},gyro_numerator_z[10:0]};
    reg [31:0] y_squared, z_squared;
    wire [31:0] square_product = $signed(square_operand) * $signed(square_operand);
    wire [32:0] norm_squared = {1'b0,square_product} + {1'b0,y_squared} + {1'b0,z_squared};
    reg accel_norm_valid;
    reg signed [31:0] accel_roll, accel_pitch;
    reg signed [31:0] roll_prediction, pitch_prediction, yaw_prediction;
    reg signed [19:0] roll_error, pitch_error;
    reg signed [31:0] division_input, roll_correction, pitch_correction;
    reg division_start;
    wire signed [31:0] division_quotient;
    wire division_done;
    wire signed [31:0] roll_error_calc = wrap_angle(accel_roll-roll_prediction);
    wire signed [31:0] pitch_error_calc = wrap_angle(accel_pitch-pitch_prediction);

    reg atan_start, sqrt_start;
    reg signed [31:0] atan_x, atan_y;
    wire signed [31:0] atan_result;
    wire atan_done;
    reg [32:0] sqrt_value;
    wire [16:0] sqrt_result;
    wire sqrt_done;
    imu_attitude_atan2 atan_unit (
        .clk(clk), .rst_n(rst_n), .start(atan_start), .x_in(atan_x), .y_in(atan_y),
        .done(atan_done), .angle_mdeg(atan_result)
    );
    imu_attitude_sqrt sqrt_unit (
        .clk(clk), .rst_n(rst_n), .start(sqrt_start), .value(sqrt_value),
        .done(sqrt_done), .result(sqrt_result)
    );
    imu_attitude_divider #(.DIVISOR(FILTER_DENOMINATOR)) divider_unit (
        .clk(clk), .rst_n(rst_n), .start(division_start),
        .dividend(division_input), .done(division_done), .quotient(division_quotient)
    );

    function signed [31:0] wrap_angle;
        input signed [31:0] value;
        begin
            if (value >= 180000) wrap_angle = value - 360000;
            else if (value < -180000) wrap_angle = value + 360000;
            else wrap_angle = value;
        end
    endfunction
    wire signed [31:0] roll_next = wrap_angle(roll_mdeg + step_x);
    wire signed [31:0] pitch_next = wrap_angle(pitch_mdeg + step_y);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; bias_valid<=0; calibration_count<=0;
            gyro_sum_x<=0; gyro_sum_y<=0; gyro_sum_z<=0;
            calibration_sample_x<=0; calibration_sample_y<=0; calibration_sample_z<=0;
            calibration_last<=0; carry_x<=0; carry_y<=0; carry_z<=0;
            gyro_bias_x<=0; gyro_bias_y<=0; gyro_bias_z<=0;
            bias_division_start<=0; bias_division_input<=0;
            remainder_x<=0; remainder_y<=0; remainder_z<=0;
            update_valid<=0; attitude_valid<=0; roll_mdeg<=0; pitch_mdeg<=0; yaw_mdeg<=0;
            ax<=0; ay<=0; az<=0; gx<=0; gy<=0; gz<=0;
            square_operand<=0; y_squared<=0; z_squared<=0; accel_norm_valid<=0;
            gyro_numerator_x<=0; gyro_numerator_y<=0; gyro_numerator_z<=0;
            accel_roll<=0; accel_pitch<=0;
            roll_prediction<=0; pitch_prediction<=0; yaw_prediction<=0;
            roll_error<=0; pitch_error<=0;
            division_input<=0; division_start<=0;
            roll_correction<=0; pitch_correction<=0;
            atan_start<=0; atan_x<=0; atan_y<=0; sqrt_start<=0; sqrt_value<=0;
        end else begin
            update_valid<=0;
            atan_start<=0;
            sqrt_start<=0;
            division_start<=0;
            bias_division_start<=0;
            if (!ready_in) begin
                state<=IDLE; bias_valid<=0; calibration_count<=0;
                gyro_sum_x<=0; gyro_sum_y<=0; gyro_sum_z<=0;
                calibration_sample_x<=0; calibration_sample_y<=0; calibration_sample_z<=0;
                calibration_last<=0; carry_x<=0; carry_y<=0; carry_z<=0;
                gyro_bias_x<=0; gyro_bias_y<=0; gyro_bias_z<=0;
                remainder_x<=0; remainder_y<=0; remainder_z<=0;
                update_valid<=0; attitude_valid<=0; roll_mdeg<=0; pitch_mdeg<=0; yaw_mdeg<=0;
                division_input<=0; roll_correction<=0; pitch_correction<=0;
            end else case (state)
                IDLE: if (sample_valid) begin
                    if (!bias_valid) begin
                        calibration_sample_x<={{(SUM_WIDTH-16){gyro_x[15]}},gyro_x};
                        calibration_sample_y<={{(SUM_WIDTH-16){gyro_y[15]}},gyro_y};
                        calibration_sample_z<={{(SUM_WIDTH-16){gyro_z[15]}},gyro_z};
                        calibration_last<=(calibration_count == CALIBRATION_SAMPLES-1);
                        if (calibration_count != CALIBRATION_SAMPLES-1)
                            calibration_count<=calibration_count+1'b1;
                        state<=CAL_LOW;
                    end else begin
                        ax<=accel_x; ay<=accel_y; az<=accel_z;
                        gx<=$signed({gyro_x[15],gyro_x})-$signed({gyro_bias_x[15],gyro_bias_x});
                        gy<=$signed({gyro_y[15],gyro_y})-$signed({gyro_bias_y[15],gyro_bias_y});
                        gz<=$signed({gyro_z[15],gyro_z})-$signed({gyro_bias_z[15],gyro_bias_z});
                        state<=ROLL_START;
                    end
                end
                CAL_LOW: begin
                    gyro_sum_x[11:0]<=low_sum_x[11:0]; carry_x<=low_sum_x[12];
                    gyro_sum_y[11:0]<=low_sum_y[11:0]; carry_y<=low_sum_y[12];
                    gyro_sum_z[11:0]<=low_sum_z[11:0]; carry_z<=low_sum_z[12];
                    state<=CAL_HIGH;
                end
                CAL_HIGH: begin
                    gyro_sum_x[SUM_WIDTH-1:12]<=high_sum_x;
                    gyro_sum_y[SUM_WIDTH-1:12]<=high_sum_y;
                    gyro_sum_z[SUM_WIDTH-1:12]<=high_sum_z;
                    state<=calibration_last ? CAL_MEAN : IDLE;
                end
                CAL_MEAN: begin
                    bias_division_input<=gyro_sum_x;
                    bias_division_start<=1;
                    state<=CAL_X_WAIT;
                end
                CAL_X_WAIT: if (bias_division_done) begin
                    gyro_bias_x<=bias_division_quotient;
                    bias_division_input<=gyro_sum_y;
                    bias_division_start<=1;
                    state<=CAL_Y_WAIT;
                end
                CAL_Y_WAIT: if (bias_division_done) begin
                    gyro_bias_y<=bias_division_quotient;
                    bias_division_input<=gyro_sum_z;
                    bias_division_start<=1;
                    state<=CAL_Z_WAIT;
                end
                CAL_Z_WAIT: if (bias_division_done) begin
                    gyro_bias_z<=bias_division_quotient;
                    bias_valid<=1;
                    state<=IDLE;
                end
                ROLL_START: begin
                    // Board mounting: level Z acceleration is approximately -1 g.
                    atan_x<=-$signed({{16{az[15]}},az});
                    atan_y<=$signed({{16{ay[15]}},ay});
                    atan_start<=1;
                    // 1000/(16.384*200) millidegrees/count = 1250/4096.
                    gyro_numerator_x<=gx*32'sd1250+$signed(remainder_x);
                    gyro_numerator_y<=gy*32'sd1250+$signed(remainder_y);
                    gyro_numerator_z<=gz*32'sd1250+$signed(remainder_z);
                    state<=ROLL_WAIT;
                end
                ROLL_WAIT: if (atan_done) begin
                    accel_roll<=atan_result;
                    square_operand<=ay;
                    state<=SQUARE_Y;
                end
                SQUARE_Y: begin
                    y_squared<=square_product; square_operand<=az; state<=SQUARE_Z;
                end
                SQUARE_Z: begin
                    z_squared<=square_product; square_operand<=ax; state<=SQUARE_X;
                end
                SQUARE_X: begin
                    accel_norm_valid<=(norm_squared>=33'd10737418 && norm_squared<=33'd24159191);
                    sqrt_value<={1'b0,y_squared}+{1'b0,z_squared};
                    sqrt_start<=1;
                    state<=SQRT_WAIT;
                end
                SQRT_WAIT: if (sqrt_done) begin
                    atan_x<=$signed({15'd0,sqrt_result});
                    atan_y<=-$signed({{16{ax[15]}},ax});
                    atan_start<=1;
                    state<=PITCH_WAIT;
                end
                PITCH_WAIT: if (atan_done) begin
                    // Register prediction before the wrapped error/filter arithmetic.
                    roll_prediction<=roll_next; pitch_prediction<=pitch_next;
                    yaw_prediction<=wrap_angle(yaw_mdeg+step_z);
                    accel_pitch<=atan_result; state<=UPDATE;
                end
                UPDATE: begin
                    if (!attitude_valid) begin
                        roll_mdeg<=accel_norm_valid ? accel_roll : 32'sd0;
                        pitch_mdeg<=accel_norm_valid ? accel_pitch : 32'sd0;
                        update_valid<=1; yaw_mdeg<=0; attitude_valid<=1; state<=IDLE;
                    end else begin
                        remainder_x<=remainder_next_x;
                        remainder_y<=remainder_next_y;
                        remainder_z<=remainder_next_z;
                        if (accel_norm_valid) begin
                            roll_error<=roll_error_calc[19:0];
                            pitch_error<=pitch_error_calc[19:0];
                            division_input<=roll_error_calc * (FILTER_DENOMINATOR-GYRO_WEIGHT);
                            division_start<=1;
                            state<=ROLL_DIV_WAIT;
                        end else begin
                            roll_mdeg<=roll_next; pitch_mdeg<=pitch_next;
                            update_valid<=1; yaw_mdeg<=wrap_angle(yaw_mdeg+step_z); state<=IDLE;
                        end
                    end
                end
                ROLL_DIV_WAIT: if (division_done) begin
                    roll_correction<=division_quotient;
                    division_input<=pitch_error * (FILTER_DENOMINATOR-GYRO_WEIGHT);
                    division_start<=1;
                    state<=PITCH_DIV_WAIT;
                end
                PITCH_DIV_WAIT: if (division_done) begin
                    pitch_correction<=division_quotient;
                    state<=BLEND;
                end
                BLEND: begin
                    // Blend the shortest angular difference across +/-180 degrees.
                    roll_mdeg<=wrap_angle(roll_prediction+$signed(roll_correction));
                    pitch_mdeg<=wrap_angle(pitch_prediction+$signed(pitch_correction));
                    update_valid<=1; yaw_mdeg<=yaw_prediction; state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule

// Signed mean of the accumulated 16-bit gyro samples. The magnitude is
// divided one bit per cycle, then the sign is restored (truncate toward zero).
module imu_attitude_bias_divider #(
    parameter integer INPUT_WIDTH=24,
    parameter integer DIVISOR=200
)(
    input wire clk, rst_n, enable, start,
    input wire signed [INPUT_WIDTH-1:0] dividend,
    output reg done,
    output reg signed [15:0] quotient
);
    localparam integer REM_WIDTH=(DIVISOR<2) ? 1 : $clog2(DIVISOR+1);
    localparam integer INDEX_WIDTH=$clog2(INPUT_WIDTH);
    localparam [REM_WIDTH:0] DIVISOR_VALUE=DIVISOR;
    reg busy, negative;
    reg [INDEX_WIDTH-1:0] iteration;
    reg [INPUT_WIDTH-1:0] dividend_shift, quotient_shift;
    reg [REM_WIDTH-1:0] remainder;
    wire [REM_WIDTH:0] shifted_remainder={remainder,dividend_shift[INPUT_WIDTH-1]};
    wire take_bit=shifted_remainder>=DIVISOR_VALUE;
    wire [REM_WIDTH:0] reduced_remainder=shifted_remainder-DIVISOR_VALUE;
    wire [INPUT_WIDTH-1:0] quotient_next={quotient_shift[INPUT_WIDTH-2:0],take_bit};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy<=0; negative<=0; iteration<=0; done<=0; quotient<=0;
            dividend_shift<=0; quotient_shift<=0; remainder<=0;
        end else if (!enable) begin
            busy<=0; negative<=0; iteration<=0; done<=0; quotient<=0;
            dividend_shift<=0; quotient_shift<=0; remainder<=0;
        end else begin
            done<=0;
            if (start && !busy) begin
                busy<=1; negative<=dividend[INPUT_WIDTH-1]; iteration<=0;
                dividend_shift<=dividend[INPUT_WIDTH-1] ? (~dividend+1'b1) : dividend;
                quotient_shift<=0; remainder<=0;
            end else if (busy) begin
                dividend_shift<={dividend_shift[INPUT_WIDTH-2:0],1'b0};
                remainder<=take_bit ? reduced_remainder[REM_WIDTH-1:0] : shifted_remainder[REM_WIDTH-1:0];
                quotient_shift<=quotient_next;
                if (iteration==INPUT_WIDTH-1) begin
                    busy<=0; done<=1;
                    quotient<=negative ? -$signed(quotient_next[15:0]) : $signed(quotient_next[15:0]);
                end else iteration<=iteration+1'b1;
            end
        end
    end
endmodule

// Vectoring CORDIC; one rotation per clock, with fractional vector precision.
module imu_attitude_atan2(
    input wire clk, rst_n, start,
    input wire signed [31:0] x_in, y_in,
    output reg done,
    output reg signed [31:0] angle_mdeg
);
    reg busy;
    reg [3:0] iteration;
    reg signed [31:0] x, y, z;
    wire signed [31:0] x_shift=x>>>iteration;
    wire signed [31:0] y_shift=y>>>iteration;
    wire signed [31:0] step=atan_step(iteration);
    wire signed [31:0] x_next=(y>0) ? x+y_shift : (y<0) ? x-y_shift : x;
    wire signed [31:0] y_next=(y>0) ? y-x_shift : (y<0) ? y+x_shift : y;
    wire signed [31:0] z_next=(y>0) ? z+step : (y<0) ? z-step : z;
    function signed [31:0] atan_step;
        input [3:0] index;
        begin
            case(index)
                0:atan_step=45000; 1:atan_step=26565; 2:atan_step=14036;
                3:atan_step=7130; 4:atan_step=3576; 5:atan_step=1789;
                6:atan_step=895; 7:atan_step=448; 8:atan_step=224;
                9:atan_step=112; 10:atan_step=56; 11:atan_step=28;
                12:atan_step=14; 13:atan_step=7; 14:atan_step=3;
                default:atan_step=2;
            endcase
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy<=0; done<=0; iteration<=0; x<=0; y<=0; z<=0; angle_mdeg<=0;
        end else begin
            done<=0;
            if (start && !busy) begin
                busy<=1; iteration<=0;
                if (x_in<0) begin
                    x<=-(x_in<<<12); y<=-(y_in<<<12);
                    z<=(y_in>=0) ? 32'sd180000 : -32'sd180000;
                end else begin x<=x_in<<<12; y<=y_in<<<12; z<=0; end
            end else if (busy) begin
                x<=x_next; y<=y_next; z<=z_next;
                if (iteration==15) begin
                    busy<=0; done<=1;
                    if (z_next>=180000) angle_mdeg<=z_next-360000;
                    else if (z_next< -180000) angle_mdeg<=z_next+360000;
                    else angle_mdeg<=z_next;
                end else iteration<=iteration+1'b1;
            end
        end
    end
endmodule

// Restoring integer square root, two input bits per clock (17 clocks).
module imu_attitude_sqrt(
    input wire clk, rst_n, start,
    input wire [32:0] value,
    output reg done,
    output reg [16:0] result
);
    reg busy;
    reg [4:0] iteration;
    reg [33:0] radicand;
    reg [34:0] remainder;
    reg [16:0] root;
    wire [34:0] shifted_remainder={remainder[32:0],radicand[33:32]};
    wire [34:0] trial={16'd0,root,2'b01};
    wire take_bit=shifted_remainder>=trial;
    wire [16:0] next_root={root[15:0],take_bit};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy<=0; done<=0; iteration<=0; radicand<=0; remainder<=0; root<=0; result<=0;
        end else begin
            done<=0;
            if (start && !busy) begin
                busy<=1; iteration<=0; radicand<={1'b0,value}; remainder<=0; root<=0;
            end else if (busy) begin
                radicand<={radicand[31:0],2'b00};
                remainder<=take_bit ? shifted_remainder-trial : shifted_remainder;
                root<=next_root;
                if (iteration==16) begin
                    busy<=0; done<=1; result<=next_root;
                end else iteration<=iteration+1'b1;
            end
        end
    end
endmodule

// Signed restoring divide. Two axes share this 32-cycle unit.
module imu_attitude_divider #(
    parameter integer DIVISOR=100
)(
    input wire clk, rst_n, start,
    input wire signed [31:0] dividend,
    output reg done,
    output reg signed [31:0] quotient
);
    localparam integer ITERATIONS=32;
    localparam [8:0] DIVISOR_VALUE=DIVISOR;
    reg busy, negative;
    reg [5:0] iteration;
    reg [31:0] dividend_shift, quotient_shift;
    reg [7:0] remainder;
    wire [8:0] shifted_remainder={remainder,dividend_shift[31]};
    wire quotient_bit=(shifted_remainder>=DIVISOR_VALUE);
    wire [8:0] reduced_remainder=shifted_remainder-DIVISOR_VALUE;
    wire [31:0] quotient_next={quotient_shift[30:0],quotient_bit};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy<=0; negative<=0; iteration<=0;
            dividend_shift<=0; quotient_shift<=0; remainder<=0;
            quotient<=0; done<=0;
        end else begin
            done<=0;
            if (start && !busy) begin
                busy<=1; negative<=dividend[31]; iteration<=0;
                dividend_shift<=dividend[31] ? (~dividend+32'd1) : dividend;
                quotient_shift<=0; remainder<=0;
            end else if (busy) begin
                dividend_shift<={dividend_shift[30:0],1'b0};
                quotient_shift<=quotient_next;
                remainder<=quotient_bit ? reduced_remainder[7:0] : shifted_remainder[7:0];
                if (iteration==ITERATIONS-1) begin
                    busy<=0; done<=1;
                    quotient<=negative ? -$signed(quotient_next) : $signed(quotient_next);
                end else iteration<=iteration+1'b1;
            end
        end
    end
endmodule
