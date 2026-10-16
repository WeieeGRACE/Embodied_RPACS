`timescale 1ns/1ps
// 20 Hz atomic text snapshots for the IMU bench. UART is 115200 8N1.
module imu_uart_telemetry #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200,
    parameter integer REPORT_CYCLES = CLK_HZ / 20
)(
    input wire clk,
    input wire rst_n,
    input wire [7:0] chip_id,
    input wire [7:0] init_status,
    input wire [7:0] power_conf_readback,
    input wire [7:0] init_ctrl_readback,
    input wire [7:0] sensor_error,
    input wire ready_in,
    input wire [7:0] error_code,
    input wire [31:0] sample_sequence,
    input wire int1,
    input wire int2,
    input wire signed [15:0] accel_x,
    input wire signed [15:0] accel_y,
    input wire signed [15:0] accel_z,
    input wire signed [15:0] gyro_x,
    input wire signed [15:0] gyro_y,
    input wire signed [15:0] gyro_z,
    output wire tx
);
    localparam ST_IDLE = 0;
    localparam ST_LABEL = 1;
    localparam ST_NUMBER = 2;
    localparam ST_DIGITS = 3;
    localparam ST_SEPARATOR = 4;
    localparam ST_CR = 5;
    localparam ST_LF = 6;
    localparam ST_SEND = 7;

    reg [3:0] state;
    reg [31:0] interval_count;
    reg [3:0] field;
    reg [3:0] label_index;
    reg [8:0] line_length;
    reg [8:0] line_index;
    reg [7:0] line [0:255];
    reg [7:0] digits [0:9];
    reg [3:0] digit_count;
    reg [31:0] magnitude;

    reg [7:0] snap_id;
    reg [7:0] snap_init_status;
    reg [7:0] snap_power, snap_ctrl, snap_sensor_error;
    reg snap_ready;
    reg [7:0] snap_error;
    reg [31:0] snap_sequence;
    reg snap_int1;
    reg snap_int2;
    reg signed [15:0] snap_ax, snap_ay, snap_az;
    reg signed [15:0] snap_gx, snap_gy, snap_gz;

    wire sender_ready;
    wire sender_done;
    wire sender_valid = (state == ST_SEND);
    wire [31:0] field_value;
    wire field_signed;
    wire [3:0] label_length;
    wire [7:0] current_label_char;

    function [3:0] field_label_length;
        input [3:0] f;
        begin
            case (f)
                4'd0: field_label_length = 3; // ID=
                4'd1: field_label_length = 6; // READY=
                4'd2: field_label_length = 4; // ERR=
                4'd3: field_label_length = 4; // SEQ=
                4'd4: field_label_length = 5; // INT1=
                4'd5: field_label_length = 5; // INT2=
                4'd6: field_label_length = 6; // ISTAT=
                4'd13: field_label_length = 4; // PWR=
                4'd14: field_label_length = 5; // CTRL=
                4'd15: field_label_length = 5; // SERR=
                default: field_label_length = 3; // AX= ... GZ=
            endcase
        end
    endfunction

    function [7:0] field_label_char;
        input [3:0] f;
        input [3:0] n;
        begin
            case (f)
                4'd0: case (n) 0: field_label_char="I"; 1: field_label_char="D"; default: field_label_char="="; endcase
                4'd1: case (n) 0: field_label_char="R"; 1: field_label_char="E"; 2: field_label_char="A"; 3: field_label_char="D"; 4: field_label_char="Y"; default: field_label_char="="; endcase
                4'd2: case (n) 0: field_label_char="E"; 1: field_label_char="R"; 2: field_label_char="R"; default: field_label_char="="; endcase
                4'd3: case (n) 0: field_label_char="S"; 1: field_label_char="E"; 2: field_label_char="Q"; default: field_label_char="="; endcase
                4'd4: case (n) 0: field_label_char="I"; 1: field_label_char="N"; 2: field_label_char="T"; 3: field_label_char="1"; default: field_label_char="="; endcase
                4'd5: case (n) 0: field_label_char="I"; 1: field_label_char="N"; 2: field_label_char="T"; 3: field_label_char="2"; default: field_label_char="="; endcase
                4'd6: case (n) 0: field_label_char="I"; 1: field_label_char="S"; 2: field_label_char="T"; 3: field_label_char="A"; 4: field_label_char="T"; default: field_label_char="="; endcase
                4'd7: case (n) 0: field_label_char="A"; 1: field_label_char="X"; default: field_label_char="="; endcase
                4'd8: case (n) 0: field_label_char="A"; 1: field_label_char="Y"; default: field_label_char="="; endcase
                4'd9: case (n) 0: field_label_char="A"; 1: field_label_char="Z"; default: field_label_char="="; endcase
                4'd10: case (n) 0: field_label_char="G"; 1: field_label_char="X"; default: field_label_char="="; endcase
                4'd11: case (n) 0: field_label_char="G"; 1: field_label_char="Y"; default: field_label_char="="; endcase
                4'd13: case (n) 0: field_label_char="P"; 1: field_label_char="W"; 2: field_label_char="R"; default: field_label_char="="; endcase
                4'd14: case (n) 0: field_label_char="C"; 1: field_label_char="T"; 2: field_label_char="R"; 3: field_label_char="L"; default: field_label_char="="; endcase
                4'd15: case (n) 0: field_label_char="S"; 1: field_label_char="E"; 2: field_label_char="R"; 3: field_label_char="R"; default: field_label_char="="; endcase
                default: case (n) 0: field_label_char="G"; 1: field_label_char="Z"; default: field_label_char="="; endcase
            endcase
        end
    endfunction

    assign label_length = field_label_length(field);
    assign current_label_char = field_label_char(field, label_index);

    assign field_signed = (field >= 4'd7 && field <= 4'd12);
    assign field_value =
        (field == 4'd0) ? {24'd0, snap_id} :
        (field == 4'd1) ? {31'd0, snap_ready} :
        (field == 4'd2) ? {24'd0, snap_error} :
        (field == 4'd3) ? snap_sequence :
        (field == 4'd4) ? {31'd0, snap_int1} :
        (field == 4'd5) ? {31'd0, snap_int2} :
        (field == 4'd6) ? {24'd0, snap_init_status} :
        (field == 4'd7) ? {{16{snap_ax[15]}}, snap_ax} :
        (field == 4'd8) ? {{16{snap_ay[15]}}, snap_ay} :
        (field == 4'd9) ? {{16{snap_az[15]}}, snap_az} :
        (field == 4'd10) ? {{16{snap_gx[15]}}, snap_gx} :
        (field == 4'd11) ? {{16{snap_gy[15]}}, snap_gy} :
        (field == 4'd13) ? {24'd0, snap_power} :
        (field == 4'd14) ? {24'd0, snap_ctrl} :
        (field == 4'd15) ? {24'd0, snap_sensor_error} :
        {{16{snap_gz[15]}}, snap_gz};

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) sender (
        .clk(clk), .rst_n(rst_n), .data(line[line_index]),
        .valid(sender_valid), .ready(sender_ready), .tx(tx), .done(sender_done)
    );

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            interval_count <= 0;
            field <= 0;
            label_index <= 0;
            line_length <= 0;
            line_index <= 0;
            digit_count <= 0;
            magnitude <= 0;
            snap_id <= 0;
            snap_init_status <= 0;
            snap_power <= 0; snap_ctrl <= 0; snap_sensor_error <= 0;
            snap_ready <= 0;
            snap_error <= 0;
            snap_sequence <= 0;
            snap_int1 <= 0;
            snap_int2 <= 0;
            snap_ax <= 0; snap_ay <= 0; snap_az <= 0;
            snap_gx <= 0; snap_gy <= 0; snap_gz <= 0;
            for (i = 0; i < 10; i = i + 1)
                digits[i] <= 0;
        end else begin
            if (interval_count >= REPORT_CYCLES - 1)
                interval_count <= 0;
            else
                interval_count <= interval_count + 1'b1;

            case (state)
                ST_IDLE: begin
                    if ((interval_count >= REPORT_CYCLES - 1) && sender_ready) begin
                        snap_id <= chip_id;
                        snap_init_status <= init_status;
                        snap_power <= power_conf_readback;
                        snap_ctrl <= init_ctrl_readback;
                        snap_sensor_error <= sensor_error;
                        snap_ready <= ready_in;
                        snap_error <= error_code;
                        snap_sequence <= sample_sequence;
                        snap_int1 <= int1;
                        snap_int2 <= int2;
                        snap_ax <= accel_x; snap_ay <= accel_y; snap_az <= accel_z;
                        snap_gx <= gyro_x; snap_gy <= gyro_y; snap_gz <= gyro_z;
                        field <= 0;
                        label_index <= 0;
                        line_length <= 0;
                        line_index <= 0;
                        state <= ST_LABEL;
                    end
                end
                ST_LABEL: begin
                    line[line_length] <= current_label_char;
                    line_length <= line_length + 1'b1;
                    if (label_index == label_length - 1'b1) begin
                        label_index <= 0;
                        digit_count <= 0;
                        magnitude <= field_value[31] ? (~field_value + 1'b1) : field_value;
                        state <= ST_NUMBER;
                    end else begin
                        label_index <= label_index + 1'b1;
                    end
                end
                ST_NUMBER: begin
                    if ((digit_count == 0) && field_signed && field_value[31]) begin
                        line[line_length] <= "-";
                        line_length <= line_length + 1'b1;
                    end
                    digits[digit_count] <= 8'd48 + (magnitude % 10);
                    digit_count <= digit_count + 1'b1;
                    magnitude <= magnitude / 10;
                    if (magnitude < 10)
                        state <= ST_DIGITS;
                end
                ST_DIGITS: begin
                    line[line_length] <= digits[digit_count - 1'b1];
                    line_length <= line_length + 1'b1;
                    digit_count <= digit_count - 1'b1;
                    if (digit_count == 1) begin
                        if (field == 4'd15) begin
                            state <= ST_CR;
                        end else begin
                            line[line_length + 1'b1] <= " ";
                            line_length <= line_length + 9'd2;
                            field <= field + 1'b1;
                            label_index <= 0;
                            state <= ST_LABEL;
                        end
                    end
                end
                ST_CR: begin
                    line[line_length] <= 8'd13;
                    line_length <= line_length + 1'b1;
                    state <= ST_LF;
                end
                ST_LF: begin
                    line[line_length] <= 8'd10;
                    line_length <= line_length + 1'b1;
                    line_index <= 0;
                    state <= ST_SEND;
                end
                ST_SEND: begin
                    if (sender_ready) begin
                        if (line_index == line_length - 1'b1)
                            state <= ST_IDLE;
                        else
                            line_index <= line_index + 1'b1;
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
