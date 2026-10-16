`timescale 1ns/1ps
module robot_encoder_filter50_test_top #(parameter integer REPORT_CYCLES=5000000) (
    input wire clk_50m, reset_n, encoder_a_l, encoder_b_l, encoder_a_r, encoder_b_r,
    output wire led_left_dir, led_right_dir, led_left_fault, led_right_fault,
    output wire uart_tx, test_left_fault, test_right_fault
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] reset_pipe;
    always @(posedge clk_50m or negedge reset_n)
        if (!reset_n) reset_pipe <= 0;
        else reset_pipe <= {reset_pipe[0],1'b1};
    wire rst_n = reset_pipe[1];
    wire signed [31:0] position_l, position_r;
    wire signed [1:0] delta_l, delta_r;
    wire direction_l, direction_r;
    wire [31:0] illegal_l, illegal_r;
    wire fault_l, fault_r;
    // Board measurement: chassis-forward gives negative raw left counts.
    dual_ab_quadrature_encoder #(.FILTER_CYCLES(50),.INVERT_L(1),.INVERT_R(0)) encoder (
        .clk(clk_50m), .rst_n(rst_n),
        .encoder_a_l(encoder_a_l), .encoder_b_l(encoder_b_l),
        .encoder_a_r(encoder_a_r), .encoder_b_r(encoder_b_r),
        .clear_fault_l(1'b0), .clear_fault_r(1'b0),
        .position_l(position_l), .position_r(position_r),
        .delta_count_l(delta_l), .delta_count_r(delta_r),
        .direction_l(direction_l), .direction_r(direction_r),
        .illegal_transition_l(illegal_l), .illegal_transition_r(illegal_r),
        .encoder_fault_l(fault_l), .encoder_fault_r(fault_r));
    assign led_left_dir = direction_l;
    assign led_right_dir = direction_r;
    assign led_left_fault = fault_l;
    assign led_right_fault = fault_r;
    encoder_uart_telemetry #(.REPORT_CYCLES(REPORT_CYCLES)) telemetry(
        .clk(clk_50m),.rst_n(rst_n),.position_l(position_l),.position_r(position_r),
        .direction_l(direction_l),.direction_r(direction_r),
        .illegal_l(illegal_l),.illegal_r(illegal_r),.fault_l(fault_l),.fault_r(fault_r),.tx(uart_tx));
    assign test_left_fault = fault_l;
    assign test_right_fault = fault_r;
endmodule


