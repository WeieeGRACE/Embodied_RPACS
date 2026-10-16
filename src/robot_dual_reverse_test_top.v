`timescale 1ns/1ps
// Separate entry point preserves the forward bench project and wiring.
module robot_dual_reverse_test_top(
 input wire clk_50m,reset_n,key_enable_n,key_estop_n,
 input wire encoder_a_l,encoder_b_l,encoder_a_r,encoder_b_r,
 output wire PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,uart_tx,
 output wire led_enable,led_estop,led_fault,led_saturated);
 robot_closed_loop_test_top #(.TARGET(-16'sd10),.KP(16'sd32),.KI(16'sd1),.KD(16'sd0),.TEST_BOTH(1)) bench(
 .uart_rx(1'b1),.clk_50m(clk_50m),.reset_n(reset_n),.key_enable_n(key_enable_n),.key_estop_n(key_estop_n),
 .encoder_a_l(encoder_a_l),.encoder_b_l(encoder_b_l),.encoder_a_r(encoder_a_r),.encoder_b_r(encoder_b_r),
 .PWMA(PWMA),.AIN1(AIN1),.AIN2(AIN2),.PWMB(PWMB),.BIN1(BIN1),.BIN2(BIN2),.STBY(STBY),.uart_tx(uart_tx),
 .led_enable(led_enable),.led_estop(led_estop),.led_fault(led_fault),.led_saturated(led_saturated));
endmodule



