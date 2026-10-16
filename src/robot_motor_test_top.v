`timescale 1ns/1ps
module robot_motor_test_top(input wire clk_50m, reset_n, output wire PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,output wire led_heartbeat,led_enable,led_stby,led_fault);
reg [1:0] reset_pipe; reg [25:0] heartbeat_count;
always @(posedge clk_50m or negedge reset_n) if(!reset_n) begin reset_pipe<=0; heartbeat_count<=0; end else begin reset_pipe<={reset_pipe[0],1'b1}; heartbeat_count<=heartbeat_count+1'b1; end
wire rst_n_sync=reset_pipe[1]; wire signed [15:0] pwm_command_l=16'sd625; wire signed [15:0] pwm_command_r=16'sd625;
tb6612_driver_interface #(.CLK_HZ(50000000),.PWM_HZ(20000),.DIR_DEADTIME_CYCLES(2500)) driver(.clk(clk_50m),.rst_n(rst_n_sync),.enable(1'b1),.emergency_stop(1'b0),.fault(1'b0),.watchdog_timeout(1'b0),.pwm_signed_l(pwm_command_l),.pwm_signed_r(pwm_command_r),.PWMA(PWMA),.AIN1(AIN1),.AIN2(AIN2),.PWMB(PWMB),.BIN1(BIN1),.BIN2(BIN2),.STBY(STBY));
assign led_heartbeat=heartbeat_count[25]; assign led_enable=rst_n_sync; assign led_stby=STBY; assign led_fault=1'b0;
endmodule
