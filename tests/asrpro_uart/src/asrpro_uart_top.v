`timescale 1ns/1ps
module asrpro_uart_top #(parameter integer CLK_HZ=50000000,BAUD=115200,
 parameter integer KEY_CYCLES=1000000,FRAME_TIMEOUT_CYCLES=1000000)(
 input wire clk_50m,reset_n,asr_uart_rx,uart_rx,key_step_n,key_fault_n,
 output wire asr_uart_tx,uart_tx,led_ready,led_busy,led_error,led_rx,
 output wire PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,
 input wire imu_gnd_reserved,imu_spi_sck,imu_spi_mosi,imu_spi_miso,imu_cs_n,imu_int1,imu_int2,
 input wire tof_front_gpio1,tof_left_gpio1,tof_right_gpio1,
 inout wire tof_front_scl,tof_front_sda,tof_front_xshut_n,
 inout wire tof_left_scl,tof_left_sda,tof_left_xshut_n,
 inout wire tof_right_scl,tof_right_sda,tof_right_xshut_n);
 (* ASYNC_REG="TRUE" *) reg [1:0] reset_pipe;
 always @(posedge clk_50m or negedge reset_n)
  if(!reset_n) reset_pipe<=0;else reset_pipe<={reset_pipe[0],1'b1};
 wire rst_n=reset_pipe[1];reg [31:0] tick;
 always @(posedge clk_50m or negedge rst_n) if(!rst_n) tick<=0;else tick<=tick+1'b1;
 assign {PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY}=7'b0;
 assign {tof_front_scl,tof_front_sda,tof_front_xshut_n,
         tof_left_scl,tof_left_sda,tof_left_xshut_n,
         tof_right_scl,tof_right_sda,tof_right_xshut_n}=9'bz;
 wire [7:0] rx_data,debug_data,command,inject_status,status,response,last_command;
 wire rx_valid,rx_error,debug_valid,debug_error,command_valid,inject_valid;
 wire [31:0] rx_tick,debug_tick,good,bad,timeouts,debug_parse_errors,debug_control_errors;
 wire [31:0] overflows,log_overflows;wire [1:0] gear;
 wire step,fault,event_valid,event_pair;
 asr_uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) receiver(
  .clk(clk_50m),.rst_n(rst_n),.rx(asr_uart_rx),.tick(tick),.data(rx_data),
  .valid(rx_valid),.frame_error(rx_error),.start_tick(rx_tick));
 asr_frame_rx #(.TIMEOUT_CYCLES(FRAME_TIMEOUT_CYCLES)) parser(
  .clk(clk_50m),.rst_n(rst_n),.tick(tick),.byte_start_tick(rx_tick),.data(rx_data),
  .valid(rx_valid),.frame_error(rx_error),.command_valid(command_valid),.command(command),
  .good_count(good),.bad_count(bad),.timeout_count(timeouts));
 asr_uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) debug_receiver(
  .clk(clk_50m),.rst_n(rst_n),.rx(uart_rx),.tick(tick),.data(debug_data),
  .valid(debug_valid),.frame_error(debug_error),.start_tick(debug_tick));
 asr_debug_input #(.TIMEOUT_CYCLES(CLK_HZ/10)) debug_parser(
  .clk(clk_50m),.rst_n(rst_n),.tick(tick),.data(debug_data),.valid(debug_valid),
  .frame_error(debug_error),.inject_valid(inject_valid),.inject_status(inject_status),
  .error_count(debug_parse_errors));
 asr_key #(.CYCLES(KEY_CYCLES)) step_key(.clk(clk_50m),.rst_n(rst_n),.key_n(key_step_n),.press(step));
 asr_key #(.CYCLES(KEY_CYCLES)) fault_key(.clk(clk_50m),.rst_n(rst_n),.key_n(key_fault_n),.press(fault));
 asr_test_control control(.clk(clk_50m),.rst_n(rst_n),.command_valid(command_valid),.command(command),
  .step(step),.fault(fault),.inject_valid(inject_valid),.inject_status(inject_status),
  .status(status),.gear(gear),.event_valid(event_valid),.event_pair(event_pair),
  .response(response),.last_command(last_command),.debug_error_count(debug_control_errors));
 asr_feedback #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) feedback(
  .clk(clk_50m),.rst_n(rst_n),.event_valid(event_valid),.event_pair(event_pair),
  .response(response),.status(status),.tx(asr_uart_tx),.overflow_count(overflows));
 reg [31:0] rx_hold,report_timer;
 wire [31:0] all_bad=bad; // parser already counts each UART framing error once
 wire [31:0] all_debug=debug_parse_errors+debug_control_errors;
 wire report=(report_timer==CLK_HZ-1);
 always @(posedge clk_50m or negedge rst_n) begin
  if(!rst_n) begin rx_hold<=0;report_timer<=0;end
  else begin
   if(command_valid) rx_hold<=CLK_HZ/10;else if(rx_hold!=0) rx_hold<=rx_hold-1'b1;
   if(report) report_timer<=0;else report_timer<=report_timer+1'b1;
  end
 end
 asr_diagnostics #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) diagnostics(
  .clk(clk_50m),.rst_n(rst_n),.event_valid(event_valid || report),
  .command(last_command),.response(event_valid && event_pair?response:8'hFF),
  .status(status),.gear(gear),.good(good),.bad(all_bad),.timeouts(timeouts),
  .overflows(overflows),.debug_errors(all_debug),.tx(uart_tx),.log_overflows(log_overflows));
 assign led_ready=rst_n && status!=6;
 assign led_busy=(status==1 || status==2 || status==8'h10);
 assign led_error=(status==6 || all_bad!=0 || timeouts!=0 || overflows!=0 || all_debug!=0 || log_overflows!=0);
 assign led_rx=(rx_hold!=0);
endmodule
