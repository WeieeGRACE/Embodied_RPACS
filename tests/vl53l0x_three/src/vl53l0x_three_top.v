`timescale 1ns/1ps
module vl53l0x_three_top #(
 parameter integer CLK_HZ=50000000,I2C_HZ=100000,BAUD=115200,
 parameter integer RESET_CYCLES=CLK_HZ/1000,BOOT_CYCLES=CLK_HZ/50,
 parameter integer POLL_CYCLES=CLK_HZ/100,MAX_POLLS=100,
 parameter integer TIMEOUT_CYCLES=CLK_HZ/100,ROUND_CYCLES=CLK_HZ,
 parameter integer OLED_I2C_HZ=250000,OLED_POWERUP_CYCLES=CLK_HZ/10,
 parameter integer OLED_WAIT_CYCLES=CLK_HZ*7/50
)(input wire clk_50m,reset_n,imu_gnd_reserved,
 input wire tof_front_gpio1,tof_left_gpio1,tof_right_gpio1,
 inout wire tof_front_scl,tof_front_sda,tof_front_xshut_n,
 inout wire tof_left_scl,tof_left_sda,tof_left_xshut_n,
 inout wire tof_right_scl,tof_right_sda,tof_right_xshut_n,
 output wire uart_tx,oled_scl,inout wire oled_sda,
 output wire PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY);
 (* ASYNC_REG="TRUE" *) reg [1:0] reset_pipe;
 always @(posedge clk_50m or negedge reset_n)
  if(!reset_n) reset_pipe<=0; else reset_pipe<={reset_pipe[0],1'b1};
 wire rst_n=reset_pipe[1];
 // Keep motors disabled even if the robot remains wired during this test.
 assign {PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY}=7'b0;
 (* syn_keep=1 *) wire ground_reservation=imu_gnd_reserved;
 wire [2:0] seen,valid,gpio_sync;
 wire [47:0] distances,ambients,signals;
 wire [23:0] errors,models,revisions,pre_periods,final_periods;
 wire [14:0] read_masks; wire [287:0] raw_results;
 wire [47:0] diagnostics_data;
 (* ASYNC_REG="TRUE" *) reg [8:0] pads_meta,pads_sync;
 always @(posedge clk_50m or negedge rst_n) begin
  if(!rst_n) begin pads_meta<=0; pads_sync<=0; end
  else begin
   pads_meta<={tof_right_scl,tof_right_sda,tof_right_xshut_n,
               tof_left_scl,tof_left_sda,tof_left_xshut_n,
               tof_front_scl,tof_front_sda,tof_front_xshut_n};
   pads_sync<=pads_meta;
  end
 end
 wire [11:0] statuses; wire [95:0] sequences;
 wire event_valid; wire [1:0] event_channel;
 tof_three #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ),.RESET_CYCLES(RESET_CYCLES),
  .BOOT_CYCLES(BOOT_CYCLES),.POLL_CYCLES(POLL_CYCLES),.MAX_POLLS(MAX_POLLS),
  .TIMEOUT_CYCLES(TIMEOUT_CYCLES),.ROUND_CYCLES(ROUND_CYCLES)) sensors(
  .clk(clk_50m),.rst_n(rst_n),.gpio1({tof_right_gpio1,tof_left_gpio1,tof_front_gpio1}),
  .scl({tof_right_scl,tof_left_scl,tof_front_scl}),
  .sda({tof_right_sda,tof_left_sda,tof_front_sda}),
  .xshut_n({tof_right_xshut_n,tof_left_xshut_n,tof_front_xshut_n}),
  .gpio_sync(gpio_sync),.seen(seen),.valid(valid),.distances(distances),
  .ambients(ambients),.signals(signals),.errors(errors),.models(models),.revisions(revisions),
  .pre_periods(pre_periods),.final_periods(final_periods),.statuses(statuses),.sequences(sequences),
  .event_valid(event_valid),.event_channel(event_channel),.diagnostics(diagnostics_data),.read_masks(read_masks),.raw_results(raw_results));
 tof_uart #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) diagnostics(
  .clk(clk_50m),.rst_n(rst_n),.event_valid(event_valid),.event_channel(event_channel),
  .valid(valid),.gpio_sync(gpio_sync),.distances(distances),.ambients(ambients),.signals(signals),
  .errors(errors),.models(models),.revisions(revisions),.pre_periods(pre_periods),
  .final_periods(final_periods),.statuses(statuses),.sequences(sequences),.diagnostics(diagnostics_data),
  .bus_levels(pads_sync),.read_masks(read_masks),.raw_results(raw_results),.tx(uart_tx));
 wire [671:0] text; wire frame_done;
 tof_page page(.seen(seen),.valid(valid),.distances(distances),.errors(errors),.statuses(statuses),.text(text));
 tof_oled_text #(.CLK_HZ(CLK_HZ),.I2C_HZ(OLED_I2C_HZ),
  .POWERUP_CYCLES(OLED_POWERUP_CYCLES),.FRAME_WAIT_CYCLES(OLED_WAIT_CYCLES)) display(
  .clk(clk_50m),.rst_n(rst_n),.text(text),.frame_done(frame_done),.scl(oled_scl),.sda(oled_sda));
endmodule
