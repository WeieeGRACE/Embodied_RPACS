`timescale 1ns/1ps
// Diagnostic port of GY-VL53L0X / ARDUINO / VL53L0X.ino.
// No undocumented initialization or calibration is implied by sample_valid.
module tof_driver #(
 parameter integer CLK_HZ=50000000,I2C_HZ=100000,
 parameter integer RESET_CYCLES=CLK_HZ/1000,BOOT_CYCLES=CLK_HZ/50,
 parameter integer POLL_CYCLES=CLK_HZ/100,MAX_POLLS=100,
 parameter integer TIMEOUT_CYCLES=CLK_HZ/100
)(input wire clk,rst_n,trigger,recover_req,input wire gpio1,
 inout wire scl,sda,xshut_n,output wire gpio_sync,
 output reg busy,done,sample_valid,output reg [7:0] error,
 output reg [15:0] distance_mm,ambient,signal_count,
 output reg [7:0] model_id,revision,pre_period,final_period,
 output reg [3:0] range_status,output reg [15:0] diagnostic,
 output reg [4:0] read_mask,output reg [95:0] raw_result);
 localparam RESET=0,BOOT=1,IDLE=2,ISSUE=3,WAIT_IO=4,POLL_WAIT=5;
 reg [2:0] state; reg [31:0] timer; reg [15:0] polls;
 reg [2:0] phase; reg pull_reset,recover,poll_timeout,io_start;
 reg io_write; reg [7:0] io_addr; reg [3:0] io_len;
 wire [15:0] io_diagnostic;
 wire io_busy,io_done; wire [7:0] io_error; wire [95:0] bytes;
 (* ASYNC_REG="TRUE" *) reg [1:0] irq;
 assign gpio_sync=irq[1];
 assign xshut_n=pull_reset?1'b0:1'bz;
 always @(posedge clk or negedge rst_n)
  if(!rst_n) irq<=0; else irq<={irq[0],gpio1};
 always @* begin
  io_write=0; io_len=1; io_addr=0;
  case(phase)
   0: io_addr=8'hc2;
   1: io_addr=8'hc0;
   2: io_addr=8'h50;
   3: io_addr=8'h70;
   4: begin io_addr=0; io_write=1; end
   5: io_addr=8'h14;
   6: begin io_addr=8'h14; io_len=12; end
   default: io_addr=0;
  endcase
 end
 tof_i2c_reg #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ),.TIMEOUT_CYCLES(TIMEOUT_CYCLES)) bus(
  .clk(clk),.rst_n(rst_n),.start(io_start),.write_en(io_write),
  .reg_addr(io_addr),.write_data(8'h01),.read_len(io_len),
  .busy(io_busy),.done(io_done),.error(io_error),.rx_data(bytes),.scl(scl),.sda(sda),.diagnostic(io_diagnostic));
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=RESET; timer<=0; polls<=0; phase<=0; pull_reset<=1; recover<=0;
   poll_timeout<=0; io_start<=0; busy<=1; done<=0; sample_valid<=0;
   error<=0; diagnostic<=0; read_mask<=0; raw_result<=0; distance_mm<=0; ambient<=0; signal_count<=0;
   model_id<=0; revision<=0; pre_period<=0; final_period<=0; range_status<=0;
  end else begin
   done<=0; io_start<=0;
   case(state)
    RESET: if(timer>=RESET_CYCLES-1) begin
     timer<=0; pull_reset<=0; state<=BOOT;
    end else timer<=timer+1'b1;
    BOOT: if(timer>=BOOT_CYCLES-1) begin
     timer<=0;
     if(recover) begin recover<=0; state<=ISSUE; end
     else begin busy<=0; state<=IDLE; end
    end else timer<=timer+1'b1;
    IDLE: if(trigger) begin
     busy<=1; phase<=0; polls<=0; poll_timeout<=0; read_mask<=0; raw_result<=0;
     revision<=0; model_id<=0; pre_period<=0; final_period<=0; diagnostic<=0;
     if(error!=0 || recover_req) begin recover<=1; pull_reset<=1; timer<=0; state<=RESET; end
     else state<=ISSUE;
    end
    ISSUE: begin io_start<=1; state<=WAIT_IO; end
    WAIT_IO: if(io_done) begin
     if(io_error!=0) begin
      error<=io_error; diagnostic<=io_diagnostic; sample_valid<=0; busy<=0; done<=1; state<=IDLE;
     end else case(phase)
      0,1,2,3: begin
       read_mask[phase]<=1;
       case(phase)
        0: revision<=bytes[7:0]; 1: model_id<=bytes[7:0];
        2: pre_period<=bytes[7:0]; 3: final_period<=bytes[7:0];
       endcase
       phase<=phase+1'b1; state<=ISSUE;
      end
      4: begin phase<=5; timer<=0; state<=POLL_WAIT; end
      5: begin
       polls<=polls+1'b1;
       if(bytes[0] || polls>=MAX_POLLS-1) begin
        poll_timeout<=!bytes[0]; phase<=6; state<=ISSUE;
       end else begin timer<=0; state<=POLL_WAIT; end
      end
      6: begin
       read_mask[4]<=1; raw_result<=bytes;
       distance_mm<={bytes[87:80],bytes[95:88]};
       ambient<={bytes[55:48],bytes[63:56]};
       signal_count<={bytes[71:64],bytes[79:72]};
       range_status<=bytes[6:3]; error<=poll_timeout?8'd2:8'd0;
       sample_valid<=!poll_timeout; busy<=0; done<=1; state<=IDLE;
      end
      default: state<=IDLE;
     endcase
    end
    POLL_WAIT: if(timer>=POLL_CYCLES-1) begin timer<=0; state<=ISSUE; end
     else timer<=timer+1'b1;
    default: state<=RESET;
   endcase
  end
 end
endmodule

module tof_three #(
 parameter integer CLK_HZ=50000000,I2C_HZ=100000,
 parameter integer RESET_CYCLES=CLK_HZ/1000,BOOT_CYCLES=CLK_HZ/50,
 parameter integer POLL_CYCLES=CLK_HZ/100,MAX_POLLS=100,
 parameter integer TIMEOUT_CYCLES=CLK_HZ/100,ROUND_CYCLES=CLK_HZ
)(input wire clk,rst_n,input wire [2:0] gpio1,
 inout wire [2:0] scl,sda,xshut_n,output wire [2:0] gpio_sync,
 output reg [2:0] seen,valid,output reg [47:0] distances,ambients,signals,
 output reg [23:0] errors,models,revisions,pre_periods,final_periods,
 output reg [11:0] statuses,output reg [95:0] sequences,
 output reg event_valid,output reg [1:0] event_channel,output reg [47:0] diagnostics,
 output reg [14:0] read_masks,output reg [287:0] raw_results);
 wire [2:0] busy,done,sample_valid; reg [2:0] trigger;
 wire [47:0] raw_distance,raw_ambient,raw_signal;
 wire [23:0] raw_error,raw_model,raw_revision,raw_pre,raw_final;
 wire [14:0] channel_masks; wire [287:0] channel_results;
 wire [11:0] raw_status; wire [47:0] raw_diagnostic;
 genvar g;
 generate for(g=0;g<3;g=g+1) begin: channels
  tof_driver #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ),.RESET_CYCLES(RESET_CYCLES),
   .BOOT_CYCLES(BOOT_CYCLES),.POLL_CYCLES(POLL_CYCLES),.MAX_POLLS(MAX_POLLS),
   .TIMEOUT_CYCLES(TIMEOUT_CYCLES)) driver(
   .clk(clk),.rst_n(rst_n),.trigger(trigger[g]),.recover_req(1'b0),.gpio1(gpio1[g]),
   .scl(scl[g]),.sda(sda[g]),.xshut_n(xshut_n[g]),.gpio_sync(gpio_sync[g]),
   .busy(busy[g]),.done(done[g]),.sample_valid(sample_valid[g]),
   .error(raw_error[g*8+:8]),.distance_mm(raw_distance[g*16+:16]),
   .ambient(raw_ambient[g*16+:16]),.signal_count(raw_signal[g*16+:16]),
   .model_id(raw_model[g*8+:8]),.revision(raw_revision[g*8+:8]),
   .pre_period(raw_pre[g*8+:8]),.final_period(raw_final[g*8+:8]),.range_status(raw_status[g*4+:4]),.diagnostic(raw_diagnostic[g*16+:16]),
   .read_mask(channel_masks[g*5+:5]),.raw_result(channel_results[g*96+:96]));
 end endgenerate
 reg [1:0] selected; reg [1:0] state; reg [31:0] timer;
 integer i;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   selected<=0; state<=0; timer<=0; trigger<=0; event_valid<=0; event_channel<=0;
   seen<=0; valid<=0; distances<=0; ambients<=0; signals<=0; errors<=0;
   diagnostics<=0; read_masks<=0; raw_results<=0; models<=0; revisions<=0; pre_periods<=0; final_periods<=0; statuses<=0; sequences<=0;
  end else begin
   trigger<=0; event_valid<=0;
   for(i=0;i<3;i=i+1) if(done[i]) begin
    diagnostics[i*16+:16]<=raw_diagnostic[i*16+:16];
    read_masks[i*5+:5]<=channel_masks[i*5+:5]; raw_results[i*96+:96]<=channel_results[i*96+:96];
    seen[i]<=1; valid[i]<=sample_valid[i]; distances[i*16+:16]<=raw_distance[i*16+:16];
    ambients[i*16+:16]<=raw_ambient[i*16+:16]; signals[i*16+:16]<=raw_signal[i*16+:16];
    errors[i*8+:8]<=raw_error[i*8+:8]; models[i*8+:8]<=raw_model[i*8+:8];
    revisions[i*8+:8]<=raw_revision[i*8+:8]; pre_periods[i*8+:8]<=raw_pre[i*8+:8];
    final_periods[i*8+:8]<=raw_final[i*8+:8]; statuses[i*4+:4]<=raw_status[i*4+:4];
    sequences[i*32+:32]<=sequences[i*32+:32]+1'b1;
    event_valid<=1; event_channel<=i;
   end
   case(state)
    0: if(!busy[selected]) begin trigger[selected]<=1; state<=1; end
    1: if(done[selected]) begin
     if(selected==2) begin timer<=0; state<=2; end
     else begin selected<=selected+1'b1; state<=0; end
    end
    2: if(timer>=ROUND_CYCLES-1) begin selected<=0; state<=0; timer<=0; end
     else timer<=timer+1'b1;
   endcase
  end
 end
endmodule
