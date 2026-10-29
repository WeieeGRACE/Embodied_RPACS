`timescale 1ns/1ps
// Three autonomous buses. Lane zero=front, one=left, two=right.
// A channel only counts as failed after FAIL_MS of continuous bad samples, and
// distance age is reported as `degraded` long before it would latch a fault.
// That keeps transient bus/EMI glitches recoverable (stop, wait, resume) while
// a permanently dead sensor still latches a fault for the operator.
//
// Exact CLIP_MM (the VL53L0X minimum-range clamp) is never used as a geometry
// value, but two consecutive clips prove the channel still finishes
// measurements. They are therefore treated as a fresh post-turn shot for the
// route barrier and reported on `reused`, so a noisy channel cannot
// stall a 90-degree detour forever.
module obstacle_sensors #(
 parameter integer CLK_HZ=50000000,I2C_HZ=100000,RESET_CYCLES=CLK_HZ/1000,
 BOOT_CYCLES=CLK_HZ/50,POLL_CYCLES=CLK_HZ/100,MAX_POLLS=10,
 TIMEOUT_CYCLES=CLK_HZ/100,STALE_MS=250,NEAR_MM=85,SIDE_NEAR_MM=85,
 CLIP_MM=20,CLIP_CONFIRM=2,
 FAIL_MS=2000,AGE_FAULT_MS=1500,HOLD_MS=120
)(input wire clk,rst_n,refresh,monitor_enable,input wire [15:0] front_near_mm,input wire [2:0] gpio1,
 inout wire [2:0] scl,sda,xshut_n,
 output wire [47:0] distances,ages,raw_distances,comm_ages,
 output wire [2:0] seen,valid,post_turn,event_mask,raw_event_mask,reused,
 output wire [2:0] degraded,
 output wire [23:0] errors,output wire [11:0] statuses,
 output wire [14:0] fault_bits,output wire fault);
 genvar g;
 generate for(g=0;g<3;g=g+1)begin: channels
  wire busy,done,ok;wire [15:0] mm;wire [7:0] err,id;wire [4:0] mask;
  reg trigger,inflight,current_shot;
  reg [1:0] clip_count;
  reg have,good,confirmed,fresh;
  reg [15:0] accepted,raw,age,comm_age;
  reg [7:0] error;reg [3:0] status;
  reg [31:0] tick_count;
  reg [15:0] bad_ms;
  reg [4:0] faults;
  reg accepted_event,raw_event;
  wire ms=tick_count==CLK_HZ/1000-1;
  wire [3:0] driver_status;
  wire complete_ok=ok && id==8'hee && mask==5'h1f && err==0;
  wire accept_now=done && complete_ok && mm!=CLIP_MM;
  wire communicate_now=done && complete_ok;
  wire clip_now=done && complete_ok && mm==CLIP_MM;
  // Only armed operation latches persistent failures. Short glitches stop the
  // route through `degraded` and recover as soon as the channel is healthy.
  wire failed_now=!good;
  wire bad_latch=bad_ms>=FAIL_MS-1;
  assign distances[g*16+:16]=accepted;assign raw_distances[g*16+:16]=raw;
  assign ages[g*16+:16]=age;assign comm_ages[g*16+:16]=comm_age;
  assign seen[g]=have;
  assign reused[g]=confirmed;
  assign valid[g]=good && (age<STALE_MS || confirmed) && comm_age<STALE_MS &&
                  accepted>(g==0 ? front_near_mm:SIDE_NEAR_MM);
  assign post_turn[g]=fresh;
  assign errors[g*8+:8]=error;assign statuses[g*4+:4]=status;
  assign event_mask[g]=accepted_event;assign raw_event_mask[g]=raw_event;
  // Long stale data drops the channel out of trusted navigation immediately;
  // the route stops and retries long before any fault would latch. Confirmed
  // clips keep the channel usable because it still completes measurements.
  assign degraded[g]=monitor_enable && (failed_now || comm_age>=HOLD_MS ||
                    (age>=HOLD_MS && !confirmed));
  // Preserve OA2 bit layout; the former persistent20/near groups are reserved.
  assign fault_bits[g]=faults[0];assign fault_bits[3+g]=faults[1];
  assign fault_bits[6+g]=faults[2];assign fault_bits[9+g]=1'b0;assign fault_bits[12+g]=1'b0;
  tof_driver #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ),.RESET_CYCLES(RESET_CYCLES),
  .BOOT_CYCLES(BOOT_CYCLES),.POLL_CYCLES(POLL_CYCLES),.MAX_POLLS(MAX_POLLS),.TIMEOUT_CYCLES(TIMEOUT_CYCLES)) driver(
  .clk(clk),.rst_n(rst_n),.trigger(trigger),.gpio1(gpio1[g]),.scl(scl[g]),.sda(sda[g]),.xshut_n(xshut_n[g]),
  .gpio_sync(),.busy(busy),.done(done),.sample_valid(ok),.error(err),.distance_mm(mm),.ambient(),.signal_count(),
  .model_id(id),.revision(),.pre_period(),.final_period(),.range_status(driver_status),.diagnostic(),.read_mask(mask),.raw_result());
  always @(posedge clk or negedge rst_n)
  if(!rst_n)begin
  trigger<=0;inflight<=0;current_shot<=0;clip_count<=0;have<=0;good<=0;confirmed<=0;fresh<=0;
  accepted<=0;raw<=0;age<=0;comm_age<=0;error<=0;status<=0;tick_count<=0;
  bad_ms<=0;faults<=0;accepted_event<=0;raw_event<=0;
  end else begin
  trigger<=0;accepted_event<=0;raw_event<=0;
  if(ms)tick_count<=0;else tick_count<=tick_count+1'b1;
  if(ms)begin
  if(age!=65535)age<=age+1'b1;
  if(comm_age!=65535)comm_age<=comm_age+1'b1;
  if(failed_now && bad_ms!=65535)bad_ms<=bad_ms+1'b1;
  else if(!failed_now)bad_ms<=0;
  end
  if(monitor_enable)begin
  if(comm_age>=FAIL_MS && !communicate_now)faults[1]<=1;
  if(age>=AGE_FAULT_MS && !accept_now && !confirmed)faults[2]<=1;
  if(bad_latch)faults[0]<=1;
  end
  if(!inflight && !busy && !refresh)begin trigger<=1;inflight<=1;current_shot<=1;end
  if(done)begin
  inflight<=0;raw_event<=1;raw<=mm;error<=err;status<=driver_status;
  if(!complete_ok)begin good<=0;confirmed<=0;clip_count<=0;if(monitor_enable && bad_latch)faults[0]<=1;end
  else begin
  comm_age<=0;
  if(clip_now)begin
   // Min-range clamp: keep the last accepted distance and only mark the shot
   // as delivered once two clips in a row prove the channel is still ranging.
   if(clip_count!=2'd3)clip_count<=clip_count+1'b1;
   if(clip_count>=CLIP_CONFIRM-1)begin confirmed<=1;if(current_shot)fresh<=1;end
  end else begin
   clip_count<=0;confirmed<=0;accepted<=mm;age<=0;have<=1;good<=1;accepted_event<=1;
   if(current_shot)fresh<=1;
  end
  end
  end
  // Refresh invalidates active shots and wins over same-cycle completion/start.
  if(refresh)begin fresh<=0;current_shot<=0;trigger<=0;end
  end
 end endgenerate
 assign fault=|fault_bits;
endmodule
