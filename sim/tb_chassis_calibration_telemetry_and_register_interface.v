`timescale 1ns/1ps
module tb_chassis_calibration_telemetry_and_register_interface;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, req=0, write=0;
    reg [11:0] addr=0;
    reg [31:0] wdata=0;
    reg [3:0] wstrb=0;
    wire ack,error;
    wire [31:0] rdata;
    reg signed [31:0] speed_l=0,speed_r=0,position_l=0,position_r=0;
    reg signed [15:0] pwm_l=0,pwm_r=0;
    reg [31:0] illegal_count_l=0,illegal_count_r=0,status_in=0,fault_in=0;
    reg running_l=0,running_r=0;
    reg [31:0] voltage_mv=0;
    reg signed [31:0] temperature_mc=0;
    reg voltage_valid=0,temperature_valid=0;
    wire control_enable,config_updated,target_updated,soft_reset_req,fault_clear_req;
    wire [2:0] mode;
    wire signed [31:0] target_l,target_r;
    wire [31:0] fault_clear_mask,fault_latched;
    wire [15:0] pwm_limit,pwm_min;
    wire [31:0] wheelbase_mm;
    reg completed=0;
    integer checks=0, j,k,seed=32'h134579ab;
    reg [31:0] got, expected, data_random, mask_random;
    reg [3:0] strobe_random;
    reg [31:0] model [0:20];
    reg [31:0] old_sequence, old_missed;
    wire default_enable;
    wire [15:0] default_pwm_limit;
    wire [31:0] default_counts;
    chassis_calibration_telemetry_and_register_interface #(
        .CLK_HZ(1000),.PWM_PERIOD_CYCLES(2500),.RUNTIME_WIDTH(4)
    ) dut (
        .clk(clk),.rst_n(rst_n),.req(req),.write(write),.addr(addr),.wdata(wdata),.wstrb(wstrb),
        .ack(ack),.error(error),.rdata(rdata),
        .speed_l(speed_l),.speed_r(speed_r),.position_l(position_l),.position_r(position_r),
        .pwm_l(pwm_l),.pwm_r(pwm_r),.illegal_count_l(illegal_count_l),.illegal_count_r(illegal_count_r),
        .status_in(status_in),.fault_in(fault_in),.running_l(running_l),.running_r(running_r),
        .voltage_mv(voltage_mv),.temperature_mc(temperature_mc),
        .voltage_valid(voltage_valid),.temperature_valid(temperature_valid),
        .control_enable(control_enable),.mode(mode),.target_l(target_l),.target_r(target_r),
        .config_updated(config_updated),.target_updated(target_updated),.soft_reset_req(soft_reset_req),
        .fault_clear_req(fault_clear_req),.fault_clear_mask(fault_clear_mask),.fault_latched(fault_latched),
        .pwm_limit(pwm_limit),.pwm_min(pwm_min),.wheelbase_mm(wheelbase_mm),
        .direction_cfg(),
        .counts_per_rev_l(),
        .counts_per_rev_r(),
        .wheel_diameter_l_mm(),
        .wheel_diameter_r_mm(),
        .max_speed_counts(),
        .kp_l(),
        .ki_l(),
        .kd_l(),
        .kp_r(),
        .ki_r(),
        .kd_r(),
        .gain_shift(),
        .ramp_step(),
        .stall_pwm_threshold(),
        .stall_speed_threshold(),
        .stall_timeout_ms(),
        .integral_limit()
    );
    chassis_calibration_telemetry_and_register_interface defaults (
        .clk(clk),.rst_n(rst_n),.req(1'b0),.write(1'b0),.addr(12'd0),.wdata(32'd0),.wstrb(4'd0),
        .speed_l(32'd0),.speed_r(32'd0),.position_l(32'd0),.position_r(32'd0),
        .pwm_l(16'd0),.pwm_r(16'd0),.illegal_count_l(32'd0),.illegal_count_r(32'd0),
        .status_in(32'd0),.fault_in(32'd0),.running_l(1'b0),.running_r(1'b0),
        .voltage_mv(32'd0),.temperature_mc(32'd0),.voltage_valid(1'b0),.temperature_valid(1'b0),
        .control_enable(default_enable),.pwm_limit(default_pwm_limit),.counts_per_rev_l(default_counts),
        .ack(),
        .error(),
        .rdata(),
        .mode(),
        .target_l(),
        .target_r(),
        .config_updated(),
        .target_updated(),
        .soft_reset_req(),
        .fault_clear_req(),
        .fault_clear_mask(),
        .fault_latched(),
        .direction_cfg(),
        .counts_per_rev_r(),
        .wheel_diameter_l_mm(),
        .wheel_diameter_r_mm(),
        .wheelbase_mm(),
        .max_speed_counts(),
        .kp_l(),
        .ki_l(),
        .kd_l(),
        .kp_r(),
        .ki_r(),
        .kd_r(),
        .gain_shift(),
        .pwm_min(),
        .ramp_step(),
        .stall_pwm_threshold(),
        .stall_speed_threshold(),
        .stall_timeout_ms(),
        .integral_limit()
    );
    task check;
        input condition;
        input [8*120-1:0] description;
        begin
            checks=checks+1;
            if (condition !== 1'b1) begin
                $display("FAIL %0d at %0t: %0s",checks,$time,description);
                $fatal(1);
            end
        end
    endtask
    task bus;
        input wr;
        input [11:0] a;
        input [31:0] d;
        input [3:0] s;
        input fail_expected;
        begin
            @(negedge clk); req=1;write=wr;addr=a;wdata=d;wstrb=s;
            @(posedge clk); #1;
            check(ack === 1 && error === fail_expected,"transaction ack/error");
            got=rdata;
            @(negedge clk); req=0;write=0;wstrb=0;
        end
    endtask
    task wr;
        input [11:0] a;
        input [31:0] d;
        begin bus(1,a,d,15,0); end
    endtask
    task rd;
        input [11:0] a;
        input [31:0] value;
        begin bus(0,a,0,0,0); check(got===value,"read value"); end
    endtask
    task reset_dut;
        begin
            @(negedge clk); rst_n=0;req=0;fault_in=0;running_l=0;running_r=0;
            #1; check(control_enable===0 && target_l===0 && ack===0,"asynchronous reset");
            repeat(2) @(negedge clk);
            rst_n=1;
        end
    endtask
    initial begin
        if ($test$plusargs("inject_failure")) begin
            $display("EXPECTED FAILURE: runner exit-code self-test"); $fatal(1);
        end
        reset_dut;
        check(default_enable===0 && default_pwm_limit===1000 && default_counts===1456,"default parameter instance");
        rd(12'h000,32'h00010000); rd(12'h004,0); rd(12'h030,0);
        rd(12'h104,1456);rd(12'h10c,66);rd(12'h114,240);rd(12'h138,1000);
        rd(12'h180,0);rd(12'h1b8,1000);
        bus(0,12'h001,0,0,1);bus(1,12'h005,1,15,1);
        bus(0,12'hffc,0,0,1);bus(1,12'h03c,1,15,1);
        bus(1,12'h000,32'hffffffff,15,1);bus(1,12'h200,1,15,1);
        bus(1,12'h180,1,15,1);bus(1,12'h304,1,15,1);
        rd(12'h004,0);
        // Byte lanes and zero strobe.
        wr(12'h008,32'h12345678);
        bus(1,12'h008,32'habcdef01,4'b0101,0);rd(12'h008,32'h12cd5601);
        bus(1,12'h008,32'hffffffff,0,0);rd(12'h008,32'h12cd5601);
        check(target_l===0 && target_r===0,"shadow targets do not leak");
        wr(12'h008,100);wr(12'h00c,-200);wr(12'h010,1);
        check(target_l===100 && target_r===-200 && target_updated,"atomic signed target commit");
        @(posedge clk); #1;check(!target_updated && !ack,"pulse and ack deassert on idle");
        wr(12'h008,32'h80000000);bus(1,12'h010,1,15,1);
        check(target_l===100 && target_r===-200,"reject most negative target without overflow");
        wr(12'h008,1001);bus(1,12'h010,1,15,1);
        wr(12'h008,1000);wr(12'h00c,-1000);wr(12'h010,1);
        check(target_l===1000 && target_r===-1000,"target range endpoints");
        wr(12'h008,0);wr(12'h00c,0);wr(12'h010,1);
        // Configuration atomicity, signed coefficients and validation.
        wr(12'h114,333);wr(12'h11c,32'h80000000);
        check(wheelbase_mm===240,"shadow config hidden");wr(12'h01c,1);
        check(config_updated && wheelbase_mm===333,"config atomically committed");rd(12'h19c,32'h80000000);
        @(posedge clk);#1;check(!config_updated,"config pulse one cycle");
        wr(12'h004,7);check(control_enable && mode===3,"enable and mode");
        wr(12'h114,444);bus(1,12'h01c,1,15,1);check(wheelbase_mm===333,"enabled commit rejected");
        wr(12'h004,0);wr(12'h104,0);bus(1,12'h01c,1,15,1);wr(12'h104,1456);
        wr(12'h138,2501);bus(1,12'h01c,1,15,1);wr(12'h138,1000);
        wr(12'h13c,1001);bus(1,12'h01c,1,15,1);wr(12'h13c,100);
        wr(12'h134,32);bus(1,12'h01c,1,15,1);wr(12'h134,31);
        wr(12'h140,0);bus(1,12'h01c,1,15,1);wr(12'h140,65536);bus(1,12'h01c,1,15,1);wr(12'h140,65535);
        wr(12'h100,16);bus(1,12'h01c,1,15,1);wr(12'h100,15);
        wr(12'h118,32'h80000000);bus(1,12'h01c,1,15,1);wr(12'h118,1000);
        wr(12'h148,32'hffffffff);bus(1,12'h01c,1,15,1);wr(12'h148,1);
        wr(12'h14c,0);bus(1,12'h01c,1,15,1);wr(12'h14c,500);
        wr(12'h150,32'h80000000);bus(1,12'h01c,1,15,1);wr(12'h150,0);
        wr(12'h144,1001);bus(1,12'h01c,1,15,1);wr(12'h144,500);
        wr(12'h01c,1);check(wheelbase_mm===444 && pwm_min===100,"valid boundary config commit");
        wr(12'h008,1000);wr(12'h010,1);wr(12'h118,999);bus(1,12'h01c,1,15,1);
        wr(12'h008,0);wr(12'h010,1);wr(12'h01c,1);
        // Manual telemetry capture and sign extension, validity, hold.
        speed_l=-123;speed_r=456;position_l=32'h80000000;position_r=32'h7fffffff;
        pwm_l=-32768;pwm_r=32767;illegal_count_l=17;illegal_count_r=19;
        status_in=32'hdeadcafe;voltage_mv=12001;temperature_mc=-250;
        voltage_valid=1;temperature_valid=0;
        wr(12'h014,1);rd(12'h030,1);rd(12'h034,1);
        rd(12'h200,-123);rd(12'h204,456);rd(12'h208,32'h80000000);rd(12'h20c,32'h7fffffff);
        rd(12'h210,32'hffff8000);rd(12'h214,32767);rd(12'h218,17);rd(12'h21c,19);
        rd(12'h220,32'hdeadcafe);rd(12'h228,12001);rd(12'h22c,-250);rd(12'h230,1);
        speed_l=999;rd(12'h200,-123);
        wr(12'h018,1);wr(12'h014,1);rd(12'h200,-123);rd(12'h308,1);rd(12'h034,1);
        wr(12'h018,0);wr(12'h014,1);rd(12'h200,999);rd(12'h034,2);
        // Period=1 captures each cycle, hold applies on its write edge.
        wr(12'h024,1);wr(12'h018,1);
        old_sequence=dut.snapshot_sequence;old_missed=dut.missed_samples;
        repeat(5) @(posedge clk);#1;
        check(dut.snapshot_sequence===old_sequence && dut.missed_samples===old_missed+5,"periodic hold missed count");
        @(negedge clk);req=1;write=1;addr=12'h018;wdata=0;wstrb=1;
        @(posedge clk);#1;check(dut.snapshot_sequence===old_sequence+1,"release and due same edge captures");
        @(negedge clk);req=0;wr(12'h024,0);
        old_sequence=dut.snapshot_sequence;repeat(4) @(posedge clk);#1;
        check(dut.snapshot_sequence===old_sequence,"period zero disables");
        // Force only to reach impractically distant 32-bit saturation boundaries.
        wr(12'h018,1);
        @(negedge clk);dut.missed_samples=32'hfffffffe;
        wr(12'h014,1);wr(12'h014,1);rd(12'h308,32'hffffffff);
        // Fault set dominates W1C; re-enable is always explicit.
        wr(12'h004,1);
        @(negedge clk);fault_in=5;
        @(posedge clk);#1;check(!control_enable && fault_latched===5,"live fault disables");
        bus(1,12'h004,1,15,1);
        wr(12'h300,5);check(fault_clear_req && fault_clear_mask===5 && fault_latched===5,"W1C cannot remove live fault");
        @(posedge clk);#1;check(!fault_clear_req && fault_clear_mask===0,"fault clear pulse");
        @(negedge clk);fault_in=0;
        wr(12'h300,1);rd(12'h300,4);bus(1,12'h004,1,15,1);
        wr(12'h300,4);rd(12'h300,0);check(!control_enable,"clear never reenables");
        wr(12'h004,1);check(control_enable,"explicit reenable");
        @(negedge clk);fault_in=32'h100;req=1;write=1;addr=12'h004;wdata=1;wstrb=1;
        @(posedge clk);#1;check(!control_enable && error && fault_latched===32'h100,"same-edge enable fault priority");
        @(negedge clk);req=0;fault_in=0;
        bus(1,12'h300,32'h100,4'b0001,0);rd(12'h300,32'h100);
        bus(1,12'h300,32'h100,4'b0010,0);rd(12'h300,0);
        wr(12'h020,1);check(soft_reset_req,"soft reset request");
        @(posedge clk);#1;check(!soft_reset_req,"soft reset one cycle");
        bus(1,12'h020,1,0,0);check(!soft_reset_req,"zero strobe command no-op");
        // Runtime saturation using legal small-width parameter.
        running_l=1;running_r=0;repeat(20) @(posedge clk);#1;
        rd(12'h30c,15);rd(12'h310,0);
        running_r=1;repeat(20) @(posedge clk);#1;rd(12'h310,15);rd(12'h30c,15);
        // Reset during an enabled transaction clears all state.
        wr(12'h004,1);reset_dut;rd(12'h004,0);rd(12'h034,0);rd(12'h308,0);rd(12'h30c,0);
        rd(12'h114,240);rd(12'h008,0);
        // Timestamp and runtime tick use exact CLK_HZ/1000 boundaries.
        @(negedge clk);dut.ms_count=0;
        old_sequence=dut.timestamp_ms;
        repeat(3) @(posedge clk);#1;
        check(dut.timestamp_ms===old_sequence+3,"1 ms tick exact for CLK_HZ=1000");
        // N=3 periodic boundary; simultaneous manual+periodic generates one snapshot.
        @(negedge clk);req=1;write=1;addr=12'h024;wdata=3;wstrb=15;
        @(posedge clk);#1;old_sequence=dut.snapshot_sequence;
        @(negedge clk);req=0;
        repeat(2) @(posedge clk);#1;
        check(dut.snapshot_sequence===old_sequence,"period N waits N-1 cycles");
        @(negedge clk);req=1;write=1;addr=12'h014;wdata=1;wstrb=1;
        @(posedge clk);#1;check(dut.snapshot_sequence===old_sequence+1,"manual and periodic coalesce");
        check(dut.snapshot[13]===dut.timestamp_ms-1,"snapshot timestamp is edge input value");
        @(negedge clk);req=0;wr(12'h024,0);
        for(j=1;j<=5;j=j+1) begin
            bus(0,12'h100+4*j,0,0,0);expected=got;
            wr(12'h100+4*j,0);bus(1,12'h01c,1,15,1);
            wr(12'h100+4*j,expected);
        end
        wr(12'h118,0);bus(1,12'h01c,1,15,1);wr(12'h118,1000);
        wr(12'h138,0);wr(12'h13c,0);wr(12'h144,0);wr(12'h01c,1);
        check(pwm_limit===0 && pwm_min===0,"zero PWM limit accepted");
        reset_dut;
        // Independent randomized byte-write software model, fixed seed.
        for(j=0;j<21;j=j+1) begin
            bus(0,12'h100+4*j,0,0,0);model[j]=got;
        end
        for(j=0;j<600;j=j+1) begin
            k=($random(seed)&32'h7fffffff)%21;
            data_random=$random(seed);strobe_random=$random(seed);
            mask_random={{8{strobe_random[3]}},{8{strobe_random[2]}},{8{strobe_random[1]}},{8{strobe_random[0]}}};
            bus(1,12'h100+4*k,data_random,strobe_random,0);
            model[k]=(model[k]&~mask_random)|(data_random&mask_random);
            rd(12'h100+4*k,model[k]);
        end
        // Back-to-back transactions without lowering req: read sees prior write.
        @(negedge clk);req=1;write=1;addr=12'h008;wdata=32'h13579bdf;wstrb=15;
        @(posedge clk);#1;check(ack && !error,"burst write ack");
        @(negedge clk);write=0;
        @(posedge clk);#1;check(ack && !error && rdata===32'h13579bdf,"burst read after write");
        @(negedge clk);write=1;addr=12'h00c;wdata=32'h2468ace0;
        @(posedge clk);#1;check(ack && !error,"burst second write");
        @(negedge clk);write=0;
        @(posedge clk);#1;check(ack && !error && rdata===32'h2468ace0,"burst second read");
        @(negedge clk);req=0;
        @(posedge clk);#1;check(!ack && !error && rdata===0,"idle response");
        completed=1;
        $display("PASS: chassis register interface %0d checks, seed=0x134579ab",checks);
        $finish;
    end
    initial begin #1000000; $display("FAIL: test timeout");$fatal(1);end
endmodule
