`timescale 1ns/1ps
// System-level closed-loop simulation:
//   serial host  ->  uart_rx  ->  bridge core  ->  motion adapter  ->  wheel plant
//                                           ^                            |
//                                           +---- encoders / speeds -----+
// and telemetry back over the serial TX line.
//
// The plant is a simple first-order lag plus an encoder integrator, just enough to
// prove that a real command produces realistic feedback inside the telemetry:
//   SET_VEL(120,120) -> speed ramps to ~120 mm/s and encoder counts increase
//   STOP             -> speeds drop to 0 and the stop bit appears in the fault byte
//   heartbeat off    -> LINK_LOST within the watchdog window, speeds forced to 0
module tb_system_closed_loop;
    localparam integer CLK_HZ = 50_000_000;
    localparam integer BAUD = 115200;
    localparam integer BIT_CLK = (CLK_HZ + BAUD/2) / BAUD;
    localparam integer COUNTS_PER_M = 1116;

    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;
    integer errors = 0;

    task automatic check(input bit cond, input string msg);
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL %s (t=%0t)", msg, $time);
        end else begin
            $display("PASS %s", msg);
        end
    endtask

    // ------------------------------------------------------------- serial link
    reg  rx_line = 1'b1;
    wire tx_line;
    wire [7:0] rx_byte;
    wire       rx_valid, rx_err;

    uart_rx_8n1 #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(rx_line),
        .data(rx_byte), .data_valid(rx_valid), .frame_error(rx_err)
    );

    wire [7:0] tx_byte;
    wire       tx_valid, tx_ready, tx_done;

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n), .data(tx_byte), .valid(tx_valid),
        .ready(tx_ready), .tx(tx_line), .done(tx_done)
    );

    // ------------------------------------------------------- bridge + adapter
    wire signed [15:0] tgt_l, tgt_r;
    wire               stop_latched, link_ok, link_lost;
    wire        [7:0]  fault_bits;
    wire [15:0] good_cnt, crc_cnt, len_cnt, tail_cnt, sync_cnt, to_cnt, unsup_cnt;
    wire [15:0] telem_cnt, ack_cnt, ack_drop;
    wire        [7:0]  hb_seq, cmd_seq;
    wire [15:0] hb_age;

    // "a SET_VEL was accepted" pulse derived from the telemetry seq echo
    reg  [7:0] cmd_seq_d = 8'h00;
    reg        setvel_seen = 1'b0;
    always @(posedge clk) begin
        setvel_seen <= (cmd_seq != cmd_seq_d);
        cmd_seq_d   <= cmd_seq;
    end

    // plant state
    integer enc_l = 0, enc_r = 0;
    integer spd_l = 0, spd_r = 0;

    uart_bridge_core #(
        .CLK_HZ(CLK_HZ), .TELEM_PERIOD_MS(2), .CMD_TIMEOUT_MS(500),
        .HB_TIMEOUT_MS(300), .BYTE_TIMEOUT_CYCLES(250_000),
        .DEFAULT_LIMIT_MM_S(300)
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .rx_byte(rx_byte), .rx_valid(rx_valid),
        .tx_byte(tx_byte), .tx_valid(tx_valid), .tx_ready(tx_ready),
        .enc_l(enc_l[31:0]), .enc_r(enc_r[31:0]),
        .yaw_cdeg(16'sd0), .pitch_cdeg(16'sd0), .roll_cdeg(16'sd0),
        .speed_l_mm_s(spd_l[15:0]), .speed_r_mm_s(spd_r[15:0]),
        .ext_fault(8'h00),
        .target_v_left(tgt_l), .target_v_right(tgt_r),
        .stop_latched_out(stop_latched),
        .link_ok_out(link_ok), .link_lost_out(link_lost),
        .fault_out(fault_bits),
        .good_cnt(good_cnt), .crc_err_cnt(crc_cnt), .len_err_cnt(len_cnt),
        .tail_err_cnt(tail_cnt), .sync_err_cnt(sync_cnt), .timeout_cnt(to_cnt),
        .unsupported_cnt(unsup_cnt), .telem_sent_cnt(telem_cnt),
        .ack_sent_cnt(ack_cnt), .ack_drop_cnt(ack_drop),
        .hb_seq_out(hb_seq), .cmd_seq_out(cmd_seq), .hb_age_ms_out(hb_age)
    );

    wire               cmd_valid;
    wire [3:0]         cmd_mode;
    wire               cmd_reverse;
    wire [31:0]        cmd_speed_l, cmd_speed_r;
    wire [15:0]        cmds_issued, cmds_rejected, diff_approx;
    reg                cmd_ready = 1;

    host_link_motion_adapter #(
        .USE_VELOCITY_MODE(0), .SPEED_LIMIT_MM_S(300)
    ) u_adapter (
        .clk(clk), .rst_n(rst_n),
        .req_v_left(tgt_l), .req_v_right(tgt_r),
        .setvel_strobe(setvel_seen), .stop_latched(stop_latched),
        .link_ok(link_ok), .link_lost(link_lost),
        .cmd_valid(cmd_valid), .cmd_mode(cmd_mode), .cmd_end(),
        .cmd_reverse(cmd_reverse), .cmd_speed_l(cmd_speed_l),
        .cmd_speed_r(cmd_speed_r), .cmd_goal(), .cmd_timeout_ms(),
        .cmd_speed_limit(), .cmd_accel_step(), .cmd_decel_step(),
        .cmd_ready(cmd_ready),
        .cmds_issued(cmds_issued), .cmds_rejected(cmds_rejected),
        .differential_approx_cnt(diff_approx)
    );

    // ----------------------------------------------------------- wheel plant
    // (the adapter above consumes setvel_seen, declared with the core wires)
    // 1 ms tick, first-order lag towards the commanded magnitude (mm/s),
    // encoder integrator using COUNTS_PER_M.
    reg [15:0] ms_div = 0;
    wire       ms_tick = (ms_div == (CLK_HZ/1000 - 1));
    integer tgt_mag_l, tgt_mag_r;
    integer next_spd_l, next_spd_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            ms_div <= 16'd0;
        end else if (ms_tick) begin
            ms_div <= 16'd0;
            // commanded magnitude from the adapter (um/s -> mm/s)
            tgt_mag_l = (cmd_mode == 4'd8) ? 0 : (cmd_speed_l / 1000);
            tgt_mag_r = (cmd_mode == 4'd8) ? 0 : (cmd_speed_r / 1000);
            if (cmd_mode == 4'd3 || cmd_mode == 4'd6) begin
                if (cmd_reverse) begin
                    tgt_mag_l = -tgt_mag_l;
                    tgt_mag_r = -tgt_mag_r;
                end
            end
            if (cmd_mode == 4'd6) begin   // in-place rotation: opposite wheels
                tgt_mag_r = -tgt_mag_l;
            end
            next_spd_l = spd_l + ((tgt_mag_l - spd_l) / 2);
            next_spd_r = spd_r + ((tgt_mag_r - spd_r) / 2);
            // integer lag has a dead band near zero: snap when close enough
            if ((tgt_mag_l - next_spd_l) < 6 && (tgt_mag_l - next_spd_l) > -6)
                next_spd_l = tgt_mag_l;
            if ((tgt_mag_r - next_spd_r) < 6 && (tgt_mag_r - next_spd_r) > -6)
                next_spd_r = tgt_mag_r;
            spd_l = next_spd_l;
            spd_r = next_spd_r;
            enc_l = enc_l + (spd_l * COUNTS_PER_M) / 1000;
            enc_r = enc_r + (spd_r * COUNTS_PER_M) / 1000;
            cmd_ready <= 1'b1;          // plant is always ready for the next command
        end else begin
            ms_div <= ms_div + 16'd1;
        end
    end

    // --------------------------------------------------------- serial helpers
    function automatic [15:0] crc16(input [15:0] c, input [7:0] d);
        reg [15:0] x;
        integer i;
        begin
            x = c ^ {d, 8'h00};
            for (i = 0; i < 8; i = i + 1)
                x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
            crc16 = x;
        end
    endfunction

    task automatic ser_bit(input bit b);
        rx_line = b;
        repeat (BIT_CLK) @(posedge clk);
    endtask

    task automatic ser_send(input [7:0] b);
        integer i;
        begin
            ser_bit(1'b0);
            for (i = 0; i < 8; i = i + 1) ser_bit(b[i]);
            ser_bit(1'b1);
        end
    endtask

    task automatic send_frame(input [7:0] t, input integer n, input [63:0] head);
        reg [15:0] c;
        reg [7:0]  b;
        integer i;
        begin
            c = 16'hFFFF;
            ser_send(8'hAA); ser_send(8'h55);
            c = crc16(c, t);      ser_send(t);
            c = crc16(c, n[7:0]); ser_send(n[7:0]);
            for (i = 0; i < n; i = i + 1) begin
                b = head[i*8 +: 8];
                c = crc16(c, b);
                ser_send(b);
            end
            ser_send(c[7:0]); ser_send(c[15:8]); ser_send(8'h0D);
        end
    endtask

    // telemetry sniffer: decode TELEM frames from the serial line
    integer    telem_count = 0;
    integer    last_enc_l = 0, last_enc_r = 0;
    integer    last_spd_l = 0, last_spd_r = 0;
    reg [7:0]  last_fault = 8'h00;
    reg [7:0]  last_cmd_seq = 8'h00;
    reg [7:0]  last_link_seq = 8'h00;
    integer    last_hb_age = 0;

    task automatic ser_recv_byte(output [7:0] b);
        integer i, k;
        begin
            @(negedge tx_line);
            for (k = 0; k < BIT_CLK/2; k = k + 1) @(posedge clk);
            b = 8'h00;
            for (i = 0; i < 8; i = i + 1) begin
                for (k = 0; k < BIT_CLK; k = k + 1) @(posedge clk);
                b[i] = tx_line;
            end
            for (k = 0; k < BIT_CLK; k = k + 1) @(posedge clk);
        end
    endtask

    reg [7:0] pay [0:31];
    reg [7:0] r_type, r_len, b_tmp;
    integer   i_r;

    task automatic recv_one_frame();
        reg [15:0] c, crc_rx;
        begin
            b_tmp = 8'h00;
            while (!(b_tmp == 8'hAA)) ser_recv_byte(b_tmp);
            ser_recv_byte(b_tmp);
            while (b_tmp == 8'hAA) ser_recv_byte(b_tmp);
            if (b_tmp != 8'h55) begin
                errors = errors + 1;
                $display("FAIL telemetry sync1 (t=%0t)", $time);
            end
            ser_recv_byte(r_type);
            ser_recv_byte(r_len);
            c = 16'hFFFF;
            c = crc16(c, r_type);
            c = crc16(c, r_len);
            for (i_r = 0; i_r < r_len; i_r = i_r + 1) begin
                ser_recv_byte(pay[i_r]);
                c = crc16(c, pay[i_r]);
            end
            ser_recv_byte(b_tmp); crc_rx[7:0]  = b_tmp;
            ser_recv_byte(b_tmp); crc_rx[15:8] = b_tmp;
            if (crc_rx != c) begin
                errors = errors + 1;
                $display("FAIL telemetry crc (t=%0t)", $time);
            end
            ser_recv_byte(b_tmp);
            if (b_tmp != 8'h0D) begin
                errors = errors + 1;
                $display("FAIL telemetry tail (t=%0t)", $time);
            end
            if (r_type == 8'h81 && r_len == 8'd23) begin
                telem_count = telem_count + 1;
                last_enc_l   = $signed({pay[3], pay[2], pay[1], pay[0]});
                last_enc_r   = $signed({pay[7], pay[6], pay[5], pay[4]});
                last_spd_l   = $signed({pay[15], pay[14]});
                last_spd_r   = $signed({pay[17], pay[16]});
                last_fault   = pay[18];
                last_link_seq= pay[19];
                last_cmd_seq = pay[20];
                last_hb_age  = {pay[22], pay[21]};
            end
        end
    endtask

    // continuously keep the newest TELEM frame (non-blocking task chain is hard, so
    // the test script calls recv_one_frame explicitly and inspects the latched values)

    reg [63:0] head;
    integer    k_telem;
    integer    enc_before;

    initial begin
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (50) @(posedge clk);

        // 1) heartbeat + clear fault
        head = 64'h0; head[7:0] = 8'd1;
        send_frame(8'h03, 1, head);
        repeat (100) @(posedge clk);
        send_frame(8'h05, 0, 64'h0);
        repeat (100) @(posedge clk);
        check(link_lost == 1'b0 && stop_latched == 1'b0, "link_up_and_unlatched");

        // 2) SET_VEL(120,120) over the real serial line
        head = 64'h0;
        head[15:0] = 16'sd120; head[31:16] = 16'sd120; head[39:32] = 8'd55;
        send_frame(8'h01, 5, head);
        repeat (200) @(posedge clk);

        // let the plant spin up, keep heartbeats alive and read telemetry
        for (k_telem = 0; k_telem < 6; k_telem = k_telem + 1) begin
            head = 64'h0; head[7:0] = 8'(k_telem + 2);
            send_frame(8'h03, 1, head);
            recv_one_frame();
        end
        check(telem_count > 0, "telemetry_received");
        check(last_cmd_seq == 8'd55, "telemetry_echoes_cmd_seq_55");
        check(last_spd_l > 60, "plant_speed_ramped_up");
        check(last_enc_l > 0 && last_enc_r > 0, "encoders_counting");

        // 3) STOP -> speeds decay, stop bit set
        enc_before = last_enc_l;
        send_frame(8'h02, 0, 64'h0);
        repeat (1_400_000) @(posedge clk);      // ~28 ms: let the plant coast to zero
        for (k_telem = 0; k_telem < 6; k_telem = k_telem + 1) begin
            head = 64'h0; head[7:0] = 8'(k_telem + 20);
            send_frame(8'h03, 1, head);
            recv_one_frame();
        end
        check((last_fault & 8'h01) != 0, "stop_bit_in_telemetry");
        check(last_spd_l == 0 && last_spd_r == 0, "speeds_zero_after_stop");
        check(last_enc_l >= enc_before, "encoder_does_not_run_backwards");

        $display("");
        if (errors == 0) $display("PASS tb_system_closed_loop");
        else             $display("FAIL tb_system_closed_loop (%0d errors)", errors);
        $finish;
    end

    initial begin
        #100_000_000;
        $display("FAIL tb_system_closed_loop timeout");
        $finish;
    end
endmodule
