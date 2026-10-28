`timescale 1ns/1ps
// Functional testbench for the host link core (byte level, no serial timing).
//
// Covers: CRC standard vectors, frame parser error taxonomy, command decoding,
// safety arbitration (STOP latch / link loss / clear fault), telemetry cadence.
module tb_core;
    localparam integer BAUD_DIV = 1;

    reg clk = 0;
    always #10 clk = ~clk;          // 50 MHz

    reg rst_n = 0;
    integer errors = 0;
    integer checks_run = 0;

    task automatic check(input bit cond, input string msg);
        checks_run = checks_run + 1;
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL %s  (t=%0t)", msg, $time);
        end else begin
            $display("PASS %s", msg);
        end
    endtask

    // ---------------------------------------------------------------- CRC unit
    reg         crc_init = 0, crc_valid = 0;
    reg  [7:0]  crc_din = 0;
    wire [15:0] crc_out;

    crc16_ccitt u_crc (.clk(clk), .rst_n(rst_n), .init(crc_init),
                       .data(crc_din), .valid(crc_valid), .crc(crc_out));

    task automatic crc_feed(input [7:0] b);
        @(negedge clk); crc_din = b; crc_valid = 1;
        @(negedge clk); crc_valid = 0;
    endtask

    task automatic crc_reset();
        @(negedge clk); crc_init = 1;
        @(negedge clk); crc_init = 0;
    endtask

    // ------------------------------------------------------------- framer unit
    reg  [7:0]  fr_byte = 0;
    reg         fr_valid = 0;
    wire        fr_frame_valid;
    wire [7:0]  fr_type, fr_len;
    wire [63:0] fr_head;
    wire [15:0] fr_good, fr_crc, fr_len_err, fr_tail, fr_sync, fr_timeout;

    host_frame_rx #(.MAX_LEN(32), .TIMEOUT_CYCLES(20000), .CNT_W(16)) u_fr (
        .clk(clk), .rst_n(rst_n), .byte_in(fr_byte), .byte_valid(fr_valid),
        .frame_valid(fr_frame_valid), .frame_type(fr_type), .frame_len(fr_len),
        .payload_head(fr_head),
        .good_cnt(fr_good), .crc_err_cnt(fr_crc), .len_err_cnt(fr_len_err),
        .tail_err_cnt(fr_tail), .sync_err_cnt(fr_sync), .timeout_cnt(fr_timeout)
    );

    integer fr_frames = 0;
    always @(posedge clk) if (fr_frame_valid) fr_frames = fr_frames + 1;

    task automatic fr_send(input [7:0] b);
        @(negedge clk); fr_byte = b; fr_valid = 1;
        @(negedge clk); fr_valid = 0;
        repeat (3) @(negedge clk);
    endtask

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

    // build + send one frame through the framer byte input
    task automatic fr_frame(input [7:0] t, input integer n, input [63:0] head,
                            input bit bad_crc = 0, input [7:0] tail = 8'h0D);
        reg [15:0] c;
        reg [7:0]  b;
        integer i;
        begin
            c = 16'hFFFF;
            fr_send(8'hAA); fr_send(8'h55);
            c = crc16(c, t);   fr_send(t);
            c = crc16(c, n[7:0]); fr_send(n[7:0]);
            for (i = 0; i < n; i = i + 1) begin
                b = head[i*8 +: 8];
                c = crc16(c, b);
                fr_send(b);
            end
            if (bad_crc) c = ~c;
            fr_send(c[7:0]); fr_send(c[15:8]); fr_send(tail);
        end
    endtask

    // ------------------------------------------------------------------ core
    reg  [7:0] rx_byte = 0;
    reg        rx_valid = 0;
    wire [7:0] tx_byte;
    wire       tx_valid;
    wire       tx_ready = 1'b1;

    wire signed [15:0] target_l, target_r;
    wire        stop_out, link_ok_out, link_lost_out;
    wire        setvel_pulse, estop_out;
    wire [7:0]  fault_out;
    wire [15:0] good_c, crc_c, len_c, tail_c, sync_c, to_c, unsup_c;
    wire [15:0] telem_cnt, ack_cnt, ack_drop;
    wire [7:0]  hb_seq, cmd_seq;
    wire [15:0] hb_age;

    reg  [7:0]  ext_fault = 8'h00;

    uart_bridge_core #(
        .CLK_HZ(50_000_000),
        .TELEM_PERIOD_MS(1),
        .CMD_TIMEOUT_MS(1000),
        .HB_TIMEOUT_MS(3),
        .BYTE_TIMEOUT_CYCLES(20000),
        .DEFAULT_LIMIT_MM_S(300)
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .rx_byte(rx_byte), .rx_valid(rx_valid),
        .tx_byte(tx_byte), .tx_valid(tx_valid), .tx_ready(tx_ready),
        .enc_l(32'sd1234), .enc_r(-32'sd5678),
        .yaw_cdeg(-16'sd9000), .pitch_cdeg(16'sd150), .roll_cdeg(-16'sd75),
        .speed_l_mm_s(16'sd180), .speed_r_mm_s(-16'sd160),
        .ext_fault(ext_fault),
        .target_v_left(target_l), .target_v_right(target_r),
        .setvel_strobe_out(setvel_pulse), .estop_latched_out(estop_out),
        .stop_latched_out(stop_out), .link_ok_out(link_ok_out),
        .link_lost_out(link_lost_out), .fault_out(fault_out),
        .good_cnt(good_c), .crc_err_cnt(crc_c), .len_err_cnt(len_c),
        .tail_err_cnt(tail_c), .sync_err_cnt(sync_c), .timeout_cnt(to_c),
        .unsupported_cnt(unsup_c), .telem_sent_cnt(telem_cnt),
        .ack_sent_cnt(ack_cnt), .ack_drop_cnt(ack_drop),
        .hb_seq_out(hb_seq), .cmd_seq_out(cmd_seq), .hb_age_ms_out(hb_age)
    );

    // collect transmitted bytes for inspection
    reg [7:0] tx_hist [0:8191];
    integer   tx_n = 0;
    always @(posedge clk) begin
        if (tx_valid && tx_ready && tx_n < 8192) begin
            tx_hist[tx_n] = tx_byte;
            tx_n = tx_n + 1;
        end
    end

    // count the setvel_strobe pulses exposed by the core
    integer setvel_edges = 0;
    always @(posedge clk) if (rst_n && setvel_pulse) setvel_edges = setvel_edges + 1;

    task automatic send_b(input [7:0] b);
        @(negedge clk); rx_byte = b; rx_valid = 1;
        @(negedge clk); rx_valid = 0;
        repeat (10) @(negedge clk);
    endtask

    task automatic send_frame(input [7:0] t, input integer n, input [63:0] head);
        reg [15:0] c;
        reg [7:0]  b;
        integer i;
        begin
            c = 16'hFFFF;
            send_b(8'hAA); send_b(8'h55);
            c = crc16(c, t);      send_b(t);
            c = crc16(c, n[7:0]); send_b(n[7:0]);
            for (i = 0; i < n; i = i + 1) begin
                b = head[i*8 +: 8];
                c = crc16(c, b);
                send_b(b);
            end
            send_b(c[7:0]); send_b(c[15:8]); send_b(8'h0D);
            repeat (50) @(negedge clk);
        end
    endtask

    // scan tx_hist for the last frame of a given type; returns 0 if none
    reg [7:0] last_ack_status, last_ack_type, last_ack_seq;
    integer   telem_in_hist, ack_in_hist;
    task automatic scan_tx();
        integer j, n;
        reg [7:0] t, l;
        reg [15:0] c, crc_rx;
        begin
            telem_in_hist = 0; ack_in_hist = 0;
            j = 0;
            while (j + 6 < tx_n) begin
                if (tx_hist[j] == 8'hAA && tx_hist[j+1] == 8'h55) begin
                    t = tx_hist[j+2];
                    l = tx_hist[j+3];
                    if (j + 6 + l < tx_n) begin
                        c = 16'hFFFF;
                        c = crc16(c, t);
                        c = crc16(c, l);
                        for (n = 0; n < l; n = n + 1) c = crc16(c, tx_hist[j+4+n]);
                        crc_rx = {tx_hist[j+5+l], tx_hist[j+4+l]};
                        if (crc_rx == c && tx_hist[j+6+l] == 8'h0D) begin
                            if (t == 8'h81) telem_in_hist = telem_in_hist + 1;
                            if (t == 8'h82) begin
                                ack_in_hist = ack_in_hist + 1;
                                last_ack_type   = tx_hist[j+4];
                                last_ack_seq    = tx_hist[j+5];
                                last_ack_status = tx_hist[j+6];
                            end
                        end
                    end
                    j = j + 1;
                end else begin
                    j = j + 1;
                end
            end
        end
    endtask

    reg [63:0] head;
    reg [63:0] head_setvel;
    integer i;
    reg [15:0] unsup_c_before;

    // ------------------------------------------------------------ test script
    initial begin
        repeat (10) @(negedge clk);
        rst_n = 1;
        repeat (10) @(negedge clk);
        // canonical SET_VEL payload used by several later tests
        head_setvel        = 64'h0;
        head_setvel[15:0]  = 16'sd200;
        head_setvel[31:16] = -16'sd150;
        head_setvel[39:32] = 8'd7;

        // ================= CRC vectors =================
        check(crc_out == 16'hFFFF, "crc_v1_reset_value_is_0xFFFF");
        crc_reset();
        for (i = 0; i < 9; i = i + 1) crc_feed(8'h31 + i[7:0]);
        check(crc_out == 16'h29B1, "crc_v2_123456789_is_0x29B1");
        crc_reset();
        crc_feed(8'h01);
        check(crc_out == 16'hF1D1, "crc_v3_byte01_is_0xF1D1");
        crc_reset();
        crc_feed(8'h81); crc_feed(8'h17);
        for (i = 0; i < 23; i = i + 1) crc_feed(8'h00);
        check(crc_out == 16'hD80F, "crc_v4_telem_zeros_is_0xD80F");

        // ================= framer =================
        head = 64'h0;
        head[15:0]  = 16'sd200;
        head[31:16] = -16'sd150;
        head[39:32] = 8'd7;
        fr_frame(8'h01, 5, head);
        repeat (20) @(negedge clk);
        check(fr_frames == 1, "fram_pre_good_frame_decoded");
        check(fr_type == 8'h01 && fr_len == 8'd5, "framer_type_len");
        check(fr_head[15:0] == 16'sd200 && fr_head[31:16] == -16'sd150
              && fr_head[39:32] == 8'd7, "framer_payload_head");

        fr_frame(8'h01, 5, head, 1'b1);        // bad CRC
        repeat (20) @(negedge clk);
        check(fr_crc == 16'd1 && fr_frames == 1, "framer_bad_crc_rejected");

        fr_frame(8'h01, 5, head, 1'b0, 8'h00); // bad tail
        repeat (20) @(negedge clk);
        check(fr_tail == 16'd1 && fr_frames == 1, "framer_bad_tail_rejected");

        fr_send(8'hAA); fr_send(8'h55); fr_send(8'h01); fr_send(8'h21); // LEN=33
        repeat (20) @(negedge clk);
        check(fr_len_err == 16'd1 && fr_frames == 1, "framer_len_over_max_rejected");

        fr_send(8'h12); fr_send(8'h34);
        fr_frame(8'h03, 1, 64'hA5);
        repeat (20) @(negedge clk);
        check(fr_frames == 2, "framer_resync_after_garbage");

        // inter-byte timeout: start a frame then stall longer than TIMEOUT_CYCLES
        fr_send(8'hAA); fr_send(8'h55); fr_send(8'h01);
        repeat (25000) @(negedge clk);
        check(fr_timeout >= 16'd1, "framer_inter_byte_timeout");

        // ================= core: link up =================
        check(link_ok_out == 1'b0 && stop_out == 1'b0, "core_powerup_link_not_ok");
        head = 64'h0; head[7:0] = 8'd3;
        send_frame(8'h03, 1, head);            // HEARTBEAT seq=3
        repeat (20) @(negedge clk);
        check(link_ok_out == 1'b1 && hb_seq == 8'd3, "core_heartbeat_links_up");

        // power-up LINK_LOST is latched: the spec requires an explicit CLEAR_FAULT
        head = 64'h0;
        head[15:0] = 16'sd200; head[31:16] = -16'sd150; head[39:32] = 8'd7;
        tx_n = 0;
        send_frame(8'h01, 5, head);            // SET_VEL is refused before enabling
        repeat (400) @(negedge clk);
        check(link_lost_out == 1'b1 && target_l == 16'sd0,
              "core_setvel_refused_before_clear_fault");
        scan_tx();
        check(ack_in_hist == 1 && last_ack_status == 8'h01,
              "core_setvel_ack_link_lost");

        send_frame(8'h05, 0, 64'h0);           // CLEAR_FAULT
        repeat (200) @(negedge clk);
        check(link_lost_out == 1'b0, "core_clear_fault_clears_latched_link_lost");

        head = 64'h0;
        head[15:0] = 16'sd200; head[31:16] = -16'sd150; head[39:32] = 8'd7;
        tx_n = 0;
        send_frame(8'h01, 5, head);            // SET_VEL
        repeat (400) @(negedge clk);
        check(target_l == 16'sd200 && target_r == -16'sd150, "core_setvel_applied");
        check(cmd_seq == 8'd7, "core_setvel_seq_echo");
        scan_tx();
        check(ack_in_hist == 1 && last_ack_status == 8'h00 && last_ack_seq == 8'd7,
              "core_setvel_ack_ok");

        head = 64'h0;
        head[15:0] = 16'sd500; head[31:16] = 16'sd0; head[39:32] = 8'd8;
        tx_n = 0;
        send_frame(8'h01, 5, head);            // over limit -> clamp + ACK 0x03
        repeat (200) @(negedge clk);
        check(target_l == 16'sd300, "core_setvel_clamped_to_limit");
        scan_tx();
        check(ack_in_hist == 1 && last_ack_status == 8'h03, "core_setvel_ack_over_limit");
        check(fault_out[3] == 1'b1, "core_over_limit_fault_bit");

        // ================= core: STOP latch =================
        tx_n = 0;
        send_frame(8'h02, 0, 64'h0);
        repeat (200) @(negedge clk);
        check(stop_out == 1'b1 && target_l == 16'sd0 && target_r == 16'sd0,
              "core_stop_latched_and_zero_speed");
        scan_tx();
        check(ack_in_hist == 1 && last_ack_status == 8'h00, "core_stop_ack");

        head = 64'h0;
        head[15:0] = 16'sd100; head[31:16] = 16'sd100; head[39:32] = 8'd9;
        tx_n = 0;
        send_frame(8'h01, 5, head);            // rejected while stopped
        repeat (200) @(negedge clk);
        check(target_l == 16'sd0, "core_setvel_rejected_while_stopped");
        scan_tx();
        check(ack_in_hist == 1 && last_ack_status == 8'h02, "core_setvel_ack_stop_latched");

        // ================= core: CLEAR_FAULT =================
        send_frame(8'h03, 1, head);            // heartbeat (head[7:0]=9)
        repeat (50) @(negedge clk);
        send_frame(8'h05, 0, 64'h0);           // CLEAR_FAULT
        repeat (200) @(negedge clk);
        check(stop_out == 1'b0, "core_clear_fault_releases_stop");

        // ================= core: telemetry cadence =================
        repeat (5000) @(negedge clk);          // > 1 ms so at least one TELEM sent
        tx_n = 0;
        repeat (200000) @(negedge clk);        // 4 ms -> expect ~4 TELEM frames
        scan_tx();
        check(telem_in_hist >= 3 && telem_in_hist <= 6, "core_telem_cadence_follows_period");
        check(telem_cnt >= 3, "core_telem_counter");

        // ================= core: heartbeat timeout =================
        // HB_TIMEOUT_MS = 3 ms; stop sending heartbeats
        repeat (200000) @(negedge clk);        // 4 ms
        check(link_ok_out == 1'b0 && link_lost_out == 1'b1, "core_link_lost_after_timeout");
        check(target_l == 16'sd0 && target_r == 16'sd0, "core_link_lost_forces_zero_speed");

        // ================= core: clear fault requires live link =================
        send_frame(8'h05, 0, 64'h0);           // CLEAR_FAULT with dead link
        repeat (200) @(negedge clk);
        check(stop_out == 1'b0, "core_stop_cleared_but_link_still_lost");
        check(link_lost_out == 1'b1, "core_clear_fault_without_link_keeps_link_lost");

        // ================= core: unknown command =================
        send_frame(8'h7E, 0, 64'h0);
        repeat (200) @(negedge clk);
        check(unsup_c == 16'd1, "core_unsupported_type_counted");

        // ================= core: malformed lengths are rejected =================
        // link is still down here, so first restore it
        send_frame(8'h03, 1, head);
        repeat (50000) @(negedge clk);
        send_frame(8'h03, 1, head);
        repeat (200) @(negedge clk);
        check(link_ok_out == 1'b1, "core_link_restored");

        unsup_c_before = unsup_c;
        tx_n = 0;
        send_frame(8'h05, 2, 64'h0);           // CLEAR_FAULT with LEN=2 (illegal)
        repeat (300) @(negedge clk);
        scan_tx();
        check(ack_in_hist == 1 && last_ack_status == 8'h04,
              "core_malformed_clear_fault_ack_unsupported");
        check(unsup_c == unsup_c_before + 16'd1,
              "core_malformed_clear_fault_counted");

        tx_n = 0;
        send_frame(8'h03, 2, 64'h0);           // HEARTBEAT with LEN=2 (illegal)
        repeat (300) @(negedge clk);
        scan_tx();
        check(ack_in_hist == 1 && last_ack_status == 8'h04,
              "core_malformed_heartbeat_ack_unsupported");

        tx_n = 0;
        send_frame(8'h02, 1, 64'h0);           // STOP with LEN=1 (illegal)
        repeat (300) @(negedge clk);
        scan_tx();
        check(stop_out == 1'b1, "core_malformed_stop_still_stops");
        check(ack_in_hist == 1 && last_ack_status == 8'h04,
              "core_malformed_stop_ack_unsupported");

        // ================= core: hardware E-stop is latched =================
        send_frame(8'h05, 0, 64'h0);           // clear the stop first
        repeat (300) @(negedge clk);
        check(stop_out == 1'b0, "core_stop_cleared_before_estop_test");

        send_frame(8'h03, 1, head);
        repeat (200) @(negedge clk);
        send_frame(8'h01, 5, head_setvel);     // a valid speed command
        repeat (200) @(negedge clk);
        check(target_l == 16'sd200, "core_speed_active_before_estop");

        ext_fault = 8'h04;                     // press the E-stop (bit2)
        repeat (50) @(negedge clk);
        check(estop_out == 1'b1, "core_estop_latched");
        check(stop_out == 1'b1, "core_estop_asserts_stop_out");
        check(fault_out[2] == 1'b1, "core_estop_fault_bit");
        check(target_l == 16'sd0 && target_r == 16'sd0,
              "core_estop_forces_zero_speed");

        ext_fault = 8'h00;                     // release the button
        repeat (50) @(negedge clk);
        check(estop_out == 1'b1 && target_l == 16'sd0,
              "core_estop_stays_latched_after_release");

        send_frame(8'h03, 1, head);
        repeat (200) @(negedge clk);
        send_frame(8'h05, 0, 64'h0);           // explicit clear
        repeat (300) @(negedge clk);
        check(estop_out == 1'b0 && stop_out == 1'b0,
              "core_estop_cleared_by_clear_fault");
        check(fault_out[2] == 1'b0, "core_estop_fault_bit_cleared");

        // ================= core: clamp handles the int16 minimum =================
        send_frame(8'h03, 1, head);
        repeat (200) @(negedge clk);
        head_setvel        = 64'h0;
        head_setvel[15:0]  = -16'sd32768;      // -v would overflow int16 here
        head_setvel[31:16] = 16'sd32767;
        head_setvel[39:32] = 8'd9;
        send_frame(8'h01, 5, head_setvel);
        repeat (300) @(negedge clk);
        check(target_l == -16'sd300 && target_r == 16'sd300,
              "core_clamp_handles_int16_min");

        // ================= core: setvel_strobe output =================
        send_frame(8'h03, 1, head);
        repeat (200) @(negedge clk);
        setvel_edges = 0;
        send_frame(8'h01, 5, head_setvel);
        repeat (300) @(negedge clk);
        check(setvel_edges == 1, "core_setvel_strobe_pulses_once");

        $display("");
        if (errors == 0) $display("PASS tb_core (%0d checks)", checks_run);
        else             $display("FAIL tb_core (%0d errors)", errors);
        $finish;
    end

    // watchdog: hard stop (50 ms sim time is plenty for this suite)
    initial begin
        #50_000_000;
        $display("FAIL tb_core timeout");
        $finish;
    end
endmodule
