`timescale 1ns/1ps
// End-to-end loopback testbench: real UART timing (115200 8N1) on both directions.
//
// TB drives the FPGA rx line bit by bit, receives the FPGA tx line bit by bit and
// verifies the ACK / TELEM frames including CRC. This is the closest simulation to
// the board test done through COM14.
module tb_loopback;
    localparam integer CLK_HZ  = 50_000_000;
    localparam integer BAUD    = 115200;
    localparam integer BIT_CLK = (CLK_HZ + BAUD/2) / BAUD;   // 434 clocks per bit

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

    // ------------------------------------------------------------- DUT chain
    reg  rx_line = 1'b1;
    wire tx_line;

    wire [7:0] rx_byte;
    wire       rx_valid, rx_err;

    uart_rx_8n1 #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(rx_line),
        .data(rx_byte), .data_valid(rx_valid), .frame_error(rx_err)
    );

    wire [7:0] tx_byte;
    wire       tx_valid;
    wire       tx_ready;
    wire       tx_done;

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n), .data(tx_byte), .valid(tx_valid),
        .ready(tx_ready), .tx(tx_line), .done(tx_done)
    );

    wire signed [15:0] target_l, target_r;
    wire        stop_out, link_ok_out, link_lost_out;
    wire [7:0]  fault_out;
    wire [15:0] good_c, crc_c, len_c, tail_c, sync_c, to_c, unsup_c;
    wire [15:0] telem_cnt, ack_cnt, ack_drop;
    wire [7:0]  hb_seq, cmd_seq;
    wire [15:0] hb_age;

    uart_bridge_core #(
        .CLK_HZ(CLK_HZ),
        .TELEM_PERIOD_MS(20),
        .CMD_TIMEOUT_MS(500),
        .HB_TIMEOUT_MS(200),
        .BYTE_TIMEOUT_CYCLES(250_000),
        .DEFAULT_LIMIT_MM_S(300)
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .rx_byte(rx_byte), .rx_valid(rx_valid),
        .tx_byte(tx_byte), .tx_valid(tx_valid), .tx_ready(tx_ready),
        .enc_l(32'sd1000), .enc_r(-32'sd2000),
        .yaw_cdeg(-16'sd4500), .pitch_cdeg(16'sd125), .roll_cdeg(-16'sd50),
        .speed_l_mm_s(16'sd123), .speed_r_mm_s(-16'sd321),
        .ext_fault(8'h00),
        .target_v_left(target_l), .target_v_right(target_r),
        .stop_latched_out(stop_out), .link_ok_out(link_ok_out),
        .link_lost_out(link_lost_out), .fault_out(fault_out),
        .good_cnt(good_c), .crc_err_cnt(crc_c), .len_err_cnt(len_c),
        .tail_err_cnt(tail_c), .sync_err_cnt(sync_c), .timeout_cnt(to_c),
        .unsupported_cnt(unsup_c), .telem_sent_cnt(telem_cnt),
        .ack_sent_cnt(ack_cnt), .ack_drop_cnt(ack_drop),
        .hb_seq_out(hb_seq), .cmd_seq_out(cmd_seq), .hb_age_ms_out(hb_age)
    );

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
            ser_bit(1'b0);                   // start
            for (i = 0; i < 8; i = i + 1) ser_bit(b[i]);
            ser_bit(1'b1);                   // stop
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

    // receive one byte from the DUT tx line (mid-bit sampling)
    task automatic ser_recv_byte(output [7:0] b);
        integer i, k;
        begin
            @(negedge tx_line);                       // start bit falling edge
            for (k = 0; k < BIT_CLK/2; k = k + 1) @(posedge clk);
            b = 8'h00;
            for (i = 0; i < 8; i = i + 1) begin
                for (k = 0; k < BIT_CLK; k = k + 1) @(posedge clk);
                b[i] = tx_line;
            end
            for (k = 0; k < BIT_CLK; k = k + 1) @(posedge clk);   // stop bit
        end
    endtask

    // receive a whole frame: returns type, len, payload bytes into pay[0..31]
    reg [7:0] pay [0:31];
    reg [7:0] r_type, r_len;
    integer   r_i, r_k;
    task automatic recv_frame(output [7:0] ft, output [7:0] fl);
        reg [15:0] c, crc_rx;
        reg [7:0]  b;
        begin
            // hunt for AA 55
            b = 8'h00;
            while (!(b == 8'hAA)) ser_recv_byte(b);
            ser_recv_byte(b);
            while (b == 8'hAA) ser_recv_byte(b);       // tolerate AA AA
            check(b == 8'h55, "loop_sync1_is_55");
            ser_recv_byte(ft);
            ser_recv_byte(fl);
            c = 16'hFFFF;
            c = crc16(c, ft);
            c = crc16(c, fl);
            for (r_i = 0; r_i < fl; r_i = r_i + 1) begin
                ser_recv_byte(pay[r_i]);
                c = crc16(c, pay[r_i]);
            end
            ser_recv_byte(b); crc_rx[7:0]  = b;
            ser_recv_byte(b); crc_rx[15:8] = b;
            check(crc_rx == c, "loop_crc_ok");
            ser_recv_byte(b);
            check(b == 8'h0D, "loop_tail_0d");
        end
    endtask

    reg [7:0] ft, fl;
    reg [63:0] head;
    integer    found_ack, found_telem;
    reg [7:0]  ack_status, ack_seq, ack_acked;
    integer    guard;

    // capture buffers for the ACK/TELEM window
    integer    n_cap, k, m;
    reg [7:0]  cap_type [0:2];
    reg [7:0]  cap_len  [0:2];
    reg [7:0]  cap_pay  [0:2][0:31];
    reg [7:0]  telem_len_c, telem_yaw_l, telem_yaw_h;
    reg [7:0]  telem_link_seq, telem_cmd_seq;
    reg [7:0]  telem_encr [0:3];

    // ------------------------------------------------------------------ script
    initial begin
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (50) @(posedge clk);

        // 1) heartbeat over the real serial line
        head = 64'h0; head[7:0] = 8'd11;
        fork
            send_frame(8'h03, 1, head);
        join
        repeat (2000) @(posedge clk);
        check(link_ok_out == 1'b1 && hb_seq == 8'd11, "loop_heartbeat_links_up");

        // 2) CLEAR_FAULT (power-up LINK_LOST must be cleared explicitly)
        send_frame(8'h05, 0, 64'h0);
        repeat (2000) @(posedge clk);
        check(link_lost_out == 1'b0, "loop_clear_fault_releases_link_lost");

        // 3) SET_VEL: start the listener BEFORE sending so the ACK cannot be missed
        head = 64'h0;
        head[15:0] = 16'sd200; head[31:16] = -16'sd150; head[39:32] = 8'd42;
        n_cap = 0;
        fork
            begin : cap_proc
                integer m, b_i;
                for (m = 0; m < 3; m = m + 1) begin
                    recv_frame(cap_type[m], cap_len[m]);
                    for (b_i = 0; b_i < 23; b_i = b_i + 1)
                        cap_pay[m][b_i] = pay[b_i];
                    n_cap = n_cap + 1;
                end
            end
            send_frame(8'h01, 5, head);
        join

        check(target_l == 16'sd200 && target_r == -16'sd150, "loop_setvel_applied");

        found_ack = 0; found_telem = 0;
        for (k = 0; k < n_cap; k = k + 1) begin
            if (cap_type[k] == 8'h82) begin
                found_ack  = 1;
                ack_status = cap_pay[k][2];
                ack_seq    = cap_pay[k][1];
                ack_acked  = cap_pay[k][0];
            end
            if (cap_type[k] == 8'h81) begin
                found_telem = 1;
                telem_len_c = cap_len[k];
                for (m = 0; m < 4; m = m + 1) telem_encr[m] = cap_pay[k][4+m];
                telem_yaw_l = cap_pay[k][8];
                telem_yaw_h = cap_pay[k][9];
                telem_link_seq = cap_pay[k][19];
                telem_cmd_seq  = cap_pay[k][20];
            end
        end
        check(found_ack == 1, "loop_ack_received");
        if (found_ack) begin
            check(ack_status == 8'h00, "loop_ack_status_ok");
            check(ack_seq == 8'd42, "loop_ack_seq_echo");
            check(ack_acked == 8'h01, "loop_ack_acked_type");
        end

        // 4) verify the telemetry frame captured in the same window
        check(found_telem == 1, "loop_telem_received");
        if (found_telem) begin
            check(telem_len_c == 8'd23, "loop_telem_len_23");
            check(telem_encr[0] == 8'h30 && telem_encr[1] == 8'hF8 &&
                  telem_encr[2] == 8'hFF && telem_encr[3] == 8'hFF,
                  "loop_telem_enc_r_is_minus_2000");
            check(telem_yaw_l == 8'h6C && telem_yaw_h == 8'hEE, "loop_telem_yaw_minus_45deg");
            check(telem_link_seq == 8'd11, "loop_telem_link_seq_is_heartbeat_seq");
            check(telem_cmd_seq == 8'd42, "loop_telem_cmd_seq_is_setvel_seq");
        end

        // 5) STOP over serial -> ACK ok
        send_frame(8'h02, 0, 64'h0);
        repeat (2000) @(posedge clk);
        check(stop_out == 1'b1, "loop_stop_latched");

        $display("");
        if (errors == 0) $display("PASS tb_loopback");
        else             $display("FAIL tb_loopback (%0d errors)", errors);
        $finish;
    end

    initial begin
        #200_000_000;                          // 200 ms sim time guard
        $display("FAIL tb_loopback timeout");
        $finish;
    end
endmodule
