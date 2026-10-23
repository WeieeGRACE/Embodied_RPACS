`timescale 1ns/1ps

module tb_ov5647_camera;
    localparam integer H_TOTAL = 2200;
    localparam integer V_TOTAL = 1125;
    localparam integer H_ACTIVE = 1920;
    localparam integer V_ACTIVE = 1080;
    localparam integer FRAME_CYCLES = H_TOTAL * V_TOTAL;

    reg clk50 = 0;
    reg byte_clk = 0;
    reg pixel_clk = 0;
    reg ddr_clk = 0;
    reg rst_n = 0;
    reg rx_rst_n = 0;
    reg pipe_rst_n = 0;
    reg video_rst_n = 0;
    reg ddr_rst_n = 0;
    always #10 clk50 = ~clk50;
    always #5 byte_clk = ~byte_clk;
    always #2.5 pixel_clk = ~pixel_clk;
    always #7 ddr_clk = ~ddr_clk;

    integer failures = 0;
    integer tests_complete = 0;

    task automatic check_expect(input logic condition, input string message);
        begin
            if (condition !== 1'b1) begin
                $display("FAIL @%0t: %s", $time, message);
                failures = failures + 1;
            end
        end
    endtask

    // ------------------------------------------------------------------
    // Camera initialization and an open-drain SCCB/I2C ACK + register model.
    tri1 i2c_sda;
    wire i2c_scl;
    wire init_done;
    reg sda_slave_ack = 0;
    reg i2c_active = 0;
    integer i2c_bit = 0;
    integer i2c_byte = 0;
    integer i2c_write_count = 0;
    integer i2c_ack_count = 0;
    integer i2c_errors = 0;
    reg [7:0] i2c_shift = 0;
    reg [7:0] i2c_addr_hi = 0;
    reg [7:0] i2c_addr_lo = 0;
    reg [23:0] expected_lut_word;
    assign i2c_sda = sda_slave_ack ? 1'b0 : 1'bz;

    camera_init #(
        .SYS_CLOCK(50_000_000),
        .SCL_CLOCK(1_000_000),
        .INIT_WAIT_CYCLES(4),
        .INITIAL_WRITE_DELAY_CYCLES(8)
    ) cam_dut (
        .Clk(clk50), .Rst_n(rst_n), .Init_Done(init_done),
        .i2c_sclk(i2c_scl), .i2c_sdat(i2c_sda)
    );

    always @(negedge i2c_sda) begin
        if (i2c_scl === 1'b1) begin
            i2c_active = 1;
            i2c_bit = 0;
            i2c_byte = 0;
            i2c_shift = 0;
        end
    end

    always @(posedge i2c_scl) begin
        if (i2c_active) begin
            if (i2c_bit < 8) begin
                i2c_shift = {i2c_shift[6:0], i2c_sda};
                i2c_bit = i2c_bit + 1;
            end else begin
                i2c_ack_count = i2c_ack_count + 1;
                if (i2c_sda !== 1'b0) i2c_errors = i2c_errors + 1;
                case (i2c_byte)
                    0: if (i2c_shift !== 8'h6c) i2c_errors = i2c_errors + 1;
                    1: i2c_addr_hi = i2c_shift;
                    2: i2c_addr_lo = i2c_shift;
                    3: begin
                        if (i2c_write_count < 58) begin
                            expected_lut_word = cam_dut.ov5647_init_table.rom[i2c_write_count];
                            if ({i2c_addr_hi, i2c_addr_lo, i2c_shift} !== expected_lut_word)
                                i2c_errors = i2c_errors + 1;
                        end else begin
                            i2c_errors = i2c_errors + 1;
                        end
                        i2c_write_count = i2c_write_count + 1;
                    end
                    default: i2c_errors = i2c_errors + 1;
                endcase
                i2c_byte = i2c_byte + 1;
                i2c_shift = 0;
                i2c_bit = 0;
            end
        end
    end

    always @(negedge i2c_scl) begin
        if (i2c_active) begin
            if (i2c_bit == 8) sda_slave_ack = 1;
            else if (i2c_bit == 0) sda_slave_ack = 0;
        end
    end

    always @(posedge i2c_sda) begin
        if (i2c_active && i2c_scl === 1'b1) begin
            if (i2c_byte != 4) i2c_errors = i2c_errors + 1;
            i2c_active = 0;
            sda_slave_ack = 0;
        end
    end

    // ------------------------------------------------------------------
    // CSI-2 byte interface, RAW-8 byte converter and Bayer processing.
    reg csi_ready = 0;
    reg [7:0] csi_data0 = 0;
    reg [7:0] csi_data1 = 0;
    wire csi_sp_en, csi_lp_en, csi_lp_av_en, csi_ecc_ok;
    wire [7:0] csi_ecc;
    wire [15:0] csi_wc;
    wire [1:0] csi_vc;
    wire [5:0] csi_dt;
    wire [15:0] csi_payload;
    wire [1:0] csi_payload_dv;
    wire csi_fv, csi_lv;
    wire [7:0] csi_pixel;
    wire bayer_vs, bayer_de;
    wire [7:0] bayer_r, bayer_g, bayer_b;
    integer csi_short_packets = 0;
    integer csi_long_packets = 0;
    integer csi_ecc_packets = 0;
    integer csi_payload_bytes = 0;
    integer csi_raw_wc8 = 0;
    integer converted_pixels = 0;
    integer bayer_pixels = 0;
    integer bayer_nonzero = 0;
    integer fv_active_samples = 0;
    integer lv_active_samples = 0;
    integer next_expected_pixel = 0;
    reg pixel_scoreboard_enable = 0;

    MIPI_DSI_CSI2_RX_Top rx_dut (
        .I_RSTN(rx_rst_n), .I_BYTE_CLK(byte_clk), .I_REF_DT(6'h2a),
        .I_READY(csi_ready), .I_DATA0(csi_data0), .I_DATA1(csi_data1),
        .O_SP_EN(csi_sp_en), .O_LP_EN(csi_lp_en), .O_LP_AV_EN(csi_lp_av_en),
        .O_ECC_OK(csi_ecc_ok), .O_ECC(csi_ecc), .O_WC(csi_wc),
        .O_VC(csi_vc), .O_DT(csi_dt), .O_PAYLOAD(csi_payload),
        .O_PAYLOAD_DV(csi_payload_dv)
    );

    MIPI_Byte_to_Pixel_Converter_Top converter_dut (
        .I_RSTN(pipe_rst_n), .I_BYTE_CLK(byte_clk), .I_PIXEL_CLK(pixel_clk),
        .I_SP_EN(csi_sp_en), .I_LP_AV_EN(csi_lp_av_en), .I_DT(csi_dt),
        .I_WC(csi_wc), .I_PAYLOAD_DV(csi_payload_dv), .I_PAYLOAD(csi_payload),
        .O_FV(csi_fv), .O_LV(csi_lv), .O_PIXEL(csi_pixel)
    );

    bayer_rgb bayer_dut (
        .rst_n(pipe_rst_n), .pix_clk(pixel_clk), .vs_i(csi_fv),
        .de_i(csi_lv), .data_i(csi_pixel), .vs_o(bayer_vs), .de_o(bayer_de),
        .r_o(bayer_r), .g_o(bayer_g), .b_o(bayer_b)
    );

    // Seed the known line width so this short synthetic frame can exercise the
    // Bayer line buffer without waiting for a previous frame to measure it.
    initial force bayer_dut.video_format_detect_inst.vd_hres_reg = 16'd1920;

    always @(posedge byte_clk) begin
        if (rx_rst_n) begin
            if (csi_sp_en) csi_short_packets = csi_short_packets + 1;
            if (csi_lp_en) begin
                csi_long_packets = csi_long_packets + 1;
                if (csi_dt !== 6'h2a) failures = failures + 1;
                if (csi_dt == 6'h2a && csi_wc == 16'd1920) csi_raw_wc8 = csi_raw_wc8 + 1;
            end
            if (csi_ecc_ok) csi_ecc_packets = csi_ecc_packets + 1;
            csi_payload_bytes = csi_payload_bytes + csi_payload_dv[0] + csi_payload_dv[1];
        end
    end

    always @(posedge pixel_clk) begin
        if (pipe_rst_n) begin
            if (csi_fv === 1'b0) fv_active_samples = fv_active_samples + 1;
            if (csi_lv) begin
                lv_active_samples = lv_active_samples + 1;
                if (pixel_scoreboard_enable && next_expected_pixel < 4*1920) begin
                    if (csi_pixel !== (8'h40 + ((next_expected_pixel / 1920) * 8'h20))) begin
                        $display("CSI pixel mismatch[%0d]: got %02x expected %02x", next_expected_pixel,
                                 csi_pixel, (8'h40 + ((next_expected_pixel / 1920) * 8'h20)));
                        failures = failures + 1;
                    end
                end
                converted_pixels = converted_pixels + 1;
                next_expected_pixel = next_expected_pixel + 1;
            end
            if (bayer_de) begin
                bayer_pixels = bayer_pixels + 1;
                if ((bayer_r | bayer_g | bayer_b) != 0) bayer_nonzero = bayer_nonzero + 1;
            end
        end
    end

    task automatic send_lane_pair(input [7:0] lane0, input [7:0] lane1);
        begin
            @(negedge byte_clk);
            csi_ready = 1;
            csi_data0 = lane0;
            csi_data1 = lane1;
            @(negedge byte_clk);
            csi_ready = 0;
            csi_data0 = 0;
            csi_data1 = 0;
        end
    endtask

    function automatic [7:0] make_ecc(input [23:0] d);
        reg [7:0] e;
        begin
            e[0] = d[0]^d[1]^d[2]^d[4]^d[5]^d[7]^d[10]^d[11]^d[13]^d[16]^d[20]^d[21]^d[22]^d[23];
            e[1] = d[0]^d[1]^d[3]^d[4]^d[6]^d[8]^d[10]^d[12]^d[14]^d[17]^d[20]^d[21]^d[22]^d[23];
            e[2] = d[0]^d[2]^d[3]^d[5]^d[6]^d[9]^d[11]^d[12]^d[15]^d[18]^d[20]^d[21]^d[22];
            e[3] = d[1]^d[2]^d[3]^d[7]^d[8]^d[9]^d[13]^d[14]^d[15]^d[19]^d[20]^d[21]^d[23];
            e[4] = d[4]^d[5]^d[6]^d[7]^d[8]^d[9]^d[16]^d[17]^d[18]^d[19]^d[20]^d[22]^d[23];
            e[5] = d[10]^d[11]^d[12]^d[13]^d[14]^d[15]^d[16]^d[17]^d[18]^d[19]^d[21]^d[22]^d[23];
            e[7:6] = 0;
            make_ecc = e;
        end
    endfunction

    task automatic send_csi_header(input [5:0] dt, input [15:0] wc, input bit corrupt_ecc);
        reg [7:0] ecc_byte;
        reg [7:0] di;
        begin
            di = {2'b00, dt};
            ecc_byte = make_ecc({wc[15:8], wc[7:0], di});
            if (corrupt_ecc) ecc_byte = ecc_byte ^ 8'h01;
            send_lane_pair(di, wc[7:0]);
            send_lane_pair(wc[15:8], ecc_byte);
        end
    endtask

    task automatic send_short_packet(input [5:0] dt);
        begin
            send_csi_header(dt, 16'd0, 0);
        end
    endtask

    task automatic send_pixel_pair(input [7:0] first_pixel, input [7:0] second_pixel);
        begin
            send_lane_pair(first_pixel, second_pixel);
        end
    endtask

    // ------------------------------------------------------------------
    // DDR3 application-port function model checks burst data round trips.
    reg [2:0] app_cmd = 0;
    reg app_cmd_en = 0;
    reg [27:0] app_addr = 0;
    reg app_wr_data_en = 0;
    reg app_wr_data_end = 0;
    reg [127:0] app_wr_data = 0;
    reg [15:0] app_wr_mask = 0;
    wire app_cmd_ready, app_wr_data_rdy;
    wire app_rd_valid, app_rd_end;
    wire [127:0] app_rd_data;
    integer app_write_count, app_read_count, app_error_count;
    ddr3_app_memory_model ddr_model (
        .clk(ddr_clk), .rst_n(ddr_rst_n), .cmd(app_cmd), .cmd_en(app_cmd_en),
        .addr(app_addr), .cmd_ready(app_cmd_ready), .wr_data_en(app_wr_data_en),
        .wr_data_end(app_wr_data_end), .wr_data(app_wr_data),
        .wr_data_mask(app_wr_mask), .wr_data_rdy(app_wr_data_rdy),
        .rd_data_valid(app_rd_valid), .rd_data_end(app_rd_end),
        .rd_data(app_rd_data), .write_count(app_write_count),
        .read_count(app_read_count), .error_count(app_error_count)
    );

    reg memory_test_done = 0;
    reg [127:0] expected_ddr [0:3];
    task automatic issue_ddr_write(input integer index, input [127:0] data);
        begin
            @(negedge ddr_clk);
            app_cmd = 3'b001;
            app_addr = 28'h100 + (index * 16);
            app_cmd_en = 1;
            @(negedge ddr_clk);
            app_cmd_en = 0;
            wait (app_wr_data_rdy === 1'b1);
            app_wr_data = data;
            app_wr_mask = 0;
            app_wr_data_en = 1;
            app_wr_data_end = 1;
            @(negedge ddr_clk);
            app_wr_data_en = 0;
            app_wr_data_end = 0;
        end
    endtask

    task automatic issue_ddr_read(input integer index, input [127:0] expected);
        begin
            @(negedge ddr_clk);
            app_cmd = 3'b000;
            app_addr = 28'h100 + (index * 16);
            app_cmd_en = 1;
            @(negedge ddr_clk);
            app_cmd_en = 0;
            wait (app_rd_valid === 1'b1);
            check_expect(app_rd_end === 1'b1, "DDR read completion marker");
            check_expect(app_rd_data === expected, "DDR application write/read data correspondence");
            @(negedge ddr_clk);
        end
    endtask

    // ------------------------------------------------------------------
    // 1080p timing generator and differential TMDS contract monitor.
    wire video_rden, video_de, video_hs, video_vs;
    reg [23:0] video_rgb = 0;
    always @(posedge pixel_clk) begin
        if (video_de) video_rgb <= video_rgb + 24'h010307;
        else video_rgb <= 0;
    end
    syn_gen sync_dut (
        .I_pxl_clk(pixel_clk), .I_rst_n(video_rst_n),
        .I_h_total(16'd2200), .I_h_sync(16'd44), .I_h_bporch(16'd148),
        .I_h_res(16'd1920), .I_v_total(16'd1125), .I_v_sync(16'd5),
        .I_v_bporch(16'd36), .I_v_res(16'd1080),
        .I_rd_hres(16'd1920), .I_rd_vres(16'd1080),
        .I_hs_pol(1'b1), .I_vs_pol(1'b1), .O_rden(video_rden),
        .O_de(video_de), .O_hs(video_hs), .O_vs(video_vs)
    );

    wire tmds_clk_p, tmds_clk_n;
    wire [2:0] tmds_data_n;
    wire [2:0] tmds_data_p;
    integer tmds_transitions = 0;
    integer video_cycle_count = 0;
    integer video_frame_edges = 0;
    integer frame_pixel_count = 0;
    integer frame_active_line_count = 0;
    integer frame_hsync_count = 0;
    integer current_active_width = 0;
    integer current_hsync_width = 0;
    integer hsync_global_count = 0;
    integer last_hsync_rise = -1;
    integer frame_start_cycle = 0;
    integer previous_tmds_data = -1;
    integer frame_interval_checks = 0;
    reg video_prev_hs = 0;
    reg video_prev_vs = 0;
    reg video_prev_de = 0;
    reg frame_counting = 0;
    reg timing_test_done = 0;
    tmds_link_sim_model tmds_dut (
        .pixel_clk(pixel_clk), .rst_n(video_rst_n), .de(video_de), .rgb(video_rgb),
        .tmds_clk_p(tmds_clk_p), .tmds_clk_n(tmds_clk_n),
        .tmds_data_p(tmds_data_p), .tmds_data_n(tmds_data_n)
    );

    always @(posedge pixel_clk) begin : video_scoreboard
        reg second_frame_edge;
        #0.1;
        if (!video_rst_n) begin
            video_cycle_count = 0;
            video_frame_edges = 0;
            frame_pixel_count = 0;
            frame_active_line_count = 0;
            frame_hsync_count = 0;
            current_active_width = 0;
            current_hsync_width = 0;
            last_hsync_rise = -1;
            frame_start_cycle = 0;
            frame_counting = 0;
            timing_test_done = 0;
            video_prev_hs = 0;
            video_prev_vs = 0;
            video_prev_de = 0;
            previous_tmds_data = -1;
        end else begin
            video_cycle_count = video_cycle_count + 1;
            second_frame_edge = 0;
            if (!video_prev_vs && video_vs) begin
                video_frame_edges = video_frame_edges + 1;
                if (video_frame_edges == 1) begin
                    frame_counting = 1;
                    frame_start_cycle = video_cycle_count;
                    frame_pixel_count = 0;
                    frame_active_line_count = 0;
                    frame_hsync_count = 0;
                end else if (video_frame_edges == 2) begin
                    second_frame_edge = 1;
                    check_expect(video_cycle_count - frame_start_cycle == FRAME_CYCLES,
                           "1080p frame total equals 2200 x 1125 clocks");
                    check_expect(frame_pixel_count == H_ACTIVE * V_ACTIVE,
                           "1080p active picture contains 1920 x 1080 pixels");
                    check_expect(frame_active_line_count == V_ACTIVE,
                           "1080p active picture has 1080 lines");
                    check_expect(frame_hsync_count == V_TOTAL,
                           "1080p frame contains 1125 horizontal sync periods");
                    frame_interval_checks = frame_interval_checks + 1;
                    frame_counting = 0;
                    timing_test_done = 1;
                end
            end

            if (video_hs && !video_prev_hs) begin
                if (last_hsync_rise >= 0)
                    check_expect(video_cycle_count - last_hsync_rise == H_TOTAL,
                           "horizontal total equals 2200 clocks");
                last_hsync_rise = video_cycle_count;
                hsync_global_count = hsync_global_count + 1;
                current_hsync_width = 1;
                if (frame_counting && !second_frame_edge)
                    frame_hsync_count = frame_hsync_count + 1;
            end else if (video_hs) begin
                current_hsync_width = current_hsync_width + 1;
            end else if (video_prev_hs) begin
                check_expect(current_hsync_width == 44, "horizontal sync width equals 44 clocks");
            end

            if (video_de && !video_prev_de) begin
                current_active_width = 1;
                if (frame_counting && !second_frame_edge)
                    frame_active_line_count = frame_active_line_count + 1;
            end else if (video_de) begin
                current_active_width = current_active_width + 1;
            end else if (video_prev_de) begin
                check_expect(current_active_width == H_ACTIVE, "horizontal active width equals 1920 pixels");
            end
            if (video_de && frame_counting && !second_frame_edge)
                frame_pixel_count = frame_pixel_count + 1;

            if (tmds_clk_n !== ~tmds_clk_p) check_expect(1'b0, "TMDS clock differential pair is complementary");
            if (tmds_data_n !== ~tmds_data_p) check_expect(1'b0, "TMDS data differential lanes are complementary");
            if (previous_tmds_data >= 0 && previous_tmds_data != tmds_data_p)
                tmds_transitions = tmds_transitions + 1;
            previous_tmds_data = tmds_data_p;
            video_prev_hs = video_hs;
            video_prev_vs = video_vs;
            video_prev_de = video_de;
        end
    end

    // Independent guards make a missing done pulse or stalled clock fail the batch run.
    initial begin
        #35_000_000;
        if (!timing_test_done) $fatal(1, "TIMEOUT: 1080p timing validation did not finish");
    end

    initial begin : main_test
        integer before_payload;
        integer before_sp;
        integer before_lp;
        integer row;
        integer col;
        reg [7:0] p0;
        reg [7:0] p1;

        #120;
        rst_n = 1;
        rx_rst_n = 1;
        pipe_rst_n = 1;
        video_rst_n = 1;
        ddr_rst_n = 1;

        // Reject a corrupted long-packet ECC without emitting a payload/header.
        before_lp = csi_long_packets;
        before_sp = csi_short_packets;
        send_csi_header(6'h2a, 16'd4, 1);
        repeat (3) @(posedge byte_clk);
        check_expect(rx_dut.bad_ecc_seen == 1, "bad CSI-2 ECC is detected");
        check_expect(csi_long_packets == before_lp && csi_short_packets == before_sp,
               "bad-ECC packet is discarded");

        // Deliver a valid header then truncate the declared four-byte payload.
        before_payload = csi_payload_bytes;
        send_csi_header(6'h2a, 16'd4, 0);
        send_pixel_pair(8'h12, 8'h34);
        repeat (4) @(posedge byte_clk);
        check_expect(csi_payload_bytes - before_payload == 2,
               "truncated RAW-8 packet reports only bytes actually received");
        repeat (10) @(posedge byte_clk);
        check_expect(rx_dut.truncated_packet_count == 1, "truncated CSI-2 payload times out and parser recovers");

        // Start the good frame from a clean receiver/converter state.
        rx_rst_n = 0;
        pipe_rst_n = 0;
        repeat (3) @(posedge byte_clk);
        rx_rst_n = 1;
        pipe_rst_n = 1;
        next_expected_pixel = 0;
        converted_pixels = 0;
        bayer_pixels = 0;
        bayer_nonzero = 0;
        pixel_scoreboard_enable = 1;
        send_short_packet(6'h00); // Frame Start
        for (row = 0; row < 4; row = row + 1) begin
            send_short_packet(6'h02); // Line Start
            send_csi_header(6'h2a, 16'd1920, 0); // RAW-8, 1920 bytes per active line
            for (col = 0; col < 1920; col = col + 2) begin
                p0 = 8'h40 + (row * 8'h20);
                p1 = p0;
                send_pixel_pair(p0, p1);
            end
            send_pixel_pair(8'h5a, 8'ha5); // long-packet CRC bytes, stripped by RX
            send_short_packet(6'h03); // Line End
        end
        send_short_packet(6'h01); // Frame End
        repeat (120) @(posedge pixel_clk);

        check_expect(csi_short_packets == 10, "CSI-2 frame and four line markers are parsed");
        check_expect(csi_long_packets == 5, "truncated and four valid RAW-8 long packets are recognized");
        check_expect(csi_ecc_packets == 15, "valid CSI-2 headers pass ECC checking");
        check_expect(csi_raw_wc8 == 4, "four RAW-8 lines report DT 0x2a and WC 1920");
        check_expect(csi_payload_bytes == 7682, "two truncated bytes plus 7680 RAW-8 pixels are output");
        check_expect(converted_pixels == 7680, "byte-to-pixel converter outputs every valid RAW-8 byte once");
        check_expect(next_expected_pixel == 7680, "pixel payload order matches the synthetic CSI-2 frame");
        check_expect(bayer_pixels >= 7000, "Bayer processing emits active RGB pixels after initial line latency");
        check_expect(bayer_nonzero > 0, "Bayer RGB channels contain nonzero pixel data");

        // Four deterministic DDR3 application bursts are written and read back.
        expected_ddr[0] = 128'h0123456789abcdef_1122334455667788;
        expected_ddr[1] = 128'hfedcba9876543210_8877665544332211;
        expected_ddr[2] = 128'h55aa55aa55aa55aa_1234567890abcdef;
        expected_ddr[3] = 128'h0f1e2d3c4b5a6978_89abcdef01234567;
        for (row = 0; row < 4; row = row + 1)
            issue_ddr_write(row, expected_ddr[row]);
        for (row = 0; row < 4; row = row + 1)
            issue_ddr_read(row, expected_ddr[row]);
        check_expect(app_write_count == 4 && app_read_count == 4,
               "DDR application model completes four writes and four reads");
        check_expect(app_error_count == 0, "DDR application model reports no protocol/data errors");
        memory_test_done = 1;

        wait (init_done === 1'b1);
        check_expect(i2c_write_count == 58, "OV5647 initialization writes all 58 table entries");
        check_expect(i2c_ack_count == 232, "OV5647 register address/data bytes all receive ACK");
        check_expect(i2c_errors == 0, "SCCB address, register order, data and ACK sequence are valid");

        wait (timing_test_done === 1'b1);
        check_expect(frame_interval_checks == 1, "one complete 1080p frame interval is measured");
        check_expect(hsync_global_count >= V_TOTAL, "horizontal sync remains active through a full frame");
        check_expect(tmds_transitions > 100, "TMDS contract model shows active data transitions");

        tests_complete = 1;
        if (failures != 0) $fatal(1, "OV5647 regression failed with %0d checks", failures);
        $display("PASS: OV5647 init=%0d writes/%0d ACKs; CSI short=%0d long=%0d payload=%0d bytes; converted=%0d; Bayer=%0d; DDR=%0d/%0d; timing=%0dx%0d in %0d clocks; TMDS transitions=%0d",
                 i2c_write_count, i2c_ack_count, csi_short_packets, csi_long_packets,
                 csi_payload_bytes, converted_pixels, bayer_pixels, app_write_count,
                 app_read_count, H_ACTIVE, V_ACTIVE, FRAME_CYCLES, tmds_transitions);
        $finish;
    end
endmodule
