`timescale 1ns/1ps

// Protocol-level simulation adapters for the Gowin encrypted IP wrappers in
// the ACG525 example.  ModelSim 10.7 cannot compile the supplied Gowin 2.3
// protected regions, so these models preserve the documented wrapper ports
// and exercise the byte-level contracts.  They are not replacements for
// vendor-IP signoff or electrical simulation.

module MIPI_DSI_CSI2_RX_Top (
    input I_RSTN,
    input I_BYTE_CLK,
    input [5:0] I_REF_DT,
    input I_READY,
    input [7:0] I_DATA0,
    input [7:0] I_DATA1,
    output reg O_SP_EN,
    output reg O_LP_EN,
    output reg O_LP_AV_EN,
    output reg O_ECC_OK,
    output reg [7:0] O_ECC,
    output reg [15:0] O_WC,
    output reg [1:0] O_VC,
    output reg [5:0] O_DT,
    output reg [15:0] O_PAYLOAD,
    output reg [1:0] O_PAYLOAD_DV
);
    reg [1:0] header_phase;
    reg [7:0] data_id;
    reg [7:0] wc_low;
    reg [15:0] payload_left;
    reg [1:0] crc_left;
    reg in_payload;
    integer payload_idle_cycles;
    integer truncated_packet_count;
    integer bad_ecc_seen;

    function automatic [7:0] csi2_header_ecc(input [23:0] d);
        begin
            csi2_header_ecc[0] = d[0]^d[1]^d[2]^d[4]^d[5]^d[7]^d[10]^d[11]^d[13]^d[16]^d[20]^d[21]^d[22]^d[23];
            csi2_header_ecc[1] = d[0]^d[1]^d[3]^d[4]^d[6]^d[8]^d[10]^d[12]^d[14]^d[17]^d[20]^d[21]^d[22]^d[23];
            csi2_header_ecc[2] = d[0]^d[2]^d[3]^d[5]^d[6]^d[9]^d[11]^d[12]^d[15]^d[18]^d[20]^d[21]^d[22];
            csi2_header_ecc[3] = d[1]^d[2]^d[3]^d[7]^d[8]^d[9]^d[13]^d[14]^d[15]^d[19]^d[20]^d[21]^d[23];
            csi2_header_ecc[4] = d[4]^d[5]^d[6]^d[7]^d[8]^d[9]^d[16]^d[17]^d[18]^d[19]^d[20]^d[22]^d[23];
            csi2_header_ecc[5] = d[10]^d[11]^d[12]^d[13]^d[14]^d[15]^d[16]^d[17]^d[18]^d[19]^d[21]^d[22]^d[23];
            csi2_header_ecc[7:6] = 2'b00;
        end
    endfunction

    always @(posedge I_BYTE_CLK or negedge I_RSTN) begin : csi2_parser
        reg [23:0] header_data;
        reg [7:0] received_ecc;
        integer payload_count;
        if (!I_RSTN) begin
            header_phase <= 0;
            data_id <= 0;
            wc_low <= 0;
            payload_left <= 0;
            crc_left <= 0;
            in_payload <= 0;
            payload_idle_cycles <= 0;
            truncated_packet_count <= 0;
            O_SP_EN <= 0;
            O_LP_EN <= 0;
            O_LP_AV_EN <= 0;
            O_ECC_OK <= 0;
            O_ECC <= 0;
            O_WC <= 0;
            O_VC <= 0;
            O_DT <= 0;
            O_PAYLOAD <= 0;
            O_PAYLOAD_DV <= 0;
            bad_ecc_seen <= 0;
        end else begin
            O_SP_EN <= 0;
            O_LP_EN <= 0;
            O_LP_AV_EN <= 0;
            O_ECC_OK <= 0;
            O_PAYLOAD_DV <= 0;
            if (I_READY) begin
                payload_idle_cycles <= 0;
                if (crc_left != 0) begin
                    crc_left <= 0;
                end else if (in_payload) begin
                    payload_count = (payload_left > 1) ? 2 : 1;
                    O_PAYLOAD <= {I_DATA1, I_DATA0};
                    O_PAYLOAD_DV <= (payload_count == 2) ? 2'b11 : 2'b01;
                    payload_left <= payload_left - payload_count;
                    if (payload_left <= payload_count) begin
                        payload_left <= 0;
                        in_payload <= 0;
                        crc_left <= 2;
                    end
                end else if (header_phase == 0) begin
                    data_id <= I_DATA0;
                    wc_low <= I_DATA1;
                    header_phase <= 1;
                end else begin
                    header_data = {I_DATA0, wc_low, data_id};
                    received_ecc = I_DATA1;
                    header_phase <= 0;
                    O_ECC <= received_ecc;
                    O_WC <= {I_DATA0, wc_low};
                    O_VC <= data_id[7:6];
                    O_DT <= data_id[5:0];
                    if (received_ecc[5:0] == (csi2_header_ecc(header_data) & 8'h3f)) begin
                        O_ECC_OK <= 1;
                        if ({I_DATA0, wc_low} == 0) begin
                            O_SP_EN <= 1;
                        end else if (data_id[5:0] == I_REF_DT) begin
                            O_LP_EN <= 1;
                            O_LP_AV_EN <= 1;
                            payload_left <= {I_DATA0, wc_low};
                            in_payload <= 1;
                        end
                    end else begin
                        bad_ecc_seen <= bad_ecc_seen + 1;
                    end
                end
            end else if (in_payload) begin
                if (payload_idle_cycles >= 7) begin
                    in_payload <= 0;
                    payload_left <= 0;
                    header_phase <= 0;
                    payload_idle_cycles <= 0;
                    truncated_packet_count <= truncated_packet_count + 1;
                end else begin
                    payload_idle_cycles <= payload_idle_cycles + 1;
                end
            end
        end
    end
endmodule


module MIPI_Byte_to_Pixel_Converter_Top (
    input I_RSTN,
    input I_BYTE_CLK,
    input I_PIXEL_CLK,
    input I_SP_EN,
    input I_LP_AV_EN,
    input [5:0] I_DT,
    input [15:0] I_WC,
    input [1:0] I_PAYLOAD_DV,
    input [15:0] I_PAYLOAD,
    output reg O_FV,
    output reg O_LV,
    output reg [7:0] O_PIXEL
);
    reg frame_active;
    reg line_active;
    reg [8:0] pixel_fifo [0:1023];
    reg [10:0] write_ptr;
    reg [10:0] read_ptr;

    always @(posedge I_BYTE_CLK or negedge I_RSTN) begin : enqueue_pixels
        if (!I_RSTN) begin
            frame_active <= 0;
            line_active <= 0;
            write_ptr <= 0;
        end else begin
            if (I_SP_EN) begin
                case (I_DT)
                    6'h00: frame_active <= 1;
                    6'h01: begin frame_active <= 0; line_active <= 0; end
                    6'h02: line_active <= 1;
                    6'h03: line_active <= 0;
                    default: ;
                endcase
            end
            if (I_LP_AV_EN) begin
                line_active <= 1;
            end
            if (I_PAYLOAD_DV[0]) begin
                pixel_fifo[write_ptr[9:0]] <= {line_active, I_PAYLOAD[7:0]};
                write_ptr <= write_ptr + 1'b1;
            end
            if (I_PAYLOAD_DV[1]) begin
                pixel_fifo[(write_ptr + I_PAYLOAD_DV[0]) & 10'h3ff] <= {line_active, I_PAYLOAD[15:8]};
                write_ptr <= write_ptr + I_PAYLOAD_DV[0] + 1'b1;
            end
        end
    end

    always @(posedge I_PIXEL_CLK or negedge I_RSTN) begin
        if (!I_RSTN) begin
            O_FV <= 1;
            O_LV <= 0;
            O_PIXEL <= 0;
            read_ptr <= 0;
        end else begin
            O_FV <= !frame_active;
            if (read_ptr != write_ptr) begin
                O_LV <= pixel_fifo[read_ptr[9:0]][8];
                O_PIXEL <= pixel_fifo[read_ptr[9:0]][7:0];
                read_ptr <= read_ptr + 1'b1;
            end else begin
                O_LV <= 0;
            end
        end
    end
endmodule


// Compact app-side DDR3 memory model: one command selects a 16-byte burst;
// writes are committed on wr_data_en and reads return one cycle after command.
module ddr3_app_memory_model (
    input clk,
    input rst_n,
    input [2:0] cmd,
    input cmd_en,
    input [27:0] addr,
    output wire cmd_ready,
    input wr_data_en,
    input wr_data_end,
    input [127:0] wr_data,
    input [15:0] wr_data_mask,
    output wire wr_data_rdy,
    output reg rd_data_valid,
    output reg rd_data_end,
    output reg [127:0] rd_data,
    output integer write_count,
    output integer read_count,
    output integer error_count
);
    reg [127:0] memory [0:255];
    reg valid [0:255];
    reg write_pending;
    reg read_pending;
    reg [7:0] selected_index;
    integer i;

    assign cmd_ready = 1'b1;
    assign wr_data_rdy = write_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_pending <= 0;
            read_pending <= 0;
            selected_index <= 0;
            rd_data_valid <= 0;
            rd_data_end <= 0;
            rd_data <= 0;
            write_count <= 0;
            read_count <= 0;
            error_count <= 0;
            for (i=0; i<256; i=i+1) begin
                memory[i] <= 0;
                valid[i] <= 0;
            end
        end else begin
            rd_data_valid <= 0;
            rd_data_end <= 0;
            if (cmd_en && cmd_ready) begin
                selected_index <= addr[11:4];
                if (cmd == 3'b001) begin
                    write_pending <= 1;
                end else if (cmd == 3'b000) begin
                    read_pending <= 1;
                end else begin
                    error_count <= error_count + 1;
                end
            end
            if (wr_data_en && wr_data_rdy) begin
                if (wr_data_mask != 0) error_count <= error_count + 1;
                memory[selected_index] <= wr_data;
                valid[selected_index] <= 1;
                write_count <= write_count + 1;
                if (wr_data_end) write_pending <= 0;
            end else if (wr_data_en) begin
                error_count <= error_count + 1;
            end
            if (read_pending) begin
                if (valid[selected_index]) begin
                    rd_data <= memory[selected_index];
                    rd_data_valid <= 1;
                    rd_data_end <= 1;
                    read_count <= read_count + 1;
                end else begin
                    error_count <= error_count + 1;
                end
                read_pending <= 0;
            end
        end
    end
endmodule


// Differential serializer contract model; it checks link-level complementary
// output behavior without claiming to model Gowin OSER10 analog behavior.
module tmds_link_sim_model (
    input pixel_clk,
    input rst_n,
    input de,
    input [23:0] rgb,
    output wire tmds_clk_p,
    output wire tmds_clk_n,
    output reg [2:0] tmds_data_p,
    output wire [2:0] tmds_data_n
);
    reg [2:0] phase;
    assign tmds_clk_p = pixel_clk;
    assign tmds_clk_n = ~pixel_clk;
    assign tmds_data_n = ~tmds_data_p;

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            phase <= 0;
            tmds_data_p <= 0;
        end else if (de) begin
            tmds_data_p <= {rgb[23] ^ phase[0], rgb[15] ^ phase[1], rgb[7] ^ phase[2]};
            phase <= phase + 1'b1;
        end else begin
            tmds_data_p <= {1'b0, 2'b01};
        end
    end
endmodule
