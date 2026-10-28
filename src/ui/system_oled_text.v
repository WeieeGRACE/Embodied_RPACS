`timescale 1ns/1ps
// Four-line 6x8 ASCII renderer. Character (row*21+column) is at text[8*n+:8].
// Based on the existing imu_attitude_oled renderer; snapshot once per frame.
// Refreshes at approximately 5 Hz over the board's 250 kHz I2C link.
module system_oled_text #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 250000,
    parameter integer POWERUP_CYCLES = CLK_HZ / 10,
    parameter integer FRAME_WAIT_CYCLES = CLK_HZ * 7 / 50
)(
    input wire clk,
    input wire rst_n,
    input wire [671:0] text,
    output reg frame_done,
    output wire scl,
    inout wire sda
);
    localparam [3:0] ST_POWERUP           = 4'd0,
                     ST_INIT_LAUNCH       = 4'd1,
                     ST_INIT_WAIT         = 4'd2,
                     ST_PAGE_COMMAND      = 4'd3,
                     ST_PAGE_COMMAND_WAIT = 4'd4,
                     ST_PIXEL_LAUNCH      = 4'd5,
                     ST_PIXEL_WAIT        = 4'd6,
                     ST_FRAME_WAIT        = 4'd7,
                     ST_PIXEL_PREPARE     = 4'd8,
                     ST_PIXEL_LOOKUP      = 4'd9;

    reg [3:0] state;
    reg [31:0] timer;
    reg [4:0] init_index;
    reg [2:0] page;
    reg [1:0] page_command_index;
    reg [6:0] column;
    reg clear_mode;
    reg byte_start;
    reg byte_data_mode;
    reg [7:0] byte_data;
    reg byte_data_mode_latched;
    reg [7:0] byte_data_latched;
    wire byte_busy;
    wire byte_done;
    wire byte_scl;
    reg [671:0] display_text;
    // Character scan position within one text row plus the glyph column.
    reg [4:0] char_index;
    reg [2:0] char_col;
    reg [9:0] font_addr;
    reg [7:0] font_byte;
    // 128 characters x 6 columns, initialized from oled_font.hex. The
    // synchronous read lets synthesis place the table in a BSRAM pROM
    // instead of a per-character LUT mux tree.
    reg [7:0] font_rom [0:767];
    initial $readmemh("src/ui/oled_font.hex", font_rom);

    oled_i2c_write_byte #(.CLK_HZ(CLK_HZ), .I2C_HZ(I2C_HZ)) i2c_writer (
        .clk(clk), .rst_n(rst_n), .start(byte_start),
        .data_mode(byte_data_mode_latched), .data_in(byte_data_latched),
        .busy(byte_busy), .done(byte_done), .scl(byte_scl), .sda(sda)
    );
    assign scl = byte_scl;

    function [7:0] init_command;
        input [4:0] index;
        begin
            case (index)
                0: init_command = 8'hAE;
                1: init_command = 8'h00;
                2: init_command = 8'h10;
                3: init_command = 8'h40;
                4: init_command = 8'h81;
                5: init_command = 8'hCF;
                6: init_command = 8'hA1;
                7: init_command = 8'hC8;
                8: init_command = 8'hA6;
                9: init_command = 8'hA8;
                10: init_command = 8'h3F;
                11: init_command = 8'hD3;
                12: init_command = 8'h00;
                13: init_command = 8'hD5;
                14: init_command = 8'h80;
                15: init_command = 8'hD9;
                16: init_command = 8'hF1;
                17: init_command = 8'hDA;
                18: init_command = 8'h12;
                19: init_command = 8'hDB;
                20: init_command = 8'h30;
                21: init_command = 8'h20;
                22: init_command = 8'h02;
                23: init_command = 8'h8D;
                24: init_command = 8'h14;
                default: init_command = 8'hAF;
            endcase
        end
    endfunction

    function [7:0] screen_character;
        input [1:0] row;
        input [4:0] position;
        begin
            screen_character = display_text[(row*21+position)*8+:8];
        end
    endfunction

    wire [7:0] current_char = screen_character(page, char_index);
    // font address = character * 6 + glyph column (10 bits hold 767).
    wire [10:0] font_addr_next = {current_char[6:0], 3'b000}
                               + {1'b0, current_char[7:0], 1'b0}
                               + {8'b0, char_col};

    function [7:0] page_command;
        input [2:0] page_number;
        input [1:0] command_index;
        begin
            case (command_index)
                0: page_command = 8'hB0 | page_number;
                1: page_command = 8'h00;
                default: page_command = 8'h10;
            endcase
        end
    endfunction

    always @* begin
        byte_data_mode = 1'b0;
        byte_data = 8'h00;
        case (state)
            ST_INIT_LAUNCH: byte_data = init_command(init_index);
            ST_PAGE_COMMAND: byte_data = page_command(
                clear_mode ? page : {page[1:0],1'b0}, page_command_index);
            ST_PIXEL_LAUNCH: begin
                byte_data_mode = 1'b1;
                if (clear_mode) begin
                    byte_data = 0;
                end else if (column < 126) begin
                    byte_data = font_byte;
                end else begin
                    byte_data = 0;
                end
            end
            default: begin end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_POWERUP;
            timer <= 0;
            init_index <= 0;
            page <= 0;
            page_command_index <= 0;
            column <= 0;
            char_index <= 0;
            char_col <= 0;
            font_addr <= 0;
            font_byte <= 0;
            clear_mode <= 1'b1;
            byte_start <= 0;
            byte_data_mode_latched <= 0;
            byte_data_latched <= 0;
            display_text <= {84{8'h20}}; frame_done <= 0;
        end else begin
            frame_done <= 0;
            byte_start <= 0;
            case (state)
                ST_POWERUP: begin
                    if (timer >= POWERUP_CYCLES - 1) begin
                        timer <= 0;
                        init_index <= 0;
                        state <= ST_INIT_LAUNCH;
                    end else timer <= timer + 1'b1;
                end
                ST_INIT_LAUNCH: begin
                    byte_start <= 1;
                    byte_data_latched <= byte_data;
                    byte_data_mode_latched <= byte_data_mode;
                    state <= ST_INIT_WAIT;
                end
                ST_INIT_WAIT: begin
                    if (byte_done) begin
                        if (init_index == 25) begin
                            page <= 0;
                            page_command_index <= 0;
                            clear_mode <= 1'b1;
                            state <= ST_PAGE_COMMAND;
                        end else begin
                            init_index <= init_index + 1'b1;
                            state <= ST_INIT_LAUNCH;
                        end
                    end
                end
                ST_PAGE_COMMAND: begin
                    byte_start <= 1;
                    byte_data_latched <= byte_data;
                    byte_data_mode_latched <= byte_data_mode;
                    state <= ST_PAGE_COMMAND_WAIT;
                end
                ST_PAGE_COMMAND_WAIT: begin
                    if (byte_done) begin
                        if (page_command_index != 2) begin
                            page_command_index <= page_command_index + 1'b1;
                            state <= ST_PAGE_COMMAND;
                        end else begin
                            page_command_index <= 0;
                            column <= 0;
                            char_index <= 0;
                            char_col <= 0;
                            state <= ST_PIXEL_PREPARE;
                        end
                    end
                end
                // Separate signed decimal formatting from glyph lookup and I2C launch.
                ST_PIXEL_PREPARE: begin
                    font_addr <= font_addr_next[9:0];
                    state <= ST_PIXEL_LOOKUP;
                end
                ST_PIXEL_LOOKUP: begin
                    state <= ST_PIXEL_LAUNCH;
                end
                ST_PIXEL_LAUNCH: begin
                    byte_start <= 1;
                    byte_data_latched <= byte_data;
                    byte_data_mode_latched <= byte_data_mode;
                    state <= ST_PIXEL_WAIT;
                end
                ST_PIXEL_WAIT: begin
                    if (byte_done) begin
                        if (column == 127) begin
                            column <= 0;
                            char_index <= 0;
                            char_col <= 0;
                            if (clear_mode && page != 7) begin
                                page <= page + 1'b1;
                                page_command_index <= 0;
                                state <= ST_PAGE_COMMAND;
                            end else if (clear_mode) begin
                                clear_mode <= 1'b0;
                                display_text <= text;
                                page <= 0;
                                page_command_index <= 0;
                                state <= ST_PAGE_COMMAND;
                            end else if (page != 3) begin
                                page <= page + 1'b1;
                                page_command_index <= 0;
                                state <= ST_PAGE_COMMAND;
                            end else begin
                                frame_done <= 1;
                                timer <= 0;
                                state <= ST_FRAME_WAIT;
                            end
                        end else begin
                            column <= column + 1'b1;
                            char_col <= char_col == 5 ? 3'd0 : char_col + 1'b1;
                            if (char_col == 5) char_index <= char_index + 1'b1;
                            state <= ST_PIXEL_PREPARE;
                        end
                    end
                end
                ST_FRAME_WAIT: begin
                    if (timer >= FRAME_WAIT_CYCLES - 1) begin
                                display_text <= text;
                        timer <= 0;
                        page <= 0;
                        page_command_index <= 0;
                        clear_mode <= 1'b0;
                        state <= ST_PAGE_COMMAND;
                    end else timer <= timer + 1'b1;
                end
                default: begin
                    state <= ST_POWERUP;
                    timer <= 0;
                end
            endcase
        end
    end
endmodule
