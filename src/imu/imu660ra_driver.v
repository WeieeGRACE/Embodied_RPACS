`timescale 1ns/1ps
// IMU660RA register/initialization sequencer. The public axes update atomically
// only after both six-byte bursts have completed.
module imu660ra_driver #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SPI_HALF_PERIOD = 25,
    parameter integer POWERUP_CYCLES = CLK_HZ / 50,
    parameter integer CONFIG_WAIT_CYCLES = CLK_HZ / 50,
    parameter integer ID_RETRY_CYCLES = CLK_HZ / 1000,
    parameter integer RETRY_CYCLES = CLK_HZ,
    parameter integer SAMPLE_CYCLES = CLK_HZ / 200,
    parameter integer HEALTH_CHECK_CYCLES = 0,
    parameter integer ID_RETRIES = 10,
    parameter CONFIG_HEX_FILE = "src/imu/imu660ra_config.hex"
)(
    input wire clk,
    input wire rst_n,
    input wire spi_miso,
    input wire imu_int1,
    input wire imu_int2,
    output wire spi_sclk,
    output wire spi_mosi,
    output reg spi_cs_n,
    output reg signed [15:0] accel_x,
    output reg signed [15:0] accel_y,
    output reg signed [15:0] accel_z,
    output reg signed [15:0] gyro_x,
    output reg signed [15:0] gyro_y,
    output reg signed [15:0] gyro_z,
    output reg sample_valid,
    output reg [31:0] sample_sequence,
    output reg [7:0] chip_id,
    output reg [7:0] init_status,
    output reg [7:0] power_conf_readback,
    output reg [7:0] init_ctrl_readback,
    output reg [7:0] sensor_error,
    output reg ready,
    output reg [7:0] error_code,
    output wire int1_sync,
    output wire int2_sync
);
    localparam [5:0]
        ST_POWER_WAIT       = 0,
        ST_ID_CHECK         = 1,
        ST_ID_RETRY_DELAY   = 2,
        ST_WRITE_BEGIN      = 3,
        ST_WRITE_CMD_WAIT   = 4,
        ST_WRITE_DATA_WAIT  = 5,
        ST_WRITE_END        = 6,
        ST_CONFIG_BEGIN     = 7,
        ST_CONFIG_CMD_WAIT  = 8,
        ST_CONFIG_ROM_WAIT  = 9,
        ST_CONFIG_SEND      = 10,
        ST_CONFIG_BYTE_WAIT = 11,
        ST_CONFIG_CTRL_DONE = 12,
        ST_CONFIG_WAIT      = 13,
        ST_STATUS_CHECK     = 14,
        ST_PWRCTRL_DONE     = 15,
        ST_ACC_CONF_DONE    = 16,
        ST_GYR_CONF_DONE    = 17,
        ST_ACC_RANGE_DONE   = 18,
        ST_GYR_RANGE_DONE   = 19,
        ST_RUN              = 20,
        ST_ACC_DECODE       = 21,
        ST_GYRO_DECODE      = 22,
        ST_RETRY_WAIT       = 23,
        ST_CONFIG_STREAM_DONE = 29,
        ST_READ_BEGIN       = 24,
        ST_READ_CMD_WAIT    = 25,
        ST_READ_BYTE_BEGIN  = 26,
        ST_READ_BYTE_WAIT   = 27,
        ST_READ_END         = 28,
        ST_WRITE_DATA_BEGIN = 30,
        ST_POWER_CONF_WAIT  = 31,
        ST_ID_SUCCESS_WAIT  = 32,
        ST_WRITE_RECOVERY   = 33,
        ST_CONFIG_RECOVERY  = 34,
        ST_INIT_ADDR_HIGH   = 35,
        ST_DIAG_POWER       = 36,
        ST_DIAG_CTRL        = 37,
        ST_DIAG_ERROR       = 38,
        ST_INIT_RESULT      = 39,
        ST_SOFT_RESET_WAIT  = 40,
        ST_HEALTH_ID = 41, ST_HEALTH_ERROR = 42;

    // Bosch SensorAPI bmi2_set_regs waits 2 us after normal-mode writes.
    // The existing 1 ms PWR_CONF wait also covers the APS transition.
    localparam integer WRITE_RECOVERY_CYCLES =
        (CLK_HZ / 500000 > 0) ? CLK_HZ / 500000 : 1;

    // Values match imu660ra_state_enum in zf_device_imu660ra.h.
    localparam [7:0] IMU_STATE_NO_ERROR = 8'd0;
    localparam [7:0] IMU_STATE_SELF_CHECK_ERROR = 8'd4;
    localparam [7:0] IMU_STATE_SET_CONFIG_ERROR = 8'd5;

    reg [5:0] state;
    reg [5:0] after_operation;
    reg [7:0] operation_reg;
    reg [7:0] operation_value;
    reg [3:0] operation_length;
    reg [3:0] read_index;
    reg [7:0] read_buffer [0:6];
    reg [3:0] id_attempt;
    reg sensor_reset_done;
    reg [31:0] delay_count;
    reg [31:0] sample_timer;
    reg sample_pending;
    reg [31:0] health_timer;
    reg health_pending;
    reg [12:0] config_index;
    wire [7:0] config_rom_byte;
    reg [12:0] config_rom_address;
    reg [7:0] acc_x_work, acc_x_hi_work;
    reg [7:0] acc_y_work, acc_y_hi_work;
    reg [7:0] acc_z_work, acc_z_hi_work;

    reg spi_start;
    reg [7:0] spi_tx_data;
    wire spi_ready;
    wire spi_busy;
    wire spi_done;
    wire [7:0] spi_rx_data;

    (* ASYNC_REG = "TRUE" *) reg [1:0] int1_pipe;
    (* ASYNC_REG = "TRUE" *) reg [1:0] int2_pipe;
    assign int1_sync = int1_pipe[1];
    assign int2_sync = int2_pipe[1];

    imu660ra_spi_master #(.HALF_PERIOD(SPI_HALF_PERIOD)) spi_master (
        .clk(clk), .rst_n(rst_n), .start(spi_start), .tx_data(spi_tx_data),
        .ready(spi_ready), .busy(spi_busy), .done(spi_done),
        .rx_data(spi_rx_data), .spi_miso(spi_miso),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi)
    );

    imu660ra_config_rom #(.INIT_FILE(CONFIG_HEX_FILE)) config_rom (
        .clk(clk), .address(config_rom_address), .data(config_rom_byte)
    );

    task schedule_write;
        input [7:0] reg_addr;
        input [7:0] reg_value;
        input [5:0] next_state;
        begin
            operation_reg <= reg_addr;
            operation_value <= reg_value;
            after_operation <= next_state;
            state <= ST_WRITE_BEGIN;
        end
    endtask

    task schedule_read;
        input [7:0] reg_addr;
        input [3:0] read_length;
        input [5:0] next_state;
        begin
            operation_reg <= reg_addr;
            operation_length <= read_length;
            after_operation <= next_state;
            state <= 6'd24;
        end
    endtask


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            int1_pipe <= 0;
            int2_pipe <= 0;
        end else begin
            int1_pipe <= {int1_pipe[0], imu_int1};
            int2_pipe <= {int2_pipe[0], imu_int2};
        end
    end

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_POWER_WAIT;
            after_operation <= ST_POWER_WAIT;
            operation_reg <= 0;
            operation_value <= 0;
            operation_length <= 0;
            read_index <= 0;
            id_attempt <= 0;
            sensor_reset_done <= 0;
            delay_count <= 0;
            sample_timer <= 0;
            sample_pending <= 0;
            health_timer <= 0; health_pending <= 0;
            config_index <= 0;
            config_rom_address <= 0;
            spi_start <= 0;
            spi_tx_data <= 0;
            spi_cs_n <= 1;
            accel_x <= 0;
            accel_y <= 0;
            accel_z <= 0;
            gyro_x <= 0;
            gyro_y <= 0;
            gyro_z <= 0;
            acc_x_work <= 0;
            acc_x_hi_work <= 0;
            acc_y_work <= 0;
            acc_y_hi_work <= 0;
            acc_z_work <= 0;
            acc_z_hi_work <= 0;
            sample_valid <= 0;
            sample_sequence <= 0;
            chip_id <= 0;
            init_status <= 0;
            power_conf_readback <= 0;
            init_ctrl_readback <= 0;
            sensor_error <= 0;
            ready <= 0;
            error_code <= IMU_STATE_NO_ERROR;
            for (i = 0; i < 7; i = i + 1)
                read_buffer[i] <= 0;
        end else begin
            spi_start <= 0;
            sample_valid <= 0;

            if (ready && !sample_pending) begin
                if (sample_timer >= SAMPLE_CYCLES - 1) begin
                    sample_timer <= 0;
                    sample_pending <= 1;
                end else begin
                    sample_timer <= sample_timer + 1'b1;
                end
            end else if (!ready) begin
                sample_timer <= 0;
            end

            if (HEALTH_CHECK_CYCLES != 0 && ready) begin
                if (!health_pending) begin
                    if (health_timer >= HEALTH_CHECK_CYCLES-1) begin
                        health_pending <= 1; health_timer <= 0;
                    end else health_timer <= health_timer+1'b1;
                end
            end else begin health_timer <= 0; health_pending <= 0; end
            case (state)
                ST_POWER_WAIT: begin
                    spi_cs_n <= 1;
                    if (delay_count >= POWERUP_CYCLES - 1) begin
                        delay_count <= 0;
                        id_attempt <= 0;
                        schedule_read(8'h00, 4'd2, ST_ID_CHECK);
                        state <= ST_READ_BEGIN;
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end

                ST_ID_CHECK: begin
                    chip_id <= read_buffer[0];
                    if (read_buffer[0] == 8'h24) begin
                        // Match the C driver: leave one millisecond between
                        // the successful ID transaction and PWR_CONF.
                        delay_count <= 0;
                        if (!sensor_reset_done) begin
                            // Restore a known sensor state even when only the
                            // FPGA was reprogrammed. Bosch bmi2_soft_reset uses
                            // CMD=B6, a 2 ms wait, then an SPI dummy ID read.
                            sensor_reset_done <= 1;
                            schedule_write(8'h7E, 8'hB6, ST_SOFT_RESET_WAIT);
                        end else begin
                            state <= ST_ID_SUCCESS_WAIT;
                        end
                    end else if (id_attempt < ID_RETRIES) begin
                        id_attempt <= id_attempt + 1'b1;
                        delay_count <= 0;
                        state <= ST_ID_RETRY_DELAY;
                    end else begin
                        ready <= 0;
                        error_code <= IMU_STATE_SELF_CHECK_ERROR;
                        delay_count <= 0;
                        state <= ST_RETRY_WAIT;
                    end
                end

                ST_ID_RETRY_DELAY: begin
                    if (delay_count >= ID_RETRY_CYCLES - 1) begin
                        delay_count <= 0;
                        schedule_read(8'h00, 4'd2, ST_ID_CHECK);
                        state <= ST_READ_BEGIN;
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end

                ST_ID_SUCCESS_WAIT: begin
                    if (delay_count >= (CLK_HZ / 1000) - 1) begin
                        delay_count <= 0;
                        schedule_write(8'h7C, 8'h00, ST_CONFIG_CTRL_DONE);
                        state <= ST_WRITE_BEGIN;
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end
                ST_SOFT_RESET_WAIT: begin
                    if (delay_count >= (CLK_HZ / 500) - 1) begin
                        delay_count <= 0;
                        id_attempt <= 0;
                        schedule_read(8'h00, 4'd2, ST_ID_CHECK);
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end

                ST_WRITE_BEGIN: begin
                    spi_cs_n <= 0;
                    spi_tx_data <= operation_reg;
                    spi_start <= 1;
                    state <= ST_WRITE_CMD_WAIT;
                end
                ST_WRITE_CMD_WAIT: begin
                    if (spi_done) begin
                        spi_tx_data <= operation_value;
                        state <= ST_WRITE_DATA_BEGIN;
                    end
                end
                ST_WRITE_DATA_BEGIN: begin
                    spi_start <= 1;
                    state <= ST_WRITE_DATA_WAIT;
                end
                ST_WRITE_DATA_WAIT: begin
                    if (spi_done)
                        state <= ST_WRITE_END;
                end
                ST_WRITE_END: begin
                    spi_cs_n <= 1;
                    delay_count <= 0;
                    state <= ST_WRITE_RECOVERY;
                end
                ST_WRITE_RECOVERY: begin
                    if (delay_count >= WRITE_RECOVERY_CYCLES - 1) begin
                        delay_count <= 0;
                        state <= after_operation;
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end

                ST_CONFIG_BEGIN: begin
                    spi_cs_n <= 0;
                    config_index <= 0;
                    config_rom_address <= 0;
                    spi_tx_data <= 8'h5E;
                    spi_start <= 1;
                    state <= ST_CONFIG_CMD_WAIT;
                end
                ST_CONFIG_CMD_WAIT: begin
                    if (spi_done) begin
                        state <= ST_CONFIG_ROM_WAIT;
                    end
                end
                ST_CONFIG_ROM_WAIT: begin
                    // The ROM is synchronous; address is registered one cycle
                    // before its byte is sent to SPI.
                    state <= ST_CONFIG_SEND;
                end
                ST_CONFIG_SEND: begin
                    spi_tx_data <= config_rom_byte;
                    spi_start <= 1;
                    state <= ST_CONFIG_BYTE_WAIT;
                end
                ST_CONFIG_BYTE_WAIT: begin
                    if (spi_done) begin
                        if (config_index == 13'd8191) begin
                            spi_cs_n <= 1;
                            delay_count <= 0;
                            state <= ST_CONFIG_RECOVERY;
                        end else begin
                            config_index <= config_index + 1'b1;
                            config_rom_address <= config_index + 1'b1;
                            state <= ST_CONFIG_ROM_WAIT;
                        end
                    end
                end
                ST_CONFIG_CTRL_DONE: begin
                    if (operation_reg == 8'h7C && operation_value == 8'h00) begin
                        // IMU660RA needs time to leave power-save mode before
                        // INIT_CTRL and the configuration image are written.
                        delay_count <= 0;
                        state <= ST_POWER_CONF_WAIT;
                    end else if (operation_reg == 8'h59 && operation_value == 8'h00) begin
                        // Explicitly rewind the configuration word address on
                        // every upload, including retries without sensor power cycling.
                        // Bosch SensorAPI upload_file writes INIT_ADDR before INIT_DATA.
                        schedule_write(8'h5B, 8'h00, ST_INIT_ADDR_HIGH);
                    end else begin
                        // The first write in initialization is PWR_CONF=0.
                        schedule_write(8'h59, 8'h00, ST_CONFIG_CTRL_DONE);
                        state <= ST_WRITE_BEGIN;
                    end
                end
                ST_POWER_CONF_WAIT: begin
                    if (delay_count >= (CLK_HZ / 1000) - 1) begin
                        delay_count <= 0;
                        schedule_write(8'h59, 8'h00, ST_CONFIG_CTRL_DONE);
                        state <= ST_WRITE_BEGIN;
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end
                ST_INIT_ADDR_HIGH: begin
                    schedule_write(8'h5C, 8'h00, ST_CONFIG_BEGIN);
                end
                ST_CONFIG_STREAM_DONE: begin
                    delay_count <= 0;
                    schedule_write(8'h59, 8'h01, ST_CONFIG_WAIT);
                    state <= ST_WRITE_BEGIN;
                end
                ST_CONFIG_RECOVERY: begin
                    if (delay_count >= WRITE_RECOVERY_CYCLES - 1) begin
                        delay_count <= 0;
                        state <= ST_CONFIG_STREAM_DONE;
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end
                ST_CONFIG_WAIT: begin
                    if (delay_count >= CONFIG_WAIT_CYCLES - 1) begin
                        delay_count <= 0;
                        schedule_read(8'h21, 4'd2, ST_STATUS_CHECK);
                        state <= ST_READ_BEGIN;
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end
                ST_STATUS_CHECK: begin
                    init_status <= read_buffer[0];
                    schedule_read(8'h7C, 4'd2, ST_DIAG_POWER);
                end
                ST_DIAG_POWER: begin
                    power_conf_readback <= read_buffer[0];
                    schedule_read(8'h59, 4'd2, ST_DIAG_CTRL);
                end
                ST_DIAG_CTRL: begin
                    init_ctrl_readback <= read_buffer[0];
                    schedule_read(8'h02, 4'd2, ST_DIAG_ERROR);
                end
                ST_DIAG_ERROR: begin
                    sensor_error <= read_buffer[0];
                    state <= ST_INIT_RESULT;
                end
                ST_INIT_RESULT: begin
                    if (init_status == 8'h01) begin
                        schedule_write(8'h7D, 8'h0E, ST_PWRCTRL_DONE);
                        state <= ST_WRITE_BEGIN;
                    end else begin
                        ready <= 0;
                        error_code <= IMU_STATE_SET_CONFIG_ERROR;
                        delay_count <= 0;
                        state <= ST_RETRY_WAIT;
                    end
                end
                ST_PWRCTRL_DONE: begin
                    schedule_write(8'h40, 8'hA7, ST_ACC_CONF_DONE);
                    state <= ST_WRITE_BEGIN;
                end
                ST_ACC_CONF_DONE: begin
                    schedule_write(8'h42, 8'hA9, ST_GYR_CONF_DONE);
                    state <= ST_WRITE_BEGIN;
                end
                ST_GYR_CONF_DONE: begin
                    schedule_write(8'h41, 8'h02, ST_ACC_RANGE_DONE);
                    state <= ST_WRITE_BEGIN;
                end
                ST_ACC_RANGE_DONE: begin
                    schedule_write(8'h43, 8'h00, ST_GYR_RANGE_DONE);
                    state <= ST_WRITE_BEGIN;
                end
                ST_GYR_RANGE_DONE: begin
                    error_code <= IMU_STATE_NO_ERROR;
                    ready <= 1;
                    sample_timer <= 0;
                    sample_pending <= 0;
                    state <= ST_RUN;
                end

                ST_RUN: begin
                    if (HEALTH_CHECK_CYCLES != 0 && health_pending) begin
                        health_pending <= 0;
                        schedule_read(8'h00, 4'd2, ST_HEALTH_ID);
                    end else if (sample_pending) begin
                        sample_pending <= 0;
                        operation_reg <= 8'h0C;
                        operation_length <= 4'd7;
                        after_operation <= ST_ACC_DECODE;
                        state <= ST_READ_BEGIN;
                    end
                end
                ST_HEALTH_ID: begin
                    chip_id <= read_buffer[0];
                    if (read_buffer[0] != 8'h24) begin
                        ready <= 0; error_code <= 8'd6;
                        delay_count <= 0; state <= ST_RETRY_WAIT;
                    end else schedule_read(8'h02, 4'd2, ST_HEALTH_ERROR);
                end
                ST_HEALTH_ERROR: begin
                    sensor_error <= read_buffer[0];
                    if (read_buffer[0] != 0) begin
                        ready <= 0; error_code <= 8'd7;
                        delay_count <= 0; state <= ST_RETRY_WAIT;
                    end else state <= ST_RUN;
                end
                ST_ACC_DECODE: begin
                    acc_x_work <= read_buffer[0];
                    acc_x_hi_work <= read_buffer[1];
                    acc_y_work <= read_buffer[2];
                    acc_y_hi_work <= read_buffer[3];
                    acc_z_work <= read_buffer[4];
                    acc_z_hi_work <= read_buffer[5];
                    operation_reg <= 8'h12;
                    operation_length <= 4'd7;
                    after_operation <= ST_GYRO_DECODE;
                    state <= ST_READ_BEGIN;
                end
                ST_GYRO_DECODE: begin
                    accel_x <= {acc_x_hi_work, acc_x_work};
                    accel_y <= {acc_y_hi_work, acc_y_work};
                    accel_z <= {acc_z_hi_work, acc_z_work};
                    gyro_x <= {read_buffer[1], read_buffer[0]};
                    gyro_y <= {read_buffer[3], read_buffer[2]};
                    gyro_z <= {read_buffer[5], read_buffer[4]};
                    sample_sequence <= sample_sequence + 1'b1;
                    sample_valid <= 1;
                    state <= ST_RUN;
                end

                ST_READ_BEGIN: begin
                    spi_cs_n <= 0;
                    spi_tx_data <= operation_reg | 8'h80;
                    spi_start <= 1;
                    state <= ST_READ_CMD_WAIT;
                end
                ST_READ_CMD_WAIT: begin
                    if (spi_done) begin
                        read_index <= 0;
                        state <= ST_READ_BYTE_BEGIN;
                    end
                end
                ST_READ_BYTE_BEGIN: begin
                    spi_tx_data <= 8'hFF;
                    spi_start <= 1;
                    state <= ST_READ_BYTE_WAIT;
                end
                ST_READ_BYTE_WAIT: begin
                    if (spi_done) begin
                        if (read_index != 0)
                            read_buffer[read_index - 1'b1] <= spi_rx_data;
                        if (read_index == operation_length - 1'b1) begin
                            state <= ST_READ_END;
                        end else begin
                            read_index <= read_index + 1'b1;
                            state <= ST_READ_BYTE_BEGIN;
                        end
                    end
                end
                ST_READ_END: begin
                    spi_cs_n <= 1;
                    state <= after_operation;
                end

                ST_RETRY_WAIT: begin
                    spi_cs_n <= 1;
                    if (delay_count >= RETRY_CYCLES - 1) begin
                        delay_count <= 0;
                        id_attempt <= 0;
                        sensor_reset_done <= 0;
                        state <= ST_POWER_WAIT;
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end

                default: begin
                    spi_cs_n <= 1;
                    state <= ST_POWER_WAIT;
                    ready <= 0;
                end
            endcase
        end
    end
endmodule
