`timescale 1ns/1ps
// Standalone Verilog-2001 quadrature decoder. All control inputs belong to clk.
module dual_ab_quadrature_encoder #(
    parameter integer FILTER_CYCLES = 1,
    parameter integer INVERT_L = 0,
    parameter integer INVERT_R = 0
)(
    input wire clk, rst_n,
    input wire encoder_a_l, encoder_b_l, encoder_a_r, encoder_b_r,
    input wire clear_fault_l, clear_fault_r,
    output wire signed [31:0] position_l, position_r,
    output wire signed [1:0] delta_count_l, delta_count_r,
    output wire direction_l, direction_r,
    output wire [31:0] illegal_transition_l, illegal_transition_r,
    output wire encoder_fault_l, encoder_fault_r
);
    dual_ab_encoder_channel #(.FILTER_CYCLES(FILTER_CYCLES), .INVERT(INVERT_L)) left_channel (
        .clk(clk), .rst_n(rst_n), .a(encoder_a_l), .b(encoder_b_l),
        .clear_fault(clear_fault_l), .position(position_l), .delta_count(delta_count_l),
        .direction(direction_l), .illegal_transition(illegal_transition_l), .encoder_fault(encoder_fault_l));
    dual_ab_encoder_channel #(.FILTER_CYCLES(FILTER_CYCLES), .INVERT(INVERT_R)) right_channel (
        .clk(clk), .rst_n(rst_n), .a(encoder_a_r), .b(encoder_b_r),
        .clear_fault(clear_fault_r), .position(position_r), .delta_count(delta_count_r),
        .direction(direction_r), .illegal_transition(illegal_transition_r), .encoder_fault(encoder_fault_r));
endmodule

module dual_ab_encoder_channel #(
    parameter integer FILTER_CYCLES = 1,
    parameter integer INVERT = 0
)(
    input wire clk, rst_n, a, b, clear_fault,
    output reg signed [31:0] position,
    output reg signed [1:0] delta_count,
    output reg direction,
    output reg [31:0] illegal_transition,
    output reg encoder_fault
);
    function integer bits_needed;
        input integer value;
        integer v;
        begin
            bits_needed = 1;
            v = value - 1;
            while (v > 1) begin bits_needed = bits_needed + 1; v = v >> 1; end
        end
    endfunction
    localparam integer FILTER_WIDTH = bits_needed(FILTER_CYCLES);
    (* ASYNC_REG = "TRUE" *) reg [1:0] ab_meta, ab_sync;
    reg [1:0] sync_valid;
    reg [1:0] candidate, previous;
    reg [FILTER_WIDTH-1:0] stable_count;
    reg initialized;
    reg signed [1:0] step;
    wire accept_state = sync_valid[1] &&
        ((FILTER_CYCLES == 1) || ((ab_sync == candidate) && (stable_count == FILTER_CYCLES-1)));
    // Positive sequence: 00 -> 10 -> 11 -> 01 -> 00.
    always @* begin
        step = 0;
        case ({previous, ab_sync})
            4'b0010, 4'b1011, 4'b1101, 4'b0100: step = INVERT ? -2'sd1 : 2'sd1;
            4'b0001, 4'b0111, 4'b1110, 4'b1000: step = INVERT ? 2'sd1 : -2'sd1;
            default: step = 0;
        endcase
    end
    // synthesis translate_off
    initial begin
        if (FILTER_CYCLES < 1 || (INVERT != 0 && INVERT != 1)) begin
            $display("ERROR: FILTER_CYCLES must be >=1; INVERT must be 0 or 1");
            $fatal(1);
        end
    end
    // synthesis translate_on
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ab_meta <= 0; ab_sync <= 0; sync_valid <= 0;
        end else begin
            ab_meta <= {a,b}; ab_sync <= ab_meta;
            sync_valid <= {sync_valid[0],1'b1};
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            candidate <= 0; previous <= 0; stable_count <= 0; initialized <= 0;
            position <= 0; delta_count <= 0; direction <= 0;
            illegal_transition <= 0; encoder_fault <= 0;
        end else begin
            delta_count <= 0;
            if (clear_fault) begin illegal_transition <= 0; encoder_fault <= 0; end
            if (sync_valid[1]) begin
                if (ab_sync != candidate) begin
                    candidate <= ab_sync;
                    stable_count <= 1;
                end else if (stable_count < FILTER_CYCLES) begin
                    // Hold at N-1 after acceptance; avoids requiring a representation of N.
                    if (stable_count < FILTER_CYCLES-1) stable_count <= stable_count + 1'b1;
                end
                if (accept_state) begin
                    previous <= ab_sync;
                    if (!initialized) initialized <= 1;
                    else if (step != 0) begin
                        position <= position + {{30{step[1]}},step};
                        delta_count <= step;
                        direction <= !step[1];
                    end else if ((previous ^ ab_sync) == 2'b11) begin
                        encoder_fault <= 1;
                        if (clear_fault) illegal_transition <= 1;
                        else if (illegal_transition != 32'hffffffff)
                            illegal_transition <= illegal_transition + 1'b1;
                    end
                end
            end
        end
    end
endmodule
