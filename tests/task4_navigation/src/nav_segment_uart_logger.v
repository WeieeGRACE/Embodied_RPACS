`timescale 1ns/1ps
// Nonblocking compact segment/fault logger, 115200 8N1 through uart_tx.
module nav_segment_uart_logger #(
    parameter integer CLK_HZ=50000000,
    parameter integer BAUD=115200
) (
    input wire clk,input wire rst_n,
    input wire record_valid,input wire [3:0] segment_id,
    input wire [31:0] distance_mm,input wire signed [31:0] yaw_mdeg,
    input wire [7:0] wall_id,input wire [7:0] fault_code,
    output wire tx,output reg [7:0] dropped_records
);
    reg active;
    reg [5:0] index;
    reg [3:0] saved_segment;
    reg [31:0] saved_distance;
    reg signed [31:0] saved_yaw;
    reg [7:0] saved_wall,saved_fault;
    wire tx_ready,tx_done;
    wire tx_valid=active && tx_ready;
    reg [7:0] tx_data;
    uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) tx_i(
      .clk(clk),.rst_n(rst_n),.data(tx_data),.valid(tx_valid),.ready(tx_ready),.tx(tx),.done(tx_done));

    function automatic [7:0] hexchar(input [3:0] n);
        begin hexchar=n<10 ? 8'd48+n : 8'd55+n;end
    endfunction
    function automatic [7:0] field_char(input [5:0] pos,input [3:0] seg,
        input [31:0] distance_value,input signed [31:0] yaw,input [7:0] wall,input [7:0] fault);
        begin
            case(pos)
                0:field_char="T";1:field_char="4";2:field_char=" ";
                3:field_char="S";4:field_char="E";5:field_char="G";6:field_char="=";
                7:field_char=hexchar({4'd0,seg});8:field_char=" ";
                9:field_char="D";10:field_char="I";11:field_char="S";12:field_char="T";13:field_char="=";
                22:field_char=" ";23:field_char="Y";24:field_char="A";25:field_char="W";26:field_char="=";
                35:field_char=" ";36:field_char="W";37:field_char="A";38:field_char="L";39:field_char="L";40:field_char="=";
                43:field_char=" ";44:field_char="F";45:field_char="A";46:field_char="U";47:field_char="L";48:field_char="T";49:field_char="=";
                52:field_char=8'h0d;53:field_char=8'h0a;
                default:begin
                    if(pos>=14 && pos<=21) field_char=hexchar(distance_value[(21-pos)*4+:4]);
                    else if(pos>=27 && pos<=34) field_char=hexchar(yaw[(34-pos)*4+:4]);
                    else if(pos>=41 && pos<=42) field_char=hexchar(wall[(42-pos)*4+:4]);
                    else if(pos>=50 && pos<=51) field_char=hexchar(fault[(51-pos)*4+:4]);
                    else field_char="?";
                end
            endcase
        end
    endfunction

    always @* tx_data=field_char(index,saved_segment,saved_distance,saved_yaw,saved_wall,saved_fault);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            active<=0;index<=0;saved_segment<=0;saved_distance<=0;saved_yaw<=0;
            saved_wall<=0;saved_fault<=0;dropped_records<=0;
        end else begin
            if(record_valid) begin
                if(!active) begin
                    active<=1;index<=0;saved_segment<=segment_id;saved_distance<=distance_mm;
                    saved_yaw<=yaw_mdeg;saved_wall<=wall_id;saved_fault<=fault_code;
                end else if(dropped_records!=8'hff) dropped_records<=dropped_records+1'b1;
            end
            if(tx_valid) begin
                if(index==53) begin active<=0;index<=0;end
                else index<=index+1'b1;
            end
        end
    end
endmodule
