//---------------------------------------------------------------------
// File name  : shift_line.v
// Module name: shift_line
// Created by : 
// ---------------------------------------------------------------------
// Release history
// VERSION |  Date       | AUTHOR       |    DESCRIPTION
// ---------------------------------------------------------------------
//  1.0    |  6-May-2016 |              | initial 
// ---------------------------------------------------------------------

module shift_line #
(
  parameter   ADDR_WIDTH=11, //RAM深度= 2^ADDR_WIDTH
  parameter   DATA_WIDTH=24
)
(
  input                    aclr     ,
  input                    clken    ,
  input                    clock    ,
  input  [DATA_WIDTH-1: 0] shiftin  ,
  output [DATA_WIDTH-1: 0] shiftout ,
  input  [15: 0]           delay_num
);

//==============================================
reg  [ADDR_WIDTH-1: 0] shiftin_addr; //RAM深度2048，所以位宽11位
reg  [ADDR_WIDTH-1: 0] shiftout_addr;

//==============================================
ram_line #  //24bitx2048
(
  .ADDR_WIDTH  (ADDR_WIDTH),
  .DATA_WIDTH  (DATA_WIDTH)
)
ram_line_inst
(
  .clk    (clock        ),
  .wraddr (shiftin_addr ),
	.wren   (clken        ),
	.data   (shiftin      ),
	.rdaddr (shiftout_addr),
	.rden   (clken        ),
	.q      (shiftout     )
);

//==============================================
always @(posedge clock or posedge aclr)
begin
  if(aclr)
    shiftin_addr <= 0;
  else if(clken)
    shiftin_addr <= shiftin_addr + 2'd1;
  else
    shiftin_addr <= shiftin_addr;
end

always @(posedge clock or posedge aclr)
begin
  if(aclr)
    shiftout_addr <= 0;
  else if(clken)
    shiftout_addr <= shiftin_addr - (delay_num - 3'd2);
  else
    shiftout_addr <= shiftout_addr;
end

endmodule
