//---------------------------------------------------------------------
// File name  : ram_line.v
// Module name: ram_line
// Created by : 
// ---------------------------------------------------------------------
// Release history
// VERSION |  Date       | AUTHOR       |    DESCRIPTION
// ---------------------------------------------------------------------
//  1.0    | 27-May-2016 |              | initial 
// ---------------------------------------------------------------------

module ram_line  #
(  
  parameter   ADDR_WIDTH=11, //RAM深度= 2^ADDR_WIDTH
  parameter   DATA_WIDTH=16
)
(
  input                         clk   ,
  input       [ADDR_WIDTH-1:0]  wraddr,
  input                         wren  ,
  input       [DATA_WIDTH-1:0]  data  ,
  input       [ADDR_WIDTH-1:0]  rdaddr,
  input                         rden  ,
  output reg  [DATA_WIDTH-1:0]  q     
);

reg   [DATA_WIDTH-1:0] ram[(1<<ADDR_WIDTH)-1:0];  ///* synthesis syn_ramstyle="block_ram " */

always@(posedge clk)
begin
  if(wren)  
    ram[wraddr]<=data;
end

always@(posedge clk)
begin
  if(rden)  
    q<=ram[rdaddr];
  else
    q<=q;
end

endmodule