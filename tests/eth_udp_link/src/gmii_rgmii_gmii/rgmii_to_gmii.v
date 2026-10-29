// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2021/07/21 00:00:00
// Module Name   : rgmii_to_gmii
// Description   : 以太网接收rgmii转gmii
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module rgmii_to_gmii(
  reset,

  rgmii_rx_clk,
  rgmii_rxd,
  rgmii_rxdv,

  gmii_rx_clk,
  gmii_rxdv,
  gmii_rxd,
  gmii_rxer
);



  input         reset;

  input         rgmii_rx_clk;
  input  [3:0]  rgmii_rxd;
  input         rgmii_rxdv;

  output        gmii_rx_clk;
  output [7:0]  gmii_rxd;
  output        gmii_rxdv;
  output        gmii_rxer;

  assign gmii_rx_clk = rgmii_rx_clk;

  genvar i;
  generate
    for(i=0;i<4;i=i+1)
    begin: rgmii_rxd_i
      IDDR U_IDDR_dq1 (
        .Q0   (gmii_rxd[i]   ), // 1-bit output for positive edge of clock
        .Q1   (gmii_rxd[i+4] ), // 1-bit output for negative edge of clock
        .D    (rgmii_rxd[i]  ), // 1-bit DDR data input
        .CLK  (rgmii_rx_clk  ) // 1-bit clock input
      );
    end
  endgenerate

  IDDR U_IDDR_dv1 (
    .Q0   (gmii_rxer    ), // 1-bit output for positive edge of clock
    .Q1   (gmii_rxdv    ), // 1-bit output for negative edge of clock
    .D    (rgmii_rxdv   ), // 1-bit DDR data input
    .CLK  (rgmii_rx_clk ) // 1-bit clock input
  );

endmodule