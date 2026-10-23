//Copyright (C)2014-2023 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//GOWIN Version: V1.9.9 Beta-6
//Part Number: GW5A-LV25UG324ES
//Device: GW5A-25
//Device Version: A
//Created Time: Fri Nov 17 17:13:56 2023

module Gowin_MIPI_DPHY (rx_clk_o, d0ln_hsrxd, d1ln_hsrxd, d0ln_hsrxd_vld, d1ln_hsrxd_vld, di_lprx0_n, di_lprx0_p, di_lprx1_n, di_lprx1_p, di_lprxck_n, di_lprxck_p, ck_n, ck_p, d0_n, d0_p, d1_n, d1_p, d2_n, d2_p, d3_n, d3_p, lptxen_ln0, lptxen_ln1, lptxen_lnck, do_lptx0_n, do_lptx1_n, do_lptxck_n, do_lptx0_p, do_lptx1_p, do_lptxck_p, hsrx_en_ck, hsrx_en_d0, hsrx_en_d1, hsrx_odten_ck, hsrx_odten_d0, hsrx_odten_d1, lprx_en_ck, lprx_en_d0, lprx_en_d1, rx_drst_n);

output rx_clk_o;
output [7:0] d0ln_hsrxd;
output [7:0] d1ln_hsrxd;
output d0ln_hsrxd_vld;
output d1ln_hsrxd_vld;
output di_lprx0_n;
output di_lprx0_p;
output di_lprx1_n;
output di_lprx1_p;
output di_lprxck_n;
output di_lprxck_p;
inout ck_n;
inout ck_p;
inout d0_n;
inout d0_p;
inout d1_n;
inout d1_p;
inout d2_n;
inout d2_p;
inout d3_n;
inout d3_p;
input lptxen_ln0;
input lptxen_ln1;
input lptxen_lnck;
input do_lptx0_n;
input do_lptx1_n;
input do_lptxck_n;
input do_lptx0_p;
input do_lptx1_p;
input do_lptxck_p;
input hsrx_en_ck;
input hsrx_en_d0;
input hsrx_en_d1;
input hsrx_odten_ck;
input hsrx_odten_d0;
input hsrx_odten_d1;
input lprx_en_ck;
input lprx_en_d0;
input lprx_en_d1;
input rx_drst_n;

wire rx_clk_o_o;
wire tx_clk_o;
wire walign_dvld;
wire [7:0] d0ln_hsrxd_w;
wire [7:0] d1ln_hsrxd_w;
wire [15:0] d2ln_hsrxd;
wire [15:0] d3ln_hsrxd;
wire d2ln_hsrxd_vld;
wire d3ln_hsrxd_vld;
wire di_lprx2_n;
wire di_lprx2_p;
wire di_lprx3_n;
wire di_lprx3_p;
wire [7:0] mrdata;
wire alpedo_lane0;
wire alpedo_lane1;
wire alpedo_lane2;
wire alpedo_lane3;
wire alpedo_laneck;
wire d0ln_deskew_done;
wire d1ln_deskew_done;
wire d2ln_deskew_done;
wire d3ln_deskew_done;
wire d0ln_deskew_error;
wire d1ln_deskew_error;
wire d2ln_deskew_error;
wire d3ln_deskew_error;
wire gw_vcc;
wire gw_gnd;

assign walign_dvld = d0ln_deskew_done & d1ln_deskew_done & d2ln_deskew_done & d3ln_deskew_done;
assign rx_clk_o = rx_clk_o_o;
assign gw_vcc = 1'b1;
assign gw_gnd = 1'b0;

MIPI_DPHY mipi_dphy_inst (
    .RX_CLK_O(rx_clk_o_o),
    .TX_CLK_O(tx_clk_o),
    .D0LN_HSRXD({d0ln_hsrxd_w[7:0],d0ln_hsrxd[7:0]}),
    .D1LN_HSRXD({d1ln_hsrxd_w[7:0],d1ln_hsrxd[7:0]}),
    .D2LN_HSRXD(d2ln_hsrxd),
    .D3LN_HSRXD(d3ln_hsrxd),
    .D0LN_HSRXD_VLD(d0ln_hsrxd_vld),
    .D1LN_HSRXD_VLD(d1ln_hsrxd_vld),
    .D2LN_HSRXD_VLD(d2ln_hsrxd_vld),
    .D3LN_HSRXD_VLD(d3ln_hsrxd_vld),
    .DI_LPRX0_N(di_lprx0_n),
    .DI_LPRX0_P(di_lprx0_p),
    .DI_LPRX1_N(di_lprx1_n),
    .DI_LPRX1_P(di_lprx1_p),
    .DI_LPRX2_N(di_lprx2_n),
    .DI_LPRX2_P(di_lprx2_p),
    .DI_LPRX3_N(di_lprx3_n),
    .DI_LPRX3_P(di_lprx3_p),
    .DI_LPRXCK_N(di_lprxck_n),
    .DI_LPRXCK_P(di_lprxck_p),
    .MRDATA(mrdata),
    .ALPEDO_LANE0(alpedo_lane0),
    .ALPEDO_LANE1(alpedo_lane1),
    .ALPEDO_LANE2(alpedo_lane2),
    .ALPEDO_LANE3(alpedo_lane3),
    .ALPEDO_LANECK(alpedo_laneck),
    .D0LN_DESKEW_DONE(d0ln_deskew_done),
    .D1LN_DESKEW_DONE(d1ln_deskew_done),
    .D2LN_DESKEW_DONE(d2ln_deskew_done),
    .D3LN_DESKEW_DONE(d3ln_deskew_done),
    .D0LN_DESKEW_ERROR(d0ln_deskew_error),
    .D1LN_DESKEW_ERROR(d1ln_deskew_error),
    .D2LN_DESKEW_ERROR(d2ln_deskew_error),
    .D3LN_DESKEW_ERROR(d3ln_deskew_error),
    .CK_N(ck_n),
    .CK_P(ck_p),
    .D0_N(d0_n),
    .D0_P(d0_p),
    .D1_N(d1_n),
    .D1_P(d1_p),
    .D2_N(d2_n),
    .D2_P(d2_p),
    .D3_N(d3_n),
    .D3_P(d3_p),
    .HSRX_STOP(gw_gnd),
    .HSTXEN_LN0(gw_gnd),
    .HSTXEN_LN1(gw_gnd),
    .HSTXEN_LN2(gw_gnd),
    .HSTXEN_LN3(gw_gnd),
    .HSTXEN_LNCK(gw_gnd),
    .LPTXEN_LN0(lptxen_ln0),
    .LPTXEN_LN1(lptxen_ln1),
    .LPTXEN_LN2(gw_gnd),
    .LPTXEN_LN3(gw_gnd),
    .LPTXEN_LNCK(lptxen_lnck),
    .PWRON_RX(gw_vcc),
    .PWRON_TX(gw_vcc),
    .RESET(gw_gnd),
    .RX_CLK_1X(rx_clk_o_o),
    .TX_CLK_1X(tx_clk_o),
    .TXDPEN_LN0(gw_gnd),
    .TXDPEN_LN1(gw_gnd),
    .TXDPEN_LN2(gw_gnd),
    .TXDPEN_LN3(gw_gnd),
    .TXDPEN_LNCK(gw_gnd),
    .TXHCLK_EN(gw_gnd),
    .CKLN_HSTXD({gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc}),
    .D0LN_HSTXD({gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc}),
    .D1LN_HSTXD({gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc}),
    .D2LN_HSTXD({gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc}),
    .D3LN_HSTXD({gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc,gw_vcc}),
    .HSTXD_VLD(gw_gnd),
    .CK0(gw_gnd),
    .CK90(gw_gnd),
    .CK180(gw_gnd),
    .CK270(gw_gnd),
    .DO_LPTX0_N(do_lptx0_n),
    .DO_LPTX1_N(do_lptx1_n),
    .DO_LPTX2_N(gw_gnd),
    .DO_LPTX3_N(gw_gnd),
    .DO_LPTXCK_N(do_lptxck_n),
    .DO_LPTX0_P(do_lptx0_p),
    .DO_LPTX1_P(do_lptx1_p),
    .DO_LPTX2_P(gw_gnd),
    .DO_LPTX3_P(gw_gnd),
    .DO_LPTXCK_P(do_lptxck_p),
    .HSRX_EN_CK(hsrx_en_ck),
    .HSRX_EN_D0(hsrx_en_d0),
    .HSRX_EN_D1(hsrx_en_d1),
    .HSRX_EN_D2(gw_gnd),
    .HSRX_EN_D3(gw_gnd),
    .HSRX_ODTEN_CK(hsrx_odten_ck),
    .HSRX_ODTEN_D0(hsrx_odten_d0),
    .HSRX_ODTEN_D1(hsrx_odten_d1),
    .HSRX_ODTEN_D2(gw_gnd),
    .HSRX_ODTEN_D3(gw_gnd),
    .LPRX_EN_CK(lprx_en_ck),
    .LPRX_EN_D0(lprx_en_d0),
    .LPRX_EN_D1(lprx_en_d1),
    .LPRX_EN_D2(gw_gnd),
    .LPRX_EN_D3(gw_gnd),
    .RX_DRST_N(rx_drst_n),
    .TX_DRST_N(gw_gnd),
    .WALIGN_DVLD(walign_dvld),
    .MA_INC(gw_gnd),
    .MCLK(gw_gnd),
    .MOPCODE({gw_gnd,gw_gnd}),
    .MWDATA({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
    .D0LN_DESKEW_REQ(gw_gnd),
    .D1LN_DESKEW_REQ(gw_gnd),
    .D2LN_DESKEW_REQ(gw_gnd),
    .D3LN_DESKEW_REQ(gw_gnd),
    .HSRX_DLYDIR_LANE0(gw_gnd),
    .HSRX_DLYDIR_LANE1(gw_gnd),
    .HSRX_DLYDIR_LANE2(gw_gnd),
    .HSRX_DLYDIR_LANE3(gw_gnd),
    .HSRX_DLYDIR_LANECK(gw_gnd),
    .HSRX_DLYLDN_LANE0(gw_gnd),
    .HSRX_DLYLDN_LANE1(gw_gnd),
    .HSRX_DLYLDN_LANE2(gw_gnd),
    .HSRX_DLYLDN_LANE3(gw_gnd),
    .HSRX_DLYLDN_LANECK(gw_gnd),
    .HSRX_DLYMV_LANE0(gw_gnd),
    .HSRX_DLYMV_LANE1(gw_gnd),
    .HSRX_DLYMV_LANE2(gw_gnd),
    .HSRX_DLYMV_LANE3(gw_gnd),
    .HSRX_DLYMV_LANECK(gw_gnd),
    .ALP_EDEN_LANE0(gw_gnd),
    .ALP_EDEN_LANE1(gw_gnd),
    .ALP_EDEN_LANE2(gw_gnd),
    .ALP_EDEN_LANE3(gw_gnd),
    .ALP_EDEN_LANECK(gw_gnd),
    .ALPEN_LN0(gw_gnd),
    .ALPEN_LN1(gw_gnd),
    .ALPEN_LN2(gw_gnd),
    .ALPEN_LN3(gw_gnd),
    .ALPEN_LNCK(gw_gnd),
    .D0LN_HSRX_DREN(hsrx_en_d0),
    .D1LN_HSRX_DREN(hsrx_en_d1),
    .D2LN_HSRX_DREN(gw_gnd),
    .D3LN_HSRX_DREN(gw_gnd)
);

defparam mipi_dphy_inst.TX_PLLCLK = "NONE";
defparam mipi_dphy_inst.RX_ALIGN_BYTE = 8'b10111000;
defparam mipi_dphy_inst.RX_HS_8BIT_MODE = 1'b1;
defparam mipi_dphy_inst.RX_LANE_ALIGN_EN = 1'b0;
defparam mipi_dphy_inst.HSRX_EN = 1'b1;
defparam mipi_dphy_inst.HSRX_LANESEL = 4'b0011;
defparam mipi_dphy_inst.HSRX_LANESEL_CK = 1'b1;
defparam mipi_dphy_inst.LPTX_EN_LN0 = 1'b1;
defparam mipi_dphy_inst.LPTX_EN_LN1 = 1'b1;
defparam mipi_dphy_inst.LPTX_EN_LNCK = 1'b1;
defparam mipi_dphy_inst.RX_ONE_BYTE0_MATCH = 1'b0;
defparam mipi_dphy_inst.EQ_CS_LANE0 = 3'b100;
defparam mipi_dphy_inst.EQ_CS_LANE1 = 3'b100;
defparam mipi_dphy_inst.EQ_CS_LANE2 = 3'b100;
defparam mipi_dphy_inst.EQ_CS_LANE3 = 3'b100;
defparam mipi_dphy_inst.EQ_CS_LANECK = 3'b100;
defparam mipi_dphy_inst.EQ_RS_LANE0 = 3'b100;
defparam mipi_dphy_inst.EQ_RS_LANE1 = 3'b100;
defparam mipi_dphy_inst.EQ_RS_LANE2 = 3'b100;
defparam mipi_dphy_inst.EQ_RS_LANE3 = 3'b100;
defparam mipi_dphy_inst.EQ_RS_LANECK = 3'b100;
defparam mipi_dphy_inst.HSRX_EQ_EN_LANE0 = 1'b1;
defparam mipi_dphy_inst.HSRX_EQ_EN_LANE1 = 1'b1;
defparam mipi_dphy_inst.HSRX_EQ_EN_LANE2 = 1'b1;
defparam mipi_dphy_inst.HSRX_EQ_EN_LANE3 = 1'b1;
defparam mipi_dphy_inst.HSRX_EQ_EN_LANECK = 1'b1;
defparam mipi_dphy_inst.HSRX_ODT_EN = 1'b1;

endmodule //Gowin_MIPI_DPHY
