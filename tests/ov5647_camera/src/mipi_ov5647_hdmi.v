
`define     HS_EN_DLY   10
module mipi_ov5647_hdmi
(
    input       clk_50M ,
    input       Rst_n   ,

    //OV5647 MIPI
    inout       ck_n    ,
    inout       ck_p    ,
    inout       d0_n    ,
    inout       d0_p    ,
    inout       d1_n    ,
    inout       d1_p    ,
    output      O_CSI_SCL,
    inout       IO_CSI_SDA,  
    output      camera_pwdn,


    //DDR
	output	   [13:0] O_ddr_addr      ,
	output	   [2:0]  O_ddr_ba        ,
	output 	   	      O_ddr_cs_n      ,
	output 	   	      O_ddr_ras_n     ,
	output 	   	      O_ddr_cas_n     ,
	output 	   	      O_ddr_we_n      ,
	output 	   	      O_ddr_clk       ,
	output 	   	      O_ddr_clk_n     ,
	output 	   	      O_ddr_cke       ,
	output 	   	      O_ddr_odt       ,
	output 	   	      O_ddr_reset_n   ,
	output 	   [1:0]  O_ddr_dqm       ,
	inout  	   [15:0] IO_ddr_dq       ,
	inout  	   [1:0]  IO_ddr_dqs      ,
	inout  	   [1:0]  IO_ddr_dqs_n    ,

    //HDMI
	output	   		  O_tmds_clk_p      ,
	output	   		  O_tmds_clk_n      ,
	output	   [2:0]  O_tmds_data_p     ,//{r,g,b}
	output	   [2:0]  O_tmds_data_n     ,

    output [1:0]      led
);

    localparam  DT_RAW8 = 6'h2A;     // CSI-2 RAW-8, not RGB888
    //Gowin_PLL
    wire byte_clk;
    wire lvds_pclk;

    //Gowin_MIPI_DPHY
    wire [7:0]     d0ln_hsrxd;
    wire [7:0]     d1ln_hsrxd;
    wire [1:0]     hsrxd_vld;
    wire [1:0]     lp_data0;
    wire [1:0]     lp_data1;

    reg             odt_en_msk;
    reg  [1:0]      lp0_reg_0   =2'b11;
    reg  [1:0]      lp0_reg_1   =2'b11;
    reg  [1:0]      lp1_reg_0   =2'b11;
    wire            from0to3    = (lp0_reg_1==0)&(lp0_reg_0==3);
    wire            from1to0    = (lp0_reg_1==1)&(lp0_reg_0==0);
    wire            from1to2    = (lp0_reg_1==1)&(lp0_reg_0==2);
    wire            from1to3    = (lp0_reg_1==1)&(lp0_reg_0==3);
    wire            from3to1    = (lp0_reg_1==3)&(lp0_reg_0==1);
    wire            fromXto3    = (lp0_reg_1!=3)&(lp0_reg_0==3);
    wire            from1toX    = (lp0_reg_1==1)&(lp0_reg_0!=1);
    reg             reg3to1;
    reg  [5:0]      hsrx_cnt;
    reg             rx_drst_n   =1'b1;
    reg  [1:0]      odt_en;
    reg             hsrx_en_msk;
    reg             byte_ready;
    reg  [7:0]      byte_d0;
    reg  [7:0]      byte_d1;

    wire            sp_en;
    wire            lp_en;
    wire            lp_av_en;
    wire            ecc_ok;
    wire [7:0]      ecc;
    wire [15:0]     wc;
    wire [ 1:0]     vc;
    wire [ 5:0]     dt;
    wire [ 1:0]     payload_dv;
    wire [15:0]     payload;

    reg             sp_en_r;
    reg             lp_av_en_r;
    reg  [ 1:0]     payload_dv_r;
    reg  [15:0]     payload_r;

    wire            pixel_clk;
    wire            csi_clk     = pixel_clk;
    wire            csi_fv      ;
    wire            csi_lv      ;
    wire [ 7:0]     csi_pixel   ;

    wire            pxl_vs      ;
    wire            pxl_de      ;
    wire [ 7:0]     pxl_r       ;
    wire [ 7:0]     pxl_g       ;
    wire [ 7:0]     pxl_b       ;


    wire camera_init_done;
    assign camera_pwdn = 1'd0;
    //MIPI摄像头OV5647初始化
    camera_init camera_init
    (
      .Clk(clk_50M),
      .Rst_n(Rst_n),
      .Init_Done(camera_init_done),
      .i2c_sclk(O_CSI_SCL),
      .i2c_sdat(IO_CSI_SDA)
    );


    assign led[0] = camera_init_done;
    Gowin_PLL Gowin_PLL(
        .clkout0(lvds_pclk), //output clkout0
        .clkin(byte_clk), //input clkin
        .reset(~Rst_n) //input reset
    );

    assign  pixel_clk = lvds_pclk;
    
    Gowin_MIPI_DPHY Gowin_MIPI_DPHY(
        .rx_clk_o(byte_clk), //output rx_clk_o
        .d0ln_hsrxd(d0ln_hsrxd), //output [7:0] d0ln_hsrxd
        .d1ln_hsrxd(d1ln_hsrxd), //output [7:0] d1ln_hsrxd
        .d0ln_hsrxd_vld(hsrxd_vld[0]), //output d0ln_hsrxd_vld
        .d1ln_hsrxd_vld(hsrxd_vld[1]), //output d1ln_hsrxd_vld
        .di_lprx0_n(lp_data0[0]), //output di_lprx0_n
        .di_lprx0_p(lp_data0[1]), //output di_lprx0_p
        .di_lprx1_n(lp_data1[0]), //output di_lprx1_n
        .di_lprx1_p(lp_data1[1]), //output di_lprx1_p
        .di_lprxck_n(), //output di_lprxck_n
        .di_lprxck_p(), //output di_lprxck_p
        .ck_n(ck_n), //inout ck_n
        .ck_p(ck_p), //inout ck_p
        .d0_n(d0_n), //inout d0_n
        .d0_p(d0_p), //inout d0_p
        .d1_n(d1_n), //inout d1_n
        .d1_p(d1_p), //inout d1_p
        .d2_n(), //inout d2_n
        .d2_p(), //inout d2_p
        .d3_n(), //inout d3_n
        .d3_p(), //inout d3_p
        .lptxen_ln0(1'b0), //input lptxen_ln0
        .lptxen_ln1(1'b0), //input lptxen_ln1
        .lptxen_lnck(1'b0), //input lptxen_lnck
        .do_lptx0_n(1'b1), //input do_lptx0_n
        .do_lptx1_n(1'b1), //input do_lptx1_n
        .do_lptxck_n(1'b1), //input do_lptxck_n
        .do_lptx0_p(1'b1), //input do_lptx0_p
        .do_lptx1_p(1'b1), //input do_lptx1_p
        .do_lptxck_p(1'b1), //input do_lptxck_p
        .hsrx_en_ck(1'b1), //input hsrx_en_ck
        .hsrx_en_d0(1'b1), //input hsrx_en_d0
        .hsrx_en_d1(1'b1), //input hsrx_en_d1
        .hsrx_odten_ck(1'b1), //input hsrx_odten_ck
        .hsrx_odten_d0(odt_en[0]), //input hsrx_odten_d0
        .hsrx_odten_d1(odt_en[1]), //input hsrx_odten_d1
        .lprx_en_ck(1'b1), //input lprx_en_ck
        .lprx_en_d0(1'b1), //input lprx_en_d0
        .lprx_en_d1(1'b1), //input lprx_en_d1
        .rx_drst_n(rx_drst_n) //input rx_drst_n
    );

    always @(posedge byte_clk or negedge Rst_n) begin
        if (~Rst_n)             odt_en_msk  <= 'b0;
        else if (odt_en_msk)    odt_en_msk  <= !(from1to2|from1to3|fromXto3);
        else if (1)             odt_en_msk  <= from3to1;

        if (~Rst_n)             reg3to1     <= 1'b0;
        else if (1)             reg3to1     <= reg3to1 ? !from1toX : from3to1;

        if (~Rst_n)             hsrx_cnt    <= 'b0;
        else if (odt_en)        hsrx_cnt    <= `HS_EN_DLY;
        else if (hsrx_cnt>0)    hsrx_cnt    <= hsrx_cnt - 2'd1;
    end

    always @(posedge byte_clk) begin
        lp0_reg_0   <= lp_data0;
        lp1_reg_0   <= lp_data1;
        lp0_reg_1   <= lp0_reg_0;

        rx_drst_n   <= !(reg3to1&from1to0);

        odt_en      <= { odt_en_msk&(lp0_reg_0==0)
                       , odt_en_msk&(lp1_reg_0==0)
                       };

        hsrx_en_msk <= (hsrx_cnt>0);
        byte_ready  <= hsrx_en_msk&hsrxd_vld[0];
        byte_d0     <= d0ln_hsrxd[7:0];
        byte_d1     <= d1ln_hsrxd[7:0];
    end

	MIPI_DSI_CSI2_RX_Top MIPI_DSI_CSI2_RX_Top(
		.I_RSTN(Rst_n), //input I_RSTN
		.I_BYTE_CLK(byte_clk), //input I_BYTE_CLK
		.I_REF_DT(DT_RAW8), // CSI-2 RAW-8 data type
		.I_READY(byte_ready), //input I_READY
		.I_DATA0(byte_d0), //input [7:0] I_DATA0
		.I_DATA1(byte_d1), //input [7:0] I_DATA1
		.O_SP_EN(sp_en), //output O_SP_EN
		.O_LP_EN(lp_en), //output O_LP_EN
		.O_LP_AV_EN(lp_av_en), //output O_LP_AV_EN
		.O_ECC_OK(ecc_ok), //output O_ECC_OK
		.O_ECC(ecc), //output [7:0] O_ECC
		.O_WC(wc), //output [15:0] O_WC
		.O_VC(vc), //output [1:0] O_VC
		.O_DT(dt), //output [5:0] O_DT
		.O_PAYLOAD(payload), //output [15:0] O_PAYLOAD
		.O_PAYLOAD_DV(payload_dv) //output [1:0] O_PAYLOAD_DV
	);

    always @(posedge byte_clk) begin
        sp_en_r         <= ecc_ok & sp_en & ({wc,ecc,vc,dt}!=0);
        lp_av_en_r      <= ecc_ok & lp_av_en;
        payload_dv_r    <= payload_dv;
        payload_r       <= payload;
    end

	MIPI_Byte_to_Pixel_Converter_Top MIPI_Byte_to_Pixel_Converter_Top(
		.I_RSTN(Rst_n), //input I_RSTN
		.I_BYTE_CLK(byte_clk), //input I_BYTE_CLK
		.I_PIXEL_CLK(csi_clk), //input I_PIXEL_CLK
		.I_SP_EN(sp_en_r), //input I_SP_EN
		.I_LP_AV_EN(lp_av_en_r), //input I_LP_AV_EN
		.I_DT(dt), //input [5:0] I_DT
		.I_WC(wc), //input [15:0] I_WC
		.I_PAYLOAD_DV(payload_dv_r), //input [1:0] I_PAYLOAD_DV
		.I_PAYLOAD(payload_r), //input [15:0] I_PAYLOAD
		.O_FV(csi_fv), //output O_FV
		.O_LV(csi_lv), //output O_LV
		.O_PIXEL(csi_pixel) //output [7:0] O_PIXEL
	);

    bayer_rgb u_bayer_rgb(  
        .rst_n          ( Rst_n             ),
        .pix_clk        ( csi_clk           ),
        .vs_i           ( csi_fv            ),
        .de_i           ( csi_lv            ),
        .data_i         ( csi_pixel         ),

        .vs_o           ( pxl_vs            ),
        .de_o           ( pxl_de            ),
        .r_o            ( pxl_r             ),
        .g_o            ( pxl_g             ),
        .b_o            ( pxl_b             )
    );

    wire init_calib_complete;
    wire dma_clk;
    wire ch0_vfb_clk_in;
    wire ch0_vfb_vs_in;
    wire ch0_vfb_de_in;
    wire[15:0] ch0_vfb_data_in;
    wire pix_clk;
    wire syn_off0_vs;
    wire syn_off0_re;
    wire off0_syn_de;
    wire [15:0] off0_syn_data;
    wire vout0fifo_empty;
    wire cmd_ready;
    wire [2:0] cmd;
    wire cmd_en;
    wire [5:0] app_burst_number;
    wire [27:0] addr;
    wire wr_data_rdy;
    wire wr_data_en;
    wire wr_data_end;
    wire [127:0] wr_data;
    wire [15:0]wr_data_mask;
    wire rd_data_valid;
    wire rd_data_end;
    wire [127:0] rd_data;

    assign ch0_vfb_clk_in = csi_clk;
    assign ch0_vfb_vs_in = pxl_vs;
    assign ch0_vfb_de_in = pxl_de;
    assign ch0_vfb_data_in = {pxl_r[7:3],pxl_g[7:2],pxl_b[7:3]};
    assign led[1] = init_calib_complete;

	Video_Frame_Buffer_Top Video_Frame_Buffer_Top(
		.I_rst_n(init_calib_complete), //input I_rst_n
		.I_dma_clk(dma_clk), //input I_dma_clk
		.I_wr_halt(1'd0), //input [0:0] I_wr_halt
		.I_rd_halt(1'd0), //input [0:0] I_rd_halt
		.I_vin0_clk(ch0_vfb_clk_in), //input I_vin0_clk
		.I_vin0_vs_n(ch0_vfb_vs_in), //input I_vin0_vs_n
		.I_vin0_de(ch0_vfb_de_in), //input I_vin0_de
		.I_vin0_data(ch0_vfb_data_in), //input [15:0] I_vin0_data
		.O_vin0_fifo_full(), //output O_vin0_fifo_full
		.I_vout0_clk(pix_clk), //input I_vout0_clk
		.I_vout0_vs_n(~syn_off0_vs), //input I_vout0_vs_n
		.I_vout0_de(syn_off0_re), //input I_vout0_de
		.O_vout0_den(off0_syn_de), //output O_vout0_den
		.O_vout0_data(off0_syn_data), //output [15:0] O_vout0_data
		.O_vout0_fifo_empty(vout0fifo_empty), //output O_vout0_fifo_empty
		.I_cmd_ready(cmd_ready), //input I_cmd_ready
		.O_cmd(cmd), //output [2:0] O_cmd
		.O_cmd_en(cmd_en), //output O_cmd_en
		.O_addr(addr), //output [27:0] O_addr
		.I_wr_data_rdy(wr_data_rdy), //input I_wr_data_rdy
		.O_wr_data_en(wr_data_en), //output O_wr_data_en
		.O_wr_data_end(wr_data_end), //output O_wr_data_end
		.O_wr_data(wr_data), //output [127:0] O_wr_data
		.O_wr_data_mask(wr_data_mask), //output [15:0] O_wr_data_mask
		.I_rd_data_valid(rd_data_valid), //input I_rd_data_valid
		.I_rd_data_end(rd_data_end), //input I_rd_data_end
		.I_rd_data(rd_data), //input [127:0] I_rd_data
		.I_init_calib_complete(init_calib_complete) //input I_init_calib_complete
	);

    wire ddr_pll_lock;
    wire clk_200M;
    ddr_pll ddr_pll(
        .lock(ddr_pll_lock), //output lock
        .clkout0(clk_200M), //output clkout0
        .clkin(clk_50M) //input clkin
    );


	DDR3_Memory_Interface_Top DDR3_Memory_Interface_Top(
		.clk(clk_50M), //input clk
		.pll_stop(), //output pll_stop
		.memory_clk(clk_200M), //input memory_clk
		.pll_lock(ddr_pll_lock), //input pll_lock
		.rst_n(Rst_n), //input rst_n
		.cmd_ready(cmd_ready), //output cmd_ready
		.cmd(cmd), //input [2:0] cmd
		.cmd_en(cmd_en), //input cmd_en
		.addr(addr), //input [27:0] addr
		.wr_data_rdy(wr_data_rdy), //output wr_data_rdy
		.wr_data(wr_data), //input [127:0] wr_data
		.wr_data_en(wr_data_en), //input wr_data_en
		.wr_data_end(wr_data_end), //input wr_data_end
		.wr_data_mask(wr_data_mask), //input [15:0] wr_data_mask
		.rd_data(rd_data), //output [127:0] rd_data
		.rd_data_valid(rd_data_valid), //output rd_data_valid
		.rd_data_end(rd_data_end), //output rd_data_end
		.sr_req(1'b0), //input sr_req
		.ref_req(1'b0), //input ref_req
		.sr_ack(), //output sr_ack
		.ref_ack(), //output ref_ack
		.init_calib_complete(init_calib_complete), //output init_calib_complete
		.clk_out(dma_clk), //output clk_out
		.ddr_rst(), //output ddr_rst
		.burst(1'b1), //input burst
		.O_ddr_addr(O_ddr_addr), //output [13:0] O_ddr_addr
		.O_ddr_ba(O_ddr_ba), //output [2:0] O_ddr_ba
		.O_ddr_cs_n(O_ddr_cs_n), //output O_ddr_cs_n
		.O_ddr_ras_n(O_ddr_ras_n), //output O_ddr_ras_n
		.O_ddr_cas_n(O_ddr_cas_n), //output O_ddr_cas_n
		.O_ddr_we_n(O_ddr_we_n), //output O_ddr_we_n
		.O_ddr_clk(O_ddr_clk), //output O_ddr_clk
		.O_ddr_clk_n(O_ddr_clk_n), //output O_ddr_clk_n
		.O_ddr_cke(O_ddr_cke), //output O_ddr_cke
		.O_ddr_odt(O_ddr_odt), //output O_ddr_odt
		.O_ddr_reset_n(O_ddr_reset_n), //output O_ddr_reset_n
		.O_ddr_dqm(O_ddr_dqm), //output [1:0] O_ddr_dqm
		.IO_ddr_dq(IO_ddr_dq), //inout [15:0] IO_ddr_dq
		.IO_ddr_dqs(IO_ddr_dqs), //inout [1:0] IO_ddr_dqs
		.IO_ddr_dqs_n(IO_ddr_dqs_n) //inout [1:0] IO_ddr_dqs_n
	);

    //---------------------------------------------------
    //输出同步时序模块
    wire out_de;
    syn_gen syn_gen_inst
    (                                   
        .I_pxl_clk   (pix_clk       ),//40MHz      //65MHz      //74.25MHz    //148.5
        .I_rst_n     (Rst_n           ),//800x600    //1024x768   //1280x720    //1920x1080    
        .I_h_total   (16'd2200        ),// 16'd1056  // 16'd1344  // 16'd1650   // 16'd2200  
        .I_h_sync    (16'd44          ),// 16'd128   // 16'd136   // 16'd40     // 16'd44   
        .I_h_bporch  (16'd148         ),// 16'd88    // 16'd160   // 16'd220    // 16'd148   
        .I_h_res     (16'd1920       ),// 16'd800   // 16'd1024  // 16'd1280   // 16'd1920  
        .I_v_total   (16'd1125         ),// 16'd628   // 16'd806   // 16'd750    // 16'd1125   
        .I_v_sync    (16'd5           ),// 16'd4     // 16'd6     // 16'd5      // 16'd5      
        .I_v_bporch  (16'd36          ),// 16'd23    // 16'd29    // 16'd20     // 16'd36      
        .I_v_res     (16'd1080        ),// 16'd600   // 16'd768   // 16'd720    // 16'd1080   
        .I_rd_hres   (16'd1920       ),
        .I_rd_vres   (16'd1080         ),
        .I_hs_pol    (1'b1            ),//HS polarity , 0:负极性，1：正极性
        .I_vs_pol    (1'b1            ),//VS polarity , 0:负极性，1：正极性
        .O_rden      (syn_off0_re     ),
        .O_de        (out_de          ),   
        .O_hs        (syn_off0_hs     ),
        .O_vs        (syn_off0_vs     )
    );

    //rgb data
    wire syn_off0_hs;
    wire        rgb_vs     ;
    wire        rgb_hs     ;
    wire        rgb_de     ;
    wire [23:0] rgb_data   ;  

	assign rgb_data    = {off0_syn_data[15:11],3'd0,off0_syn_data[10:5],2'd0,off0_syn_data[4:0],3'd0};//{r,g,b}
	assign rgb_vs      = syn_off0_vs;//syn_off0_vs;
	assign rgb_hs      = syn_off0_hs;//syn_off0_hs;
	assign rgb_de 	   = out_de;//off0_syn_de;


    wire TMDS_lock;
    wire serial_clk;
    wire hdmi4_rst_n;

    TMDS_PLL TMDS_PLL(
        .lock(TMDS_lock), //output lock
        .clkout0(serial_clk), //output clkout0
        .clkin(clk_50M) //input clkin
    );

    assign hdmi4_rst_n = Rst_n&TMDS_lock;

    CLKDIV u_clkdiv
    (.RESETN(hdmi4_rst_n)
    ,.HCLKIN(serial_clk) //clk  x5
    ,.CLKOUT(pix_clk)    //clk  x1
    ,.CALIB(1'b1)
    );
    defparam u_clkdiv.DIV_MODE="5";


	DVI_TX_Top DVI_TX_Top(
		.I_rst_n(hdmi4_rst_n), //input I_rst_n
		.I_serial_clk(serial_clk), //input I_serial_clk
		.I_rgb_clk(pix_clk), //input I_rgb_clk
		.I_rgb_vs(rgb_vs), //input I_rgb_vs
		.I_rgb_hs(rgb_hs), //input I_rgb_hs
		.I_rgb_de(rgb_de), //input I_rgb_de
		.I_rgb_r(rgb_data[23:16]), //input [7:0] I_rgb_r
		.I_rgb_g(rgb_data[15: 8]), //input [7:0] I_rgb_g
		.I_rgb_b(rgb_data[ 7: 0]), //input [7:0] I_rgb_b
		.O_tmds_clk_p(O_tmds_clk_p), //output O_tmds_clk_p
		.O_tmds_clk_n(O_tmds_clk_n), //output O_tmds_clk_n
		.O_tmds_data_p(O_tmds_data_p), //output [2:0] O_tmds_data_p
		.O_tmds_data_n(O_tmds_data_n) //output [2:0] O_tmds_data_n
	);


endmodule
