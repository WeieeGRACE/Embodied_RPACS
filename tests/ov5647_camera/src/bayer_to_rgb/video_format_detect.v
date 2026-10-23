//--------------------------------------------------------------------
// File name         : video_format_detect.v
// Module name       : video_format_detect
// Module Description: video format messure
// Created by        : Caojie
// -------------------------------------------------------------------
// Release history
// VERSION|      Date   |   AUTHOR     |        DESCRIPTION
// -------------------------------------------------------------------
//   1.0  | 23-Jul-2009 | Caojie       |      initial version
// -------------------------------------------------------------------
//   1.1  |  9-Mar-2010 | Caojie       |      model changed
// -------------------------------------------------------------------
//   2.0  |  8-Feb-2017 | Caojie       |  add more information
// -------------------------------------------------------------------

module video_format_detect
(
  input             rst_n         ,  //active low
  input             vd_clk        ,  //像素时钟，pixel_clock
  input             vd_vs         ,  //必须负极性
  input             vd_de         ,             
  output reg [15:0] vd_hres_reg      //horizontal resolution
);

//=============================================================================
  //用 vd_clk 采样2拍
  reg          vs_tmp1 = 1'b0;
  reg          vs_tmp2 = 1'b0;
  reg          de_tmp1 = 1'b0;
  reg          de_tmp2 = 1'b0;
  
  //-----------------------------------------------
  //水平分辨率
  reg  [15:0]  hcnt;  // horizontal counter by vd_clk
  reg  [15:0]  vd_hres;

//====================== vd_vs/vd_hs/vd_de/ ===============================
// 用 vd_clk 分别对vd_de信号延时采样
//=========================================================================
  always@(posedge vd_clk)
  begin
    vs_tmp1 <= vd_vs;
    vs_tmp2 <= vs_tmp1;
  end

  always@(posedge vd_clk)
  begin
    de_tmp1 <= vd_de;
    de_tmp2 <= de_tmp1;
  end

//==================== hcnt ================================================
// 统计水平分辨率，即每一行中de有效区域的时钟数
// 在de信号为高时，对时钟进行计数，统计时钟个数，
// 在第16行的hs信号下降沿，获取计数值
//==========================================================================
  always@(posedge vd_clk or negedge rst_n)
  begin
    if(rst_n == 1'b0)
      hcnt <= 16'd0;
    else if(de_tmp1 == 1'b0 && vd_de == 1'b1)  // de rising edge
      hcnt <= 16'd0;
    else if(de_tmp1 == 1'b1)
      hcnt <= hcnt + 1'b1;
    else
      hcnt <= hcnt;
  end
  
  always@(posedge vd_clk or negedge rst_n)
  begin
    if(rst_n == 1'b0) 
      vd_hres <= 16'd0;
    else if(de_tmp2 & !de_tmp1)  // hs falling edge 
      vd_hres <= hcnt;
    else
      vd_hres <= vd_hres;
  end

//==========================================================================
//======在每场VS之后输出频率，分辨率检测值========
//==========================================================================
//vd_clk时钟域
  always@(posedge vd_clk or negedge rst_n)
  begin
    if(rst_n == 1'b0) 
      begin
`ifdef SIM
        vd_hres_reg    <= 16'd1080;  
`else
        vd_hres_reg    <= 16'd0;  
`endif
      end
    else if(vs_tmp2 & !vs_tmp1)  // vs falling edge, and not halt
      begin 
        vd_hres_reg    <= vd_hres   ; 
     end
    else 
      begin   
        vd_hres_reg    <= vd_hres_reg   ;
      end
  end
//==========================================================================

endmodule

