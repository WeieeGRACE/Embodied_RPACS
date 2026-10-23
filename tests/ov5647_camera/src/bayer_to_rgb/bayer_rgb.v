//---------------------------------------------------------------------
// File name  : bayer_rgb.v
// Module name: bayer_rgb
// Created by : 
// ---------------------------------------------------------------------
// Release history
// VERSION |  Date       | AUTHOR     |    DESCRIPTION
// ---------------------------------------------------------------------
//  1.0    | 12-Jue-2017 |            |     initial 
// ---------------------------------------------------------------------


module bayer_rgb
(
  input             rst_n     ,
  input             pix_clk   ,
  input             vs_i      , //必须负极性
  input             de_i      ,
  input      [7:0]  data_i    ,
  output reg        vs_o      ,
  output reg        de_o      ,
  output reg [7:0]  r_o       ,
  output reg [7:0]  g_o       ,
  output reg [7:0]  b_o       
);

//==================================================================== 
wire [15:0] hres  ;

//---------------------------
//用 pix_clk 采样
reg          vs_tmp1   = 1'b0;
reg          vs_tmp2   = 1'b0;
reg          vs_tmp3   = 1'b0;
reg          de_tmp1   = 1'b0;
reg          de_tmp2   = 1'b0;
reg          de_tmp3   = 1'b0;
reg  [7:0]   data_tmp1 = 8'd0;
reg  [7:0]   data_tmp2 = 8'd0;
reg  [7:0]   data_tmp3 = 8'd0;
reg  [7:0]   data_tmp4 = 8'd0;

reg  [15:0] dehcnt;
reg  [15:0] devcnt;

reg  [15:0] dehcnt_d1;
reg  [15:0] devcnt_d1;

//---------------------
wire [7:0]  shift_data1  ;

reg  [7:0]  shift_data1_d1 = 8'd0  ;
reg  [7:0]  shift_data1_d2 = 8'd0  ;

//====================================================================================
video_format_detect video_format_detect_inst
(
  .rst_n       (rst_n  ),  //active low
  .vd_clk      (pix_clk),  //像素时钟，pixel_clock
  .vd_vs       (vs_i   ),  //必须负极性
  .vd_de       (de_i   ),             
  .vd_hres_reg (hres   )   //horizontal resolution
);

//====================================================================================
//----------------------------------------------------------------
// 用 pix_clk 分别对vd_de信号延时采样
//----------------------------------------------------------------
  always@(posedge pix_clk)
  begin
    vs_tmp1 <= vs_i;
    vs_tmp2 <= vs_tmp1;
    vs_tmp3 <= vs_tmp2;
  end

  always@(posedge pix_clk)
  begin
    de_tmp1 <= de_i;
    de_tmp2 <= de_tmp1;
    de_tmp3 <= de_tmp2;
  end
  
  always@(posedge pix_clk)
  begin
    data_tmp1 <= data_i;
    data_tmp2 <= data_tmp1;
    data_tmp3 <= data_tmp2;
    data_tmp4 <= data_tmp3;
  end
  
//----------------------------------------------------------------
// vcnt统计垂直分辨率，即一场图像中de有效的行数
// 在de信号下降沿计数，在vs信号上升沿计数器清零，从0,1,2,3,...
//----------------------------------------------------------------
  always@(posedge pix_clk or negedge rst_n)
  begin
    if(rst_n == 1'b0)
      devcnt <= 16'd0;
    else if(!vs_tmp2 & vs_tmp1)  // vs rising edge
      devcnt <= 16'd0;
    else if(de_tmp2 & !de_tmp1)  // de falling edge
      devcnt <= devcnt + 1'b1;
    else
      devcnt <= devcnt;
  end

//----------------------------------------------------------------
// hcnt统计水平分辨率，即每一行中de有效区域的时钟数
// 在de信号为高时，对时钟进行计数，统计时钟个数，获取计数值, 从0,1,2,...
//----------------------------------------------------------------
  always@(posedge pix_clk or negedge rst_n)
  begin
    if(rst_n == 1'b0)
      dehcnt <= 16'd0;
    else if(de_tmp1 == 1'b0 && de_i == 1'b1)  // de rising edge
      dehcnt <= 16'd0;
    else if(de_tmp2 == 1'b1)
      dehcnt <= dehcnt + 1'b1;
    else
      dehcnt <= dehcnt;
  end

//====================================================================================
//line buffer
shift_line # 
(
  .ADDR_WIDTH  (11),
  .DATA_WIDTH  (8 )
)
shift_line_inst0  
(  
  .aclr      (~rst_n           ),
  .clken     (de_tmp2          ),
  .clock     (pix_clk          ),
  .shiftin   (data_tmp2        ),
  .shiftout  (shift_data1      ),
  .delay_num (hres             )
);

//==========================================================================
//行场计数延时
always @(posedge pix_clk or negedge rst_n)
begin
  if(~rst_n)
    begin
      dehcnt_d1 <= 16'd0;
      devcnt_d1 <= 16'd0;
    end
  else
    begin
      dehcnt_d1 <= dehcnt;
      devcnt_d1 <= devcnt;
    end
end

//数据输出延时1拍
always @(posedge pix_clk or negedge rst_n)
begin
    if(~rst_n)  shift_data1_d1 <= 'b0;
    else if (1) shift_data1_d1 <= shift_data1;
    
    if(~rst_n)  shift_data1_d2 <= 'b0;
    else if (1) shift_data1_d2 <= shift_data1_d1;
end


//===========================================================================
//bayer RGB to RGB，以相邻4点为单位，颜色各分量取邻近值
// B G B G ...
// G R G R ...
// B G B G ...
// G R G R ...
//行场计数延时
always @(posedge pix_clk or negedge rst_n)
begin
  if(~rst_n)
    begin
      r_o <= 8'd0;
      g_o <= 8'd0;
      b_o <= 8'd0;
    end
  else if(devcnt_d1>=16'd1)
    begin
      case({devcnt_d1[0],dehcnt_d1[0]})
        2'b00:
          begin
            r_o <= shift_data1   ; //前一行
            g_o <= shift_data1_d1; //前一行
            b_o <= data_tmp3     ; //当前行
          end
        2'b01:
          begin
            r_o <= shift_data1_d1; //前一行
            g_o <= shift_data1_d2; //前一行
            b_o <= data_tmp4     ; //当前行
          end
        2'b10:
          begin
            r_o <= data_tmp2     ; //当前行
            g_o <= shift_data1   ; //前一行
            b_o <= shift_data1_d1; //前一行
          end                    
        2'b11:                   
          begin                  
            r_o <= data_tmp3     ; //当前行
            g_o <= shift_data1_d1; //前一行
            b_o <= shift_data1_d2; //前一行
          end
      endcase
    end
  else
    begin
      r_o <= 8'd0;
      g_o <= 8'd0;
      b_o <= 8'd0;
    end
end

always@(posedge pix_clk)
begin
  vs_o   <= vs_tmp3;
end

always@(posedge pix_clk)
begin
  if(devcnt_d1>=16'd1)
    de_o <= de_tmp3;
  else
    de_o <= 1'b0;
end
  
endmodule