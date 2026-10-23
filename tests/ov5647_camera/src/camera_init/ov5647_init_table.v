module ov5647_init_table
(
	clk,
	addr,
	q
);

  parameter ADDR_WIDTH   = 8;
  parameter DATA_WIDTH   = 24;
  
  input clk;
  input [(ADDR_WIDTH-1):0] addr;
  output reg [(DATA_WIDTH-1):0] q;
  
  reg [23:0] rom[57:0];
  
  always@(*) begin
  		rom[0 ] = 24'h0100_00; //0x0100[0] software standby                   //////////////// Global Initial
		rom[1 ] = 24'h0103_01; //0x0103[0] software reset, 1 reset     // then need to wait at least 5ms
		rom[2 ] = 24'h3016_08; //[3] mipi pad enable
		rom[3 ] = 24'h3018_44; //0x3018[7:5] MIPI lane, 0x3018[2] MIPI enable
		rom[4 ] = 24'h4800_04; //MIPI CTRL 00 [4] 1:send line short packet   0:not send
		rom[5 ] = 24'h3106_05; //SRB CTRL  PLL clock divider     //05             
		rom[6 ] = 24'h0100_01; //0x0100[0] wake up from software standby      //////////////// End Global Initial 
		rom[7 ] = 24'h0100_00; //0x0100[0] software standby                   //////////////// Set 1080p:1920*1080@30fps
		rom[8 ] = 24'h3034_08; //[3:0] 8:8 bit mode  A:10 bit mode 
		rom[9 ] = 24'h3035_11; //0x3035[7:4] pll_div, 0x3035[3:0] divider_mipi 
		rom[10] = 24'h3036_50; //0x3036[7:0] pll_multiplier //x64 //x50
		rom[11] = 24'h303c_11; //PLL ctrol 
		rom[12] = 24'h3800_01; //0x3800[3:0] = dvp_hstart[11:8]
		rom[13] = 24'h3801_5c; //0x3801[7:0] = dvp_hstart[7:0]   //348
		rom[14] = 24'h3802_01; //0x3802[3:0] = dvp_vstart[11:8]
		rom[15] = 24'h3803_b2; //0x3803[7:0] = dvp_vstart[7:0]   //434
		rom[16] = 24'h3804_08; //0x3804[3:0] = dvp_hend[11:8]
		rom[17] = 24'h3805_e3; //0x3805[7:0] = dvp_hend[7:0]     //2275
		rom[18] = 24'h3806_05; //0x3806[3:0] = dvp_vend[11:8]
		rom[19] = 24'h3807_f1; //0x3807[7:0] = dvp_vend[7:0]     //1521
		rom[20] = 24'h3808_07; //0x3808[3:0] = dvp_hout[11:8]
		rom[21] = 24'h3809_80; //0x3809[7:0] = dvp_hout[7:0]     //1920
		rom[22] = 24'h380a_04; //0x380a[3:0] = dvp_vout[11:8]
		rom[23] = 24'h380b_38; //0x380b[7:0] = dvp_vout[7:0]     //1080
		rom[24] = 24'h380c_09; //0x380c[4:0] = h_total[12:8]
		rom[25] = 24'h380d_70; //0x380d[7:0] = h_total[7:0]   //70//2416    //4e//2382
		rom[26] = 24'h380e_04; //0x380e[4:0] = v_total[12:8]
		rom[27] = 24'h380f_50; //0x380f[7:0] = v_total[7:0]   //50//1104    //60//1120
		rom[28] = 24'h3814_11; //h subsample inc
		rom[29] = 24'h3815_11; //v subsample inc  
		rom[30] = 24'h3820_06; //TC_REG20 [2:1] filp
		rom[31] = 24'h3821_00; //TC_REG21 [2:1] mirror 
		rom[32] = 24'h503d_00; //[7] testpattern_en 0:disable, 1:enable //80 //82 //92
		rom[33] = 24'h3612_5b; //0x3600~0x3637 Analog control registers
		rom[34] = 24'h3618_04; //0x3600~0x3637 Analog control registers
		rom[35] = 24'h3708_64; //0x3700~0x373a Analog control registers //64
		rom[36] = 24'h3709_12; //0x3700~0x373a Analog control registers
		rom[37] = 24'h370c_03; //0x3700~0x373a Analog control registers ////03 or 0f
		rom[38] = 24'h3630_2e; //0x3600~0x3637 Analog control registers
		rom[39] = 24'h3632_e2; //0x3600~0x3637 Analog control registers
		rom[40] = 24'h3633_23; //0x3600~0x3637 Analog control registers
		rom[41] = 24'h3634_44; //0x3600~0x3637 Analog control registers
        //! Adj. MIPI parameters
        rom[42] = 24'h4824_00;
        rom[43] = 24'h4825_32;  // LPX_P_MIN
        rom[44] = 24'h4830_00;  // UI_LPX_P_MIN
        rom[45] = 24'h4826_00;
        rom[46] = 24'h4827_32;  // HX_PREPAREE_MIN
        rom[47] = 24'h4831_04;  // UI_HX_PREPAREE_MIN
        rom[48] = 24'h4818_00;
        rom[49] = 24'h4819_96;  // HS_ZERO_MIN
        rom[50] = 24'h482A_20;  // UI_HS_ZERO_MIN
        rom[51] = 24'h481A_00;
        rom[52] = 24'h481B_3C;  // HS_TRAIL_MIN
        rom[53] = 24'h482B_04;  // UI_HS_TRAIL_MIN
        rom[54] = 24'h4828_00;
        rom[55] = 24'h4829_64;  // HS_EXIT_MIN
        rom[56] = 24'h4832_04;  // UI_HS_EXIT_MIN
        rom[57] = 24'h0100_01;  //0x0100[0] wake up from software standby      //////////////// End Set 1080p
  end
  
  
  always @ (posedge clk)
  begin
    q <= rom[addr];
  end
  
  
 endmodule