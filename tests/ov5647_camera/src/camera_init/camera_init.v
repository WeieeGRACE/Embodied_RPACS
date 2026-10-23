/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : camera_init
// Description   : 摄像头初始化驱动模块
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module camera_init
(
  input      Clk,
  input      Rst_n,

  output reg Init_Done,

  output     i2c_sclk,
  inout      i2c_sdat
);

    parameter SYS_CLOCK       = 50_000_000;//系统时钟采用50MHz
    parameter SCL_CLOCK       = 200_000;   //SCL总线时钟采用400kHz
    // Keep board behavior unchanged by default; testbench may shorten both waits.
    parameter INIT_WAIT_CYCLES = 32'd16_000_000;
    parameter INITIAL_WRITE_DELAY_CYCLES = 32'h0050_0000;

	wire [7:0]device_id;
	wire addr_mode;
	wire [15:0]addr;
	wire [7:0]wrdata;
	
	reg [7:0]cnt;
	wire [23:0]lut;
	
	reg wrreg_req;
    wire [7:0]lut_size;
	wire [7:0]rddata;
	wire RW_Done;
	wire ack;
	reg [31:0] i2c_dly_cnt_max;
	
	assign device_id = 8'h6C;
	assign addr_mode = 1'b1;
	assign addr = lut[23:8];
	assign wrdata = lut[7:0];
	assign lut_size = 58;
	
	ov5647_init_table ov5647_init_table
	(
		.addr (cnt ),
		.clk  (Clk ),
		.q    (lut )
	);

	i2c_control #(
    .SYS_CLOCK (SYS_CLOCK ),
    .SCL_CLOCK (SCL_CLOCK )
    )i2c_control(
		.Clk         (Clk             ),
		.Rst_n       (Rst_n           ),
		.wrreg_req   (wrreg_req       ),
		.rdreg_req   (1'b0            ),
		.addr        (addr            ),
		.addr_mode   (addr_mode       ),
		.wrdata      (wrdata          ),
		.rddata      (rddata          ),
		.device_id   (device_id       ),
		.RW_Done     (RW_Done         ),
		.ack         (ack             ),
		.dly_cnt_max (i2c_dly_cnt_max ),
		.i2c_sclk    (i2c_sclk        ),
		.i2c_sdat    (i2c_sdat        )
	);

	wire Go;   //initial enable
	reg [31:0] delay_cnt;

	always@(posedge Clk or negedge Rst_n)
	if(~Rst_n)
		delay_cnt <= 32'd0;
	else if (delay_cnt >= INIT_WAIT_CYCLES)
		delay_cnt <= INIT_WAIT_CYCLES;
	else
		delay_cnt <= delay_cnt + 1'd1;

	//当延时时间到，开始使能初始化模块对OV5647的寄存器进行写入  
	assign Go = (delay_cnt == INIT_WAIT_CYCLES - 1'b1) ? 1'b1 : 1'b0;

	always@(posedge Clk or negedge Rst_n)
	if(~Rst_n)
		cnt <= 0;
	else if(Go) 
		cnt <= 0;
	else if(cnt < lut_size)begin
		if(RW_Done && (!ack))
			cnt <= cnt + 1'b1;
		else
			cnt <= cnt;
		end
	else
		cnt <= 0;
		
	always@(posedge Clk or negedge Rst_n)
	if(~Rst_n)
		Init_Done <= 0;
	else if(Go) 
		Init_Done <= 0;
	else if(cnt == lut_size)
		Init_Done <= 1;		

	reg [1:0]state;

	always@(posedge Clk or negedge Rst_n)
	if(~Rst_n)begin
		state <= 0;
		wrreg_req <= 1'b0;
		i2c_dly_cnt_max <= 32'd0;
	end
	else if(cnt < lut_size)begin
		case(state)
		0:  begin
                if(Go)
                  state <= 1;
                else
                  state <= 0;
            end
		1:
			begin
                wrreg_req <= 1'b1;
                state <= 2;
                if(cnt == 0)
	                    i2c_dly_cnt_max <= INITIAL_WRITE_DELAY_CYCLES; // preserve the original first-write delay by default
                else
                    i2c_dly_cnt_max <= 32'h20;
			end
		2:
			begin
                wrreg_req <= 1'b0;
                if(RW_Done)
                    state <= 1;
                else
                    state <= 2;
			end
		default:state <= 0;
		endcase
	end
	else
		state <= 0;
		
endmodule
