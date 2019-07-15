//============================================================================
//  Acorn System1 port to MiSTer
//  2019 Dave Wood (oldgit)
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [44:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output  [7:0] VIDEO_ARX,
	output  [7:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output  [1:0] VGA_SL,

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)
	input         TAPE_IN,

	// SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..5 - USR1..USR4
	// Set USER_OUT to 1 to read from USER_IN.
	input   [5:0] USER_IN,
	output  [5:0] USER_OUT,

	input         OSD_STATUS
);

assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;
 
//assign LED_USER  = ioctl_download | (vsd_sel & sd_act);
//assign LED_DISK  = {1'b1,~vsd_sel & sd_act};
assign LED_POWER = 0;

assign VIDEO_ARX = status[1] ? 8'd16 : 8'd4;
assign VIDEO_ARY = status[1] ? 8'd9  : 8'd3; 

wire [1:0] scale = status[3:2];

`include "build_id.v" 
parameter CONF_STR = {
	"System1;;",
	"-;",
//	"S,VHD;",
//	"OC,Autostart,Yes,No;",
	"-;",
	"O1,Aspect ratio,4:3,16:9;",
	"O23,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%;",
//	"OA,Mouse as Joystick,Yes,No;",
//	"OB,Swap Joysticks,No,Yes;",
	"-;",
//	"O4,Audio,Atom,SID;",
//	"O56,Co-Processor,None,MOS65C02;",
//	"O79,Default video mode,0,1,2,3,4,5,6,7;",
	"-;",
	"R0,Reset;",
	"JA,Fire;",
	"V,v",`BUILD_DATE
};

/////////////////  CLOCKS  ////////////////////////

wire clk_sys;
wire clk_16;
wire clk_25;
wire clk_32;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_16),
	.outclk_1(clk_25),
	.outclk_2(clk_32),
	.outclk_3(clk_sys)
);


/////////////////  HPS  ///////////////////////////

wire [31:0] status;
wire  [1:0] buttons;

wire [15:0] joy1, joy2;
wire  [7:0] joy1_x,joy1_y,joy2_x,joy2_y;

wire 			ps2_clk, ps2_data, mse_clk, mse_dat;

wire        ioctl_download;
wire  [7:0] ioctl_index;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        forced_scandoubler;

wire [31:0] sd_lba;
wire        sd_rd;
wire        sd_wr;
wire        sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din;
wire        sd_buff_wr;
wire        img_mounted;
wire        img_readonly;
wire [63:0] img_size;
wire        sd_ack_conf;
wire [24:0] ps2_mouse;
wire [10:0] ps2_key;
wire [64:0] RTC;

hps_io #(.STRLEN($size(CONF_STR)>>3)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.conf_str(CONF_STR),

	.buttons(buttons),
	.status(status),
	.forced_scandoubler(forced_scandoubler),

	.RTC(RTC),

	.ps2_kbd_clk_out(ps2_clk),
	.ps2_kbd_data_out(ps2_data),
	.ps2_mouse_clk_out(mse_clk),
	.ps2_mouse_data_out(mse_dat),
	
	.ps2_mouse(ps2_mouse),
	.ps2_key(ps2_key),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),

	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_ack_conf(sd_ack_conf),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),

	.joystick_0(joy1),
	.joystick_1(joy2),
	.joystick_analog_0({joy1_y,joy1_x}),
	.joystick_analog_1({joy2_y,joy2_x})
);
/*
	wire       pressed = ps2_key[9];
	wire [8:0] code    = ps2_key[8:0];
always @(posedge clk_32) begin
	reg old_state;
	old_state <= ps2_key[10];
	
	if(old_state != ps2_key[10]) begin
		casex(code)
			'hX75: tswU           	<= pressed; // up
			'hX72: tswD		        	<= pressed; // down
			'h045: tsw0      			<= pressed; // 0
			'h016: tsw1       		<= pressed; // 1
			'h01e: tsw2   				<= pressed; // 2
			'h026: tsw3  				<= pressed; // 3
			'h025: tsw4   				<= pressed; // 4
			'h02e: tsw5   				<= pressed; // 5
			'h036: tsw6      			<= pressed; // 6
			'h03d: tsw7      			<= pressed; // 7
			'h03e: tsw8		   		<= pressed; // 8
			'h046: tsw9      			<= pressed; // 9
			'h01c: tswa       		<= pressed; // a
			'h032: tswb   				<= pressed; // b
			'h021: tswc  				<= pressed; // c
			'h023: tswd   				<= pressed; // d
			'h024: tswe   				<= pressed; // e
			'h02b: tswf      			<= pressed; // f
			'h03a: tswm      			<= pressed; // m
			'h034: tswg		   		<= pressed; // g
			'h03d: tswp   				<= pressed; // p
			'h01b: tsws  				<= pressed; // s
			'h04b: tswl   				<= pressed; // l
			'h02d: tswr   				<= pressed; // r
			'h005: tswrst      		<= pressed; // F1

		endcase
	end
end

reg tsw0 = 1'b0;
reg tsw1 = 1'b0;
reg tsw2 = 1'b0;
reg tsw3 = 1'b0;
reg tsw4 = 1'b0;
reg tsw5 = 1'b0;
reg tsw6 = 1'b0;
reg tsw7 = 1'b0;
reg tsw8 = 1'b0;
reg tsw9 = 1'b0;
reg tswa = 1'b0;
reg tswb = 1'b0;
reg tswc = 1'b0;
reg tswd = 1'b0;
reg tswe = 1'b0;
reg tswf = 1'b0;
reg tswm = 1'b0;
reg tswl = 1'b0;
reg tswg = 1'b0;
reg tswr = 1'b0;
reg tswp = 1'b0;
reg tswU = 1'b0;
reg tsws = 1'b0;
reg tswD = 1'b0;
reg tswrst = 0;
*/
/////////////////  RESET  /////////////////////////

wire reset = RESET | status[0] | buttons[1] | (~status[12] & img_mounted);

////////////////  MEMORY  /////////////////////////



wire        mem_we,ram_en,rom_en;
wire [15:0] mem_addr;
wire  [7:0] mem_din,ram_dout,rom_dout;


monitor rom
(
	.address(mem_addr),
	.clken(rom_en),
	.clock(clk_sys),
	.q(rom_dout)
);

memory ram
(
	.address(mem_addr),
	.clken(ram_en),
	.clock(clk_sys),
	.data(mem_din),
	.wren(mem_we),
	.q(ram_dout)
);
///////////////////////////////////////////////////


system1 system1
(
	.clk25(clk_25),	
	.reset(reset),
	
	.ch0(ch0),
	.ch1(ch1),
	.ch2(ch2),
	.ch3(ch3),
	.ch4(ch4),
	.ch5(ch5),
	.ch6(ch6),
	.ch7(ch7),
	.sw0(sw0),
	.sw1(sw1),
	.sw2(sw2),
	.sw3(sw3),
	.sw4(sw4),
	.sw5(sw5),
	.sw6(sw6),
	.sw7(sw7),
	.sw8(sw8),
	.sw9(sw9),
	.swa(swa),
	.swb(swb),
	.swc(swc),
	.swd(swd),
	.swe(swe),
	.swf(swf),
	.swrst(swrst),
	.swm(swm),
	.swl(swl),
	.swg(swg),
	.swr(swr),
	.swp(swp),
	.swU(swU),
	.sws(sws),
	.swD(swD),


   .RAMWE(mem_we),
   .ADR(mem_addr),
   .RAMDin(mem_din),
   .RAMDout(ram_dout),
	.ROMDout(rom_dout),
	.cas_in(1'b0),
	.cas_out(),
	.rom_cs(rom_en),
	.ram_cs(ram_en)

);

wire [17:0] sid_audio;
wire a_audio;

assign AUDIO_L = status[4] ? sid_audio[15:0] : {{16{a_audio}}};
assign AUDIO_R = status[4] ? sid_audio[15:0] : {{16{a_audio}}};
assign AUDIO_MIX = 0;
assign AUDIO_S = status[4] ? 0 : 0;

wire hs, vs, hblank, vblank, ce_pix, clk_sel;
wire [7:0] r,g,b;

assign CLK_VIDEO = clk_25;
video_mixer #(640, 0) mixer
(
	.clk_sys(CLK_VIDEO),
	
	.ce_pix(1'b1),
	.ce_pix_out(CE_PIXEL),

	.hq2x(scale == 1),
	.scanlines(0),
	.scandoubler(scale),

	.R(r),
	.G(g),
	.B(b),

	.mono(0),

	.HSync(hs),
	.VSync(vs),
	.HBlank(hblank),
	.VBlank(vblank),

	.VGA_R(VGA_R),
	.VGA_G(VGA_G),
	.VGA_B(VGA_B),
	.VGA_VS(VGA_VS),
	.VGA_HS(VGA_HS),
	.VGA_DE(VGA_DE)
);

assign VGA_F1 = 0;
assign VGA_SL = scale ? scale - 1'd1 : 2'd0;

/////////////////// vga  /////////////////////
wire sw0,sw1,sw2,sw3,sw4,sw5,sw6,sw7,sw8,sw9,swa,swb,swc,swd,swe,swf,swrst,swm,swl,swg,swr,swp,swU,sws,swD;

wire [8:0] ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7;
//assign ch0 = 8'b00111111;
//assign ch1 = 8'b01110111;
//assign ch2 = 8'b10000000;
//assign ch3 = 8'b00111111;
//assign ch4 = 8'b00111111;
//assign ch5 = 8'b01111001;
//assign ch6 = 8'b01110001;
//assign ch7 = 8'b11110111;

vga vga
(
	.clk(clk_25),
	.rst(reset),
	
	.mbtnL(mbtnL),
	.mbtnR(mbtnR),
	.mbtnM(mbtnM),
	.mx(mx),
	.my(my),
	.ch0(ch0),
	.ch1(ch1),
	.ch2(ch2),
	.ch3(ch3),
	.ch4(ch4),
	.ch5(ch5),
	.ch6(ch6),
	.ch7(ch7),
	.osw0(sw0),
	.osw1(sw1),
	.osw2(sw2),
	.osw3(sw3),
	.osw4(sw4),
	.osw5(sw5),
	.osw6(sw6),
	.osw7(sw7),
	.osw8(sw8),
	.osw9(sw9),
	.oswa(swa),
	.oswb(swb),
	.oswc(swc),
	.oswd(swd),
	.oswe(swe),
	.oswf(swf),
	.oswrst(swrst),
	.oswm(swm),
	.oswl(swl),
	.oswg(swg),
	.oswr(swr),
	.oswp(swp),
	.oswU(swU),
	.osws(sws),
	.oswD(swD),
	
	.r(r),
	.g(g),
	.b(b),
	.hs(hs),
	.vs(vs),
	.hblank(hblank),
	.vblank(vblank)
);

/////////////////// mouse ////////////////////
wire [10:0] mx,my;
wire x1,y1,mbtnL,mbtnR,mbtnM;

ps2_mouse mouse
(
	.clk(clk_25),
	.ce(1'b1),
	.reset(reset),
	.ps2_mouse(ps2_mouse),
	.mx(mx),
	.my(my),
	.mbtnL(mbtnL),
	.mbtnR(mbtnR),
	.mbtnM(mbtnM)
);
/*
//////////////////   SD   ///////////////////

wire sdclk;
wire sdmosi;
wire sdmiso = vsd_sel ? vsdmiso : SD_MISO;
wire sdss;

reg vsd_sel = 0;
always @(posedge clk_sys) if(img_mounted) vsd_sel <= |img_size;

wire vsdmiso;
sd_card sd_card
(
	.*,

	.clk_spi(clk_sys),
	.sdhc(1),
	.sck(sdclk),
	.ss(sdss | ~vsd_sel),
	.mosi(sdmosi),
	.miso(vsdmiso)
);

assign SD_CS   = sdss   |  vsd_sel;
assign SD_SCK  = sdclk  & ~vsd_sel;
assign SD_MOSI = sdmosi & ~vsd_sel;

reg sd_act;

always @(posedge clk_sys) begin
	reg old_mosi, old_miso;
	integer timeout = 0;

	old_mosi <= sdmosi;
	old_miso <= sdmiso;

	sd_act <= 0;
	if(timeout < 2000000) begin
		timeout <= timeout + 1;
		sd_act <= 1;
	end

	if((old_mosi ^ sdmosi) || (old_miso ^ sdmiso)) timeout <= 0;
end
*/



endmodule

//////////////////////////////////////////////

module spram #(parameter DATAWIDTH=8, ADDRWIDTH=8, NUMWORDS=1<<ADDRWIDTH, MEM_INIT_FILE="")
(
	input	                 clock,
	input	 [ADDRWIDTH-1:0] address,
	input	 [DATAWIDTH-1:0] data,
	input	                 wren,
	output [DATAWIDTH-1:0] q
);

altsyncram altsyncram_component
(
	.address_a (address),
	.clock0 (clock),
	.data_a (data),
	.wren_a (wren),
	.q_a (q),
	.aclr0 (1'b0),
	.aclr1 (1'b0),
	.address_b (1'b1),
	.addressstall_a (1'b0),
	.addressstall_b (1'b0),
	.byteena_a (1'b1),
	.byteena_b (1'b1),
	.clock1 (1'b1),
	.clocken0 (1'b1),
	.clocken1 (1'b1),
	.clocken2 (1'b1),
	.clocken3 (1'b1),
	.data_b (1'b1),
	.eccstatus (),
	.q_b (),
	.rden_a (1'b1),
	.rden_b (1'b1),
	.wren_b (1'b0)
);

defparam
	altsyncram_component.clock_enable_input_a = "BYPASS",
	altsyncram_component.clock_enable_output_a = "BYPASS",
	altsyncram_component.init_file = MEM_INIT_FILE,
	altsyncram_component.intended_device_family = "Cyclone V",
	altsyncram_component.lpm_type = "altsyncram",
	altsyncram_component.numwords_a = NUMWORDS,
	altsyncram_component.operation_mode = "SINGLE_PORT",
	altsyncram_component.outdata_aclr_a = "NONE",
	altsyncram_component.outdata_reg_a = "UNREGISTERED",
	altsyncram_component.power_up_uninitialized = "FALSE",
	altsyncram_component.read_during_write_mode_port_a = "NEW_DATA_NO_NBE_READ",
	altsyncram_component.widthad_a = ADDRWIDTH,
	altsyncram_component.width_a = DATAWIDTH,
	altsyncram_component.width_byteena_a = 1;


endmodule
