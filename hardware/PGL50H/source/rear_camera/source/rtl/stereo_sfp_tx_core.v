`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Meyesemi
// Engineer: stereo SFP TX integration (2026-10-13)
//
// Create Date: 2024-03-17
// Design Name:  stereo_sfp_tx_core
// Module Name:  stereo_sfp_tx_core
// Project Name: stereo_sfp_tx（PGL50H：双目 OV5640 -> SFP 光纤输出）
// Target Devices: Pango PGL50H-6FBG484
//
// Description:
//   双目摄像头采集 -> DDR3 帧缓存 -> 1280x360 左右拼接 -> SFP(HSST lane2) 发送
//     ch0 = 相机1(左眼)，1280x720 缩放到 640x360
//     ch1 = 相机2(右眼)，1280x720 缩放到 640x360
//     ch2/ch3 = 原 HDMI/单路相机通道，stereo 模式停用（不写 DDR）
//   本地 HDMI 输出通路保持实例化但渲染源被置空（黑屏），仅维持原时钟/复位结构
//
// Revision:
//   1.0 - 原始版本：3 路 HDMI 输入 + 1 路 OV5640，四画面本地 HDMI 输出
//   2.0 - stereo 版本（当前）：cmos1/cmos2 双目输入，1280x360 拼接走 SFP
//////////////////////////////////////////////////////////////////////////////////
`define UD #1
// stereo 模式：CMOS1(左眼)/CMOS2(右眼) 是唯一视频源，分别走 DDR 通道 ch0/ch1。

module stereo_sfp_tx_core#(
	parameter MEM_ROW_ADDR_WIDTH   = 15         ,
	parameter MEM_COL_ADDR_WIDTH   = 10         ,
	parameter MEM_BADDR_WIDTH      = 3          ,
	parameter MEM_DQ_WIDTH         =  32        ,
	parameter MEM_DM_WIDTH         =  MEM_DQ_WIDTH/8,
	parameter MEM_DQS_WIDTH        =  MEM_DQ_WIDTH/8
)(
	input                                sys_clk              /* synthesis PAP_MARK_DEBUG="true" */,//50Mhz
    input                                rst_n                 ,//系统复位
    //按键
    input                                key1                ,
    input                                key2                ,
    input                                key3                ,
    input                                key4                ,
    input                                key5                ,
    input                                key6                ,
    input                                key7                ,


//DDR
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_cs_n                  ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a                     ,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,
    output reg                           heart_beat_led            ,
    output                               ddr_init_done             ,
	
//MS72xx       
    output                               rstn_out                  ,
    output                               iic_tx_scl                ,
    inout                                iic_tx_sda                ,
    output                               iic_scl                    ,
    inout                                iic_sda                    ,
    output                               hdmi_int_led              ,//HDMI_OUT初始化完成
    output                               hdmi_int_led2              ,//HDMI_OUT初始化完成

//hsst	
	input                                i_p_refckn_0              , // 
	input                                i_p_refckp_0              , // 
	input                                i_p_l2rxn                 ,
	input                                i_p_l2rxp                 ,
	input                                i_p_l3rxn                 ,
	input                                i_p_l3rxp                 ,
	output                               o_p_l2txn                 ,
	output                               o_p_l2txp                 ,
	output                               o_p_l3txn                 ,
	output                               o_p_l3txp                 ,
	output wire                          SFP_TX_DISABLE0           ,
	output wire                          SFP_TX_DISABLE1     	      ,  
//HDMI_in
    input             pixclk_in                /* synthesis PAP_MARK_DEBUG="true" */,                            
    input             vs_in                    /* synthesis PAP_MARK_DEBUG="true" */, 
    input             hs_in                    , 
    input             de_in                    /* synthesis PAP_MARK_DEBUG="true" */,
    input     [7:0]   r_in                        , 
    input     [7:0]   g_in                    , 
    input     [7:0]   b_in                        ,      

//HDMI_2接口
    input             pixclk_2_in,
    input             vs_2_in/*synthesis PAP_MARK_DEBUG="true"*/, 
    input             hs_2_in, 
    input             de_2_in/*synthesis PAP_MARK_DEBUG="true"*/,
    input     [7:0]   r_2_in/*synthesis PAP_MARK_DEBUG="true"*/, 
    input     [7:0]   g_2_in/*synthesis PAP_MARK_DEBUG="true"*/, 
    input     [7:0]   b_2_in/*synthesis PAP_MARK_DEBUG="true"*/,  


//HDMI_3接口
    input             pixclk_3_in,
    input             vs_3_in/*synthesis PAP_MARK_DEBUG="true"*/, 
    input             hs_3_in/*synthesis PAP_MARK_DEBUG="true"*/, 
    input             de_3_in/*synthesis PAP_MARK_DEBUG="true"*/,
    input     [7:0]   r_3_in, 
    input     [7:0]   g_3_in, 
    input     [7:0]   b_3_in,  

//OV5640 CMOS1 interface (pin-compatible with the reference design)
    inout             cmos1_scl,
    inout             cmos1_sda,
    input              cmos1_vsync,
    input              cmos1_href,
    input              cmos1_pclk,
    input      [7:0]   cmos1_data,
    output             cmos1_reset,
//OV5640 CMOS2 interface - second eye of the stereo camera module.
//Pin-out taken from 10_HDMI_DDR3_OV5640_test/hdmi_ddr_ov5640_top.fdc.
    inout             cmos2_scl,
    inout             cmos2_sda,
    input              cmos2_vsync,
    input              cmos2_href,
    input              cmos2_pclk,
    input      [7:0]   cmos2_data,
    output             cmos2_reset,
//本地 HDMI 输出（时序保留，渲染源被置空，实际输出黑屏）
    output                               pix_clk                   ,//pixclk
    output     reg                       vs_out                    , 
    output     reg                       hs_out                    , 
    output     reg                       de_out                    ,
    output     reg[7:0]                  r_out                     , 
    output     reg[7:0]                  g_out                     , 
    output     reg[7:0]                  b_out         			   ,

    //hdmi
    output            rstn_out2,
    output            rstn_out3,
	//debug
	//output								 debug_led		        ,
    output                                debug_led2			
	
	
);
assign rstn_out2 = rstn_out;
assign rstn_out3 = rstn_out;
/////////////////////////////////////////////////////////////////////////////////////
// ENABLE_DDR
    parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;//28
    parameter TH_1S = 27'd33000000;
/////////////////////////////////////////////////////////////////////////////////////
	wire						    core_clk			;
	wire						    ddr_init_done		;	
    wire                        pll_lock            ;
    wire                        ddr_reset_n          ;
    wire                        initial_en          ;
    wire                        cmos_init_done      ;
    wire                        cmos_config_error   ;
    wire                        camera_capture_enable;
    wire                        camera_capture_reset_n;
    wire                        cmos2_init_done     ;
    wire                        cmos2_config_error  ;
    wire                        camera2_capture_reset_n;
    wire                        camera_online       ;
    wire                        camera_retry_active ;
    wire[15:0]                  cmos1_d_16bit       ;
    wire                        cmos1_href_16bit    ;
    reg [7:0]                   cmos1_d_d0          ;
    reg                         cmos1_href_d0       ;
    reg                         cmos1_vsync_d0      ;
    // second eye of the stereo module
    wire[15:0]                  cmos2_d_16bit       ;
    wire                        cmos2_href_16bit    ;
    reg [7:0]                   cmos2_d_d0          ;
    reg                         cmos2_href_d0       ;
    reg                         cmos2_vsync_d0      ;
    wire                        camera_line_error   ;
    wire                        camera_frame_error  ;
    wire                        camera_frame_valid  ;
    wire                        camera_frame_valid_toggle;
    wire                        camera_frame_start  ;
//axi bus   
    wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr                 ;
    wire                        axi_awuser_ap              ;
    wire [3:0]                  axi_awuser_id              ;
    wire [3:0]                  axi_awlen                  ;
    wire                        axi_awready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_awvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata                  ;
    wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb                  ;
    wire                        axi_wready                 ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [3:0]                  axi_wusero_id              ;
    wire                        axi_wusero_last            ;
    wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr                 ;
    wire                        axi_aruser_ap              ;
    wire [3:0]                  axi_aruser_id              ;
    wire [3:0]                  axi_arlen                  ;
    wire                        axi_arready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_arvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata                   /* synthesis syn_keep = 1 */;
    wire                        axi_rvalid                  /* synthesis syn_keep = 1 */;
    wire [3:0]                  axi_rid                    ;
    wire                        axi_rlast                  ;
    reg  [26:0]                 cnt                        ;
    reg  [15:0]                 cnt_1                      ;
	
/************************************************************
AXI参数定定义
************************************************************/	
parameter MEM_DATA_BITS          = 256;             //external memory user interface data width
parameter ADDR_BITS              = 25;             //external memory user interface address width
parameter BUSRT_BITS             = 10;             //external memory user interface burst width
wire                            wr_burst_data_req;
wire                            wr_burst_finish;
wire                            rd_burst_finish;
wire                            rd_burst_req;
wire                            wr_burst_req;
wire[BUSRT_BITS - 1:0]          rd_burst_len;
wire[BUSRT_BITS - 1:0]          wr_burst_len;
wire[ADDR_BITS - 1:0]           rd_burst_addr;
wire[ADDR_BITS - 1:0]           wr_burst_addr;
wire                            rd_burst_data_valid;
wire[MEM_DATA_BITS - 1 : 0]     rd_burst_data;
wire[MEM_DATA_BITS - 1 : 0]     wr_burst_data;

wire                            video_clk;         //video pixel clock
wire                            video_clk5x;
wire                            hs;
wire                            vs;
wire                            de;
wire[15:0]                      vout_data;
wire[15:0]                      v2_safe_data;
wire[15:0]                      sfp_output_data;
wire[15:0]						vout_data_2;
wire[15:0]                      cmos_16bit_data;
wire                            cmos_16bit_wr;
wire[9:0]                       lut_index;
wire[31:0]                      lut_data;

wire                            ui_clk;
// Master Write Address
wire [3:0]                      s00_axi_awid;
wire [63:0]                     s00_axi_awaddr;
wire [7:0]                      s00_axi_awlen;    // burst length: 0-255
wire [2:0]                      s00_axi_awsize;   // burst size: fixed 2'b011
wire [1:0]                      s00_axi_awburst;  // burst type: fixed 2'b01(incremental burst)
wire                            s00_axi_awlock;   // lock: fixed 2'b00
wire [3:0]                      s00_axi_awcache;  // cache: fiex 2'b0011
wire [2:0]                      s00_axi_awprot;   // protect: fixed 2'b000
wire [3:0]                      s00_axi_awqos;    // qos: fixed 2'b0000
wire [0:0]                      s00_axi_awuser;   // user: fixed 32'd0
wire                            s00_axi_awvalid;
wire                            s00_axi_awready;
// master write data
wire [MEM_DATA_BITS - 1 : 0]    s00_axi_wdata/*synthesis PAP_MARK_DEBUG = "ture"*/;
wire [MEM_DATA_BITS/8 - 1:0]    s00_axi_wstrb;
wire                            s00_axi_wlast;
wire [0:0]                      s00_axi_wuser;
wire                            s00_axi_wvalid;
wire                            s00_axi_wready;
// master write response
wire [3:0]                      s00_axi_bid;
wire [1:0]                      s00_axi_bresp;
wire [0:0]                      s00_axi_buser;
wire                            s00_axi_bvalid;
wire                            s00_axi_bready;
// master read address
wire [3:0]                      s00_axi_arid;
wire [63:0]                     s00_axi_araddr;
wire [7:0]                      s00_axi_arlen;
wire [2:0]                      s00_axi_arsize;
wire [1:0]                      s00_axi_arburst;
wire [1:0]                      s00_axi_arlock;
wire [3:0]                      s00_axi_arcache;
wire [2:0]                      s00_axi_arprot;
wire [3:0]                      s00_axi_arqos;
wire [0:0]                      s00_axi_aruser;
wire                            s00_axi_arvalid;
wire                            s00_axi_arready;
// master read data
wire [3:0]                      s00_axi_rid;
wire [MEM_DATA_BITS - 1 : 0]    s00_axi_rdata/*synthesis PAP_MARK_DEBUG = "ture"*/;
wire [1:0]                      s00_axi_rresp;
wire                            s00_axi_rlast;
wire [0:0]                      s00_axi_ruser;
wire                            s00_axi_rvalid;
wire                            s00_axi_rready;	

/************************************************************
hsst参数定义
************************************************************/	
wire          i_wtchdg_clr_0 ;
assign        i_wtchdg_clr_0=1'b0;

assign         SFP_TX_DISABLE0       = 1'b0 ;
assign         SFP_TX_DISABLE1       = 1'b0 ;


wire tx0_clk;
wire gt0_txfsmresetdone;
wire gt1_txfsmresetdone;
wire gt2_txfsmresetdone;
wire gt3_txfsmresetdone;
wire[31:0] tx0_data;
wire[3:0] tx0_kchar;
wire tx1_clk;
wire[31:0] tx1_data;
wire[3:0] tx1_kchar; 
wire tx2_clk;
wire[31:0] tx2_data;
wire[3:0] tx2_kchar;
wire tx3_clk;
wire[31:0] tx3_data;
wire[3:0] tx3_kchar;
  
wire rx0_clk;
wire[31:0] rx0_data;
wire[3:0] rx0_kchar;
wire rx1_clk;
wire[31:0] rx1_data ;
wire[3:0] rx1_kchar ;
wire rx2_clk;
wire[31:0] rx2_data ;
wire[3:0] rx2_kchar ;
wire rx3_clk;
wire[31:0] rx3_data;
wire[3:0] rx3_kchar;

reg[31:0] gt_tx_data ;
reg[3:0] gt_tx_ctrl ;

wire[31:0] gt_tx_data0 ;
wire[3:0] gt_tx_ctrl0 ;
wire[31:0] gt_tx_data1 ;
wire[3:0] gt_tx_ctrl1 ;

wire rx_clk;
wire tx_clk;
wire[31:0] rx_data   ;
wire[3:0] rx_kchar   ;

assign tx_clk = tx2_clk;
assign rx_clk = rx3_clk;
assign rx_data = rx2_data;
assign rx_kchar = rx2_kchar;
/*
assign tx0_data = gt_tx_data;
assign tx0_kchar = gt_tx_ctrl;
assign tx1_data = gt_tx_data;
assign tx1_kchar = gt_tx_ctrl;
*/

assign tx2_data = gt_tx_data;
assign tx2_kchar = gt_tx_ctrl;

assign tx3_data = gt_tx_data;
assign tx3_kchar = gt_tx_ctrl;


/************************************************************
通道一
************************************************************/
wire                            ch0_wr_burst_data_req;
wire                            ch0_wr_burst_finish;
wire                            ch0_rd_burst_finish;
wire                            ch0_rd_burst_req;
wire                            ch0_wr_burst_req;
wire[BUSRT_BITS - 1:0]          ch0_rd_burst_len;
wire[BUSRT_BITS - 1:0]          ch0_wr_burst_len;
wire[ADDR_BITS - 1:0]           ch0_rd_burst_addr;
wire[ADDR_BITS - 1:0]           ch0_wr_burst_addr;
wire                            ch0_rd_burst_data_valid;
wire[MEM_DATA_BITS - 1 : 0]     ch0_rd_burst_data;
wire[MEM_DATA_BITS - 1 : 0]     ch0_wr_burst_data;

wire                            ch0_read_req;
wire                            ch0_read_req_ack;
wire                            ch0_read_en;
wire[15:0]                      ch0_read_data;
wire                            ch0_write_en;
wire[15:0]                      ch0_write_data;
wire                            ch0_write_req;
wire                            ch0_write_req_ack;
wire[1:0]                       ch0_write_addr_index;
wire[1:0]                       ch0_read_addr_index;

wire                            read_req;
wire                            read_req_ack;
wire                            read_en/*synthesis PAP_MARK_DEBUG = "ture"*/;
wire[15:0]                      read_data/*synthesis PAP_MARK_DEBUG = "ture"*/;
wire                            write_en/*synthesis PAP_MARK_DEBUG = "ture"*/;
wire[15:0]                      write_data /*synthesis PAP_MARK_DEBUG = "ture"*/;
wire                            write_req;
wire                            write_req_ack;
wire[1:0]                       write_addr_index;
wire[1:0]                       read_addr_index;
/************************************************************
通道二
************************************************************/
wire                            ch1_wr_burst_data_req;
wire                            ch1_wr_burst_finish;
wire                            ch1_rd_burst_finish;
wire                            ch1_rd_burst_req;
wire                            ch1_wr_burst_req;
wire[BUSRT_BITS - 1:0]          ch1_rd_burst_len;
wire[BUSRT_BITS - 1:0]          ch1_wr_burst_len;
wire[ADDR_BITS - 1:0]           ch1_rd_burst_addr;
wire[ADDR_BITS - 1:0]           ch1_wr_burst_addr;
wire                            ch1_rd_burst_data_valid;
wire[MEM_DATA_BITS - 1 : 0]     ch1_rd_burst_data;
wire[MEM_DATA_BITS - 1 : 0]     ch1_wr_burst_data;
wire                            ch1_read_req;
wire                            ch1_read_req_ack;
wire                            ch1_read_en;
wire[15:0]                      ch1_read_data;
wire                            ch1_write_en;
wire[15:0]                      ch1_write_data;
wire                            ch1_write_req;
wire                            ch1_write_req_ack;
wire[1:0]                       ch1_write_addr_index;
wire[1:0]                       ch1_read_addr_index;
/************************************************************
通道三
************************************************************/
wire                            ch2_wr_burst_data_req;
wire                            ch2_wr_burst_finish;
wire                            ch2_rd_burst_finish;
wire                            ch2_rd_burst_req;
wire                            ch2_wr_burst_req;
wire[BUSRT_BITS - 1:0]          ch2_rd_burst_len;
wire[BUSRT_BITS - 1:0]          ch2_wr_burst_len;
wire[ADDR_BITS - 1:0]           ch2_rd_burst_addr;
wire[ADDR_BITS - 1:0]           ch2_wr_burst_addr;
wire                            ch2_rd_burst_data_valid;
wire[MEM_DATA_BITS - 1 : 0]     ch2_rd_burst_data;
wire[MEM_DATA_BITS - 1 : 0]     ch2_wr_burst_data;
wire                            ch2_read_req;
wire                            ch2_read_req_ack;
wire                            ch2_read_en;
wire[15:0]                      ch2_read_data;
wire                            ch2_write_en;
wire[15:0]                      ch2_write_data;
wire                            ch2_write_req;
wire                            ch2_write_req_ack;
wire[1:0]                       ch2_write_addr_index;
wire[1:0]                       ch2_read_addr_index;
/************************************************************
通道四
************************************************************/
wire                            ch3_wr_burst_data_req;
wire                            ch3_wr_burst_finish;
wire                            ch3_rd_burst_finish;
wire                            ch3_rd_burst_req;
wire                            ch3_wr_burst_req;
wire[BUSRT_BITS - 1:0]          ch3_rd_burst_len;
wire[BUSRT_BITS - 1:0]          ch3_wr_burst_len;
wire[ADDR_BITS - 1:0]           ch3_rd_burst_addr;
wire[ADDR_BITS - 1:0]           ch3_wr_burst_addr;
wire                            ch3_rd_burst_data_valid;
wire[MEM_DATA_BITS - 1 : 0]     ch3_rd_burst_data;
wire[MEM_DATA_BITS - 1 : 0]     ch3_wr_burst_data;
wire                            ch3_read_req;
wire                            ch3_read_req_ack;
wire                            ch3_read_en;
wire[15:0]                      ch3_read_data;
wire                            ch3_write_en;
wire[15:0]                      ch3_write_data;
wire                            ch3_write_req;
wire                            ch3_write_req_ack;
wire[1:0]                       ch3_write_addr_index;
wire[1:0]                       ch3_read_addr_index;
// Channel 4 (single OV5640 path) is removed in stereo mode - see the note at
// ch3_write_data/ch3_write_en below. No remnant wires are kept here.

/************************************************************
HDMI-only DDR readers (logical channels 0..3 map to arbiter 4..7)
************************************************************/
wire ch4_rd_burst_finish, ch4_rd_burst_req, ch4_rd_burst_data_valid;
wire [BUSRT_BITS-1:0] ch4_rd_burst_len;
wire [ADDR_BITS-1:0] ch4_rd_burst_addr;
wire [MEM_DATA_BITS-1:0] ch4_rd_burst_data;
wire ch5_rd_burst_finish, ch5_rd_burst_req, ch5_rd_burst_data_valid;
wire [BUSRT_BITS-1:0] ch5_rd_burst_len;
wire [ADDR_BITS-1:0] ch5_rd_burst_addr;
wire [MEM_DATA_BITS-1:0] ch5_rd_burst_data;
wire ch6_rd_burst_finish, ch6_rd_burst_req, ch6_rd_burst_data_valid;
wire [BUSRT_BITS-1:0] ch6_rd_burst_len;
wire [ADDR_BITS-1:0] ch6_rd_burst_addr;
wire [MEM_DATA_BITS-1:0] ch6_rd_burst_data;
wire ch7_rd_burst_finish, ch7_rd_burst_req, ch7_rd_burst_data_valid;
wire [BUSRT_BITS-1:0] ch7_rd_burst_len;
wire [ADDR_BITS-1:0] ch7_rd_burst_addr;
wire [MEM_DATA_BITS-1:0] ch7_rd_burst_data;

wire ch4_read_req, ch4_read_req_ack, ch4_read_en;
wire [15:0] ch4_read_data;
wire ch4_read_fifo_empty, ch4_read_fifo_full, ch4_read_overflow_sticky;
wire ch5_read_req, ch5_read_req_ack, ch5_read_en;
wire [15:0] ch5_read_data;
wire ch5_read_fifo_empty, ch5_read_fifo_full, ch5_read_overflow_sticky;
wire ch6_read_req, ch6_read_req_ack, ch6_read_en;
wire [15:0] ch6_read_data;
wire ch6_read_fifo_empty, ch6_read_fifo_full, ch6_read_overflow_sticky;
wire ch7_read_req, ch7_read_req_ack, ch7_read_en;
wire [15:0] ch7_read_data;
wire ch7_read_fifo_empty, ch7_read_fifo_full, ch7_read_overflow_sticky;


reset_dly delay_u0(
.clk       (sys_clk        ),
.rst_n     (rst_n ),
.rst_n_dly (phy_rst_n          )
);

/////////////////////////////////////////////////////////////////////////////////////
//PLL
    wire cfg_clk;
    wire clk_25M;
    wire locked;
    pll u_pll (
        .clkin1   (  sys_clk    ),//50MHz
        .clkout0  (  pix_clk    ),//
        .clkout1  (  cfg_clk    ),//10MHz
        .clkout2  (  clk_25M    ),//25M
        .pll_lock (  locked     )
    );

// OV5640 activity detection remains alive even while capture is reset. A slow
// PCLK counter bit and a VS toggle can therefore be monitored from 50 MHz.
reg [8:0] camera_pclk_activity_count;
reg       camera_vsync_raw_d;
reg       camera_vsync_activity_toggle;
always @(posedge cmos1_pclk or negedge rst_n) begin
    if(!rst_n) begin
        camera_pclk_activity_count <= 9'd0;
        camera_vsync_raw_d <= 1'b0;
        camera_vsync_activity_toggle <= 1'b0;
    end
    else begin
        camera_pclk_activity_count <= camera_pclk_activity_count + 1'b1;
        camera_vsync_raw_d <= cmos1_vsync;
        if(cmos1_vsync && !camera_vsync_raw_d)
            camera_vsync_activity_toggle <= ~camera_vsync_activity_toggle;
    end
end

camera_supervisor u_camera_supervisor (
    .clk                    (sys_clk),
    .rst_n                  (rstn_out),
    .config_done            (cmos_init_done),
    .config_error           (cmos_config_error),
    .pclk_activity_toggle   (camera_pclk_activity_count[8]),
    .vs_activity_toggle     (camera_vsync_activity_toggle),
    .camera_reset_n         (cmos1_reset),
    .config_start           (initial_en),
    .capture_enable         (camera_capture_enable),
    .camera_online          (camera_online),
    .retry_active           (camera_retry_active)
);

ov5640_reg_config u_camera_reg_config (
    .clk_25M       (clk_25M),
    .camera_rstn   (cmos1_reset),
    .initial_en    (initial_en),
    .i2c_sclk      (cmos1_scl),
    .i2c_sdat      (cmos1_sda),
    .reg_conf_done (cmos_init_done),
    .config_error  (cmos_config_error),
    .reg_index     (),
    .clock_20k     ()
);

wire camera_capture_async_n = rstn_out && camera_capture_enable;
(* ASYNC_REG="TRUE" *) reg [1:0] camera_capture_reset_sync;
always @(posedge cmos1_pclk or negedge camera_capture_async_n) begin
    if(!camera_capture_async_n)
        camera_capture_reset_sync <= 2'b00;
    else
        camera_capture_reset_sync <= {camera_capture_reset_sync[0], 1'b1};
end
assign camera_capture_reset_n = camera_capture_reset_sync[1];

always @(posedge cmos1_pclk or negedge camera_capture_reset_n) begin
    if(!camera_capture_reset_n) begin
        cmos1_d_d0     <= 8'd0;
        cmos1_href_d0  <= 1'b0;
        cmos1_vsync_d0 <= 1'b0;
    end
    else begin
        cmos1_d_d0     <= cmos1_data;
        cmos1_href_d0  <= cmos1_href;
        cmos1_vsync_d0 <= cmos1_vsync;
    end
end

ov5640_byte_capture cmos1_8_16bit (
    .pclk        (cmos1_pclk),
    .rst_n       (camera_capture_reset_n),
    .href        (cmos1_href_d0),
    .data        (cmos1_d_d0),
    .vsync       (cmos1_vsync_d0),
    .pixel_valid (cmos1_href_16bit),
    .pixel_data  (cmos1_d_16bit),
    .line_error  (camera_line_error),
    .frame_error (camera_frame_error),
    .frame_valid (camera_frame_valid),
    .frame_valid_toggle(camera_frame_valid_toggle),
    .frame_start (camera_frame_start)
);

//------------------------------------------------------------------------------
// Second eye (cmos2). Both sensors share one I2C configuration table and one
// capture-enable signal; only the bus, the pixel clock and the reset release
// are per-eye. cmos2_reset mirrors cmos1_reset so both sensors come out of
// reset together.
//------------------------------------------------------------------------------
assign cmos2_reset = cmos1_reset;

ov5640_reg_config u_camera2_reg_config (
    .clk_25M       (clk_25M),
    .camera_rstn   (cmos2_reset),
    .initial_en    (initial_en),
    .i2c_sclk      (cmos2_scl),
    .i2c_sdat      (cmos2_sda),
    .reg_conf_done (cmos2_init_done),
    .config_error  (cmos2_config_error),
    .reg_index     (),
    .clock_20k     ()
);

(* ASYNC_REG="TRUE" *) reg [1:0] camera2_capture_reset_sync;
always @(posedge cmos2_pclk or negedge camera_capture_async_n) begin
    if(!camera_capture_async_n)
        camera2_capture_reset_sync <= 2'b00;
    else
        camera2_capture_reset_sync <= {camera2_capture_reset_sync[0], 1'b1};
end
assign camera2_capture_reset_n = camera2_capture_reset_sync[1];

always @(posedge cmos2_pclk or negedge camera2_capture_reset_n) begin
    if(!camera2_capture_reset_n) begin
        cmos2_d_d0     <= 8'd0;
        cmos2_href_d0  <= 1'b0;
        cmos2_vsync_d0 <= 1'b0;
    end
    else begin
        cmos2_d_d0     <= cmos2_data;
        cmos2_href_d0  <= cmos2_href;
        cmos2_vsync_d0 <= cmos2_vsync;
    end
end

ov5640_byte_capture cmos2_8_16bit (
    .pclk        (cmos2_pclk),
    .rst_n       (camera2_capture_reset_n),
    .href        (cmos2_href_d0),
    .data        (cmos2_d_d0),
    .vsync       (cmos2_vsync_d0),
    .pixel_valid (cmos2_href_16bit),
    .pixel_data  (cmos2_d_16bit),
    .line_error  (),
    .frame_error (),
    .frame_valid (),
    .frame_valid_toggle(),
    .frame_start ()
);

wire    init_over_rx;
wire    init_over;
wire    init_over_2;

assign    hdmi_int_led    =    init_over; 
assign    hdmi_int_led2    =    init_over_2; 
 
// Keep DDR startup entirely in the fixed 50 MHz reference domain. Camera,
// HDMI-control and HSST resets do not depend on camera initialization.
wire system_startup_done;
ddr_startup_reset #(.STARTUP_DELAY_CYCLES(50000)) u_ddr_startup_reset (
    .sys_clk          (sys_clk),
    .external_reset_n (rst_n),
    .board_pll_locked (locked),
    .system_reset_n   (system_startup_done),
    .ddr_reset_n      (ddr_reset_n)
);

assign rstn_out = system_startup_done;
 //配置HDMI IN3 和HDMI OUT
assign init_over = 1'b1;
assign iic_tx_scl = 1'b1;
assign iic_tx_sda = 1'bz;
//配置HDMI_IN1 和HDMI_IN2
ms7200_double_crtl ms7200_double_crtl_inst(
    .clk(cfg_clk),              // input
    .rst_n(rstn_out&&init_over),          // input
    .init_over(init_over_2),  // output
    .iic_scl(iic_scl),      // output
    .iic_sda(iic_sda)       // inout
);


/************************************************************
HSST数据处理 利用hdmi进来的数据
************************************************************/

always@(posedge tx_clk)
begin
    gt_tx_data  <= gt_tx_data0;
    gt_tx_ctrl <= gt_tx_ctrl0;           
end


//32位数据对齐模块
wire[31:0] rx_data_align /* synthesis PAP_MARK_DEBUG="true" */;
wire[3:0] rx_ctrl_align /* synthesis PAP_MARK_DEBUG="true" */;
word_align word_align_m0
(
    .rst                        (~rstn_out               ),
    .rx_clk                     (rx_clk                  ),
    .gt_rx_data                 (rx_data                 ),
    .gt_rx_ctrl                 (rx_kchar                ),
    .rx_data_align              (rx_data_align           ),
    .rx_ctrl_align              (rx_ctrl_align           )
);


//视频数据解析模块（SFP 接收链，输出未用；vout_width 参数不参与实际功能）
wire vs_wr;
wire de_wr;
wire[15:0] vout_data_r;

video_packet_rec video_packet_rec_m0
(
	.rst                        (~rstn_out                  ),
	.rx_clk                     (rx_clk                  ),
	.gt_rx_data                 (rx_data_align           ),
	.gt_rx_ctrl                 (rx_ctrl_align           ),
	.vout_width                 (16'd1920                ),
	
	.vs                         (vs_wr                   ),
	.de                         (de_wr                   ),
	.vout_data                  (vout_data_r             )
);





/************************************************************
模式/帧索引握手：change_mode 在 tx_clk 域发布各通道帧完成索引，
下方 always 块把模式载荷同步进各个写时钟域（stereo 模式为惰性握手）
************************************************************/
//对各自通道打两拍
reg    hdmi_vs_in_d0;
reg    hdmi_vs_in_d1;
reg    hdmi_vs_2_in_d0;
reg    hdmi_vs_2_in_d1;
reg    hdmi_vs_3_in_d0;
reg    hdmi_vs_3_in_d1;

always@(posedge pixclk_in or negedge rstn_out)    begin
    if(!rstn_out)
    begin
        hdmi_vs_in_d0    <=    1'd0;
        hdmi_vs_in_d1    <=    1'd0;
    end
    else
    begin
        hdmi_vs_in_d0    <=    vs_in;
        hdmi_vs_in_d1    <=    hdmi_vs_in_d0;
    end
end

always@(posedge pixclk_2_in or negedge rstn_out)    begin
    if(!rstn_out)
    begin
        hdmi_vs_2_in_d0    <=    1'd0;
        hdmi_vs_2_in_d1    <=    1'd0;
    end
    else
    begin
        hdmi_vs_2_in_d0    <=    vs_2_in;
        hdmi_vs_2_in_d1    <=    hdmi_vs_2_in_d0;
    end
end

always@(posedge pixclk_3_in or negedge rstn_out)    begin
    if(!rstn_out)
    begin
        hdmi_vs_3_in_d0    <=    1'd0;
        hdmi_vs_3_in_d1    <=    1'd0;
    end
    else
    begin
        hdmi_vs_3_in_d0    <=    vs_3_in;
        hdmi_vs_3_in_d1    <=    hdmi_vs_3_in_d0;
    end
end


//在各自1080p的时候不能去突发请求
//通道一 camera1
wire    hdmi_0_write_req;
cmos_write_req_gen cmos_write_req_gen_m0(
	.rst                        (~camera_capture_reset_n  ),
	.pclk                       (cmos1_pclk           ),
	.cmos_vsync                 (cmos1_vsync_d0           ),
	.enable                     (ch0_write_enable_local      ),
	.write_req                  (hdmi_0_write_req            ),
	.write_addr_index           (ch0_write_addr_index     ),
	.read_addr_index            (ch0_read_addr_index      ),
	.write_req_ack              (ch0_write_req_ack        )
);
assign ch0_write_req = hdmi_0_write_req;
//通道二 camera2
wire    hdmi_1_write_req;
cmos_write_req_gen cmos_write_req_gen_m1(
	.rst                        (~camera2_capture_reset_n),
	.pclk                       (cmos2_pclk               ),
	.cmos_vsync                 (cmos2_vsync_d0           ),
	.enable                     (ch1_write_enable_local       ),
	.write_req                  (hdmi_1_write_req            ),
	.write_addr_index           (ch1_write_addr_index     ),
	.read_addr_index            (ch1_read_addr_index      ),
	.write_req_ack              (ch1_write_req_ack        )
);
assign ch1_write_req = hdmi_1_write_req;
//通道三 HDMI_2 
wire    hdmi_2_write_req;
cmos_write_req_gen cmos_write_req_gen_m2(
	.rst                        (~rstn_out                ),
	.pclk                       (pixclk_3_in              ),
	.cmos_vsync                 (hdmi_vs_3_in_d1          ),
	.enable                     (ch2_write_enable_local   ),
	.write_req                  (hdmi_2_write_req            ),
	.write_addr_index           (ch2_write_addr_index     ),
	.read_addr_index            (ch2_read_addr_index      ),
	.write_req_ack              (ch2_write_req_ack        )
);
assign ch2_write_req = hdmi_2_write_req;
 //通道四 OV5640 - disabled in stereo mode
cmos_write_req_gen cmos_write_req_gen_m3(
	.rst                        (~camera_capture_reset_n  ),
	.pclk                       (cmos1_pclk               ),
	.cmos_vsync                 (cmos1_vsync_d0           ),
	.enable                     (1'b0                     ),
	.write_req                  (ch3_write_req            ),
	.write_addr_index           (ch3_write_addr_index     ),
	.read_addr_index            (ch3_read_addr_index      ),
	.write_req_ack              (ch3_write_req_ack        )
);

/************************************************************
缩放处理
************************************************************/
wire              hdmi_1_scale_de;
wire    [23:0]    hdmi_1_scale_data;
video_scale_process#(
    .PIX_DATA_WIDTH       ( 24 )
)u_video_scale_process_0(
    .video_clk            ( pixclk_in          ),
    .rst_n                ( rstn_out             ),
    .frame_sync_n         ( ~hdmi_vs_in_d1       ),
    .video_data_in        ( {r_in,g_in,b_in}        ),
    .video_data_valid     ( de_in     ),
    .video_data_out       ( hdmi_1_scale_data       ),
    .video_data_out_valid ( hdmi_1_scale_de ),
    .video_ready          ( 1'b1          ),
    .video_width_in       ( 1920       ),
    .video_height_in      ( 1080      ),
    .video_width_out      ( 960      ),
    .video_height_out     ( 540     )
);
wire              hdmi_2_scale_de;
wire    [23:0]    hdmi_2_scale_data;
video_scale_process#(
    .PIX_DATA_WIDTH       ( 24 )
)u_video_scale_process_1(
    .video_clk            ( pixclk_2_in          ),
    .rst_n                ( rstn_out             ),
    .frame_sync_n         ( ~hdmi_vs_2_in_d1       ),
    .video_data_in        ( {r_2_in,g_2_in,b_2_in}        ),
    .video_data_valid     ( de_2_in     ),
    .video_data_out       ( hdmi_2_scale_data       ),
    .video_data_out_valid ( hdmi_2_scale_de ),
    .video_ready          ( 1'b1          ),
    .video_width_in       ( 1920       ),
    .video_height_in      ( 1080      ),
    .video_width_out      ( 960      ),
    .video_height_out     ( 540     )
);
wire              hdmi_3_scale_de;
wire    [23:0]    hdmi_3_scale_data;
video_scale_process#(
    .PIX_DATA_WIDTH       ( 24 )
)u_video_scale_process_2(
    .video_clk            ( pixclk_3_in          ),
    .rst_n                ( rstn_out             ),
    .frame_sync_n         ( ~hdmi_vs_3_in_d1       ),
    .video_data_in        ( {r_3_in,g_3_in,b_3_in}        ),
    .video_data_valid     ( de_3_in     ),
    .video_data_out       ( hdmi_3_scale_data       ),
    .video_data_out_valid ( hdmi_3_scale_de ),
    .video_ready          ( 1'b1          ),
    .video_width_in       ( 1920       ),
    .video_height_in      ( 1080      ),
    .video_width_out      ( 960      ),
    .video_height_out     ( 540     )
);

// Stereo scaling for the SFP link: each eye is reduced to 640x360 so that the
// two of them fit side by side into one 1280x360 frame. 640x360 is also the
// geometry the PG2L50H receiver expects for every mosaic cell, so no host-side
// change is needed.
wire              cam1_scale_de;
wire    [15:0]    cam1_scale_data;
wire              cam2_scale_de;
wire    [15:0]    cam2_scale_data;

video_scale_process#(
    .PIX_DATA_WIDTH       ( 16 )
)u_cam1_scale_process(
    .video_clk            ( cmos1_pclk            ),
    .rst_n                ( camera_capture_reset_n ),
    .frame_sync_n         ( ~cmos1_vsync_d0       ),
    .video_data_in        ( cmos1_d_16bit         ),
    .video_data_valid     ( cmos1_href_16bit      ),
    .video_data_out       ( cam1_scale_data       ),
    .video_data_out_valid ( cam1_scale_de         ),
    .video_ready          ( 1'b1                  ),
    .video_width_in       ( 1280                  ),
    .video_height_in      ( 720                   ),
    .video_width_out      ( 640                   ),
    .video_height_out     ( 360                   )
);

video_scale_process#(
    .PIX_DATA_WIDTH       ( 16 )
)u_cam2_scale_process(
    .video_clk            ( cmos2_pclk             ),
    .rst_n                ( camera2_capture_reset_n ),
    .frame_sync_n         ( ~cmos2_vsync_d0        ),
    .video_data_in        ( cmos2_d_16bit          ),
    .video_data_valid     ( cmos2_href_16bit       ),
    .video_data_out       ( cam2_scale_data        ),
    .video_data_out_valid ( cam2_scale_de          ),
    .video_ready          ( 1'b1                   ),
    .video_width_in       ( 1280                   ),
    .video_height_in      ( 720                    ),
    .video_width_out      ( 640                    ),
    .video_height_out     ( 360                    )
);






//按键控制（stereo 模式显示布局固定，不需要按键切换）
wire    [7:0]    mode;
wire             mode_toggle;
// 显示模式固定为 1，附带恒定 toggle 供 change_mode 完成启动握手。
assign mode = 8'd1;
assign mode_toggle = 1'b1;

wire    [7:0]    display_mode_wr;    //需要跨时钟到各个写
wire [7:0] display_mode_rd              /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_1_width            /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_1_height           /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_2_width            /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_2_height           /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_3_width            /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_3_height           /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_1_width_offset     /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_1_height_offset    /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_2_width_offset     /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_2_height_offset    /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_3_width_offset     /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] hdmi_out_3_height_offset    /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] camera_width                /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] camera_height               /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] camera_width_offset         /* synthesis PAP_MARK_DEBUG="true" */;
wire [11:0] camera_height_offset        /* synthesis PAP_MARK_DEBUG="true" */;
wire write_mode_toggle                  /* synthesis PAP_MARK_DEBUG="true" */;
wire switch_blank                       /* synthesis PAP_MARK_DEBUG="true" */;
wire [3:0] display_channel_valid        /* synthesis PAP_MARK_DEBUG="true" */;
wire switch_timeout_sticky              /* synthesis PAP_MARK_DEBUG="true" */;
wire [7:0] pending_mode_debug           /* synthesis PAP_MARK_DEBUG="true" */;
wire [7:0] active_mode_debug            /* synthesis PAP_MARK_DEBUG="true" */;
wire    [3:0]   frame_done_toggle_bus;
wire    [3:0]   frame_done_tag_bus;
wire    [7:0]   frame_done_index_bus;
wire    [7:0]   published_frame_index   /* synthesis PAP_MARK_DEBUG="true" */;
wire    [3:0]   display_state_version   /* synthesis PAP_MARK_DEBUG="true" */;
// Stereo SFP mode writes camera1/camera2 to DDR channels 0/1.  The original
// HDMI design put its single camera on channel 3; keeping that source-alive
// mapping here prevented change_mode from ever publishing channels 0/1 and
// forced sfp_video_fallback to transmit color bars forever.  Frame-completion
// events below still independently qualify each camera before it is displayed.
wire    [2:0]   hdmi_source_alive;
wire    [3:0]   source_alive_bus = {2'b00, camera_online, camera_online};

video_source_watchdog u_video_source_watchdog (
    .clk          (sys_clk),
    .rst_n        (rstn_out),
    .async_vs     ({vs_3_in, vs_2_in, vs_in}),
    .source_alive (hdmi_source_alive)
);

(* ASYNC_REG="TRUE" *) reg ddr_ready_tx_meta;
(* ASYNC_REG="TRUE" *) reg ddr_ready_tx;
always @(posedge tx_clk or negedge rstn_out) begin
    if(!rstn_out) begin
        ddr_ready_tx_meta <= 1'b0;
        ddr_ready_tx      <= 1'b0;
    end
    else begin
        ddr_ready_tx_meta <= ddr_init_done;
        ddr_ready_tx      <= ddr_ready_tx_meta;
    end
end

change_mode change_mode_inst
    (
    .video_out_vs(vs),                             // final composed output VS
    .rst_n(rstn_out && ddr_ready_tx),                         // wait for calibrated DDR

    .video_clk(tx_clk),                                // input

    .display_mode(mode),                          // input[7:0]
    .display_mode_toggle(mode_toggle),
    .frame_done_toggle(frame_done_toggle_bus),
    .frame_done_tag(frame_done_tag_bus),
    .frame_done_index(frame_done_index_bus),
    .source_alive(source_alive_bus),

    .hdmi_out_1_width(hdmi_out_1_width),                  // output[11:0]
    .hdmi_out_1_height(hdmi_out_1_height),                // output[11:0]
    .hdmi_out_2_width(hdmi_out_2_width),                  // output[11:0]
    .hdmi_out_2_height(hdmi_out_2_height),                // output[11:0]
    .hdmi_out_3_width(hdmi_out_3_width),                  // output[11:0]
    .hdmi_out_3_height(hdmi_out_3_height),                // output[11:0]
    .hdmi_out_1_width_offset(hdmi_out_1_width_offset),    // output[11:0]
    .hdmi_out_1_height_offset(hdmi_out_1_height_offset),  // output[11:0]
    .hdmi_out_2_width_offset(hdmi_out_2_width_offset),    // output[11:0]
    .hdmi_out_2_height_offset(hdmi_out_2_height_offset),  // output[11:0]
    .hdmi_out_3_width_offset(hdmi_out_3_width_offset),    // output[11:0]
    .hdmi_out_3_height_offset(hdmi_out_3_height_offset),  // output[11:0]
    .camera_width(camera_width),
    .camera_height(camera_height),
    .camera_width_offset(camera_width_offset),
    .camera_height_offset(camera_height_offset),

    .display_mode_wr(display_mode_wr),                    // output[7:0]
    .display_mode_rd(display_mode_rd),                    // output[7:0]
    .write_mode_toggle(write_mode_toggle),
    .switch_blank(switch_blank),
    .channel_valid(display_channel_valid),
    .switch_timeout_sticky(switch_timeout_sticky),
    .published_frame_index(published_frame_index),
    .display_state_version(display_state_version),
    .pending_mode_debug(pending_mode_debug),
    .active_mode_debug(active_mode_debug)
);

// Stable mode payload plus toggle handshake into each asynchronous input domain.
(* ASYNC_REG="TRUE" *) reg [7:0] display_mode_wr_1_meta, display_mode_wr_1_sync;
(* ASYNC_REG="TRUE" *) reg       write_toggle_1_meta, write_toggle_1_sync;
(* ASYNC_REG="TRUE" *) reg [7:0] display_mode_wr_2_meta, display_mode_wr_2_sync;
(* ASYNC_REG="TRUE" *) reg       write_toggle_2_meta, write_toggle_2_sync;
(* ASYNC_REG="TRUE" *) reg [7:0] display_mode_wr_3_meta, display_mode_wr_3_sync;
(* ASYNC_REG="TRUE" *) reg       write_toggle_3_meta, write_toggle_3_sync;
(* ASYNC_REG="TRUE" *) reg [7:0] display_mode_wr_4_meta, display_mode_wr_4_sync;
(* ASYNC_REG="TRUE" *) reg       write_toggle_4_meta, write_toggle_4_sync;
reg [7:0] display_mode_wr_1_local, display_mode_wr_2_local, display_mode_wr_3_local;
reg [7:0] display_mode_wr_4_local;
reg       write_tag_1_local, write_tag_2_local, write_tag_3_local, write_tag_4_local;
reg       write_status_clear_0, write_status_clear_1, write_status_clear_2, write_status_clear_3;
reg       cmos1_vsync_capture_d;
wire      cmos1_vsync_capture_rise = cmos1_vsync_d0 && !cmos1_vsync_capture_d;
reg       cmos2_vsync_capture_d;
wire      cmos2_vsync_capture_rise = cmos2_vsync_d0 && !cmos2_vsync_capture_d;

// DDR channel 0 is camera1.  Its mode tag must advance on the camera VS,
// otherwise the stopped HDMI input clock leaves every finished frame tagged
// with the old mode and change_mode rejects it.
always @(posedge cmos1_pclk or negedge camera_capture_reset_n) begin
    if(!camera_capture_reset_n) begin
        display_mode_wr_1_meta <= 8'd7; display_mode_wr_1_sync <= 8'd7;
        write_toggle_1_meta <= 1'b0; write_toggle_1_sync <= 1'b0;
        display_mode_wr_1_local <= 8'd7; write_tag_1_local <= 1'b0;
        write_status_clear_0 <= 1'b1;
    end else begin
        display_mode_wr_1_meta <= display_mode_wr;
        display_mode_wr_1_sync <= display_mode_wr_1_meta;
        write_toggle_1_meta <= write_mode_toggle;
        write_toggle_1_sync <= write_toggle_1_meta;
        write_status_clear_0 <= cmos1_vsync_capture_rise;
        // Apply the new raw/scaled format only at this input's own VS.
        // Comparing the payload as well as the toggle recovers correctly if
        // this pixel clock was stopped long enough to miss an even number of toggles.
        if(cmos1_vsync_capture_rise &&
           (display_mode_wr_1_meta == display_mode_wr_1_sync) &&
           ((write_toggle_1_sync != write_tag_1_local) ||
            (display_mode_wr_1_sync != display_mode_wr_1_local))) begin
            display_mode_wr_1_local <= display_mode_wr_1_sync;
            write_tag_1_local <= write_toggle_1_sync;
            write_status_clear_0 <= 1'b1;
        end
    end
end

always @(posedge cmos1_pclk or negedge camera_capture_reset_n) begin
    if(!camera_capture_reset_n) begin
        display_mode_wr_4_meta <= 8'd7; display_mode_wr_4_sync <= 8'd7;
        write_toggle_4_meta <= 1'b0; write_toggle_4_sync <= 1'b0;
        display_mode_wr_4_local <= 8'd7; write_tag_4_local <= 1'b0;
        write_status_clear_3 <= 1'b1;
        cmos1_vsync_capture_d <= 1'b0;
    end else begin
        cmos1_vsync_capture_d <= cmos1_vsync_d0;
        display_mode_wr_4_meta <= display_mode_wr;
        display_mode_wr_4_sync <= display_mode_wr_4_meta;
        write_toggle_4_meta <= write_mode_toggle;
        write_toggle_4_sync <= write_toggle_4_meta;
        write_status_clear_3 <= cmos1_vsync_capture_rise;
        if(cmos1_vsync_capture_rise &&
           (display_mode_wr_4_meta == display_mode_wr_4_sync) &&
           ((write_toggle_4_sync != write_tag_4_local) ||
            (display_mode_wr_4_sync != display_mode_wr_4_local))) begin
            display_mode_wr_4_local <= display_mode_wr_4_sync;
            write_tag_4_local <= write_toggle_4_sync;
            write_status_clear_3 <= 1'b1;
        end
    end
end

// DDR channel 1 is camera2 and uses its own pixel clock and frame sync.
always @(posedge cmos2_pclk or negedge camera2_capture_reset_n) begin
    if(!camera2_capture_reset_n) begin
        display_mode_wr_2_meta <= 8'd7; display_mode_wr_2_sync <= 8'd7;
        write_toggle_2_meta <= 1'b0; write_toggle_2_sync <= 1'b0;
        display_mode_wr_2_local <= 8'd7; write_tag_2_local <= 1'b0;
        write_status_clear_1 <= 1'b1;
        cmos2_vsync_capture_d <= 1'b0;
    end else begin
        cmos2_vsync_capture_d <= cmos2_vsync_d0;
        display_mode_wr_2_meta <= display_mode_wr;
        display_mode_wr_2_sync <= display_mode_wr_2_meta;
        write_toggle_2_meta <= write_mode_toggle;
        write_toggle_2_sync <= write_toggle_2_meta;
        write_status_clear_1 <= cmos2_vsync_capture_rise;
        if(cmos2_vsync_capture_rise &&
           (display_mode_wr_2_meta == display_mode_wr_2_sync) &&
           ((write_toggle_2_sync != write_tag_2_local) ||
            (display_mode_wr_2_sync != display_mode_wr_2_local))) begin
            display_mode_wr_2_local <= display_mode_wr_2_sync;
            write_tag_2_local <= write_toggle_2_sync;
            write_status_clear_1 <= 1'b1;
        end
    end
end

always @(posedge pixclk_3_in or negedge rstn_out) begin
    if(!rstn_out) begin
        display_mode_wr_3_meta <= 8'd7; display_mode_wr_3_sync <= 8'd7;
        write_toggle_3_meta <= 1'b0; write_toggle_3_sync <= 1'b0;
        display_mode_wr_3_local <= 8'd7; write_tag_3_local <= 1'b0;
        write_status_clear_2 <= 1'b1;
    end else begin
        display_mode_wr_3_meta <= display_mode_wr;
        display_mode_wr_3_sync <= display_mode_wr_3_meta;
        write_toggle_3_meta <= write_mode_toggle;
        write_toggle_3_sync <= write_toggle_3_meta;
        write_status_clear_2 <= hdmi_vs_3_in_d0 && !hdmi_vs_3_in_d1;
        if((hdmi_vs_3_in_d0 && !hdmi_vs_3_in_d1) &&
           (display_mode_wr_3_meta == display_mode_wr_3_sync) &&
           ((write_toggle_3_sync != write_tag_3_local) ||
            (display_mode_wr_3_sync != display_mode_wr_3_local))) begin
            display_mode_wr_3_local <= display_mode_wr_3_sync;
            write_tag_3_local <= write_toggle_3_sync;
            write_status_clear_2 <= 1'b1;
        end
    end
end

// Stereo SFP mode: channel0 = camera1, channel1 = camera2, both always
// writing. The three HDMI inputs are no longer part of the SFP path, so their
// write enables are parked low and the raw/scaled select is unused.
wire ch0_write_enable_local = 1'b1;
wire ch1_write_enable_local = 1'b1;
wire ch2_write_enable_local = 1'b0;
wire ch0_write_full_local = 1'b0;
wire ch1_write_full_local = 1'b0;
wire ch2_write_full_local = 1'b0;
/************************************************************
ddr读写控制（ch0=左眼 / ch1=右眼 / ch2、ch3 停用）
************************************************************/
wire    ch0_clk;
wire    ch1_clk;
wire    ch2_clk;
wire    ch3_clk;


//通道一 camera1 (left eye) - the scaled stereo channel feeding the SFP link
assign ch0_write_data = cam1_scale_data;
assign ch0_write_en   = cam1_scale_de;
assign ch0_clk        = cmos1_pclk;

//通道二 camera2 (right eye)
assign ch1_write_data = cam2_scale_data;
assign ch1_write_en   = cam2_scale_de;
assign ch1_clk        = cmos2_pclk;

//通道三 HDMI_2 - not part of the stereo SFP path, kept but disabled
assign ch2_write_data = 16'd0;
assign ch2_write_en   = 1'b0;
assign ch2_clk        = pixclk_3_in;

// Channel 4 (the single OV5640 path) is dead in stereo mode. The whole
// camera_channel4_select + 960x540 scaler chain is removed instead of being
// parked: leaving it in place made its combinational outputs constant-fold
// onto the global GND net and PDS reported
// "Net _$$_GND_$$_ has multiple drivers". Channel 3 stays disabled through
// ch3_write_len_cfg = 0.
assign ch3_write_data = 16'd0;
assign ch3_write_en   = 1'b0;
assign ch3_clk        = cmos1_pclk;

localparam [24:0] FRAME_LEN_FULL   = 25'd129600;
localparam [24:0] FRAME_LEN_SCALED = 25'd32400;
localparam [24:0] FRAME_LEN_CAMERA = 25'd57600;
// 640x360 RGB565 stereo cell: 640*360/16 = 14400 128-bit words.
localparam [24:0] FRAME_LEN_STEREO = 25'd14400;

wire [7:0] hdmi_display_mode;
wire [3:0] hdmi_channel_valid;
wire       hdmi_switch_blank;
wire [7:0] hdmi_frame_index;
wire [15:0] hdmi_video_data;
wire hdmi_video_hs, hdmi_video_vs, hdmi_video_de;
wire [3:0] hdmi_read_underflow_sticky;

// 本地 HDMI 渲染通路置空：channel_valid=0 时 display_renderer_4ch 输出黑屏，
// 仅维持 pix_clk 域时序（原工程结构保留）。
assign hdmi_display_mode = 8'd1;
assign hdmi_channel_valid = 4'd0;
assign hdmi_switch_blank = 1'b1;
assign hdmi_frame_index = 8'd0;

// Stereo SFP mode uses a fixed geometry: two 640x360 channels, nothing else.
wire [24:0] ch0_write_len_cfg = FRAME_LEN_STEREO;
wire [24:0] ch1_write_len_cfg = FRAME_LEN_STEREO;
wire [24:0] ch2_write_len_cfg = 25'd0;
wire [24:0] ch3_write_len_cfg = 25'd0;
wire [24:0] ch0_read_len_cfg  = FRAME_LEN_STEREO;
wire [24:0] ch1_read_len_cfg  = FRAME_LEN_STEREO;
wire [24:0] ch2_read_len_cfg  = 25'd0;
wire [24:0] ch3_read_len_cfg  = 25'd0;
wire [24:0] ch4_read_len_cfg  = hdmi_channel_valid[0] ? ((hdmi_display_mode == 8'd1) ? FRAME_LEN_FULL : FRAME_LEN_SCALED) : 25'd0;
wire [24:0] ch5_read_len_cfg  = hdmi_channel_valid[1] ? ((hdmi_display_mode == 8'd2) ? FRAME_LEN_FULL : FRAME_LEN_SCALED) : 25'd0;
wire [24:0] ch6_read_len_cfg  = hdmi_channel_valid[2] ? ((hdmi_display_mode == 8'd3) ? FRAME_LEN_FULL : FRAME_LEN_SCALED) : 25'd0;
wire [24:0] ch7_read_len_cfg  = hdmi_channel_valid[3] ? ((hdmi_display_mode == 8'd4) ? FRAME_LEN_CAMERA : FRAME_LEN_SCALED) : 25'd0;

wire ch0_read_fifo_empty, ch1_read_fifo_empty, ch2_read_fifo_empty, ch3_read_fifo_empty       /* synthesis PAP_MARK_DEBUG="true" */;
wire ch0_read_fifo_full, ch1_read_fifo_full, ch2_read_fifo_full, ch3_read_fifo_full          /* synthesis PAP_MARK_DEBUG="true" */;
wire ch0_write_fifo_full, ch1_write_fifo_full, ch2_write_fifo_full, ch3_write_fifo_full       /* synthesis PAP_MARK_DEBUG="true" */;
wire ch0_read_overflow_sticky  /* synthesis PAP_MARK_DEBUG="true" */;
wire ch1_read_overflow_sticky  /* synthesis PAP_MARK_DEBUG="true" */;
wire ch2_read_overflow_sticky  /* synthesis PAP_MARK_DEBUG="true" */;
wire ch3_read_overflow_sticky  /* synthesis PAP_MARK_DEBUG="true" */;
wire ch0_write_overflow_sticky /* synthesis PAP_MARK_DEBUG="true" */;
wire ch1_write_overflow_sticky /* synthesis PAP_MARK_DEBUG="true" */;
wire ch2_write_overflow_sticky /* synthesis PAP_MARK_DEBUG="true" */;
wire ch3_write_overflow_sticky /* synthesis PAP_MARK_DEBUG="true" */;
wire ch0_frame_done_toggle, ch1_frame_done_toggle, ch2_frame_done_toggle, ch3_frame_done_toggle;
wire ch0_frame_done_tag, ch1_frame_done_tag, ch2_frame_done_tag, ch3_frame_done_tag;
wire [1:0] ch0_complete_addr_index, ch1_complete_addr_index, ch2_complete_addr_index, ch3_complete_addr_index /* synthesis PAP_MARK_DEBUG="true" */;

assign frame_done_toggle_bus = {ch3_frame_done_toggle,ch2_frame_done_toggle,ch1_frame_done_toggle,ch0_frame_done_toggle};
assign frame_done_tag_bus    = {ch3_frame_done_tag,ch2_frame_done_tag,ch1_frame_done_tag,ch0_frame_done_tag};
assign frame_done_index_bus  = {ch3_complete_addr_index,ch2_complete_addr_index,
                                ch1_complete_addr_index,ch0_complete_addr_index};

// K5 startup-stage indication: fast blink before DDR PLL lock, slow blink
// during calibration, solid after calibration while waiting for the first
// complete frame, then a heartbeat once video traffic is alive.
(* ASYNC_REG="TRUE" *) reg ddr_pll_lock_sys_meta, ddr_pll_lock_sys;
(* ASYNC_REG="TRUE" *) reg ddr_init_sys_meta, ddr_init_sys;
(* ASYNC_REG="TRUE" *) reg [3:0] frame_done_sys_meta, frame_done_sys;
reg [3:0] frame_done_sys_seen;
reg       first_complete_frame_sys;
reg [24:0] startup_debug_counter;

always @(posedge sys_clk or negedge rst_n) begin
    if(!rst_n) begin
        ddr_pll_lock_sys_meta   <= 1'b0;
        ddr_pll_lock_sys        <= 1'b0;
        ddr_init_sys_meta       <= 1'b0;
        ddr_init_sys            <= 1'b0;
        frame_done_sys_meta     <= 4'b0000;
        frame_done_sys          <= 4'b0000;
        frame_done_sys_seen     <= 4'b0000;
        first_complete_frame_sys<= 1'b0;
        startup_debug_counter   <= 25'd0;
    end
    else begin
        ddr_pll_lock_sys_meta <= pll_lock;
        ddr_pll_lock_sys      <= ddr_pll_lock_sys_meta;
        ddr_init_sys_meta     <= ddr_init_done;
        ddr_init_sys          <= ddr_init_sys_meta;
        frame_done_sys_meta   <= frame_done_toggle_bus;
        frame_done_sys        <= frame_done_sys_meta;
        frame_done_sys_seen   <= frame_done_sys;
        startup_debug_counter <= startup_debug_counter + 1'b1;

        if(!ddr_init_sys)
            first_complete_frame_sys <= 1'b0;
        else if(|(frame_done_sys ^ frame_done_sys_seen))
            first_complete_frame_sys <= 1'b1;
    end
end

assign debug_led2 = !system_startup_done      ? 1'b0 :
                    !ddr_pll_lock_sys         ? startup_debug_counter[20] :
                    !ddr_init_sys             ? startup_debug_counter[22] :
                    !first_complete_frame_sys ? 1'b1 :
                                                startup_debug_counter[24];

frame_read_write
#
(
	.MEM_DATA_BITS              (256                      ),
	.READ_DATA_BITS             (16                       ),
	.WRITE_DATA_BITS            (16                       ),
	.ADDR_BITS                  (25                       ),
	.BUSRT_BITS                 (10                       ),
	.BURST_SIZE                 (16                       ) //?
)
frame_read_write_m0
(
	.rst                        (~ddr_init_done           ),
	.mem_clk                    (core_clk                 ),
	.rd_burst_req               (ch0_rd_burst_req             ),
	.rd_burst_len               (ch0_rd_burst_len             ),
	.rd_burst_addr              (ch0_rd_burst_addr            ),
	.rd_burst_data_valid        (ch0_rd_burst_data_valid      ),
	.rd_burst_data              (ch0_rd_burst_data            ),
	.rd_burst_finish            (ch0_rd_burst_finish          ),
	.read_clk                   (tx_clk                  ),
	.read_req                   (ch0_read_req            ),
	.read_req_ack               (ch0_read_req_ack         ),
	.read_finish                (                         ),
	.read_addr_0                (25'd0                    ), //The first frame address is 0
	.read_addr_1                (25'd2073600              ), //The second frame address is 24'd2073600 ,large enough address space for one frame of video
	.read_addr_2                (25'd4147200              ),
	.read_addr_3                (25'd6220800              ),
	// Read the bank this channel finished writing last. Using the channel's own
	// completion index keeps camera2 correct even though change_mode only
	// publishes indices for the channels its mode mask selects.
	.read_addr_index            (ch0_complete_addr_index  ),
	.read_len                   (ch0_read_len_cfg         ),
	.read_en                    (ch0_read_en              ),
	.read_data                  (ch0_read_data            ),
	.read_fifo_empty            (ch0_read_fifo_empty      ),
	.read_fifo_full             (ch0_read_fifo_full       ),
	.read_status_clear          (switch_blank || v2_vs    ),
	.read_overflow_sticky       (ch0_read_overflow_sticky ),

	.wr_burst_req               (ch0_wr_burst_req             ),
	.wr_burst_len               (ch0_wr_burst_len             ),
	.wr_burst_addr              (ch0_wr_burst_addr            ),
	.wr_burst_data_req          (ch0_wr_burst_data_req        ),
	.wr_burst_data              (ch0_wr_burst_data            ),
	.wr_burst_finish            (ch0_wr_burst_finish          ),
	.write_clk                  (ch0_clk                     ),
	.write_req                  (ch0_write_req            ),
	.write_req_ack              (ch0_write_req_ack        ),
	.write_finish               (                         ),
	.write_addr_0               (25'd0                    ),
	.write_addr_1               (25'd2073600              ),
	.write_addr_2               (25'd4147200              ),
	.write_addr_3               (25'd6220800              ),
	.write_addr_index           (ch0_write_addr_index     ),
	.write_tag                  (write_tag_1_local        ),
	.write_len                  (ch0_write_len_cfg        ),
	.write_en                   (ch0_write_en             ),
	.write_data                 (ch0_write_data           ),
	.write_status_clear         (write_status_clear_0     ),
	.write_frame_error          (1'b0                    ),
	.write_frame_valid          (1'b1                    ),
	.write_frame_valid_toggle   (1'b0                    ),
	.write_validation_required  (1'b0                    ),
	.write_overflow_sticky      (ch0_write_overflow_sticky),
	.write_fifo_full            (ch0_write_fifo_full      ),
	.write_finish_toggle        (ch0_frame_done_toggle    ),
	.write_finish_tag           (ch0_frame_done_tag       ),
	.write_finish_index         (ch0_complete_addr_index  )
);
//通道二
frame_read_write#(
	.MEM_DATA_BITS              (256                      ),
	.READ_DATA_BITS             (16                       ),
	.WRITE_DATA_BITS            (16                       ),
	.ADDR_BITS                  (25                       ),
	.BUSRT_BITS                 (10                       ),
	.BURST_SIZE                 (16                       ) //?
) frame_read_write_m1
(
	.rst                        (~ddr_init_done           ),
	.mem_clk                    (core_clk                 ),
	.rd_burst_req               (ch1_rd_burst_req         ),
	.rd_burst_len               (ch1_rd_burst_len         ),
	.rd_burst_addr              (ch1_rd_burst_addr        ),
	.rd_burst_data_valid        (ch1_rd_burst_data_valid  ),
	.rd_burst_data              (ch1_rd_burst_data        ),
	.rd_burst_finish            (ch1_rd_burst_finish      ),
	.read_clk                   (tx_clk                  ),
	.read_req                   (ch1_read_req             ),
	.read_req_ack               (ch1_read_req_ack         ),
	.read_finish                (                         ),
	.read_addr_0                (25'd8294400              ), //The first frame address is 0
	.read_addr_1                (25'd10368000             ), //The second frame address is 25'd2073600 ,large enough address space for one frame of video
	.read_addr_2                (25'd12441600             ),
	.read_addr_3                (25'd14515200             ),
	.read_addr_index            (ch1_complete_addr_index  ),
	.read_len                   (ch1_read_len_cfg         ),
	.read_en                    (ch1_read_en              ),
	.read_data                  (ch1_read_data            ),
	.read_fifo_empty            (ch1_read_fifo_empty      ),
	.read_fifo_full             (ch1_read_fifo_full       ),
	.read_status_clear          (switch_blank || v2_vs    ),
	.read_overflow_sticky       (ch1_read_overflow_sticky ),

	.wr_burst_req               (ch1_wr_burst_req         ),
	.wr_burst_len               (ch1_wr_burst_len         ),
	.wr_burst_addr              (ch1_wr_burst_addr        ),
	.wr_burst_data_req          (ch1_wr_burst_data_req    ),
	.wr_burst_data              (ch1_wr_burst_data        ),
	.wr_burst_finish            (ch1_wr_burst_finish      ),
	.write_clk                  (ch1_clk                ),
	.write_req                  (ch1_write_req            ),
	.write_req_ack              (ch1_write_req_ack        ),
	.write_finish               (                         ),
	.write_addr_0               (25'd8294400               ),
	.write_addr_1               (25'd10368000             ),
	.write_addr_2               (25'd12441600             ),
	.write_addr_3               (25'd14515200             ),
	.write_addr_index           (ch1_write_addr_index     ),
	.write_tag                  (write_tag_2_local        ),
	.write_len                  (ch1_write_len_cfg        ),
	.write_en                   (ch1_write_en             ),
	.write_data                 (ch1_write_data           ),
	.write_status_clear         (write_status_clear_1     ),
	.write_frame_error          (1'b0                    ),
	.write_frame_valid          (1'b1                    ),
	.write_frame_valid_toggle   (1'b0                    ),
	.write_validation_required  (1'b0                    ),
	.write_overflow_sticky      (ch1_write_overflow_sticky),
	.write_fifo_full            (ch1_write_fifo_full      ),
	.write_finish_toggle        (ch1_frame_done_toggle    ),
	.write_finish_tag           (ch1_frame_done_tag       ),
	.write_finish_index         (ch1_complete_addr_index  )
);

//通道三
frame_read_write#(
	.MEM_DATA_BITS              (256                      ),
	.READ_DATA_BITS             (16                       ),
	.WRITE_DATA_BITS            (16                       ),
	.ADDR_BITS                  (25                       ),
	.BUSRT_BITS                 (10                       ),
	.BURST_SIZE                 (16                       ) //?
) frame_read_write_m2
(
	.rst                        (~ddr_init_done           ),
	.mem_clk                    (core_clk                 ),
	.rd_burst_req               (ch2_rd_burst_req         ),
	.rd_burst_len               (ch2_rd_burst_len         ),
	.rd_burst_addr              (ch2_rd_burst_addr        ),
	.rd_burst_data_valid        (ch2_rd_burst_data_valid  ),
	.rd_burst_data              (ch2_rd_burst_data        ),
	.rd_burst_finish            (ch2_rd_burst_finish      ),
	.read_clk                   (tx_clk                  ),
	.read_req                   (ch2_read_req             ),
	.read_req_ack               (ch2_read_req_ack         ),
	.read_finish                (                         ),
	.read_addr_0                (25'd16588800             ),
	.read_addr_1                (25'd18662400             ),
	.read_addr_2                (25'd20736000             ),
	.read_addr_3                (25'd22809600             ),
	.read_addr_index            (published_frame_index[5:4]),
	.read_len                   (ch2_read_len_cfg         ),
	.read_en                    (ch2_read_en              ),
	.read_data                  (ch2_read_data            ),
	.read_fifo_empty            (ch2_read_fifo_empty      ),
	.read_fifo_full             (ch2_read_fifo_full       ),
	.read_status_clear          (switch_blank || v2_vs    ),
	.read_overflow_sticky       (ch2_read_overflow_sticky ),

	.wr_burst_req               (ch2_wr_burst_req         ),
	.wr_burst_len               (ch2_wr_burst_len         ),
	.wr_burst_addr              (ch2_wr_burst_addr        ),
	.wr_burst_data_req          (ch2_wr_burst_data_req    ),
	.wr_burst_data              (ch2_wr_burst_data        ),
	.wr_burst_finish            (ch2_wr_burst_finish      ),
	.write_clk                  (ch2_clk                ),
	.write_req                  (ch2_write_req            ),
	.write_req_ack              (ch2_write_req_ack        ),
	.write_finish               (                         ),
	.write_addr_0               (25'd16588800             ),
	.write_addr_1               (25'd18662400             ),
	.write_addr_2               (25'd20736000             ),
	.write_addr_3               (25'd22809600             ),
	.write_addr_index           (ch2_write_addr_index     ),
	.write_tag                  (write_tag_3_local        ),
	.write_len                  (ch2_write_len_cfg        ),
	.write_en                   (ch2_write_en             ),
	.write_data                 (ch2_write_data           ),
	.write_status_clear         (write_status_clear_2     ),
	.write_frame_error          (1'b0                    ),
	.write_frame_valid          (1'b1                    ),
	.write_frame_valid_toggle   (1'b0                    ),
	.write_validation_required  (1'b0                    ),
	.write_overflow_sticky      (ch2_write_overflow_sticky),
	.write_fifo_full            (ch2_write_fifo_full      ),
	.write_finish_toggle        (ch2_frame_done_toggle    ),
	.write_finish_tag           (ch2_frame_done_tag       ),
	.write_finish_index         (ch2_complete_addr_index  )
);

frame_read_write#(
	.MEM_DATA_BITS              (256                      ),
	.READ_DATA_BITS             (16                       ),
	.WRITE_DATA_BITS            (16                       ),
	.ADDR_BITS                  (25                       ),
	.BUSRT_BITS                 (10                       ),
	.BURST_SIZE                 (16                       )
) frame_read_write_m3
(
	.rst                        (~ddr_init_done || !camera_capture_reset_n),
	.mem_clk                    (core_clk                 ),
	.rd_burst_req               (ch3_rd_burst_req         ),
	.rd_burst_len               (ch3_rd_burst_len         ),
	.rd_burst_addr              (ch3_rd_burst_addr        ),
	.rd_burst_data_valid        (ch3_rd_burst_data_valid  ),
	.rd_burst_data              (ch3_rd_burst_data        ),
	.rd_burst_finish            (ch3_rd_burst_finish      ),
	.read_clk                   (tx_clk                   ),
	.read_req                   (ch3_read_req             ),
	.read_req_ack               (ch3_read_req_ack         ),
	.read_finish                (                          ),
	.read_addr_0                (25'd24883200             ),
	.read_addr_1                (25'd26956800             ),
	.read_addr_2                (25'd29030400             ),
	.read_addr_3                (25'd31104000             ),
	.read_addr_index            (published_frame_index[7:6]),
	.read_len                   (ch3_read_len_cfg         ),
	.read_en                    (ch3_read_en              ),
	.read_data                  (ch3_read_data            ),
	.read_fifo_empty            (ch3_read_fifo_empty      ),
	.read_fifo_full             (ch3_read_fifo_full       ),
	.read_status_clear          (switch_blank || v2_vs    ),
	.read_overflow_sticky       (ch3_read_overflow_sticky ),

	.wr_burst_req               (ch3_wr_burst_req         ),
	.wr_burst_len               (ch3_wr_burst_len         ),
	.wr_burst_addr              (ch3_wr_burst_addr        ),
	.wr_burst_data_req          (ch3_wr_burst_data_req    ),
	.wr_burst_data              (ch3_wr_burst_data        ),
	.wr_burst_finish            (ch3_wr_burst_finish      ),
	.write_clk                  (ch3_clk                  ),
	.write_req                  (ch3_write_req            ),
	.write_req_ack              (ch3_write_req_ack        ),
	.write_finish               (                          ),
	.write_addr_0               (25'd24883200             ),
	.write_addr_1               (25'd26956800             ),
	.write_addr_2               (25'd29030400             ),
	.write_addr_3               (25'd31104000             ),
	.write_addr_index           (ch3_write_addr_index     ),
	.write_tag                  (write_tag_4_local        ),
	.write_len                  (ch3_write_len_cfg        ),
	.write_en                   (ch3_write_en             ),
	.write_data                 (ch3_write_data           ),
	.write_status_clear         (write_status_clear_3     ),
	.write_frame_error          (camera_frame_error       ),
	.write_frame_valid          (camera_frame_valid       ),
	.write_frame_valid_toggle   (camera_frame_valid_toggle),
	.write_validation_required  (1'b1                    ),
	.write_overflow_sticky      (ch3_write_overflow_sticky),
	.write_fifo_full            (ch3_write_fifo_full      ),
	.write_finish_toggle        (ch3_frame_done_toggle    ),
	.write_finish_tag           (ch3_frame_done_tag       ),
	.write_finish_index         (ch3_complete_addr_index  )
);

// The local HDMI renderer owns four independent asynchronous read FIFOs. It
// reuses the same sixteen frame buffers but never samples the SFP pixel domain.
frame_read_only #(.MEM_DATA_BITS(256), .READ_DATA_BITS(16), .ADDR_BITS(25), .BUSRT_BITS(10), .BURST_SIZE(16))
frame_read_only_hdmi0 (
    .rst(~ddr_init_done), .mem_clk(core_clk),
    .rd_burst_req(ch4_rd_burst_req), .rd_burst_len(ch4_rd_burst_len), .rd_burst_addr(ch4_rd_burst_addr),
    .rd_burst_data_valid(ch4_rd_burst_data_valid), .rd_burst_data(ch4_rd_burst_data), .rd_burst_finish(ch4_rd_burst_finish),
    .read_clk(pix_clk), .read_req(ch4_read_req), .read_req_ack(ch4_read_req_ack), .read_finish(),
    .read_addr_0(25'd0), .read_addr_1(25'd2073600), .read_addr_2(25'd4147200), .read_addr_3(25'd6220800),
    .read_addr_index(hdmi_frame_index[1:0]), .read_len(ch4_read_len_cfg),
    .read_en(ch4_read_en), .read_data(ch4_read_data), .read_fifo_empty(ch4_read_fifo_empty), .read_fifo_full(ch4_read_fifo_full),
    .read_status_clear(hdmi_switch_blank || hdmi_video_vs), .read_overflow_sticky(ch4_read_overflow_sticky)
);

frame_read_only #(.MEM_DATA_BITS(256), .READ_DATA_BITS(16), .ADDR_BITS(25), .BUSRT_BITS(10), .BURST_SIZE(16))
frame_read_only_hdmi1 (
    .rst(~ddr_init_done), .mem_clk(core_clk),
    .rd_burst_req(ch5_rd_burst_req), .rd_burst_len(ch5_rd_burst_len), .rd_burst_addr(ch5_rd_burst_addr),
    .rd_burst_data_valid(ch5_rd_burst_data_valid), .rd_burst_data(ch5_rd_burst_data), .rd_burst_finish(ch5_rd_burst_finish),
    .read_clk(pix_clk), .read_req(ch5_read_req), .read_req_ack(ch5_read_req_ack), .read_finish(),
    .read_addr_0(25'd8294400), .read_addr_1(25'd10368000), .read_addr_2(25'd12441600), .read_addr_3(25'd14515200),
    .read_addr_index(hdmi_frame_index[3:2]), .read_len(ch5_read_len_cfg),
    .read_en(ch5_read_en), .read_data(ch5_read_data), .read_fifo_empty(ch5_read_fifo_empty), .read_fifo_full(ch5_read_fifo_full),
    .read_status_clear(hdmi_switch_blank || hdmi_video_vs), .read_overflow_sticky(ch5_read_overflow_sticky)
);

frame_read_only #(.MEM_DATA_BITS(256), .READ_DATA_BITS(16), .ADDR_BITS(25), .BUSRT_BITS(10), .BURST_SIZE(16))
frame_read_only_hdmi2 (
    .rst(~ddr_init_done), .mem_clk(core_clk),
    .rd_burst_req(ch6_rd_burst_req), .rd_burst_len(ch6_rd_burst_len), .rd_burst_addr(ch6_rd_burst_addr),
    .rd_burst_data_valid(ch6_rd_burst_data_valid), .rd_burst_data(ch6_rd_burst_data), .rd_burst_finish(ch6_rd_burst_finish),
    .read_clk(pix_clk), .read_req(ch6_read_req), .read_req_ack(ch6_read_req_ack), .read_finish(),
    .read_addr_0(25'd16588800), .read_addr_1(25'd18662400), .read_addr_2(25'd20736000), .read_addr_3(25'd22809600),
    .read_addr_index(hdmi_frame_index[5:4]), .read_len(ch6_read_len_cfg),
    .read_en(ch6_read_en), .read_data(ch6_read_data), .read_fifo_empty(ch6_read_fifo_empty), .read_fifo_full(ch6_read_fifo_full),
    .read_status_clear(hdmi_switch_blank || hdmi_video_vs), .read_overflow_sticky(ch6_read_overflow_sticky)
);

frame_read_only #(.MEM_DATA_BITS(256), .READ_DATA_BITS(16), .ADDR_BITS(25), .BUSRT_BITS(10), .BURST_SIZE(16))
frame_read_only_hdmi3 (
    .rst(~ddr_init_done), .mem_clk(core_clk),
    .rd_burst_req(ch7_rd_burst_req), .rd_burst_len(ch7_rd_burst_len), .rd_burst_addr(ch7_rd_burst_addr),
    .rd_burst_data_valid(ch7_rd_burst_data_valid), .rd_burst_data(ch7_rd_burst_data), .rd_burst_finish(ch7_rd_burst_finish),
    .read_clk(pix_clk), .read_req(ch7_read_req), .read_req_ack(ch7_read_req_ack), .read_finish(),
    .read_addr_0(25'd24883200), .read_addr_1(25'd26956800), .read_addr_2(25'd29030400), .read_addr_3(25'd31104000),
    .read_addr_index(hdmi_frame_index[7:6]), .read_len(ch7_read_len_cfg),
    .read_en(ch7_read_en), .read_data(ch7_read_data), .read_fifo_empty(ch7_read_fifo_empty), .read_fifo_full(ch7_read_fifo_full),
    .read_status_clear(hdmi_switch_blank || hdmi_video_vs), .read_overflow_sticky(ch7_read_overflow_sticky)
);

display_renderer_4ch u_hdmi_renderer (
    .video_clk(pix_clk), .rst(~rstn_out),
    .display_mode(hdmi_display_mode), .channel_valid(hdmi_channel_valid), .switch_blank(hdmi_switch_blank),
    .ch0_read_req(ch4_read_req), .ch0_read_req_ack(ch4_read_req_ack), .ch0_read_en(ch4_read_en),
    .ch0_read_data(ch4_read_data), .ch0_read_empty(ch4_read_fifo_empty),
    .ch1_read_req(ch5_read_req), .ch1_read_req_ack(ch5_read_req_ack), .ch1_read_en(ch5_read_en),
    .ch1_read_data(ch5_read_data), .ch1_read_empty(ch5_read_fifo_empty),
    .ch2_read_req(ch6_read_req), .ch2_read_req_ack(ch6_read_req_ack), .ch2_read_en(ch6_read_en),
    .ch2_read_data(ch6_read_data), .ch2_read_empty(ch6_read_fifo_empty),
    .ch3_read_req(ch7_read_req), .ch3_read_req_ack(ch7_read_req_ack), .ch3_read_en(ch7_read_en),
    .ch3_read_data(ch7_read_data), .ch3_read_empty(ch7_read_fifo_empty),
    .read_underflow_sticky(hdmi_read_underflow_sticky),
    .video_data(hdmi_video_data), .video_hs(hdmi_video_hs), .video_vs(hdmi_video_vs), .video_de(hdmi_video_de)
);

/************************************************************
视频输出选择通道
************************************************************/
always @(posedge pix_clk or negedge rstn_out) begin
    if(!rstn_out) begin
        r_out  <= 8'd0;
        g_out  <= 8'd0;
        b_out  <= 8'd0;
        vs_out <= 1'b0;
        hs_out <= 1'b0;
        de_out <= 1'b0;
    end
    else begin
        r_out  <= {hdmi_video_data[15:11],3'b000};
        g_out  <= {hdmi_video_data[10:5],2'b00};
        b_out  <= {hdmi_video_data[4:0],3'b000};
        vs_out <= hdmi_video_vs;
        hs_out <= hdmi_video_hs;
        de_out <= hdmi_video_de;
    end
end


/************************************************************
产生坐标和时序 联合使用
************************************************************/
//产生色彩叠加
wire [7:0]    color_bar_r;
wire [7:0]    color_bar_g;
wire [7:0]    color_bar_b;
wire [15:0]   color_bar_rgb565 = {color_bar_r[7:3],color_bar_g[7:2],color_bar_b[7:3]};
wire [15:0]   v0_data     ;
wire          v0_hs        ;
wire          v0_vs        ;
wire          v0_de        ;



// The mode controller commits all geometry at an output VS edge.  Invalid or
// missing sources keep their assigned region black until a complete frame is ready.
// Stereo SFP layout: camera1 at (0,0), camera2 at (640,0), each 640x360,
// inside one 1280x360 SFP frame. Channels 2/3 are switched off (zero size).
wire [11:0] video_left_offset_0 = 12'd0;
wire [11:0] video_top_offset_0  = 12'd0;
wire [11:0] video_left_offset_1 = 12'd640;
wire [11:0] video_top_offset_1  = 12'd0;
wire [11:0] video_left_offset_2 = 12'd0;
wire [11:0] video_top_offset_2  = 12'd0;
wire [11:0] video_left_offset_3 = 12'd0;
wire [11:0] video_top_offset_3  = 12'd0;
wire [11:0] video_width_0  = 12'd640;
wire [11:0] video_height_0 = 12'd360;
wire [11:0] video_width_1  = 12'd640;
wire [11:0] video_height_1 = 12'd360;
wire [11:0] video_width_2  = 12'd0;
wire [11:0] video_height_2 = 12'd0;
wire [11:0] video_width_3  = 12'd0;
wire [11:0] video_height_3 = 12'd0;
    

//display_mode;
color_bar color_bar_m0(
	.clk                        (tx_clk                ),
	.rst                        (~rstn_out               ),
	.hs                         (color_bar_hs             ),
	.vs                         (color_bar_vs             ),
	.de                         (color_bar_de             ),
	.rgb_r                      (color_bar_r              ),
	.rgb_g                      (color_bar_g              ),
	.rgb_b                      (color_bar_b              )
);

// The SFP link gets its own 1280x360 canvas instead of the 1920x1080 HDMI
// one, so only the stitched stereo frame travels over the fiber. The local
// HDMI output keeps the full 1920x1080 timing untouched.
wire        sfp_timing_hs;
wire        sfp_timing_vs;
wire        sfp_timing_de;
wire [15:0] sfp_timing_x;
wire [15:0] sfp_timing_y;

sfp_timing_gen #(
    .H_ACTIVE (16'd1280),
    .V_ACTIVE (16'd360)
) u_sfp_timing_gen (
    .clk   (tx_clk         ),
    .rst   (~rstn_out      ),
    .hs    (sfp_timing_hs  ),
    .vs    (sfp_timing_vs  ),
    .de    (sfp_timing_de  ),
    .x_cnt (sfp_timing_x   ),
    .y_cnt (sfp_timing_y   )
);
//读写偏移请求
//generate a frame read data request
wire ch0_read_underflow_sticky /* synthesis PAP_MARK_DEBUG="true" */;
wire ch1_read_underflow_sticky /* synthesis PAP_MARK_DEBUG="true" */;
wire ch2_read_underflow_sticky /* synthesis PAP_MARK_DEBUG="true" */;
wire ch3_read_underflow_sticky /* synthesis PAP_MARK_DEBUG="true" */;
video_rect_read_data video_rect_read_data_m0
(
	.video_clk                  (tx_clk                ),
	.rst                        (~rstn_out               ),
	.video_left_offset          (video_left_offset_0    ),
	.video_top_offset           (video_top_offset_0    ),
	.video_width                (video_width_0            ),
	.video_height	            (video_height_0          ),
	.read_req                   (ch0_read_req             ),
	.read_req_ack               (ch0_read_req_ack         ),
	.read_en                    (ch0_read_en              ),
	.read_data                  (ch0_read_data            ),
	.read_empty                 (ch0_read_fifo_empty      ),
	.status_clear               (switch_blank             ),
	.underflow_sticky           (ch0_read_underflow_sticky),
	.timing_hs                  (sfp_timing_hs            ),
	.timing_vs                  (sfp_timing_vs            ),
	.timing_de                  (sfp_timing_de            ),
	.timing_data 	            (ddr_ready_tx ? 16'h0000 : color_bar_rgb565),
	.hs                         (v0_hs                    ),
	.vs                         (v0_vs                    ),
	.de                         (v0_de                    ),
	.vout_data                  (v0_data                  )
);
//通道二
wire    [15:0]    v1_data;
wire    v1_hs;
wire    v1_vs;
wire    v1_de;

video_rect_read_data video_rect_read_data_m1
(
	.video_clk                  (tx_clk                ),
	.rst                        (~rstn_out               ),
	.video_left_offset          (video_left_offset_1    ),
	.video_top_offset           (video_top_offset_1    ),
	.video_width                (video_width_1           ),
	.video_height	            (video_height_1        ),
	.read_req                   (ch1_read_req             ),
	.read_req_ack               (ch1_read_req_ack         ),
	.read_en                    (ch1_read_en              ),
	.read_data                  (ch1_read_data            ),
	.read_empty                 (ch1_read_fifo_empty      ),
	.status_clear               (switch_blank             ),
	.underflow_sticky           (ch1_read_underflow_sticky),
	.timing_hs                  (v0_hs                    ),
	.timing_vs                  (v0_vs                    ),
	.timing_de                  (v0_de                    ),
	.timing_data 	            (v0_data                  ),
	.hs                         (v1_hs                       ),
	.vs                         (v1_vs                       ),
	.de                         (v1_de                       ),
	.vout_data                  (v1_data                )
);
//通道三
wire    [15:0]    v2_data;
wire    v2_hs;
wire    v2_vs;
wire    v2_de;
video_rect_read_data video_rect_read_data_m2
(
	.video_clk                  (tx_clk                ),
	.rst                        (~rstn_out               ),
	.video_left_offset          (video_left_offset_2  ),
	.video_top_offset           (video_top_offset_2   ),
	.video_width                (video_width_2            ),
	.video_height	            (video_height_2           ),
	.read_req                   (ch2_read_req             ),
	.read_req_ack               (ch2_read_req_ack         ),
	.read_en                    (ch2_read_en              ),
	.read_data                  (ch2_read_data            ),
	.read_empty                 (ch2_read_fifo_empty      ),
	.status_clear               (switch_blank             ),
	.underflow_sticky           (ch2_read_underflow_sticky),
	.timing_hs                  (v1_hs                    ),
	.timing_vs                  (v1_vs                    ),
	.timing_de                  (v1_de                    ),
	.timing_data 	            (v1_data                  ),
	.hs                         (v2_hs                       ),
	.vs                         (v2_vs                       ),
	.de                         (v2_de                       ),
	.vout_data                  (v2_data                )
);
assign v2_safe_data = switch_blank ? 16'h0000 : v2_data;

//通道四
video_rect_read_data video_rect_read_data_m3
(
	.video_clk                  (tx_clk                 ),
	.rst                        (~rstn_out               ),
	.video_left_offset          (video_left_offset_3    ),
	.video_top_offset           (video_top_offset_3     ),
	.video_width                (video_width_3          ),
	.video_height	            (video_height_3         ),
	.read_req                   (ch3_read_req             ),
	.read_req_ack               (ch3_read_req_ack         ),
	.read_en                    (ch3_read_en              ),
	.read_data                  (ch3_read_data            ),
	.read_empty                 (ch3_read_fifo_empty      ),
	.status_clear               (switch_blank             ),
	.underflow_sticky           (ch3_read_underflow_sticky),
	.timing_hs                  (v2_hs                    ),
	.timing_vs                  (v2_vs                    ),
	.timing_de                  (v2_de                    ),
	.timing_data 	            (v2_safe_data             ),
	.x_cnt                      (                         ),
	.y_cnt                      (                         ),
	.hs                         (hs                       ),
	.vs                         (vs                       ),
	.de                         (de                       ),
	.vout_data                  (vout_data                )
);

// DDR failure remains visible as a legal color-bar video stream. The timing
// generator and packetizer continue to run even while all frame FIFOs reset.
sfp_video_fallback u_sfp_video_fallback (
    .ddr_ready      (ddr_ready_tx),
    .has_valid_frame(|display_channel_valid),
    .switch_blank   (switch_blank),
    .color_bar_data (color_bar_rgb565),
    .composed_data  (vout_data),
    .output_data    (sfp_output_data)
);

/************************************************************
SFP 发送：1280x360 拼接帧 -> video_packet_send -> HSST lane2 TX
（DDR 未就绪时由 sfp_video_fallback 送彩条，保证链路持续有码流）
************************************************************/
video_packet_send video_packet_send_m0
(
	.rst                        (~rstn_out                   ),
	.tx_clk                     (tx_clk                      ),

	.pclk                       (tx_clk                   ),
	.vs                         (vs                       ),
	.de                         (de                       ),
	.vin_data                   (sfp_output_data          ),
	.vin_width                  (16'd1280                     ),
	
	.gt_tx_data                 (gt_tx_data0                 ),
	.gt_tx_ctrl                 (gt_tx_ctrl0                 )
);



/************************************************************
ddr控制器例化
************************************************************/    
DDR3_50H #
(
   //***************************************************************************
   // The following parameters are Memory Feature
   //***************************************************************************
   .MEM_ROW_WIDTH          (MEM_ROW_ADDR_WIDTH),     
   .MEM_COLUMN_WIDTH       (MEM_COL_ADDR_WIDTH),     
   .MEM_BANK_WIDTH         (MEM_BADDR_WIDTH   ),     
   .MEM_DQ_WIDTH           (MEM_DQ_WIDTH      ),     
   .MEM_DM_WIDTH           (MEM_DM_WIDTH      ),     
   .MEM_DQS_WIDTH          (MEM_DQS_WIDTH     ),     
   .CTRL_ADDR_WIDTH        (CTRL_ADDR_WIDTH   )     
)
u_DDR3_50H 
(
    .ref_clk                   (sys_clk            ),
    .resetn                    (ddr_reset_n        ),// independent asynchronous-assert/synchronous-release reset
    .ddr_init_done             (ddr_init_done      ),// output
    .ddrphy_clkin              (core_clk           ),// output
    .pll_lock                  (pll_lock           ),// output
	//写地址通道
    .axi_awaddr                (s00_axi_awaddr         ),// input [27:0]
    .axi_awuser_ap             (1'b0               ),// input
    .axi_awuser_id             (s00_axi_awid      ),// input [3:0]
    .axi_awlen                 (s00_axi_awlen          ),// input [3:0]
    .axi_awready               (s00_axi_awready        ),// output
    .axi_awvalid               (s00_axi_awvalid        ),// input
	//写通道
    .axi_wdata                 (s00_axi_wdata          ),
    .axi_wstrb                 (s00_axi_wstrb          ),// input [31:0]
    .axi_wready                (s00_axi_wready         ),// output
    .axi_wusero_id             (                   ),// output [3:0]
    .axi_wusero_last           (axi_wusero_last    ),// output
	//读地址通道
    .axi_araddr                (s00_axi_araddr         ),// input [27:0]
    .axi_aruser_ap             (1'b0               ),// input
    .axi_aruser_id             (s00_axi_arid      ),// input [3:0]
    .axi_arlen                 (s00_axi_arlen          ),// input [3:0]
    .axi_arready               (s00_axi_arready        ),// output
    .axi_arvalid               (s00_axi_arvalid        ),// input
	//读通道
    .axi_rdata                 (s00_axi_rdata          ),// output [255:0]
    .axi_rid                   (s00_axi_rid            ),// output [3:0]
    .axi_rlast                 (s00_axi_rlast          ),// output
    .axi_rvalid                (s00_axi_rvalid         ),// output

    .apb_clk                   (1'b0               ),// input
    .apb_rst_n                 (1'b1               ),// input
    .apb_sel                   (1'b0               ),// input
    .apb_enable                (1'b0               ),// input
    .apb_addr                  (8'b0               ),// input [7:0]
    .apb_write                 (1'b0               ),// input
    .apb_ready                 (                   ), // output
    .apb_wdata                 (16'b0              ),// input [15:0]
    .apb_rdata                 (                   ),// output [15:0]
    .apb_int                   (                   ),// output

    .mem_rst_n                 (mem_rst_n          ),// output
    .mem_ck                    (mem_ck             ),// output
    .mem_ck_n                  (mem_ck_n           ),// output
    .mem_cke                   (mem_cke            ),// output
    .mem_cs_n                  (mem_cs_n           ),// output
    .mem_ras_n                 (mem_ras_n          ),// output
    .mem_cas_n                 (mem_cas_n          ),// output
    .mem_we_n                  (mem_we_n           ),// output
    .mem_odt                   (mem_odt            ),// output
    .mem_a                     (mem_a              ),// output [14:0]
    .mem_ba                    (mem_ba             ),// output [2:0]
    .mem_dqs                   (mem_dqs            ),// inout [3:0]
    .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
    .mem_dq                    (mem_dq             ),// inout [31:0]
    .mem_dm                    (mem_dm             ),// output [3:0]
    //debug
    .debug_data                (                   ),// output [135:0]
    .debug_slice_state         (                   ),// output [51:0]
    .debug_calib_ctrl          (                   ),// output [21:0]
    .ck_dly_set_bin            (                   ),// output [7:0]
    .force_ck_dly_en           (1'b0               ),// input
    .force_ck_dly_set_bin      (8'h05              ),// input [7:0]
    .dll_step                  (                   ),// output [7:0]
    .dll_lock                  (                   ),// output
    .init_read_clk_ctrl        (2'b0               ),// input [1:0]
    .init_slip_step            (4'b0               ),// input [3:0]
    .force_read_clk_ctrl       (1'b0               ),// input
    .ddrphy_gate_update_en     (1'b0               ),// input
    .update_com_val_err_flag   (                   ),// output [3:0]
    .rd_fake_stop              (1'b0               ) // input
);



/************************************************************
AXI主机
************************************************************/ 
assign s00_axi_bvalid =1'b1; 
aq_axi_master_256	u_aq_axi_master
(
	  .ARESETN                     (ddr_init_done                             ),
	  .ACLK                        (core_clk                                  ),
	  .M_AXI_AWID                  (s00_axi_awid                              ),
	  .M_AXI_AWADDR                (s00_axi_awaddr                            ),
	  .M_AXI_AWLEN                 (s00_axi_awlen                             ),
	  .M_AXI_AWSIZE                (s00_axi_awsize                            ),
	  .M_AXI_AWBURST               (s00_axi_awburst                           ),
	  .M_AXI_AWLOCK                (s00_axi_awlock                            ),
	  .M_AXI_AWCACHE               (s00_axi_awcache                           ),
	  .M_AXI_AWPROT                (s00_axi_awprot                            ),
	  .M_AXI_AWQOS                 (s00_axi_awqos                             ),
	  .M_AXI_AWUSER                (s00_axi_awuser                            ),
	  .M_AXI_AWVALID               (s00_axi_awvalid                           ),
	  .M_AXI_AWREADY               (s00_axi_awready                           ),
	  .M_AXI_WDATA                 (s00_axi_wdata                             ),
	  .M_AXI_WSTRB                 (s00_axi_wstrb                             ),
	  .M_AXI_WLAST                 (s00_axi_wlast                             ),
	  .M_AXI_WUSER                 (s00_axi_wuser                             ),
	  .M_AXI_WVALID                (s00_axi_wvalid                            ),
	  .M_AXI_WREADY                (s00_axi_wready                            ),
	  .M_AXI_BID                   (s00_axi_bid                               ),
	  .M_AXI_BRESP                 (s00_axi_bresp                             ),
	  .M_AXI_BUSER                 (s00_axi_buser                             ),
      .M_AXI_BVALID                (s00_axi_bvalid                            ),

	  .M_AXI_BREADY                (s00_axi_bready                            ),
	  .M_AXI_ARID                  (s00_axi_arid                              ),
	  .M_AXI_ARADDR                (s00_axi_araddr                            ),
	  .M_AXI_ARLEN                 (s00_axi_arlen                             ),
	  .M_AXI_ARSIZE                (s00_axi_arsize                            ),
	  .M_AXI_ARBURST               (s00_axi_arburst                           ),
	  .M_AXI_ARLOCK                (s00_axi_arlock                            ),
	  .M_AXI_ARCACHE               (s00_axi_arcache                           ),
	  .M_AXI_ARPROT                (s00_axi_arprot                            ),
	  .M_AXI_ARQOS                 (s00_axi_arqos                             ),
	  .M_AXI_ARUSER                (s00_axi_aruser                            ),
	  .M_AXI_ARVALID               (s00_axi_arvalid                           ),
	  .M_AXI_ARREADY               (s00_axi_arready                           ),
	  .M_AXI_RID                   (s00_axi_rid                               ),
	  .M_AXI_RDATA                 (s00_axi_rdata                             ),
	  .M_AXI_RRESP                 (s00_axi_rresp                             ),
	  .M_AXI_RLAST                 (s00_axi_rlast                             ),
	  .M_AXI_RUSER                 (s00_axi_ruser                             ),
	  .M_AXI_RVALID                (s00_axi_rvalid                            ),
	  .M_AXI_RREADY                (s00_axi_rready                            ),
	  .MASTER_RST                  (1'b0                                      ),
	  .WR_START                    (wr_burst_req                              ),
	  .WR_ADRS                     ({wr_burst_addr,5'd0}                      ),
	  .WR_LEN                      ({wr_burst_len, 5'd0}                      ),
	  .WR_READY                    (                                          ),
	  .WR_FIFO_RE                  (wr_burst_data_req                         ),
	  .WR_FIFO_EMPTY               (1'b0                                      ),
	  .WR_FIFO_AEMPTY              (1'b0                                      ),
	  .WR_FIFO_DATA                (wr_burst_data                             ),
	  .WR_DONE                     (wr_burst_finish                           ),
	  .RD_START                    (rd_burst_req                              ),
	  .RD_ADRS                     ({rd_burst_addr,5'd0}                      ),
	  .RD_LEN                      ({rd_burst_len, 5'd0}                       ),
	  .RD_READY                    (                                          ),
	  .RD_FIFO_WE                  (rd_burst_data_valid                       ),
	  .RD_FIFO_FULL                (1'b0                                      ),
	  .RD_FIFO_AFULL               (1'b0                                      ),
	  .RD_FIFO_DATA                (rd_burst_data                             ),
	  .RD_DONE                     (rd_burst_finish                           ),
	  .DEBUG                       (                                          )
);
/************************************************************
通道仲裁模块
************************************************************/ 
//读仲裁
mem_read_arbi 
#(
	.MEM_DATA_BITS               (MEM_DATA_BITS),
	.ADDR_BITS                   (ADDR_BITS    ),
	.BUSRT_BITS                  (BUSRT_BITS   )
)
mem_read_arbi_m0
(
	.rst_n                        (ddr_init_done),
	.mem_clk                      (core_clk),
	.ch0_rd_burst_req             (ch0_rd_burst_req),
	.ch0_rd_burst_len             (ch0_rd_burst_len),
	.ch0_rd_burst_addr            (ch0_rd_burst_addr),
	.ch0_rd_burst_data_valid      (ch0_rd_burst_data_valid),
	.ch0_rd_burst_data            (ch0_rd_burst_data),
	.ch0_rd_burst_finish          (ch0_rd_burst_finish),
	
	.ch1_rd_burst_req             (ch1_rd_burst_req),
	.ch1_rd_burst_len             (ch1_rd_burst_len),
	.ch1_rd_burst_addr            (ch1_rd_burst_addr),
	.ch1_rd_burst_data_valid      (ch1_rd_burst_data_valid),
	.ch1_rd_burst_data            (ch1_rd_burst_data),
	.ch1_rd_burst_finish          (ch1_rd_burst_finish),

	.ch2_rd_burst_req             (ch2_rd_burst_req),
	.ch2_rd_burst_len             (ch2_rd_burst_len),
	.ch2_rd_burst_addr            (ch2_rd_burst_addr),
	.ch2_rd_burst_data_valid      (ch2_rd_burst_data_valid),
	.ch2_rd_burst_data            (ch2_rd_burst_data),
	.ch2_rd_burst_finish          (ch2_rd_burst_finish),

	.ch3_rd_burst_req             (ch3_rd_burst_req),
	.ch3_rd_burst_len             (ch3_rd_burst_len),
	.ch3_rd_burst_addr            (ch3_rd_burst_addr),
	.ch3_rd_burst_data_valid      (ch3_rd_burst_data_valid),
	.ch3_rd_burst_data            (ch3_rd_burst_data),
	.ch3_rd_burst_finish          (ch3_rd_burst_finish),

	.ch4_rd_burst_req             (ch4_rd_burst_req),
	.ch4_rd_burst_len             (ch4_rd_burst_len),
	.ch4_rd_burst_addr            (ch4_rd_burst_addr),
	.ch4_rd_burst_data_valid      (ch4_rd_burst_data_valid),
	.ch4_rd_burst_data            (ch4_rd_burst_data),
	.ch4_rd_burst_finish          (ch4_rd_burst_finish),

	.ch5_rd_burst_req             (ch5_rd_burst_req),
	.ch5_rd_burst_len             (ch5_rd_burst_len),
	.ch5_rd_burst_addr            (ch5_rd_burst_addr),
	.ch5_rd_burst_data_valid      (ch5_rd_burst_data_valid),
	.ch5_rd_burst_data            (ch5_rd_burst_data),
	.ch5_rd_burst_finish          (ch5_rd_burst_finish),

	.ch6_rd_burst_req             (ch6_rd_burst_req),
	.ch6_rd_burst_len             (ch6_rd_burst_len),
	.ch6_rd_burst_addr            (ch6_rd_burst_addr),
	.ch6_rd_burst_data_valid      (ch6_rd_burst_data_valid),
	.ch6_rd_burst_data            (ch6_rd_burst_data),
	.ch6_rd_burst_finish          (ch6_rd_burst_finish),

	.ch7_rd_burst_req             (ch7_rd_burst_req),
	.ch7_rd_burst_len             (ch7_rd_burst_len),
	.ch7_rd_burst_addr            (ch7_rd_burst_addr),
	.ch7_rd_burst_data_valid      (ch7_rd_burst_data_valid),
	.ch7_rd_burst_data            (ch7_rd_burst_data),
	.ch7_rd_burst_finish          (ch7_rd_burst_finish),
	
	.rd_burst_req                 (rd_burst_req),
	.rd_burst_len                 (rd_burst_len),
	.rd_burst_addr                (rd_burst_addr),
	.rd_burst_data_valid          (rd_burst_data_valid),
	.rd_burst_data                (rd_burst_data),
	.rd_burst_finish              (rd_burst_finish)	
);
//写仲裁
mem_write_arbi
#(
	.MEM_DATA_BITS               (MEM_DATA_BITS),
	.ADDR_BITS                   (ADDR_BITS    ),
	.BUSRT_BITS                  (BUSRT_BITS   )
)
mem_write_arbi_m0(
	.rst_n                       (ddr_init_done),
	.mem_clk                     (core_clk),
	
	.ch0_wr_burst_req            (ch0_wr_burst_req),
	.ch0_wr_burst_len            (ch0_wr_burst_len),
	.ch0_wr_burst_addr           (ch0_wr_burst_addr),
	.ch0_wr_burst_data_req       (ch0_wr_burst_data_req),
	.ch0_wr_burst_data           (ch0_wr_burst_data),
	.ch0_wr_burst_finish         (ch0_wr_burst_finish),
	
	.ch1_wr_burst_req            (ch1_wr_burst_req),
	.ch1_wr_burst_len            (ch1_wr_burst_len),
	.ch1_wr_burst_addr           (ch1_wr_burst_addr),
	.ch1_wr_burst_data_req       (ch1_wr_burst_data_req),
	.ch1_wr_burst_data           (ch1_wr_burst_data),
	.ch1_wr_burst_finish         (ch1_wr_burst_finish),
	
	.ch2_wr_burst_req            (ch2_wr_burst_req),
	.ch2_wr_burst_len            (ch2_wr_burst_len),
	.ch2_wr_burst_addr           (ch2_wr_burst_addr),
	.ch2_wr_burst_data_req       (ch2_wr_burst_data_req),
	.ch2_wr_burst_data           (ch2_wr_burst_data),
	.ch2_wr_burst_finish         (ch2_wr_burst_finish),

	.ch3_wr_burst_req            (ch3_wr_burst_req),
	.ch3_wr_burst_len            (ch3_wr_burst_len),
	.ch3_wr_burst_addr           (ch3_wr_burst_addr),
	.ch3_wr_burst_data_req       (ch3_wr_burst_data_req),
	.ch3_wr_burst_data           (ch3_wr_burst_data),
	.ch3_wr_burst_finish         (ch3_wr_burst_finish),



	.wr_burst_req(wr_burst_req),
	.wr_burst_len(wr_burst_len),
	.wr_burst_addr(wr_burst_addr),
	.wr_burst_data_req(wr_burst_data_req),
	.wr_burst_data(wr_burst_data),
	.wr_burst_finish(wr_burst_finish)	
);


/************************************************************
hsst模块例化
************************************************************/ 
hsst_core u_hsst_core (
  .i_free_clk(sys_clk),                        // input
  .i_pll_rst_0(~rstn_out),                      // input
  .i_wtchdg_clr_0(i_wtchdg_clr_0),                // input
  .o_wtchdg_st_0(o_wtchdg_st_0),                  // output [1:0]
  .o_txlane_done_2(o_txlane_done_2),              // output
  .o_rxlane_done_3(o_rxlane_done_3),              // output
  .i_p_refckn_0(i_p_refckn_0),                    // input
  .i_p_refckp_0(i_p_refckp_0),                    // input
  .o_p_clk2core_tx_2(tx2_clk),         			 // output
  .i_p_tx2_clk_fr_core(tx2_clk),      			// input
  .i_p_tx3_clk_fr_core(tx2_clk),      			// input
  .o_p_clk2core_rx_2(rx3_clk),          		// output
  .i_p_rx2_clk_fr_core(rx3_clk),      			// input
  .i_p_rx3_clk_fr_core(rx3_clk),      			// input
  .i_p_l2rxn(i_p_l2rxn),                          // input
  .i_p_l2rxp(i_p_l2rxp),                          // input
  .i_p_l3rxn(i_p_l3rxn),                          // input
  .i_p_l3rxp(i_p_l3rxp),                          // input
  .o_p_l2txn(o_p_l2txn),                          // output
  .o_p_l2txp(o_p_l2txp),                          // output
  .o_p_l3txn(o_p_l3txn),                          // output
  .o_p_l3txp(o_p_l3txp),                          // output
  .i_txd_2(tx2_data),                              // input [31:0]
  .i_tdispsel_2(4'b0),                    // input [3:0]
  .i_tdispctrl_2(4'b0),                  // input [3:0]
  .i_txk_2(tx2_kchar),                              // input [3:0]
  .i_txd_3(tx3_data),                              // input [31:0]
  .i_tdispsel_3(4'b0),                    // input [3:0]
  .i_tdispctrl_3(4'b0),                  // input [3:0]
  .i_txk_3(tx3_kchar),                              // input [3:0]
  .o_rxd_2(rx2_data),                              // output [31:0]
  .o_rxk_2(rx2_kchar),                              // output [3:0]
  .o_rxd_3(rx3_data),                              // output [31:0]
  .o_rxk_3(rx3_kchar)                               // output [3:0]
);
/************************************************************
心跳指示
************************************************************/ 
//心跳信号
     always@(posedge core_clk) begin
        if (!ddr_init_done)
            cnt <= 27'd0;
        else if ( cnt >= TH_1S )
            cnt <= 27'd0;
        else
            cnt <= cnt + 27'd1;
     end

     always @(posedge core_clk)
        begin
        if (!ddr_init_done)
            heart_beat_led <= 1'b0;
        else if ( cnt >= TH_1S )
            heart_beat_led <= ~heart_beat_led;
    end
                 
/////////////////////////////////////////////////////////////////////////////////////
endmodule
