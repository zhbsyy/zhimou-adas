`timescale 1ns / 1ps
`default_nettype none

//==============================================================================
// stereo_sfp_tx_top
//------------------------------------------------------------------------------
// PGL50H 顶层 wrapper（工程名 stereo_sfp_tx）：
//   双目 OV5640 -> DDR3 帧缓存 -> 1280x360 左右拼接 -> SFP(HSST lane2) 光纤输出
//
// 数据流：
//   cmos1(左眼) 1280x720 --缩放640x360--> DDR ch0 \
//                                               --> tx_clk 域拼接 1280x360
//   cmos2(右眼) 1280x720 --缩放640x360--> DDR ch1 /      -> video_packet_send
//                                                          -> HSST lane2 TX
//
// HDMI 端口现状（stereo 模式停用，仅为保留原工程时钟/复位结构而实例化）：
//   - 三路 HDMI 输入：不作为视频源；wrapper 中 pixclk_2/3_in 接 sys_clk、
//     数据/同步信号接常量，对应 DDR 写通道全部关闭
//   - 本地 HDMI 输出：时序仍在运行，但渲染源被置空（黑屏）
//==============================================================================
module stereo_sfp_tx_top (
    input wire sys_clk, input wire rst_n,
    output wire mem_rst_n, output wire mem_ck, output wire mem_ck_n,
    output wire mem_cke, output wire mem_cs_n, output wire mem_ras_n,
    output wire mem_cas_n, output wire mem_we_n, output wire mem_odt,
    output wire [14:0] mem_a, output wire [2:0] mem_ba,
    inout wire [3:0] mem_dqs, inout wire [3:0] mem_dqs_n,
    inout wire [31:0] mem_dq, output wire [3:0] mem_dm,
    output wire heart_beat_led, output wire ddr_init_done,
    output wire rstn_out, output wire rstn_out2, output wire rstn_out3,
    output wire iic_scl, inout wire iic_sda,
    output wire hdmi_int_led, output wire hdmi_int_led2, output wire debug_led2,
    input wire i_p_refckn_0, input wire i_p_refckp_0,
    input wire i_p_l2rxn, input wire i_p_l2rxp,
    input wire i_p_l3rxn, input wire i_p_l3rxp,
    output wire o_p_l2txn, output wire o_p_l2txp,
    output wire o_p_l3txn, output wire o_p_l3txp,
    output wire SFP_TX_DISABLE0, output wire SFP_TX_DISABLE1,

    // HDMI_IN1 - kept wired but not used as a capture source in stereo mode
    input wire pixclk_in, input wire vs_in, input wire hs_in, input wire de_in,
    input wire [7:0] r_in, input wire [7:0] g_in, input wire [7:0] b_in,

    // stereo camera eye 1 (left)
    inout wire cmos1_scl, inout wire cmos1_sda,
    input wire cmos1_vsync, input wire cmos1_href, input wire cmos1_pclk,
    input wire [7:0] cmos1_data, output wire cmos1_reset,

    // stereo camera eye 2 (right)
    inout wire cmos2_scl, inout wire cmos2_sda,
    input wire cmos2_vsync, input wire cmos2_href, input wire cmos2_pclk,
    input wire [7:0] cmos2_data, output wire cmos2_reset
);

    stereo_sfp_tx_core u_video_pipeline (
        .sys_clk(sys_clk), .rst_n(rst_n),
        .key1(1'b1), .key2(1'b1), .key3(1'b1), .key4(1'b1),
        .key5(1'b1), .key6(1'b1), .key7(1'b1),
        .mem_rst_n(mem_rst_n), .mem_ck(mem_ck), .mem_ck_n(mem_ck_n),
        .mem_cke(mem_cke), .mem_cs_n(mem_cs_n), .mem_ras_n(mem_ras_n),
        .mem_cas_n(mem_cas_n), .mem_we_n(mem_we_n), .mem_odt(mem_odt),
        .mem_a(mem_a), .mem_ba(mem_ba), .mem_dqs(mem_dqs),
        .mem_dqs_n(mem_dqs_n), .mem_dq(mem_dq), .mem_dm(mem_dm),
        .heart_beat_led(heart_beat_led), .ddr_init_done(ddr_init_done),
        .rstn_out(rstn_out), .rstn_out2(rstn_out2), .rstn_out3(rstn_out3),
        .iic_tx_scl(), .iic_tx_sda(), .iic_scl(iic_scl), .iic_sda(iic_sda),
        .hdmi_int_led(hdmi_int_led), .hdmi_int_led2(hdmi_int_led2),
        .debug_led2(debug_led2),
        .i_p_refckn_0(i_p_refckn_0), .i_p_refckp_0(i_p_refckp_0),
        .i_p_l2rxn(i_p_l2rxn), .i_p_l2rxp(i_p_l2rxp),
        .i_p_l3rxn(i_p_l3rxn), .i_p_l3rxp(i_p_l3rxp),
        .o_p_l2txn(o_p_l2txn), .o_p_l2txp(o_p_l2txp),
        .o_p_l3txn(o_p_l3txn), .o_p_l3txp(o_p_l3txp),
        .SFP_TX_DISABLE0(SFP_TX_DISABLE0), .SFP_TX_DISABLE1(SFP_TX_DISABLE1),
        .pixclk_in(pixclk_in), .vs_in(vs_in), .hs_in(hs_in), .de_in(de_in),
        .r_in(r_in), .g_in(g_in), .b_in(b_in),
        // HDMI_IN2 / HDMI_IN3 parked: fixed clock, no pixel data
        .pixclk_2_in(sys_clk), .vs_2_in(1'b0), .hs_2_in(1'b0), .de_2_in(1'b0),
        .r_2_in(8'd0), .g_2_in(8'd0), .b_2_in(8'd0),
        .pixclk_3_in(sys_clk), .vs_3_in(1'b0), .hs_3_in(1'b0), .de_3_in(1'b0),
        .r_3_in(8'd0), .g_3_in(8'd0), .b_3_in(8'd0),
        .cmos1_scl(cmos1_scl), .cmos1_sda(cmos1_sda),
        .cmos1_vsync(cmos1_vsync), .cmos1_href(cmos1_href),
        .cmos1_pclk(cmos1_pclk), .cmos1_data(cmos1_data),
        .cmos1_reset(cmos1_reset),
        .cmos2_scl(cmos2_scl), .cmos2_sda(cmos2_sda),
        .cmos2_vsync(cmos2_vsync), .cmos2_href(cmos2_href),
        .cmos2_pclk(cmos2_pclk), .cmos2_data(cmos2_data),
        .cmos2_reset(cmos2_reset),
        .pix_clk(), .vs_out(), .hs_out(), .de_out(),
        .r_out(), .g_out(), .b_out()
    );
endmodule
`default_nettype wire
