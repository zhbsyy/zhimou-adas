`timescale 1ns / 1ps

//==============================================================================
// sfp_video_rx
// -----------------------------------------------------------------------------
// SFP receive path for the PG2L50H board: HSSTLP lane2 -> word align -> video
// packet decode -> 1280x360 RGB565 stream (vs / de / data).
//
// Ported from sfp_image_rx/src/sfp_img_recv.v (same HSSTLP family). The only
// functional change is vout_width: the link now carries a 1280x360 stitched
// stereo frame instead of a 1920x1080 HDMI frame.
//
// NOTE 2026-10-13: hsst_core 已手工切到 PLL1（PCIe 占 PLL0 + LANE0/1 + REFCK0）：
//   PLL0_EN=FALSE / PLL1_EN=TRUE / CH2_TX_RX_PLL_SEL=PLL1 / PLL1_REF_SEL=Diff_REFCK1
// 因此本模块的 ref_clk_n/p 接的是 hsst_core 的 i_p_refck_1（REFCK1=E10/F10，
// 约束见 project/sfp_rx_addon.fdc）。线速率 5Gbps / 32bit / 125MHz 不变，
// 与对端 PGL50H(hdmi_sfp_tx) 一致。若底板上 E10/F10 没有时钟，可把 hsst_core.v
// 的 PLL1_REF_SEL 改回 "Diff_REFCK0" 并把 ref_clk 改接到 i_p_refck_0
// 与 PCIe 共用 100MHz），fdc 引脚约束同步删除。
//==============================================================================
module sfp_video_rx (
    input  wire        free_clk,        // free running clock for HSST config
    input  wire        ref_clk_n,       // SFP reference clock, negative
    input  wire        ref_clk_p,       // SFP reference clock, positive
    input  wire        sfp_rxn,         // SFP serial input, negative
    input  wire        sfp_rxp,         // SFP serial input, positive
    output wire        sfp_txn,         // SFP serial output (unused)
    output wire        sfp_txp,
    output wire        sfp_tx_disable,  // 0 = optical module enabled
    input  wire        rst_n,           // async reset, active low

    output wire        rx_clk,          // recovered receive clock
    output wire        rxlane_done,     // RX lane initialisation done
    output wire        pll1_lock,
    output wire        rx_sigdet,
    output wire        cdr_align,
    output wire        pcs_synced,
    output wire        vs,              // frame sync
    output wire        de,              // data enable
    output wire [15:0] vout_data        // RGB565 pixel
);

    wire        pll_rst_d;
    wire        i_pll_rst_0;
    wire        i_p_cfg_rst;
    wire        i_wtchdg_clr_0;

    wire [31:0] sfp_rx_data;
    wire [3:0]  sfp_rx_kchar;
    wire [31:0] sfp_rx_data_aligned;
    wire [3:0]  sfp_rx_kchar_aligned;

    wire [31:0] sfp_tx_data  = 32'd0;
    wire [3:0]  sfp_tx_kchar = 4'b0;
    wire        sfp_txlane_done;
    wire        sfp_tx_clk;
    wire [1:0]  o_wtchdg_st_0;

    wire rx_async_reset_n = rst_n && rxlane_done;
    (* ASYNC_REG="TRUE" *) reg [1:0] rx_reset_pipe;
    always @(posedge rx_clk or negedge rx_async_reset_n) begin
        if (!rx_async_reset_n)
            rx_reset_pipe <= 2'b00;
        else
            rx_reset_pipe <= {rx_reset_pipe[0], 1'b1};
    end

    assign i_p_cfg_rst    = ~rst_n;
    assign pll_rst_d      = i_p_cfg_rst;
    assign i_pll_rst_0    = pll_rst_d;
    assign i_wtchdg_clr_0 = pll_rst_d;
    assign sfp_tx_disable = 1'b0;

    hsst_core u_hsst_core_sfp (
      .i_free_clk         (free_clk),
      .i_pll_rst_0        (i_pll_rst_0),
      .i_pll_rst_1        (i_pll_rst_0),
      .i_wtchdg_clr_0     (i_wtchdg_clr_0),
      .o_wtchdg_st_0      (o_wtchdg_st_0),
      .o_pll_done_0       (),
      .o_txlane_done_2    (sfp_txlane_done),
      .o_rxlane_done_2    (rxlane_done),
      .i_p_refckn_0       (ref_clk_n),
      .i_p_refckp_0       (ref_clk_p),
      .i_p_refckn_1       (ref_clk_n),
      .i_p_refckp_1       (ref_clk_p),
      .o_p_clk2core_tx_2  (sfp_tx_clk),
      .o_p_clk2core_rx_2  (rx_clk),
      .i_p_tx2_clk_fr_core(sfp_tx_clk),
      .i_p_rx2_clk_fr_core(rx_clk),
      .o_p_pll_lock_0     (),
      .o_p_pll_lock_1     (pll1_lock),
      .o_p_rx_sigdet_sta_2(rx_sigdet),
      .o_p_lx_cdr_align_2 (cdr_align),
      .o_p_pcs_lsm_synced_2(pcs_synced),
      .i_p_l2rxn          (sfp_rxn),
      .i_p_l2rxp          (sfp_rxp),
      .o_p_l2txn          (sfp_txn),
      .o_p_l2txp          (sfp_txp),
      .i_txd_2            (sfp_tx_data),
      .i_tdispsel_2       (4'b0),
      .i_tdispctrl_2      (4'b0),
      .i_txk_2            (sfp_tx_kchar),
      .o_rxstatus_2       (),
      .o_rxd_2            (sfp_rx_data),
      .o_rdisper_2        (),
      .o_rdecer_2         (),
      .o_rxk_2            (sfp_rx_kchar)
    );

    word_align u_word_align (
        .rst           (~rx_reset_pipe[1]),
        .rx_clk        (rx_clk),
        .gt_rx_data    (sfp_rx_data),
        .gt_rx_ctrl    (sfp_rx_kchar),
        .rx_data_align (sfp_rx_data_aligned),
        .rx_ctrl_align (sfp_rx_kchar_aligned)
    );

    video_packet_rec u_video_packet_rec (
        .rst         (~rx_reset_pipe[1]),
        .rx_clk      (rx_clk),
        .gt_rx_data  (sfp_rx_data_aligned),
        .gt_rx_ctrl  (sfp_rx_kchar_aligned),
        .vout_width  (16'd1280),
        .vs          (vs),
        .de          (de),
        .vout_data   (vout_data)
    );

endmodule
