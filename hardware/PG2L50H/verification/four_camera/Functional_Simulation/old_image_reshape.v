module old_image_reshape(
    input                           clk                     ,
    input                           rst_n                   ,

    input                           img_vs               ,
    input                           img_data_valid       ,
    input       [15:0]              img_data             ,

    input       [3:0]               channel_select          , //bit3:ch3, bit2:ch2, bit1:ch1, bit0:ch0

    output reg                      img_data_valid_out   ,
    output reg  [15:0]              img_data_out
);


// ============================================================================
// [隔行黑修复 · 变体A]  2026-10-09
// ----------------------------------------------------------------------------
// 改了两处（本文件内，其它文件一律不动）：
//   ① COL_NUM 1280 → 640
//   ② 输出门控去掉 col_cnt[0]==1，只保留 row_cnt[0]==1
//
// 为什么（证据链见 tools_pc/sim_image_reshape.py 的对照表）：
//   本模块的时钟是 pclk_in_test = cmos1_pclk_16bit，由 GTP_IOCLKDIV_E2
//   DIV_FACTOR="2" 产生，即 **PCLK/2**（cmos_8_16bit.v:72-79）。
//   在这个时钟域里，每个 pixel_clk 出 1 个像素，而 img_data_valid 是
//   【行级 DE】（cmos_8_16bit.v:118-136 上升沿置位/下降沿清零后打 3 拍），
//   所以每行有效 clock 数 ≠ 1280。
//
//   于是原设计 COL_NUM=1280 会让 col_cnt 跨两行才回绕一次，row_cnt 两行才 +1，
//   再叠加 col_cnt[0]&row_cnt[0] 的"1/4 门控"，每帧只输出约 115200 px
//   = 14400 词；而帧缓存的 FRAME_LEN=28800 词、读侧又无条件读满 28800 词
//   → **有一半的帧缓存从未被写过，读出来就是零** → 实测的"恰好 50.0% 零行、
//     恰好 180/360 行"。
//
//   改成 COL_NUM=640 + 只抽奇行后：
//       640 px × 360 行 = 230400 px = 28800 词 = FRAME_LEN（正好写满）
//   零行消失，且水平方向的 2:1 丢点抽取一并取消（画质同时变好）。
//
// 回退：用 image_reshape_原版备份.v 覆盖本文件，重下 PCLK21 位流即可。
// ============================================================================
localparam  [11:0]  COL_NUM = 12'd640;         // ← 原 1280（本时钟域每行只有 640 个像素）
localparam  [11:0]  HALF_COL_NUM = 12'd320;    // ← 原 640（未被引用，一并改对以免误读）
localparam  [11:0]  ROW_NUM = 12'd720;        
localparam  [11:0]  HALF_ROW_NUM = 12'd360;

reg                 img_vs_dly1        ;
reg                 img_vs_dly2        ;
reg                 img_vs_pos         ;

reg [11:0]          col_cnt               ;
reg [11:0]          row_cnt               ;


always @(posedge clk) begin
    if(!rst_n) begin
        img_vs_dly1 <= 1'b0;
        img_vs_dly2 <= 1'b0;
        img_vs_pos  <= 1'b0;
    end
    else begin
        img_vs_dly1 <= img_vs;
        img_vs_dly2 <= img_vs_dly1;
        img_vs_pos  <= img_vs_dly1 & (~img_vs_dly2);
    end
end

always @(posedge clk)begin
    if (!rst_n)begin
        col_cnt <= 12'd1;
    end
    else if (img_data_valid == 1'b1 && col_cnt == COL_NUM)begin
        col_cnt <= 12'd1;
    end
    else if (img_vs_pos == 1'b1)begin
        col_cnt <= 12'd1;
    end
    else if (img_data_valid == 1'b1)begin
        col_cnt <= col_cnt + 12'd1;
    end
    else begin
        col_cnt <= col_cnt;
    end
end

always @(posedge clk)begin
    if (!rst_n)begin
        row_cnt <= 12'd1;
    end
    else if (img_data_valid == 1'b1 && col_cnt == COL_NUM && row_cnt == ROW_NUM)begin
        row_cnt <= 12'd1;
    end
    else if (img_vs_pos == 1'b1)begin
        row_cnt <= 12'd1;
    end
    else if (img_data_valid == 1'b1 && col_cnt == COL_NUM)begin
        row_cnt <= row_cnt + 12'd1;
    end
    else begin
        row_cnt <= row_cnt;
    end
end

always @(posedge clk)begin
    if (!rst_n)begin
        img_data_valid_out <= 1'b0;
        img_data_out <= 24'd0;
    end
    // [隔行黑修复·变体A] 去掉水平 2:1 丢点抽取，只保留垂直 2:1。
    // 原来: col_cnt[0]==1'b1 && row_cnt[0]==1'b1  → 每帧约 14400 词（只有 FRAME_LEN 的一半）
    // 现在: 只按 row_cnt[0]                      → 每帧 28800 词（正好写满帧缓存）
    else if (img_data_valid == 1'b1 && row_cnt[0] == 1'b1)begin
        img_data_valid_out <= img_data_valid;
        img_data_out <= img_data;
    end
    else begin
        img_data_valid_out <= 1'b0;
        img_data_out <= img_data_out;
    end
end

endmodule