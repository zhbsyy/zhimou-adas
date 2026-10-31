`timescale 1ns/1ps

// Four-layer 1920x1080 RGB565 compositor.  Timing is generated locally in the
// renderer clock domain; only read requests/data cross through asynchronous
// frame FIFOs.
module display_renderer_4ch
(
    input             video_clk,
    input             rst,
    input      [7:0]  display_mode,
    input      [3:0]  channel_valid,
    input             switch_blank,

    output            ch0_read_req,
    input             ch0_read_req_ack,
    output            ch0_read_en,
    input      [15:0] ch0_read_data,
    input             ch0_read_empty,

    output            ch1_read_req,
    input             ch1_read_req_ack,
    output            ch1_read_en,
    input      [15:0] ch1_read_data,
    input             ch1_read_empty,

    output            ch2_read_req,
    input             ch2_read_req_ack,
    output            ch2_read_en,
    input      [15:0] ch2_read_data,
    input             ch2_read_empty,

    output            ch3_read_req,
    input             ch3_read_req_ack,
    output            ch3_read_en,
    input      [15:0] ch3_read_data,
    input             ch3_read_empty,

    output     [3:0]  read_underflow_sticky,
    output     [15:0] video_data,
    output            video_hs,
    output            video_vs,
    output            video_de
);

reg [11:0] width_0, height_0, left_0, top_0;
reg [11:0] width_1, height_1, left_1, top_1;
reg [11:0] width_2, height_2, left_2, top_2;
reg [11:0] width_3, height_3, left_3, top_3;

always @(*) begin
    width_0  = 12'd960; height_0 = 12'd540; left_0 = 12'd0;   top_0 = 12'd0;
    width_1  = 12'd960; height_1 = 12'd540; left_1 = 12'd960; top_1 = 12'd0;
    width_2  = 12'd960; height_2 = 12'd540; left_2 = 12'd0;   top_2 = 12'd540;
    width_3  = 12'd960; height_3 = 12'd540; left_3 = 12'd960; top_3 = 12'd540;
    case(display_mode)
        8'd1: begin
            width_0=12'd1920; height_0=12'd1080; left_0=12'd0; top_0=12'd0;
            width_1=0; height_1=0; width_2=0; height_2=0; width_3=0; height_3=0;
        end
        8'd2: begin
            width_0=0; height_0=0; width_1=12'd1920; height_1=12'd1080; left_1=0; top_1=0;
            width_2=0; height_2=0; width_3=0; height_3=0;
        end
        8'd3: begin
            width_0=0; height_0=0; width_1=0; height_1=0;
            width_2=12'd1920; height_2=12'd1080; left_2=0; top_2=0; width_3=0; height_3=0;
        end
        8'd4: begin
            width_0=0; height_0=0; width_1=0; height_1=0; width_2=0; height_2=0;
            width_3=12'd1280; height_3=12'd720; left_3=12'd320; top_3=12'd180;
        end
        8'd5: begin
            width_0=12'd960; height_0=12'd540; left_0=0; top_0=0;
            width_1=0; height_1=0;
            width_2=12'd960; height_2=12'd540; left_2=12'd960; top_2=0;
            width_3=0; height_3=0;
        end
        8'd6: begin
            width_0=0; height_0=0;
            width_1=12'd960; height_1=12'd540; left_1=0; top_1=0;
            width_2=12'd960; height_2=12'd540; left_2=12'd960; top_2=0;
            width_3=0; height_3=0;
        end
        default: begin end
    endcase
end

wire [11:0] active_width_0  = (!switch_blank && channel_valid[0]) ? width_0  : 12'd0;
wire [11:0] active_height_0 = (!switch_blank && channel_valid[0]) ? height_0 : 12'd0;
wire [11:0] active_width_1  = (!switch_blank && channel_valid[1]) ? width_1  : 12'd0;
wire [11:0] active_height_1 = (!switch_blank && channel_valid[1]) ? height_1 : 12'd0;
wire [11:0] active_width_2  = (!switch_blank && channel_valid[2]) ? width_2  : 12'd0;
wire [11:0] active_height_2 = (!switch_blank && channel_valid[2]) ? height_2 : 12'd0;
wire [11:0] active_width_3  = (!switch_blank && channel_valid[3]) ? width_3  : 12'd0;
wire [11:0] active_height_3 = (!switch_blank && channel_valid[3]) ? height_3 : 12'd0;

wire timing_hs, timing_vs, timing_de;
wire [7:0] unused_r, unused_g, unused_b;
color_bar timing_source (
    .clk(video_clk), .rst(rst),
    .hs(timing_hs), .vs(timing_vs), .de(timing_de),
    .rgb_r(unused_r), .rgb_g(unused_g), .rgb_b(unused_b)
);

wire [15:0] layer0_data, layer1_data, layer2_data, layer3_data;
wire layer0_hs, layer0_vs, layer0_de;
wire layer1_hs, layer1_vs, layer1_de;
wire layer2_hs, layer2_vs, layer2_de;
wire layer3_hs, layer3_vs, layer3_de;

video_rect_read_data layer0 (
    .video_clk(video_clk), .rst(rst),
    .video_left_offset(left_0), .video_top_offset(top_0),
    .video_width(active_width_0), .video_height(active_height_0),
    .read_req(ch0_read_req), .read_req_ack(ch0_read_req_ack),
    .read_en(ch0_read_en), .read_data(ch0_read_data), .read_empty(ch0_read_empty),
    .status_clear(switch_blank), .underflow_sticky(read_underflow_sticky[0]),
    .timing_hs(timing_hs), .timing_vs(timing_vs), .timing_de(timing_de), .timing_data(16'h0000),
    .x_cnt(), .y_cnt(), .hs(layer0_hs), .vs(layer0_vs), .de(layer0_de), .vout_data(layer0_data)
);

video_rect_read_data layer1 (
    .video_clk(video_clk), .rst(rst),
    .video_left_offset(left_1), .video_top_offset(top_1),
    .video_width(active_width_1), .video_height(active_height_1),
    .read_req(ch1_read_req), .read_req_ack(ch1_read_req_ack),
    .read_en(ch1_read_en), .read_data(ch1_read_data), .read_empty(ch1_read_empty),
    .status_clear(switch_blank), .underflow_sticky(read_underflow_sticky[1]),
    .timing_hs(layer0_hs), .timing_vs(layer0_vs), .timing_de(layer0_de), .timing_data(layer0_data),
    .x_cnt(), .y_cnt(), .hs(layer1_hs), .vs(layer1_vs), .de(layer1_de), .vout_data(layer1_data)
);

video_rect_read_data layer2 (
    .video_clk(video_clk), .rst(rst),
    .video_left_offset(left_2), .video_top_offset(top_2),
    .video_width(active_width_2), .video_height(active_height_2),
    .read_req(ch2_read_req), .read_req_ack(ch2_read_req_ack),
    .read_en(ch2_read_en), .read_data(ch2_read_data), .read_empty(ch2_read_empty),
    .status_clear(switch_blank), .underflow_sticky(read_underflow_sticky[2]),
    .timing_hs(layer1_hs), .timing_vs(layer1_vs), .timing_de(layer1_de), .timing_data(layer1_data),
    .x_cnt(), .y_cnt(), .hs(layer2_hs), .vs(layer2_vs), .de(layer2_de), .vout_data(layer2_data)
);

video_rect_read_data layer3 (
    .video_clk(video_clk), .rst(rst),
    .video_left_offset(left_3), .video_top_offset(top_3),
    .video_width(active_width_3), .video_height(active_height_3),
    .read_req(ch3_read_req), .read_req_ack(ch3_read_req_ack),
    .read_en(ch3_read_en), .read_data(ch3_read_data), .read_empty(ch3_read_empty),
    .status_clear(switch_blank), .underflow_sticky(read_underflow_sticky[3]),
    .timing_hs(layer2_hs), .timing_vs(layer2_vs), .timing_de(layer2_de), .timing_data(layer2_data),
    .x_cnt(), .y_cnt(), .hs(layer3_hs), .vs(layer3_vs), .de(layer3_de), .vout_data(layer3_data)
);

assign video_hs   = layer3_hs;
assign video_vs   = layer3_vs;
assign video_de   = layer3_de;
assign video_data = switch_blank ? 16'h0000 : layer3_data;

endmodule
