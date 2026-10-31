`timescale 1ns/1ps

// Seven-mode display control in the SFP clock domain. A mode is committed at
// output VS, but each source becomes visible independently after its own first
// complete frame. Channels 0..2 are HDMI; channel 3 is OV5640.
module change_mode(
    input wire video_out_vs, input wire rst_n, input wire video_clk,
    input wire [7:0] display_mode, input wire display_mode_toggle,
    input wire [3:0] frame_done_toggle, input wire [3:0] frame_done_tag,
    input wire [7:0] frame_done_index, input wire [3:0] source_alive,
    output reg [11:0] hdmi_out_1_width, output reg [11:0] hdmi_out_1_height,
    output reg [11:0] hdmi_out_2_width, output reg [11:0] hdmi_out_2_height,
    output reg [11:0] hdmi_out_3_width, output reg [11:0] hdmi_out_3_height,
    output reg [11:0] hdmi_out_1_width_offset, output reg [11:0] hdmi_out_1_height_offset,
    output reg [11:0] hdmi_out_2_width_offset, output reg [11:0] hdmi_out_2_height_offset,
    output reg [11:0] hdmi_out_3_width_offset, output reg [11:0] hdmi_out_3_height_offset,
    output reg [11:0] camera_width, output reg [11:0] camera_height,
    output reg [11:0] camera_width_offset, output reg [11:0] camera_height_offset,
    output wire [7:0] display_mode_wr, output wire [7:0] display_mode_rd,
    output wire write_mode_toggle, output wire switch_blank,
    output wire [3:0] channel_valid, output wire switch_timeout_sticky,
    output wire [7:0] published_frame_index, output wire [3:0] display_state_version,
    output wire [7:0] pending_mode_debug, output wire [7:0] active_mode_debug
);

localparam [1:0] S_RUN=2'd0, S_WAIT_VS=2'd1, S_SETTLE=2'd2, S_UNBLANK=2'd3;

function [3:0] mode_channel_mask;
    input [7:0] value;
    begin
        case(value)
            // Mode 1 is the fixed stereo SFP mode: camera1/camera2 occupy
            // channels 0/1, even though the legacy HDMI mode used only ch0.
            8'd1: mode_channel_mask=4'b0011; 8'd2: mode_channel_mask=4'b0010;
            8'd3: mode_channel_mask=4'b0100; 8'd4: mode_channel_mask=4'b1000;
            8'd5: mode_channel_mask=4'b0101; 8'd6: mode_channel_mask=4'b0110;
            default: mode_channel_mask=4'b1111;
        endcase
    end
endfunction

function mode_is_valid;
    input [7:0] value;
    begin mode_is_valid=(value>=8'd1)&&(value<=8'd7); end
endfunction

(* ASYNC_REG="TRUE" *) reg [7:0] mode_meta,mode_sync_0,mode_sync_1;
(* ASYNC_REG="TRUE" *) reg mode_toggle_meta,mode_toggle_sync_0,mode_toggle_sync_1;
(* ASYNC_REG="TRUE" *) reg [3:0] done_toggle_meta,done_toggle_sync_0,done_toggle_sync_1;
(* ASYNC_REG="TRUE" *) reg [3:0] done_tag_meta,done_tag_sync_0,done_tag_sync_1;
(* ASYNC_REG="TRUE" *) reg [7:0] done_index_meta,done_index_sync_0,done_index_sync_1;
(* ASYNC_REG="TRUE" *) reg [3:0] alive_meta,alive_sync;

always @(posedge video_clk or negedge rst_n) begin
    if(!rst_n) begin
        mode_meta<=7; mode_sync_0<=7; mode_sync_1<=7;
        mode_toggle_meta<=0; mode_toggle_sync_0<=0; mode_toggle_sync_1<=0;
        done_toggle_meta<=0; done_toggle_sync_0<=0; done_toggle_sync_1<=0;
        done_tag_meta<=0; done_tag_sync_0<=0; done_tag_sync_1<=0;
        done_index_meta<=0; done_index_sync_0<=0; done_index_sync_1<=0;
        alive_meta<=0; alive_sync<=0;
    end else begin
        mode_meta<=display_mode; mode_sync_0<=mode_meta; mode_sync_1<=mode_sync_0;
        mode_toggle_meta<=display_mode_toggle; mode_toggle_sync_0<=mode_toggle_meta;
        mode_toggle_sync_1<=mode_toggle_sync_0;
        done_toggle_meta<=frame_done_toggle; done_toggle_sync_0<=done_toggle_meta;
        done_toggle_sync_1<=done_toggle_sync_0;
        done_tag_meta<=frame_done_tag; done_tag_sync_0<=done_tag_meta;
        done_tag_sync_1<=done_tag_sync_0;
        done_index_meta<=frame_done_index; done_index_sync_0<=done_index_meta;
        done_index_sync_1<=done_index_sync_0;
        alive_meta<=source_alive; alive_sync<=alive_meta;
    end
end

reg video_out_vs_d;
always @(posedge video_clk or negedge rst_n)
    if(!rst_n) video_out_vs_d<=0; else video_out_vs_d<=video_out_vs;
wire output_vs_rise=video_out_vs&&!video_out_vs_d;

reg [1:0] state;
reg [7:0] pending_mode,active_mode,display_mode_wr_reg,display_mode_rd_reg;
reg write_mode_toggle_reg,switch_blank_reg,switch_timeout_reg,mode_toggle_seen;
reg [3:0] channel_valid_reg,required_mask,recovery_mask,done_toggle_seen;
reg [7:0] pending_frame_index,published_index_reg;
reg [3:0] display_version_reg;
reg [1:0] settle_count;
reg [2:0] missing_frame_count;
integer channel_number;

wire mode_request_ready=(mode_toggle_sync_1!=mode_toggle_seen)&&
                        (mode_sync_0==mode_sync_1)&&mode_is_valid(mode_sync_1);
wire [3:0] done_event=done_toggle_sync_1^done_toggle_seen;
wire [3:0] tag_match=~(done_tag_sync_1^{4{write_mode_toggle_reg}});
wire accept_frames=(state==S_RUN)||(state==S_UNBLANK);
wire [3:0] matching_done=done_event&tag_match&required_mask&alive_sync&{4{accept_frames}};
wire [3:0] publish_mask=recovery_mask|matching_done;
wire [3:0] retained_valid=channel_valid_reg&alive_sync&required_mask;

reg [11:0] next_w1,next_h1,next_w2,next_h2,next_w3,next_h3;
reg [11:0] next_x1,next_y1,next_x2,next_y2,next_x3,next_y3;
reg [11:0] next_cw,next_ch,next_cx,next_cy;
always @(*) begin
    next_w1=960; next_h1=540; next_x1=0; next_y1=0;
    next_w2=960; next_h2=540; next_x2=960; next_y2=0;
    next_w3=960; next_h3=540; next_x3=0; next_y3=540;
    next_cw=0; next_ch=0; next_cx=0; next_cy=0;
    case(pending_mode)
        1: begin next_w1=1920;next_h1=1080;next_w2=0;next_h2=0;next_w3=0;next_h3=0; end
        2: begin next_w1=0;next_h1=0;next_w2=1920;next_h2=1080;next_w3=0;next_h3=0;next_x2=0; end
        3: begin next_w1=0;next_h1=0;next_w2=0;next_h2=0;next_w3=1920;next_h3=1080;next_x3=0;next_y3=0; end
        4: begin next_w1=0;next_h1=0;next_w2=0;next_h2=0;next_w3=0;next_h3=0;
                 next_cw=1280;next_ch=720;next_cx=320;next_cy=180; end
        5: begin next_w2=0;next_h2=0;next_w3=960;next_h3=540;next_x3=960;next_y3=0; end
        6: begin next_w1=0;next_h1=0;next_w2=960;next_h2=540;next_x2=0;
                 next_w3=960;next_h3=540;next_x3=960;next_y3=0; end
        7: begin next_cw=960;next_ch=540;next_cx=960;next_cy=540; end
        default: begin end
    endcase
end

always @(posedge video_clk or negedge rst_n) begin
    if(!rst_n) begin
        state<=S_RUN; pending_mode<=7; active_mode<=7;
        display_mode_wr_reg<=7; display_mode_rd_reg<=7;
        write_mode_toggle_reg<=0; switch_blank_reg<=0;
        channel_valid_reg<=0; required_mask<=4'b1111; recovery_mask<=0;
        pending_frame_index<=0; published_index_reg<=0; display_version_reg<=0;
        switch_timeout_reg<=0; mode_toggle_seen<=0; done_toggle_seen<=0;
        settle_count<=0; missing_frame_count<=0;
        hdmi_out_1_width<=960;hdmi_out_1_height<=540;
        hdmi_out_2_width<=960;hdmi_out_2_height<=540;
        hdmi_out_3_width<=960;hdmi_out_3_height<=540;
        hdmi_out_1_width_offset<=0;hdmi_out_1_height_offset<=0;
        hdmi_out_2_width_offset<=960;hdmi_out_2_height_offset<=0;
        hdmi_out_3_width_offset<=0;hdmi_out_3_height_offset<=540;
        camera_width<=960;camera_height<=540;camera_width_offset<=960;camera_height_offset<=540;
    end else begin
        done_toggle_seen<=done_toggle_sync_1;
        for(channel_number=0;channel_number<4;channel_number=channel_number+1)
            if(matching_done[channel_number])
                pending_frame_index[channel_number*2 +: 2]<=done_index_sync_1[channel_number*2 +: 2];
        if(accept_frames) recovery_mask<=recovery_mask|matching_done;

        if(output_vs_rise&&(state==S_RUN||state==S_UNBLANK)) begin
            channel_valid_reg<=retained_valid|publish_mask;
            for(channel_number=0;channel_number<4;channel_number=channel_number+1)
                if(publish_mask[channel_number]) begin
                    if(matching_done[channel_number])
                        published_index_reg[channel_number*2 +: 2]<=done_index_sync_1[channel_number*2 +: 2];
                    else published_index_reg[channel_number*2 +: 2]<=pending_frame_index[channel_number*2 +: 2];
                end
            if((retained_valid!=channel_valid_reg) || (|publish_mask))
                display_version_reg<=display_version_reg+1'b1;
            recovery_mask<=0;
            if((alive_sync&required_mask)!=required_mask) begin
                if(missing_frame_count==5) switch_timeout_reg<=1;
                else missing_frame_count<=missing_frame_count+1'b1;
            end else missing_frame_count<=0;
            if(state==S_UNBLANK) begin switch_blank_reg<=0;state<=S_RUN; end
        end

        case(state)
            S_WAIT_VS: if(!mode_request_ready && output_vs_rise) begin
                active_mode<=pending_mode;display_mode_wr_reg<=pending_mode;display_mode_rd_reg<=pending_mode;
                required_mask<=mode_channel_mask(pending_mode);channel_valid_reg<=0;recovery_mask<=0;
                switch_blank_reg<=1;switch_timeout_reg<=0;missing_frame_count<=0;
                display_version_reg<=display_version_reg+1'b1;
                hdmi_out_1_width<=next_w1;hdmi_out_1_height<=next_h1;
                hdmi_out_2_width<=next_w2;hdmi_out_2_height<=next_h2;
                hdmi_out_3_width<=next_w3;hdmi_out_3_height<=next_h3;
                hdmi_out_1_width_offset<=next_x1;hdmi_out_1_height_offset<=next_y1;
                hdmi_out_2_width_offset<=next_x2;hdmi_out_2_height_offset<=next_y2;
                hdmi_out_3_width_offset<=next_x3;hdmi_out_3_height_offset<=next_y3;
                camera_width<=next_cw;camera_height<=next_ch;camera_width_offset<=next_cx;camera_height_offset<=next_cy;
                settle_count<=0;state<=S_SETTLE;
            end
            S_SETTLE: if(settle_count==2) begin
                write_mode_toggle_reg<=~write_mode_toggle_reg;state<=S_UNBLANK;
            end else settle_count<=settle_count+1'b1;
            default: begin end
        endcase

        // Give a newly synchronized key request final priority over unblank or
        // settle transitions occurring on the same clock.
        if(mode_request_ready) begin
            mode_toggle_seen<=mode_toggle_sync_1;
            if(mode_sync_1!=active_mode) begin
                pending_mode<=mode_sync_1;
                state<=S_WAIT_VS;
            end
        end
    end
end

assign display_mode_wr=display_mode_wr_reg; assign display_mode_rd=display_mode_rd_reg;
assign write_mode_toggle=write_mode_toggle_reg; assign switch_blank=switch_blank_reg;
assign channel_valid=channel_valid_reg; assign switch_timeout_sticky=switch_timeout_reg;
assign published_frame_index=published_index_reg; assign display_state_version=display_version_reg;
assign pending_mode_debug=pending_mode; assign active_mode_debug=active_mode;
endmodule
