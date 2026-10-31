`timescale 1ns / 1ps

//==============================================================================
// sfp_stream_split
// -----------------------------------------------------------------------------
// The PGL50H board (project stereo_sfp_tx) sends ONE 1280x360 frame over a
// single SFP lane:
//     columns    0 ..  639  -> camera1 of the remote stereo module
//     columns  640 .. 1279  -> camera2 of the remote stereo module
// This module cuts that stream back into two independent 640x360 video pipes so
// they can be written into DDR as channel2 / channel3 and stitched together
// with the two local cameras. 640x360 is exactly the geometry every mosaic cell
// already uses, so FRAME_LEN, the bank map and the PCIe host protocol stay
// untouched.
//==============================================================================
module sfp_stream_split #(
    parameter FRAME_WIDTH  = 16'd1280,
    parameter HALF_WIDTH   = 16'd640,
    parameter FRAME_HEIGHT = 16'd360,
    parameter VS_PULSE_LEN = 6'd32      // vsync pulse width given to the writer
)(
    input  wire        rx_clk,
    input  wire        rst,             // high active

    input  wire        sfp_vs,          // frame sync from video_packet_rec
    input  wire        sfp_de,          // data enable from video_packet_rec
    input  wire [15:0] sfp_data,        // RGB565 pixel

    output wire        cam_a_vs,
    output wire        cam_a_de,
    output wire [15:0] cam_a_data,

    output wire        cam_b_vs,
    output wire        cam_b_de,
    output wire [15:0] cam_b_data,

    output wire        frame_active     // high while a valid frame is streaming
);

    reg        sfp_vs_d;
    reg [15:0] col_cnt;
    reg [15:0] row_cnt;
    reg [5:0]  vs_pulse_cnt;
    reg        vs_pulse;

    wire vs_rise = sfp_vs & ~sfp_vs_d;

    always @(posedge rx_clk) begin
        if (rst) sfp_vs_d <= 1'b0;
        else     sfp_vs_d <= sfp_vs;
    end

    // Column counter: holds the index of the pixel present on sfp_data.
    always @(posedge rx_clk) begin
        if (rst) begin
            col_cnt <= 16'd0;
        end
        else if (vs_rise) begin
            col_cnt <= 16'd0;
        end
        else if (sfp_de) begin
            col_cnt <= (col_cnt == FRAME_WIDTH - 16'd1) ? 16'd0 : col_cnt + 16'd1;
        end
    end

    // Row counter: one increment per completed line, cleared at frame start.
    always @(posedge rx_clk) begin
        if (rst) begin
            row_cnt <= 16'd0;
        end
        else if (vs_rise) begin
            row_cnt <= 16'd0;
        end
        else if (sfp_de && (col_cnt == FRAME_WIDTH - 16'd1)) begin
            row_cnt <= (row_cnt == FRAME_HEIGHT - 16'd1) ? row_cnt : row_cnt + 16'd1;
        end
    end

    // The packet decoder emits a short frame-sync pulse; the DDR writer expects
    // a vsync with a usable width, so stretch it to VS_PULSE_LEN clocks.
    always @(posedge rx_clk) begin
        if (rst) begin
            vs_pulse     <= 1'b0;
            vs_pulse_cnt <= 6'd0;
        end
        else if (vs_rise) begin
            vs_pulse     <= 1'b1;
            vs_pulse_cnt <= 6'd0;
        end
        else if (vs_pulse) begin
            if (vs_pulse_cnt == VS_PULSE_LEN - 6'd1) begin
                vs_pulse     <= 1'b0;
                vs_pulse_cnt <= 6'd0;
            end
            else begin
                vs_pulse_cnt <= vs_pulse_cnt + 6'd1;
            end
        end
    end

    wire in_frame = (row_cnt < FRAME_HEIGHT);

    assign cam_a_vs   = vs_pulse;
    assign cam_b_vs   = vs_pulse;
    assign cam_a_de   = sfp_de && in_frame && (col_cnt < HALF_WIDTH);
    assign cam_b_de   = sfp_de && in_frame && (col_cnt >= HALF_WIDTH);
    assign cam_a_data = sfp_data;
    assign cam_b_data = sfp_data;
    assign frame_active = in_frame & sfp_de;

endmodule
