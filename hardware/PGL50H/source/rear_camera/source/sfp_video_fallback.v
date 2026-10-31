`timescale 1ns/1ps

// Keeps the SFP timing stream diagnostically useful during DDR startup.
module sfp_video_fallback
(
    input  wire        ddr_ready,
    input  wire        has_valid_frame,
    input  wire        switch_blank,
    input  wire [15:0] color_bar_data,
    input  wire [15:0] composed_data,
    output wire [15:0] output_data
);

assign output_data = (!ddr_ready || !has_valid_frame) ? color_bar_data :
                     switch_blank                     ? 16'h0000       : composed_data;

endmodule
