`timescale 1ns/1ps

// OV5640 byte-to-pixel capture. All state remains in the sensor PCLK domain;
// no divided clock is generated. Byte phase is restarted at every line and
// frame boundary so a damaged line cannot corrupt the following lines.
module ov5640_byte_capture #(
    parameter integer LINE_BYTES  = 2560,
    parameter integer FRAME_LINES = 720
)(
    input  wire        pclk,
    input  wire        rst_n,
    input  wire        href,
    input  wire [7:0]  data,
    input  wire        vsync,

    output reg         pixel_valid,
    output reg  [15:0] pixel_data,
    output reg         line_error,
    output reg         frame_error,
    output reg         frame_valid,
    output reg         frame_valid_toggle,
    output reg         frame_start
);

reg        vsync_d;
reg        href_d;
reg        byte_phase;
reg [7:0]  first_byte;
reg [12:0] line_byte_count;
reg [10:0] frame_line_count;
reg        frame_started;

wire vsync_rise = vsync && !vsync_d;
wire href_rise  = href && !href_d;
wire href_fall  = !href && href_d;

always @(posedge pclk or negedge rst_n) begin
    if(!rst_n) begin
        vsync_d          <= 1'b0;
        href_d           <= 1'b0;
        byte_phase       <= 1'b0;
        first_byte       <= 8'd0;
        line_byte_count  <= 13'd0;
        frame_line_count <= 11'd0;
        frame_started    <= 1'b0;
        pixel_valid      <= 1'b0;
        pixel_data       <= 16'd0;
        line_error       <= 1'b0;
        frame_error      <= 1'b0;
        frame_valid      <= 1'b0;
        frame_valid_toggle <= 1'b0;
        frame_start      <= 1'b0;
    end
    else begin
        vsync_d     <= vsync;
        href_d      <= href;
        pixel_valid <= 1'b0;
        line_error  <= 1'b0;
        frame_start <= 1'b0;

        if(vsync_rise) begin
            // Validate the frame that has just ended. The result is held and
            // transferred with a toggle so DDR cannot publish before checking
            // for missing/extra lines at the VS boundary.
            if(frame_started) begin
                frame_valid <= (frame_line_count == FRAME_LINES) &&
                               !frame_error && !href && !byte_phase;
                frame_valid_toggle <= ~frame_valid_toggle;
            end
            else
                frame_valid <= 1'b0;
            byte_phase       <= 1'b0;
            line_byte_count  <= 13'd0;
            frame_line_count <= 11'd0;
            frame_started    <= 1'b1;
            frame_error      <= 1'b0;
            frame_start      <= 1'b1;
        end
        else if(frame_started) begin
            if(href_rise) begin
                byte_phase      <= 1'b0;
                line_byte_count <= 13'd0;
                if(frame_line_count >= FRAME_LINES) begin
                    line_error  <= 1'b1;
                    frame_error <= 1'b1;
                end
            end

            if(href) begin
                // The first byte is consumed in the same PCLK in which HREF
                // rises; do not compare the stale count from the prior line.
                if(href_rise) begin
                    line_byte_count <= 13'd1;
                    first_byte      <= data;
                    byte_phase      <= 1'b1;
                end
                else if(line_byte_count < LINE_BYTES) begin
                    line_byte_count <= line_byte_count + 1'b1;
                    if(!byte_phase) begin
                        first_byte <= data;
                        byte_phase <= 1'b1;
                    end
                    else begin
                        pixel_data  <= {first_byte, data};
                        pixel_valid <= 1'b1;
                        byte_phase  <= 1'b0;
                    end
                end
                else begin
                    line_error  <= 1'b1;
                    frame_error <= 1'b1;
                end
            end

            if(href_fall) begin
                byte_phase <= 1'b0;
                if((line_byte_count != LINE_BYTES) || byte_phase) begin
                    line_error  <= 1'b1;
                    frame_error <= 1'b1;
                end
                else if(frame_line_count < FRAME_LINES) begin
                    frame_line_count <= frame_line_count + 1'b1;
                end
                else begin
                    line_error  <= 1'b1;
                    frame_error <= 1'b1;
                end
            end
        end
    end
end

endmodule
