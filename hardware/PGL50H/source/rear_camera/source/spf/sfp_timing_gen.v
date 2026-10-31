`timescale 1ns / 1ps

// SFP link timing generator.
// The SFP packetizer runs in the 125 MHz GT clock domain and ships one 16-bit
// pixel per clock while de is asserted. A dedicated 1280x360 timing keeps the
// link payload to exactly one stitched stereo frame (640x360 left + 640x360
// right) instead of pushing the full 1920x1080 HDMI canvas over the fiber.
module sfp_timing_gen #(
    parameter H_ACTIVE = 16'd1280,
    parameter H_FP     = 16'd20,
    parameter H_SYNC   = 16'd20,
    parameter H_BP     = 16'd60,
    parameter V_ACTIVE = 16'd360,
    parameter V_FP     = 16'd4,
    parameter V_SYNC   = 16'd4,
    parameter V_BP     = 16'd12,
    parameter HS_POL   = 1'b0,   // sync pulse is low
    parameter VS_POL   = 1'b1    // sync pulse is high: rising edge = frame start
)(
    input  wire        clk,
    input  wire        rst,      // high active
    output reg         hs,
    output reg         vs,
    output reg         de,
    output reg  [15:0] x_cnt,
    output reg  [15:0] y_cnt
);

localparam H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;
localparam V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;
localparam H_SYNC_START = H_ACTIVE + H_FP;
localparam H_SYNC_END   = H_SYNC_START + H_SYNC;
localparam V_SYNC_START = V_ACTIVE + V_FP;
localparam V_SYNC_END   = V_SYNC_START + V_SYNC;

always @(posedge clk) begin
    if (rst) begin
        x_cnt <= 16'd0;
        y_cnt <= 16'd0;
    end
    else if (x_cnt == H_TOTAL - 16'd1) begin
        x_cnt <= 16'd0;
        y_cnt <= (y_cnt == V_TOTAL - 16'd1) ? 16'd0 : y_cnt + 16'd1;
    end
    else begin
        x_cnt <= x_cnt + 16'd1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        hs <= ~HS_POL;
        vs <= ~VS_POL;
        de <= 1'b0;
    end
    else begin
        hs <= (x_cnt >= H_SYNC_START && x_cnt < H_SYNC_END) ? HS_POL : ~HS_POL;
        vs <= (y_cnt >= V_SYNC_START && y_cnt < V_SYNC_END) ? VS_POL : ~VS_POL;
        de <= (x_cnt < H_ACTIVE) && (y_cnt < V_ACTIVE);
    end
end

endmodule
