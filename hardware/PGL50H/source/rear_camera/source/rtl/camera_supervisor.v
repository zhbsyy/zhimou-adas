`timescale 1ns/1ps

// Camera-only reset and retry controller. The DDR, HDMI and HSST reset trees do
// not depend on this state machine.
module camera_supervisor #(
    parameter integer CLK_HZ                = 50000000,
    parameter integer RESET_LOW_CYCLES      = 500000,
    parameter integer RESET_SETTLE_CYCLES   = 1250000,
    parameter integer CONFIG_TIMEOUT_CYCLES = 100000000,
    parameter integer FIRST_VS_TIMEOUT      = 50000000,
    parameter integer PCLK_TIMEOUT_CYCLES   = 5000000,
    parameter integer VS_TIMEOUT_CYCLES     = 25000000
)(
    input  wire clk,
    input  wire rst_n,
    input  wire config_done,
    input  wire config_error,
    input  wire pclk_activity_toggle,
    input  wire vs_activity_toggle,
    output reg  camera_reset_n,
    output reg  config_start,
    output reg  capture_enable,
    output reg  camera_online,
    output wire retry_active
);

localparam [2:0] S_RESET_LOW  = 3'd0;
localparam [2:0] S_SETTLE     = 3'd1;
localparam [2:0] S_CONFIG     = 3'd2;
localparam [2:0] S_WAIT_VIDEO = 3'd3;
localparam [2:0] S_RUN        = 3'd4;

(* ASYNC_REG="TRUE" *) reg pclk_meta, pclk_sync;
(* ASYNC_REG="TRUE" *) reg vs_meta, vs_sync;
reg pclk_seen;
reg vs_seen;
reg [2:0] state;
reg [31:0] state_count;
reg [31:0] pclk_quiet_count;
reg [31:0] vs_quiet_count;

wire pclk_event = pclk_sync ^ pclk_seen;
wire vs_event   = vs_sync ^ vs_seen;
assign retry_active = (state != S_RUN);

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        pclk_meta <= 1'b0;
        pclk_sync <= 1'b0;
        vs_meta   <= 1'b0;
        vs_sync   <= 1'b0;
    end
    else begin
        pclk_meta <= pclk_activity_toggle;
        pclk_sync <= pclk_meta;
        vs_meta   <= vs_activity_toggle;
        vs_sync   <= vs_meta;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state             <= S_RESET_LOW;
        state_count       <= 32'd0;
        pclk_quiet_count  <= 32'd0;
        vs_quiet_count    <= 32'd0;
        pclk_seen         <= 1'b0;
        vs_seen           <= 1'b0;
        camera_reset_n    <= 1'b0;
        config_start      <= 1'b0;
        capture_enable    <= 1'b0;
        camera_online     <= 1'b0;
    end
    else begin
        pclk_seen <= pclk_sync;
        vs_seen   <= vs_sync;

        if(pclk_event)
            pclk_quiet_count <= 32'd0;
        else if(pclk_quiet_count < PCLK_TIMEOUT_CYCLES)
            pclk_quiet_count <= pclk_quiet_count + 1'b1;

        if(vs_event)
            vs_quiet_count <= 32'd0;
        else if(vs_quiet_count < VS_TIMEOUT_CYCLES)
            vs_quiet_count <= vs_quiet_count + 1'b1;

        case(state)
            S_RESET_LOW: begin
                camera_reset_n <= 1'b0;
                config_start   <= 1'b0;
                capture_enable <= 1'b0;
                camera_online  <= 1'b0;
                if(state_count >= RESET_LOW_CYCLES-1) begin
                    state            <= S_SETTLE;
                    state_count      <= 32'd0;
                    camera_reset_n   <= 1'b1;
                    pclk_quiet_count <= 32'd0;
                    vs_quiet_count   <= 32'd0;
                end
                else
                    state_count <= state_count + 1'b1;
            end

            S_SETTLE: begin
                camera_reset_n <= 1'b1;
                if(state_count >= RESET_SETTLE_CYCLES-1) begin
                    state_count  <= 32'd0;
                    config_start <= 1'b1;
                    state        <= S_CONFIG;
                end
                else
                    state_count <= state_count + 1'b1;
            end

            S_CONFIG: begin
                if(config_error || (state_count >= CONFIG_TIMEOUT_CYCLES-1)) begin
                    state          <= S_RESET_LOW;
                    state_count    <= 32'd0;
                    camera_reset_n <= 1'b0;
                    config_start   <= 1'b0;
                end
                else if(config_done) begin
                    state            <= S_WAIT_VIDEO;
                    state_count      <= 32'd0;
                    config_start     <= 1'b0;
                    capture_enable   <= 1'b1;
                    pclk_quiet_count <= 32'd0;
                    vs_quiet_count   <= 32'd0;
                end
                else
                    state_count <= state_count + 1'b1;
            end

            S_WAIT_VIDEO: begin
                if(vs_event) begin
                    state         <= S_RUN;
                    state_count   <= 32'd0;
                    camera_online <= 1'b1;
                end
                else if((pclk_quiet_count >= PCLK_TIMEOUT_CYCLES-1) ||
                        (state_count >= FIRST_VS_TIMEOUT-1)) begin
                    state          <= S_RESET_LOW;
                    state_count    <= 32'd0;
                    camera_reset_n <= 1'b0;
                    capture_enable <= 1'b0;
                end
                else
                    state_count <= state_count + 1'b1;
            end

            S_RUN: begin
                if((pclk_quiet_count >= PCLK_TIMEOUT_CYCLES-1) ||
                   (vs_quiet_count >= VS_TIMEOUT_CYCLES-1)) begin
                    state          <= S_RESET_LOW;
                    state_count    <= 32'd0;
                    camera_reset_n <= 1'b0;
                    capture_enable <= 1'b0;
                    camera_online  <= 1'b0;
                end
            end

            default: state <= S_RESET_LOW;
        endcase
    end
end

endmodule
