`timescale 1ns/1ps

// VS activity monitor for the three HDMI inputs. VS pulses are much wider than
// one 50 MHz clock, so a level synchronizer is sufficient and remains alive
// even when an input pixel clock stops completely.
module video_source_watchdog #(
    parameter integer LOSS_CYCLES = 25000000
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [2:0] async_vs,
    output reg  [2:0] source_alive
);

(* ASYNC_REG="TRUE" *) reg [2:0] vs_meta;
(* ASYNC_REG="TRUE" *) reg [2:0] vs_sync;
reg [2:0] vs_d;
reg [24:0] quiet_count [0:2];
integer channel;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        vs_meta      <= 3'b000;
        vs_sync      <= 3'b000;
        vs_d         <= 3'b000;
        source_alive <= 3'b000;
        for(channel=0; channel<3; channel=channel+1)
            quiet_count[channel] <= 25'd0;
    end
    else begin
        vs_meta <= async_vs;
        vs_sync <= vs_meta;
        vs_d    <= vs_sync;
        for(channel=0; channel<3; channel=channel+1) begin
            if(vs_sync[channel] && !vs_d[channel]) begin
                quiet_count[channel] <= 25'd0;
                source_alive[channel] <= 1'b1;
            end
            else if(quiet_count[channel] >= LOSS_CYCLES-1) begin
                quiet_count[channel] <= quiet_count[channel];
                source_alive[channel] <= 1'b0;
            end
            else
                quiet_count[channel] <= quiet_count[channel] + 1'b1;
        end
    end
end

endmodule
