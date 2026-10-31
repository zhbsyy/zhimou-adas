`timescale 1ns/1ps

// Board startup controller.  PLL lock is synchronized into the fixed
// reference domain.  DDR reset asserts asynchronously and is released through
// two reference-clock flip-flops after the requested power-up delay.
module ddr_startup_reset
#(
    parameter STARTUP_DELAY_CYCLES = 50000
)
(
    input  wire sys_clk,
    input  wire external_reset_n,
    input  wire board_pll_locked,
    output reg  system_reset_n,
    output wire ddr_reset_n
);

(* ASYNC_REG="TRUE" *) reg pll_locked_meta;
(* ASYNC_REG="TRUE" *) reg pll_locked_sync;
(* ASYNC_REG="TRUE" *) reg [1:0] ddr_reset_pipe;
reg [31:0] startup_count;

always @(posedge sys_clk or negedge external_reset_n) begin
    if(!external_reset_n) begin
        pll_locked_meta <= 1'b0;
        pll_locked_sync <= 1'b0;
    end
    else begin
        pll_locked_meta <= board_pll_locked;
        pll_locked_sync <= pll_locked_meta;
    end
end

always @(posedge sys_clk or negedge external_reset_n) begin
    if(!external_reset_n) begin
        startup_count  <= 32'd0;
        system_reset_n <= 1'b0;
    end
    else if(!pll_locked_sync) begin
        startup_count  <= 32'd0;
        system_reset_n <= 1'b0;
    end
    else if(!system_reset_n) begin
        if(startup_count >= STARTUP_DELAY_CYCLES-1)
            system_reset_n <= 1'b1;
        else
            startup_count <= startup_count + 1'b1;
    end
end

always @(posedge sys_clk or negedge system_reset_n) begin
    if(!system_reset_n)
        ddr_reset_pipe <= 2'b00;
    else
        ddr_reset_pipe <= {ddr_reset_pipe[0],1'b1};
end

assign ddr_reset_n = ddr_reset_pipe[1];

endmodule
