`timescale 1ns/1ps

// Read-only view of an existing DDR frame buffer.  The write side remains in
// frame_read_write; this module gives the HDMI renderer an independent FIFO
// and request stream without allocating another set of frame addresses.
module frame_read_only
#(
    parameter MEM_DATA_BITS  = 256,
    parameter READ_DATA_BITS = 16,
    parameter ADDR_BITS      = 25,
    parameter BUSRT_BITS     = 10,
    parameter BURST_SIZE     = 16
)
(
    input                           rst,
    input                           mem_clk,
    output                          rd_burst_req,
    output [BUSRT_BITS-1:0]         rd_burst_len,
    output [ADDR_BITS-1:0]          rd_burst_addr,
    input                           rd_burst_data_valid,
    input  [MEM_DATA_BITS-1:0]      rd_burst_data,
    input                           rd_burst_finish,

    input                           read_clk,
    input                           read_req,
    output                          read_req_ack,
    output                          read_finish,
    input  [ADDR_BITS-1:0]          read_addr_0,
    input  [ADDR_BITS-1:0]          read_addr_1,
    input  [ADDR_BITS-1:0]          read_addr_2,
    input  [ADDR_BITS-1:0]          read_addr_3,
    input  [1:0]                    read_addr_index,
    input  [ADDR_BITS-1:0]          read_len,
    input                           read_en,
    output [READ_DATA_BITS-1:0]     read_data,
    output                          read_fifo_empty,
    output                          read_fifo_full,
    input                           read_status_clear,
    output reg                      read_overflow_sticky
);

wire [15:0] wrusedw;
wire        read_fifo_aclr;
(* ASYNC_REG="TRUE" *) reg read_status_clear_meta;
(* ASYNC_REG="TRUE" *) reg read_status_clear_sync;

afifo_64i_16o_128 read_buf (
    .wr_clk         (mem_clk),
    .wr_rst         (read_fifo_aclr),
    .wr_en          (rd_burst_data_valid && !read_fifo_full),
    .wr_data        (rd_burst_data),
    .wr_full        (read_fifo_full),
    .wr_water_level (wrusedw[8:0]),
    .almost_full    (),
    .rd_clk         (read_clk),
    .rd_rst         (read_fifo_aclr),
    .rd_en          (read_en && !read_fifo_empty),
    .rd_data        (read_data),
    .rd_empty       (read_fifo_empty),
    .rd_water_level (),
    .almost_empty   ()
);

always @(posedge mem_clk or posedge rst) begin
    if(rst) begin
        read_status_clear_meta <= 1'b0;
        read_status_clear_sync <= 1'b0;
        read_overflow_sticky   <= 1'b0;
    end
    else begin
        read_status_clear_meta <= read_status_clear;
        read_status_clear_sync <= read_status_clear_meta;
        if(read_status_clear_sync)
            read_overflow_sticky <= 1'b0;
        else if(rd_burst_data_valid && read_fifo_full)
            read_overflow_sticky <= 1'b1;
    end
end

frame_fifo_read
#(
    .MEM_DATA_BITS (MEM_DATA_BITS),
    .ADDR_BITS     (ADDR_BITS),
    .BUSRT_BITS    (BUSRT_BITS),
    .FIFO_DEPTH    (128),
    .BURST_SIZE    (BURST_SIZE)
)
frame_fifo_read_inst (
    .rst             (rst),
    .mem_clk         (mem_clk),
    .rd_burst_req    (rd_burst_req),
    .rd_burst_len    (rd_burst_len),
    .rd_burst_addr   (rd_burst_addr),
    .rd_burst_data_valid(rd_burst_data_valid),
    .rd_burst_finish (rd_burst_finish),
    .read_req        (read_req),
    .read_req_ack    (read_req_ack),
    .read_finish     (read_finish),
    .read_addr_0     (read_addr_0),
    .read_addr_1     (read_addr_1),
    .read_addr_2     (read_addr_2),
    .read_addr_3     (read_addr_3),
    .read_addr_index (read_addr_index),
    .read_len        (read_len),
    .fifo_aclr       (read_fifo_aclr),
    .wrusedw         (wrusedw)
);

endmodule
