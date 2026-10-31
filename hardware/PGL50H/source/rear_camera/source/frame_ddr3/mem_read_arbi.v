`timescale 1ns/1ps

// Fair eight-channel DDR read arbiter. Channels 0..3 feed SFP and 4..7 HDMI.
module mem_read_arbi
#(
    parameter MEM_DATA_BITS = 32,
    parameter ADDR_BITS     = 23,
    parameter BUSRT_BITS    = 10
)
(
    input rst_n,
    input mem_clk,
    input ch0_rd_burst_req, input [BUSRT_BITS-1:0] ch0_rd_burst_len, input [ADDR_BITS-1:0] ch0_rd_burst_addr,
    output ch0_rd_burst_data_valid, output [MEM_DATA_BITS-1:0] ch0_rd_burst_data, output ch0_rd_burst_finish,
    input ch1_rd_burst_req, input [BUSRT_BITS-1:0] ch1_rd_burst_len, input [ADDR_BITS-1:0] ch1_rd_burst_addr,
    output ch1_rd_burst_data_valid, output [MEM_DATA_BITS-1:0] ch1_rd_burst_data, output ch1_rd_burst_finish,
    input ch2_rd_burst_req, input [BUSRT_BITS-1:0] ch2_rd_burst_len, input [ADDR_BITS-1:0] ch2_rd_burst_addr,
    output ch2_rd_burst_data_valid, output [MEM_DATA_BITS-1:0] ch2_rd_burst_data, output ch2_rd_burst_finish,
    input ch3_rd_burst_req, input [BUSRT_BITS-1:0] ch3_rd_burst_len, input [ADDR_BITS-1:0] ch3_rd_burst_addr,
    output ch3_rd_burst_data_valid, output [MEM_DATA_BITS-1:0] ch3_rd_burst_data, output ch3_rd_burst_finish,
    input ch4_rd_burst_req, input [BUSRT_BITS-1:0] ch4_rd_burst_len, input [ADDR_BITS-1:0] ch4_rd_burst_addr,
    output ch4_rd_burst_data_valid, output [MEM_DATA_BITS-1:0] ch4_rd_burst_data, output ch4_rd_burst_finish,
    input ch5_rd_burst_req, input [BUSRT_BITS-1:0] ch5_rd_burst_len, input [ADDR_BITS-1:0] ch5_rd_burst_addr,
    output ch5_rd_burst_data_valid, output [MEM_DATA_BITS-1:0] ch5_rd_burst_data, output ch5_rd_burst_finish,
    input ch6_rd_burst_req, input [BUSRT_BITS-1:0] ch6_rd_burst_len, input [ADDR_BITS-1:0] ch6_rd_burst_addr,
    output ch6_rd_burst_data_valid, output [MEM_DATA_BITS-1:0] ch6_rd_burst_data, output ch6_rd_burst_finish,
    input ch7_rd_burst_req, input [BUSRT_BITS-1:0] ch7_rd_burst_len, input [ADDR_BITS-1:0] ch7_rd_burst_addr,
    output ch7_rd_burst_data_valid, output [MEM_DATA_BITS-1:0] ch7_rd_burst_data, output ch7_rd_burst_finish,
    output reg rd_burst_req,
    output reg [BUSRT_BITS-1:0] rd_burst_len,
    output reg [ADDR_BITS-1:0] rd_burst_addr,
    input rd_burst_data_valid,
    input [MEM_DATA_BITS-1:0] rd_burst_data,
    input rd_burst_finish
);

reg busy;
reg [2:0] owner;
reg [2:0] scan_index;
reg [15:0] timeout_count;
reg [7:0] finish_pulse;

wire [7:0] request_bus = {
    ch7_rd_burst_req && (ch7_rd_burst_len != 0),
    ch6_rd_burst_req && (ch6_rd_burst_len != 0),
    ch5_rd_burst_req && (ch5_rd_burst_len != 0),
    ch4_rd_burst_req && (ch4_rd_burst_len != 0),
    ch3_rd_burst_req && (ch3_rd_burst_len != 0),
    ch2_rd_burst_req && (ch2_rd_burst_len != 0),
    ch1_rd_burst_req && (ch1_rd_burst_len != 0),
    ch0_rd_burst_req && (ch0_rd_burst_len != 0)
};

always @(posedge mem_clk or negedge rst_n) begin
    if(!rst_n) begin
        busy          <= 1'b0;
        owner         <= 3'd0;
        scan_index    <= 3'd0;
        timeout_count <= 16'd0;
        finish_pulse  <= 8'd0;
        rd_burst_req  <= 1'b0;
        rd_burst_len  <= {BUSRT_BITS{1'b0}};
        rd_burst_addr <= {ADDR_BITS{1'b0}};
    end
    else begin
        finish_pulse <= 8'd0;
        if(!busy) begin
            timeout_count <= 16'd0;
            rd_burst_req  <= 1'b0;
            if(request_bus[scan_index]) begin
                busy         <= 1'b1;
                owner        <= scan_index;
                rd_burst_req <= 1'b1;
                case(scan_index)
                    3'd0: begin rd_burst_len<=ch0_rd_burst_len; rd_burst_addr<=ch0_rd_burst_addr; end
                    3'd1: begin rd_burst_len<=ch1_rd_burst_len; rd_burst_addr<=ch1_rd_burst_addr; end
                    3'd2: begin rd_burst_len<=ch2_rd_burst_len; rd_burst_addr<=ch2_rd_burst_addr; end
                    3'd3: begin rd_burst_len<=ch3_rd_burst_len; rd_burst_addr<=ch3_rd_burst_addr; end
                    3'd4: begin rd_burst_len<=ch4_rd_burst_len; rd_burst_addr<=ch4_rd_burst_addr; end
                    3'd5: begin rd_burst_len<=ch5_rd_burst_len; rd_burst_addr<=ch5_rd_burst_addr; end
                    3'd6: begin rd_burst_len<=ch6_rd_burst_len; rd_burst_addr<=ch6_rd_burst_addr; end
                    default: begin rd_burst_len<=ch7_rd_burst_len; rd_burst_addr<=ch7_rd_burst_addr; end
                endcase
            end
            else
                scan_index <= scan_index + 3'd1;
        end
        else begin
            timeout_count <= timeout_count + 16'd1;
            if(rd_burst_data_valid)
                rd_burst_req <= 1'b0;
            if(rd_burst_finish) begin
                busy                <= 1'b0;
                rd_burst_req        <= 1'b0;
                finish_pulse[owner] <= 1'b1;
                scan_index          <= owner + 3'd1;
                timeout_count       <= 16'd0;
            end
            else if(timeout_count >= 16'd8000) begin
                // Release the DDR port. The requester remains high and is retried.
                busy          <= 1'b0;
                rd_burst_req  <= 1'b0;
                scan_index    <= owner + 3'd1;
                timeout_count <= 16'd0;
            end
        end
    end
end

assign ch0_rd_burst_data_valid = busy && owner==3'd0 ? rd_burst_data_valid : 1'b0;
assign ch1_rd_burst_data_valid = busy && owner==3'd1 ? rd_burst_data_valid : 1'b0;
assign ch2_rd_burst_data_valid = busy && owner==3'd2 ? rd_burst_data_valid : 1'b0;
assign ch3_rd_burst_data_valid = busy && owner==3'd3 ? rd_burst_data_valid : 1'b0;
assign ch4_rd_burst_data_valid = busy && owner==3'd4 ? rd_burst_data_valid : 1'b0;
assign ch5_rd_burst_data_valid = busy && owner==3'd5 ? rd_burst_data_valid : 1'b0;
assign ch6_rd_burst_data_valid = busy && owner==3'd6 ? rd_burst_data_valid : 1'b0;
assign ch7_rd_burst_data_valid = busy && owner==3'd7 ? rd_burst_data_valid : 1'b0;

assign ch0_rd_burst_data = busy && owner==3'd0 ? rd_burst_data : {MEM_DATA_BITS{1'b0}};
assign ch1_rd_burst_data = busy && owner==3'd1 ? rd_burst_data : {MEM_DATA_BITS{1'b0}};
assign ch2_rd_burst_data = busy && owner==3'd2 ? rd_burst_data : {MEM_DATA_BITS{1'b0}};
assign ch3_rd_burst_data = busy && owner==3'd3 ? rd_burst_data : {MEM_DATA_BITS{1'b0}};
assign ch4_rd_burst_data = busy && owner==3'd4 ? rd_burst_data : {MEM_DATA_BITS{1'b0}};
assign ch5_rd_burst_data = busy && owner==3'd5 ? rd_burst_data : {MEM_DATA_BITS{1'b0}};
assign ch6_rd_burst_data = busy && owner==3'd6 ? rd_burst_data : {MEM_DATA_BITS{1'b0}};
assign ch7_rd_burst_data = busy && owner==3'd7 ? rd_burst_data : {MEM_DATA_BITS{1'b0}};

assign ch0_rd_burst_finish = finish_pulse[0];
assign ch1_rd_burst_finish = finish_pulse[1];
assign ch2_rd_burst_finish = finish_pulse[2];
assign ch3_rd_burst_finish = finish_pulse[3];
assign ch4_rd_burst_finish = finish_pulse[4];
assign ch5_rd_burst_finish = finish_pulse[5];
assign ch6_rd_burst_finish = finish_pulse[6];
assign ch7_rd_burst_finish = finish_pulse[7];

endmodule
