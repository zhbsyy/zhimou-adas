module video_packet_rec(
	input rst,
	input rx_clk,
	input[31:0] gt_rx_data,
	input[3:0] gt_rx_ctrl,
	input[15:0] vout_width,
	
	output vs,
	output reg de,
	output[15:0] vout_data
);
	reg[31:0] gt_rx_data_d0;
	reg[3:0] gt_rx_ctrl_d0;
reg[31:0] gt_rx_data_d1;
	reg[3:0] gt_rx_ctrl_d1;
reg[31:0] gt_rx_data_d2;
	reg[3:0] gt_rx_ctrl_d2;
reg vs_r;
reg[15:0] vs_cnt;
always@(posedge rx_clk )
begin
gt_rx_data_d2<=gt_rx_data_d1;
gt_rx_ctrl_d2<=gt_rx_ctrl_d1;
gt_rx_data_d1<=gt_rx_data_d0;
gt_rx_ctrl_d1<=gt_rx_ctrl_d0;
gt_rx_data_d0<=gt_rx_data;
gt_rx_ctrl_d0<=gt_rx_ctrl;
end
always@(posedge rx_clk or posedge rst)
begin
    if(rst)
        vs_cnt <= 16'd0;
    else if(vs_r)
        vs_cnt <= vs_cnt + 16'd1;
    else
        vs_cnt <= 16'd0;
end

//�ָ�֡ͬ���ź�
always@(posedge rx_clk or posedge rst)
begin
	if(rst)
		vs_r <= 1'b0;
else if(gt_rx_ctrl == 4'b0001 && gt_rx_data == 32'hff_00_00_bc )    
		vs_r <= 1'b1;
	else if(vs_cnt > 16'd100)
		vs_r <= 1'b0;
end
localparam IDLE           = 0;
localparam READ_LINE      = 1;
localparam SEND_LINE_END  = 2;

reg[2:0] state /* synthesis PAP_MARK_DEBUG="true" */;
reg[15:0] data_cnt;
reg[15:0] wr_cnt;


wire buffer_rd_en;
reg wr_en;
wire rd_empty /* synthesis PAP_MARK_DEBUG="true" */;
wire wr_full  /* synthesis PAP_MARK_DEBUG="true" */;
wire wr_accept /* synthesis PAP_MARK_DEBUG="true" */;
wire rd_accept /* synthesis PAP_MARK_DEBUG="true" */;
reg [12:0] pixel_count /* synthesis PAP_MARK_DEBUG="true" */;
reg overflow_seen /* synthesis PAP_MARK_DEBUG="true" */;
reg underflow_seen /* synthesis PAP_MARK_DEBUG="true" */;
reg frame_error /* synthesis PAP_MARK_DEBUG="true" */;

assign wr_accept = wr_en && !wr_full && !frame_error;
assign rd_accept = (state == READ_LINE) && !rd_empty &&
                   (pixel_count != 13'd0) && !frame_error;
assign buffer_rd_en = rd_accept;
assign vs = vs_r;

fifo_2048_32i_16o fifo_2048_32i_16o_m0(
    .wr_clk(rx_clk),
    .wr_rst(vs_r),
    .wr_en(wr_accept),
    .wr_data(gt_rx_data),
    .wr_full(wr_full),
    .almost_full(),
    .rd_clk(rx_clk),
    .rd_rst(vs_r),
    .rd_en(buffer_rd_en),
    .rd_data(vout_data),
    .rd_empty(rd_empty),
    .almost_empty());

// Track FIFO occupancy in 16-bit pixel units.  One accepted 32-bit write
// contributes two pixels and one accepted read consumes one pixel.
always@(posedge rx_clk or posedge rst) begin
    if(rst)
        pixel_count <= 13'd0;
    else if(vs_r || frame_error)
        pixel_count <= 13'd0;
    else begin
        case ({wr_accept, rd_accept})
            2'b10: pixel_count <= pixel_count + 13'd2;
            2'b01: pixel_count <= pixel_count - 13'd1;
            2'b11: pixel_count <= pixel_count + 13'd1;
            default: pixel_count <= pixel_count;
        endcase
    end
end

// Keep fault information sticky for the current frame.  Once an overflow or
// underflow is detected, suppress the rest of the frame and resynchronize on
// the next frame marker.
always@(posedge rx_clk or posedge rst) begin
    if(rst) begin
        overflow_seen <= 1'b0;
        underflow_seen <= 1'b0;
        frame_error <= 1'b0;
    end else if(vs_r) begin
        overflow_seen <= 1'b0;
        underflow_seen <= 1'b0;
        frame_error <= 1'b0;
    end else begin
        if(wr_en && wr_full) begin
            overflow_seen <= 1'b1;
            frame_error <= 1'b1;
        end
        if((state == READ_LINE) && (rd_empty || (pixel_count == 13'd0))) begin
            underflow_seen <= 1'b1;
            frame_error <= 1'b1;
        end
    end
end

always@(posedge rx_clk or posedge rst)
begin
	if(rst)
		de <= 1'b0;
	else if(vs_r || frame_error)
		de <= 1'b0;
	else
		de <= rd_accept;
end

//������ͬ���ź�
always@(posedge rx_clk or posedge rst)
begin
	if(rst)
		wr_en <= 1'b0;
	else if(vs_r || frame_error)
		wr_en <= 1'b0;
	else if(gt_rx_ctrl == 4'b0001 && gt_rx_data == 32'hff_00_02_bc)
		wr_en <= 1'b1;
	else if(wr_cnt == ({1'b0,vout_width[15:1]} - 16'd1))
		wr_en <= 1'b0;
		
end

always@(posedge rx_clk or posedge rst)
begin
	if(rst)
		wr_cnt <= 16'd0;
	else if(vs_r || frame_error)
		wr_cnt <= 16'd0;
	else if(wr_accept)
		wr_cnt <= wr_cnt + 16'd1;
	else
		wr_cnt <= 16'd0;
end

always@(posedge rx_clk or posedge rst)
begin
	if(rst)
		data_cnt <= 16'd0;
	else if(vs_r || frame_error)
		data_cnt <= 16'd0;
	else if(rd_accept)
		data_cnt <= data_cnt + 16'd1;
	else if(state != READ_LINE)
		data_cnt <= 16'd0;
end
always@(posedge rx_clk or posedge rst)
begin
	if(rst)
	begin
		state <= IDLE;
	end
	else if(vs_r || frame_error)
	begin
		state <= IDLE;
	end
	else
	begin
		case(state)
			IDLE:
			begin
				// Start only after one complete line of 16-bit pixels is buffered.
				if({3'd0, pixel_count} >= vout_width)
					state <= READ_LINE;
				else
					state <= IDLE;
			end
			READ_LINE:
			begin
				if(data_cnt == (vout_width[15:0]- 16'd1))
					state <= IDLE;
				else
					state <= READ_LINE;
			end
			
		endcase
	end
end

endmodule 
