`timescale 1ns/1ps
module frame_fifo_write
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter ADDR_BITS              = 23,
	parameter BUSRT_BITS             = 10,
	parameter BURST_SIZE             = 128
)               
(
	input                            rst,                  
	input                            mem_clk,                    // external memory controller user interface clock
	output reg                       wr_burst_req,               // to external memory controller,send out a burst write request  
	output reg[BUSRT_BITS - 1:0]     wr_burst_len,               // to external memory controller,data length of the burst write request, not bytes 
	output reg[ADDR_BITS - 1:0]      wr_burst_addr,              // to external memory controller,base address of the burst write request 
	input                            wr_burst_data_req,          // from external memory controller,write data request ,before data 1 clock 
	input                            wr_burst_finish,            // from external memory controller,burst write finish
	input                            write_req,                  // data write module write request,keep '1' until read_req_ack = '1'
	output reg                       write_req_ack,              // data write module write request response
	output                           write_finish,               // data write module write request finish
	input[ADDR_BITS - 1:0]           write_addr_0,               // data write module write request base address 0, used when write_addr_index = 0
	input[ADDR_BITS - 1:0]           write_addr_1,               // data write module write request base address 1, used when write_addr_index = 1
	input[ADDR_BITS - 1:0]           write_addr_2,               // data write module write request base address 1, used when write_addr_index = 2
	input[ADDR_BITS - 1:0]           write_addr_3,               // data write module write request base address 1, used when write_addr_index = 3
	input[1:0]                       write_addr_index,           // select valid base address from write_addr_0 write_addr_1 write_addr_2 write_addr_3
	input                            write_tag,                  // configuration generation associated with this frame
	input                            write_frame_error,          // invalidates the current frame after a FIFO overflow
	input                            write_frame_valid,          // complete source-frame format validation
	input                            write_frame_valid_toggle,   // one toggle per source VS validation
	input                            write_validation_required,  // zero for HDMI, one for strict camera frames
	input[ADDR_BITS - 1:0]           write_len,                  // data write module write request data length
	output reg                       fifo_aclr,                  // to fifo asynchronous clear
	input[15:0]                      rdusedw,                    // from fifo read used words
	output reg                       finish_toggle,
	output reg                       finish_tag,
	output reg[1:0]                  finish_addr_index
);
localparam ONE                       = 256'd1;                   //256 bit '1'   you can use ONE[n-1:0] for n bit '1'
localparam ZERO                      = 256'd0;                   //256 bit '0'
//write state machine code
localparam S_IDLE                    = 0;                        //idle state,waiting for write
localparam S_ACK                     = 1;                        //written request response
localparam S_CHECK_FIFO              = 2;                        //check the FIFO status, ensure that there is enough space to burst write
localparam S_WRITE_BURST             = 3;                        //begin a burst write
localparam S_WRITE_BURST_END         = 4;                        //a burst write complete
localparam S_END                     = 5;                        //a frame of data is written to complete

reg                                 write_req_d0;                //asynchronous write request, synchronize to 'mem_clk' clock domain,first beat
reg                                 write_req_d1;                //the second
reg                                 write_req_d2;                //third,Why do you need 3 ? Here's the design habit
reg[ADDR_BITS - 1:0]                write_len_d0;                //asynchronous write_len(write data length), synchronize to 'mem_clk' clock domain first
reg[ADDR_BITS - 1:0]                write_len_d1;                //second
reg[ADDR_BITS - 1:0]                write_len_latch;             //lock write data length
reg[ADDR_BITS - 1:0]                write_cnt;                   //write data counter
reg[1:0]                            write_addr_index_d0;
reg[1:0]                            write_addr_index_d1;
reg[1:0]                            write_addr_index_latch;
reg                                 write_tag_d0;
reg                                 write_tag_d1;
reg                                 write_tag_latch;
(* ASYNC_REG="TRUE" *) reg          write_error_d0;
(* ASYNC_REG="TRUE" *) reg          write_error_d1;
(* ASYNC_REG="TRUE" *) reg          write_valid_d0, write_valid_d1;
(* ASYNC_REG="TRUE" *) reg          valid_toggle_d0, valid_toggle_d1;
(* ASYNC_REG="TRUE" *) reg          validation_required_d0, validation_required_d1;
reg                                 frame_bad_latch;
reg                                 validation_toggle_latch;
reg                                 validation_seen_latch;
reg                                 validation_good_latch;
reg                                 validation_required_latch;
reg[3:0]                            state;                       //state machine

wire validation_event = valid_toggle_d1 ^ validation_toggle_latch;
wire validation_pass = !validation_required_latch ||
                       (validation_seen_latch && validation_good_latch) ||
                       (validation_event && write_valid_d1);
wire validation_fail = validation_required_latch && validation_event && !write_valid_d1;
assign write_finish = ((state == S_END) && !frame_bad_latch && !write_error_d1 &&
                       validation_pass && !validation_fail) ? 1'b1 : 1'b0;
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		write_req_d0    <=  1'b0;
		write_req_d1    <=  1'b0;
		write_req_d2    <=  1'b0;
		write_len_d0    <=  ZERO[ADDR_BITS - 1:0];              //equivalent to write_len_d0 <= 0;
		write_len_d1    <=  ZERO[ADDR_BITS - 1:0];              //equivalent to write_len_d1 <= 0;
		write_addr_index_d0    <=  2'b00;
		write_addr_index_d1    <=  2'b00;
		write_tag_d0           <=  1'b0;
		write_tag_d1           <=  1'b0;
		write_error_d0         <=  1'b0;
		write_error_d1         <=  1'b0;
		write_valid_d0         <=  1'b0;
		write_valid_d1         <=  1'b0;
		valid_toggle_d0        <=  1'b0;
		valid_toggle_d1        <=  1'b0;
		validation_required_d0 <=  1'b0;
		validation_required_d1 <=  1'b0;
	end
	else
	begin
		write_req_d0    <=  write_req;
		write_req_d1    <=  write_req_d0;
		write_req_d2    <=  write_req_d1;
		write_len_d0    <=  write_len;
		write_len_d1    <=  write_len_d0;
		write_addr_index_d0 <= write_addr_index;
		write_addr_index_d1 <= write_addr_index_d0;
		write_tag_d0 <= write_tag;
		write_tag_d1 <= write_tag_d0;
		write_error_d0 <= write_frame_error;
		write_error_d1 <= write_error_d0;
		write_valid_d0 <= write_frame_valid;
		write_valid_d1 <= write_valid_d0;
		valid_toggle_d0 <= write_frame_valid_toggle;
		valid_toggle_d1 <= valid_toggle_d0;
		validation_required_d0 <= write_validation_required;
		validation_required_d1 <= validation_required_d0;
	end 
end


always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		write_len_latch <= ZERO[ADDR_BITS - 1:0];
		wr_burst_addr <= ZERO[ADDR_BITS - 1:0];
		wr_burst_req <= 1'b0;
		write_cnt <= ZERO[ADDR_BITS - 1:0];
		fifo_aclr <= 1'b0;
		write_req_ack <= 1'b0;
		wr_burst_len <= ZERO[BUSRT_BITS - 1:0];
		write_addr_index_latch <= 2'b00;
		write_tag_latch <= 1'b0;
		finish_toggle <= 1'b0;
		finish_tag <= 1'b0;
		finish_addr_index <= 2'b00;
		frame_bad_latch <= 1'b0;
		validation_toggle_latch <= 1'b0;
		validation_seen_latch <= 1'b0;
		validation_good_latch <= 1'b0;
		validation_required_latch <= 1'b0;
	end
	else
	begin
		if(write_error_d1)
			frame_bad_latch <= 1'b1;
		if(validation_required_latch && validation_event)
		begin
			validation_toggle_latch <= valid_toggle_d1;
			validation_seen_latch <= 1'b1;
			validation_good_latch <= write_valid_d1;
			if(!write_valid_d1)
				frame_bad_latch <= 1'b1;
		end
		case(state)
			//idle state,waiting for write write_req_d2 == '1' goto the 'S_ACK'
			S_IDLE:
			begin
				fifo_aclr <= 1'b0;
				wr_burst_req <= 1'b0;
				if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				write_req_ack <= 1'b0;
			end
			//'S_ACK' state completes the write request response, the FIFO reset, the address latch, and the data length latch
			S_ACK:
			begin
				//after write request revocation(write_req_d2 == '0'),goto 'S_CHECK_FIFO',write_req_ack goto '0'
				if(write_req_d2 == 1'b0)
				begin
					state <= S_CHECK_FIFO;
					fifo_aclr <= 1'b0;
					write_req_ack <= 1'b0;
				end
				else
				begin
					//write request response
					write_req_ack <= 1'b1;
					//FIFO reset
					fifo_aclr <= 1'b1;
					//select valid base address from write_addr_0 write_addr_1 write_addr_2 write_addr_3
					if(write_addr_index_d1 == 2'd0)
						wr_burst_addr <= write_addr_0;
					else if(write_addr_index_d1 == 2'd1)
						wr_burst_addr <= write_addr_1;
					else if(write_addr_index_d1 == 2'd2)
						wr_burst_addr <= write_addr_2;
					else if(write_addr_index_d1 == 2'd3)
						wr_burst_addr <= write_addr_3;
					//latch data length
					write_len_latch <= write_len_d1;
					write_addr_index_latch <= write_addr_index_d1;
					write_tag_latch <= write_tag_d1;
					frame_bad_latch <= write_error_d1;
					validation_toggle_latch <= valid_toggle_d1;
					validation_seen_latch <= !validation_required_d1;
					validation_good_latch <= !validation_required_d1;
					validation_required_latch <= validation_required_d1;
				end
				//write data counter reset, write_cnt <= 0;
				write_cnt <= ZERO[ADDR_BITS - 1:0];
			end
			S_CHECK_FIFO:
			begin
				// An invalid frame may be discarded only while no DDR burst is active.
				if(frame_bad_latch || write_error_d1 || validation_fail)
				begin
					state <= S_IDLE;
					fifo_aclr <= 1'b1;
					write_cnt <= ZERO[ADDR_BITS - 1:0];
				end
				//if there is a write request at this time, enter the 'S_ACK' state
				else if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				//if the FIFO space is a burst write request, goto burst write state
				else if(rdusedw >= BURST_SIZE)
				begin
					state <= S_WRITE_BURST;
					wr_burst_len <= BURST_SIZE[BUSRT_BITS - 1:0];
					wr_burst_req <= 1'b1;
				end
			end
			
			S_WRITE_BURST:
			begin
				//burst finish
				if(wr_burst_finish == 1'b1)
				begin
					wr_burst_req <= 1'b0;
					state <= S_WRITE_BURST_END;
					//write counter + burst length
					write_cnt <= write_cnt + BURST_SIZE[ADDR_BITS - 1:0];
					//the next burst write address is generated
					wr_burst_addr <= wr_burst_addr + BURST_SIZE[ADDR_BITS - 1:0];
				end     
			end
			S_WRITE_BURST_END:
			begin
				if(frame_bad_latch || write_error_d1)
				begin
					state <= S_IDLE;
					fifo_aclr <= 1'b1;
					write_cnt <= ZERO[ADDR_BITS - 1:0];
				end
				//if there is a write request at this time, enter the 'S_ACK' state
				else if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				//if the write counter value is less than the frame length, continue writing,
				//otherwise the writing is complete
				else if(write_cnt < write_len_latch)
				begin
					state <= S_CHECK_FIFO;
				end
				else
				begin
					state <= S_END;
				end
			end
			S_END:
			begin
				if(frame_bad_latch || write_error_d1)
				begin
					fifo_aclr <= 1'b1;
					state <= S_IDLE;
				end
				else if(validation_pass)
				begin
					finish_toggle <= ~finish_toggle;
					finish_tag <= write_tag_latch;
					finish_addr_index <= write_addr_index_latch;
					state <= S_IDLE;
				end
				else if(write_req_d2)
				begin
					// A new VS arrived before validation of the preceding
					// frame. Discard it and immediately accept the new one.
					state <= S_ACK;
				end
			end
			default:
				state <= S_IDLE;
		endcase
	end
end
endmodule
