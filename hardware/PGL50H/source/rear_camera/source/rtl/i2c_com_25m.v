`timescale 1ns/1ps

// Four-byte SCCB writer. State advances from a clock-enable while every
// register remains in the 25 MHz domain. clock_i2c is only a waveform used to
// form SCL; it is never used as a logic clock.
module i2c_com_25m(
    input  wire        clk,
    input  wire        tick_20k,
    input  wire        clock_i2c,
    input  wire        camera_rstn,
    input  wire [31:0] i2c_data,
    input  wire        start,
    output wire        ack_error,
    output reg         tr_end,
    output wire        i2c_sclk,
    inout  wire        i2c_sdat
);

reg [5:0] cycle_count;
reg       release_sdat;
reg       force_sclk;
reg [3:0] nack;

assign ack_error = |nack;
assign i2c_sclk = force_sclk |
                  (((cycle_count >= 6'd4) && (cycle_count <= 6'd39)) ?
                   ~clock_i2c : 1'b0);
assign i2c_sdat = release_sdat ? 1'bz : 1'b0;

always @(posedge clk or negedge camera_rstn) begin
    if(!camera_rstn) begin
        cycle_count <= 6'd0;
        release_sdat <= 1'b1;
        force_sclk <= 1'b1;
        nack <= 4'b1111;
        tr_end <= 1'b0;
    end
    else if(tick_20k) begin
        if(!start)
            cycle_count <= 6'd0;
        else if(cycle_count < 6'd63)
            cycle_count <= cycle_count + 1'b1;

        case(cycle_count)
            6'd0:  begin nack<=4'b1111; tr_end<=1'b0; force_sclk<=1'b1; release_sdat<=1'b1; end
            6'd1:  release_sdat<=1'b0;
            6'd2:  force_sclk<=1'b0;
            6'd3:  release_sdat<=i2c_data[31];
            6'd4:  release_sdat<=i2c_data[30];
            6'd5:  release_sdat<=i2c_data[29];
            6'd6:  release_sdat<=i2c_data[28];
            6'd7:  release_sdat<=i2c_data[27];
            6'd8:  release_sdat<=i2c_data[26];
            6'd9:  release_sdat<=i2c_data[25];
            6'd10: release_sdat<=i2c_data[24];
            6'd11: release_sdat<=1'b1;
            6'd12: begin release_sdat<=i2c_data[23]; nack[0]<=i2c_sdat; end
            6'd13: release_sdat<=i2c_data[22];
            6'd14: release_sdat<=i2c_data[21];
            6'd15: release_sdat<=i2c_data[20];
            6'd16: release_sdat<=i2c_data[19];
            6'd17: release_sdat<=i2c_data[18];
            6'd18: release_sdat<=i2c_data[17];
            6'd19: release_sdat<=i2c_data[16];
            6'd20: release_sdat<=1'b1;
            6'd21: begin release_sdat<=i2c_data[15]; nack[1]<=i2c_sdat; end
            6'd22: release_sdat<=i2c_data[14];
            6'd23: release_sdat<=i2c_data[13];
            6'd24: release_sdat<=i2c_data[12];
            6'd25: release_sdat<=i2c_data[11];
            6'd26: release_sdat<=i2c_data[10];
            6'd27: release_sdat<=i2c_data[9];
            6'd28: release_sdat<=i2c_data[8];
            6'd29: release_sdat<=1'b1;
            6'd30: begin release_sdat<=i2c_data[7]; nack[2]<=i2c_sdat; end
            6'd31: release_sdat<=i2c_data[6];
            6'd32: release_sdat<=i2c_data[5];
            6'd33: release_sdat<=i2c_data[4];
            6'd34: release_sdat<=i2c_data[3];
            6'd35: release_sdat<=i2c_data[2];
            6'd36: release_sdat<=i2c_data[1];
            6'd37: release_sdat<=i2c_data[0];
            6'd38: release_sdat<=1'b1;
            6'd39: begin nack[3]<=i2c_sdat; force_sclk<=1'b0; release_sdat<=1'b0; end
            6'd40: force_sclk<=1'b1;
            6'd41: begin release_sdat<=1'b1; tr_end<=1'b1; end
            default: begin end
        endcase
    end
end

endmodule
