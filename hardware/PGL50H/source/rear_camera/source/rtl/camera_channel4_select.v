// OV5640 RGB565 byte/colour reorder and channel-4 mode selection.
module camera_channel4_select
(
    input  wire [7:0]  display_mode,
    input  wire [15:0] packed_rgb565,
    input  wire         packed_de,
    input  wire [15:0] scaled_rgb565,
    input  wire         scaled_de,
    output wire         write_enable,
    output wire         write_full_size,
    output wire [15:0]  rgb565,
    output wire [15:0]  write_data,
    output wire         write_de,
    output wire [24:0]  write_len
);

localparam [24:0] FRAME_LEN_CAMERA = 25'd57600;
localparam [24:0] FRAME_LEN_SCALED = 25'd32400;

assign write_enable    = (display_mode == 8'd4) || (display_mode == 8'd7);
assign write_full_size = (display_mode == 8'd4);

// The OV5640 reference design emits two RGB565 bytes in this order.
assign rgb565 = {packed_rgb565[4:0], packed_rgb565[10:5], packed_rgb565[15:11]};

assign write_data = write_full_size ? rgb565 : scaled_rgb565;
assign write_de   = write_enable && (write_full_size ? packed_de : scaled_de);
assign write_len  = write_enable ?
                    (write_full_size ? FRAME_LEN_CAMERA : FRAME_LEN_SCALED) : 25'd0;

endmodule
