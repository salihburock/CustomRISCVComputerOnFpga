module VGA_Controller (
    input  wire sys_clk,      // MUST BE 25 MHz for 640x480!
    input  wire sys_rst_n,
    output wire [13:0] vram_read_addr,
    output wire        vram_read_en,
    input  wire [31:0] vram_read_data,
    output wire h_sync_i,
    output wire v_sync_i,
    output reg  [2:0] RED,
    output reg  [2:0] GREEN,
    output reg  [1:0] BLUE
);

// 640x480 @ 60Hz Industry Standard Timings (25 MHz Clock)
parameter H_ACTIVE = 640;
parameter H_FRONT  = 16;
parameter H_SYNC   = 96;
parameter H_BACK   = 48;
parameter H_TOTAL  = 800;

parameter V_ACTIVE = 480;
parameter V_FRONT  = 10;
parameter V_SYNC   = 2;
parameter V_BACK   = 33;
parameter V_TOTAL  = 525;

reg [9:0] h_cnt = 0;
reg [9:0] v_cnt = 0;

// Active LOW Sync Pulses (Standard for 640x480)
assign h_sync_i = ~(h_cnt >= (H_ACTIVE + H_FRONT) && h_cnt < (H_ACTIVE + H_FRONT + H_SYNC));
assign v_sync_i = ~(v_cnt >= (V_ACTIVE + V_FRONT) && v_cnt < (V_ACTIVE + V_FRONT + V_SYNC));

// ── Look-ahead counters (1 cycle in the future) ───────────────────────────
// SDPB BRAM is in "bypass" read_mode → 1 cycle latency from addr to dout.
// Present the address for the NEXT pixel so data lands aligned with it.
wire [9:0] h_cnt_n = (h_cnt == H_TOTAL - 1) ? 10'd0 : h_cnt + 10'd1;
wire [9:0] v_cnt_n = (h_cnt == H_TOTAL - 1)
                     ? ((v_cnt == V_TOTAL - 1) ? 10'd0 : v_cnt + 10'd1)
                     : v_cnt;

// Look-ahead address generation (used by BRAM this cycle)
wire image_active_n         = (h_cnt_n < 640) && (v_cnt_n >= 40) && (v_cnt_n < 440);
wire [8:0]  pixel_x_n       = h_cnt_n[9:1];
wire [7:0]  pixel_y_n       = (v_cnt_n - 10'd40) >> 1;
wire [15:0] absolute_pixel_n = (pixel_y_n * 320) + pixel_x_n;

// Current-pixel decode (selectors for the data arriving NOW)
wire image_active        = (h_cnt < 640) && (v_cnt >= 40) && (v_cnt < 440);
wire [8:0]  pixel_x      = h_cnt[9:1];
wire [7:0]  pixel_y      = (v_cnt - 40) >> 1;
wire [15:0] absolute_pixel = (pixel_y * 320) + pixel_x;
wire [1:0]  sub_pixel    = absolute_pixel[1:0];

// VRAM handles 32 bits (4 pixels) per address — present LOOK-AHEAD addr
assign vram_read_addr = absolute_pixel_n[15:2];
assign vram_read_en   = image_active_n;

always @(posedge sys_clk) begin
    if (!sys_rst_n) begin
        h_cnt <= 0;
        v_cnt <= 0;
        RED <= 0; GREEN <= 0; BLUE <= 0;
    end else begin
        // Counters
        if (h_cnt == H_TOTAL - 1) begin
            h_cnt <= 0;
            if (v_cnt == V_TOTAL - 1) v_cnt <= 0;
            else v_cnt <= v_cnt + 1;
        end else begin
            h_cnt <= h_cnt + 1;
        end

        // Pixel Drawing — LSB-first byte order: RISC-V (little-endian) stores
        // pixel 0 at bits[7:0], pixel 3 at bits[31:24] of each 32-bit word.
        if (image_active) begin
            case (sub_pixel)
                2'd0: begin RED <= vram_read_data[7:5];   GREEN <= vram_read_data[4:2];   BLUE <= vram_read_data[1:0];  end
                2'd1: begin RED <= vram_read_data[15:13]; GREEN <= vram_read_data[12:10]; BLUE <= vram_read_data[9:8];   end
                2'd2: begin RED <= vram_read_data[23:21]; GREEN <= vram_read_data[20:18]; BLUE <= vram_read_data[17:16]; end
                2'd3: begin RED <= vram_read_data[31:29]; GREEN <= vram_read_data[28:26]; BLUE <= vram_read_data[25:24]; end
            endcase
        end else begin
            RED <= 0; GREEN <= 0; BLUE <= 0; // Draw Black Cinematic Borders
        end
    end
end
endmodule
