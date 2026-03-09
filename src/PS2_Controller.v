module PS2_Controller(
    input sys_clk,     // 100MHz ddr_user_clk
    input sys_rst_n,
    input PS2_D_I,
    input PS2_CLK_I,
    output reg [31:0] DOOM_KEYS = 32'd0 
);

// --- 1. Level Shifter Inversion ---
wire ps2_d_raw   = !PS2_D_I; 
wire ps2_clk_raw = !PS2_CLK_I;

// --- 2. Lighter Digital Debounce Filter (320ns) ---
reg [4:0] clk_filter = 0;
reg [4:0] dat_filter = 0;
reg ps2_clk_clean = 1;
reg ps2_dat_clean = 1;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        clk_filter <= 5'd0;
        dat_filter <= 5'd0;
        ps2_clk_clean <= 1'b1;
        ps2_dat_clean <= 1'b1;
    end else begin
        // Filter the Clock
        if (ps2_clk_raw == 1'b1) begin
            if (clk_filter != 5'h1F) clk_filter <= clk_filter + 1;
            else ps2_clk_clean <= 1'b1;
        end else begin
            if (clk_filter != 5'h00) clk_filter <= clk_filter - 1;
            else ps2_clk_clean <= 1'b0;
        end
        
        // Filter the Data
        if (ps2_d_raw == 1'b1) begin
            if (dat_filter != 5'h1F) dat_filter <= dat_filter + 1;
            else ps2_dat_clean <= 1'b1;
        end else begin
            if (dat_filter != 5'h00) dat_filter <= dat_filter - 1;
            else ps2_dat_clean <= 1'b0;
        end
    end
end

// --- 3. Falling Edge Detection ---
reg ps2_clk_prev = 1'b1;
wire clk_falling_edge = (!ps2_clk_clean && ps2_clk_prev);

// --- 4. Timeout Watchdog ---
reg [16:0] timeout_counter = 0;
wire frame_timeout = (timeout_counter >= 17'd100_000); 

// --- 5. Direct-Index PS/2 Receiver ---
reg [10:0] shift_reg = 0;
reg [3:0]  bit_count = 0;
reg [7:0]  latest_byte = 0;
reg        byte_ready = 0;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        ps2_clk_prev <= 1'b1;
        shift_reg <= 0;
        bit_count <= 0;
        latest_byte <= 0;
        byte_ready <= 0;
        timeout_counter <= 0;
    end else begin
        ps2_clk_prev <= ps2_clk_clean;
        byte_ready <= 0; 
        
        if (bit_count > 0) begin
            timeout_counter <= timeout_counter + 1;
            if (frame_timeout) bit_count <= 0; // Hard reset on stall
        end else begin
            timeout_counter <= 0;
        end

        if (clk_falling_edge) begin
            timeout_counter <= 0; 
            
            if (bit_count == 0) begin
                // Wait for Start Bit (Must be 0)
                if (ps2_dat_clean == 1'b0) begin
                    shift_reg[0] <= ps2_dat_clean;
                    bit_count <= 1;
                end
            end else begin
                // Directly map incoming bits to their proper array slot
                shift_reg[bit_count] <= ps2_dat_clean;
                
                if (bit_count == 10) begin
                    // Frame complete! (bit 0 = Start, bits 8:1 = Data, bit 10 = incoming Stop)
                    // We safely check shift_reg[0] because it was locked in 10 cycles ago!
                    if (shift_reg[0] == 1'b0 && ps2_dat_clean == 1'b1) begin
                        latest_byte <= shift_reg[8:1];
                        byte_ready <= 1; 
                    end
                    bit_count <= 0; 
                end else begin
                    bit_count <= bit_count + 1;
                end
            end
        end
    end
end

// --- 6. Protocol Decoder State Machine ---
localparam S_IDLE    = 2'd0;
localparam S_EXT     = 2'd1; 
localparam S_RELEASE = 2'd2; 

reg [1:0] dec_state = S_IDLE;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        dec_state <= S_IDLE;
        DOOM_KEYS <= 32'd0;
    end else if (byte_ready) begin
        case (dec_state)
            S_IDLE: begin
                if (latest_byte == 8'hE0) dec_state <= S_EXT;
                else if (latest_byte == 8'hF0) dec_state <= S_RELEASE;
                else begin
                    // KEY PRESSED
                    case (latest_byte)
                        8'h1D: DOOM_KEYS[0] <= 1'b1; // W
                        8'h1B: DOOM_KEYS[1] <= 1'b1; // S
                        8'h1C: DOOM_KEYS[2] <= 1'b1; // A
                        8'h23: DOOM_KEYS[3] <= 1'b1; // D
                        8'h14: DOOM_KEYS[4] <= 1'b1; // L-Ctrl
                        8'h29: DOOM_KEYS[5] <= 1'b1; // Space
                        8'h12: DOOM_KEYS[6] <= 1'b1; // L-Shift
                        8'h76: DOOM_KEYS[7] <= 1'b1; // Esc
                        8'h5A: DOOM_KEYS[8] <= 1'b1; // Enter
                    endcase
                end
            end
            
            S_EXT: begin
                if (latest_byte == 8'hF0) dec_state <= S_RELEASE;
                else dec_state <= S_IDLE; 
            end
            
            S_RELEASE: begin
                // KEY RELEASED
                case (latest_byte)
                    8'h1D: DOOM_KEYS[0] <= 1'b0; // W
                    8'h1B: DOOM_KEYS[1] <= 1'b0; // S
                    8'h1C: DOOM_KEYS[2] <= 1'b0; // A
                    8'h23: DOOM_KEYS[3] <= 1'b0; // D
                    8'h14: DOOM_KEYS[4] <= 1'b0; // L-Ctrl
                    8'h29: DOOM_KEYS[5] <= 1'b0; // Space
                    8'h12: DOOM_KEYS[6] <= 1'b0; // L-Shift
                    8'h76: DOOM_KEYS[7] <= 1'b0; // Esc
                    8'h5A: DOOM_KEYS[8] <= 1'b0; // Enter
                endcase
                dec_state <= S_IDLE; 
            end
        endcase
    end
end

endmodule