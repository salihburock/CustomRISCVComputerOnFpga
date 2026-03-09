module Main_Computer(
    input sys_clk,   // 27 MHz crystal for the PLL
    input sys_rst_n, // Reset button
    input MISO,
    output TX,
    output wire MOSI,
    output wire SPI_CS,
    output wire SPI_CLK,
    output wire debug_cs,
    output wire debug_clk,
    output wire debug_mosi,
    output wire debug_miso,
    
    // --- DDR3 Physical Pins (Mapped exactly to Sipeed .cst) ---
    output [13:0] ddr_addr, 
    output [2:0]  ddr_bank,
    output        ddr_cs,
    output        ddr_ras,
    output        ddr_cas,
    output        ddr_we,
    output        ddr_ck,
    output        ddr_ck_n,
    output        ddr_cke,
    output        ddr_odt,
    output        ddr_reset_n,
    output [1:0]  ddr_dm,
    inout  [15:0] ddr_dq,
    inout  [1:0]  ddr_dqs,
    inout  [1:0]  ddr_dqs_n
);

assign debug_cs = SPI_CS;
assign debug_clk = SPI_CLK;
assign debug_mosi = MOSI;
assign debug_miso = MISO; 

// ==============================================================================
// 1. DDR3 MEMORY INTERFACE
// ==============================================================================
wire ddr_memory_clk;       
wire ddr_pll_lock;         
wire ddr_user_clk;         
wire ddr_logic_rst;        
wire ddr_calib_complete;   

wire        ddr_cmd_ready; 
reg  [2:0]  ddr_cmd = 0;       
reg         ddr_cmd_en = 0;    
reg  [26:0] user_addr = 0; // Renamed to prevent collision with physical ddr_addr pin     

wire         ddr_wr_data_rdy; 
reg  [127:0] ddr_wr_data = 0;     
reg          ddr_wr_data_en = 0;  
reg          ddr_wr_data_end = 0; 

wire [127:0] ddr_rd_data;       
wire         ddr_rd_data_valid; 
wire         ddr_rd_data_end;   

Gowin_rPLL r_clock(
    .clkout(ddr_memory_clk), 
    .lock(ddr_pll_lock),
    .reset(~sys_rst_n),      
    .clkin(sys_clk)      
);

DDR3_Memory_Interface_Top main_ram(
    .clk(sys_clk),                       
    .memory_clk(ddr_memory_clk),         
    .pll_lock(ddr_pll_lock),             
    .rst_n(sys_rst_n),                   
    .clk_out(ddr_user_clk),              
    .ddr_rst(ddr_logic_rst),             
    .init_calib_complete(ddr_calib_complete), 
    
    .app_burst_number(6'b000000), 
    
    .cmd_ready(ddr_cmd_ready),
    .cmd(ddr_cmd), 
    .cmd_en(ddr_cmd_en), 
    .addr({1'b0, user_addr}), 
    
    .wr_data_rdy(ddr_wr_data_rdy), 
    .wr_data(ddr_wr_data), 
    .wr_data_en(ddr_wr_data_en), 
    .wr_data_end(ddr_wr_data_end), 
    .wr_data_mask(16'h0000), 
    
    .rd_data(ddr_rd_data), 
    .rd_data_valid(ddr_rd_data_valid), 
    .rd_data_end(ddr_rd_data_end), 
    
    // FIXED: Hardwired maintenance signals to 0 to prevent floating bugs
    .sr_req(1'b0), 
    .ref_req(1'b0), 
    .sr_ack(), 
    .ref_ack(), 
    .burst(1'b0), // FIXED: Must be 0 so the controller flushes the FIFO to RAM!
    
    // --- PHY PINS CONNECTED TO SIPEED TOP LEVEL NAMES ---
    .O_ddr_addr(ddr_addr), 
    .O_ddr_ba(ddr_bank), 
    .O_ddr_cs_n(ddr_cs), 
    .O_ddr_ras_n(ddr_ras), 
    .O_ddr_cas_n(ddr_cas), 
    .O_ddr_we_n(ddr_we), 
    .O_ddr_clk(ddr_ck), 
    .O_ddr_clk_n(ddr_ck_n), 
    .O_ddr_cke(ddr_cke), 
    .O_ddr_odt(ddr_odt), 
    .O_ddr_reset_n(ddr_reset_n), 
    .O_ddr_dqm(ddr_dm), 
    .IO_ddr_dq(ddr_dq), 
    .IO_ddr_dqs(ddr_dqs), 
    .IO_ddr_dqs_n(ddr_dqs_n) 
);

// ==============================================================================
// 2. UART CONTROLLER
// ==============================================================================
reg uart_wr_en = 0;
reg [7:0] uart_data = 0;
wire uart_write_done;

UART_Controller #(
    .BAUD_RATE(9600),
    .CLOCK_FREQ(99562500) 
) debug_uart (
    .sys_clk(ddr_user_clk), 
    .sys_rst_n(sys_rst_n),
    .write_enable(uart_wr_en),
    .data_to_send(uart_data),
    .RX(1'b1), 
    .TX(TX),
    .write_done(uart_write_done),
    .read_done(),
    .data_readed()
);

// ==============================================================================
// 3. SPI CONTROLLER
// ==============================================================================
reg spi_read_enable = 0;
reg [7:0] spi_cmd = 8'h03; 
reg [23:0] spi_address = 24'h20_00_00; 
reg [7:0] spi_total_bits = 160;        // FIXED: Now 8-bit to comfortably hold 160
wire [127:0] spi_128bit_data;          
wire spi_done;

SPI_Controller spi_inst (
    .sys_clk(ddr_user_clk),        
    .sys_rst_n(sys_rst_n),
    .MISO(MISO),
    .read_enable(spi_read_enable), 
    .command(spi_cmd),             
    .address(spi_address),  
    .total_bits(spi_total_bits),   
    .data_readed(spi_128bit_data), 
    .MOSI(MOSI),
    .CS(SPI_CS),
    .CLK(SPI_CLK),
    .done(spi_done)                
);

// ==============================================================================
// 4. DMA BOOTLOADER STATE MACHINE
// ==============================================================================
localparam S_BOOT_DELAY      = 4'd0;
localparam S_WAIT_CALIB      = 4'd1;
localparam S_UART_SEND       = 4'd2;
localparam S_UART_WAIT_DONE  = 4'd3;
localparam S_SPI_START       = 4'd4;
localparam S_SPI_WAIT        = 4'd5;
localparam S_DDR_WAIT        = 4'd6;
localparam S_DDR_WRITE       = 4'd7;
localparam S_CS_HOLD         = 4'd8; // FIXED: New state for Chip Select hold time
localparam S_CHECK_PROGRESS  = 4'd9;
localparam S_HALT            = 4'd10;

reg [3:0]  state = S_BOOT_DELAY;
reg [3:0]  return_state = S_BOOT_DELAY; 
reg [24:0] timer = 0;          
reg [1:0]  byte_idx = 0;        
reg [31:0] uart_msg = 0;   
reg [17:0] progress_counter = 0; 


// ==============================================================================
// 5. RAM READ STATE MACHINE
// ==============================================================================

localparam R_UART_SEND       = 4'd0;
localparam R_UART_WAIT_DONE  = 4'd1;
localparam R_DDR_WAIT        = 4'd2;
localparam R_DDR_READ        = 4'd3;
localparam R_DDR_WAIT_DATA   = 4'd4;
localparam R_SEND_WORDS      = 4'd5;
localparam R_CHECK_WORDS     = 4'd6;
localparam R_CHECK_PROGRESS  = 4'd7;
localparam R_HALT            = 4'd8;


// ==============================================================================
// 6. COMPUTER SIGNALS
// ==============================================================================
reg power_up = 1;
reg computer_hlt = 1;
reg [31:0]program_counter = 0; //Max ram address is 27 bit + 5 bit for I/O
reg [127:0]memory_read_reg = 0;
reg [2:0] word_idx = 0; // Tracks which of the four 32-bit words we are sending


always @(posedge ddr_user_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin        
        state <= S_BOOT_DELAY;
        spi_read_enable <= 0;
        uart_wr_en <= 0;
        
        spi_address <= 24'h20_00_00; 
        user_addr <= 27'd0;           
        
        ddr_cmd_en <= 0;
        ddr_wr_data_en <= 0;
        ddr_wr_data_end <= 0;
        
        timer <= 0;
        byte_idx <= 0;
        progress_counter <= 0;

        computer_hlt <= 1;
        power_up <= 1;
        program_counter <= 32'd0;
        memory_read_reg <= 0;
        
    end else begin
        if (power_up == 1 && computer_hlt == 1) begin //Bootloader part
            case (state)
            S_BOOT_DELAY: begin
                if (timer >= 1_000_000) begin 
                    timer <= 0;
                    uart_msg <= 32'hDE_AD_BE_EF; 
                    return_state <= S_WAIT_CALIB;
                    state <= S_UART_SEND; 
                end else begin
                    timer <= timer + 1;
                end
            end
            
            S_WAIT_CALIB: begin
                if (ddr_calib_complete) begin
                    uart_msg <= 32'h44_44_52_33; 
                    return_state <= S_SPI_START; 
                    state <= S_UART_SEND;
                end
            end

            S_UART_SEND: begin
                case (byte_idx)
                    2'd0: uart_data <= uart_msg[31:24];
                    2'd1: uart_data <= uart_msg[23:16];
                    2'd2: uart_data <= uart_msg[15:8];
                    2'd3: uart_data <= uart_msg[7:0];
                endcase
                uart_wr_en <= 1;
                state <= S_UART_WAIT_DONE;
            end

            S_UART_WAIT_DONE: begin
                if (uart_write_done) begin
                    uart_wr_en <= 0;
                    if (byte_idx == 3) begin
                        byte_idx <= 0;
                        state <= return_state; 
                    end else begin
                        byte_idx <= byte_idx + 1;
                        state <= S_UART_SEND;
                    end
                end
            end

            S_SPI_START: begin
                spi_cmd <= 8'h03; 
                spi_total_bits <= 160; 
                spi_read_enable <= 1;
                state <= S_SPI_WAIT;
            end

            S_SPI_WAIT: begin
                if (spi_done) begin
                    spi_read_enable <= 0;
                    state <= S_DDR_WAIT;
                end
            end

            S_DDR_WAIT: begin
                if (ddr_cmd_ready && ddr_wr_data_rdy) begin
                    ddr_cmd <= 3'b000; 
                    ddr_cmd_en <= 1;
                    
                    ddr_wr_data <= spi_128bit_data; 
                    ddr_wr_data_en <= 1;
                    ddr_wr_data_end <= 1;
                    
                    state <= S_DDR_WRITE;
                end
            end

            S_DDR_WRITE: begin
                ddr_cmd_en <= 0;
                ddr_wr_data_en <= 0;
                ddr_wr_data_end <= 0;
                
                spi_address <= spi_address + 24'd16; 
                user_addr <= user_addr + 27'd8;        
                
                progress_counter <= progress_counter + 18'd16;
                
                timer <= 0;
                state <= S_CS_HOLD; // Jump to the new wait state!
            end
            
            // FIXED: 500 nanosecond wait to let the SPI chip breathe
            S_CS_HOLD: begin
                if (timer >= 50) begin
                    state <= S_CHECK_PROGRESS;
                end else begin
                    timer <= timer + 1;
                end
            end
            
            S_CHECK_PROGRESS: begin
                if (spi_address >= 24'h80_00_00) begin
                    uart_msg <= 32'h44_4F_4E_45; 
                    return_state <= S_HALT;
                    state <= S_UART_SEND;
                end 
                else if (progress_counter >= 102400) begin
                    progress_counter <= 0;
                    uart_msg <= 32'h2B_2B_2B_2B; 
                    return_state <= S_SPI_START; 
                    state <= S_UART_SEND;
                end 
                else begin
                    state <= S_SPI_START; 
                end
            end
            
            S_HALT: begin
                computer_hlt <= 0; //Start CPU
                power_up <= 0;
                state <= R_DDR_WAIT; 
                return_state <= R_DDR_WAIT;
                user_addr <= 27'd0; //Reset the RAM address
                program_counter <= 32'd0;                
            end
            
            default: state <= S_BOOT_DELAY;

            endcase
            end

        else if (computer_hlt == 0) begin //CPU Running
            case (state)
            R_UART_SEND: begin
                case (byte_idx)
                    2'd0: uart_data <= uart_msg[31:24];
                    2'd1: uart_data <= uart_msg[23:16];
                    2'd2: uart_data <= uart_msg[15:8];
                    2'd3: uart_data <= uart_msg[7:0];
                endcase
                uart_wr_en <= 1;
                state <= R_UART_WAIT_DONE;
            end

            R_UART_WAIT_DONE: begin
                if (uart_write_done) begin
                    uart_wr_en <= 0;
                    if (byte_idx == 3) begin
                        byte_idx <= 0;
                        state <= return_state; 
                    end else begin
                        byte_idx <= byte_idx + 1;
                        state <= R_UART_SEND;
                    end
                end
            end

            R_DDR_WAIT: begin
                if (ddr_cmd_ready) begin
                    ddr_cmd <= 3'b001; // 001 = READ Command
                    ddr_cmd_en <= 1;
                    state <= R_DDR_READ;
                end
            end

            R_DDR_READ: begin
                ddr_cmd_en <= 0; // Immediately pull LOW!
                state <= R_DDR_WAIT_DATA;
            end

            R_DDR_WAIT_DATA: begin 
                if (ddr_rd_data_valid == 1) begin
                    memory_read_reg <= ddr_rd_data; // Save all 16 bytes
                    word_idx <= 0;                  // Reset word counter
                    state <= R_SEND_WORDS;          
                end
            end
                        
            //Chops the 128-bit register into four 32-bit UART messages
            R_SEND_WORDS: begin
                if (word_idx == 0) uart_msg <= memory_read_reg[127:96]; // Word 0
                else if (word_idx == 1) uart_msg <= memory_read_reg[95:64];  // Word 1
                else if (word_idx == 2) uart_msg <= memory_read_reg[63:32];  // Word 2
                else if (word_idx == 3) uart_msg <= memory_read_reg[31:0];   // Word 3

                byte_idx <= 0;
                return_state <= R_CHECK_WORDS; 
                state <= R_UART_SEND;
            end

            R_CHECK_WORDS: begin
                if (word_idx == 3) begin
                    user_addr <= user_addr + 27'd8; // MUST BE 8!
                    state <= R_CHECK_PROGRESS;
                end else begin
                    word_idx <= word_idx + 1;
                    state <= R_SEND_WORDS;
                end
            end
            
            R_CHECK_PROGRESS: begin
                // Stop after reading 32 bytes total
                if (user_addr >= 27'd16) begin
                    uart_msg <= 32'hFF_00_FF_00; 
                    return_state <= R_HALT;
                    state <= R_UART_SEND;
                end else begin
                    state <= R_DDR_WAIT;
                end
            end
            
            R_HALT: begin
                computer_hlt <= 1; 
                state <= R_HALT; 
            end
            
            default: state <= R_DDR_WAIT;
            
            endcase
        end

        else begin //Program END 
            
        end
    end
end

endmodule