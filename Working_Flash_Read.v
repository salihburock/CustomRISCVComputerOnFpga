module Main_Computer(
    input sys_clk, 
    input sys_rst_n,  
    input manual_clk, 
    input clk_select, 
    input MISO,
    output TX,
    output clk_led,
    output wire MOSI,
    output wire SPI_CS,
    output wire SPI_CLK,
    output wire debug_cs,
    output wire debug_clk,
    output wire debug_mosi,
    output wire debug_miso
);

wire c_clk; 
wire c_clk_inverted = !c_clk; 
wire HLT; 

assign debug_cs = SPI_CS;
assign debug_clk = SPI_CLK;
assign debug_mosi = MOSI;
assign debug_miso = MISO; 

Clock main_clock(
    .sys_clk(sys_clk),
    .manual_clk(manual_clk),
    .clk_select(clk_select),
    .HLT(HLT),
    .sys_rst_n(sys_rst_n),
    .c_clk(c_clk)
);

assign clk_led = c_clk_inverted;

// --- UART Signals ---
reg uart_wr_en = 0;
reg[7:0] uart_data = 0;
wire uart_write_done;

UART_Controller #(
    .BAUD_RATE(9600),
    .CLOCK_FREQ(27000000)
) debug_uart (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .write_enable(uart_wr_en),
    .data_to_send(uart_data),
    .RX(1'b1), 
    .TX(TX),
    .write_done(uart_write_done),
    .read_done(),
    .data_readed()
);

// --- SPI Signals ---
reg spi_read_enable = 0;
reg [7:0] spi_cmd = 8'h03; 
reg [23:0] spi_address = 24'h00_00_00;
reg [6:0] spi_total_bits = 64; 
wire[31:0] spi_32bit_data;
wire spi_done;

SPI_Controller spi_inst (
    .sys_clk(sys_clk),             
    .sys_rst_n(sys_rst_n),
    .MISO(MISO),
    .read_enable(spi_read_enable), 
    .command(spi_cmd),             
    .address(spi_address),  
    .total_bits(spi_total_bits),   
    .data_readed(spi_32bit_data),  
    .MOSI(MOSI),
    .CS(SPI_CS),
    .CLK(SPI_CLK),
    .done(spi_done)                
);

// --- State Machine Parameters ---
localparam S_IDLE           = 4'd0;
localparam S_SPI_START      = 4'd1;
localparam S_SPI_WAIT       = 4'd2;
localparam S_UART_SEND      = 4'd3;
localparam S_UART_WAIT_DONE = 4'd4;
localparam S_UART_CLEANUP   = 4'd5;
localparam S_HALT           = 4'd6; // New state to stop at 8MB

// --- FSM Registers ---
reg[3:0] state = S_IDLE;
reg [24:0] timer = 0;          
reg [1:0] byte_idx = 0;        
reg [31:0] current_data = 0;   
reg [3:0] init_step = 0; 

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin        
        state <= S_IDLE;
        spi_read_enable <= 0;
        uart_wr_en <= 0;
        spi_address <= 24'h20_00_00;
        spi_cmd <= 8'h03;
        spi_total_bits <= 64;
        timer <= 0;
        byte_idx <= 0;
        current_data <= 0;
        init_step <= 0;
    end else begin
        case (state)
            
            S_IDLE: begin
                case (init_step)
                    0: begin // 0. BOOT DELAY
                        if (timer == 270_000) begin 
                            timer <= 0;
                            init_step <= 1; 
                        end else begin
                            timer <= timer + 1;
                        end
                    end
                    1: begin // 1. UART Test
                        current_data <= 32'hDE_AD_BE_EF; 
                        byte_idx <= 0;
                        state <= S_UART_SEND; 
                    end
                    2: begin // 2. Continuous 0x03 Read Loop
                        spi_cmd <= 8'h03; 
                        // spi_address is updated dynamically in S_UART_CLEANUP
                        spi_total_bits <= 64; // 8 cmd + 24 addr + 32 data
                        state <= S_SPI_START;
                    end
                endcase
            end

            S_SPI_START: begin
                spi_read_enable <= 1;
                state <= S_SPI_WAIT;
            end

            S_SPI_WAIT: begin
                if (spi_done) begin
                    current_data <= spi_32bit_data; 
                    spi_read_enable <= 0;
                    byte_idx <= 0;
                    state <= S_UART_SEND; // Instantly send, no 1-second delay
                end
            end

            S_UART_SEND: begin
                case (byte_idx)
                    2'd0: uart_data <= current_data[31:24];
                    2'd1: uart_data <= current_data[23:16];
                    2'd2: uart_data <= current_data[15:8];
                    2'd3: uart_data <= current_data[7:0];
                endcase
                uart_wr_en <= 1;
                state <= S_UART_WAIT_DONE;
            end

            S_UART_WAIT_DONE: begin
                if (uart_write_done) begin
                    uart_wr_en <= 0;
                    state <= S_UART_CLEANUP;
                end
            end

            S_UART_CLEANUP: begin
                if (byte_idx == 3) begin
                    if (init_step == 1) begin
                        init_step <= 2; // UART test done, begin the grand read!
                        spi_address <= 24'h20_00_00;
                        state <= S_IDLE;  
                    end else begin
                        // Increment address by 4 bytes (32 bits)
                        spi_address <= spi_address + 24'd4;
                        
                        // Check if we hit the 8MB ceiling (0x800000 - 4 bytes = 0x7FFFFC)
                        if (spi_address >= 24'h7F_FF_FC) begin
                            state <= S_HALT; 
                        end else begin
                            state <= S_IDLE; // Loop back and read the next 4 bytes
                        end
                    end
                end else begin
                    byte_idx <= byte_idx + 1;
                    state <= S_UART_SEND; // Send next byte of the current 4-byte block
                end
            end
            
            S_HALT: begin
                // 8MB has been completely dumped. Safe to power down or reset.
                state <= S_HALT;
            end
            
            default: state <= S_IDLE;
        endcase
    end
end
endmodule