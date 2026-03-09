module SPI_Controller #(
    parameter CLK_DIV = 7, // 0 = Full Speed, 31 = Logic Analyzer Speed
    parameter DATA_LENGTH = 128
    )(
    input wire sys_clk,        
    input wire sys_rst_n,      
    input wire MISO,           
    
    input wire read_enable,   
    input wire [7:0] command,  
    input wire [23:0] address, 
    input wire [7:0] total_bits,   
    
    output reg [DATA_LENGTH-1:0] data_readed, 
    output reg MOSI,           
    output reg CS,             
    output wire CLK,           
    output reg done            
);

    reg spi_clk;
    reg com_status;
    reg [7:0] current_bit; 
    reg [4:0] speed_divider = 0; 
    
    assign CLK = spi_clk;
    
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            spi_clk <= 0; 
            CS <= 1;
            MOSI <= 1;
            done <= 0;
            com_status <= 0;
            current_bit <= 0;
            data_readed <= 32'b0;
            speed_divider <= 0;

        end else begin
            
            // STATE 1: Start the transaction
            if (read_enable && !com_status && !done) begin
                com_status <= 1;
                CS <= 0;            
                current_bit <= 0;
                MOSI <= command[7]; 
                spi_clk <= 0;
                data_readed <= 32'b0;
                speed_divider <= 0;
            end 
            
            // STATE 2: The Active Transaction
            else if (com_status) begin

                if (speed_divider >= CLK_DIV) begin
                    speed_divider <= 0;

                    if (current_bit == (total_bits-1) && spi_clk == 1) begin
                        com_status <= 0;
                        CS <= 1;
                        MOSI <= 1;
                        done <= 1;
                        spi_clk <= 0;
                    end else begin
                        spi_clk <= ~spi_clk; 
                        
                        if (spi_clk == 0) begin 
                            // --- RISING EDGE: Sample MISO ---
                            data_readed <= {data_readed[DATA_LENGTH-2:0], MISO};  //FIFO shifthing read
                        end else begin
                            // --- FALLING EDGE: Update MOSI ---
                            if (current_bit < 7) begin
                                MOSI <= command[6 - current_bit];
                            end else if (current_bit < 31) begin
                                MOSI <= address[23 - (current_bit - 7)];
                            end else begin
                                MOSI <= 0; 
                            end
                            current_bit <= current_bit + 1;
                        end
                    end
                end

                else begin
                    speed_divider <= speed_divider + 1;
                end
            end 
            
            // STATE 3: Clear the done flag
            else if (!read_enable && done) begin
                done <= 0;
            end
            
            // STATE 4: Idle State (Only happens when com_status is 0 and not starting)
            else if (!com_status && !done) begin
                CS <= 1;
                MOSI <= 1;
                spi_clk <= 0; 
                speed_divider <= 0;
            end
            
        end
    end
endmodule