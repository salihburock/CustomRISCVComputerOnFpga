module UART_Controller
#(
    parameter BAUD_RATE = 9600,
    parameter CLOCK_FREQ = 99562500
)(
    input sys_clk,
    input sys_rst_n,
    input write_enable,
    input [7:0] data_to_send,
    input RX,
    output reg TX = 1,
    output reg write_done = 0,
    output reg read_done = 0,
    output reg [7:0] data_readed = 8'b11111111
);

localparam counter_up_limit = CLOCK_FREQ / BAUD_RATE;

reg [31:0] clock_counter = 0;
reg [3:0] current_bit = 0; // supports 0-9 (start + 8 data + stop)
reg write_status = 0;
reg write_status_prev = 0;

reg current_RX_bit = 1;
reg prev_RX_bit = 1;
reg [31:0] clock_counter_RX = 0;
reg [3:0] current_bit_RX = 0; // supports 0-9 (start + 8 data + stop)
reg read_status = 0;
reg read_status_prev = 0;



always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        TX <= 1;
        clock_counter <= 0;
        current_bit <= 0;
        write_status <= 0;
        write_status_prev <= 0;
        write_done <= 0;
        read_done <= 0;
        data_readed <= 8'b11111111;
        current_RX_bit <= 1;
        prev_RX_bit <= 1;
        clock_counter_RX <= 0;
        current_bit_RX <= 0; // supports 0-9 (start + 8 data + stop)
        read_status <= 0;
        read_status_prev <= 0;

//----------For TX-------------------------------------------------------------
    end else begin
        if (write_enable && !write_status && !write_status_prev) begin
            write_status <= 1;
        end else if (!write_enable) begin
            write_done <= 0;
            write_status <= 0;
            write_status_prev <= 0;
        end

        if (clock_counter >= counter_up_limit) begin
            clock_counter <= 0;

            if (write_status && !write_status_prev) begin
                TX <= 0; // Start bit
                write_status_prev <= 1;
            end
            else if (write_status) begin
                if (current_bit < 8) begin
                    TX <= data_to_send[current_bit];
                    current_bit <= current_bit + 1;
                end
                else if (current_bit == 8) begin
                    TX <= 1; // Stop bit
                    current_bit <= current_bit + 1;
                end
                else begin
                    current_bit <= 0;
                    write_status <= 0;
                    write_done <= 1;
                end
            end

        end else begin
            clock_counter <= clock_counter + 1;
        end

//--------------For RX--------------------------------------------------------------------------------------------

        current_RX_bit <= RX;
        prev_RX_bit <= current_RX_bit;

        if (current_RX_bit == 0 && prev_RX_bit == 1) begin
            if (!read_status && !read_status_prev) begin
                read_status <= 1;
                read_done <= 0;
                data_readed <= 8'b1111_1111; // Just in case
            end
        end
        

        if (read_status) begin
            
            if ((clock_counter_RX >= counter_up_limit) && read_status_prev) begin
                clock_counter_RX <= 0;
                if (current_bit_RX < 8) begin
                    data_readed[current_bit_RX] <= RX;             
                    current_bit_RX <= current_bit_RX + 1;
                end
               

                else if(current_bit_RX == 8) begin
                    if (RX == 1) begin // stop bit should be high
                        read_status <= 0;
                        read_status_prev <= 0;
                        read_done <= 1;
                        current_bit_RX <= 0;
                    end else begin
                        read_status <= 0;
                        read_status_prev <= 0;
                        read_done <= 0; //Read is invalid restart from beginning
                        current_bit_RX <= 0;
                        data_readed <= 8'b1111_1111;
                    end
                end
                    
            end else begin
                clock_counter_RX <= clock_counter_RX + 1;
            end

            if ((clock_counter_RX >= (counter_up_limit / 2)) && !read_status_prev) begin
                read_status_prev <= 1;
                clock_counter_RX <= 0;
                current_bit_RX <= 0;
            end
        end
    end
end
endmodule
