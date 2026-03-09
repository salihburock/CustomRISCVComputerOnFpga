module CPU_Registers( //LUT Memory
    input clk,
    
    // Read Port 1
    input [4:0] read1_addr,
    output [31:0] read1_data,
    
    // Read Port 2
    input [4:0] read2_addr,
    output [31:0] read2_data,
    
    // Write Port
    input [4:0] write_addr,
    input [31:0] write_data,
    input write_enable
);

    // Create the 32 registers, each 32 bits wide! (Synthesizer will turn this into LUTs)
    reg [31:0] registers [0:31]; 
    
    // Initialize everything to 0 on startup
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 32'd0;
        end
    end

    // --- ASYNCHRONOUS READS (Instant data output) ---
    // If address is 0, always output 0. Otherwise, output the register data.
    assign read1_data = (read1_addr == 5'd0) ? 32'd0 : registers[read1_addr];
    assign read2_data = (read2_addr == 5'd0) ? 32'd0 : registers[read2_addr];

    // --- SYNCHRONOUS WRITE (Saves data on the clock edge) ---
    always @(posedge clk) begin
        if (write_enable && write_addr != 5'd0) begin
            registers[write_addr] <= write_data;
        end
    end


endmodule
