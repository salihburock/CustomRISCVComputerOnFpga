module Main_Computer(
    input sys_clk,   
    input sys_rst_n, 
    input MISO,
    input PS2_DATA_I,
    input PS2_CLK_I,
    output PS2_DATA_DEBUG,
    output PS2_CLK_DEBUG,
    output TX,
    output wire MOSI,
    output wire SPI_CS,
    output wire SPI_CLK,
    output wire debug_cs,
    output wire debug_clk,
    output wire debug_mosi,
    output wire debug_miso,
    
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
    inout  [1:0]  ddr_dqs_n,
    output wire VGA_HSYNC,
    output wire VGA_VSYNC,
    output wire[2:0] VGA_R,
    output wire[2:0] VGA_G,
    output wire[1:0] VGA_B
);

assign debug_cs = SPI_CS;
assign debug_clk = SPI_CLK;
assign debug_mosi = MOSI;
assign debug_miso = MISO; 

assign PS2_DATA_DEBUG = ~PS2_DATA_I;
assign PS2_CLK_DEBUG = ~PS2_CLK_I;

// ==============================================================================
// 1. HARDWARE ARBITER & MULTIPLEXER WIRES
// ==============================================================================
wire [2:0]   ddr_cmd;
wire         ddr_cmd_en;
wire [26:0]  user_addr;
wire [127:0] ddr_wr_data;
wire         ddr_wr_data_en;
wire         ddr_wr_data_end;
wire [15:0]  ddr_wr_data_mask; 
wire [7:0]   uart_data;
wire         uart_wr_en;

reg boot_done = 0; 
reg cpu_done  = 0; 

wire cpu_active = (boot_done == 1 && cpu_done == 0);

assign ddr_cmd          = cpu_active ? cpu_ddr_cmd          : boot_ddr_cmd;
assign ddr_cmd_en       = cpu_active ? cpu_ddr_cmd_en       : boot_ddr_cmd_en;
assign user_addr        = cpu_active ? cpu_user_addr        : boot_user_addr;
assign ddr_wr_data      = cpu_active ? cpu_ddr_wr_data      : boot_ddr_wr_data;
assign ddr_wr_data_en   = cpu_active ? cpu_ddr_wr_data_en   : boot_ddr_wr_data_en;
assign ddr_wr_data_end  = cpu_active ? cpu_ddr_wr_data_end  : boot_ddr_wr_data_end;
assign ddr_wr_data_mask = cpu_active ? cpu_ddr_wr_data_mask : boot_ddr_wr_data_mask; 
assign uart_data        = cpu_active ? cpu_uart_data        : boot_uart_data;
assign uart_wr_en       = cpu_active ? cpu_uart_wr_en       : boot_uart_wr_en;

// CPU READS THIS DIRECTLY!
wire [31:0]  ps2_data;  

// ==============================================================================
// 2. DDR3 MEMORY INTERFACE
// ==============================================================================
wire ddr_memory_clk;       
wire ddr_pll_lock;         
wire ddr_user_clk;         
wire ddr_logic_rst;        
wire ddr_calib_complete;   
wire ddr_cmd_ready; 
wire ddr_wr_data_rdy; 
wire [127:0] ddr_rd_data;       
wire ddr_rd_data_valid; 
wire ddr_rd_data_end;   

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
    .wr_data_mask(ddr_wr_data_mask), 
    .rd_data(ddr_rd_data), 
    .rd_data_valid(ddr_rd_data_valid), 
    .rd_data_end(ddr_rd_data_end), 
    .sr_req(1'b0), 
    .ref_req(1'b0), 
    .sr_ack(), 
    .ref_ack(), 
    .burst(1'b0), 
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
// 3. UART CONTROLLER
// ==============================================================================
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
// 4. SPI CONTROLLER
// ==============================================================================
reg spi_read_enable = 0;
reg [7:0] spi_cmd = 8'h03; 
reg [23:0] spi_address = 24'h10_00_00; 
reg [7:0] spi_total_bits = 160;        
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
// 5. DMA BOOTLOADER STATE MACHINE
// ==============================================================================
localparam B_BOOT_DELAY      = 4'd0;
localparam B_WAIT_CALIB      = 4'd1;
localparam B_UART_SEND       = 4'd2;
localparam B_UART_WAIT_DONE  = 4'd3;
localparam B_SPI_START       = 4'd4;
localparam B_SPI_WAIT        = 4'd5;
localparam B_DDR_WAIT        = 4'd6;
localparam B_DDR_WRITE       = 4'd7;
localparam B_CS_HOLD         = 4'd8; 
localparam B_CHECK_PROGRESS  = 4'd9;
localparam B_HALT            = 4'd10;

reg [3:0]  b_state = B_BOOT_DELAY;
reg [3:0]  b_return_state = B_BOOT_DELAY; 
reg [24:0] timer = 0;          
reg [1:0]  boot_byte_idx = 0;        
reg [31:0] boot_uart_msg = 0;   
reg [17:0] progress_counter = 0; 

reg [2:0]   boot_ddr_cmd = 0;
reg         boot_ddr_cmd_en = 0;
reg [26:0]  boot_user_addr = 0;
reg [127:0] boot_ddr_wr_data = 0;
reg         boot_ddr_wr_data_en = 0;
reg         boot_ddr_wr_data_end = 0;
reg [7:0]   boot_uart_data = 0;
reg         boot_uart_wr_en = 0;

wire [15:0] boot_ddr_wr_data_mask = 16'h0000; 

always @(posedge ddr_user_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin        
        b_state <= B_BOOT_DELAY;
        spi_read_enable <= 0;
        boot_uart_wr_en <= 0;
        spi_address <= 24'h10_00_00; 
        boot_user_addr <= 27'd0;           
        boot_ddr_cmd_en <= 0;
        boot_ddr_wr_data_en <= 0;
        boot_ddr_wr_data_end <= 0;
        timer <= 0;
        boot_byte_idx <= 0;
        progress_counter <= 0;
        boot_done <= 0; 
    end else if (!boot_done) begin 
        case (b_state)
            B_BOOT_DELAY: begin
                if (timer >= 1_000_000) begin 
                    timer <= 0;
                    boot_uart_msg <= 32'hDE_AD_BE_EF; 
                    b_return_state <= B_WAIT_CALIB;
                    b_state <= B_UART_SEND; 
                end else begin
                    timer <= timer + 1;
                end
            end
            
            B_WAIT_CALIB: begin
                if (ddr_calib_complete) begin
                    boot_uart_msg <= 32'h44_44_52_33; 
                    b_return_state <= B_SPI_START; 
                    b_state <= B_UART_SEND;
                end
            end

            B_UART_SEND: begin
                case (boot_byte_idx)
                    2'd0: boot_uart_data <= boot_uart_msg[31:24];
                    2'd1: boot_uart_data <= boot_uart_msg[23:16];
                    2'd2: boot_uart_data <= boot_uart_msg[15:8];
                    2'd3: boot_uart_data <= boot_uart_msg[7:0];
                endcase
                boot_uart_wr_en <= 1;
                b_state <= B_UART_WAIT_DONE;
            end

            B_UART_WAIT_DONE: begin
                if (uart_write_done) begin
                    boot_uart_wr_en <= 0;
                    if (boot_byte_idx == 3) begin
                        boot_byte_idx <= 0;
                        b_state <= b_return_state; 
                    end else begin
                        boot_byte_idx <= boot_byte_idx + 1;
                        b_state <= B_UART_SEND;
                    end
                end
            end

            B_SPI_START: begin
                spi_cmd <= 8'h03; 
                spi_total_bits <= 160; 
                spi_read_enable <= 1;
                b_state <= B_SPI_WAIT;
            end

            B_SPI_WAIT: begin
                if (spi_done) begin
                    spi_read_enable <= 0;
                    b_state <= B_DDR_WAIT;
                end
            end

            B_DDR_WAIT: begin
                if (ddr_cmd_ready && ddr_wr_data_rdy) begin
                    boot_ddr_cmd <= 3'b000; 
                    boot_ddr_cmd_en <= 1;
                    boot_ddr_wr_data <= spi_128bit_data; 
                    boot_ddr_wr_data_en <= 1;
                    boot_ddr_wr_data_end <= 1;
                    b_state <= B_DDR_WRITE;
                end
            end

            B_DDR_WRITE: begin
                boot_ddr_cmd_en <= 0;
                boot_ddr_wr_data_en <= 0;
                boot_ddr_wr_data_end <= 0;
                spi_address <= spi_address + 24'd16; 
                boot_user_addr <= boot_user_addr + 27'd8;        
                progress_counter <= progress_counter + 18'd16;
                timer <= 0;
                b_state <= B_CS_HOLD;
            end
            
            B_CS_HOLD: begin
                if (timer >= 50) begin
                    b_state <= B_CHECK_PROGRESS;
                end else begin
                    timer <= timer + 1;
                end
            end
            
            B_CHECK_PROGRESS: begin
                if (spi_address >= 24'h80_00_00) begin
                    boot_uart_msg <= 32'h44_4F_4E_45; 
                    b_return_state <= B_HALT;
                    b_state <= B_UART_SEND;
                end 
                else if (progress_counter >= 102400) begin
                    progress_counter <= 0;
                    boot_uart_msg <= 32'h2B_2B_2B_2B; 
                    b_return_state <= B_SPI_START; 
                    b_state <= B_UART_SEND;
                end 
                else begin
                    b_state <= B_SPI_START; 
                end
            end
            
            B_HALT: begin
                boot_done <= 1; 
                b_state <= B_HALT; 
            end
            
            default: b_state <= B_BOOT_DELAY;
        endcase
    end 
    // --- CTRL+ESC SOFT REBOOT TRIGGER FOR BOOTLOADER ---
    else if (cpu_done && ps2_data[4] && ps2_data[7]) begin
        b_state <= B_BOOT_DELAY;
        spi_read_enable <= 0;
        boot_uart_wr_en <= 0;
        spi_address <= 24'h10_00_00; 
        boot_user_addr <= 27'd0;           
        boot_ddr_cmd_en <= 0;
        boot_ddr_wr_data_en <= 0;
        boot_ddr_wr_data_end <= 0;
        timer <= 0;
        boot_byte_idx <= 0;
        progress_counter <= 0;
        boot_done <= 0; 
    end
end

// ==============================================================================
// Keyboard Controller (PS/2)
// ==============================================================================
PS2_Controller keyboard (
    .sys_clk(ddr_user_clk), 
    .sys_rst_n(sys_rst_n),
    .PS2_D_I(PS2_DATA_I),
    .PS2_CLK_I(PS2_CLK_I),
    .DOOM_KEYS(ps2_data)  
);

// ==============================================================================
// VGA Controller
// ==============================================================================

// 1. The 40MHz Clock and Lock Signals
wire vga_clk;
wire vga_lock;
wire vga_rst_n = sys_rst_n & vga_lock; 

Gowin_rPLL_VGA vga_clock_generator (
    .clkout(vga_clk),
    .lock(vga_lock),
    .clkin(sys_clk)
);

wire [13:0] vga_read_addr;
wire        vga_read_en;
wire [31:0] active_display_data; // Pre-declared for instantiation

VGA_Controller display_out (
    .sys_clk(vga_clk),             
    .sys_rst_n(vga_rst_n),         
    .vram_read_addr(vga_read_addr), 
    .vram_read_en(vga_read_en),
    .vram_read_data(active_display_data), // Reads the multiplexed signal
    .h_sync_i(VGA_HSYNC),
    .v_sync_i(VGA_VSYNC),
    .RED(VGA_R),
    .GREEN(VGA_G),
    .BLUE(VGA_B)
);

// ==============================================================================
// VRAM (Main Display) & CRASH MULTIPLEXER (Algorithmic BSOD)
// ==============================================================================

reg [13:0]  vram_addr_out = 0;           
reg [31:0]  vram_data_out = 0;           
reg         vram_write_en = 0;           

wire [31:0] main_vram_read_data;

// --- 1. The Main VRAM (Active during normal gameplay) ---
Gowin_SDPB_VRAM vram(
    .clka(ddr_user_clk),       
    .cea(vram_write_en),       
    .reseta(~sys_rst_n),       
    .ada(vram_addr_out),       
    .din(vram_data_out),       

    .clkb(vga_clk),            
    .ceb(vga_read_en),         
    .resetb(~sys_rst_n),       
    .oce(1'b0),                 
    .adb(vga_read_addr),       
    .dout(main_vram_read_data) 
);

// --- 2. Zero-RAM Algorithmic Blue Screen MUX ---
// If CPU halts, override the VRAM data and output Pure Blue (0x03030303)
assign active_display_data = cpu_done ? 32'h03_03_03_03 : main_vram_read_data;


// ==============================================================================
// SYSTEM HARDWARE TIMER (1ms resolution)
// ==============================================================================
reg [31:0] ms_counter = 0; 
reg [16:0] tick_counter = 0;

always @(posedge ddr_user_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        ms_counter <= 32'd0;
        tick_counter <= 17'd0;
    end else if (boot_done && !cpu_done) begin 
        if (tick_counter >= 17'd99_999) begin 
            tick_counter <= 17'd0;
            ms_counter <= ms_counter + 1;
        end else begin
            tick_counter <= tick_counter + 1;
        end
    end 
    // --- CTRL+ESC SOFT REBOOT TRIGGER FOR TIMER ---
    else if (cpu_done && ps2_data[4] && ps2_data[7]) begin
        ms_counter <= 32'd0;
        tick_counter <= 17'd0;
    end
end

// ==============================================================================
// 6. CPU REGISTERS
// ==============================================================================
reg [4:0] read1_addr = 0;
wire [31:0] read1_data;
    
reg [4:0] read2_addr = 0;
wire [31:0] read2_data;
    
reg [4:0] write_addr = 0;
reg [31:0] write_data = 0;
reg write_enable = 0;

CPU_Registers cpu_register_controller(
    .clk(ddr_user_clk),
    .read1_addr(read1_addr),
    .read1_data(read1_data),
    .read2_addr(read2_addr),
    .read2_data(read2_data),
    .write_addr(write_addr),
    .write_data(write_data),
    .write_enable(write_enable)
);

// ==============================================================================
// 7. COMPUTER STATE MACHINE (Now 6-bit for Prefetch States)
// ==============================================================================
localparam C_UART_SEND         = 6'd0;
localparam C_UART_WAIT_DONE    = 6'd1;
localparam C_DDR_WAIT_READ     = 6'd2;
localparam C_DDR_READ          = 6'd3;
localparam C_DDR_WAIT_DATA     = 6'd4;
localparam C_LOAD_INSTR        = 6'd5;
localparam C_DECODE            = 6'd6; 

localparam C_EXEC_R_TYPE       = 6'd7;
localparam C_EXEC_I_TYPE       = 6'd8;
localparam C_EXEC_LOAD         = 6'd9;
localparam C_EXEC_STORE        = 6'd10;
localparam C_EXEC_BRANCH       = 6'd11;
localparam C_EXEC_JAL          = 6'd12;
localparam C_EXEC_JALR         = 6'd13;
localparam C_EXEC_LUI          = 6'd14;
localparam C_EXEC_AUIPC        = 6'd15;

localparam C_EXECUTE           = 6'd16; 
localparam C_FINISH_DATA_READ  = 6'd17; 
localparam C_DDR_WAIT_WRITE    = 6'd18;
localparam C_DDR_WRITE         = 6'd19;
localparam C_IO_READ           = 6'd20; 
localparam C_IO_WRITE          = 6'd21; 

localparam C_HALT              = 6'd22;
localparam C_HALT_SETUP        = 6'd23;
localparam C_HALT_FETCH        = 6'd24;
localparam C_HALT_NEXT         = 6'd25;
localparam C_HALT_FOREVER      = 6'd26;

localparam C_DEBUG_PRINT_PC    = 6'd27;
localparam C_DEBUG_PRINT_INSTR = 6'd28;

localparam C_CHECK_ILLEGAL_R   = 6'd29;
localparam C_REG_FETCH_WAIT    = 6'd30; 

// --- NEW PREFETCH STATES ---
localparam C_PREFETCH_ISSUE    = 6'd31;
localparam C_PREFETCH_CLEANUP  = 6'd32;

reg [5:0]  state = C_DDR_WAIT_READ;        
reg [5:0]  return_state = C_DDR_WAIT_READ; 
reg [5:0]  pending_exec_state = 0; 

// CPU Private Wires
reg [2:0]   cpu_ddr_cmd = 0;
reg         cpu_ddr_cmd_en = 0;
reg [26:0]  cpu_user_addr = 0;
reg [127:0] cpu_ddr_wr_data = 0;
reg         cpu_ddr_wr_data_en = 0;
reg         cpu_ddr_wr_data_end = 0;
reg [15:0]  cpu_ddr_wr_data_mask = 16'h0000;
reg [7:0]   cpu_uart_data = 0;
reg         cpu_uart_wr_en = 0;
reg [1:0]   cpu_byte_idx = 0;        
reg [31:0]  cpu_uart_msg = 0;

// Sub-Word Memory Alignment Wires
reg [31:0]  raw_word = 0;
reg [3:0]   base_mask = 0;
reg [31:0]  active_payload = 0;

reg [31:0]  program_counter = 0; 
reg [127:0] memory_read_reg = 0; // Now only used for Data Loads
reg [31:0]  first_instr = 0;
reg [31:0]  second_instr = 0;
reg [31:0]  third_instr = 0;
reg [31:0]  fourth_instr = 0;

// --- DDR3 TRANSACTION QUEUE & BACKGROUND CATCHER ---
reg [1:0]  rq_head = 0;
reg [1:0]  rq_tail = 0;
reg [3:0]  rq_is_data = 0; // 1 = Data Load, 0 = Instruction/Prefetch
reg [26:0] rq_addr_0=0, rq_addr_1=0, rq_addr_2=0, rq_addr_3=0;

reg [127:0] pf_read_reg = 0;
reg [26:0]  pf_ready_addr = 27'h7FFFFFF; // Initializes to invalid address
reg         pf_valid = 0;

reg [127:0] dmem_read_reg = 0;
reg         dmem_valid = 0;
// ----------------------------------------------------

// Internal CPU decoding wires
reg [31:0]  current_instr = 0;
reg [6:0]   opcode = 0;
reg [2:0]   funct3 = 0; 
reg [6:0]   funct7 = 0; 

wire [31:0] imm_i = {{20{current_instr[31]}}, current_instr[31:20]};
wire [31:0] imm_s = {{20{current_instr[31]}}, current_instr[31:25], current_instr[11:7]};
wire [31:0] imm_b = {{20{current_instr[31]}}, current_instr[7], current_instr[30:25], current_instr[11:8], 1'b0};
wire [31:0] imm_j = {{12{current_instr[31]}}, current_instr[19:12], current_instr[20], current_instr[30:21], 1'b0};
wire [31:0] imm_u = {current_instr[31:12], 12'b0};

reg         is_instruction_fetch = 1; 
reg [31:0]  data_addr = 0;            
reg [31:0]  cpu_store_data = 0; 

reg [4:0]   dump_reg_idx = 0; 

always @(posedge ddr_user_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin        
        state <= C_DDR_WAIT_READ;
        cpu_ddr_cmd_en <= 0;
        cpu_ddr_wr_data_en <= 0;
        cpu_ddr_wr_data_end <= 0;
        cpu_ddr_wr_data_mask <= 16'h0000;
        cpu_uart_wr_en <= 0;
        cpu_user_addr <= 27'd0;
        program_counter <= 32'd0;
        memory_read_reg <= 0;
        cpu_byte_idx <= 0;
        cpu_done <= 0;
        is_instruction_fetch <= 1;
        data_addr <= 32'd0;
        cpu_store_data <= 32'd0;
        vram_write_en <= 0;
        dump_reg_idx <= 5'd0;
        pending_exec_state <= 0;
        
        // Reset Queue
        rq_head <= 0; rq_tail <= 0; rq_is_data <= 0;
        pf_valid <= 0; dmem_valid <= 0; pf_ready_addr <= 27'h7FFFFFF;

    end else if (boot_done && !cpu_done) begin        
        
        // ========================================================
        // GLOBAL SNOOPING QUEUE (Background Data Catcher)
        // ========================================================
        if (ddr_rd_data_valid) begin
            if (rq_is_data[rq_tail]) begin
                // It was a Data Load Request
                dmem_read_reg <= ddr_rd_data;
                dmem_valid <= 1;
            end else begin
                // It was an Instruction Prefetch Request
                pf_read_reg <= ddr_rd_data;
                pf_valid <= 1;
                // Save the exact address this block belongs to
                if (rq_tail == 2'd0) pf_ready_addr <= rq_addr_0;
                else if (rq_tail == 2'd1) pf_ready_addr <= rq_addr_1;
                else if (rq_tail == 2'd2) pf_ready_addr <= rq_addr_2;
                else pf_ready_addr <= rq_addr_3;
            end
            rq_tail <= rq_tail + 1; // Advance the FIFO
        end
        // ========================================================

        case (state)
            C_UART_SEND: begin
                case (cpu_byte_idx)
                    2'd0: cpu_uart_data <= cpu_uart_msg[31:24];
                    2'd1: cpu_uart_data <= cpu_uart_msg[23:16];
                    2'd2: cpu_uart_data <= cpu_uart_msg[15:8];
                    2'd3: cpu_uart_data <= cpu_uart_msg[7:0];
                endcase
                cpu_uart_wr_en <= 1;
                state <= C_UART_WAIT_DONE;
            end

            C_UART_WAIT_DONE: begin
                if (uart_write_done) begin
                    cpu_uart_wr_en <= 0;
                    if (cpu_byte_idx == 3) begin
                        cpu_byte_idx <= 0;
                        state <= return_state; 
                    end else begin
                        cpu_byte_idx <= cpu_byte_idx + 1;
                        state <= C_UART_SEND;
                    end
                end
            end

            C_DDR_WAIT_READ: begin
                write_enable <= 0;
                
                if (is_instruction_fetch == 0 && data_addr >= 32'h40000000) begin
                    state <= C_IO_READ;
                end 
                else if (is_instruction_fetch == 1'b1) begin
                    // CACHE HIT: Is the data already waiting for us in the background?
                    if (pf_valid && pf_ready_addr == {program_counter[27:4], 3'b000}) begin
                        pf_valid <= 0; // Consume the cache!
                        first_instr  <= {pf_read_reg[103:96], pf_read_reg[111:104], pf_read_reg[119:112], pf_read_reg[127:120]};
                        second_instr <= {pf_read_reg[71:64],  pf_read_reg[79:72],   pf_read_reg[87:80],   pf_read_reg[95:88]};
                        third_instr  <= {pf_read_reg[39:32],  pf_read_reg[47:40],   pf_read_reg[55:48],   pf_read_reg[63:56]};
                        fourth_instr <= {pf_read_reg[7:0],    pf_read_reg[15:8],    pf_read_reg[23:16],   pf_read_reg[31:24]};
                        state <= C_LOAD_INSTR; // ZERO DDR3 LATENCY!
                    end 
                    // CACHE MISS: Ask the Memory Controller
                    else if (ddr_cmd_ready) begin
                        cpu_ddr_cmd <= 3'b001; 
                        cpu_ddr_cmd_en <= 1;
                        cpu_user_addr <= {program_counter[27:4], 3'b000}; 
                        // Queue as Instruction (0)
                        rq_is_data[rq_head] <= 0;
                        case (rq_head)
                            2'd0: rq_addr_0 <= {program_counter[27:4], 3'b000};
                            2'd1: rq_addr_1 <= {program_counter[27:4], 3'b000};
                            2'd2: rq_addr_2 <= {program_counter[27:4], 3'b000};
                            2'd3: rq_addr_3 <= {program_counter[27:4], 3'b000};
                        endcase
                        rq_head <= rq_head + 1;
                        state <= C_DDR_READ;
                    end
                end 
                else if (ddr_cmd_ready) begin 
                    // Normal Data Load
                    cpu_ddr_cmd <= 3'b001; 
                    cpu_ddr_cmd_en <= 1;
                    cpu_user_addr <= {data_addr[27:4], 3'b000}; 
                    rq_is_data[rq_head] <= 1; // Queue as Data (1)
                    rq_head <= rq_head + 1;
                    state <= C_DDR_READ;
                end
            end

            C_DDR_READ: begin
                cpu_ddr_cmd_en <= 0; 
                state <= C_DDR_WAIT_DATA;
            end

            C_DDR_WAIT_DATA: begin 
                // We wait for the Global Catcher to flag that data arrived
                if (is_instruction_fetch) begin
                    if (pf_valid) begin
                        // Check if it's the data we actually want (Protects against stray branch prefetches)
                        if (pf_ready_addr == {program_counter[27:4], 3'b000}) begin
                            pf_valid <= 0;
                            first_instr  <= {pf_read_reg[103:96], pf_read_reg[111:104], pf_read_reg[119:112], pf_read_reg[127:120]};
                            second_instr <= {pf_read_reg[71:64],  pf_read_reg[79:72],   pf_read_reg[87:80],   pf_read_reg[95:88]};
                            third_instr  <= {pf_read_reg[39:32],  pf_read_reg[47:40],   pf_read_reg[55:48],   pf_read_reg[63:56]};
                            fourth_instr <= {pf_read_reg[7:0],    pf_read_reg[15:8],    pf_read_reg[23:16],   pf_read_reg[31:24]};
                            state <= C_LOAD_INSTR;
                        end else begin
                            pf_valid <= 0; // Throw away stale branch prefetch
                        end
                    end
                end else begin
                    // It was a Data Load
                    if (dmem_valid) begin
                        dmem_valid <= 0;
                        memory_read_reg <= dmem_read_reg;
                        state <= C_FINISH_DATA_READ;
                    end
                end
            end
                        
            C_LOAD_INSTR: begin
                write_enable <= 0;
                case (program_counter[3:2])
                    2'b00: current_instr <= first_instr;
                    2'b01: current_instr <= second_instr;
                    2'b10: current_instr <= third_instr;
                    2'b11: current_instr <= fourth_instr;
                endcase
                state <= C_DECODE; 
            end

            C_DEBUG_PRINT_PC: begin
                cpu_uart_msg <= program_counter; 
                cpu_byte_idx <= 0;
                return_state <= C_DEBUG_PRINT_INSTR; 
                state <= C_UART_SEND;
            end

            C_DEBUG_PRINT_INSTR: begin
                cpu_uart_msg <= current_instr;   
                cpu_byte_idx <= 0;
                return_state <= C_DECODE;        
                state <= C_UART_SEND;
            end

            C_DECODE: begin
                opcode <= current_instr[6:0];
                funct3 <= current_instr[14:12];
                funct7 <= current_instr[31:25];
                
                read1_addr <= current_instr[19:15];
                read2_addr <= current_instr[24:20];
                write_addr <= current_instr[11:7];
                
                is_instruction_fetch <= 0; 
                return_state <= C_EXECUTE; 

                case (current_instr[6:0])
                    7'b0110011: state <= C_CHECK_ILLEGAL_R; 
                    
                    7'b0010011: begin pending_exec_state <= C_EXEC_I_TYPE; state <= C_REG_FETCH_WAIT; end
                    7'b0000011: begin pending_exec_state <= C_EXEC_LOAD;   state <= C_REG_FETCH_WAIT; end
                    7'b0100011: begin pending_exec_state <= C_EXEC_STORE;  state <= C_REG_FETCH_WAIT; end
                    7'b1100011: begin pending_exec_state <= C_EXEC_BRANCH; state <= C_REG_FETCH_WAIT; end
                    7'b1100111: begin pending_exec_state <= C_EXEC_JALR;   state <= C_REG_FETCH_WAIT; end
                    
                    7'b1101111: state <= C_EXEC_JAL;
                    7'b0110111: state <= C_EXEC_LUI;
                    7'b0010111: state <= C_EXEC_AUIPC;
                    
                    7'b0001111: state <= C_EXECUTE; 
                    7'b1110011: state <= C_EXECUTE; 
                    
                    default:    state <= C_HALT; 
                endcase
            end

            C_CHECK_ILLEGAL_R: begin
                if (funct7 == 7'b0000001) begin
                    state <= C_HALT; 
                end else begin
                    state <= C_EXEC_R_TYPE; 
                end
            end

            C_REG_FETCH_WAIT: begin
                state <= pending_exec_state; 
            end

            C_EXEC_R_TYPE: begin
                write_enable <= 1; 
                case (funct3)
                    3'b000: begin
                        if (funct7 == 7'b0100000) write_data <= read1_data - read2_data; 
                        else write_data <= read1_data + read2_data; 
                    end
                    3'b001: write_data <= read1_data << read2_data[4:0]; 
                    3'b010: write_data <= ($signed(read1_data) < $signed(read2_data)) ? 32'd1 : 32'd0; 
                    3'b011: write_data <= (read1_data < read2_data) ? 32'd1 : 32'd0; 
                    3'b100: write_data <= read1_data ^ read2_data; 
                    3'b101: begin
                        if (funct7 == 7'b0100000) 
                            write_data <= $signed(read1_data) >>> read2_data[4:0]; 
                        else 
                            write_data <= read1_data >> read2_data[4:0]; 
                    end
                    3'b110: write_data <= read1_data | read2_data; 
                    3'b111: write_data <= read1_data & read2_data; 
                endcase
                state <= return_state; 
            end

            C_EXEC_I_TYPE: begin
                write_enable <= 1; 
                case (funct3)
                    3'b000: write_data <= read1_data + imm_i; 
                    3'b001: write_data <= read1_data << imm_i[4:0]; 
                    3'b010: write_data <= ($signed(read1_data) < $signed(imm_i)) ? 32'd1 : 32'd0; 
                    3'b011: write_data <= (read1_data < imm_i) ? 32'd1 : 32'd0; 
                    3'b100: write_data <= read1_data ^ imm_i; 
                    3'b101: begin
                        if (current_instr[30] == 1'b1) 
                            write_data <= $signed(read1_data) >>> imm_i[4:0]; 
                        else 
                            write_data <= read1_data >> imm_i[4:0]; 
                    end
                    3'b110: write_data <= read1_data | imm_i; 
                    3'b111: write_data <= read1_data & imm_i; 
                endcase
                state <= return_state; 
            end

            C_EXEC_LOAD: begin
                data_addr <= read1_data + imm_i; 
                is_instruction_fetch <= 0; 
                state <= C_DDR_WAIT_READ;  
            end

            C_EXEC_STORE: begin
                data_addr <= read1_data + imm_s; 
                cpu_store_data <= read2_data; 
                state <= C_DDR_WAIT_WRITE; 
            end

            C_EXEC_BRANCH: begin
                state <= return_state; 
                case (funct3)
                    3'b000: if (read1_data == read2_data) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1; 
                        state <= C_DDR_WAIT_READ; 
                    end
                    3'b001: if (read1_data != read2_data) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1; 
                        state <= C_DDR_WAIT_READ; 
                    end
                    3'b100: if ($signed(read1_data) < $signed(read2_data)) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1; 
                        state <= C_DDR_WAIT_READ; 
                    end
                    3'b101: if ($signed(read1_data) >= $signed(read2_data)) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1; 
                        state <= C_DDR_WAIT_READ; 
                    end
                    3'b110: if (read1_data < read2_data) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1; 
                        state <= C_DDR_WAIT_READ; 
                    end
                    3'b111: if (read1_data >= read2_data) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1; 
                        state <= C_DDR_WAIT_READ; 
                    end
                endcase
            end

            C_EXEC_JAL: begin
                write_data <= program_counter + 32'd4;
                write_enable <= 1;
                program_counter <= program_counter + imm_j;
                is_instruction_fetch <= 1; 
                state <= C_DDR_WAIT_READ; 
            end

            C_EXEC_JALR: begin
                write_data <= program_counter + 32'd4;
                write_enable <= 1;
                program_counter <= (read1_data + imm_i) & 32'hFFFF_FFFE;
                is_instruction_fetch <= 1; 
                state <= C_DDR_WAIT_READ; 
            end

            C_EXEC_LUI: begin
                write_data <= imm_u;
                write_enable <= 1;
                state <= C_EXECUTE; 
            end

            C_EXEC_AUIPC: begin
                write_data <= program_counter + imm_u;
                write_enable <= 1;
                state <= C_EXECUTE; 
            end

            // ========================================================
            // I/O OPERATIONS 
            // ========================================================
            C_IO_READ: begin
                if (data_addr == 32'h4010_0000) begin
                    write_data <= ps2_data;
                end else if (data_addr == 32'h4030_0000) begin 
                    write_data <= ms_counter;
                // --- NEW V-SYNC REGISTER ---
                end else if (data_addr == 32'h4040_0000) begin 
                    write_data <= {31'd0, VGA_VSYNC};
                // ---------------------------
                end else begin
                    write_data <= 32'd0; 
                end
                write_enable <= 1; 
                state <= C_EXECUTE; 
            end

            C_IO_WRITE: begin
                if (data_addr >= 32'h4000_0000 && data_addr <= 32'h4000_FFFC) begin
                    vram_addr_out <= data_addr[15:2]; 
                    vram_data_out <= cpu_store_data; 
                    vram_write_en <= 1; 
                    state <= C_EXECUTE; 
                end
                else if (data_addr == 32'h4020_0000) begin
                    cpu_uart_msg <= cpu_store_data; 
                    cpu_byte_idx <= 0;
                    return_state <= C_EXECUTE; 
                    state <= C_UART_SEND;  
                end
                else begin
                    state <= C_EXECUTE;
                end
            end

            // ========================================================
            // THE WRAP-UP STATE & PREFETCH TRIGGER
            // ========================================================
            C_EXECUTE: begin
                write_enable <= 0; 
                vram_write_en <= 0; 
                
                program_counter <= program_counter + 32'd4;
                is_instruction_fetch <= 1; 
                
                // If we just finished executing the very first instruction (00) of a block
                // Let's trigger a background prefetch for the NEXT block!
                if (program_counter[3:2] == 2'b00) begin 
                    state <= C_PREFETCH_ISSUE;
                end else if (program_counter[3:2] == 2'b11) begin 
                    state <= C_DDR_WAIT_READ; // We finished the 4th, time to get new block
                end else begin
                    state <= C_LOAD_INSTR; // Go to 2nd or 3rd instruction
                end
            end

            // --- THE BACKGROUND PREFETCHER STATES ---
            C_PREFETCH_ISSUE: begin
                if (ddr_cmd_ready) begin
                    cpu_ddr_cmd <= 3'b001;
                    cpu_ddr_cmd_en <= 1;
                    // Request the NEXT 128-bit block (Current Base PC + 16 bytes)
                    cpu_user_addr <= {program_counter[27:4], 3'b000} + 27'd8; 
                    rq_is_data[rq_head] <= 0; // Label as Instruction request
                    case (rq_head)
                        2'd0: rq_addr_0 <= {program_counter[27:4], 3'b000} + 27'd8;
                        2'd1: rq_addr_1 <= {program_counter[27:4], 3'b000} + 27'd8;
                        2'd2: rq_addr_2 <= {program_counter[27:4], 3'b000} + 27'd8;
                        2'd3: rq_addr_3 <= {program_counter[27:4], 3'b000} + 27'd8;
                    endcase
                    rq_head <= rq_head + 1;
                    state <= C_PREFETCH_CLEANUP;
                end
            end

            C_PREFETCH_CLEANUP: begin
                cpu_ddr_cmd_en <= 0;
                state <= C_LOAD_INSTR; // Safely return to executing the current block!
            end
            // ----------------------------------------

            // ========================================================
            // LOAD STATE (BYTE/HALFWORD/WORD)
            // ========================================================
            C_FINISH_DATA_READ: begin
                // 1. Extract the raw 32-bit word. 
                case (data_addr[3:2])
                    2'b00: raw_word = {memory_read_reg[103:96], memory_read_reg[111:104], memory_read_reg[119:112], memory_read_reg[127:120]};
                    2'b01: raw_word = {memory_read_reg[71:64],  memory_read_reg[79:72],   memory_read_reg[87:80],   memory_read_reg[95:88]};
                    2'b10: raw_word = {memory_read_reg[39:32],  memory_read_reg[47:40],   memory_read_reg[55:48],   memory_read_reg[63:56]};
                    2'b11: raw_word = {memory_read_reg[7:0],    memory_read_reg[15:8],    memory_read_reg[23:16],   memory_read_reg[31:24]};
                endcase

                // 2. Slice and Extend the data based on funct3!
                case (funct3)
                    3'b000: // LB (Load Byte, Sign-Extended)
                        case (data_addr[1:0])
                            2'b00: write_data <= {{24{raw_word[7]}},  raw_word[7:0]};
                            2'b01: write_data <= {{24{raw_word[15]}}, raw_word[15:8]};
                            2'b10: write_data <= {{24{raw_word[23]}}, raw_word[23:16]};
                            2'b11: write_data <= {{24{raw_word[31]}}, raw_word[31:24]};
                        endcase
                    3'b100: // LBU (Load Byte, Zero-Extended)
                        case (data_addr[1:0])
                            2'b00: write_data <= {24'd0, raw_word[7:0]};
                            2'b01: write_data <= {24'd0, raw_word[15:8]};
                            2'b10: write_data <= {24'd0, raw_word[23:16]};
                            2'b11: write_data <= {24'd0, raw_word[31:24]};
                        endcase
                    3'b001: // LH (Load Halfword, Sign-Extended)
                        if (data_addr[1] == 1'b0) write_data <= {{16{raw_word[15]}}, raw_word[15:0]};
                        else                      write_data <= {{16{raw_word[31]}}, raw_word[31:16]};
                    3'b101: // LHU (Load Halfword, Zero-Extended)
                        if (data_addr[1] == 1'b0) write_data <= {16'd0, raw_word[15:0]};
                        else                      write_data <= {16'd0, raw_word[31:16]};
                    3'b010: // LW (Load Word)
                        write_data <= raw_word;
                    default: 
                        write_data <= raw_word;
                endcase

                write_enable <= 1; 
                state <= C_EXECUTE; 
            end

            // ========================================================
            // STORE STATE (BYTE/HALFWORD/WORD MASKS) 
            // ========================================================
            C_DDR_WAIT_WRITE: begin
                if (data_addr >= 32'h40000000) begin
                    state <= C_IO_WRITE;
                end
                else if (ddr_cmd_ready && ddr_wr_data_rdy) begin
                    cpu_ddr_cmd <= 3'b000; 
                    cpu_ddr_cmd_en <= 1;
                    cpu_user_addr <= {data_addr[27:4], 3'b000}; 
                    
                    // 1. Copy the target data to all possible byte-lanes (Dynamic Payload)
                    case (funct3[1:0])
                        2'b00: active_payload = {4{cpu_store_data[7:0]}}; // SB 
                        2'b01: active_payload = {2{cpu_store_data[7:0], cpu_store_data[15:8]}}; // SH 
                        2'b10: active_payload = {cpu_store_data[7:0], cpu_store_data[15:8], cpu_store_data[23:16], cpu_store_data[31:24]}; // SW
                        default: active_payload = {cpu_store_data[7:0], cpu_store_data[15:8], cpu_store_data[23:16], cpu_store_data[31:24]};
                    endcase
                    
                    cpu_ddr_wr_data <= {4{active_payload}}; 

                    // 2. Generate the 4-bit Word Mask
                    case (funct3[1:0])
                        2'b00: // SB (Store Byte)
                            case (data_addr[1:0])
                                2'b00: base_mask = 4'b0111; // Enable Byte 0
                                2'b01: base_mask = 4'b1011; // Enable Byte 1
                                2'b10: base_mask = 4'b1101; // Enable Byte 2
                                2'b11: base_mask = 4'b1110; // Enable Byte 3
                            endcase
                        2'b01: // SH (Store Halfword)
                            if (data_addr[1] == 1'b0) base_mask = 4'b0011;
                            else                      base_mask = 4'b1100;
                        2'b10: // SW (Store Word)
                            base_mask = 4'b0000;
                        default: 
                            base_mask = 4'b0000;
                    endcase

                    // 3. Shift the 4-bit mask into the gigantic 16-bit DDR3 mask
                    case (data_addr[3:2])
                        2'b00: cpu_ddr_wr_data_mask <= {base_mask, 12'hFFF}; 
                        2'b01: cpu_ddr_wr_data_mask <= {4'hF, base_mask, 8'hFF}; 
                        2'b10: cpu_ddr_wr_data_mask <= {8'hFF, base_mask, 4'hF}; 
                        2'b11: cpu_ddr_wr_data_mask <= {12'hFFF, base_mask}; 
                    endcase

                    cpu_ddr_wr_data_en <= 1;
                    cpu_ddr_wr_data_end <= 1;
                    state <= C_DDR_WRITE; 
                end       
            end

            C_DDR_WRITE: begin
                cpu_ddr_cmd_en <= 0;
                cpu_ddr_wr_data_en <= 0;
                cpu_ddr_wr_data_end <= 0;
                state <= C_EXECUTE; 
            end

            // ========================================================
            // CRASH / END OF PROGRAM DUMP
            // ========================================================
            C_HALT: begin
                dump_reg_idx <= 5'd0;    
                state <= C_HALT_SETUP;
            end

            C_HALT_SETUP: begin
                read1_addr <= dump_reg_idx; 
                state <= C_HALT_FETCH;
            end

            C_HALT_FETCH: begin
                cpu_uart_msg <= read1_data; 
                cpu_byte_idx <= 0;
                return_state <= C_HALT_NEXT; 
                state <= C_UART_SEND;        
            end

            C_HALT_NEXT: begin
                if (dump_reg_idx == 5'd31) begin
                    state <= C_HALT_FOREVER; 
                end else begin
                    dump_reg_idx <= dump_reg_idx + 1; 
                    state <= C_HALT_SETUP;
                end
            end

            C_HALT_FOREVER: begin
                cpu_done <= 1; 
                state <= C_HALT_FOREVER;
            end
                        
            default: state <= C_DDR_WAIT_READ;
        endcase
    end 
    // --- CTRL+ESC SOFT REBOOT TRIGGER FOR CPU ---
    else if (cpu_done && ps2_data[4] && ps2_data[7]) begin
        state <= C_DDR_WAIT_READ;
        cpu_ddr_cmd_en <= 0;
        cpu_ddr_wr_data_en <= 0;
        cpu_ddr_wr_data_end <= 0;
        cpu_ddr_wr_data_mask <= 16'h0000;
        cpu_uart_wr_en <= 0;
        cpu_user_addr <= 27'd0;
        program_counter <= 32'd0;
        memory_read_reg <= 0;
        cpu_byte_idx <= 0;
        is_instruction_fetch <= 1;
        data_addr <= 32'd0;
        cpu_store_data <= 32'd0;
        vram_write_en <= 0;
        dump_reg_idx <= 5'd0;
        pending_exec_state <= 0;
        cpu_done <= 0; 
        
        rq_head <= 0; rq_tail <= 0; rq_is_data <= 0;
        pf_valid <= 0; dmem_valid <= 0; pf_ready_addr <= 27'h7FFFFFF;
    end
end

endmodule