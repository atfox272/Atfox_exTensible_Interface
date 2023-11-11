`timescale 1ns / 1ps

module ATI_tb#(
    // SYSTEM BUS parameter
    parameter DATA_BUS_WIDTH            = 64,
    parameter ADDR_BUS_WIDTH            = 64,
    parameter CHANNEL_WIDTH             = 2,
    parameter DEVICE_CHANNEL_ID         = 2'h00,
    parameter DATA_TYPE_WIDTH           = 2,    // DoubleWord - Word - Byte (-> 3 type)
    parameter BYTE_TYPE_ENCODE          = 0,
    parameter WORD_TYPE_ENCODE          = 1,
    parameter DOUBLEWORD_TYPE_ENCODE    = 2,
    // Device parameter 
    parameter DEVICE_DATA_WIDTH         = 8,
    
    // DEEP CONFIGURATION
    parameter INTERNAL_CLOCK            = 115200 * 8,
    parameter PACKET_TIMEOUT            = 260416,       // 2 transactions timeout ( = 120Mhz / (9600 / 20))
    parameter MASTER_BUFFER_SIZE        = 4,
    parameter SLAVE_BUFFER_SIZE         = 32,           // 4 DoubleWords
    
    parameter POINTER_MASTER_BUF        = $clog2(MASTER_BUFFER_SIZE),
    parameter POINTER_SLAVE_BUF         = $clog2(MASTER_BUFFER_SIZE)
    );
    localparam BD115200_8N1      = 8'b11100011;
    localparam DATA_WIDTH        = 8;
    reg                                 clk;
    // SYSTEM BUS interface ////////////////////
    // DATA_BUS
    reg     [DATA_BUS_WIDTH - 1:0]      data_bus_in;
    wire    [DATA_BUS_WIDTH - 1:0]      data_bus_out;
    // ADDRESS_BUS
    reg [ADDR_BUS_WIDTH - 1:0]          addr_bus_in;
    // CONTROL_BUS
    wire                                buffer_rd_available;
    wire                                buffer_wr_available;
    reg                                 rd_req;
    reg                                 wr_req;
    reg [DATA_TYPE_WIDTH - 1:0]         data_type_encode;      // Doubleword - Word - Byte
    // DEVICE interface ////////////////////////
    wire [DEVICE_DATA_WIDTH - 1:0]      device_data_in;
    wire [DEVICE_DATA_WIDTH - 1:0]      device_data_out;
    wire                                device_wr_ins;
    wire                                device_rd_ins;
    
    wire device_rd_available;    // Buffer of device is available (not full)
    wire device_wr_available;    // Buffer of device is available (not full)
    
    reg rst_n;
    
    Atfox_exTensible_Interface
        #(
        .DEVICE_CHANNEL_ID(2'b00),
        .PACKET_TIMEOUT(10000)
        ) Atfox_exTensible_Interface_UART (
        .clk(clk),
        .data_bus_in(data_bus_in),
        .data_bus_out(data_bus_out),
        .addr_bus_in(addr_bus_in),
        .buffer_rd_available(buffer_rd_available),
        .buffer_wr_available(buffer_wr_available),
        .rd_req(rd_req),
        .wr_req(wr_req),
        .data_type_encode(data_type_encode),
        
        .device_data_in(device_data_in),
        .device_data_out(device_data_out),
        .device_wr_ins(device_wr_ins),
        .device_rd_ins(device_rd_ins),
        .device_rd_available(device_rd_available),
        .device_wr_available(device_wr_available),
        
        .rst_n(rst_n)
        );

    wire [DATA_WIDTH - 1:0] common_config_register = BD115200_8N1;
    
    wire RX_1;
    wire TX_1;
    uart_peripheral 
        #(
        .INTERNAL_CLOCK(INTERNAL_CLOCK)
        )uart_peripheral_1(
        .clk(clk),
        .RX(RX_1),
        .TX(TX_1),
        .RX_config_register(common_config_register),
        .TX_config_register(common_config_register),
        // TX interface 
        .data_in(device_data_out),
        .TX_use(device_wr_ins),
        .TX_flag(),
        .TX_complete(),
        .TX_available(device_wr_available),
        // RX interface
        .data_out(device_data_in),
        .RX_use(device_rd_ins),
        .RX_flag(device_rd_available),
        .RX_available(),
        
        .rst_n(rst_n)
        );
      
    wire RX_2;
    wire TX_2;
    reg  [DATA_WIDTH - 1:0] data_in_2;
    reg  TX_use_2;
    wire [DATA_WIDTH - 1:0] data_out_2;
    wire RX_flag_2;
    uart_peripheral 
        #(
        .RX_FLAG_TYPE(1'b0),
        .FIFO_DEPTH(1024),
        .INTERNAL_CLOCK(INTERNAL_CLOCK)
        )uart_peripheral_2(
        .clk(clk),
        .RX(RX_2),
        .TX(TX_2),
        .RX_config_register(common_config_register),
        .TX_config_register(common_config_register),
        // TX interface 
        .data_in(data_in_2),
        .TX_use(TX_use_2),
        .TX_flag(),
        .TX_complete(),
        .TX_available(),
        // RX interface
        .data_out(data_out_2),
        .RX_use(),
        .RX_flag(RX_flag_2),
        .RX_available(),
        
        .rst_n(rst_n)
        );        
    assign RX_1 = TX_2;
    assign RX_2 = TX_1;
    
    initial begin
        clk <= 0;
        data_bus_in <= 0;
        addr_bus_in <= 0;
        rd_req <= 0;
        wr_req <= 0;
        data_type_encode <= 0;
        data_in_2 <= 0;
        TX_use_2 <= 0;
        rst_n <= 1;
        #1 rst_n <= 0;
        #9 rst_n <= 1;
    end
    initial begin
        forever #1 clk <= ~clk;
    end
    
    initial begin
        #11;
        
        for(int i = 0; i < 10; i = i + 1) begin
        #1;
        data_in_2 <= 8'hff - i;
        #1 TX_use_2 <= 1;
        #2 TX_use_2 <= 0;
        end 
    end
    initial begin
        #2000;
        $stop;
        #2000;
        $stop;
        #2000;
        $stop;
        #2000;
        $stop;
        #2000;
        $stop;
    end
endmodule
