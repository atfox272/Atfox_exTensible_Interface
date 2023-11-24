`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 
// Design Name: 
// Module Name: Atfox_exTensible_Interface
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Atfox-exTensible-Interface (ATI) is a slave's interface, which support for ATI System Bus Structure
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
// ATI Bus Structure (Interface for slave)
// ADDRESS_BUS format
// <CHANNEL_ID> - <ADDRESS_ID>
// Note:
//  - LOAD_TYPE:    + BYTE*         -> invalid (align to WORD automatically) (cause segmentation in buffer)
//                  + WORD          -> valid     
//                  + DOUBLE_WORD   -> valid
//
//  - STORE_TYPE:   + BYTE          -> valid 
//                  + WORD          -> valid     
//                  + DOUBLE_WORD   -> valid
//

module Atfox_exTensible_Interface
    #(
    // SYSTEM BUS parameter
    parameter DATA_BUS_WIDTH            = 64,
    parameter ADDR_BUS_WIDTH            = 64,
    parameter CHANNEL_AMOUNT            = 4,
    parameter CHANNEL_WIDTH             = $clog2(CHANNEL_AMOUNT),
    parameter CHANNEL_ID                = 2'b00,
    parameter DATA_TYPE_WIDTH           = 2,    // DoubleWord - Word - Byte (-> 3 type)
    parameter BYTE_TYPE_ENCODE          = 0,
    parameter WORD_TYPE_ENCODE          = 1,
    parameter DOUBLEWORD_TYPE_ENCODE    = 2,
    // Device parameter 
    parameter INTERFACE_MASTER_ENCODE   = 3,
    parameter INTERFACE_DMEM_ENCODE     = 0,
    parameter INTERFACE_PERP_ENCODE     = 1,
    parameter INTERFACE_GPIO_ENCODE     = 2,
    parameter INTERFACE_TYPE            = INTERFACE_PERP_ENCODE,
    parameter DEVICE_DATA_WIDTH         = (INTERFACE_TYPE == INTERFACE_DMEM_ENCODE) ? 64 : 8,
    
    /* For Master */
    parameter M_ATI_DATA_WIDTH          = 64,
    parameter M_ATI_ADDR_WIDTH          = 64,
    /* For Memory */
    parameter INTERFACE_DMEM_THROUGH    = 0,    // 1 - Pass interface through, 0 - Use buffer in Writing Case (Will increase latency - 1cycle)
    /* For GPIO */
    parameter PORT_AMOUNT               = 2,
    parameter PIN_AMOUNT                = 8,
    
    // DEEP CONFIGURATION
    parameter INTERNAL_CLOCK            = 125000000,
    parameter PACKET_TIMEOUT            = 260416,       // 2 transactions timeout ( = 120Mhz / (9600 / 20))
    parameter MASTER_BUFFER_SIZE        = 4,
    parameter SLAVE_BUFFER_SIZE         = 32,           // 4 DoubleWords
    
    parameter POINTER_MASTER_BUF        = $clog2(MASTER_BUFFER_SIZE),
    parameter POINTER_SLAVE_BUF         = $clog2(SLAVE_BUFFER_SIZE)
    )
    (
    input                               clk,
    /*          Master's interface           */
    output [M_ATI_DATA_WIDTH - 1:0]     m_ati_rdata,
    input  [M_ATI_DATA_WIDTH - 1:0]     m_ati_wdata,
    input  [M_ATI_ADDR_WIDTH - 1:0]     m_ati_addr,
    output                              m_ati_rd_available,
    output                              m_ati_wr_available,
    input                               m_ati_rd_req,
    input                               m_ati_wr_req,
    input  [DATA_TYPE_WIDTH - 1:0]      m_ati_data_type,
    /*          Master's stream           */
    input  [M_ATI_DATA_WIDTH - 1:0]     m_atis_rdata_bus   [0:CHANNEL_AMOUNT - 1],
    output [M_ATI_DATA_WIDTH - 1:0]     m_atis_wdata_bus,
    output [M_ATI_ADDR_WIDTH - 1:0]     m_atis_addr_bus,
    input                               m_atis_rd_available[0:CHANNEL_AMOUNT - 1],
    input                               m_atis_wr_available[0:CHANNEL_AMOUNT - 1],
    output                              m_atis_rd_req,
    output                              m_atis_wr_req,
    output [DATA_TYPE_WIDTH - 1:0]      m_atis_data_type,
    
    /*          Slave's interface           */
    // SYSTEM BUS interface ////////////////////
    // DATA_BUS
    input  [DATA_BUS_WIDTH - 1:0]       data_bus_in,
    inout  [DATA_BUS_WIDTH - 1:0]       data_bus_out,
    // ADDRESS_BUS
    input  [ADDR_BUS_WIDTH - 1:0]       addr_bus_in,
    // CONTROL_BUS
    inout                               buffer_rd_available,
    inout                               buffer_wr_available,
    input                               rd_req,
    input                               wr_req,
    input  [DATA_TYPE_WIDTH - 1:0]      data_type_encode,      // Doubleword - Word - Byte
    // DEVICE interface ////////////////////////
    input  [DEVICE_DATA_WIDTH - 1:0]    device_data_in,
    output [DEVICE_DATA_WIDTH - 1:0]    device_data_out,
    output  logic                       device_wr_ins,
    output  logic                       device_rd_ins,
    input                               device_rd_available,    // Buffer of device is available (not full)
    input                               device_wr_available,    // Buffer of device is available (not full)
    /* For Memory & GPIO */
    output [ADDR_BUS_WIDTH - 1:0]       device_addr_rd,
    output [ADDR_BUS_WIDTH - 1:0]       device_addr_wr,
    output [DATA_TYPE_WIDTH - 1:0]      device_data_type_rd,      // Doubleword - Word - Byte
    output [DATA_TYPE_WIDTH - 1:0]      device_data_type_wr,      // Doubleword - Word - Byte
    input                               rst_n
    );
    
    localparam BYTE_WIDTH                   = 8;
    localparam WORD_WIDTH                   = 32;
    localparam DOUBLEWORD_WIDTH             = 64;
    
    wire address_decoder = (addr_bus_in[ADDR_BUS_WIDTH - 1: ADDR_BUS_WIDTH - CHANNEL_WIDTH] == CHANNEL_ID);
    generate
    if(INTERFACE_TYPE == INTERFACE_PERP_ENCODE) begin     : INTERFACE_PERP_BLOCK
    
    reg rd_device_state;
    localparam OUT_WORD_BLOCK_STATE         = 1'd0;    
    localparam IN_WORD_BLOCK_STATE          = 1'd1;   
    
    localparam BLOCK_AMOUNT                 = DATA_BUS_WIDTH / DEVICE_DATA_WIDTH;
    localparam BLOCK_INDEX_WIDTH            = $clog2(BLOCK_AMOUNT);
    
    localparam WORD_BLOCK_SIZE_BYTE         = (WORD_WIDTH / BYTE_WIDTH);    // 4
    localparam WORD_BLOCK_AMOUNT            = SLAVE_BUFFER_SIZE / WORD_BLOCK_SIZE_BYTE;
    localparam WORD_BLOCK_INDEX_W           = $clog2(WORD_BLOCK_AMOUNT);
    
    localparam DOUBLEWORD_BLOCK_SIZE_WORD   = (DOUBLEWORD_WIDTH / WORD_WIDTH);
    localparam DOUBLEWORD_BLOCK_AMOUNT      = WORD_BLOCK_AMOUNT / DOUBLEWORD_BLOCK_SIZE_WORD;
    
    reg [DATA_BUS_WIDTH - 1:0]          master_buffer [0:MASTER_BUFFER_SIZE - 1];
    reg [DATA_TYPE_WIDTH - 1:0]         data_type_mbuf[0:MASTER_BUFFER_SIZE - 1];   // data_type of element in master_buffer
    reg [POINTER_MASTER_BUF - 1:0]      front_pointer_mbuf;
    reg [POINTER_MASTER_BUF - 1:0]      rear_pointer_mbuf;
    
    reg [DEVICE_DATA_WIDTH - 1:0]       slave_buffer  [0:SLAVE_BUFFER_SIZE - 1];
    reg [DATA_TYPE_WIDTH - 1:0]         data_type_sbuf[0:SLAVE_BUFFER_SIZE - 1];    // data_type of element in slave_buffer
    reg [POINTER_SLAVE_BUF - 1:0]       front_pointer_sbuf;
    reg [POINTER_SLAVE_BUF - 1:0]       rear_pointer_sbuf;
    logic [POINTER_SLAVE_BUF - 1:0]     front_pointer_sbuf_next;
    
    wire                                wr_req_decode;
    wire                                rd_req_decode;
    wire                                full_mbuf;
    wire                                empty_mbuf;
    wire                                full_sbuf;
    wire                                empty_sbuf;
    logic[DATA_BUS_WIDTH - 1:0]         data_bus_out_buf;
    
    
    wire [WORD_WIDTH - 1:0]                             word_block_sbuf         [0:WORD_BLOCK_AMOUNT - 1];
    reg  [BYTE_WIDTH - 1:0]                             amt_word_block_sbuf     [0:WORD_BLOCK_AMOUNT - 1];
    wire                                                valid_word_block_sbuf   [0:WORD_BLOCK_AMOUNT - 1];
    reg                                                 valid_word_block_sbuf_hi[0:WORD_BLOCK_AMOUNT - 1];
    reg                                                 valid_word_block_sbuf_lo[0:WORD_BLOCK_AMOUNT - 1];
    wire [WORD_BLOCK_INDEX_W - 1:0]                     index_word_block_wr_cur;
    wire [WORD_BLOCK_INDEX_W - 1:0]                     index_word_block_wr_next;
    wire [WORD_BLOCK_INDEX_W - 1:0]                     index_word_block_wr_prev;
    wire                                                full_word_block_wr;
    wire [WORD_BLOCK_INDEX_W - 1:0]                     index_word_block_rd_cur;
    wire [WORD_BLOCK_INDEX_W - 1:0]                     index_word_block_rd_next;
    wire [POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W - 1:0] offset_word_block;
    
    wire[DOUBLEWORD_WIDTH - 1:0]    dword_block_sbuf        [0:DOUBLEWORD_BLOCK_AMOUNT - 1];
    wire[BYTE_WIDTH - 1:0]          amt_dword_block_sbuf    [0:DOUBLEWORD_BLOCK_AMOUNT - 1];
    wire                            valid_dword_block_sbuf  [0:DOUBLEWORD_BLOCK_AMOUNT - 1];
    
    logic buffer_rd_valid;
    logic buffer_rd_valid_dword;
    logic buffer_rd_valid_word;
    
    reg     word_block_timeout_en;
    wire    word_block_timeout_flag;
    
    real_time 
        #(
        .MAX_COUNTER(PACKET_TIMEOUT)
        ) timeout_packet (
        .clk(clk),
        .counter_enable(word_block_timeout_en),
        .limit_counter(PACKET_TIMEOUT),
        .limit_flag(word_block_timeout_flag),
        .rst_n(rst_n) 
        );
    
    // Address decoder   
    assign data_bus_out         = (address_decoder) ? data_bus_out_buf  : {64{1'bz}};
    assign buffer_wr_available  = (address_decoder) ? ~full_mbuf        : {1{1'bz}};
    assign buffer_rd_available  = (address_decoder) ? buffer_rd_valid   : {1{1'bz}};
    assign wr_req_decode        = (address_decoder) ? wr_req : 1'b0;
    assign rd_req_decode        = (address_decoder) ? rd_req : 1'b0;
    
    assign full_mbuf = (rear_pointer_mbuf + 1) == front_pointer_mbuf;
    assign empty_mbuf= rear_pointer_mbuf == front_pointer_mbuf;
    
    always @(posedge clk) begin
        if(!rst_n) begin
            rear_pointer_mbuf <= 0;
        end 
        else if(wr_req_decode) begin
            master_buffer[rear_pointer_mbuf] <= data_bus_in;
            data_type_mbuf[rear_pointer_mbuf] <= data_type_encode;
            rear_pointer_mbuf <= rear_pointer_mbuf + 1;
        end
    end
    
    logic[DEVICE_DATA_WIDTH - 1:0]      block_mux;
    reg  [BLOCK_INDEX_WIDTH - 1:0]      block_index;
    logic[BLOCK_INDEX_WIDTH - 1:0]      block_index_limit;  // 1 - 4 - 8 bytes 
    wire [DATA_BUS_WIDTH - 1:0]         front_element_mbuf;
    
    assign front_element_mbuf = master_buffer[front_pointer_mbuf];
    assign device_data_out = block_mux;
    always_comb begin
    
    case(block_index)
        0: block_mux = front_element_mbuf[7:0];
        1: block_mux = front_element_mbuf[15:8];
        2: block_mux = front_element_mbuf[23:16];
        3: block_mux = front_element_mbuf[31:24];
        4: block_mux = front_element_mbuf[39:32];
        5: block_mux = front_element_mbuf[47:40];
        6: block_mux = front_element_mbuf[55:48];
        7: block_mux = front_element_mbuf[63:56];
        default: block_mux = 8'h00;
    endcase
    
    case(data_type_mbuf[front_pointer_mbuf])
        BYTE_TYPE_ENCODE: begin
            block_index_limit = BYTE_WIDTH / BYTE_WIDTH - 1;
        end 
        WORD_TYPE_ENCODE: begin
            block_index_limit = WORD_WIDTH / BYTE_WIDTH - 1;
        end
        DOUBLEWORD_TYPE_ENCODE: begin
            block_index_limit = DOUBLEWORD_WIDTH / BYTE_WIDTH - 1;
        end
        default: begin
            block_index_limit = BYTE_WIDTH / BYTE_WIDTH - 1;
        end
    endcase
    end 
    
    always @(posedge clk) begin
        if(!rst_n) begin
            front_pointer_mbuf <= 0;
            block_index <= 0;
            device_wr_ins <= 0; // Sample tp "block_mux" directly
        end
        else if(~empty_mbuf & device_wr_available) begin
            if(device_wr_ins) begin                 // Load 
                if(block_index == block_index_limit) begin
                    block_index <= 0;
                    front_pointer_mbuf <= front_pointer_mbuf + 1;
                end
                else begin
                    block_index <= block_index + 1;
                end
            end
            else begin                              // Sample
            
            end
            device_wr_ins <= ~device_wr_ins;
        end 
    end
    
    for(genvar i = 0; i < WORD_BLOCK_AMOUNT; i = i + 1) begin
    assign valid_word_block_sbuf[i] = valid_word_block_sbuf_hi[i] ^ valid_word_block_sbuf_lo[i];
    end 
    for(genvar i = 0; i < WORD_BLOCK_AMOUNT; i = i + 1) begin
        // Format of DATA_BUS in WORD_TYPE
        //  ---------------------------------------------------
        //                  |                    |
        //                  |     DON'T CARE     | // Will be preempted by sign-extend value
        //                  |                    |
        //  ---------------------------------------------------
        //                  |                    |
        //                  |DATA_WIDTH  = 3bytes|
        //                  |                    |
        //  ---------------------------------------------------
        //                  |AMOUNT_WIDTH= 1byte |
        //  ---------------------------------------------------
        assign word_block_sbuf[i][WORD_WIDTH - 1:BYTE_WIDTH] = {slave_buffer[i*WORD_BLOCK_SIZE_BYTE + 3], slave_buffer[i*WORD_BLOCK_SIZE_BYTE + 2], slave_buffer[i*WORD_BLOCK_SIZE_BYTE + 1]};
        assign word_block_sbuf[i][BYTE_WIDTH - 1:0] = amt_word_block_sbuf[i];
    end 
    
    
    for(genvar i = 0; i < DOUBLEWORD_BLOCK_AMOUNT; i = i + 1) begin
        // Format of DATA_BUS in DOUBLE_WORD_TYPE:
        //  ---------------------------------------------------
        //                  |                    |
        //                  |                    |
        //                  |                    |
        //                  |DATA_WIDTH  = 6bytes|
        //                  |                    |
        //                  |                    |
        //                  |                    |
        //  ---------------------------------------------------
        //                  |AMOUNT_WIDTH= 2byte |
        //  ---------------------------------------------------
        assign dword_block_sbuf[i][DOUBLEWORD_WIDTH - 1:BYTE_WIDTH*DOUBLEWORD_BLOCK_SIZE_WORD] = {word_block_sbuf[i + 1][WORD_WIDTH - 1:BYTE_WIDTH],
                                                                                                  word_block_sbuf[i + 0][WORD_WIDTH - 1:BYTE_WIDTH]};
        assign dword_block_sbuf[i][BYTE_WIDTH*DOUBLEWORD_BLOCK_SIZE_WORD - 1:0]  =  amt_word_block_sbuf[i + 1] + amt_word_block_sbuf[i + 0];
        assign valid_dword_block_sbuf[i] = valid_word_block_sbuf[i + 1] & valid_word_block_sbuf[i + 0]; 
    end 
    
    // Buffer Format
    //
    //      Block Contain           |
    //            1 or more data    |                   |        |              |                |
    // |----------------|-----------|-------------------|--------|--------------|----------------|
    //                  |           |       DATA        |   #3   |              |                |
    //                  |           |-------------------|        |              |                |
    //                  |           |       DATA        |   #2   | WORD         |                |
    //                  |   VALID   |-------------------|        |   BLOCK #1   |                |
    //                  |WORD_BLOCK |       DATA        |   #1   |              |                |  <- offset_word_block
    //      VALID       |           |-------------------|        |              |                |
    //    DOUBLE_WORD   |           |      RESERVED     |   #0   |              | DOUBLE_WORD    |//  Replaced by AMOUNT value in DATA_BUS 
    //      BLOCK       |-----------|-------------------|--------|--------------|       BLOCK #0 |
    //                  |           |       DATA        |   #3   |              |                |
    //                  |           |-------------------|        |              |                |
    //                  |           |       DATA        |   #2   | WORD         |                |
    //                  |   VALID   |-------------------|        |   BLOCK #0   |                |
    //                  |WORD_BLOCK |       DATA        |   #1   |              |                |  <- offset_word_block
    //                  |           |-------------------|        |              |                |
    //                  |           |      RESERVED     |   #0   |              |                |//  Replaced by AMOUNT value in DATA_BUS
    // |----------------|-----------|-------------------|--------|--------------|----------------|
    //          
    //  
    //
    assign index_word_block_wr_cur  = rear_pointer_sbuf[POINTER_SLAVE_BUF - 1: POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W];
    assign index_word_block_wr_next = rear_pointer_sbuf[POINTER_SLAVE_BUF - 1: POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W] + 1;
    assign index_word_block_wr_prev = rear_pointer_sbuf[POINTER_SLAVE_BUF - 1: POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W] - 1;
    assign full_word_block_wr = (rear_pointer_sbuf[POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W - 1: 0] == {(POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W){1'b0}});
    assign index_word_block_rd_cur  = front_pointer_sbuf[POINTER_SLAVE_BUF - 1: POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W];
    assign index_word_block_rd_next  = front_pointer_sbuf[POINTER_SLAVE_BUF - 1: POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W] + 1;
    assign offset_word_block = {{(POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W - 1){1'b0}}, 1'b1};
//    assign full_sbuf = (rear_pointer_sbuf + 1) == front_pointer_sbuf;
    assign full_sbuf = index_word_block_rd_cur == (index_word_block_wr_cur + 1);
    always @(posedge clk) begin
        if(!rst_n) begin
            rear_pointer_sbuf <= 1;
            device_rd_ins <= 0;
            word_block_timeout_en <= 0;
            for(int i = 0; i < WORD_BLOCK_AMOUNT; i = i + 1) begin
            valid_word_block_sbuf_hi[i] <= 1'b0;
            amt_word_block_sbuf[i] <= 1'b0;
            end
            rd_device_state <= OUT_WORD_BLOCK_STATE;
        end
        else begin
            case(rd_device_state) 
                OUT_WORD_BLOCK_STATE: begin
                    word_block_timeout_en <= 0;
                    if(device_rd_available & !full_sbuf) begin
                        rd_device_state <= IN_WORD_BLOCK_STATE;
                        amt_word_block_sbuf[index_word_block_wr_cur] <= 0; 
                    end
                end 
                IN_WORD_BLOCK_STATE: begin
                    if(device_rd_available) begin
                        if(device_rd_ins) begin
                            rear_pointer_sbuf <= rear_pointer_sbuf + 1;
                            device_rd_ins <= 0;
                        end 
                        else begin
                            slave_buffer[rear_pointer_sbuf] <= device_data_in;
                            amt_word_block_sbuf[index_word_block_wr_cur] <= amt_word_block_sbuf[index_word_block_wr_cur] + 1; 
                            device_rd_ins <= 1;
                        end 
                    end 
                    else if (full_word_block_wr) begin
                        rd_device_state <= OUT_WORD_BLOCK_STATE;
                        rear_pointer_sbuf <= {index_word_block_wr_cur, offset_word_block};
                        valid_word_block_sbuf_hi[index_word_block_wr_prev] <= ~valid_word_block_sbuf_lo[index_word_block_wr_prev];
                    end
                    else if(word_block_timeout_flag) begin
                        rd_device_state <= OUT_WORD_BLOCK_STATE;
                        rear_pointer_sbuf <= {index_word_block_wr_next, offset_word_block};
                        valid_word_block_sbuf_hi[index_word_block_wr_cur] <= ~valid_word_block_sbuf_lo[index_word_block_wr_cur];
                    end  
                    word_block_timeout_en <= (word_block_timeout_flag | full_word_block_wr) ? 0 : !device_rd_available;
                end 
                
            endcase 
        end
    end
    
    always @(posedge clk) begin
        if(!rst_n) begin
            front_pointer_sbuf <= 1;
        end 
        else if(rd_req_decode & buffer_rd_valid) begin
            front_pointer_sbuf <= front_pointer_sbuf_next;
        end 
    end
    for(genvar i = 0; i < WORD_BLOCK_AMOUNT; i = i + 1) begin
    always @(posedge clk) begin
        if(!rst_n) begin
            valid_word_block_sbuf_lo[i] <= 1'b0;
        end
        else if(rd_req_decode & buffer_rd_valid) begin
            if(((data_type_encode == BYTE_TYPE_ENCODE | data_type_encode == WORD_TYPE_ENCODE) & index_word_block_rd_cur == i) | data_type_encode == DOUBLEWORD_TYPE_ENCODE & (index_word_block_rd_next == i)) begin
                valid_word_block_sbuf_lo[i] <= valid_word_block_sbuf_hi[i];
            end
        end
    end 
    end
    
    always_comb begin
    buffer_rd_valid_word    = valid_word_block_sbuf[index_word_block_rd_cur];
    buffer_rd_valid_dword   = buffer_rd_valid_word & valid_word_block_sbuf[index_word_block_rd_next];
    
    case(data_type_encode)
        BYTE_TYPE_ENCODE: begin
            buffer_rd_valid = buffer_rd_valid_word;
            data_bus_out_buf = {{(DOUBLEWORD_WIDTH - WORD_WIDTH){1'b0}}, word_block_sbuf[index_word_block_rd_cur]};
            front_pointer_sbuf_next = {index_word_block_rd_next, offset_word_block};
        end
        WORD_TYPE_ENCODE: begin
            buffer_rd_valid = buffer_rd_valid_word;
            data_bus_out_buf = {{(DOUBLEWORD_WIDTH - WORD_WIDTH){1'b0}}, word_block_sbuf[index_word_block_rd_cur]};
            front_pointer_sbuf_next = {index_word_block_rd_next, offset_word_block};
        end
        DOUBLEWORD_TYPE_ENCODE: begin
            data_bus_out_buf = {dword_block_sbuf[index_word_block_rd_cur]};
            buffer_rd_valid = buffer_rd_valid_dword;
            front_pointer_sbuf_next = {index_word_block_rd_next + 1, offset_word_block};
        end
        default: begin
            buffer_rd_valid = 1'b0;
            data_bus_out_buf = {{(DOUBLEWORD_WIDTH - WORD_WIDTH){1'b0}}, word_block_sbuf[index_word_block_rd_cur]};
            front_pointer_sbuf_next = {index_word_block_rd_next, offset_word_block};
        end
    endcase
    end 
    
    end 
    
    else if(INTERFACE_TYPE == INTERFACE_DMEM_ENCODE) begin  : MEMORY_BLOCK
        /* For 1wr-1rd  Memory */
        
        if(INTERFACE_DMEM_THROUGH == 0) begin
            reg [DATA_BUS_WIDTH - 1:0]  data_bus_wr_buffer;
            reg [DATA_TYPE_WIDTH - 1:0] device_data_type_wr_buffer;
            always @(posedge clk) begin
                if(!rst_n) begin
                    data_bus_wr_buffer <= 0;
                    device_data_type_wr_buffer <= 0;
                end
                else if(device_wr_ins) begin
                    data_bus_wr_buffer <= data_bus_in;
                    device_data_type_wr_buffer <= data_type_encode;
                end
            end
            assign device_data_out      = data_bus_wr_buffer;
            assign device_data_type_wr  = device_data_type_wr_buffer;
        end
        else begin
            assign device_data_out      = data_bus_in;
            assign device_data_type_wr  = data_type_encode;
        end
        
        assign data_bus_out         = (address_decoder) ? device_data_in : {DATA_BUS_WIDTH{1'bz}};
        assign buffer_rd_available  = (address_decoder) ? device_rd_available : {1{1'bz}};
        assign buffer_wr_available  = (address_decoder) ? device_wr_available : {1{1'bz}};
        assign device_addr_rd       = (address_decoder) ? addr_bus_in : {ADDR_BUS_WIDTH{1'b0}};
        assign device_addr_wr       = (address_decoder) ? addr_bus_in : {ADDR_BUS_WIDTH{1'b0}};
        assign device_rd_ins        = (address_decoder) ? rd_req : 1'b0;
        assign device_wr_ins        = (address_decoder) ? wr_req : 1'b0;
        assign device_data_type_rd  = data_type_encode;
        
    end 
    
    else if(INTERFACE_TYPE == INTERFACE_GPIO_ENCODE) begin  : GPIO_BLOCK

        localparam ADDR_PORT_WIDTH = $clog2(PORT_AMOUNT);
        
        assign data_bus_out         = (address_decoder) ? {{(DATA_BUS_WIDTH - DEVICE_DATA_WIDTH){1'b0}}, device_data_in} : {DATA_BUS_WIDTH{1'bz}};
        assign device_addr_rd       = (address_decoder) ? addr_bus_in : {ADDR_BUS_WIDTH{1'b0}};
        assign device_addr_wr       = (address_decoder) ? addr_bus_in : {ADDR_BUS_WIDTH{1'b0}};
        assign buffer_rd_available  = (address_decoder) ? device_rd_available : {1{1'bz}};
        assign buffer_wr_available  = (address_decoder) ? device_wr_available : {1{1'bz}};
        assign device_rd_ins        = (address_decoder) ? rd_req : 1'b0;
        assign device_wr_ins        = (address_decoder) ? wr_req : 1'b0;
        assign device_data_out      =  data_bus_in[DEVICE_DATA_WIDTH - 1:0];
        assign device_data_type     = 0;    // Only-Byte
    end 
    
    else if(INTERFACE_TYPE == INTERFACE_MASTER_ENCODE) begin  : MASTER_BLOCK
        wire[CHANNEL_WIDTH - 1:0] channel_decode;
        assign channel_decode       = m_ati_addr[M_ATI_ADDR_WIDTH - 1:M_ATI_ADDR_WIDTH - CHANNEL_WIDTH];
        assign m_ati_rdata          = m_atis_rdata_bus[channel_decode];
        assign m_ati_rd_available   = m_atis_rd_available[channel_decode];
        assign m_ati_wr_available   = m_atis_wr_available[channel_decode];
        
        assign m_atis_wdata_bus     = m_ati_wdata;
        assign m_atis_addr_bus      = m_ati_addr;
        assign m_atis_rd_req        = m_ati_rd_req;
        assign m_atis_wr_req        = m_ati_wr_req;
        assign m_atis_data_type     = m_ati_data_type;
    end
    endgenerate
endmodule
// 
//  Atfox_exTensible_Interface
//        #(
//        .CHANNEL_ID(),
//        .INTERFACE_DMEM_ENCODE(INTERFACE_DMEM_ENCODE),
//        .INTERFACE_PERP_ENCODE(INTERFACE_PERP_ENCODE),
//        .INTERFACE_GPIO_ENCODE(INTERFACE_GPIO_ENCODE),
//        .INTERFACE_TYPE()
//        ) Atfox_exTensible_Interface_UART (
//        .clk(clk),
//        .data_bus_in(),
//        .data_bus_out(),
//        .addr_bus_in(),
//        .buffer_rd_available(),
//        .buffer_wr_available(),
//        .rd_req(),
//        .wr_req(),
//        .data_type_encode(),
//        .device_data_in(),
//        .device_data_out(),
//        .device_wr_ins(),
//        .device_rd_ins(),
//        .device_rd_available(),
//        .device_wr_available(),
//        .device_addr_rd(),
//        .device_addr_wr(),
//        .device_data_type_rd(),
//        .device_data_type_wr(),
//        .rst_n(rst_n)
//        );
