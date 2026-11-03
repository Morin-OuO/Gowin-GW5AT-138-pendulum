`timescale 1ns/1ps

module uart_tx_byte #(
    parameter integer CLK_HZ = 27000000,
    parameter integer BAUD = 115200
) (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [7:0] data,
    output reg tx,
    output reg busy
);
    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;
    reg [1:0] state;
    reg [15:0] clk_count;
    reg [2:0] bit_index;
    reg [7:0] shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            clk_count <= 0;
            bit_index <= 0;
            shift <= 0;
            tx <= 1;
            busy <= 0;
        end else begin
            case (state)
                IDLE: begin
                    tx <= 1;
                    busy <= 0;
                    clk_count <= 0;
                    if (start) begin
                        shift <= data;
                        tx <= 0;
                        busy <= 1;
                        state <= START;
                    end
                end
                START: begin
                    busy <= 1;
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        clk_count <= 0;
                        tx <= shift[0];
                        shift <= {1'b0, shift[7:1]};
                        bit_index <= 0;
                        state <= DATA;
                    end else clk_count <= clk_count + 1'b1;
                end
                DATA: begin
                    busy <= 1;
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        clk_count <= 0;
                        if (bit_index == 3'd7) begin
                            tx <= 1;
                            state <= STOP;
                        end else begin
                            tx <= shift[0];
                            shift <= {1'b0, shift[7:1]};
                            bit_index <= bit_index + 1'b1;
                        end
                    end else clk_count <= clk_count + 1'b1;
                end
                STOP: begin
                    busy <= 1;
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        clk_count <= 0;
                        busy <= 0;
                        state <= IDLE;
                    end else clk_count <= clk_count + 1'b1;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule

module vofa_firewater_tx #(
    parameter integer CLK_HZ = 27000000,
    parameter integer BAUD = 115200
) (
    input wire clk,
    input wire rst_n,
    input wire sample_tick,
    input wire signed [15:0] angle_cdeg,
    input wire signed [15:0] position_count,
    input wire signed [15:0] velocity_counts_per_ms,
    input wire signed [15:0] motor_command,
    output wire tx
);
    localparam [1:0] IDLE = 2'd0, SEND = 2'd1, WAIT_BUSY = 2'd2, WAIT_DONE = 2'd3;
    reg [1:0] state;
    reg [4:0] char_index;
    reg [7:0] packet [0:27];
    reg [7:0] tx_data;
    reg tx_start;
    wire tx_busy;
    wire tx_line;
    integer i;
    reg [47:0] field;

    function [7:0] ascii_digit(input integer digit);
        begin ascii_digit = digit + 8'h30; end
    endfunction

    function [47:0] format_signed16(input signed [15:0] value);
        reg [15:0] magnitude;
        begin
            if (value < 0) magnitude = -value;
            else magnitude = value;
            format_signed16 = {
                value < 0 ? 8'h2d : 8'h2b,
                ascii_digit((magnitude / 10000) % 10),
                ascii_digit((magnitude / 1000) % 10),
                ascii_digit((magnitude / 100) % 10),
                ascii_digit((magnitude / 10) % 10),
                ascii_digit(magnitude % 10)
            };
        end
    endfunction

    uart_tx_byte #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) tx_i (
        .clk(clk), .rst_n(rst_n), .start(tx_start), .data(tx_data),
        .tx(tx_line), .busy(tx_busy)
    );
    assign tx = tx_line;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            char_index <= 0;
            tx_data <= 0;
            tx_start <= 0;
            for (i = 0; i < 28; i = i + 1) packet[i] <= 8'h20;
        end else begin
            tx_start <= 0;
            case (state)
                IDLE: if (sample_tick) begin
                    field = format_signed16(angle_cdeg);
                    packet[0] <= field[47:40]; packet[1] <= field[39:32];
                    packet[2] <= field[31:24]; packet[3] <= field[23:16];
                    packet[4] <= field[15:8]; packet[5] <= field[7:0];
                    packet[6] <= ",";
                    field = format_signed16(position_count);
                    packet[7] <= field[47:40]; packet[8] <= field[39:32];
                    packet[9] <= field[31:24]; packet[10] <= field[23:16];
                    packet[11] <= field[15:8]; packet[12] <= field[7:0];
                    packet[13] <= ",";
                    field = format_signed16(velocity_counts_per_ms);
                    packet[14] <= field[47:40]; packet[15] <= field[39:32];
                    packet[16] <= field[31:24]; packet[17] <= field[23:16];
                    packet[18] <= field[15:8]; packet[19] <= field[7:0];
                    packet[20] <= ",";
                    field = format_signed16(motor_command);
                    packet[21] <= field[47:40]; packet[22] <= field[39:32];
                    packet[23] <= field[31:24]; packet[24] <= field[23:16];
                    packet[25] <= field[15:8]; packet[26] <= field[7:0];
                    packet[27] <= 8'h0a;
                    char_index <= 0;
                    state <= SEND;
                end
                SEND: if (!tx_busy) begin
                    tx_data <= packet[char_index];
                    tx_start <= 1;
                    state <= WAIT_BUSY;
                end
                WAIT_BUSY: begin
                    tx_start <= 0;
                    if (tx_busy) state <= WAIT_DONE;
                end
                WAIT_DONE: if (!tx_busy) begin
                    if (char_index == 27) state <= IDLE;
                    else begin char_index <= char_index + 1'b1; state <= SEND; end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
