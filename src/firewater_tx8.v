// firewater_tx8.v -- FireWater transmitter with eight signed 16-bit channels.
//
// Same wire format as the four-channel version (ASCII decimal, comma separated,
// '\n' terminated), just wider: 8 fields x 6 chars + 7 commas + newline = 56 B.
// At 115200 baud a frame takes ~490 us, so 100 Hz telemetry uses ~5% of the link.
`timescale 1ns/1ps

module firewater_tx8 #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_tick,
    input  wire signed [15:0] ch0,
    input  wire signed [15:0] ch1,
    input  wire signed [15:0] ch2,
    input  wire signed [15:0] ch3,
    input  wire signed [15:0] ch4,
    input  wire signed [15:0] ch5,
    input  wire signed [15:0] ch6,
    input  wire signed [15:0] ch7,
    output wire               tx
);

    localparam integer N_CH    = 8;
    localparam integer FRAME   = N_CH * 7;   // 6 chars + comma per field, last byte is '\n'

    wire signed [15:0] chans [0:N_CH-1];
    assign chans[0] = ch0;
    assign chans[1] = ch1;
    assign chans[2] = ch2;
    assign chans[3] = ch3;
    assign chans[4] = ch4;
    assign chans[5] = ch5;
    assign chans[6] = ch6;
    assign chans[7] = ch7;

    localparam [1:0] IDLE = 2'd0, SEND = 2'd1, WAIT_BUSY = 2'd2, WAIT_DONE = 2'd3;

    reg [1:0]  state;
    reg [6:0]  char_index;
    reg [7:0]  packet [0:FRAME-1];
    reg [7:0]  tx_data;
    reg        tx_start;
    wire       tx_busy;
    wire       tx_line;

    integer    i;
    reg [47:0] field;

    function [7:0] ascii_digit(input integer digit);
        begin ascii_digit = digit + 8'h30; end
    endfunction

    // +DDDDD fixed width so the host sees a stable column count
    function [47:0] format_signed16(input signed [15:0] value);
        reg [15:0] magnitude;
        begin
            if (value < 0) magnitude = -value;
            else           magnitude =  value;
            format_signed16 = {
                value < 0 ? 8'h2d : 8'h20,
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
            state      <= IDLE;
            char_index <= 0;
            tx_data    <= 0;
            tx_start   <= 0;
            for (i = 0; i < FRAME; i = i + 1) packet[i] <= 8'h20;
        end else begin
            tx_start <= 0;
            case (state)
                IDLE: if (sample_tick) begin
                    for (i = 0; i < N_CH; i = i + 1) begin
                        field = format_signed16(chans[i]);
                        packet[i*7 + 0] <= field[47:40];
                        packet[i*7 + 1] <= field[39:32];
                        packet[i*7 + 2] <= field[31:24];
                        packet[i*7 + 3] <= field[23:16];
                        packet[i*7 + 4] <= field[15:8];
                        packet[i*7 + 5] <= field[7:0];
                        if (i != N_CH - 1) packet[i*7 + 6] <= 8'h2C;   // ','
                    end
                    packet[FRAME - 1] <= 8'h0A;                        // '\n'
                    char_index <= 0;
                    state      <= SEND;
                end

                SEND: if (!tx_busy) begin
                    tx_data    <= packet[char_index];
                    tx_start   <= 1'b1;
                    state      <= WAIT_BUSY;
                end

                WAIT_BUSY: begin
                    tx_start <= 1'b0;
                    if (tx_busy) state <= WAIT_DONE;
                end

                WAIT_DONE: if (!tx_busy) begin
                    if (char_index == FRAME - 1) state <= IDLE;
                    else begin
                        char_index <= char_index + 1'b1;
                        state      <= SEND;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
