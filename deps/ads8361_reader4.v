// ads8361_reader4.v -- free-running continuous-clock ADS8361 reader.
//
// Why this shape: the previous driver stopped CLK between bursts. The ADS8361
// counts 20 clocks per conversion, so restarting the clock left the capture
// window at an arbitrary phase of that 20-clock frame -- roughly one frame in
// four landed correctly and the rest decoded as zeros. Keeping CLK free-running
// and pulsing CONVST exactly on a 20-clock boundary makes the frame phase fixed
// for good.
//
// Per datasheet:
//   * CS must be LOW to enable CONVST/RD, so CS is held low permanently.
//   * CONVST and RD are tied on the module: one pulse starts the conversion and
//     the serial read.
//   * 20 clocks per conversion. Data becomes valid after the third rising edge,
//     so with 20 samples captured from the CONVST edge the bit order is:
//        sample 0 : pre-data (ignore)
//        sample 1 : CH  flag
//        sample 2 : A/B flag
//        samples 3..18 : DB15..DB0
//        sample 19 : first trailing zero
//   * Data is valid on the falling edge -> sample in the middle of the low phase.
//   * Output is 16-bit two's complement.
//
// Mode I (M0=0, M1=0, A0=0): every conversion is Ch0, A side on SERIAL DATA A.
`timescale 1ns/1ps

module ads8361_reader4 #(
    parameter integer CLK_HZ  = 50_000_000,
    parameter integer SCLK_HZ = 100_000      // 100 kHz .. 10 MHz
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               adc_dout,
    input  wire               adc_busy,
    output reg                adc_clk,
    output reg                adc_cs_n,
    output reg                adc_convst,
    output reg                adc_m0,
    output reg                adc_m1,
    output reg                adc_a0,
    output reg signed [15:0]  data_a0,
    output reg                data_valid,
    output reg         [15:0] data_raw,
    output reg          [9:0] busy_count,
    output reg          [9:0] frame_count,
    output reg          [9:0] dout_edges,
    output reg         [19:0] frame_raw
);

    localparam integer HALF    = (CLK_HZ / (2 * SCLK_HZ) < 1) ? 1 : (CLK_HZ / (2 * SCLK_HZ));
    localparam integer SAMP_PT = HALF - 1 + (HALF / 2);   // middle of the low phase

    reg [4:0]  step;        // 0..19
    reg [31:0] cnt;         // 0 .. 2*HALF-1 inside one clock
    reg        sampled;
    reg [19:0] bits;

    reg busy_d, dout_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy_d <= 1'b0; dout_d <= 1'b0;
            busy_count <= 10'd0; dout_edges <= 10'd0;
        end else begin
            busy_d <= adc_busy;
            dout_d <= adc_dout;
            if (adc_busy && !busy_d) begin
                if (busy_count >= 10'd999) busy_count <= 10'd0;
                else                       busy_count <= busy_count + 10'd1;
            end
            if (adc_dout != dout_d) begin
                if (dout_edges >= 10'd999) dout_edges <= 10'd0;
                else                       dout_edges <= dout_edges + 10'd1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            step <= 5'd0; cnt <= 32'd0; sampled <= 1'b0; bits <= 20'd0;
            adc_clk <= 1'b1; adc_cs_n <= 1'b0; adc_convst <= 1'b0;
            adc_m0 <= 1'b0; adc_m1 <= 1'b0; adc_a0 <= 1'b0;
            data_a0 <= 16'sd0; data_valid <= 1'b0; data_raw <= 16'd0;
            frame_count <= 10'd0; frame_raw <= 20'd0;
        end else begin
            data_valid <= 1'b0;
            cnt <= cnt + 32'd1;

            // falling edge of this clock
            if (cnt == HALF - 1) adc_clk <= 1'b0;

            // sample inside the low phase
            if (!sampled && cnt >= SAMP_PT) begin
                sampled <= 1'b1;
                bits <= {bits[18:0], adc_dout};
            end

            // end of this clock: next rising edge
            if (cnt == 2 * HALF - 1) begin
                cnt     <= 32'd0;
                sampled <= 1'b0;
                adc_clk <= 1'b1;

                if (step == 5'd0) adc_convst <= 1'b1;   // start conversion + read
                if (step == 5'd1) adc_convst <= 1'b0;

                if (step == 5'd19) begin
                    step <= 5'd0;
                    // sample 1 = CH, sample 2 = A/B, samples 3..18 = data
                    if (bits[18] == 1'b0 && bits[17] == 1'b0 && bits[0] == 1'b0) begin
                        data_raw   <= bits[16:1];
                        data_a0    <= $signed(bits[16:1]);
                        data_valid <= 1'b1;
                        if (frame_count >= 10'd999) frame_count <= 10'd0;
                        else                        frame_count <= frame_count + 10'd1;
                    end
                    frame_raw <= bits;
                end else begin
                    step <= step + 5'd1;
                end
            end
        end
    end

endmodule
