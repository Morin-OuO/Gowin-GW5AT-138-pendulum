// swing_kick.v -- swing-up by extreme detection + double pulse.
//
// Ported from the reference implementation (JiangXie 16-倒立摆-自动启摆):
//
//   every JUDGE_MS (40 ms) the angle is sampled three times deeper.
//   When three consecutive samples are all beyond the centre band on one side
//   AND the middle sample is the local extreme, the rod has reached the top of
//   its swing on that side. Then a DOUBLE PULSE is applied:
//
//       +A for PULSE_MS, then -A for PULSE_MS, then stop, back to judging
//
//   The double pulse is what transfers energy at the turning point; because it
//   only happens once per detected extreme, the motion is a slow, rhythmic
//   rocking instead of a high-frequency buzz.
//
//   Two consecutive samples inside the centre band hand over to the balance
//   loop.
//
// angle here is the raw 12-bit sensor value (0..4095), centre = zero band mid.
`timescale 1ns/1ps

module swing_kick #(
    parameter integer JUDGE_MS  = 40,         // sampling period for judgement
    parameter integer PULSE_MS  = 100         // duration of each half of the pulse
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire               sample_tick,    // 5 ms tick
    input  wire signed [15:0] angle,          // raw angle (0..4095)
    input  wire signed [15:0] zero,           // centre of the zero band
    input  wire       [11:0]  capture_now,    // centre band half width (counts)
    input  wire        [7:0]  pwm_now,        // pulse magnitude, 0..100
    input  wire        [7:0]  sw_lift,        // dead-band lift
    input  wire        [7:0]  sw_dir,         // 1 = normal, 0 = inverted
    // unused extra ports kept so the top-level wiring does not change
    input  wire        [7:0]  sw_gain,
    input  wire       [11:0]  fallout_now,
    input  wire       [11:0]  swang,
    input  wire signed [15:0] location,
    input  wire       [11:0]  slim,
    input  wire       [15:0]  sw_half_ms,
    input  wire signed [15:0] cart_vel,
    output reg  signed [15:0] kick,
    output reg                swinging
);

    localparam integer JUDGE_DIV = (JUDGE_MS < 5)  ? 1 : JUDGE_MS / 5;
    localparam integer PULSE_DIV = (PULSE_MS < 5)  ? 1 : PULSE_MS / 5;

    localparam [2:0] S_JUDGE   = 3'd0,
                     S_PULSE_A = 3'd1,   // +A
                     S_PULSE_B = 3'd2,   // -A
                     S_REST    = 3'd3;

    reg [2:0]  state;
    reg        en_prev;
    reg [15:0] judge_cnt;
    reg [15:0] pulse_cnt;
    reg [15:0] a0, a1, a2;              // three-deep angle history

    wire signed [17:0] centre = $signed(zero);
    wire signed [17:0] range  = $signed({6'd0, capture_now});
    wire signed [17:0] fall_r = (fallout_now > capture_now) ? $signed({6'd0, fallout_now})
                                                           : $signed({6'd0, capture_now}) + 18'sd1;
    wire signed [17:0] ang    = $signed(angle);

    wire high_side = (a0 > centre + range) && (a1 > centre + range) && (a2 > centre + range);
    wire low_side  = (a0 < centre - range) && (a1 < centre - range) && (a2 < centre - range);
    // middle sample is the local extreme
    wire max_at_mid = (a1 < a0) && (a1 < a2);      // current angle smaller than neighbours
    wire min_at_mid = (a1 > a0) && (a1 > a2);

    wire in_band0 = (a0 > centre - range) && (a0 < centre + range);
    wire in_band1 = (a1 > centre - range) && (a1 < centre + range);

    wire signed [15:0] amp = $signed({8'd0, pwm_now}) + $signed({8'd0, sw_lift});
    wire signed [15:0] pos_amp = (sw_dir != 0) ?  amp : -amp;
    wire signed [15:0] neg_amp = (sw_dir != 0) ? -amp :  amp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_JUDGE;
            judge_cnt <= 16'd0;
            pulse_cnt <= 16'd0;
            a0 <= 16'd0; a1 <= 16'd0; a2 <= 16'd0;
            kick      <= 16'sd0;
            swinging  <= 1'b1;
            en_prev   <= 1'b0;
        end else if (!enable) begin
            state     <= S_JUDGE;
            judge_cnt <= 16'd0;
            pulse_cnt <= 16'd0;
            a0 <= 16'd0; a1 <= 16'd0; a2 <= 16'd0;
            kick      <= 16'sd0;
            swinging  <= 1'b1;
        end else if (sample_tick) begin
            // the reference starts with one pulse on the key press, which is what
            // gets the rod moving out of the sensor dead zone; without it a rod
            // hanging still never produces an extremum and nothing happens.
            en_prev <= 1'b1;
            if (enable && !en_prev) begin
                state     <= S_PULSE_A;
                pulse_cnt <= 16'd0;
                kick      <= 16'sd0;
            end

            case (state)
                S_JUDGE: begin
                    kick <= 16'sd0;
                    if (judge_cnt >= JUDGE_DIV - 1) begin
                        judge_cnt <= 16'd0;
                        // shift the history and take a new sample
                        a2 <= a1;
                        a1 <= a0;
                        a0 <= angle[15:0];

                        if (high_side && max_at_mid) begin
                            state     <= S_PULSE_A;      // rod at the + extreme
                            pulse_cnt <= 16'd0;
                        end else if (low_side && min_at_mid) begin
                            // rod at the - extreme: start with the negative pulse
                            state     <= S_PULSE_B;
                            pulse_cnt <= 16'd0;
                        end else if (in_band0 && in_band1) begin
                            swinging <= 1'b0;            // hand over to the PID
                        end
                    end else begin
                        judge_cnt <= judge_cnt + 16'd1;
                    end
                end

                S_PULSE_A: begin
                    kick <= pos_amp;
                    if (pulse_cnt >= PULSE_DIV - 1) begin
                        pulse_cnt <= 16'd0;
                        state     <= S_PULSE_B;
                    end else begin
                        pulse_cnt <= pulse_cnt + 16'd1;
                    end
                end

                S_PULSE_B: begin
                    kick <= neg_amp;
                    if (pulse_cnt >= PULSE_DIV - 1) begin
                        pulse_cnt <= 16'd0;
                        state     <= S_REST;
                    end else begin
                        pulse_cnt <= pulse_cnt + 16'd1;
                    end
                end

                default: begin
                    // S_REST: stop, then go back to judging
                    kick <= 16'sd0;
                    state <= S_JUDGE;
                    judge_cnt <= 16'd0;
                    a0 <= angle[15:0];
                end
            endcase

            // falling out resumes swinging - but only past the WIDER fall-out
            // threshold, so the pump and the balance loop do not chatter at the
            // capture edge (that chattering is what dropped the rod)
            if (!swinging) begin
                if (((angle > centre + fall_r) || (angle < centre - fall_r))) begin
                    swinging  <= 1'b1;
                    state     <= S_JUDGE;
                    judge_cnt <= 16'd0;
                end
            end
        end
    end

endmodule
