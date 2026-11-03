// swing_pump.v -- continuous energy pump with a starting kick.
//
// The previous version switched the drive between +amp and -amp based on
// sign(v * cos(theta)). That is a bang-bang approximation and it reversed at the
// wrong moment, so the rod never gained energy.
//
// This version uses the continuous energy-shaping law instead:
//
//     E(θ,θ̇) = ½θ̇² + (g/l)(cosθ − 1)          (E = 0 at the upright)
//     u ∝ θ̇ · cosθ                              push so that E increases
//
// Because the drive is proportional to θ̇·cosθ it changes sign smoothly at the
// right places: at the bottom (cosθ ≈ −1) the cart is driven opposite to the rod
// velocity, near the top it reverses with the rod. No phase decision required.
//
// Stages:
//   IDLE   wait until the rod has been still for SETTLE_TICKS
//   KICK   fixed direction for KICK_TICKS (the bottom dead point has θ̇ = 0, and
//          a purely proportional law cannot leave it)
//   PUMP   continuous law, clamped to ±pwm_now, plus dead-band feed-forward
//   hand over when |θ| < CAPTURE_ANGLE and |θ̇| small enough
//
// The angle is circular: both the error and the per-tick difference are wrapped,
// otherwise crossing the 0/4095 seam looks like a whole-turn jump.
`timescale 1ns/1ps

module swing_pump #(
    parameter integer CAPTURE_ANGLE = 400,
    parameter integer CAPTURE_VEL   = 40,
    parameter integer FALL_OUT      = 700,
    parameter integer KICK_DUTY     = 40,
    parameter integer KICK_TICKS    = 20,     // 20 x 5 ms = 100 ms
    parameter integer SETTLE_TICKS  = 20,
    parameter integer SETTLE_THRESH = 3,
    parameter integer PUMP_FF       = 14,     // dead-band feed-forward
    parameter integer TRAVEL_LIMIT  = 400,
    parameter integer CENTER_BIAS   = 30,
    parameter integer GAIN          = 2,      // law gain (duty per count-of-rate)
    parameter integer POLARITY      = 0
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire               tick,            // 5 ms
    input  wire        [11:0] angle,
    input  wire        [11:0] zero,
    input  wire        [7:0]  pwm_now,         // clamp on the pump amplitude
    input  wire signed [15:0] enc_pos,
    input  wire       [11:0]  travel_lim,      // hard limit on |base travel| (counts)
    output reg  signed [15:0] kick,
    output wire signed [15:0] drive,
    output reg                swinging
);

    localparam [1:0] S_IDLE = 2'd0, S_KICK = 2'd1, S_PUMP = 2'd2;

    // ------------------------------------------------------- circular error
    function signed [12:0] wrap12(input [11:0] a, input [11:0] b);
        reg [11:0] d;
        begin
            d = a - b;
            wrap12 = (d >= 12'd2048) ? ($signed({1'b0, d}) - 13'sd4096)
                                     :  $signed({1'b0, d});
        end
    endfunction

    wire signed [12:0] e     = wrap12(angle, zero);
    wire        [12:0] e_abs = e[12] ? (-e) : e;

    // cos(theta) in Q8, linear in |e|: +256 at the upright, 0 at horizontal,
    // -256 at the bottom. |e| runs 0..2048 counts over 0..180 degrees, so a
    // count is 1/4 of a Q8 LSB.
    wire        [12:0] e_lo    = (e_abs < 13'd1024) ? e_abs : 13'd1024;
    wire        [12:0] e_hi    = (e_abs < 13'd1024) ? 13'd0  : (e_abs - 13'd1024);
    wire        [8:0]  cos_pos = 9'd256 - {1'b0, e_lo[9:2]};   // 256 .. 0
    wire        [8:0]  cos_neg = {1'b0, e_hi[9:2]};            // 0 .. 256
    wire signed [10:0] cos_q8  = $signed({2'b00, cos_pos}) - $signed({2'b00, cos_neg});

    // ------------------------------------------------------- velocity
    reg  signed [12:0] e_prev;
    reg  signed [14:0] v_filt;
    wire signed [13:0] v_raw = e - e_prev;
    wire signed [14:0] v_ext = {{1{v_raw[13]}}, v_raw};
    wire signed [14:0] v_abs = v_filt[14] ? (-v_filt) : v_filt;

    // ------------------------------------------------------- state machine
    reg [1:0]  state;
    reg [15:0] still_cnt;
    reg [15:0] kick_cnt;
    reg [2:0]  cap_cnt;

    wire still  = (v_abs <= SETTLE_THRESH);
    wire at_top = (e_abs < CAPTURE_ANGLE) && (v_abs <= CAPTURE_VEL);

    // ------------------------------------------------------- continuous law
    // pump = GAIN * v * cos(theta) / 256
    wire signed [31:0] law_full = $signed(v_filt) * $signed(cos_q8) * GAIN;
    wire signed [31:0] law_q    = law_full >>> 8;
    wire signed [15:0] law_raw  = law_q[15:0];
    wire signed [15:0] law_pol  = POLARITY ? -law_raw : law_raw;
    wire signed [15:0] law_lim  = (law_pol >  $signed({8'd0, pwm_now})) ?  $signed({8'd0, pwm_now}) :
                                  (law_pol < -$signed({8'd0, pwm_now})) ? -$signed({8'd0, pwm_now}) :
                                   law_pol;

    // cart centring / hard travel limit
    wire        [15:0] abs_cart = enc_pos[15] ? (~enc_pos + 16'd1) : enc_pos;
    wire               cart_far  = (abs_cart > {4'd0, travel_lim});
    wire               cart_over = (abs_cart > {4'd0, travel_lim} + 16'd48);
    wire signed [15:0] center_bias = !cart_far ? 16'sd0 :
                                     (enc_pos[15] ? $signed({5'd0, CENTER_BIAS[10:0]})
                                                  : -$signed({5'd0, CENTER_BIAS[10:0]}));

    // the law only acts while pumping; the kick only while kicking
    wire signed [15:0] law_gated = (state == S_PUMP) ? law_lim : 16'sd0;
    // once the base exceeds the limit the pump law is overridden by a full
    // pull back toward the start, so the total rotation stays bounded.
    wire signed [15:0] recall = enc_pos[15] ? $signed({8'd0, pwm_now})
                                            : -$signed({8'd0, pwm_now});
    wire signed [15:0] drive_raw = cart_over ? recall
                                             : (kick + law_gated + center_bias);
    assign drive = (drive_raw > 16'sd0) ? (drive_raw + PUMP_FF) :
                   (drive_raw < 16'sd0) ? (drive_raw - PUMP_FF) : 16'sd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            swinging  <= 1'b1;
            kick      <= 16'sd0;
            e_prev    <= 13'sd0;
            v_filt    <= 15'sd0;
            still_cnt <= 16'd0;
            kick_cnt  <= 16'd0;
            cap_cnt   <= 3'd0;
        end else if (!enable) begin
            state     <= S_IDLE;
            swinging  <= 1'b1;
            kick      <= 16'sd0;
            e_prev    <= e;
            v_filt    <= 15'sd0;
            still_cnt <= 16'd0;
            kick_cnt  <= 16'd0;
            cap_cnt   <= 3'd0;
        end else if (tick) begin
            e_prev <= e;
            v_filt <= v_filt + ((v_ext - v_filt) >>> 1);

            if (still) begin
                if (still_cnt < SETTLE_TICKS) still_cnt <= still_cnt + 16'd1;
            end else begin
                still_cnt <= 16'd0;
            end

            case (state)
                S_IDLE: begin
                    kick <= 16'sd0;
                    if (still_cnt >= SETTLE_TICKS) begin
                        kick_cnt <= 16'd0;
                        state    <= S_KICK;
                    end
                end

                S_KICK: begin
                    kick     <= POLARITY ? $signed({8'd0, KICK_DUTY[7:0]})
                                         : -$signed({8'd0, KICK_DUTY[7:0]});
                    kick_cnt <= kick_cnt + 16'd1;
                    if (kick_cnt >= KICK_TICKS) begin
                        kick  <= 16'sd0;
                        state <= S_PUMP;
                    end
                end

                S_PUMP: begin
                    kick <= 16'sd0;
                    if (at_top) begin
                        if (cap_cnt >= 3'd1) begin
                            swinging  <= 1'b0;
                            state     <= S_IDLE;
                            cap_cnt   <= 3'd0;
                            still_cnt <= 16'd0;
                        end else begin
                            cap_cnt <= cap_cnt + 3'd1;
                        end
                    end else begin
                        cap_cnt <= 3'd0;
                    end
                end

                default: state <= S_IDLE;
            endcase

            if (!swinging && e_abs > FALL_OUT) begin
                swinging  <= 1'b1;
                state     <= S_IDLE;
                still_cnt <= 16'd0;
            end
        end
    end

endmodule
