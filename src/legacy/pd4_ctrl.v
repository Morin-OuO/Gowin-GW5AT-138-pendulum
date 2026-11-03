// pd4_ctrl.v -- 4-term integer PD with circular angle handling.
//
// Rewritten after studying the two reference implementations:
//   yusendesu/gowin_fpga_inverted_pendulum   (16.16 PID, derivative LPF, swing-up)
//   Sddsafdsfsd/fpga-furuta-pendulum         (4-term PD, wrap12, dead-ff, tilt)
//
// Why the structure changed, quoting the failure each item fixes:
//
//   1. wrap12 error. The angle sensor is CIRCULAR (0..4095). Subtracting two raw
//      counts makes the error jump a whole turn whenever the rod passes the
//      0/4095 seam, which slams the output to the opposite rail. Symptoms: the
//      rod "always turns the same way" and "jumps at the 0/180 crossover".
//   2. No integral by default. An integrator on a periodic angle error winds up
//      and then drives the motor even when the error is exactly zero.
//   3. Rate from a ring buffer, not a per-tick delta. One count of quantisation
//      over 5 ms becomes 200 counts/s of fake rate; a 10..32 ms window divides
//      that by 2..6. The buffer is written even while disabled so the first
//      tick after enable cannot produce a bogus derivative.
//   4. dead_ff feed-forward. Static friction means small corrections never move
//      the motor; adding a small bias in the commanded direction fixes it.
//   5. Soft limit adds a REVERSE BIAS only, never attenuates the rod term: if
//      the rod needs correcting, dropping authority is worse than stalling.
//   6. Tilt cascade: cart offset shifts the rod's balance point instead of
//      fighting it with an additive force term (which is positive feedback).
//
// Gains are Q4.20 (gain * 1048576) in 24-bit registers. All math is multiply +
// arithmetic shift; no division inside the loop.
`timescale 1ns/1ps

module pd4_ctrl #(
    parameter integer OUTPUT_LIMIT = 100,
    parameter integer SHIFT        = 20,
    parameter integer SOFT_LO      = 400,     // |enc_pos| where the reverse bias starts
    parameter integer SOFT_REV     = 60,      // reverse bias at full deflection
    parameter integer LEAN_MAX     = 90       // max setpoint shift, angle counts
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,           // control tick, 200 Hz
    input  wire               enable,
    input  wire        [11:0] ang12,          // raw angle, 0..4095
    input  wire        [11:0] zero12,         // balance point
    input  wire signed [15:0] enc_pos,        // cart position, counts
    input  wire signed [23:0] kp_a,           // Q20, duty per count of angle error
    input  wire signed [23:0] kd_a,           // Q20, duty per count of angle rate
    input  wire signed [23:0] lean_g,         // Q20, setpoint shift per cart count
    input  wire        [10:0] dead_ff,        // duty bias against friction
    input  wire        [1:0]  rate_sel,       // 0:10 ms 1:16 ms 2:32 ms (at 5 ms tick)
    output reg  signed [15:0] cmd,
    output reg  signed [12:0] dbg_a_err,
    output reg  signed [12:0] dbg_a_rate,
    output reg  signed [15:0] dbg_cmd_raw
);

    // ---------------------------------------------------------- circular diff
    function signed [12:0] wrap12(input [11:0] a, input [11:0] b);
        reg [11:0] d;
        begin
            d = a - b;                                   // mod 4096
            wrap12 = (d >= 12'd2048) ? ($signed({1'b0, d}) - 13'sd4096)
                                     :  $signed({1'b0, d});
        end
    endfunction

    // ------------------------------------------------------- setpoint (tilt)
    wire signed [47:0] lean_prod = $signed(lean_g) * $signed(enc_pos);
    wire signed [47:0] lean_raw  = lean_prod >>> SHIFT;
    wire signed [15:0] lean = (lean_raw >  LEAN_MAX) ?  LEAN_MAX  :
                              (lean_raw < -LEAN_MAX) ? -LEAN_MAX  : lean_raw[15:0];
    wire        [11:0] target12 = zero12 + lean[11:0];

    wire signed [12:0] a_err  = wrap12(ang12, target12);

    // ------------------------------------------------------- rate ring buffer
    reg [11:0] a_hist [0:31];
    reg signed [15:0] e_hist [0:31];
    reg [4:0]  wptr;

    wire [4:0] tap = (rate_sel == 2'd0) ? (wptr - 5'd2)  :   // 2 x 5 ms  = 10 ms
                     (rate_sel == 2'd1) ? (wptr - 5'd3)  :   // 3 x 5 ms  = 15 ms
                                          (wptr - 5'd6);     // 6 x 5 ms  = 30 ms

    wire [11:0]        a_tap = a_hist[tap];
    wire signed [15:0] e_tap = e_hist[tap];

    wire signed [12:0] a_rate = wrap12(ang12, a_tap);

    // ------------------------------------------------------------- 2-term PD
    wire signed [47:0] t_p = ($signed(kp_a) * $signed(a_err))  >>> SHIFT;
    wire signed [47:0] t_d = ($signed(kd_a) * $signed(a_rate)) >>> SHIFT;
    wire signed [47:0] pd_raw = t_p + t_d;

    function signed [15:0] clamp_cmd(input signed [47:0] value);
        begin
            if (value > OUTPUT_LIMIT)       clamp_cmd = OUTPUT_LIMIT;
            else if (value < -OUTPUT_LIMIT) clamp_cmd = -OUTPUT_LIMIT;
            else                            clamp_cmd = value[15:0];
        end
    endfunction

    wire signed [15:0] pd_cmd = clamp_cmd(pd_raw);

    // ------------------------------------------------------- soft limit bias
    wire        [15:0] abs_pos = enc_pos[15] ? (~enc_pos + 16'd1) : enc_pos;
    wire        [23:0] bias_raw = (abs_pos <= SOFT_LO) ? 24'd0
                                : (({8'd0, abs_pos} - SOFT_LO) << 4);
    wire        [23:0] bias_sat = (bias_raw > SOFT_REV) ? SOFT_REV[23:0] : bias_raw;
    wire signed [15:0] bias = enc_pos[15] ?  $signed({5'd0, bias_sat[10:0]})
                                          : -$signed({5'd0, bias_sat[10:0]});

    // ------------------------------------------------- dead-band feed-forward
    wire signed [15:0] cmd_pre = pd_cmd + bias;
    wire signed [15:0] cmd_ff  = (cmd_pre > 0) ? (cmd_pre + $signed({5'd0, dead_ff})) :
                                 (cmd_pre < 0) ? (cmd_pre - $signed({5'd0, dead_ff})) :
                                                 16'sd0;

    wire signed [15:0] cmd_lim = (cmd_ff >  OUTPUT_LIMIT) ?  OUTPUT_LIMIT :
                                 (cmd_ff < -OUTPUT_LIMIT) ? -OUTPUT_LIMIT : cmd_ff;

    // ---------------------------------------------------------------- update
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr        <= 5'd0;
            cmd         <= 16'sd0;
            dbg_a_err   <= 13'sd0;
            dbg_a_rate  <= 13'sd0;
            dbg_cmd_raw <= 16'sd0;
        end else if (tick) begin
            // written every tick, enabled or not: prevents a first-tick D burst
            a_hist[wptr] <= ang12;
            e_hist[wptr] <= enc_pos;
            wptr         <= wptr + 5'd1;

            dbg_a_err   <= a_err;
            dbg_a_rate  <= a_rate;
            dbg_cmd_raw <= pd_cmd;
            cmd         <= enable ? cmd_lim : 16'sd0;
        end
    end

endmodule
