// balance_ctrl.v -- cascaded angle/position controller, integer only.
//
// Rewritten from the reference structure with the two weaknesses that made the
// old version "drift at zero error" removed:
//
//   1. The position loop is OPTIONAL and OFF by default. When it is off the
//      angle setpoint is exactly `zero`, so an angle equal to the calibrated
//      zero really produces zero error and zero output.
//   2. The D term differentiates the MEASUREMENT, not the error. A change of
//      setpoint (position correction, enable, zero change) therefore cannot
//      kick the motor through the derivative.
//   3. The integrator is reset whenever the loop is disabled, and only
//      accumulates while the output is not saturated.
//
// Gains are Q4.20 (gain * 1048576) and 24 bits wide, so 1e-6 resolution.
// Everything is multiply + shift; no division in the loop.
//
//   out = Kp*e + Ki*sum(e) - Kd*(measurement delta)
//   setpoint = zero - (position loop output, only when pos_enable = 1)
`timescale 1ns/1ps

module balance_ctrl #(
    parameter integer OUTPUT_LIMIT = 100
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               angle_tick,    // inner loop, 200 Hz
    input  wire               pos_tick,      // outer loop, 20 Hz
    input  wire               enable,
    input  wire signed [15:0] angle,         // angle counts
    input  wire signed [15:0] zero_lo,       // zero band, low bound
    input  wire signed [15:0] zero_hi,       // zero band, high bound
    input  wire signed [15:0] location,      // encoder position since enable
    input  wire signed [15:0] cart_velocity, // encoder delta per control tick
    input  wire signed [23:0] kv,            // cart velocity damping (Q4.20)
    input  wire signed [23:0] kp,
    input  wire signed [23:0] ki,
    input  wire signed [23:0] kd,
    input  wire signed [23:0] pos_kp,
    input  wire signed [23:0] pos_kd,
    input  wire signed [23:0] pos_ki,
    input  wire               pos_enable,
    output reg  signed [15:0] out,
    output reg  signed [15:0] pos_out,
    output reg  signed [15:0] setpoint
);

    localparam integer SHIFT = 20;
    // Integral clamp: without it the I term grows without bound (ki=0.01 with
    // error 100 adds 1 per 5 ms tick, reaching the full output range in ~0.5 s)
    // and swamps P and D entirely - which looks like a sign error but is windup.
    localparam signed [47:0] INTEGRAL_LIMIT = 48'sd30;
    // Whole-output inversion requested on the bench: P, I, D (and the kv term)
    // all flip together, which is equivalent to reversing the drivetrain.
    localparam integer OUTPUT_INVERT = 1;

    // ------------------------------------------------------------ inner loop
    reg  signed [15:0] angle_prev;
    reg  signed [16:0] error_prev;
    reg  signed [47:0] integral;
    reg  signed [15:0] out_next;

    wire signed [17:0] measurement_delta = $signed(angle) - $signed(angle_prev);
    // Two-ended zero: inside the band the error is exactly 0, which stops the
    // integrator from winding up on a dead-band offset and absorbs mechanical
    // slop. Outside the band the error is measured from the nearest bound.
    wire signed [16:0] lo_eff  = $signed(zero_lo) - $signed(pos_out);
    wire signed [16:0] hi_eff  = $signed(zero_hi) - $signed(pos_out);
    wire signed [16:0] angle17 = $signed(angle);
    wire signed [16:0] error_now = (angle17 >= lo_eff && angle17 <= hi_eff) ? 17'sd0 :
                                   (angle17 >  hi_eff) ? (hi_eff - angle17) :
                                                          (lo_eff - angle17);

    wire signed [47:0] p_term = ($signed(kp) * $signed(error_now))          >>> SHIFT;
    wire signed [47:0] i_step = ($signed(ki) * $signed(error_now))          >>> SHIFT;
    // derivative on the ERROR (reference PID_Update: Kd*(e0-e1)). With the
    // setpoint constant this equals the old measurement derivative, but when the
    // position loop shifts the setpoint the D term reacts to it, which is what
    // helps pull the base back.
    wire signed [17:0] error_delta = error_now - error_prev;
    wire signed [47:0] d_term = ($signed(kd) * $signed(error_delta)) >>> SHIFT;
    // full-state feedback: damping the cart's own velocity as well as the
    // rod's rate is what keeps the loop from sliding away.
    wire signed [47:0] v_term = -(( $signed(kv) * $signed(cart_velocity)) >>> SHIFT);
    wire signed [47:0] raw    = p_term + $signed(integral) + d_term + v_term;

    function signed [15:0] clamp_out(input signed [47:0] value);
        begin
            if (value > OUTPUT_LIMIT)       clamp_out = OUTPUT_LIMIT;
            else if (value < -OUTPUT_LIMIT) clamp_out = -OUTPUT_LIMIT;
            else                            clamp_out = value[15:0];
        end
    endfunction

    // ------------------------------------------------------------ outer loop
    reg  signed [15:0] location_prev;
    wire signed [17:0] location_delta = $signed(location) - $signed(location_prev);
    wire signed [47:0] pos_p = ($signed(pos_kp) * (-$signed(location)))          >>> SHIFT;
    wire signed [47:0] pos_d = ($signed(pos_kd) * (-$signed(location_delta)))    >>> SHIFT;
    // integral on the position error removes the steady drift that a pure PD
    // outer loop always leaves behind
    reg  signed [47:0] pos_integral;
    wire signed [47:0] pos_i = ((pos_ki) * (-(location))) >>> SHIFT;
    wire signed [47:0] pos_int_next = pos_integral + pos_i;
    wire signed [47:0] pos_int_lim  = (pos_int_next >  48'sd100) ?  48'sd100 :
                                      (pos_int_next < -48'sd100) ? -48'sd100 : pos_int_next;
    wire signed [47:0] pos_raw = pos_p + pos_d + pos_integral;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_prev    <= 16'sd0;
            integral      <= 48'sd0;
            out           <= 16'sd0;
            out_next      <= 16'sd0;
            location_prev <= 16'sd0;
            pos_out       <= 16'sd0;
            setpoint      <= 16'sd0;
        end else if (!enable) begin
            angle_prev    <= angle;
            integral      <= 48'sd0;      // no wind-up across enable cycles
            error_prev    <= 17'sd0;
            out           <= 16'sd0;
            out_next      <= 16'sd0;
            location_prev <= location;
            pos_out       <= 16'sd0;
            setpoint      <= zero_lo;
        end else begin
            // position loop, slow
            if (pos_tick) begin
                location_prev <= location;
                if (pos_enable) begin
                    pos_integral <= pos_int_lim;
            pos_out <= clamp_out(pos_raw);
                    setpoint <= $signed(zero_lo) - clamp_out(pos_raw);
                end else begin
                    pos_out  <= 16'sd0;
                    setpoint <= $signed(zero_lo);     // readback only
                end
            end

            // angle loop, fast
            if (angle_tick) begin
                angle_prev <= angle;
                error_prev <= error_now;
                out        <= OUTPUT_INVERT ? -clamp_out(raw) : clamp_out(raw);
                if ((raw <= OUTPUT_LIMIT && raw >= -OUTPUT_LIMIT) ||
                    (raw >  OUTPUT_LIMIT && error_now < 0) ||
                    (raw < -OUTPUT_LIMIT && error_now > 0)) begin
                    if (raw > OUTPUT_LIMIT || raw < -OUTPUT_LIMIT) begin
                        integral <= integral;          // hold while saturated
                    end else begin
                        // clamp the integral: unbounded growth swamps P and D
                        if (integral + i_step >  INTEGRAL_LIMIT)      integral <=  INTEGRAL_LIMIT;
                        else if (integral + i_step < -INTEGRAL_LIMIT) integral <= -INTEGRAL_LIMIT;
                        else                                          integral <= integral + i_step;
                    end
                end
            end
        end
    end

endmodule
