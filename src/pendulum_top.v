// pendulum_top.v -- BX72 (GW5AT-138) inverted pendulum controller.
//
// Ported from the teaching source (15-倒立摆 / 16-倒立摆-自动启摆), preserving its
// structure and units so the published gains carry over directly:
//
//   * control variable is the raw angle-sensor count (12-bit, 0..4095),
//     not degrees -- exactly like AD_GetValue() in the reference
//   * cascade: position loop (50 ms) feeds the angle loop setpoint,
//     angle loop (5 ms) drives the motor
//   * motor command is -100..+100, the same numbering as Motor_SetPWM()
//
// Reference values, kept as defaults:
//   CENTER_ANGLE 2010 (here 2024, measured on this rig: raw 8096 >> 2)
//   CENTER_RANGE 500
//   Angle  PID: Kp 0.3  Ki 0.01 Kd 0.4   -> out +-100
//   Location PID: Kp 0.4 Ki 0    Kd 4    -> out +-100
//
// The angle sensor now comes from the ADS8361 instead of the STM32 ADC:
//   raw16 / 4 = 12-bit count (the sensor spans 0..1.25 V = 0..16384 raw)
//
// Safety: the controller starts DISABLED. Send the ASCII character 'E' over the
// tuning UART to toggle enable. If the pendulum leaves the centre band the
// controller drops out and the motor is cut, mirroring the reference behaviour.
`timescale 1ns/1ps

module pendulum_top #(
    parameter integer CLK_HZ       = 50_000_000,
    parameter integer BAUD         = 115200,
    parameter integer SCLK_HZ      = 100_000,
    parameter integer ANGLE_HZ     = 200,
    parameter integer POS_HZ       = 20,
    parameter integer TELEMETRY_HZ = 100,
    parameter integer PWM_HZ       = 20_000,
    parameter integer CENTER_COUNT = 2039,   // bench zero band centre
    parameter integer CENTER_RANGE = 500,
    parameter integer MOTOR_SCALE  = 10,   // b941105 setting: command 100 = 40 % duty
    parameter integer MOTOR_INVERT = 1,       // 1: negate the motor command
    parameter integer SWING_HZ        = 1,    // pumping frequency
    parameter integer SWING_PWM       = 35,   // reference START_PWM (duty %)
    parameter integer TRAVEL_LIMIT    = 900,  // |cart travel| that triggers centring bias
    parameter integer SWING_CAPTURE   = 500,  // reference CENTER_RANGE
    parameter integer SWING_FALL_OUT  = 550,  // resume pumping beyond this
    parameter integer SPEED_LIMIT     = 25,   // hard ban: |encoder velocity| per 10 ms
    parameter integer SPEED_RESUME    = 18,   // resume below this (hysteresis)
    parameter integer USE_SWING        = 1,   // initial-version pump enabled
    parameter integer USE_SPEED_GUARD  = 0,   // 1: enable the speed ban
    parameter integer USE_BAND_SAFETY = 0,    // 1: auto-cut outside the centre band
    parameter integer USE_SOFT_START  = 0,    // 1: ramp the motor command in
    // Gains are entered as plain integers in milli units: 300 means 0.300.
    // KP_Q12 etc. convert them once at elaboration into Q4.12 for the PID.
    parameter integer KP_MILLI     = 600,    // 0.600 (optimised)
    parameter integer KI_MILLI     = 41,     // 0.041
    parameter integer KD_MILLI     = 2000,   // 2.000 (optimised)
    parameter integer POS_KP_MILLI = 200,    // b941105 setting
    parameter integer POS_KD_MILLI = 2000    // b941105 setting
) (
    input  wire clk,
    input  wire key2,          // V17 KEY2 button (pressed = 0): start / stop

    // ADS8361 (angle sensor)
    input  wire adc_dout,
    input  wire adc_busy,
    output wire adc_clk,
    output wire adc_cs_n,
    output wire adc_convst,
    output wire adc_m0,
    output wire adc_m1,
    output wire adc_a0,

    // motor driver (TB6612: PWMA + AIN1 + AIN2 + STBY)
    output wire motor_pwm,
    output wire motor_in1,
    output wire motor_in2,
    output wire motor_stby,

    // quadrature encoder
    input  wire encoder_a,
    input  wire encoder_b,

    // host link (tuning + FireWater telemetry)
    input  wire uart_rx,
    output wire uart_tx
);
    // ---------------------------------------------------------- power-on reset
    // KEY2 (V17) is no longer wired as a reset: it is the start/stop button.
    // The design generates its own reset for the first ~21 ms after power-up.
    reg [19:0] por_cnt;
    always @(posedge clk) begin
        if (por_cnt != 20'hFFFFF) por_cnt <= por_cnt + 20'd1;
    end
    wire rst_n = por_cnt[19];

    // ------------------------------------------------- KEY2 start/stop button
    // PULL_UP on V17: released = 1, pressed = 0. Debounced for ~10 ms, and only
    // the press edge is used, so one press toggles the controller.
    reg key_meta, key_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_meta <= 1'b1;
            key_sync <= 1'b1;
        end else begin
            key_meta <= key2;
            key_sync <= key_meta;
        end
    end

    reg [19:0] db_cnt;
    reg        key_state;        // debounced level, 1 = released
    reg        key_press;        // one-clock pulse on the press edge
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            db_cnt    <= 20'd0;
            key_state <= 1'b1;
            key_press <= 1'b0;
        end else begin
            key_press <= 1'b0;
            if (key_sync != key_state) begin
                db_cnt <= db_cnt + 20'd1;
                if (db_cnt >= 20'h7FFFF) begin      // 20 ms debounce
                    db_cnt    <= 20'd0;
                    key_state <= key_sync;
                    if (!key_sync) key_press <= 1'b1;
                end
            end else begin
                db_cnt <= 20'd0;
            end
        end
    end

    // milli -> Q4.12 conversion happens once, at elaboration
    localparam [23:0] POS_KP_Q20 = (POS_KP_MILLI * 64'd1048576) / 64'd1000;
    localparam [23:0] POS_KD_Q20 = (POS_KD_MILLI * 64'd1048576) / 64'd1000;

    // ------------------------------------------------------- ADS8361 front end
    wire signed [15:0] adc_code;
    wire               adc_valid;
    wire        [15:0] adc_raw;
    wire         [9:0] adc_busy_count, adc_frame_count, adc_dout_edges;
    wire        [19:0] adc_frame_raw;

    ads8361_reader4 #(
        .CLK_HZ(CLK_HZ), .SCLK_HZ(SCLK_HZ)
    ) adc_i (
        .clk(clk), .rst_n(rst_n),
        .adc_dout(adc_dout), .adc_busy(adc_busy),
        .adc_clk(adc_clk), .adc_cs_n(adc_cs_n), .adc_convst(adc_convst),
        .adc_m0(adc_m0), .adc_m1(adc_m1), .adc_a0(adc_a0),
        .data_a0(adc_code), .data_valid(adc_valid), .data_raw(adc_raw),
        .busy_count(adc_busy_count), .frame_count(adc_frame_count),
        .dout_edges(adc_dout_edges), .frame_raw(adc_frame_raw)
    );

    // 16-bit code -> 12-bit count, matching AD_GetValue() resolution
    reg  [11:0] angle_count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) angle_count <= 12'd0;
        else if (adc_valid) angle_count <= adc_code[15:2];
    end

    // ------------------------------------------------------------- time bases
    localparam integer ANGLE_DIV = CLK_HZ / ANGLE_HZ;
    localparam integer POS_DIV   = CLK_HZ / POS_HZ;
    localparam integer TEL_DIV   = CLK_HZ / TELEMETRY_HZ;

    reg [31:0] angle_cnt, pos_cnt, tel_cnt;
    reg        angle_tick, pos_tick, tel_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_cnt <= 0; angle_tick <= 0;
            pos_cnt   <= 0; pos_tick   <= 0;
            tel_cnt   <= 0; tel_tick   <= 0;
        end else begin
            if (angle_cnt >= ANGLE_DIV - 1) begin angle_cnt <= 0; angle_tick <= 1'b1; end
            else begin angle_cnt <= angle_cnt + 32'd1; angle_tick <= 1'b0; end
            if (pos_cnt >= POS_DIV - 1) begin pos_cnt <= 0; pos_tick <= 1'b1; end
            else begin pos_cnt <= pos_cnt + 32'd1; pos_tick <= 1'b0; end
            if (tel_cnt >= TEL_DIV - 1) begin tel_cnt <= 0; tel_tick <= 1'b1; end
            else begin tel_cnt <= tel_cnt + 32'd1; tel_tick <= 1'b0; end
        end
    end

    // ---------------------------------------------------------- enable control
    wire [7:0] rx_data;
    wire       rx_valid, rx_frame_error;

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) uart_i (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx),
        .data(rx_data), .valid(rx_valid), .framing_error(rx_frame_error)
    );

    // Live tuning: kp=300 ki=10 kd=400 zero=2024 dir=1 en=1  (gains in milli)
    wire signed [23:0] kp_q20, ki_q20, kd_q20;
    wire signed [23:0] kp_micro, ki_micro, kd_micro;
    wire        [11:0] zero_lo, zero_hi;
    wire        [11:0] sw_limit;
    wire         [7:0] tel_sel;
    wire         [7:0] enc_invert;
    wire signed [15:0] manual_cmd;
    wire               en_req, en_update, cmd_ok, motor_dir, manual_update;
    wire               pos_loop_en;
    wire         [7:0] swing_pwm;
    wire        [15:0] swing_half_ms;
    wire         [7:0] deadband_pct, slew_limit;
    wire         [7:0] max_cmd, max_vel;
    wire signed [23:0] pos_kp_q20, pos_kd_q20, kv_q20;

    uart_cmd_parser #(
        .ZERO_DEFAULT(CENTER_COUNT),
        .KP_MILLI(KP_MILLI),
        .KI_MILLI(KI_MILLI),
        .KD_MILLI(KD_MILLI)
    ) cmd_i (
        .clk(clk), .rst_n(rst_n),
        .rx_data(rx_data), .rx_valid(rx_valid),
        .kp_q20(kp_q20), .ki_q20(ki_q20), .kd_q20(kd_q20),
        .kp_micro(kp_micro), .ki_micro(ki_micro), .kd_micro(kd_micro),
        .zero_lo(zero_lo), .zero_hi(zero_hi), .sw_limit(sw_limit), .tel_sel(tel_sel),
        .enable_req(en_req), .en_update(en_update),
        .dir(motor_dir), .pos_enable(pos_loop_en),
        .swing_pwm(swing_pwm), .swing_half_ms(swing_half_ms),
        .manual_cmd(manual_cmd),
        .pos_kp_q20(pos_kp_q20), .pos_kd_q20(pos_kd_q20), .pos_ki_q20(pos_ki_q20),
        .kv_q20(kv_q20),
        .deadband_pct(deadband_pct), .slew_limit(slew_limit),
        .max_cmd(max_cmd), .max_vel(max_vel),
        .manual_update(manual_update), .cmd_ok(cmd_ok)
    );

    // Band check and auto-cut. Both are bypassed while USE_BAND_SAFETY = 0, so
    // the loop runs wherever the pendulum is and only `en=0` stops it.
    wire in_band = (USE_BAND_SAFETY == 0) ? 1'b1 :
                   ((angle_count > (zero_lo - CENTER_RANGE)) &&
                    (angle_count < (zero_hi + CENTER_RANGE)));

    reg en_cmd;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_cmd <= 1'b0;
        end else begin
            if (en_update)     en_cmd <= en_req && in_band;
            if (key_press)     en_cmd <= ~en_cmd && in_band;   // KEY2: start / stop
            if (USE_BAND_SAFETY == 1 && en_cmd && !in_band)
                en_cmd <= 1'b0;      // pendulum fell over: cut the motor
        end
    end

    wire enabled = en_cmd && in_band;

    // ------------------------------------------------------------- encoder
    wire signed [31:0] enc_count;
    quadrature_encoder enc_i (
        .clk(clk), .rst_n(rst_n),
        .encoder_a(encoder_a), .encoder_b(encoder_b),
        .count(enc_count)
    );

    reg  signed [31:0] enc_prev;
    reg  signed [15:0] enc_velocity;
    wire signed [31:0] enc_delta = enc_count - enc_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enc_prev <= 0;
            enc_velocity <= 0;
        end else if (tel_tick) begin
            enc_prev <= enc_count;
            enc_velocity <= (enc_delta > 32767)  ? 16'sd32767 :
                            (enc_delta < -32768) ? -16'sd32768 :
                            enc_delta[15:0];
        end
    end

    // ------------------------------------------------------- enable edges
    // On the rising edge of `enabled`, capture the encoder count so the position
    // loop starts from zero (the reference does `Location = 0` here), and ramp
    // the motor command in over ~160 ms so enabling cannot kick the cart.
    reg               en_prev;
    reg  signed [31:0] enc_offset;
    reg         [5:0]  soft;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_prev    <= 1'b0;
            enc_offset <= 32'sd0;
            soft       <= 6'd0;
        end else begin
            en_prev <= enabled;
            if (enabled && !en_prev) enc_offset <= enc_count;
            // The reference resets Location = 0 right when the balance loop takes
            // over, so the position loop closes on the base position at CAPTURE,
            // not on the position at enable (which can be far away after the
            // swing-up walked the base around).
            else if (swinging_prev && !swinging) enc_offset <= enc_count;
            if (USE_SOFT_START == 0) begin
                soft <= 6'd32;                       // full authority immediately
            end else begin
                if (!enabled)                  soft <= 6'd0;
                else if (angle_tick && soft < 6'd32) soft <= soft + 6'd1;
            end
        end
    end

    // ---------------------------------------------------------- control loops
    // Rewritten controller: the D term differentiates the measurement, the
    // integrator resets on disable, and the position loop is optional so the
    // angle loop can be brought up on its own.
    wire signed [15:0] angle_out;
    wire signed [15:0] pos_out;
    wire signed [15:0] angle_target;
    // the reference resets Location to 0 when the balance loop takes over, so the
    // encoder offset is re-captured on the falling edge of swinging
    reg  swinging_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) swinging_prev <= 1'b0;
        else        swinging_prev <= swinging;
    end

    wire signed [31:0] location32 = enc_count - enc_offset;
    wire signed [15:0] location = enc_invert[0] ? -location32[15:0] : location32[15:0];
    wire signed [15:0] angle_measure = {4'd0, angle_count};

    // cart velocity over one control tick, for the kv damping term
    reg  signed [31:0] enc_prev_tick;
    reg  signed [15:0] cart_vel_tick;
    wire signed [31:0] cart_vel_delta = enc_count - enc_prev_tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enc_prev_tick <= 32'sd0;
            cart_vel_tick <= 16'sd0;
        end else if (angle_tick) begin
            enc_prev_tick <= enc_count;
            cart_vel_tick <= (cart_vel_delta >  32767) ?  16'sd32767 :
                             (cart_vel_delta < -32768) ? -16'sd32768 :
                             cart_vel_delta[15:0];
        end
    end

    // first-order filter on the cart velocity: the 5 ms encoder delta is a small
    // integer, and kv multiplies it, so unfiltered it injects output chatter.
    reg  signed [15:0] cart_vel_f;
    wire signed [16:0] cart_vel_err = cart_vel_tick - cart_vel_f;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cart_vel_f <= 16'sd0;
        else if (angle_tick) cart_vel_f <= cart_vel_f + (cart_vel_err >>> 2);
    end

    // ------------------------------------------------ swing-up (energy pump)
    wire signed [15:0] swing_out;
    wire               swinging;
    wire        [11:0] zero_center = (zero_lo + zero_hi) >> 1;
    wire signed [15:0] angle_zero   = ({4'd0, zero_center});
    wire        [7:0]  sw_dir      = 8'd1;
    wire        [7:0]  sw_gain     = 8'd1;   // energy-law gain
    wire        [7:0]  swinging_lift = 8'd20;

    // swing_kick: one bounded pulse per half swing, with a hard travel limit
    // so the base never turns more than about one motor revolution.
    swing_kick swing_i (
        .clk(clk), .rst_n(rst_n), .enable(enabled && USE_SWING != 0),
        .sample_tick(angle_tick),
        .angle(angle_measure), .zero(angle_zero),
        .pwm_now(swing_pwm), .sw_lift(swing_lift), .sw_dir(sw_dir), .sw_gain(sw_gain), .sw_half_ms(swing_half_ms), .cart_vel(enc_velocity),
        .capture_now(SWING_CAPTURE), .fallout_now(SWING_FALL_OUT),
        .swang(SWING_CAPTURE), .slim(sw_limit),
        .location(location),
        .kick(swing_out), .swinging(swinging)
    );

    wire balance_en = enabled && (!USE_SWING || !swinging);

    balance_ctrl #(.OUTPUT_LIMIT(100)) ctrl_i (
        .clk(clk), .rst_n(rst_n),
        .angle_tick(angle_tick), .pos_tick(pos_tick), .enable(balance_en),
        .angle(angle_measure),
        .zero_lo($signed({4'd0, zero_lo})),
        .zero_hi($signed({4'd0, zero_hi})),
        .location(location),
        .cart_velocity(cart_vel_tick),
        .kv(kv_q20),
        .kp(kp_q20), .ki(ki_q20), .kd(kd_q20),
        .pos_kp(pos_kp_q20), .pos_kd(pos_kd_q20), .pos_ki(pos_ki_q20),
        .pos_enable(pos_loop_en),
        .out(angle_out), .pos_out(pos_out), .setpoint(angle_target)
    );

    // ------------------------------------------- speed guard (hard ban)
    // |encoder velocity| must never exceed SPEED_LIMIT counts per 10 ms. This is
    // enforced in hardware: once tripped the drive is disabled until the speed
    // falls back below SPEED_RESUME.
    reg  signed [15:0] vel_hold;
    reg                speed_trip;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            speed_trip <= 1'b0;
            vel_hold   <= 16'sd0;
        end else begin
            if (tel_tick) vel_hold <= enc_velocity;
            if (vel_hold > $signed({8'd0, max_vel}) || vel_hold < -$signed({8'd0, max_vel}))
                speed_trip <= 1'b1;
            else if (vel_hold < SPEED_RESUME && vel_hold > -SPEED_RESUME)
                speed_trip <= 1'b0;
        end
    end

    wire drive_enable = (USE_SPEED_GUARD != 0) ? (enabled && !speed_trip) : enabled;

    // ------------------------------------------- manual override and direction
    // out=<n> bypasses the PID so the drivetrain can be tested directly.
    reg manual_en;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)             manual_en <= 1'b0;
        else if (manual_update) manual_en <= (manual_cmd != 16'sd0);
        else if (!enabled)      manual_en <= 1'b0;
    end

    // soft start: scale the command in over ~160 ms when USE_SOFT_START is set
    wire signed [47:0] ramp_product = $signed(angle_out) * $signed({1'b0, soft});
    wire signed [15:0] angle_out_ramped = ramp_product[20:5];

    // The swing-up needs the full motor authority (MOTOR_SCALE = 25, i.e. command
    // 100 = 100 % duty), but the balance loop was tuned when 100 meant 40 % duty.
    // Scale ONLY the balance path back by 0.4 (102/256) so the upright behaviour
    // stays exactly as it was tuned.
    wire signed [31:0] bal_scaled32 = (angle_out_ramped) * 32'sd102;
    wire signed [15:0] bal_scaled   = bal_scaled32[23:8];
    wire signed [15:0] auto_cmd   = (USE_SWING != 0 && swinging) ? swing_out : bal_scaled;
    wire signed [15:0] cmd_source = manual_en ? manual_cmd : auto_cmd;
    wire signed [15:0] motor_raw  = motor_dir ? -cmd_source : cmd_source;

    // No slew limiting, no velocity filtering: this loop is delay-critical, and
    // even a one-tick lag (measured) drops the balance duty from ~90% to ~9%.

    // runtime authority cap: mlim= clamps the motor command magnitude
    wire signed [15:0] motor_lim = (motor_raw >  $signed({8'd0, max_cmd})) ?  $signed({8'd0, max_cmd}) :
                                   (motor_raw < -$signed({8'd0, max_cmd})) ? -$signed({8'd0, max_cmd}) :
                                   motor_raw;

    wire signed [15:0] motor_cmd = motor_lim;

    // ---------------------------------------------------------------- motor
    motor_driver #(
        .CLK_HZ(CLK_HZ), .PWM_HZ(PWM_HZ), .DUTY_SCALE(MOTOR_SCALE)
    ) motor_i (
        .clk(clk), .rst_n(rst_n), .enable(drive_enable), .command(motor_cmd),
        .deadband_pct(deadband_pct),
        .pwm(motor_pwm), .in1(motor_in1), .in2(motor_in2)
    );

    // TB6612 standby: HIGH enables the H-bridge, LOW puts it in standby.
    // Tied to `enabled`, so a fault or a disabled controller really cuts drive.
    assign motor_stby = drive_enable;

    // gain read-back for the host: x10000 so four decimals are visible
    wire signed [23:0] kp_tele = kp_micro / 24'sd100;
    wire signed [23:0] ki_tele = ki_micro / 24'sd100;
    wire signed [23:0] kd_tele = kd_micro / 24'sd100;

    // ------------------------------------------------------------ telemetry
    // 8 channels, 100 Hz:
    //   ch0 angle count      ch1 angle PID output   ch2 encoder velocity
    //   ch3 zero count       ch4 kp x1000           ch5 ki x1000
    //   ch6 kd x1000         ch7 status (en | dir<<1 | band<<2)
    // Gains are forwarded from the parser in milli units - no scaling here,

    // status: bit0 enabled, bit1 dir, bit2 in_band, bit3 speed trip, bit4 pos loop on
    // status: bit0 enabled, bit1 dir, bit2 in_band, bit3 speed trip,
    // bit4 swinging (pump active; 0 = PID balance mode)
    wire signed [15:0] status   = {11'd0, swinging, speed_trip, in_band, motor_dir, enabled};

    // ------------------------------------------------ parameter readback (tel=1)
    // milli value = (q20 * 125) >> 17  (because 1000/1048576 = 125/131072)
    wire signed [31:0] kp_milli_x  = ($signed(kp_q20)      * 32'sd125) >>> 17;
    wire signed [31:0] pkp_milli_x = ($signed(pos_kp_q20)  * 32'sd125) >>> 17;
    wire signed [31:0] pkd_milli_x = ($signed(pos_kd_q20)  * 32'sd125) >>> 17;
    wire [15:0] rb0 = kp_milli_x[15:0];
    wire [15:0] rb1 = {4'd0, zero_lo};
    wire [15:0] rb2 = {4'd0, zero_hi};
    wire [15:0] rb3 = pkp_milli_x[15:0];
    wire [15:0] rb4 = pkd_milli_x[15:0];
    wire [15:0] rb5 = {8'd0, swing_pwm};
    wire [15:0] rb6 = {max_cmd, max_vel};
    wire [15:0] rb7 = {12'd0, pos_loop_en, enc_invert[0], motor_dir, enabled};
    firewater_tx8 #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) vofa_i (
        .clk(clk), .rst_n(rst_n), .sample_tick(tel_tick),
        .ch0(tel_sel[0] ? rb0 : {4'd0, angle_count}),
        .ch1(tel_sel[0] ? rb1 : motor_cmd),        // tel=1: zero_lo
        .ch2(tel_sel[0] ? rb2 : enc_velocity),     // tel=1: zero_hi
        .ch3(tel_sel[0] ? rb3 : {4'd0, zero_lo}),  // tel=1: pkp
        .ch4(tel_sel[0] ? rb4 : location),         // tel=1: pkd
        .ch5(tel_sel[0] ? rb5 : ki_tele[15:0]),    // tel=1: swp
        .ch6(tel_sel[0] ? rb6 : kd_tele[15:0]),    // tel=1: {mlim,vlim}
        .ch7(tel_sel[0] ? rb7 : status),           // tel=1: {pos,enc,dir,en}
        .tx(uart_tx)
    );

endmodule
