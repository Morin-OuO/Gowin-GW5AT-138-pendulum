// uart_cmd_parser.v -- ASCII tuning commands, integer in / integer out.
//
// Gains: an integer means MILLI units, a decimal gives full resolution:
//     kp=300          -> 0.300
//     kp=0.300500     -> 0.300500      (6 fraction digits accepted)
//     ki=10           -> 0.010
//     ki=0.010500     -> 0.010500
//     kd=400          -> 0.400
//
// Internally a gain is stored as Q4.20 (gain * 1048576) in a 24-bit register,
// so the control loop only multiplies and shifts - never divides.  Resolution
// is 1e-6 of a unit, and the only division happens once per command when the
// decimal text is converted.
//
// Other commands:
//     zero=2024   mechanical zero in 12-bit angle counts
//     dir=1       invert the motor command (0 = leave alone)
//     en=1 / en=0 enable / disable
//     out=<n>     manual motor command (-100..100), bypasses the PID
`timescale 1ns/1ps

module uart_cmd_parser #(
    parameter integer ZERO_DEFAULT = 2039,   // zero band centre (bench)
    parameter integer KP_MILLI     = 600,    // 0.600 (optimised)
    parameter integer KI_MILLI     = 41,     // 0.041
    parameter integer KD_MILLI     = 2000    // 2.000 (optimised)
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire        [7:0]  rx_data,
    input  wire               rx_valid,
    output reg  signed [23:0] kp_q20,
    output reg  signed [23:0] ki_q20,
    output reg  signed [23:0] kd_q20,
    output reg  signed [23:0] kp_micro,     // gain * 1e6, for telemetry
    output reg  signed [23:0] ki_micro,
    output reg  signed [23:0] kd_micro,
    output reg         [11:0] zero_lo,       // zero band, low bound
    output reg         [11:0] zero_hi,       // zero band, high bound
    output reg  signed [15:0] manual_cmd,
    output reg                manual_update,
    output reg  signed [23:0] kv_q20,          // cart velocity damping
    output reg  signed [23:0] pos_ki_q20,      // position loop integral
    output reg  signed [23:0] pos_kp_q20,      // position loop gains
    output reg  signed [23:0] pos_kd_q20,
    output reg          [7:0] tel_sel,         // 0 = live telemetry, 1 = parameter readback
    output reg          [7:0] max_cmd,         // |motor command| cap (0..100)
    output reg          [7:0] max_vel,         // speed guard limit (0..60)
    output reg          [7:0] slew_limit,      // max command change per control tick
    output reg          [7:0] enc_invert,      // 1 = negate the encoder counting
    output reg          [7:0] deadband_pct,    // friction compensation
    output reg          [7:0] swing_pwm,
    output reg         [15:0] swing_half_ms,
    output reg         [11:0] sw_limit,        // hard limit on base travel (counts, 512 = one rev)
    output reg                pos_enable,      // 1 = position loop on
    output reg                dir,
    output reg                enable_req,
    output reg                en_update,
    output reg                cmd_ok
);

    localparam [31:0] K_KP   = "kp";
    localparam [31:0] K_KI   = "ki";
    localparam [31:0] K_KD   = "kd";
    localparam [31:0] K_ZERO = "zero";
    localparam [31:0] K_ZL   = "zl";
    localparam [31:0] K_ZH   = "zh";
    localparam [31:0] K_EN   = "en";
    localparam [31:0] K_DIR  = "dir";
    localparam [31:0] K_OUT  = "out";
    localparam [31:0] K_POS  = "pos";
    localparam [31:0] K_ENC  = "enc";
    localparam [31:0] K_SWP  = "swp";
    localparam [31:0] K_SWF  = "swf";
    localparam [31:0] K_SLIM = "slim";
    localparam [31:0] K_PKI  = "pki";
    localparam [31:0] K_PKP  = "pkp";
    localparam [31:0] K_KV   = "kv";
    localparam [31:0] K_SLEW = "slew";
    localparam [31:0] K_DB   = "db";
    localparam [31:0] K_MLIM = "mlim";
    localparam [31:0] K_VLIM = "vlim";
    localparam [31:0] K_PKD  = "pkd";
    localparam [31:0] K_TEL  = "tel";

    reg [31:0] key;
    reg [2:0]  key_len;
    reg        seen_eq;

    reg        val_neg;
    reg [15:0] val_int;
    reg [23:0] val_frac;
    reg [2:0]  frac_digits;
    reg        seen_dot;

    wire [7:0] ch_lc = (rx_data >= 8'h41 && rx_data <= 8'h5A)
                       ? (rx_data + 8'h20) : rx_data;

    function key_is(input [31:0] k, input [2:0] len, input [31:0] want, input [2:0] want_len);
        key_is = (len == want_len) && (k == want);
    endfunction

    // fraction scaled to millionths
    reg [31:0] frac_micro;
    always @(*) begin
        case (frac_digits)
            3'd1:    frac_micro = (val_frac * 32'd1000000) / 32'd10;
            3'd2:    frac_micro = (val_frac * 32'd1000000) / 32'd100;
            3'd3:    frac_micro = (val_frac * 32'd1000000) / 32'd1000;
            3'd4:    frac_micro = (val_frac * 32'd1000000) / 32'd10000;
            3'd5:    frac_micro = (val_frac * 32'd1000000) / 32'd100000;
            3'd6:    frac_micro = (val_frac * 32'd1000000) / 32'd1000000;
            default: frac_micro = 32'd0;
        endcase
    end

    // An integer is MILLI units (kp=250 -> 0.250), a decimal is the real value
    // (kp=0.250500). Both end up as millionths before the Q4.20 conversion.
    wire signed [31:0] micro_pos = seen_dot
                                 ? ($signed({16'd0, val_int}) * 32'sd1000000
                                    + $signed(frac_micro[31:0]))
                                 : ($signed({16'd0, val_int}) * 32'sd1000);
    wire [31:0] micro_val = val_neg ? (-micro_pos) : micro_pos;

    // one conversion per command, never inside the control loop
    wire signed [63:0] q20_wide = ($signed({{32{micro_val[31]}}, micro_val}) * 64'sd1048576)
                                / 64'sd1000000;
    wire signed [23:0] q20_val = q20_wide[23:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key           <= 32'd0;
            key_len       <= 3'd0;
            seen_eq       <= 1'b0;
            val_neg       <= 1'b0;
            val_int       <= 16'd0;
            val_frac      <= 24'd0;
            frac_digits   <= 3'd0;
            seen_dot      <= 1'b0;
            kp_micro      <= KP_MILLI * 24'sd1000;
            ki_micro      <= KI_MILLI * 24'sd1000;
            kd_micro      <= KD_MILLI * 24'sd1000;
            kp_q20        <= (KP_MILLI * 24'sd1048576) / 24'sd1000;
            ki_q20        <= (KI_MILLI * 24'sd1048576) / 24'sd1000;
            kd_q20        <= (KD_MILLI * 24'sd1048576) / 24'sd1000;
            zero_lo       <= ZERO_DEFAULT[11:0] - 12'd1;
            zero_hi       <= ZERO_DEFAULT[11:0] + 12'd1;
            manual_cmd    <= 16'sd0;
            manual_update <= 1'b0;
            dir           <= 1'b0;   // correct polarity after the motor cable swap
            pos_enable    <= 1'b1;   // hold the base near its start
            swing_pwm     <= 8'd100; // bench: full pump amplitude
            deadband_pct  <= 8'd0;
            enc_invert    <= 8'd0;   // 23:46 build: no friction compensation
            slew_limit    <= 8'd100;
            max_cmd       <= 8'd100;
            tel_sel       <= 8'd0;
            max_vel       <= 8'd25;
            kv_q20        <= 24'sd0;
            pos_ki_q20    <= 24'sd0;    // position integral OFF by default (unstable on this rig)
            pos_kp_q20    <= 24'sd209715;  // 0.200
            pos_kd_q20    <= 24'sd2097152; // 2.000
            swing_half_ms <= 16'd285;
            sw_limit      <= 12'd256;   // +-256 counts ~ one motor revolution   // +-256 counts, 512 per revolution
            enable_req    <= 1'b0;
            en_update     <= 1'b0;
            cmd_ok        <= 1'b0;
        end else begin
            cmd_ok        <= 1'b0;
            en_update     <= 1'b0;
            manual_update <= 1'b0;

            if (rx_valid) begin
                if (ch_lc == 8'h0A || ch_lc == 8'h0D || ch_lc == 8'h3B) begin
                    if (seen_eq && key_len != 0) begin
                        if (key_is(key, key_len, K_KP, 3'd2)) begin
                            kp_micro <= micro_val[23:0];
                            kp_q20   <= q20_val;
                        end else if (key_is(key, key_len, K_KI, 3'd2)) begin
                            ki_micro <= micro_val[23:0];
                            ki_q20   <= q20_val;
                        end else if (key_is(key, key_len, K_KD, 3'd2)) begin
                            kd_micro <= micro_val[23:0];
                            kd_q20   <= q20_val;
                        end else if (key_is(key, key_len, K_ZERO, 3'd4)) begin
                            // zero= sets both bounds (single-point zero)
                            if (val_int <= 16'd4095) begin
                                zero_lo <= val_int[11:0];
                                zero_hi <= val_int[11:0];
                            end
                        end else if (key_is(key, key_len, K_ZL, 3'd2)) begin
                            if (val_int <= 16'd4095) zero_lo <= val_int[11:0];
                        end else if (key_is(key, key_len, K_ZH, 3'd2)) begin
                            if (val_int <= 16'd4095) zero_hi <= val_int[11:0];
                        end else if (key_is(key, key_len, K_OUT, 3'd3)) begin
                            // out= is expressed in the same milli units: 15 -> 0.015*1000
                            // = 15 counts, so divide the millionths back down.
                            manual_cmd    <= (micro_val >  32'sd100000000) ? 16'sd100  :
                                             (micro_val < -32'sd100000000) ? -16'sd100 :
                                             (micro_val / 32'sd1000);
                            manual_update <= 1'b1;
                        end else if (key_is(key, key_len, K_KV, 3'd2)) begin
                            kv_q20 <= q20_val;
                        end else if (key_is(key, key_len, K_SLEW, 3'd4)) begin
                            slew_limit <= (val_int > 16'd100) ? 8'd100 : val_int[7:0];
                        end else if (key_is(key, key_len, K_TEL, 3'd3)) begin
                            tel_sel <= (val_int != 0) ? 8'd1 : 8'd0;
                        end else if (key_is(key, key_len, K_MLIM, 3'd4)) begin
                            max_cmd <= (val_int > 16'd100) ? 8'd100 : val_int[7:0];
                        end else if (key_is(key, key_len, K_VLIM, 3'd4)) begin
                            max_vel <= (val_int > 16'd60) ? 8'd60 : val_int[7:0];
                        end else if (key_is(key, key_len, K_DB, 3'd2)) begin
                            deadband_pct <= (val_int > 16'd60) ? 8'd60 : val_int[7:0];
                        end else if (key_is(key, key_len, K_PKI, 3'd3)) begin
                            pos_ki_q20 <= q20_val;
                        end else if (key_is(key, key_len, K_PKP, 3'd3)) begin
                            pos_kp_q20 <= q20_val;
                        end else if (key_is(key, key_len, K_PKD, 3'd3)) begin
                            pos_kd_q20 <= q20_val;
                        end else if (key_is(key, key_len, K_SLIM, 4'd4)) begin
                            if (val_int <= 16'd4000) sw_limit <= val_int[11:0];
                        end else if (key_is(key, key_len, K_ENC, 3'd3)) begin
                            enc_invert <= (val_int != 0) ? 8'd1 : 8'd0;
                        end else if (key_is(key, key_len, K_SWP, 3'd3)) begin
                            // hard limit requested by the operator: swp <= 40
                            swing_pwm <= (val_int > 16'd100) ? 8'd100 : val_int[7:0];
                        end else if (key_is(key, key_len, K_SWF, 3'd3)) begin
                            swing_half_ms <= (val_int < 16'd20)   ? 16'd20 :
                                             (val_int > 16'd2000) ? 16'd2000 : val_int;
                        end else if (key_is(key, key_len, K_POS, 3'd3)) begin
                            pos_enable <= (val_int != 16'd0);
                        end else if (key_is(key, key_len, K_DIR, 3'd3)) begin
                            dir <= (val_int != 16'd0);
                        end else if (key_is(key, key_len, K_EN, 3'd2)) begin
                            enable_req <= (val_int != 16'd0);
                            en_update  <= 1'b1;
                        end
                        cmd_ok <= 1'b1;
                    end
                    key         <= 32'd0;
                    key_len     <= 3'd0;
                    seen_eq     <= 1'b0;
                    val_neg     <= 1'b0;
                    val_int     <= 16'd0;
                    val_frac    <= 24'd0;
                    frac_digits <= 3'd0;
                    seen_dot    <= 1'b0;
                end else if (!seen_eq) begin
                    if (ch_lc == 8'h3D) seen_eq <= 1'b1;          // '='
                    else if (key_len < 3'd4) begin
                        key     <= {key[23:0], ch_lc};
                        key_len <= key_len + 3'd1;
                    end
                end else begin
                    if (ch_lc >= 8'h30 && ch_lc <= 8'h39) begin
                        if (!seen_dot) begin
                            if (val_int < 16'd1000)
                                val_int <= val_int * 16'd10 + {8'd0, ch_lc[3:0]};
                        end else if (frac_digits < 3'd6) begin
                            val_frac    <= val_frac * 24'd10 + {20'd0, ch_lc[3:0]};
                            frac_digits <= frac_digits + 3'd1;
                        end
                    end else if (ch_lc == 8'h2D) val_neg  <= 1'b1;  // '-'
                    else if (ch_lc == 8'h2E) seen_dot <= 1'b1;      // '.'
                end
            end
        end
    end

endmodule
