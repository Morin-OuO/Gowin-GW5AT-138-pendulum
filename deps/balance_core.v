`timescale 1ns/1ps

module uart_rx #(
    parameter integer CLK_HZ = 27000000,
    parameter integer BAUD = 115200
) (
    input wire clk,
    input wire rst_n,
    input wire rx,
    output reg [7:0] data,
    output reg valid,
    output reg framing_error
);
    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam integer HALF_BIT = CLKS_PER_BIT / 2;
    localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

    reg [1:0] state;
    reg [15:0] clk_count;
    reg [2:0] bit_index;
    reg [7:0] shift;
    reg rx_meta, rx_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rx_meta <= 1'b1; rx_sync <= 1'b1; end
        else begin rx_meta <= rx; rx_sync <= rx_meta; end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            clk_count <= 0;
            bit_index <= 0;
            shift <= 0;
            data <= 0;
            valid <= 0;
            framing_error <= 0;
        end else begin
            valid <= 0;
            framing_error <= 0;
            case (state)
                IDLE: if (!rx_sync) begin state <= START; clk_count <= 0; end
                START: begin
                    if (clk_count == HALF_BIT - 1) begin
                        clk_count <= 0;
                        if (!rx_sync) begin state <= DATA; bit_index <= 0; end
                        else state <= IDLE;
                    end else clk_count <= clk_count + 1'b1;
                end
                DATA: begin
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        clk_count <= 0;
                        shift[bit_index] <= rx_sync;
                        if (bit_index == 3'd7) state <= STOP;
                        else bit_index <= bit_index + 1'b1;
                    end else clk_count <= clk_count + 1'b1;
                end
                STOP: begin
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        clk_count <= 0;
                        state <= IDLE;
                        if (rx_sync) begin data <= shift; valid <= 1; end
                        else framing_error <= 1;
                    end else clk_count <= clk_count + 1'b1;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule

module uart_pid_config (
    input wire clk,
    input wire rst_n,
    input wire [7:0] rx_data,
    input wire rx_valid,
    output reg signed [15:0] kp_q8_8,
    output reg signed [15:0] ki_q8_8,
    output reg signed [15:0] kd_q8_8,
    output reg signed [15:0] position_target,
    output reg enabled,
    output reg config_valid,
    output reg config_error
);
    localparam [2:0] WAIT_SYNC = 3'd0, WAIT_CMD = 3'd1,
                     WAIT_LO = 3'd2, WAIT_HI = 3'd3, WAIT_CRC = 3'd4;
    reg [2:0] state;
    reg [7:0] command, value_lo, value_hi, crc;
    reg ascii_active, ascii_in_value, ascii_negative, ascii_dot, ascii_digit_seen, ascii_bad;
    reg [7:0] ascii_c0, ascii_c1, ascii_c2;
    reg [1:0] ascii_command_len, ascii_frac_digits;
    reg [15:0] ascii_whole;
    reg [6:0] ascii_fraction;
    reg [31:0] ascii_math;

    function [15:0] decimal_to_q8_8(
        input [15:0] whole,
        input [6:0] fraction,
        input [1:0] fraction_digits,
        input negative
    );
        reg [31:0] magnitude;
        reg [31:0] frac_scaled;
        begin
            if (fraction_digits == 0) frac_scaled = 0;
            else if (fraction_digits == 1) frac_scaled = (fraction * 256 + 5) / 10;
            else frac_scaled = (fraction * 256 + 50) / 100;
            magnitude = (whole << 8) + frac_scaled;
            if (magnitude > 32767 && !negative) decimal_to_q8_8 = 16'h7fff;
            else if (magnitude > 32768 && negative) decimal_to_q8_8 = 16'h8000;
            else if (negative) decimal_to_q8_8 = -$signed(magnitude[15:0]);
            else decimal_to_q8_8 = magnitude[15:0];
        end
    endfunction

    function [7:0] crc8_byte(input [7:0] seed, input [7:0] value);
        integer i;
        reg [7:0] c;
        begin
            c = seed ^ value;
            for (i = 0; i < 8; i = i + 1)
                c = c[7] ? {c[6:0], 1'b0} ^ 8'h07 : {c[6:0], 1'b0};
            crc8_byte = c;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= WAIT_SYNC; command <= 0; value_lo <= 0; value_hi <= 0; crc <= 0;
            ascii_active <= 0; ascii_in_value <= 0; ascii_negative <= 0;
            ascii_dot <= 0; ascii_digit_seen <= 0; ascii_bad <= 0;
            ascii_c0 <= 0; ascii_c1 <= 0; ascii_c2 <= 0;
            ascii_command_len <= 0; ascii_frac_digits <= 0;
            ascii_whole <= 0; ascii_fraction <= 0;
            kp_q8_8 <= 16'sd384; ki_q8_8 <= 0; kd_q8_8 <= 0;
            position_target <= 0; enabled <= 0;
            config_valid <= 0; config_error <= 0;
        end else begin
            config_valid <= 0;
            config_error <= 0;
            if (rx_valid) begin
                if (ascii_active) begin
                    if (rx_data == 8'h0a || rx_data == 8'h0d) begin
                        ascii_active <= 0;
                        ascii_in_value <= 0;
                        if (ascii_bad || !ascii_in_value || !ascii_digit_seen) config_error <= 1;
                        else if (ascii_c0 == "K" && ascii_c1 == "P") begin
                            kp_q8_8 <= decimal_to_q8_8(ascii_whole, ascii_fraction,
                                                       ascii_frac_digits, ascii_negative);
                            config_valid <= 1;
                        end else if (ascii_c0 == "K" && ascii_c1 == "I") begin
                            ki_q8_8 <= decimal_to_q8_8(ascii_whole, ascii_fraction,
                                                       ascii_frac_digits, ascii_negative);
                            config_valid <= 1;
                        end else if (ascii_c0 == "K" && ascii_c1 == "D") begin
                            kd_q8_8 <= decimal_to_q8_8(ascii_whole, ascii_fraction,
                                                       ascii_frac_digits, ascii_negative);
                            config_valid <= 1;
                        end else if (ascii_c0 == "P" && ascii_c1 == "O" && ascii_c2 == "S" &&
                                     ascii_frac_digits == 0 && ascii_whole <= 32768) begin
                            if (ascii_negative) position_target <= -ascii_whole;
                            else if (ascii_whole <= 32767) position_target <= ascii_whole;
                            else config_error <= 1;
                            if (ascii_negative || ascii_whole <= 32767) config_valid <= 1;
                        end else if (ascii_c0 == "E" && ascii_c1 == "N" &&
                                     ascii_frac_digits == 0 && ascii_whole <= 1) begin
                            enabled <= ascii_whole[0];
                            config_valid <= 1;
                        end else config_error <= 1;
                    end else if (ascii_in_value) begin
                        if (rx_data >= "0" && rx_data <= "9") begin
                            ascii_digit_seen <= 1;
                            if (ascii_dot) begin
                                if (ascii_frac_digits < 2) begin
                                    ascii_math = (ascii_fraction * 32'd10) + (rx_data - 8'd48);
                                    ascii_fraction <= ascii_math[6:0];
                                    ascii_frac_digits <= ascii_frac_digits + 1'b1;
                                end else ascii_bad <= 1;
                            end else if (ascii_whole <= 3276) begin
                                ascii_math = (ascii_whole * 32'd10) + (rx_data - 8'd48);
                                ascii_whole <= ascii_math[15:0];
                            end else ascii_bad <= 1;
                        end else if (rx_data == "." && !ascii_dot && ascii_digit_seen) begin
                            ascii_dot <= 1;
                        end else if (rx_data == "-" && !ascii_digit_seen && !ascii_negative) begin
                            ascii_negative <= 1;
                        end else ascii_bad <= 1;
                    end else if (rx_data == "=") begin
                        ascii_in_value <= 1;
                    end else if (ascii_command_len == 1) begin
                        ascii_c1 <= rx_data;
                        ascii_command_len <= 2;
                    end else if (ascii_command_len == 2) begin
                        ascii_c2 <= rx_data;
                        ascii_command_len <= 3;
                    end else ascii_bad <= 1;
                end else if (state == WAIT_SYNC &&
                             (rx_data == "K" || rx_data == "I" || rx_data == "D" ||
                              rx_data == "P" || rx_data == "E")) begin
                    ascii_active <= 1;
                    ascii_in_value <= 0;
                    ascii_negative <= 0;
                    ascii_dot <= 0;
                    ascii_digit_seen <= 0;
                    ascii_bad <= 0;
                    ascii_c0 <= rx_data;
                    ascii_c1 <= 0;
                    ascii_c2 <= 0;
                    ascii_command_len <= 1;
                    ascii_frac_digits <= 0;
                    ascii_whole <= 0;
                    ascii_fraction <= 0;
                end else begin
                    case (state)
                        WAIT_SYNC: if (rx_data == 8'hA5) state <= WAIT_CMD;
                        WAIT_CMD: begin command <= rx_data; crc <= crc8_byte(0, rx_data); state <= WAIT_LO; end
                        WAIT_LO: begin value_lo <= rx_data; crc <= crc8_byte(crc, rx_data); state <= WAIT_HI; end
                        WAIT_HI: begin value_hi <= rx_data; crc <= crc8_byte(crc, rx_data); state <= WAIT_CRC; end
                        WAIT_CRC: begin
                            state <= WAIT_SYNC;
                            if (rx_data == crc) begin
                                config_valid <= 1;
                                case (command)
                                    8'h01: enabled <= value_lo[0];
                                    8'h02: kp_q8_8 <= {value_hi, value_lo};
                                    8'h03: ki_q8_8 <= {value_hi, value_lo};
                                    8'h04: kd_q8_8 <= {value_hi, value_lo};
                                    8'h05: position_target <= {value_hi, value_lo};
                                    default: begin config_valid <= 0; config_error <= 1; end
                                endcase
                            end else config_error <= 1;
                        end
                        default: state <= WAIT_SYNC;
                    endcase
                end
            end
        end
    end
endmodule

module pid_fixed #(
    parameter integer OUTPUT_LIMIT = 1000,
    parameter integer INTEGRAL_LIMIT = 8388607
) (
    input wire clk,
    input wire rst_n,
    input wire sample_tick,
    input wire enable,
    input wire signed [15:0] setpoint,
    input wire signed [15:0] measurement,
    input wire signed [15:0] kp_q8_8,
    input wire signed [15:0] ki_q8_8,
    input wire signed [15:0] kd_q8_8,
    output reg signed [15:0] output_value
);
    reg signed [16:0] previous_error;
    reg signed [23:0] integral;
    wire signed [16:0] error_now = $signed(setpoint) - $signed(measurement);
    wire signed [17:0] derivative = error_now - previous_error;
    wire signed [39:0] p_product = $signed(kp_q8_8) * $signed(error_now);
    wire signed [39:0] i_product = $signed(ki_q8_8) * $signed(error_now);
    wire signed [39:0] d_product = $signed(kd_q8_8) * $signed(derivative);
    wire signed [39:0] p_term = p_product >>> 8;
    wire signed [39:0] i_delta = i_product >>> 8;
    wire signed [39:0] d_term = d_product >>> 8;
    wire signed [39:0] integral_next = $signed(integral) + i_delta;
    wire signed [39:0] raw_output = p_term + $signed(integral) + d_term;

    function signed [15:0] saturate_output(input signed [39:0] value);
        begin
            if (value > OUTPUT_LIMIT) saturate_output = OUTPUT_LIMIT;
            else if (value < -OUTPUT_LIMIT) saturate_output = -OUTPUT_LIMIT;
            else saturate_output = value[15:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin previous_error <= 0; integral <= 0; output_value <= 0; end
        else if (!enable) begin previous_error <= 0; integral <= 0; output_value <= 0; end
        else if (sample_tick) begin
            previous_error <= error_now;
            output_value <= saturate_output(raw_output);
            if ((raw_output <= OUTPUT_LIMIT && raw_output >= -OUTPUT_LIMIT) ||
                (raw_output > OUTPUT_LIMIT && error_now < 0) ||
                (raw_output < -OUTPUT_LIMIT && error_now > 0)) begin
                if (integral_next > INTEGRAL_LIMIT) integral <= INTEGRAL_LIMIT;
                else if (integral_next < -INTEGRAL_LIMIT) integral <= -INTEGRAL_LIMIT;
                else integral <= integral_next[23:0];
            end
        end
    end
endmodule

module quadrature_encoder (
    input wire clk,
    input wire rst_n,
    input wire encoder_a,
    input wire encoder_b,
    output reg signed [31:0] count
);
    reg [1:0] ab_sync1, ab_sync2, ab_prev;
    wire [3:0] transition = {ab_prev, ab_sync2};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ab_sync1 <= 0; ab_sync2 <= 0; ab_prev <= 0; count <= 0;
        end else begin
            ab_sync1 <= {encoder_a, encoder_b};
            ab_sync2 <= ab_sync1;
            ab_prev <= ab_sync2;
            case (transition)
                4'b0001, 4'b0111, 4'b1110, 4'b1000: count <= count + 1'b1;
                4'b0010, 4'b1011, 4'b1101, 4'b0100: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end
endmodule

module motor_pwm #(
    parameter integer CLK_HZ = 27000000,
    parameter integer PWM_HZ = 20000
) (
    input wire clk,
    input wire rst_n,
    input wire signed [15:0] command,
    input wire enable,
    output reg pwm,
    output reg direction
);
    localparam integer PERIOD = CLK_HZ / PWM_HZ;
    reg [15:0] counter;
    reg [15:0] magnitude;
    always @* begin
        if (command < 0) magnitude = -command;
        else magnitude = command;
        if (magnitude > 1000) magnitude = 1000;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin counter <= 0; pwm <= 0; direction <= 0; end
        else begin
            if (counter == PERIOD - 1) counter <= 0;
            else counter <= counter + 1'b1;
            direction <= command < 0;
            pwm <= enable && (counter < ((PERIOD * magnitude) / 1000));
        end
    end
endmodule

module balance_core #(
    parameter integer CLK_HZ = 27000000,
    parameter integer BAUD = 115200,
    parameter integer CONTROL_HZ = 1000,
    parameter integer TELEMETRY_HZ = 100,
    parameter integer PWM_HZ = 20000,
    parameter integer POSITION_KP_Q8 = 2,
    parameter integer POSITION_KD_Q8 = 0,
    parameter integer MAX_ANGLE_Q8 = 2560
) (
    input wire clk,
    input wire rst_n,
    input wire uart_rx_pin,
    input wire signed [15:0] angle_q8_8,
    input wire encoder_a,
    input wire encoder_b,
    output wire motor_pwm_pin,
    output wire motor_dir_pin,
    output wire signed [15:0] encoder_velocity,
    output wire uart_tx_pin,
    output wire uart_error
);
    localparam integer CONTROL_DIV = CLK_HZ / CONTROL_HZ;
    localparam integer TELEMETRY_DIV = CLK_HZ / TELEMETRY_HZ;
    reg [31:0] control_count;
    reg [31:0] telemetry_count;
    reg sample_tick;
    reg telemetry_tick;
    wire [7:0] rx_data;
    wire rx_valid, rx_frame_error;
    wire signed [15:0] kp, ki, kd, pos_target;
    wire enabled, cfg_valid, cfg_error;
    wire signed [31:0] encoder_count;
    reg signed [31:0] previous_encoder_count;
    reg signed [15:0] encoder_velocity_reg;
    wire signed [15:0] encoder_velocity_next;
    wire signed [31:0] velocity_delta = encoder_count - previous_encoder_count;
    wire signed [15:0] position_error;
    wire signed [31:0] position_product;
    wire signed [31:0] velocity_product;
    wire signed [31:0] position_raw;
    reg signed [15:0] angle_target;
    wire signed [15:0] motor_command;
    wire signed [31:0] angle_cdeg_wide = ($signed(angle_q8_8) * 32'sd100) >>> 8;
    wire signed [15:0] angle_cdeg = angle_cdeg_wide[15:0];

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) uart_i (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx_pin),
        .data(rx_data), .valid(rx_valid), .framing_error(rx_frame_error)
    );
    uart_pid_config cfg_i (
        .clk(clk), .rst_n(rst_n), .rx_data(rx_data), .rx_valid(rx_valid),
        .kp_q8_8(kp), .ki_q8_8(ki), .kd_q8_8(kd),
        .position_target(pos_target), .enabled(enabled),
        .config_valid(cfg_valid), .config_error(cfg_error)
    );
    quadrature_encoder enc_i (
        .clk(clk), .rst_n(rst_n), .encoder_a(encoder_a), .encoder_b(encoder_b),
        .count(encoder_count)
    );
    assign position_error = pos_target - encoder_count[15:0];
    assign position_product = $signed(position_error) * POSITION_KP_Q8;
    assign velocity_product = $signed(encoder_velocity_reg) * POSITION_KD_Q8;
    assign position_raw = (position_product >>> 8) - (velocity_product >>> 8);
    assign encoder_velocity = encoder_velocity_reg;
    assign encoder_velocity_next = velocity_delta > 32767 ? 16'sd32767 :
                                   velocity_delta < -32768 ? -16'sd32768 : velocity_delta[15:0];
    always @* begin
        if (position_raw > MAX_ANGLE_Q8) angle_target = MAX_ANGLE_Q8;
        else if (position_raw < -MAX_ANGLE_Q8) angle_target = -MAX_ANGLE_Q8;
        else angle_target = position_raw[15:0];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            control_count <= 0;
            sample_tick <= 0;
            telemetry_count <= 0;
            telemetry_tick <= 0;
        end else begin
            if (control_count == CONTROL_DIV - 1) begin
                control_count <= 0;
                sample_tick <= 1;
            end else begin
                control_count <= control_count + 1'b1;
                sample_tick <= 0;
            end
            if (telemetry_count == TELEMETRY_DIV - 1) begin
                telemetry_count <= 0;
                telemetry_tick <= 1;
            end else begin
                telemetry_count <= telemetry_count + 1'b1;
                telemetry_tick <= 0;
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            previous_encoder_count <= 0;
            encoder_velocity_reg <= 0;
        end else if (sample_tick) begin
            previous_encoder_count <= encoder_count;
            encoder_velocity_reg <= encoder_velocity_next;
        end
    end

    pid_fixed pid_i (
        .clk(clk), .rst_n(rst_n), .sample_tick(sample_tick), .enable(enabled),
        .setpoint(angle_target), .measurement(angle_q8_8),
        .kp_q8_8(kp), .ki_q8_8(ki), .kd_q8_8(kd), .output_value(motor_command)
    );
    motor_pwm #(.CLK_HZ(CLK_HZ), .PWM_HZ(PWM_HZ)) motor_i (
        .clk(clk), .rst_n(rst_n), .command(motor_command), .enable(enabled),
        .pwm(motor_pwm_pin), .direction(motor_dir_pin)
    );
    vofa_firewater_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) vofa_i (
        .clk(clk), .rst_n(rst_n), .sample_tick(telemetry_tick),
        .angle_cdeg(angle_cdeg), .position_count(encoder_count[15:0]),
        .velocity_counts_per_ms(encoder_velocity_reg), .motor_command(motor_command),
        .tx(uart_tx_pin)
    );
    assign uart_error = rx_frame_error | cfg_error;
endmodule
