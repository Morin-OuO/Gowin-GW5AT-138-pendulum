// motor_driver.v -- H-bridge driver with two direction pins (TB6612 style).
//
// command is in "reference units": -100..+100, matching the teaching source
// (Motor_SetPWM takes -100..100). DUTY_SCALE converts that to the PWM duty
// resolution, so the default 10 gives 0.1% steps.
`timescale 1ns/1ps

module motor_driver #(
    parameter integer CLK_HZ    = 50_000_000,
    parameter integer PWM_HZ    = 20_000,
    parameter integer DUTY_SCALE = 10
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire signed [15:0] command,     // -100 .. +100
    input  wire        [7:0]  deadband_pct, // duty floor for any non-zero cmd
    output reg                pwm,
    output reg                in1,
    output reg                in2
);

    localparam integer PERIOD = (CLK_HZ / PWM_HZ < 2) ? 2 : (CLK_HZ / PWM_HZ);

    reg [15:0] counter;
    reg [15:0] duty;
    reg        dir_negative;

    wire signed [16:0] cmd_ext = command;
    wire        [16:0] magnitude = cmd_ext[16] ? (-cmd_ext) : cmd_ext;
    // Static friction: below ~12 % duty the motor does not turn. Map any
    // non-zero command onto [deadband .. 100 %] so small PID outputs still
    // move the cart, and so the response is roughly linear.
    wire [31:0] floor_duty = (PERIOD * deadband_pct) / 100;
    wire [31:0] span_duty  = PERIOD - floor_duty;
    wire [31:0] duty_full  = (magnitude == 0) ? 32'd0
                                             : floor_duty + (magnitude * span_duty) / 100;

    always @(*) begin
        if (duty_full > PERIOD) duty = PERIOD;
        else                    duty = duty_full[15:0];
        dir_negative = cmd_ext[16];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 16'd0;
            pwm     <= 1'b0;
            in1     <= 1'b0;
            in2     <= 1'b0;
        end else begin
            if (counter >= PERIOD - 1) counter <= 16'd0;
            else                       counter <= counter + 16'd1;

            if (!enable) begin
                pwm <= 1'b0;
                in1 <= 1'b0;
                in2 <= 1'b0;
            end else begin
                pwm <= (counter < duty);
                // direction pins: forward = IN1 high, reverse = IN2 high
                in1 <= !dir_negative;
                in2 <=  dir_negative;
            end
        end
    end

endmodule
