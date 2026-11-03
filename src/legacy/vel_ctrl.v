// vel_ctrl.v -- PI velocity loop for the swing-up phase.
//
// The swing-up drives the base with a TARGET SPEED rather than a raw duty
// pulse. Direction/magnitude come from the swing state machine; this module
// turns the speed error into a motor command.
//
//   err  = v_ref - v_meas          (counts per 10 ms)
//   integ += kvi * err             (clamped, anti-windup)
//   cmd   = kvp * err + integ / 8  (clamped to +-LIMIT)
//
// All values are plain integers so they can be tuned from the serial console.
`timescale 1ns/1ps

module vel_ctrl #(
    parameter integer LIMIT = 100
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire               tick,           // 10 ms control tick
    input  wire signed [15:0] v_ref,
    input  wire signed [15:0] v_meas,
    input  wire        [7:0]  kvp,            // command per count/10ms
    input  wire        [7:0]  kvi,            // integral step
    output reg  signed [15:0] cmd
);

    // v_ref is scaled x10 (tenths of a count per 10 ms), v_meas is raw counts/10 ms
    wire signed [17:0] err = $signed(v_ref) - ($signed(v_meas) * 18'sd10);

    reg  signed [23:0] integ;
    wire signed [24:0] integ_next = $signed(integ) + ($signed({17'd0, err}) * $signed({9'd0, kvi}));
    wire signed [23:0] integ_clamped =
        (integ_next >  24'sd512) ?  24'sd512 :
        (integ_next < -24'sd512) ? -24'sd512 : integ_next[23:0];

    wire signed [31:0] raw = ($signed({16'd0, err}) * $signed({24'd0, kvp}))
                           + (integ_clamped >>> 3);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integ <= 24'sd0;
            cmd   <= 16'sd0;
        end else if (!enable) begin
            integ <= 24'sd0;
            cmd   <= 16'sd0;
        end else if (tick) begin
            integ <= integ_clamped;
            cmd   <= (raw >  LIMIT) ?  LIMIT :
                     (raw < -LIMIT) ? -LIMIT : raw[15:0];
        end
    end

endmodule
