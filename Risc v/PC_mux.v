`timescale 1ns/1ps

// module pcmux(
//     input sel,
//     input [31:0] four, immtarget,
//     output [31:0] out_pc
// );

// if(!sel) assign out_pc = four;
// else assign out_pc = immtarget;

// endmodule

module pcmuxd(
    input clk,
    input [31:0] in,
    output [31:0] out
);
reg [31:0] pcout;

always@(posedge clk) pcout <= in;

assign out = pcout;


endmodule
