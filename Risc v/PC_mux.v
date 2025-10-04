`timescale 1ns/1ps

module pcmuxd(
    input clk,
    input [31:0] in,
    output [31:0] out
);
reg [31:0] pcout;

always@(posedge clk) pcout <= in;

assign out = pcout;

endmodule
