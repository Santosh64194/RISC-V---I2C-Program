`timescale 1ns/1ps

module regFile(
    input clk, we,
    input [4:0] a_rd1, a_rd2, a_wr,
    input [31:0] w_data,
    output [31:0] rd1, rd2
);

reg [31:0] rf [31:0];

always@(posedge clk) if(we && (a_wr != 5'b0)) rf[a_wr] <= w_data;

assign rd1 = (a_rd1 != 5'b0) ? rf[a_rd1] : 32'b0;
assign rd2 = (a_rd2 != 5'b0) ? rf[a_rd2] : 32'b0;
endmodule
