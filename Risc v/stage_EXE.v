`timescale 1ns/1ps

module stage2 (
    input clk, rst, flush, stall,
    input in_RegWrite, in_wed,
    input [1:0] in_result_src,
    input [31:0] in_pc_plus_4, in_alu_result, in_read_data,
    input [4:0] in_a_wr,
    input [2:0] in_func3,

    output [2:0] o_func3,
    output reg o_RegWrite, 
    output reg o_wed,
    output reg [1:0] o_result_src,
    output reg [31:0] o_pc_plus_4, 
    output reg [31:0] o_alu_result, 
    output reg [31:0] o_read_data,
    output reg [4:0] o_a_wr
);
    always @(posedge clk or posedge rst) begin
        if(rst or flush) begin
           o_RegWrite <= 0;
           o_wed <= 0;
           o_result_src <= 0;
           o_pc_plus_4 <= 0;
           o_alu_result <= 0;
           o_read_data <= 0;
           o_a_wr <= 0;
           o_func3 <= 0;
        end else if (!stall) begin
           o_RegWrite <= in_RegWrite;
           o_wed <= in_wed;
           o_result_src <= in_result_src;
           o_pc_plus_4 <= in_pc_plus_4;
           o_alu_result <= in_alu_result;
           o_read_data <= in_read_data;
           o_a_wr <= in_a_wr;
           o_func3 <= in_func3;
        end
    end
endmodule
