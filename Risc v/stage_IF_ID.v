`timescale 1ns/1ps

module stage1(
input clk, rst, flush, stall,
input [31:0] in_A, in_B,
input [3:0] in_control,
input in_reg_write, in_wed, in_is_branch_instr, in_is_jmp_instr, in_is_jmpr_instr, in_ALUSrc,
input [1:0] in_Result_Src,
input [31:0] in_dmem_temp_rslt, in_pc, in_pc_plus_4 , in_immediate,
input [4:0] in_rd,
input [2:0] in_func3,

output reg [2:0] o_func3,
output reg [31:0] o_A, 
output reg [31:0] o_B,
output reg [3:0] o_control,
output reg o_reg_write, 
output reg o_wed, 
output reg o_is_branch_instr, 
output reg o_is_jmp_instr, 
output reg o_is_jmpr_instr, 
output reg o_ALUSrc,
output reg [1:0] o_Result_Src,
output reg [31:0] o_dmem_temp_rslt, 
output reg [31:0] o_pc, 
output reg [31:0] o_pc_plus_4 , 
output reg [31:0] o_immediate,
output reg [4:0] o_rd
);
    always @(posedge clk or posedge rst) begin
    if(rst || flush) begin
        o_A <= 0;
        o_B <= 0;
        o_control <= 0;
        o_reg_write <= 0;
        o_wed <= 0;
        o_is_branch_instr <= 0;
        o_is_jmp_instr <= 0;
        o_is_jmpr_instr <= 0;
        o_ALUSrc <= 0;
        o_Result_Src <= 0;
        o_dmem_temp_rslt <= 0;
        o_pc <= 0;
        o_pc_plus_4 <= 0;
        o_immediate <= 0;
        o_rd <= 0;
        o_func3 <= 0;
    end else if (!stall) begin
        o_A <= in_A;
        o_B <= in_B;
        o_control <= in_control;
        o_reg_write <= in_reg_write;
        o_wed <= in_wed;
        o_is_branch_instr <= in_is_branch_instr;
        o_is_jmp_instr <= in_is_jmp_instr;
        o_is_jmpr_instr <= in_is_jmpr_instr;
        o_ALUSrc <= in_ALUSrc;
        o_Result_Src <= in_Result_Src;
        o_dmem_temp_rslt <= in_dmem_temp_rslt;
        o_pc <= in_pc;
        o_pc_plus_4 <= in_pc_plus_4;
        o_immediate <= in_immediate;
        o_rd <= in_rd;
        o_func3 <= in_func3;
    end
end
endmodule

