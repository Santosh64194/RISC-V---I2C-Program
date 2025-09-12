`timescale 1ns/1ps

module riscv_top(
    input clk,
    input rst
);
    reg  [31:0] pc;
    wire [31:0] instruction;
    wire [31:0] pc_plus_4 = pc + 32'd4;
    wire RegWrite;
    wire [3:0]  alu_control;
    wire ALUSrc, Immsrc, isbranch, is_branch_instr;
    reg [31:0] immediate;
    wire [31:0] reg_read_data1;
    wire [31:0] reg_read_data2;
    wire [31:0] alu_input2;
    wire [31:0] alu_result;
    wire alu_zero; 
    wire [31:0] result;
    wire [31:0] dmem_temp_rslt;
    wire [1:0] result_src;
    reg [31:0] tempPC;
    always @(posedge clk) begin
        if (rst)
            tempPC <= 32'h0;
        else tempPC = (is_branch_instr && isbranch) ? pc + immediate : pc_plus_4;
    end
    Imem imem_inst (
        .a(pc),
        .rd(instruction)
    );

    decoder decoder_inst (
        .instr(instruction),
        .reg_write(RegWrite),
        .alucontrol(alu_control),
        .result_src(result_src),
      .ImmSrc(Immsrc),
      .is_branch_instr(is_branch_instr)
    );
    regFile reg_file_inst (
        .clk(clk),
        .we(RegWrite),
        .a_rd1(instruction[19:15]),
        .a_rd2(instruction[24:20]),
        .a_wr(instruction[11:7]),
        .w_data(result),    
        .rd1(reg_read_data1),
        .rd2(reg_read_data2)
    );
    alu alu_inst (
        .A(reg_read_data1),
        .B(alu_input2),
        .control(alu_control),
        .result(alu_result),
        .zero(alu_zero)
    );

    branch branch_inst(
        .A(reg_read_data1),
        .B(reg_read_data2),
        .control(alu_control),
        .root(isbranch)
    );

    out_reg outreg(
        .alu_result(alu_result),
        .data_mem_result(dmem_temp_rslt),
        .PC(pc_plus_4),
        .sel(result_src),
        .result(result)
    );

    pcmuxd pc_mux(
        .clk(clk),
        .in(tempPC),
        .out(pc)
    );

    assign ALUSrc = (instruction[6:0] == 7'b0010011);
    assign alu_input2 = ALUSrc ? immediate : reg_read_data2;

always @(*) 
    if (Immsrc) 
        case (instruction[6:0])
            7'b0010011: immediate = {{20{instruction[31]}}, instruction[31:20]}; // I-type
            7'b0100011: immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // S-type
            7'b1100011: immediate = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0}; // B-type
            default: immediate = 32'b0; 
        endcase
  else
        immediate = 32'b0; 
endmodule
