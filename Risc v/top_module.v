`timescale 1ns/1ps

module riscv_top(
    input clk,
    input rst
);
    reg  [31:0] fd_pc;
    reg [31:0] ex_pc;
    reg [31:0] mw_pc;
    wire [31:0] instruction;
    wire [31:0] pc_plus_4 = fd_pc + 32'd4;
    wire [31:0] ex_pc_plus_4;
    wire [31:0] mw_pc_plus_4;
    wire RegWrite;
    wire fd_RegWrite , ex_RegWrite, mw_RegWrite;
    wire [3:0]  ctrl_signal;
    wire [3:0]  fd_ctrl_signal;
    wire [3:0] ex_ctrl_signal;
    wire [3:0] mw_ctrl_signal;
    wire ALUSrc, fd_ALUSrc, Immsrc, isbranch, is_branch_instr, fd_is_branch_instr, isJump, fd_isJump, isJumpR, fd_isJumpR;
    wire ex_ALUSrc, ex_is_branch_instr, ex_isJump, ex_isJumpR;
    wire mw_ALUSrc, mw_is_branch_instr, mw_isJump, mw_isJumpR;
    reg [31:0] immediate;
    reg [31:0] ex_immediate;
    reg [31:0] mw_immediate;
    reg [31:0] fd_immediate;
    wire [31:0] reg_read_data1;
    wire [31:0] fd_reg_read_data1;
    wire [31:0] ex_reg_read_data1;
    wire [31:0] mw_reg_read_data1;
    wire [31:0] reg_read_data2;
    wire [31:0] ex_reg_read_data2;
    wire [31:0] mw_reg_read_data2;
    wire [31:0] fd_reg_read_data2;
    wire [31:0] alu_input2;
    wire [31:0] alu_result;
    wire [31:0] ex_alu_result;
    wire [31:0] mw_alu_result;
    wire alu_zero, write_data, fd_write_data, ex_write_data, mw_write_data; 
    wire [31:0] result;
    wire [31:0] dmem_temp_rslt;
    wire [31:0] fd_dmem_temp_rslt;
    wire [31:0] ex_dmem_temp_rslt;
    wire [31:0] mw_dmem_temp_rslt;
    wire [1:0] result_src;
    wire [1:0] fd_result_src;
    wire [1:0] ex_result_src;
    wire [1:0] mw_result_src;
    reg [31:0] tempPC;
    wire [4:0] rd = instruction[11:7];
    wire [4:0] ex_rd;
    wire [4:0] mw_rd;
    wire flush, stall;
    wire [2:0] fd_func3 = instruction[14:12];
    wire [2:0] ex_func3;
    wire [2:0] mw_func3;
    always @(posedge clk) begin
        if (rst)
            tempPC <= 32'h0;
        else begin
            if((ex_is_branch_instr && isbranch) || ex_isJump) tempPC = ex_pc + ex_immediate;
            else if(ex_isJumpR) tempPC =  ex_reg_read_data1 + ex_immediate;
            else tempPC = pc_plus_4;
            end
             //tempPC = ((is_branch_instr && isbranch) || isJump) ? pc + immediate : pc_plus_4;
    end
    Imem imem_inst (
        .a(fd_pc),
        .rd(instruction)
    );

    decoder decoder_inst (
        .instr(instruction),
        .reg_write(fd_RegWrite),
        .control(fd_ctrl_signal),
        .result_src(fd_result_src),
      .ImmSrc(Immsrc),
      .is_branch_instr(fd_is_branch_instr),
      .is_jmp_instr(fd_isJump),
      .is_jmpr_instr(fd_isJumpR),
      .wed(fd_write_data)
    );
    regFile reg_file_inst (
        .clk(clk),
        .we(mw_RegWrite),
        .a_rd1(instruction[19:15]),
        .a_rd2(instruction[24:20]),
        .a_wr(mw_rd),
        .w_data(result),
        .rd1(fd_reg_read_data1),
        .rd2(fd_reg_read_data2)
    );

    stage1 stage_IF_ID(
        .clk(clk),
        .rst(rst),
        .flush(flush),
        .stall(stall),
        .in_A(fd_reg_read_data1),
        .in_B(fd_reg_read_data2),
        .in_control(fd_ctrl_signal),
        .in_reg_write(fd_RegWrite),
        .in_wed(fd_write_data),
        .in_is_branch_instr(fd_is_branch_instr),
        .in_is_jmp_instr(fd_isJump),
        .in_is_jmpr_instr(fd_isJumpR),
        .in_ALUSrc(fd_ALUSrc),
        .in_Result_Src(fd_result_src),
        .in_dmem_temp_rslt(fd_dmem_temp_rslt),
        .in_pc(fd_pc),
        .in_pc_plus_4(pc_plus_4),
        .in_immediate(fd_immediate),
        .in_rd(rd),
        .in_func3(fd_func3),

        .o_A(ex_reg_read_data1),
        .o_B(ex_reg_read_data2),
        .o_control(ex_ctrl_signal),
        .o_reg_write(ex_RegWrite),
        .o_wed(ex_write_data),
        .o_is_branch_instr(ex_is_branch_instr),
        .o_is_jmp_instr(ex_isJump),
        .o_is_jmpr_instr(ex_isJumpR),
        .o_ALUSrc(ex_ALUSrc),
        .o_Result_Src(ex_result_src),
        .o_dmem_temp_rslt(ex_dmem_temp_rslt),
        .o_pc(ex_pc),
        .o_pc_plus_4(ex_pc_plus_4),
        .o_immediate(ex_immediate),
        .o_rd(ex_rd),
        .o_func3(ex_func3)
    );

    stage2 stage_EXE(
        .clk(clk),
        .rst(rst),
        .flush(flush),
        .stall(stall),
        .in_RegWrite(ex_RegWrite),
        .in_wed(ex_write_data),
        .in_result_src(ex_result_src),
        .in_pc_plus_4(ex_pc_plus_4),
        .in_alu_result(ex_alu_result),
        .in_read_data(ex_reg_read_data2),
        .in_a_wr(ex_rd),
        .in_func3(ex_func3),

        .o_RegWrite(mw_RegWrite),
        .o_wed(mw_write_data),
        .o_result_src(mw_result_src),
        .o_pc_plus_4(mw_pc_plus_4),
        .o_alu_result(mw_alu_result),
        .o_read_data(mw_reg_read_data2),
        .o_a_wr(mw_rd),
        .o_func3(mw_func3)
    );

    alu alu_inst (
        .A(ex_reg_read_data1),
        .B(alu_input2),
        .control(ex_ctrl_signal),
        .result(ex_alu_result),
        .zero(alu_zero)
    );

    branch branch_inst(
        .A(ex_reg_read_data1),
        .B(ex_reg_read_data2),
        .control(ex_ctrl_signal),
        .root(isbranch)
    );

    out_reg outreg(
        .alu_result(mw_alu_result),
        .data_mem_result(mw_dmem_temp_rslt),
        .PC(mw_pc_plus_4),
        .sel(mw_result_src),
        .result(result)
    );

    pcmuxd pc_mux(
        .clk(clk),
        .in(tempPC),
        .out(fd_pc)
    );
    Dmem dmem(
        .clk(clk),
        .we(mw_write_data),
        .a(mw_alu_result),
        .wd(mw_reg_read_data2),
        .func3(mw_func3),
        .rd(mw_dmem_temp_rslt)
    );

    // ALUSrc having load, alu reg, store and JALR respy
    assign fd_ALUSrc = (instruction[6:0] == 7'b0000011) || (instruction[6:0] == 7'b0010011) || (instruction[6:0] == 7'b0100011) || (instruction[6:0] == 7'b1100111);
    assign alu_input2 = ex_ALUSrc ? ex_immediate : ex_reg_read_data2;
    // assign mem_addr = ex_reg_read_data1 + ex_immediate;
always @(*) 
    if (Immsrc || isJump) 
        case (instruction[6:0])
            // 7'b0010011 : immediate = {{20{instruction[31]}}, instruction[31:20]}; // I-type
            // 7'b0000011 : immediate = {{20{instruction[31]}}, instruction[31:20]}; // Load Imm
            7'b0000011 ,7'b0010011, 7'b1100111 : fd_immediate = {{20{instruction[31]}}, instruction[31:20]}; // JALR I-type
            7'b0100011 : fd_immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // S-type
            7'b1100011 : fd_immediate = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0}; // B-type
            7'b1101111 : fd_immediate = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21],1'b0}; // J-type
            default: fd_immediate = 32'b0;
        endcase
  else
        fd_immediate = 32'b0; 
endmodule
