`timescale 1ns/1ps

module riscv_top(
    input clk,
    input rst
);
    reg  [31:0] pc;
    wire [31:0] instruction;
    wire [31:0] pc_plus_4 = pc + 32'd4;
    wire RegWrite;
    wire [3:0]  ctrl_signal;
    wire ALUSrc, Immsrc, isbranch, is_branch_instr, isJump, isJumpR;
    reg [31:0] immediate;
    wire [31:0] reg_read_data1;
    wire [31:0] reg_read_data2;
    wire [31:0] alu_input2;
    wire [31:0] alu_result;
    wire alu_zero, write_data; 
    wire [31:0] result;
    wire [31:0] dmem_temp_rslt;
    wire [1:0] result_src;
    reg [31:0] tempPC;
    always @(posedge clk) begin
        if (rst)
            tempPC <= 32'h0;
        else begin
            if((is_branch_instr && isbranch) || isJump) tempPC = pc + immediate;
            else if(isJumpR) tempPC =  reg_read_data1 + immediate;
            else tempPC = pc_plus_4;
            end
            //tempPC = ((is_branch_instr && isbranch) || isJump) ? pc + immediate : pc_plus_4;
    end
    Imem imem_inst (
        .a(pc),
        .rd(instruction)
    );

    decoder decoder_inst (
        .instr(instruction),
        .reg_write(RegWrite),
        .control(ctrl_signal),
        .result_src(result_src),
      .ImmSrc(Immsrc),
      .is_branch_instr(is_branch_instr),
      .is_jmp_instr(isJump),
      .is_jmpr_instr(isJumpR),
      .wed(write_data)
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
        .control(ctrl_signal),
        .result(alu_result),
        .zero(alu_zero)
    );

    branch branch_inst(
        .A(reg_read_data1),
        .B(reg_read_data2),
        .control(ctrl_signal),
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
    Dmem dmem(
        .clk(clk),
        .we(write_data),
        .a(mem_addr),
        .wd(reg_read_data2),
        .func3(instruction[14:12]),
        .rd(dmem_temp_rslt)
    );

    assign ALUSrc = (instruction[6:0] == 7'b0010011);
    assign alu_input2 = ALUSrc ? immediate : reg_read_data2;
    assign mem_addr = reg_read_data1 + immediate;
always @(*) 
    if (Immsrc || isJump) 
        case (instruction[6:0])
            // 7'b0010011 : immediate = {{20{instruction[31]}}, instruction[31:20]}; // I-type
            // 7'b0000011 : immediate = {{20{instruction[31]}}, instruction[31:20]}; // Load Imm
            7'b0000011 ,7'b0010011, 7'b1100111 : immediate = {{20{instruction[31]}}, instruction[31:20]}; // JALR I-type
            7'b0100011 : immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // S-type
            7'b1100011 : immediate = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0}; // B-type
            7'b1101111 : immediate = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21],1'b0}; // J-type
            default: immediate = 32'b0;
        endcase
  else
        immediate = 32'b0; 
endmodule
