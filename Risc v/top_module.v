`timescale 1ns/1ps

module riscv_top_simple(
    input clk,
    input rst
);
    reg  [31:0] pc;
    wire [31:0] instruction;
    wire [31:0] pc_plus_4 = pc + 32'd4;
    wire        RegWrite;
    wire [3:0]  alu_control;
    wire        ALUSrc;
    wire [31:0] immediate;
    wire [31:0] reg_read_data1;
    wire [31:0] reg_read_data2;
    wire [31:0] alu_input2;
    wire [31:0] alu_result;
    wire        alu_zero; 

    always @(posedge clk) begin
        if (rst)
            pc <= 32'h0;
        else
            pc <= pc_plus_4;
    end
    Imem imem_inst (
        .a  (pc[9:2]),
        .rd (instruction)
    );

    decoder decoder_inst (
        .instr      (instruction),
        .reg_write  (RegWrite),
        .alucontrol (alu_control)
    );
    regFile reg_file_inst (
        .clk    (clk),
        .we     (RegWrite),
        .a_rd1  (instruction[19:15]),
        .a_rd2  (instruction[24:20]),
        .a_wr   (instruction[11:7]),
        .w_data (alu_result),    
        .rd1    (reg_read_data1),
        .rd2    (reg_read_data2)
    );
    alu alu_inst (
        .A       (reg_read_data1),
        .B       (alu_input2),
        .control (alu_control),
        .result  (alu_result),
        .zero    (alu_zero)
    );
    assign ALUSrc = (instruction[6:0] == 7'b0010011);
    assign alu_input2 = ALUSrc ? immediate : reg_read_data2;
    assign immediate = {{20{instruction[31]}}, instruction[31:20]};

endmodule
