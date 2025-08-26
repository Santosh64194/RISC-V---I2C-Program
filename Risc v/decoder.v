`timescale 1ns/1ps

module decoder(
    input [31:0] instr,
    output  reg_write,
    output reg [3:0] alucontrol 
);

reg reg_writ;
reg isImm;
reg isReg;
reg isBranch;
always@(*) begin
    isReg = (instr[6:0] == 7'b0110011);
    isImm = (instr[6:0] == 7'b0010011);
    isBranch = (instr[6:0] == 7'b1100011);
    if(isReg || isImm) reg_writ = 1;
end

always@(*) begin
case({(isReg || isImm), instr[14:12]})
    4'b1000 : alucontrol = (instr[30]) ? 4'h1 : 4'h0;
    4'b1100 : alucontrol = 4'h4;
    4'b1110 : alucontrol = 4'h3;
    4'b1111 : alucontrol = 4'h2;
    4'b1001 : alucontrol = 4'h5;
    4'b1101 : alucontrol = (instr[30]) ? 4'h7 : 4'h6;
    4'b1010 : alucontrol = 4'h9;
    4'b1011 : alucontrol = 4'h8;
endcase
end

assign reg_write = reg_writ;
endmodule
