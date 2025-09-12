`timescale 1ns/1ps

module decoder(
    input [31:0] instr,
    output reg_write,
  output reg [3:0] alucontrol,
    output [1:0] result_src,
    output ImmSrc,
  output is_branch_instr
);
assign result_src = 2'b00;
reg reg_writ;
reg isImm;
reg isReg;
reg isBranch;
assign ImmSrc = (instr[6:0] == 7'b0010011) || // I-type (ADDI)
                (instr[6:0] == 7'b0100011) || // S-type (store)
                (instr[6:0] == 7'b1100011);   // B-type (branch)
always@(*) begin
  isReg = (instr[6:0] == 7'b0110011);
    isImm = (instr[6:0] == 7'b0010011);
    isBranch = (instr[6:0] == 7'b1100011);
  reg_writ = (isReg || isImm) ? 1'b1 : 1'b0;
end

always@(*) begin
    if(isBranch)
        case(instr[14:12])
            3'b000 : alucontrol = 4'h0;
            3'b001 : alucontrol = 4'h1;
            3'b100 : alucontrol = 4'h2;
            3'b101 : alucontrol = 4'h3;
            3'b110 : alucontrol = 4'h4;
            3'b111 : alucontrol = 4'h5;
        endcase
    else if(isReg || isImm)
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
 assign is_branch_instr = isBranch; 
assign reg_write = reg_writ;
  
endmodule
