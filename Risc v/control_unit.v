// not needed as of now

`timescale 1ns/1ps

module decoder(
    input  [31:0] instr,
    output reg    reg_write,
    output reg [3:0] alucontrol
);

wire isReg = (instr[6:0] == 7'b0110011);	
wire isImm = (instr[6:0] == 7'b0010011);

always @(*) begin
    reg_write  = 1'b0;
    alucontrol = 4'h0;

    if (isReg || isImm) begin
        reg_write = 1'b1;
        case (instr[14:12])
            3'b000: alucontrol = (isReg && instr[30]) ? 4'h1 : 4'h0; // SUB/ADD, ADDI
            3'b100: alucontrol = 4'h4; // XOR/XORI
            3'b110: alucontrol = 4'h3; // OR/ORI
            3'b111: alucontrol = 4'h2; // AND/ANDI
            3'b001: alucontrol = 4'h5; // SLL/SLLI
            3'b101: alucontrol = (instr[30]) ? 4'h7 : 4'h6; // SRA/SRAI vs SRL/SRLI
            3'b010: alucontrol = 4'h9; // SLT/SLTI
            3'b011: alucontrol = 4'h8; // SLTU/SLTIU
            default: begin reg_write = 1'b0; alucontrol = 4'h0; end
        endcase
    end
end

endmodule
