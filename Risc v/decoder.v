`timescale 1ns/1ps

module decoder(
    input [31:0] instr,
    output reg_write, wed,
  output reg [3:0] control,
    output reg [1:0] result_src,
    output ImmSrc,
  output is_branch_instr,
  output is_jmp_instr, is_jmpr_instr
);

reg reg_writ, write_data_en;
reg isImm;
reg isJump, isJumpR;
reg isReg;
reg isBranch;
reg isLoad, isStore;
assign ImmSrc = (instr[6:0] == 7'b0010011) ||
                    (instr[6:0] == 7'b0000011) ||
                    (instr[6:0] == 7'b1100111) ||
                    (instr[6:0] == 7'b0100011) ||
                    (instr[6:0] == 7'b1100011);
always@(*) begin
  isReg = (instr[6:0] == 7'b0110011);
    isImm = (instr[6:0] == 7'b0010011);
    isBranch = (instr[6:0] == 7'b1100011);
    isJump = (instr[6:0] == 7'b1101111);
    isJumpR = (instr[6:0] == 7'b1100111);
    isLoad =  (instr[6:0] == 7'b0000011);
    isStore =  (instr[6:0] == 7'b0100011);
  reg_writ = (isReg || isImm || isJump || isJumpR) ? 1'b1 : 1'b0;
  write_data_en = (isStore) ? 1'b1 : 1'b0;
end

always@(*) begin
    if(isBranch)
        case(instr[14:12])
            3'b000 : control = 4'h0;
            3'b001 : control = 4'h1;
            3'b100 : control = 4'h2;
            3'b101 : control = 4'h3;
            3'b110 : control = 4'h4;
            3'b111 : control = 4'h5;
        endcase
    else if(isReg || isImm)
        case({(isReg || isImm), instr[14:12]})
            4'b1000 : control = (instr[30]) ? 4'h1 : 4'h0;
            4'b1100 : control = 4'h4;
            4'b1110 : control = 4'h3;
            4'b1111 : control = 4'h2;
            4'b1001 : control = 4'h5;
            4'b1101 : control = (instr[30]) ? 4'h7 : 4'h6;
            4'b1010 : control = 4'h9;
            4'b1011 : control = 4'h8;
        endcase
end

always@(*) begin
    if(isJump || isJumpR)
        result_src = 2'b10; // Select PC+4
    else if(isLoad) result_src = 2'b01; // Read from Dmem
    else
        result_src = 2'b00; // Default: select ALU result
end

assign is_branch_instr = isBranch; 
assign reg_write = reg_writ;
assign is_jmp_instr = isJump;
assign is_jmpr_instr = isJumpR;
assign wed = write_data_en;
endmodule
