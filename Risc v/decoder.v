`timescale 1ns/1ps

module decoder(
    input [31:0] instr,
    output reg_write, wed,
  output reg [4:0] control,
    output reg [1:0] result_src,
    output reg ImmSrc,
  output is_branch_instr,
  output is_jmp_instr, is_jmpr_instr
);

reg reg_writ, write_data_en;
reg isImm;
reg isMul;
reg isJump, isJumpR;
reg isReg;
reg isBranch;
reg isLoad, isStore;
 always@(*) begin
    if((instr[6:0] == 7'b0010011) ||
                    (instr[6:0] == 7'b0000011) ||
                    (instr[6:0] == 7'b1100111) ||
                    (instr[6:0] == 7'b0100011) ||
       (instr[6:0] == 7'b1100011)) ImmSrc = 1'b1;
    else ImmSrc = 1'b0;
  end
always@(*) begin
  isReg = (instr[6:0] == 7'b0110011);
    isImm = (instr[6:0] == 7'b0010011);
    isBranch = (instr[6:0] == 7'b1100011);
    isJump = (instr[6:0] == 7'b1101111);
    isJumpR = (instr[6:0] == 7'b1100111);
    isLoad =  (instr[6:0] == 7'b0000011);
    isStore =  (instr[6:0] == 7'b0100011);
    isMul = (instr[6:0] == 7'b0110011 && instr[31:25] == 7'b0000001);
  reg_writ = (isReg || isImm || isJump || isJumpR || isLoad || isMul) ? 1'b1 : 1'b0;
  write_data_en = (isStore) ? 1'b1 : 1'b0;
end

always@(*) begin
    if(isBranch)
        case(instr[14:12])
            3'b000 : control = 5'h0;
            3'b001 : control = 5'h1;
            3'b100 : control = 5'h2;
            3'b101 : control = 5'h3;
            3'b110 : control = 5'h4;
            3'b111 : control = 5'h5;
        endcase
     else if (isMul)
        case({instr[14:12], instr[31:25]})
            10'b0000000001 : control = 5'ha;
            10'b0010000001 : control = 5'hb;
            10'b0100000001 : control = 5'hc;
            10'b0110000001 : control = 5'hd;
            10'b1000000001 : control = 5'he;
            10'b1010000001 : control = 5'hf;
            10'b1100000001 : control = 5'h10;
            10'b1110000001 : control = 5'h11;
        endcase
    else if(isReg || isImm)
        case({(isReg || isImm), instr[14:12]})
            4'b1000 : control = (instr[30]) ? 5'h1 : 5'h0;
            4'b1100 : control = 5'h4;
            4'b1110 : control = 5'h3;
            4'b1111 : control = 5'h2;
            4'b1001 : control = 5'h5;
            4'b1101 : control = (instr[30]) ? 5'h7 : 5'h6;
            4'b1010 : control = 5'h9;
            4'b1011 : control = 5'h8;
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
