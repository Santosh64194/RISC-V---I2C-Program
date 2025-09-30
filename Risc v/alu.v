`timescale 1ns/1ps

module alu(
    input [31:0] A,B,
    input [4:0] control,
    output wire zero,
    output wire [31:0] result
);
reg [31:0] result_reg;
reg [63:0] temp_rslt;
always@(*) begin
    case(control)
    5'b00000 : result_reg = A + B;
    5'b00001 : result_reg = A - B;
    5'b00010 : result_reg = A & B;
    5'b00011 : result_reg = A | B;
    5'b00100 : result_reg = A ^ B;
    5'b00101 : result_reg = A << B[4:0]; // SLL
    5'b00110 : result_reg = A >> B[4:0]; // SRL
    5'b00111 : result_reg = $signed(A) >>> B[4:0]; // SRA
    5'b01000 : result_reg = (A < B) ? 32'b1 : 32'b0; // SLTU
    5'b01001 : result_reg = ($signed(A) < $signed(B)) ? 32'b1 : 32'b0; // SLT
    5'b01010 : result_reg = $signed(A) * $signed(B);
    5'b01011 : result_reg = temp_rslt >> 32;
    5'b01100 : result_reg = temp_rslt >> 32;
    5'b01101 : result_reg = temp_rslt >> 32;
    5'b01110 : result_reg = $signed(A) / $signed(B);
    5'b01111 : result_reg = A / B;
    5'b10000 : result_reg = $signed(A) % $signed(B);
    5'b10001 : result_reg = A % B;
    default : result_reg = 32'bx;
    endcase
end

always@(*) begin
    if (control == 5'b01011) temp_rslt =  $signed(A) * $signed(B);
    else if (control == 5'b01100) temp_rslt = $signed(A) * B;
    else if (control == 5'b01101) temp_rslt = A * B;
    else temp_rslt = 64'b0;
end
assign zero = (result_reg == 32'b0);
assign result = result_reg;

endmodule
