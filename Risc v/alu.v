`timescale 1ns/1ps

module alu(
    input [31:0] A,B,
    input [3:0] control,
    output wire zero,
    output wire [31:0] result
);

reg [31:0] result_reg;
//assign wire temp_SRA = A >> B;
always@(*) begin
    case(control)
    4'b0000 : result_reg = A + B;
    4'b0001 : result_reg = A - B;
    4'b0010 : result_reg = A & B;
    4'b0011 : result_reg = A | B;
    4'b0100 : result_reg = A ^ B;
    4'b0101 : result_reg = A << B[4:0]; // SLL
    4'b0110 : result_reg = A >> B[4:0]; // SRL
    4'b0111 : result_reg = $signed(A) >>> B[4:0]; // SRA
    4'b1000 : result_reg = (A < B) ? 32'b1 : 32'b0; // SLTU
    4'b1001 : result_reg = ($signed(A) < $signed(B)) ? 32'b1 : 32'b0; // SLT
    default : result_reg = 32'bx;
    endcase
end
assign zero = (result_reg == 32'b0);
assign result = result_reg;

endmodule
