`timescale 1ns/1ps

module branch(
    input [31:0] A,B,
    input [4:0] control,
    output root
);

reg broot;

always@(*)
    case(control)
      5'b00000 : broot = (A == B) ? 1 : 0;
      5'b00001 : broot = (A != B) ? 1 : 0;
      5'b00010 : broot = ($signed(A) < $signed(B)) ? 1 : 0;
      5'b00011 : broot = ($signed(A) >= $signed(B)) ? 1 : 0;
      5'b00100 : broot = (A < B) ? 1 : 0;
      5'b00101 : broot = (A >= B) ? 1 : 0;  
        default : broot = 0;     
    endcase

assign root = broot;
endmodule
