module branch(
    input [31:0] A,B,
    input [3:0] control,
    output root
);

reg broot;

always@(*)
    case(control)
      4'b0000 : broot = (A == B) ? 1 : 0;
      4'b0001 : broot = (A != B) ? 1 : 0;
      4'b0010 : broot = ($signed(A) < $signed(B)) ? 1 : 0;
      4'b0011 : broot = ($signed(A) >= $signed(B)) ? 1 : 0;
      4'b0100 : broot = (A < B) ? 1 : 0;
      4'b0101 : broot = (A >= B) ? 1 : 0;  
        default : broot = 0;     
    endcase

assign root = broot;
endmodule
