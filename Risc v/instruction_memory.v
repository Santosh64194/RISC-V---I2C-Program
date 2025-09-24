module Imem (
  input  wire [31:0] a,
  output reg  [31:0] rd
);
  reg [31:0] storage [0:10000];

    // initial begin
    //     storage[0] = 32'h00000013; // NOP (ADDI x0, x0, 0)
    //     storage[1] = 32'h00100093; // ADDI x1, x0, 1
    //     storage[2] = 32'h00200113; // ADDI x2, x0, 2
    //     storage[3] = 32'h002081B3; // ADD x3, x1, x2
    //     storage[4] = 32'h00310023; // SW x3, 0(x2)
    // end

  always @(*) rd = storage[a >> 2];

endmodule
