`timescale 1ns/1ps

module out_reg (
    input [31:0] alu_result, 
    input [31:0] data_mem_result,
    input [31:0] PC,
    input [1:0] sel,
    output [31:0] result
);

    reg [31:0] re;
    always@(*) begin
        case(sel)
            2'b00 : re = alu_result;
            2'b01 : re = data_mem_result;
            2'b10 : re = PC;
        endcase
    end
    assign result = re;

endmodule 
