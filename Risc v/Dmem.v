`timescale 1ns/1ps

module Dmem(
    input clk, we,
    input [31:0] a, wd,
    input [2:0] func3,
    output reg [31:0] rd
);
  reg [7:0] mem [1023:0] ;

// writing
always @(posedge clk) begin
  if(we) begin
    case (func3)
      3'b000 : mem[a] <= wd[7:0];
      3'b001: begin
        mem[a]   <= wd[7:0];
        mem[a+1] <= wd[15:8];
        end
      3'b010: begin
        mem[a]   <= wd[7:0];
        mem[a+1] <= wd[15:8];
        mem[a+2] <= wd[23:16];
        mem[a+3] <= wd[31:24];
        end
    endcase
  end
end

// reading
    wire [31:0] read_data = {mem[{a[31:2], 2'b11}], mem[{a[31:2], 2'b10}], mem[{a[31:2], 2'b01}], mem[{a[31:2], 2'b00}]};
always @(*) begin
  if(we == 0) begin
    case (func3)
    3'b000 : begin
      case (a[1:0])
        2'b00 : rd = {{24{read_data[7]}}, read_data[7:0]};
        2'b01 : rd = {{24{read_data[15]}}, read_data[15:8]};
        2'b10 : rd = {{24{read_data[23]}}, read_data[23:16]};
        2'b11 : rd = {{24{read_data[31]}}, read_data[31:24]}; 
      endcase
    end 
    3'b001: begin
      case (a[1:0])
        2'b00: rd = {{16{read_data[15]}}, read_data[15:0]};
        2'b10: rd = {{16{read_data[31]}}, read_data[31:16]};
      endcase
    end
    3'b010: rd = read_data;
    3'b100: begin
      case (a[1:0])
        2'b00: rd = {24'h0, read_data[7:0]};
        2'b01: rd = {24'h0, read_data[15:8]};
        2'b10: rd = {24'h0, read_data[23:16]};
        2'b11: rd = {24'h0, read_data[31:24]};
      endcase
      end

    3'b101: begin
      case (a[1:0])
        2'b00: rd = {16'h0, read_data[15:0]};
        2'b10: rd = {16'h0, read_data[31:16]};
      endcase
        end
    default: rd = 32'hxxxxxxxx;
  endcase
  
  end
end

endmodule
