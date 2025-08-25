`timescale 1ns / 1ps


module timer_peripheral(
    input wire clk,
    input wire rst,
    output reg tick
    );
    reg[31:0]control;
    reg[31:0]counter;
    
    reg[31:0]prescaled_cnt;
    wire [31:0] prescale_val;
    assign prescale_val = (control[3:2] == 2'b00) ? 1 :
                      (control[3:2] == 2'b01) ? 8 :
                      (control[3:2] == 2'b10) ? 64 :
                      (control[3:2] == 2'b11) ? 256 : 1;
                      
         always @(posedge clk or negedge rst) begin
        if (!rst) begin
            prescaled_cnt <= 0;
            tick <= 0;
        end else if (control[0]) begin   // timer enabled
            if (prescaled_cnt == prescale_val-1) begin
                prescaled_cnt <= 0;
                tick <= 1;               // one-cycle tick pulse
            end else begin
                prescaled_cnt <= prescaled_cnt + 1;
                tick <= 0;
            end
        end else begin
            prescaled_cnt <= 0;
            tick <= 0;
        end
    end
