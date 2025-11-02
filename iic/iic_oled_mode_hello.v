`timescale 1ns/1ps
module iic_oled_mode_hello (
    input  wire clk,
    input  wire rst,
    output wire scl,
    inout  wire sda
);

    // Instantiate your i2c master
    wire [7:0] o_data;
    wire o_done;
    wire o_ack_error;

    reg        start_flag;
    reg [7:0]  data_reg;
    reg [6:0]  slave_addr;
    reg        rw_bit;
    reg [3:0]  step;
    reg        send_next;

    iic_controller i2c_master (
        .clk(clk),
        .rst(rst),
        .i_rw(rw_bit),
        .slave_addr(slave_addr),
        .data(data_reg),
        .o_data(o_data),
        .o_done(o_done),
        .o_ack_error(o_ack_error),
        .sda(sda),
        .scl(scl)
    );

    // Sequence: send address, control byte, "Hello"
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            step <= 0;
            rw_bit <= 0;
            slave_addr <= 7'h3C;
            data_reg <= 8'h00;
            send_next <= 0;
        end else begin
            case (step)
                0: begin
                    // Send control byte (0x40)
                    data_reg <= 8'h40;
                    rw_bit <= 0;
                    step <= 1;
                    send_next <= 1;
                end

                1: if (o_done) begin
                    step <= 2;
                    data_reg <= 8'h48; // H
                    send_next <= 1;
                end

                2: if (o_done) begin
                    step <= 3;
                    data_reg <= 8'h65; // e
                    send_next <= 1;
                end

                3: if (o_done) begin
                    step <= 4;
                    data_reg <= 8'h6C; // l
                    send_next <= 1;
                end

                4: if (o_done) begin
                    step <= 5;
                    data_reg <= 8'h6C; // l
                    send_next <= 1;
                end

                5: if (o_done) begin
                    step <= 6;
                    data_reg <= 8'h6F; // o
                    send_next <= 1;
                end

                6: if (o_done) begin
                    step <= 7;
                    send_next <= 0;
                end

                default: step <= step;
            endcase
        end
    end

endmodule
