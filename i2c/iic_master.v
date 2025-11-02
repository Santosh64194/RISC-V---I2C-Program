`timescale 1ns/1ps
module iic_controller (
    input  wire       clk,
    input  wire       rst,
    input  wire       i_rw,
    input  wire [6:0] slave_addr,
    input  wire [7:0] data,
    output wire [7:0] o_data,
    output wire       o_done,
    output wire       o_ack_error,
    inout  wire       sda,
    output wire       scl
);

    localparam integer SCL_HALF_CNT = 10'd62;
    localparam integer SCL_FULL_CNT = 10'd124;

    localparam IDLE          = 5'd0;
    localparam START_COND    = 5'd1;
    localparam SEND_BYTE     = 5'd2;
    localparam SCL_LOW       = 5'd3;
    localparam SCL_HIGH      = 5'd4;
    localparam ACK_WAIT      = 5'd5;
    localparam ACK_SCL_HIGH  = 5'd6;
    localparam ACK_DECIDE    = 5'd7;
    localparam ACK_SCL_LOW   = 5'd8;
    localparam READ_BYTE     = 5'd9;
    localparam READ_SCL_LOW  = 5'd10;
    localparam READ_SCL_HIGH = 5'd11;
    localparam READ_ACK_SEND = 5'd12;
    localparam READ_ACK_HIGH = 5'd13;
    localparam READ_ACK_LOW  = 5'd14;
    localparam STOP_COND_1   = 5'd15;
    localparam STOP_COND_2   = 5'd16;
    localparam STOP_FINISH   = 5'd17;
    localparam DONE          = 5'd18;

    reg [4:0]  state, next_state;
    reg [9:0]  clk_counter, next_clk_counter;
    reg [2:0]  bit_counter, next_bit_counter;
    reg [7:0]  data_buffer, next_data_buffer;
    reg [7:0]  read_buffer, next_read_buffer;
    reg [7:0]  data_reg;
    reg [6:0]  slave_addr_reg;
    reg        rw_bit;

    reg        sda_drive, next_sda_drive;
    reg        sda_data, next_sda_data;
    reg        scl_reg, next_scl;

    reg        ack_error, next_ack_error;
    reg        sent_address_flag, next_sent_address_flag;
    reg        done_pulse, next_done_pulse;

    // New: add sampling flag to detect correct ACK timing
    reg scl_prev;
    wire scl_rising = (scl_prev == 0 && scl_reg == 1);

    assign sda = (sda_drive) ? sda_data : 1'bz;
    assign scl = scl_reg;
    assign o_data = read_buffer;
    assign o_ack_error = ack_error;
    assign o_done = done_pulse;

    // ==============================
    // Sequential Logic
    // ==============================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state               <= IDLE;
            clk_counter         <= 0;
            bit_counter         <= 3'd7;
            data_buffer         <= 8'h00;
            read_buffer         <= 8'h00;
            data_reg            <= 8'h00;
            slave_addr_reg      <= 7'h00;
            rw_bit              <= 1'b0;
            sda_drive           <= 1'b0;
            sda_data            <= 1'b1;
            scl_reg             <= 1'b1;
            ack_error           <= 1'b0;
            sent_address_flag   <= 1'b0;
            done_pulse          <= 1'b0;
            scl_prev            <= 1'b1;
        end else begin
            scl_prev            <= scl_reg;

            state               <= next_state;
            clk_counter         <= next_clk_counter;
            bit_counter         <= next_bit_counter;
            data_buffer         <= next_data_buffer;
            read_buffer         <= next_read_buffer;
            sda_drive           <= next_sda_drive;
            sda_data            <= next_sda_data;
            scl_reg             <= next_scl;
            ack_error           <= next_ack_error;
            sent_address_flag   <= next_sent_address_flag;
            done_pulse          <= next_done_pulse;

            // Capture inputs when leaving IDLE
            if (state == IDLE && next_state == START_COND) begin
                data_reg       <= data;
                slave_addr_reg <= slave_addr;
                rw_bit         <= i_rw;
            end

            // Detect ACK/NACK near the end of ACK_SCL_HIGH phase
            if (state == ACK_SCL_HIGH && clk_counter >= (SCL_FULL_CNT - 2)) begin
                // Sample SDA near the end of the high period when slave drives it
                if (sda == 1'b1)
                    ack_error <= 1'b1; // NACK
                else
                    ack_error <= 1'b0; // ACK
            end
    

            // Reading SDA during read phase (unchanged)
            if (state == READ_SCL_HIGH && clk_counter == SCL_HALF_CNT) begin
                read_buffer[bit_counter] <= sda;
            end
        end
    end

    // ==============================
    // Combinational Logic
    // ==============================
    always @(*) begin
        next_state             = state;
        next_clk_counter       = clk_counter;
        next_bit_counter       = bit_counter;
        next_data_buffer       = data_buffer;
        next_read_buffer       = read_buffer;
        next_sda_drive         = sda_drive;
        next_sda_data          = sda_data;
        next_scl               = scl_reg;
        next_ack_error         = ack_error;
        next_sent_address_flag = sent_address_flag;
        next_done_pulse        = 1'b0;

        if (state == IDLE) begin
            next_ack_error         = 1'b0;
            next_sent_address_flag = 1'b0;
            next_done_pulse        = 1'b0;
        end

        case (state)
            IDLE: begin
                next_scl = 1;
                next_sda_drive = 0;
                next_sda_data = 1;
                if (!rst) begin
                    next_state = START_COND;
                end
            end

            START_COND: begin
                next_sda_drive   = 1;
                next_sda_data    = 0;
                next_clk_counter = 0;
                next_bit_counter = 3'd7;
                next_data_buffer = {slave_addr_reg, rw_bit};
                next_state       = SEND_BYTE;
            end

            SEND_BYTE: begin
                next_scl         = 1;
                next_sda_drive   = 1;
                next_sda_data    = data_buffer[bit_counter];
                next_clk_counter = 0;
                next_state       = SCL_LOW;
            end

            SCL_LOW: begin
                next_scl = 0;
                if (clk_counter >= SCL_FULL_CNT) begin
                    next_clk_counter = 0;
                    next_state       = SCL_HIGH;
                end else begin
                    next_clk_counter = clk_counter + 1;
                end
            end

            SCL_HIGH: begin
                next_scl = 1;
                if (clk_counter >= SCL_FULL_CNT) begin
                    next_clk_counter = 0;
                    if (bit_counter == 3'd0) begin
                        next_state = ACK_WAIT;
                    end else begin
                        next_bit_counter = bit_counter - 1;
                        next_state       = SEND_BYTE;
                    end
                end else begin
                    next_clk_counter = clk_counter + 1;
                end
            end

            ACK_WAIT: begin
                next_scl         = 0;
                next_sda_drive   = 0;
                if (clk_counter >= SCL_FULL_CNT) begin
                    next_clk_counter = 0;
                    next_state = ACK_SCL_HIGH;
                end else begin
                    next_clk_counter = clk_counter + 1;
                end
            end

            // SCL HIGH during ACK phase
            ACK_SCL_HIGH: begin
                next_scl = 1;
                if (clk_counter >= SCL_FULL_CNT) begin
                    next_clk_counter = 0;
                    next_state = ACK_DECIDE;
                end else begin
                    next_clk_counter = clk_counter + 1;
                end
            end

            ACK_DECIDE: begin
                next_scl = 0;
                if (ack_error == 1'b1) begin
                    next_state = STOP_COND_1; // NACK detected
                end else begin
                    next_state = ACK_SCL_LOW;
                end
            end

            ACK_SCL_LOW: begin
                next_scl = 0;
                if (!sent_address_flag) begin
                    next_sent_address_flag = 1;
                    next_bit_counter = 3'd7;
                    if (rw_bit == 1'b0) begin
                        next_data_buffer = data_reg;
                        next_state = SEND_BYTE;
                    end else begin
                        next_state = READ_BYTE;
                    end
                end else begin
                    next_state = STOP_COND_1;
                end
            end

            READ_BYTE: begin
                next_scl = 1;
                next_sda_drive = 0;
                next_clk_counter = 0;
                next_state = READ_SCL_LOW;
            end

            READ_SCL_LOW: begin
                next_scl = 0;
                if (clk_counter >= SCL_FULL_CNT) begin
                    next_clk_counter = 0;
                    next_state = READ_SCL_HIGH;
                end else begin
                    next_clk_counter = clk_counter + 1;
                end
            end

            READ_SCL_HIGH: begin
                next_scl = 1;
                if (clk_counter >= SCL_FULL_CNT) begin
                    next_clk_counter = 0;
                    if (bit_counter == 3'd0) begin
                        next_state = READ_ACK_SEND;
                    end else begin
                        next_bit_counter = bit_counter - 1;
                        next_state = READ_BYTE;
                    end
                end else begin
                    next_clk_counter = clk_counter + 1;
                end
            end

            READ_ACK_SEND: begin
                next_scl = 0;
                next_sda_drive = 1;
                next_sda_data  = 1;
                next_clk_counter = 0;
                next_state = READ_ACK_HIGH;
            end

            READ_ACK_HIGH: begin
                next_scl = 1;
                if (clk_counter >= SCL_FULL_CNT) begin
                    next_clk_counter = 0;
                    next_state = READ_ACK_LOW;
                end else begin
                    next_clk_counter = clk_counter + 1;
                end
            end

            READ_ACK_LOW: begin
                next_scl = 0;
                next_state = STOP_COND_1;
            end

            STOP_COND_1: begin
                next_scl = 0;
                next_sda_drive = 1;
                next_sda_data  = 0;
                next_clk_counter = 0;
                next_state = STOP_COND_2;
            end

            STOP_COND_2: begin
                next_scl = 1;
                if (clk_counter >= SCL_FULL_CNT) begin
                    next_clk_counter = 0;
                    next_state = STOP_FINISH;
                end else begin
                    next_clk_counter = clk_counter + 1;
                end
            end

            STOP_FINISH: begin
                next_scl = 1;
                next_sda_drive = 0;
                next_done_pulse = 1'b1;
                next_state = DONE;
            end

            DONE: begin
                next_done_pulse = 1'b0;
                next_state = IDLE;
            end

            default: begin
                next_state = IDLE;
            end
        endcase
    end

endmodule
