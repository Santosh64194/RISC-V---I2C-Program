module i2c_master_core (
    input  wire        clk,
    input  wire        reset,

    // Control interface
    input  wire        start,       // start transaction
    input  wire [6:0]  slave_addr,  // 7-bit slave address
    input  wire [7:0]  reg_addr,    // register inside slave
    input  wire [7:0]  write_data,  // data to write
    input  wire        rw,          // 0=write, 1=read
    input  wire        valid,       // command valid

    output reg  [7:0]  read_data,   // data read from slave
    output reg         done,        // transaction done
    output reg         ack_error,   // NACK detected

    // I2C lines
    inout  wire        sda,
    output reg         scl
);

    // Internal registers
    reg [7:0] shift_reg;
    reg [3:0] bit_cnt;
    reg [2:0] state;
    reg       sda_out, sda_oe;

    wire sda_in = sda;

    assign sda = (sda_oe) ? sda_out : 1'bz;

    // FSM states
    localparam IDLE  = 3'd0,
               START = 3'd1,
               WRITE = 3'd2,
               READ  = 3'd3,
               ACK   = 3'd4,
               STOP  = 3'd5;

    reg [2:0] next_state;

    // FSM state register
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= IDLE;
        else
            state <= next_state;
    end

    // FSM next state + outputs
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            scl       <= 1'b1;
            sda_out   <= 1'b1;
            sda_oe    <= 1'b0;
            done      <= 1'b0;
            ack_error <= 1'b0;
            read_data <= 8'd0;
            bit_cnt   <= 4'd0;
        end else begin
            case (state)

            IDLE: begin
                scl     <= 1'b1;
                sda_oe  <= 1'b0;
                done    <= 1'b0;
                if (start && valid) begin
                    shift_reg <= {slave_addr, rw}; // prepare address + R/W
                    bit_cnt   <= 4'd0;
                    next_state <= START;
                end else begin
                    next_state <= IDLE;
                end
            end

            START: begin
                sda_out   <= 1'b0;
                sda_oe    <= 1'b1;
                scl       <= 1'b1;
                next_state <= WRITE;
            end

            WRITE: begin
                scl <= ~scl;
                if (scl == 1'b0) begin
                    sda_out <= shift_reg[7];
                    sda_oe  <= 1'b1;
                end else begin
                    shift_reg <= {shift_reg[6:0], 1'b0};
                    bit_cnt   <= bit_cnt + 1;
                    if (bit_cnt == 4'd7)
                        next_state <= ACK;
                end
            end

            READ: begin
                scl <= ~scl;
                if (scl == 1'b1) begin
                    shift_reg <= {shift_reg[6:0], sda_in};
                    bit_cnt   <= bit_cnt + 1;
                    if (bit_cnt == 4'd7) begin
                        read_data <= {shift_reg[6:0], sda_in};
                        next_state <= ACK;
                    end
                end
            end

            ACK: begin
                scl <= ~scl;
                if (scl == 1'b0) begin
                    if (rw == 1'b0) begin
                        sda_oe <= 0; // release SDA, slave ACKs
                    end else begin
                        sda_out <= 0; // master ACKs read
                        sda_oe  <= 1;
                    end
                end else begin
                    if (rw == 1'b0) begin
                        ack_error <= sda_in; // NACK detection
                    end
                    next_state <= STOP;
                end
            end

            STOP: begin
                scl     <= 1'b1;
                sda_out <= 1'b1;
                sda_oe  <= 1'b1;
                done    <= 1'b1;
                next_state <= IDLE;
            end

            endcase
        end
    end

endmodule
