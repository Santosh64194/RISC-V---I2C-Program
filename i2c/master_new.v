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
    reg       rw_pending; // track whether next operation is read

    wire sda_in = sda;

    assign sda = (sda_oe) ? sda_out : 1'bz;

    // FSM states
    localparam IDLE   = 3'd0,
               START  = 3'd1,
               WRITE  = 3'd2,
               READ   = 3'd3,
               ACK    = 3'd4,
               STOP   = 3'd5,
               RESTART = 3'd6;

    reg [2:0] next_state;

    // FSM state register
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= IDLE;
        else
            state <= next_state;
    end

    // FSM outputs + next_state logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            scl       <= 1'b1;
            sda_out   <= 1'b1;
            sda_oe    <= 1'b0;
            done      <= 1'b0;
            ack_error <= 1'b0;
            read_data <= 8'd0;
            bit_cnt   <= 4'd0;
            shift_reg <= 8'd0;
            rw_pending <= 1'b0;
            next_state <= IDLE;
        end else begin
            case (state)

            IDLE: begin
                scl <= 1'b1;
                sda_oe <= 1'b0;
                done <= 1'b0;
                if (start && valid) begin
                    shift_reg <= {slave_addr, 1'b0}; // initial write
                    bit_cnt <= 0;
                    rw_pending <= rw;
                    next_state <= START;
                end else begin
                    next_state <= IDLE;
                end
            end

            START: begin
                sda_out <= 1'b0;
                sda_oe <= 1'b1;
                scl <= 1'b1;
                next_state <= WRITE;
            end

            WRITE: begin
                scl <= ~scl;
                if (scl == 1'b0) begin
                    sda_out <= shift_reg[7];
                    sda_oe <= 1'b1;
                end else begin
                    shift_reg <= {shift_reg[6:0], 1'b0};
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == 4'd7)
                        next_state <= ACK;
                end
            end

            READ: begin
                scl <= ~scl;
                if (scl == 1'b1) begin
                    shift_reg <= {shift_reg[6:0], sda_in};
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == 4'd7) begin
                        read_data <= {shift_reg[6:0], sda_in};
                        next_state <= ACK;
                    end
                end
            end

            ACK: begin
                scl <= ~scl;
                if (scl == 1'b0) begin
                    if (rw_pending == 1'b0) begin
                        sda_oe <= 0; // master releases SDA, expecting ACK from slave
                    end else begin
                        sda_out <= 0; // master ACKs received byte
                        sda_oe <= 1;
                    end
                end else begin
                    if (rw_pending == 1'b0) begin
                        ack_error <= sda_in; // detect NACK from slave
                        // check if we need to restart for read
                        if (rw_pending) begin
                            shift_reg <= {slave_addr, 1'b1}; // repeated start
                            bit_cnt <= 0;
                            next_state <= RESTART;
                        end else begin
                            // check if write completed, go STOP
                            next_state <= STOP;
                        end
                    end else begin
                        // last byte read, generate NACK
                        sda_out <= 1;
                        sda_oe <= 1;
                        next_state <= STOP;
                    end
                end
            end

            RESTART: begin
                sda_out <= 1'b1;
                scl <= 1'b1;
                sda_out <= 1'b0; // repeated start
                bit_cnt <= 0;
                next_state <= WRITE; // send address with read bit
            end

            STOP: begin
                scl <= 1'b1;
                sda_out <= 1'b1;
                sda_oe <= 1'b1;
                done <= 1'b1;
                next_state <= IDLE;
            end

            endcase
        end
    end

endmodule
