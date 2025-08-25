module i2c_mmio (
    input  wire        clk,
    input  wire        reset,
    input  wire [3:0]  addr,
    input  wire [31:0] wdata,
    input  wire        write_en,
    input  wire        read_en,
    output reg  [31:0] rdata,

    inout  wire        sda,
    output wire        scl
);

    // Control registers
    reg [6:0] slave_addr;
    reg [7:0] reg_addr;
    reg [7:0] write_data;
    reg       rw;
    reg       start;

    // Status
    wire [7:0] read_data;
    wire       done;
    wire       ack_error;

    // I2C core instance
    i2c_master_core core (
        .clk(clk),
        .reset(reset),
        .start(start),
        .slave_addr(slave_addr),
        .reg_addr(reg_addr),
        .write_data(write_data),
        .rw(rw),
        .valid(start),
        .read_data(read_data),
        .done(done),
        .ack_error(ack_error),
        .sda(sda),
        .scl(scl)
    );

    // MMIO registers map
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            slave_addr <= 7'd0;
            reg_addr   <= 8'd0;
            write_data <= 8'd0;
            rw         <= 1'b0;
            start      <= 1'b0;
        end else begin
            if (write_en) begin
                case (addr)
                    4'h0: slave_addr <= wdata[6:0];
                    4'h1: reg_addr   <= wdata[7:0];
                    4'h2: write_data <= wdata[7:0];
                    4'h3: rw         <= wdata[0];
                    4'h4: start      <= wdata[0];
                endcase
            end
        end
    end

    // Readback
    always @(*) begin
        case (addr)
            4'h0: rdata = {25'd0, slave_addr};
            4'h1: rdata = {24'd0, reg_addr};
            4'h2: rdata = {24'd0, write_data};
            4'h3: rdata = {31'd0, rw};
            4'h4: rdata = {30'd0, ack_error, done};
            4'h5: rdata = {24'd0, read_data};
            default: rdata = 32'd0;
        endcase
    end

endmodule
