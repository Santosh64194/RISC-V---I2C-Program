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
    
endmodule
