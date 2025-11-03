`timescale 1ns/1ps

module riscv_top(
    input clk,
    input rst,

    output wire uart_tx,
    inout wire [31:0] GPIO_pins,
    output wire timer_tick, 
    output wire timer_pwm_signal
);
    wire  [31:0] fd_pc;
    wire [31:0] ex_pc;
    wire [31:0] instruction;
    wire [31:0] pc_plus_4 = fd_pc + 32'd4;
    wire [31:0] ex_pc_plus_4;
    wire [31:0] mw_pc_plus_4;
    wire RegWrite;
    wire fd_RegWrite , ex_RegWrite, mw_RegWrite;
    wire [4:0]  ctrl_signal;
    wire [4:0]  fd_ctrl_signal;
    wire [4:0] ex_ctrl_signal;
    wire [4:0] mw_ctrl_signal;
    reg fd_ALUSrc;
    wire ALUSrc, Immsrc, isbranch, is_branch_instr, fd_is_branch_instr, isJump, fd_isJump, isJumpR, fd_isJumpR;
    wire ex_ALUSrc, ex_is_branch_instr, ex_isJump, ex_isJumpR;
    wire mw_ALUSrc, mw_is_branch_instr, mw_isJump, mw_isJumpR;
    wire fd_is_lui, ex_is_lui, mw_is_lui;
    wire fd_is_auipc, ex_is_auipc, mw_is_auipc;
    reg [31:0] immediate;
    wire [31:0] ex_immediate;
    reg [31:0] mw_immediate;
    reg [31:0] fd_immediate;
    wire [31:0] reg_read_data1;
    wire [31:0] fd_reg_read_data1;
    wire [31:0] ex_reg_read_data1;
    wire [31:0] mw_reg_read_data1;
    wire [31:0] reg_read_data2;
    wire [31:0] ex_reg_read_data2;
    wire [31:0] mw_reg_read_data2;
    wire [31:0] fd_reg_read_data2;
    wire [31:0] alu_input2;
    wire [31:0] alu_result;
    wire [31:0] ex_alu_result;
    wire [31:0] mw_alu_result;
    wire alu_zero, write_data, fd_write_data, ex_write_data, mw_write_data; 
    wire fd_read_data_en, ex_read_data_en, mw_read_data_en;
    wire [31:0] result;
    wire [31:0] dmem_temp_rslt;
    wire [31:0] fd_dmem_temp_rslt;
    wire [31:0] ex_dmem_temp_rslt;
    wire [31:0] mw_dmem_temp_rslt;
    wire [1:0] result_src;
    wire [1:0] fd_result_src;
    wire [1:0] ex_result_src;
    wire [1:0] mw_result_src;
    reg [31:0] tempPC;
    wire [4:0] rd;
    wire [4:0] ex_rd;
    wire [4:0] mw_rd;
    wire [2:0] fd_func3 = instruction[14:12];
    wire [2:0] ex_func3;
    wire [2:0] mw_func3;
    wire [4:0] fd_rs1_addr;
    wire [4:0] ex_rs1_addr;
    wire [4:0] mw_rs1_addr;
    wire [4:0] fd_rs2_addr;
    wire [4:0] ex_rs2_addr;
    wire [4:0] mw_rs2_addr;
    wire fwd_A, fwd_B;
   wire [31:0] mw_pc;
    wire [31:0] ex_instr;
    wire [31:0] mw_instr; 
    wire [31:0] alu_input1;
  wire [31:0] jalr_data;
  wire stall;
   //peripherals

    wire        mem_write_en, mem_read_en;
    wire [31:0] mem_addr, mem_wdata, mem_rdata;
       
    wire uart_write_en, uart_read_en;
    wire [31:0] uart_addr, uart_wdata, uart_rdata;
    
    wire gpio_write_en, gpio_read_en;
    wire [31:0] gpio_addr, gpio_wdata, gpio_rdata;

    wire timer_write_en, timer_read_en;
    wire [31:0] timer_addr, timer_wdata, timer_rdata;

    // for control hazard
      wire flush;
    // for data hazard: Data forwarding ;)
    wire [31:0] forward_data_A;
    wire [31:0] forward_data_B;
    wire [31:0] new_fd_data1;
    wire [31:0] new_fd_data2;
  assign jalr_data = alu_input1 + ex_immediate;
    always @(*) begin
        if (rst)
            tempPC = 32'h0;
            else if(stall) tempPC = fd_pc;
        else if((ex_is_branch_instr && isbranch) || ex_isJump) tempPC = ex_pc + ex_immediate;
      else if(ex_isJumpR) tempPC = {jalr_data[31:1], 1'b0};
            else tempPC = pc_plus_4;
             //tempPC = ((is_branch_instr && isbranch) || isJump) ? pc + immediate : pc_plus_4;
    end
    Imem imem_inst (
        .a(fd_pc),
        .rd(instruction)
    );

    master_bus_interface master_inst(
        .clk(clk),
        .rst(rst),

        .addr       (mw_alu_result),       
        .wdata      (mw_reg_read_data2),   
        .we         (mw_write_data),       
        .re         (mw_read_data_en),      
        .rdata      (mw_dmem_temp_rslt),   

        .mem_write_en (mem_write_en),
        .mem_read_en  (mem_read_en),
        .mem_addr     (mem_addr),
        .mem_wdata    (mem_wdata),
        .mem_rdata    (mem_rdata),

        .uart_write_en (uart_write_en),
        .uart_read_en  (uart_read_en),
        .uart_addr     (uart_addr),
        .uart_wdata    (uart_wdata),
        .uart_rdata    (uart_rdata),

        .gpio_write_en (gpio_write_en),
        .gpio_read_en  (gpio_read_en),
        .gpio_addr     (gpio_addr),
        .gpio_wdata    (gpio_wdata),
        .gpio_rdata    (gpio_rdata),

        .timer_write_en (timer_write_en),
        .timer_read_en  (timer_read_en),
        .timer_addr     (timer_addr),
        .timer_wdata    (timer_wdata),
        .timer_rdata    (timer_rdata)
    );

    slave_bus_uart #(
        .CLK_FREQ(100000000),
        .BAUD_RATE(115200)
    ) uart_inst (
        .clk(clk),
        .rst(rst),
        .write_en(uart_write_en),
        .read_en(uart_read_en),
        .addr(uart_addr),
        .wdata(uart_wdata),
        .rdata(uart_rdata),
        .tx(uart_tx)
    );

    
    slave_bus_gpio gpio_inst (
        .clk(clk),
        .rst(rst),
        .write_en(gpio_write_en),
        .read_en(gpio_read_en),
        .addr(gpio_addr),
        .wdata(gpio_wdata),
        .rdata(gpio_rdata),
        .GPIO_pins(GPIO_pins)  
    );

    
    slave_bus_timer timer_inst (
        .clk(clk),
        .rst(rst),
        .write_en(timer_write_en),
        .read_en(timer_read_en),
        .addr(timer_addr),
        .wdata(timer_wdata),
        .rdata(timer_rdata),
        .tick(timer_tick),
        .pwm_signal(timer_pwm_signal)
    );

    decoder decoder_inst (
        .instr(instruction),
        .reg_write(fd_RegWrite),
        .control(fd_ctrl_signal),
        .result_src(fd_result_src),
      .ImmSrc(Immsrc),
      .is_branch_instr(fd_is_branch_instr),
      .is_jmp_instr(fd_isJump),
      .is_jmpr_instr(fd_isJumpR),
      .wed(fd_write_data),
      .red(fd_read_data_en),
      .is_lui(fd_is_lui),
      .is_auipc(fd_is_auipc)
    );
    regFile reg_file_inst (
        .clk(clk),
        .rst(rst),
        .we(mw_RegWrite),
        .a_rd1(fd_rs1_addr),
        .a_rd2(fd_rs2_addr),
        .a_wr(mw_rd),
        .w_data(result),
        .rd1(fd_reg_read_data1),
        .rd2(fd_reg_read_data2)
    );

    stage1 stage_IF_ID(
        .clk(clk),
        .rst(rst),
        .flush(flush),
        .stall(stall),
        .in_A(new_fd_data1),
        .in_B(new_fd_data2),
        .in_control(fd_ctrl_signal),
        .in_reg_write(fd_RegWrite),
        .in_wed(fd_write_data),
        .in_is_branch_instr(fd_is_branch_instr),
        .in_is_jmp_instr(fd_isJump),
        .in_is_jmpr_instr(fd_isJumpR),
        .in_ALUSrc(fd_ALUSrc),
        .in_Result_Src(fd_result_src),
        .in_dmem_temp_rslt(fd_dmem_temp_rslt),
        .in_pc(fd_pc),
        .in_pc_plus_4(pc_plus_4),
        .in_immediate(fd_immediate),
        .in_rd(rd),
        .in_func3(fd_func3),
        .in_rs1_addr(fd_rs1_addr),
        .in_rs2_addr(fd_rs2_addr),
        .in_lui(fd_is_lui),
        .in_auipc(fd_is_auipc),
      .in_instr(instruction),
      .in_red(fd_read_data_en),

        .o_red(ex_read_data_en),
        .o_rs1_addr(ex_rs1_addr),
        .o_rs2_addr(ex_rs2_addr),
        .o_A(ex_reg_read_data1),
        .o_B(ex_reg_read_data2),
        .o_control(ex_ctrl_signal),
        .o_reg_write(ex_RegWrite),
        .o_wed(ex_write_data),
        .o_is_branch_instr(ex_is_branch_instr),
        .o_is_jmp_instr(ex_isJump),
        .o_is_jmpr_instr(ex_isJumpR),
        .o_ALUSrc(ex_ALUSrc),
        .o_Result_Src(ex_result_src),
        .o_dmem_temp_rslt(ex_dmem_temp_rslt),
        .o_pc(ex_pc),
        .o_pc_plus_4(ex_pc_plus_4),
        .o_immediate(ex_immediate),
        .o_rd(ex_rd),
      .o_func3(ex_func3),
        .o_lui (ex_is_lui),
      .o_auipc(ex_is_auipc),
      .o_instr(ex_instr)
    );

    stage2 stage_EXE(
        .clk(clk),
        .rst(rst),
        .flush(flush),
        .stall(stall),
        .in_RegWrite(ex_RegWrite),
        .in_wed(ex_write_data),
        .in_result_src(ex_result_src),
        .in_pc_plus_4(ex_pc_plus_4),
        .in_alu_result(ex_alu_result),
        .in_read_data(forward_data_B),
        .in_a_wr(ex_rd),
        .in_func3(ex_func3),
        .in_rs1_addr(ex_rs1_addr),
        .in_rs2_addr(ex_rs2_addr),
        .in_pc(ex_pc),
        .in_instr(ex_instr),
        .in_red(ex_read_data_en),

        .o_red(mw_read_data_en),
        .o_rs1_addr(mw_rs1_addr),
        .o_rs2_addr(mw_rs2_addr),
        .o_RegWrite(mw_RegWrite),
        .o_wed(mw_write_data),
        .o_result_src(mw_result_src),
        .o_pc_plus_4(mw_pc_plus_4),
        .o_alu_result(mw_alu_result),
        .o_read_data(mw_reg_read_data2),
        .o_a_wr(mw_rd),
        .o_func3(mw_func3),
        .o_pc(mw_pc),
        .o_instr(mw_instr)
    );

    alu alu_inst (
      .A(alu_input1),
        .B(alu_input2),
        .control(ex_ctrl_signal),
        .result(ex_alu_result),
        .zero(alu_zero)
    );

    branch branch_inst(
        .A(forward_data_A),
        .B(forward_data_B),
        .control(ex_ctrl_signal),
        .root(isbranch)
    );

    out_reg outreg(
        .alu_result(mw_alu_result),
        .data_mem_result(mw_dmem_temp_rslt),
        .PC(mw_pc_plus_4),
        .sel(mw_result_src),
        .result(result)
    );

    pcmuxd pc_mux(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .in(tempPC),
        .out(fd_pc)
    );
    Dmem dmem(
        .clk(clk),
        .we(mw_write_data),
        .a(mw_alu_result),
        .wd(mw_reg_read_data2),
        .func3(mw_func3),
        .rd(mw_dmem_temp_rslt)
    );

    // ALUSrc having load, alu reg, store and JALR respy
    //assign fd_ALUSrc = (instruction[6:0] == 7'b0000011) || (instruction[6:0] == 7'b0010011) || (instruction[6:0] == 7'b0100011) || (instruction[6:0] == 7'b1100111);
    always@(*) begin
    if((instruction[6:0] == 7'b0000011) || (instruction[6:0] == 7'b0010011) || (instruction[6:0] == 7'b0100011) || (instruction[6:0] == 7'b1100111) || (instruction[6:0] == 7'b0110111) || (instruction[6:0] == 7'b0010111)) fd_ALUSrc = 1'b1;
    else fd_ALUSrc = 1'b0;
  end
    assign branching = isbranch && ex_is_branch_instr;
    assign flush = branching || ex_isJump || ex_isJumpR;
    assign alu_input1 = (ex_is_lui) ? 32'h0 : (ex_is_auipc) ? ex_pc : forward_data_A;
    assign alu_input2 = ex_ALUSrc ? ex_immediate : forward_data_B;
    assign rd = instruction[11:7];
    assign fd_rs1_addr = instruction[19:15];
     assign fd_rs2_addr = instruction[24:20];
    //  assign stall = 0;
    //________________________________________________________________________________________________
    // for data hazard: Data forwarding ;)
    //_________________________________________________________________________________________________
  assign fwd_A = mw_RegWrite && (mw_rd == ex_rs1_addr && mw_rd != 0);
  assign fwd_B = mw_RegWrite && (mw_rd == ex_rs2_addr && mw_rd != 0) ;
    assign forward_data_A = (fwd_A) ? result : ex_reg_read_data1;
    assign forward_data_B = (fwd_B) ? result : ex_reg_read_data2;

    assign new_fd_data1 = (mw_RegWrite && (fd_rs1_addr == mw_rd && mw_rd != 0)) ? result : fd_reg_read_data1;
    assign new_fd_data2 = (mw_RegWrite && (fd_rs2_addr == mw_rd && mw_rd != 0)) ? result : fd_reg_read_data2;
   // __________________________________________________________________________________________________
    // load use hazard

    assign load_use_hazard = (ex_result_src == 2'b01) && ex_RegWrite && (ex_rd != 0) && ((ex_rd == fd_rs1_addr) || (ex_rd == fd_rs2_addr));
   assign stall = load_use_hazard;
   //___________________________________________________________________________________________________
    // assign mem_addr = ex_reg_read_data1 + ex_immediate;
always @(*) 
    if (Immsrc || fd_isJump) 
        case (instruction[6:0])
            // 7'b0010011 : immediate = {{20{instruction[31]}}, instruction[31:20]}; // I-type
            // 7'b0000011 : immediate = {{20{instruction[31]}}, instruction[31:20]}; // Load Imm
            7'b0000011 ,7'b0010011, 7'b1100111 : fd_immediate = {{20{instruction[31]}}, instruction[31:20]}; // JALR I-type
            7'b0100011 : fd_immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // S-type
            7'b1100011 : fd_immediate = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0}; // B-type
            7'b1101111 : fd_immediate = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21],1'b0}; // J-type
            7'b0110111, 7'b0010111: fd_immediate = {instruction[31:12], 12'h000}; // U-type
           default: fd_immediate = 32'b0;
        endcase
  else
        fd_immediate = 32'b0; 
endmodule

`timescale 1ns/1ps

module stage1(
input clk, rst, flush, stall,
input [31:0] in_A, in_B,
input [4:0] in_control,
input in_reg_write, in_wed, in_is_branch_instr, in_is_jmp_instr, in_is_jmpr_instr, in_ALUSrc, in_lui, in_auipc, in_red,
input [1:0] in_Result_Src,
input [31:0] in_dmem_temp_rslt, in_pc, in_pc_plus_4 , in_immediate,
input [4:0] in_rd,
input [2:0] in_func3,
input [4:0] in_rs1_addr,
input [4:0] in_rs2_addr,
input [31:0] in_instr,

output reg [4:0] o_rs1_addr,
output reg [4:0] o_rs2_addr,
output reg [2:0] o_func3,
output reg [31:0] o_A, 
output reg [31:0] o_B,
output reg [4:0] o_control,
output reg o_reg_write, 
output reg o_wed, 
output reg o_red,
output reg o_lui, 
output reg o_auipc,
output reg o_is_branch_instr, 
output reg o_is_jmp_instr, 
output reg o_is_jmpr_instr, 
output reg o_ALUSrc,
output reg [1:0] o_Result_Src,
output reg [31:0] o_dmem_temp_rslt, 
output reg [31:0] o_pc, 
output reg [31:0] o_pc_plus_4 , 
output reg [31:0] o_immediate,
output reg [4:0] o_rd,
output reg [31:0] o_instr
);
always @(posedge clk or posedge rst) begin
    if(rst || flush) begin
        o_A <= 0;
        o_B <= 0;
        o_control <= 5'b0;
        o_reg_write <= 0;
        o_wed <= 0;
        o_is_branch_instr <= 0;
        o_is_jmp_instr <= 0;
        o_is_jmpr_instr <= 0;
        o_ALUSrc <= 0;
        o_Result_Src <= 0;
        o_dmem_temp_rslt <= 0;
        o_pc <= 0;
        o_pc_plus_4 <= 0;
        o_immediate <= 0;
        o_rd <= 0;
        o_func3 <= 0;
        o_rs1_addr <= 0;
        o_rs2_addr <= 0;
        o_lui <= 0;
        o_auipc <= 0;
        o_red <= 0;
        o_instr <= 32'h00000000;
    end else if(!stall) begin
        o_A <= in_A;
        o_B <= in_B;
        o_control <= in_control;
        o_reg_write <= in_reg_write;
        o_wed <= in_wed;
        o_is_branch_instr <= in_is_branch_instr;
        o_is_jmp_instr <= in_is_jmp_instr;
        o_is_jmpr_instr <= in_is_jmpr_instr;
        o_ALUSrc <= in_ALUSrc;
        o_Result_Src <= in_Result_Src;
        o_dmem_temp_rslt <= in_dmem_temp_rslt;
        o_pc <= in_pc;
        o_pc_plus_4 <= in_pc_plus_4;
        o_immediate <= in_immediate;
        o_rd <= in_rd;
        o_func3 <= in_func3;
        o_rs1_addr <= in_rs1_addr;
        o_rs2_addr <= in_rs2_addr;
        o_lui <= in_lui;
        o_auipc <= in_auipc;
        o_instr <= in_instr;
        o_red <= in_red;
    end
end
endmodule


`timescale 1ns/1ps

module stage2 (
    input clk, rst, flush, stall,
    input in_RegWrite, in_wed,in_red,
    input [1:0] in_result_src,
    input [31:0] in_pc_plus_4, in_alu_result, in_read_data, 
    input [4:0] in_a_wr,
    input [2:0] in_func3,
    input [4:0] in_rs1_addr,
    input [4:0] in_rs2_addr,
    input [31:0] in_pc,     
  input [31:0] in_instr,

    output reg [4:0] o_rs1_addr,
    output reg [4:0] o_rs2_addr,
    output reg [2:0] o_func3,
    output reg o_RegWrite, 
    output reg o_wed,
    output reg o_red,
    output reg [1:0] o_result_src,
    output reg [31:0] o_pc_plus_4, 
    output reg [31:0] o_alu_result, 
    output reg [31:0] o_read_data,
    output reg [4:0] o_a_wr,
    output reg [31:0] o_pc,  
    output reg [31:0] o_instr
);
    always @(posedge clk or posedge rst) begin
        if(rst || flush || stall) begin
           o_RegWrite <= 0;
           o_wed <= 0;
           o_result_src <= 0;
           o_pc_plus_4 <= 0;
           o_alu_result <= 0;
           o_read_data <= 0;
           o_a_wr <= 0;
           o_func3 <= 0;
           o_rs1_addr <= 0;
           o_rs2_addr <= 0;
            o_pc <= 32'h0;   
            o_instr <= 32'h0;
            o_red <= 0;
        end else begin
           o_RegWrite <= in_RegWrite;
           o_wed <= in_wed;
           o_result_src <= in_result_src;
           o_pc_plus_4 <= in_pc_plus_4;
           o_alu_result <= in_alu_result;
           o_read_data <= in_read_data;
           o_a_wr <= in_a_wr;
           o_func3 <= in_func3;
           o_rs1_addr <= in_rs1_addr;
           o_rs2_addr <= in_rs2_addr;
            o_pc <= in_pc;     
            o_instr <= in_instr;
            o_red <= in_red;
        end
    end
endmodule

`timescale 1ns/1ps

module regFile(
    input clk, we, rst,
    input [4:0] a_rd1, a_rd2, a_wr,
    input [31:0] w_data,
    output [31:0] rd1, rd2
);

reg [31:0] rf [31:0];
integer i;
always@(posedge clk or posedge rst) begin
    if (rst) begin
        // Reset all 32 registers to 0
        for (i = 0; i < 32; i = i + 1) begin
            rf[i] <= 32'h00000000;
        end
    end 
    // Write on positive edge (non-reset)
    else if(we && (a_wr != 5'b0)) begin
        rf[a_wr] <= w_data;
    end
end

assign rd1 = (a_rd1 != 5'b0) ? rf[a_rd1] : 32'b0;
assign rd2 = (a_rd2 != 5'b0) ? rf[a_rd2] : 32'b0;
endmodule

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

module Imem (
  input  wire [31:0] a,
  output reg  [31:0] rd
);
  reg [31:0] storage [0:10000];

initial begin
    $readmemh("fir.hex", storage);
end
  always @(*) rd = storage[a >> 2];

endmodule


`timescale 1ns/1ps

module decoder(
    input [31:0] instr,
    output reg_write, wed, red,
  output reg [4:0] control,
    output reg [1:0] result_src,
    output reg ImmSrc,
  output is_branch_instr,
  output is_jmp_instr, is_jmpr_instr, is_lui, is_auipc
);

reg reg_writ, write_data_en, read_data_en;
reg isImm;
reg isMul;
reg isJump, isJumpR;
reg isLui, isAuipc;
reg isReg;
reg isBranch;
reg isLoad, isStore;
 always@(*) begin
    if((instr[6:0] == 7'b0010011) ||
                    (instr[6:0] == 7'b0000011) ||
                    (instr[6:0] == 7'b1100111) ||
                    (instr[6:0] == 7'b0100011) ||
       (instr[6:0] == 7'b1100011) || (instr[6:0] == 7'b0110111) || (instr[6:0] == 7'b0010111)) ImmSrc = 1'b1;
    else ImmSrc = 1'b0;
  end
always@(*) begin
  isReg = (instr[6:0] == 7'b0110011);
    isImm = (instr[6:0] == 7'b0010011);
    isBranch = (instr[6:0] == 7'b1100011);
    isJump = (instr[6:0] == 7'b1101111);
    isJumpR = (instr[6:0] == 7'b1100111);
    isLoad =  (instr[6:0] == 7'b0000011);
    isStore =  (instr[6:0] == 7'b0100011);
    isLui = (instr[6:0] == 7'b0110111);
    isAuipc = (instr[6:0] == 7'b0010111);
    isMul = (instr[6:0] == 7'b0110011 && instr[31:25] == 7'b0000001);
  reg_writ = (isReg || isImm || isJump || isJumpR || isLoad || isMul || isLui || isAuipc) ? 1'b1 : 1'b0;
  write_data_en = (isStore) ? 1'b1 : 1'b0;
  read_data_en = isLoad;
end

always@(*) begin
    if(isBranch)
        case(instr[14:12])
            3'b000 : control = 5'h0;
            3'b001 : control = 5'h1;
            3'b100 : control = 5'h2;
            3'b101 : control = 5'h3;
            3'b110 : control = 5'h4;
            3'b111 : control = 5'h5;
        endcase
     else if (isMul)
        case({instr[14:12], instr[31:25]})
            10'b0000000001 : control = 5'ha;
            10'b0010000001 : control = 5'hb;
            10'b0100000001 : control = 5'hc;
            10'b0110000001 : control = 5'hd;
            10'b1000000001 : control = 5'he;
            10'b1010000001 : control = 5'hf;
            10'b1100000001 : control = 5'h10;
            10'b1110000001 : control = 5'h11;
        endcase
    else if(isReg || isImm)
        case({(isReg || isImm), instr[14:12]})
          4'b1000 : control = (instr[30] && isReg) ? 5'h1 : 5'h0;
            4'b1100 : control = 5'h4;
            4'b1110 : control = 5'h3;
            4'b1111 : control = 5'h2;
            4'b1001 : control = 5'h5;
            4'b1101 : control = (instr[30]) ? 5'h7 : 5'h6;
            4'b1010 : control = 5'h9;
            4'b1011 : control = 5'h8;
        endcase
    else if(isLui || isAuipc) control = 5'h0;
end

always@(*) begin
    if(isJump || isJumpR)
        result_src = 2'b10; // Select PC+4
    else if(isLoad) result_src = 2'b01; // Read from Dmem
    else
        result_src = 2'b00; // Default: select ALU result
end

assign is_branch_instr = isBranch; 
assign reg_write = reg_writ;
assign is_jmp_instr = isJump;
assign is_jmpr_instr = isJumpR;
assign wed = write_data_en;
assign is_lui = isLui;
assign is_auipc = isAuipc;
assign red = read_data_en;
endmodule


module branch(
    input [31:0] A,B,
    input [4:0] control,
    output root
);

reg broot;

always@(*)
    case(control)
      5'b00000 : broot = (A == B) ? 1 : 0;
      5'b00001 : broot = (A != B) ? 1 : 0;
      5'b00010 : broot = ($signed(A) < $signed(B)) ? 1 : 0;
      5'b00011 : broot = ($signed(A) >= $signed(B)) ? 1 : 0;
      5'b00100 : broot = (A < B) ? 1 : 0;
      5'b00101 : broot = (A >= B) ? 1 : 0;  
        default : broot = 0;     
    endcase

assign root = broot;
endmodule

`timescale 1ns/1ps

module alu(
    input [31:0] A,B,
    input [4:0] control,
    output wire zero,
    output wire [31:0] result
);
reg [31:0] result_reg;
reg [63:0] temp_rslt;
always@(*) begin
    case(control)
    5'b00000 : result_reg = A + B;
    5'b00001 : result_reg = A - B;
    5'b00010 : result_reg = A & B;
    5'b00011 : result_reg = A | B;
    5'b00100 : result_reg = A ^ B;
    5'b00101 : result_reg = A << B[4:0]; // SLL
    5'b00110 : result_reg = A >> B[4:0]; // SRL
    5'b00111 : result_reg = $signed(A) >>> B[4:0]; // SRA
    5'b01000 : result_reg = (A < B) ? 32'b1 : 32'b0; // SLTU
    5'b01001 : result_reg = ($signed(A) < $signed(B)) ? 32'b1 : 32'b0; // SLT
    5'b01010 : result_reg = $signed(A) * $signed(B);
    5'b01011 : result_reg = temp_rslt >> 32;
    5'b01100 : result_reg = temp_rslt >> 32;
    5'b01101 : result_reg = temp_rslt >> 32;
    5'b01110 : result_reg = $signed(A) / $signed(B);
    5'b01111 : result_reg = A / B;
    5'b10000 : result_reg = $signed(A) % $signed(B);
    5'b10001 : result_reg = A % B;
    default : result_reg = 32'bx;
    endcase
end

always@(*) begin
    if (control == 5'b01011) temp_rslt =  $signed(A) * $signed(B);
    else if (control == 5'b01100) temp_rslt = $signed(A) * B;
    else if (control == 5'b01101) temp_rslt = A * B;
    else temp_rslt = 64'b0;
end
assign zero = (result_reg == 32'b0);
assign result = result_reg;

endmodule


`timescale 1ns/1ps

// module pcmux(
//     input sel,
//     input [31:0] four, immtarget,
//     output [31:0] out_pc
// );

// if(!sel) assign out_pc = four;
// else assign out_pc = immtarget;

// endmodule

module pcmuxd(
    input clk, input rst, input stall,
    input [31:0] in, 
    output [31:0] out
);
reg [31:0] pcout;

always@(posedge clk) if (!stall) pcout <= in;

assign out = pcout;


endmodule


`timescale 1ns/1ps

`timescale 1ns/1ps

module Dmem(
    input clk, we,
    input [31:0] a, wd,
    input [2:0] func3,
    output reg [31:0] rd
);
  reg [7:0] mem [1023:0] ;
  wire [9:0] index = a[9:0];

  // writing
  always @(posedge clk) begin
    if(we) begin
      case (func3)
        3'b000 : mem[index] <= wd[7:0]; 
        3'b001: begin 
          mem[index]   <= wd[7:0];
          mem[index+1] <= wd[15:8];
        end
        3'b010: begin 
          mem[index]   <= wd[7:0];
          mem[index+1] <= wd[15:8];
          mem[index+2] <= wd[23:16];
          mem[index+3] <= wd[31:24];
        end
      endcase
    end
  end

  // reading
  wire [31:0] read_data = {mem[index+3], mem[index+2], mem[index+1], mem[index]};

  always @(*) begin
    if(we == 0) begin
      case (func3)
        3'b000 : rd = {{24{read_data[7]}}, read_data[7:0]}; 
        3'b001 : rd = {{16{read_data[15]}}, read_data[15:0]};
        3'b010 : rd = read_data;               
        3'b100 : rd = {24'h0, read_data[7:0]};              
        3'b101 : rd = {16'h0, read_data[15:0]};
        default: rd = 32'hxxxxxxxx;
      endcase
    end else begin
      rd = 32'hxxxxxxxx;
    end
  end

endmodule

`timescale 1ns / 1ps

module master_bus_interface (
    input  wire        clk,
    input  wire        rst,

    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire        we,
    input  wire        re,
    output reg  [31:0] rdata,

    output reg         mem_write_en,
    output reg         mem_read_en,
    output reg  [31:0] mem_addr,
    output reg  [31:0] mem_wdata,
    input  wire [31:0] mem_rdata,

    output reg         uart_write_en,
    output reg         uart_read_en,
    output reg  [31:0] uart_addr,
    output reg  [31:0] uart_wdata,
    input  wire [31:0] uart_rdata,

    
    output reg         gpio_write_en,
    output reg         gpio_read_en,
    output reg  [31:0] gpio_addr,
    output reg  [31:0] gpio_wdata,
    input  wire [31:0] gpio_rdata,

    output reg         timer_write_en,
    output reg         timer_read_en,
    output reg  [31:0] timer_addr,
    output reg  [31:0] timer_wdata,
    input  wire [31:0] timer_rdata
);

    localparam UART_BASE  = 32'h8000_0000;
    localparam GPIO_BASE  = 32'h8000_1000;
    localparam TIMER_BASE = 32'h8000_2000;
    localparam MEM_BASE   = 32'h0000_0000;

    wire sel_uart  = (addr[31:12] == UART_BASE[31:12]);
    wire sel_gpio  = (addr[31:12] == GPIO_BASE[31:12]);
    wire sel_timer = (addr[31:12] == TIMER_BASE[31:12]);
    wire sel_mem   = !(sel_uart || sel_gpio || sel_timer);

    always @(*) begin
        mem_write_en=0; mem_read_en=0;
        uart_write_en=0; uart_read_en=0;
        gpio_write_en=0; gpio_read_en=0;
        timer_write_en=0; timer_read_en=0;
        mem_addr=0; mem_wdata=0;
        uart_addr=0; uart_wdata=0;
        gpio_addr=0; gpio_wdata=0;
        timer_addr=0; timer_wdata=0;
        rdata=0;

        if (we) begin
            if (sel_uart) begin
                uart_write_en = 1;
                uart_addr = addr;
                uart_wdata = wdata;
            end else if (sel_gpio) begin
                gpio_write_en = 1;
                gpio_addr = addr;
                gpio_wdata = wdata;
            end else if (sel_timer) begin
                timer_write_en = 1;
                timer_addr = addr;
                timer_wdata = wdata;
            end else begin
                mem_write_en = 1;
                mem_addr = addr;
                mem_wdata = wdata;
            end
        end

        else if (re) begin
            if (sel_uart) begin
                uart_read_en = 1;
                uart_addr = addr;
                rdata = uart_rdata;
            end else if (sel_gpio) begin
                gpio_read_en = 1;
                gpio_addr = addr;
                rdata = gpio_rdata;
            end else if (sel_timer) begin
                timer_read_en = 1;
                timer_addr = addr;
                rdata = timer_rdata;
            end else begin
                mem_read_en = 1;
                mem_addr = addr;
                rdata = mem_rdata;
            end
        end
    end
endmodule

`timescale 1 ns / 1 ps




module slave_bus_gpio(
    input  wire        clk,
    input  wire        rst,
    input  wire        write_en,     // from CPU
    input  wire        read_en,      // from CPU
    input  wire [31:0] addr,         // from CPU
    input  wire [31:0] wdata,        // from CPU
    output reg  [31:0] rdata,        // to CPU
    inout wire [31:0] GPIO_pins
);

    reg [31:0] GPIO_PORT_DIR;
    reg [31:0] GPIO_OP_DATA;

    wire [31:0] GPIO_IP_DATA;

    // Address map:
    // 0x0 -> control register
    // 0x4 -> prescaled counter

    // -------- Write logic --------
    always @(posedge clk or negedge rst) begin
        if (rst)begin
            GPIO_PORT_DIR <= 32'h0;
            GPIO_OP_DATA <= 32'h0;
            end
        else if(write_en) begin
            case (addr[3:2])
                2'b00: GPIO_PORT_DIR <= wdata;
                2'b01: GPIO_OP_DATA <= wdata;
                default: ; 
            endcase
        end
    end

    always @(*) begin
        if (read_en) begin
            case (addr[3:2])
                2'b00: rdata = GPIO_PORT_DIR;
                2'b01: rdata = GPIO_OP_DATA;
                2'b10:rdata=GPIO_IP_DATA;
                default: rdata = 32'h0;
            endcase
        end else begin
            rdata = 32'h0;
        end
    end

    // -------- Timer core --------
    gpio_base u_gpio(
        .clk(clk),
        .rst(rst),
        .GPIO_PORT_DIR(GPIO_PORT_DIR),
        .GPIO_OP_DATA(GPIO_OP_DATA),
        .GPIO_IP_DATA(GPIO_IP_DATA),
        .GPIO_pins(GPIO_pins)
        
    );

endmodule

`timescale 1 ns / 1 ps

module slave_bus_timer(
    input  wire        clk,
    input  wire        rst,
    input  wire        write_en,    
    input  wire        read_en,      
    input  wire [31:0] addr,         
    input  wire [31:0] wdata,        
    output reg  [31:0] rdata,       
    output wire        tick,
    output wire        pwm_signal,
    output wire        delay
);

    // -----------------------
    // Timer registers
    // -----------------------
    reg [31:0] control_reg;
    reg [31:0] load_reg;
    reg [31:0] period_reg;
    reg [31:0] pwm_duty_reg;

    wire [31:0] prescaled_cnt;

    // -----------------------
    // Write logic
    // -----------------------
    always @(posedge clk or negedge rst) begin
        if (rst) begin
            control_reg   <= 32'h0;
            load_reg      <= 32'h0;
            period_reg    <= 32'h0;
            pwm_duty_reg  <= 32'h0;
        end else if (write_en) begin
            case (addr[3:2])
                2'b00: control_reg  <= wdata;
                2'b01: load_reg     <= wdata;
                2'b10: period_reg   <= wdata;
                2'b11: pwm_duty_reg <= wdata;
                default: ; 
            endcase
        end
    end

    // -----------------------
    // Read logic
    // -----------------------
    always @(*) begin
        if (read_en) begin
            case (addr[3:2])
                2'b00: rdata = control_reg;
                2'b01: rdata = load_reg;
                2'b10: rdata = period_reg;
                2'b11: rdata = pwm_duty_reg;
                default: rdata = 32'h0;
            endcase
        end else begin
            rdata = 32'h0;
        end
    end

    // -----------------------
    // Timer instance
    // -----------------------
    timer_base u_timer(
        .clk(clk),
        .rst(rst),
        .control(control_reg),
        .load(load_reg),
        .period(period_reg),
        .pwm_duty(pwm_duty_reg),
        .prescaled_cnt(prescaled_cnt),
        .tick(tick),
        .pwm_signal(pwm_signal),
        .delay_flag(delay_flag)
    );

endmodule

`timescale 1 ns / 1 ps


module slave_bus_uart#
(
    parameter CLK_FREQ  = 100000000,
    parameter BAUD_RATE = 115200
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        write_en,    
    input  wire        read_en,      
    input  wire [31:0] addr,         
    input  wire [31:0] wdata,        
    output reg  [31:0] rdata,        
    output wire tx
);

    reg [31:0] CONTROL;
    reg [7:0] TX_DATA;

    
    always @(posedge clk or negedge rst) begin
        if (rst)begin
            CONTROL <= 32'h0;
            TX_DATA <= 32'h0;
            end
        else if(write_en) begin
            case (addr[3:2])
                2'b00: CONTROL <= wdata;
                2'b01: TX_DATA<= wdata;
                default: ; 
            endcase
        end
    end

    always @(*) begin
        if (read_en) begin
            case (addr[3:2])
                2'b00: rdata = CONTROL;
                2'b01: rdata = TX_DATA;
                default: rdata = 32'h0;
            endcase
        end else begin
            rdata = 32'h0;
        end
    end

    uart_base u_gpio(
        .clk(clk),
        .rst(rst),
        .CONTROL(CONTROL),
        .TX_DATA(TX_DATA),
        .tx(tx)
        
    );

endmodule

`timescale 1ns / 1ps

module timer_base(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] control, //0-enable 3,2 prescale 
    input wire [31:0] load,
    input wire [31:0]period,
    input wire[31:0]pwm_duty,
    output reg  [31:0] prescaled_cnt,
    output reg         tick,
    output reg pwm_signal,
    output reg delay_flag
);

    reg [31:0] delay_cnt;
    reg[31:0]pwm_cnt;
    wire [31:0] prescale_val;
    assign prescale_val = (control[3:2] == 2'b00) ? 1   :
                          (control[3:2] == 2'b01) ? 8   :
                          (control[3:2] == 2'b10) ? 64  :
                          (control[3:2] == 2'b11) ? 256 : 1;
//prescale 
    always @(posedge clk or negedge rst) begin
        if (rst) begin
            prescaled_cnt <= 0;
            tick <= 0;
        end else begin
            tick <= 0; 

            if (control[0]) begin 
                if (prescaled_cnt == prescale_val - 1) begin
                    prescaled_cnt <= 0;
                    tick <= 1; 
                end else begin
                    prescaled_cnt <= prescaled_cnt + 1;
                end
            end else begin
                prescaled_cnt <= 0;
            end
        end
    end
    
    always @(posedge clk or negedge rst) begin
    if (rst) begin
        delay_cnt <= 0;
        delay_flag<=0;
    end 
    else if (tick) begin
        if (delay_cnt >= load) begin
            delay_cnt <= 0;
            delay_flag<=1; end

        else 
            delay_cnt <= delay_cnt + 1;
    end
end    
always @(posedge clk or negedge rst) begin
    if (rst) begin
        pwm_cnt    <= 0;
        pwm_signal <= 0;
    end
    else if (tick) begin
        // increment PWM counter
        if (pwm_cnt >= period)
            pwm_cnt <= 0;
        else
            pwm_cnt <= pwm_cnt + 1;

        // generate PWM output
        if (pwm_cnt < pwm_duty)
            pwm_signal <= 1'b1;
        else
            pwm_signal <= 1'b0;
    end
end

    endmodule

    `timescale 1ns / 1ps

module uart_base #
(
    parameter CLK_FREQ  = 100000000,
    parameter BAUD_RATE = 115200
)
(
    input  wire clk,
    input  wire rst,

    // Internal registers
    input  wire [31:0] CONTROL,
    input  wire [7:0]  TX_DATA,
    output reg         tx
    );

    reg tx_busy;
    // 1. Baud generator
    localparam integer BAUD_DIV = CLK_FREQ / BAUD_RATE;
    reg [15:0] baud_cnt = 0;
    reg baud_tick = 0;

    always @(posedge clk or negedge rst) begin
        if (rst) begin
            baud_cnt <= 0;
            baud_tick <= 0;
        end else if (baud_cnt == BAUD_DIV - 1) begin
            baud_cnt <= 0;
            baud_tick <= 1;
        end else begin
            baud_cnt <= baud_cnt + 1;
            baud_tick <= 0;
        end
    end

    // 2. UART Transmitter
    reg [9:0] tx_shift = 10'b1111111111;
    reg [3:0] bit_cnt = 0;

    wire tx_enable = CONTROL[0];  // Bit 0 = enable TX
    wire tx_start  = CONTROL[1];  // Bit 1 = start transmit

    always @(posedge clk or negedge rst) begin
        if (rst) begin
            tx <= 1'b1; // idle
            tx_busy <= 1'b0;
            tx_shift <= 10'b1111111111;
            bit_cnt <= 0;
        end else begin
            if (tx_start && !tx_busy && tx_enable) begin
                tx_shift <= {1'b1, TX_DATA, 1'b0};
                tx_busy <= 1'b1;
                bit_cnt <= 0;
                tx <= 1'b0;  // Start bit immediately
            end 
            else if (baud_tick && tx_busy) begin
                tx <= tx_shift[0];
                tx_shift <= {1'b1, tx_shift[9:1]};
                bit_cnt <= bit_cnt + 1;

                if (bit_cnt == 9) begin
                    tx_busy <= 1'b0;
                    tx <= 1'b1; // Idle after stop bit
                end
            end 
            else if (!tx_busy) begin
                tx <= 1'b1; // Keep line idle
            end
        end
    end

endmodule

`timescale 1ns / 1ps

module gpio_base(
    input  wire        clk,
    input  wire        rst,
    inout  wire [31:0] GPIO_pins,

    input  wire [31:0] GPIO_PORT_DIR,   // 1 = Output, 0 = Input
    input  wire [31:0] GPIO_OP_DATA,    // Output data to drive
    output reg  [31:0] GPIO_IP_DATA     // Input data read from pins
);

    // Sample pin inputs
    always @(posedge clk or negedge rst) begin
        if (rst)
            GPIO_IP_DATA <= 32'b0;
        else
            GPIO_IP_DATA <= GPIO_pins;
    end

    // Tri-state control for each bit
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gpio_buf
            assign GPIO_pins[i] = GPIO_PORT_DIR[i] ? GPIO_OP_DATA[i] : 1'bz;
        end
    endgenerate

endmodule