`timescale 1ns / 1ps

module verification_tb;

  // --- Parameters ---
  parameter CLK_PERIOD = 10; // 10ns = 100MHz
  
  // --- Signals ---
  reg clk;
  reg rst; // Active-low asynchronous reset

  // --- Log File ---
  integer log_file;
  
  // --- DUT (Device Under Test) ---
  riscv_top i_dut (
    .clk(clk),
    .rst(rst) // Connect to our active-low reset
  );

  // --- Clock Generator ---
  always #(CLK_PERIOD / 2) clk = ~clk;

  // =================================================================
  // == NEW LOGIC: Smarter Commit Logger ==
  // =================================================================
  
  // MODIFIED: Moved these wire declarations outside the always block.
  // This fixes the syntax error.
  wire is_load    = (i_dut.mw_result_src == 2'b01);
  wire is_store   = i_dut.mw_write_data;
  wire is_reg_wr  = (i_dut.mw_RegWrite && i_dut.mw_rd != 0);

  always @(posedge clk) begin
    
    // Check if a valid instruction is in the final (MW) stage
    if (i_dut.mw_instr != 32'h00000000) begin
      
      // Define signals from the MW stage for clarity
      // (Declarations were moved above)

      // --- Log based on instruction type ---

      // For Stores: "core 0: <pc> (<instr>) mem <addr> <data>"
       $display(i_dut.mw_pc);
      if (is_store) begin
          $fdisplay(log_file, "core   0: 0x%08x (0x%08x) mem 0x%08x 0x%08x",
                    i_dut.mw_pc,
                    i_dut.mw_instr,
                    i_dut.mw_alu_result,  // Store address
                    i_dut.mw_reg_read_data2); // Data being stored
      end
      // For Loads: "core 0: <pc> (<instr>) x<reg> <data> mem <addr>"
      else if (is_load) begin
          $fdisplay(log_file, "core   0: 0x%08x (0x%08x) x%0d 0x%08x mem 0x%08x",
                    i_dut.mw_pc,
                    i_dut.mw_instr,
                    i_dut.mw_rd,
                    i_dut.result,         // Data loaded (from result mux)
                    i_dut.mw_alu_result); // Load address
      end
      // For Register Writes (ALU, JAL, etc.): "core 0: <pc> (<instr>) x<reg> <data>"
      else if (is_reg_wr) begin
          $fdisplay(log_file, "core   0: 0x%08x (0x%08x) x%0d 0x%08x",
                    i_dut.mw_pc,
                    i_dut.mw_instr,
                    i_dut.mw_rd,
                    i_dut.result);
      end
      // For others (Branches, NOPs): "core 0: <pc> (<instr>)"
      else begin
          $fdisplay(log_file, "core   0: 0x%08x (0x%08x)",
                    i_dut.ex_pc,
                    i_dut.ex_instr);
      end
    end
  end
  // =================================================================

  // --- Simulation Control ---
  initial begin
    $display("--- SIMULATION START ---");
    log_file = $fopen("dut_commit.log", "w");

    // Initialize signals
    clk = 0;
    rst = 1; // Assert reset (active-low)

    // Load program into instruction memory
    $display("Loading program.hex into instruction memory...");
    $readmemh("startt.hex", i_dut.imem_inst.storage);
    $display("Program loaded.");

    // Wait for a bit
    #(CLK_PERIOD * 5);
    
    // De-assert reset
    rst = 0; // De-assert reset
    $display("--- PROCESSOR RESET RELEASED ---");
   

    // Let the simulation run 
    // Increase this if your program is long
    #(CLK_PERIOD * 500);

    $display("--- SIMULATION FINISHED ---");
    $fclose(log_file);
    $finish;
  end

endmodule

