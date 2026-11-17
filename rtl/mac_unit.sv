
// ===========================================================================
// Module:  mac_unit.sv
// Project: AI Inference Accelerator SoC
//
// Description:
//   3-stage pipelined Multiply-Accumulate (MAC) processing element.
//
//   Pipeline stages:
//     Stage 1: Input register (captures act and wgt)
//     Stage 2: Signed 8x8 -> 16-bit multiply
//     Stage 3: 32-bit accumulate with overflow detection
//
//   P6-A update:
//     1. clear_acc  : Clear INT32 accumulator and overflow flag.
//     2. clear_pipe : Clear input and multiplication pipeline only.
//     3. clear_pipe preserves INT32 partial sum and overflow flag.
//
//   Compatibility: iverilog 12.0, Verilator 5.x, VCS, QuestaSim
// ===========================================================================

`timescale 1ns / 1ps

module mac_unit
  import pkg_accelerator::*;
(
  input  logic                          clk,
  input  logic                          rst_n,

  // Control
  input  logic                          clear_acc,   // Clear INT32 accumulator
  input  logic                          clear_pipe,  // Clear pipeline only
  input  logic                          enable,      // Pipeline enable

  // West input: activation from left neighbor or input buffer
  input  logic signed [ACT_WIDTH-1:0]   act_in,

  // North input: weight from above neighbor or SRAM
  input  logic signed [WGT_WIDTH-1:0]   wgt_in,

  // East output: activation passed to right neighbor
  output logic signed [ACT_WIDTH-1:0]   act_out,

  // South output: weight passed to below neighbor
  output logic signed [WGT_WIDTH-1:0]   wgt_out,

  // Accumulator output
  output logic signed [ACC_WIDTH-1:0]   acc_out,

  // Status
  output logic                          overflow
);

  // -------------------------------------------------------------------------
  // Stage 1: Input Registers
  // -------------------------------------------------------------------------

  logic signed [ACT_WIDTH-1:0] act_reg;
  logic signed [WGT_WIDTH-1:0] wgt_reg;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      act_reg <= '0;
      wgt_reg <= '0;
    end else if (clear_pipe) begin
      act_reg <= '0;
      wgt_reg <= '0;
    end else if (enable) begin
      act_reg <= act_in;
      wgt_reg <= wgt_in;
    end
  end

  // Systolic data flow: pass activation east and weight south
  assign act_out = act_reg;
  assign wgt_out = wgt_reg;


  // -------------------------------------------------------------------------
  // Stage 2: Multiply
  // -------------------------------------------------------------------------

  // Signed 8x8 -> 16-bit product
  (* use_dsp = "yes" *)
  logic signed [ACT_WIDTH+WGT_WIDTH-1:0] mul_result;

  logic signed [ACT_WIDTH+WGT_WIDTH-1:0] mul_reg;

  // Combinational multiply
  assign mul_result = act_reg * wgt_reg;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mul_reg <= '0;
    end else if (clear_pipe) begin
      mul_reg <= '0;
    end else if (enable) begin
      mul_reg <= mul_result;
    end
  end


  // -------------------------------------------------------------------------
  // Stage 3: Accumulate
  // -------------------------------------------------------------------------

  logic signed [ACC_WIDTH-1:0] acc_reg;
  logic                        ovf_reg;

  // Sign-extend multiply result to accumulator width
  logic signed [ACC_WIDTH-1:0] mul_extended;

  assign mul_extended =
      {{(ACC_WIDTH-ACT_WIDTH-WGT_WIDTH)
        {mul_reg[ACT_WIDTH+WGT_WIDTH-1]}}, mul_reg};


  // -------------------------------------------------------------------------
  // Overflow Detection
  // -------------------------------------------------------------------------

  logic signed [ACC_WIDTH-1:0] sum_next;
  logic                        ovf_detect;

  assign sum_next = acc_reg + mul_extended;

  // Overflow occurs when adding two same-sign values produces
  // a result with a different sign.
  assign ovf_detect =
      (acc_reg[ACC_WIDTH-1] == mul_extended[ACC_WIDTH-1]) &&
      (sum_next[ACC_WIDTH-1] != acc_reg[ACC_WIDTH-1]);


  // -------------------------------------------------------------------------
  // Accumulator Control
  //
  // clear_acc:
  //   Clear INT32 partial sum and overflow flag.
  //
  // clear_pipe:
  //   Preserve INT32 partial sum and overflow flag.
  //
  // Priority:
  //   Reset > clear_acc > clear_pipe > enable
  // -------------------------------------------------------------------------

  always_ff @(posedge clk or negedge rst_n) begin

    if (!rst_n) begin

      acc_reg <= '0;
      ovf_reg <= 1'b0;

    end else if (clear_acc) begin

      // Clear accumulator for the first K-Tile.
      acc_reg <= '0;
      ovf_reg <= 1'b0;

    end else if (clear_pipe) begin

      // Preserve INT32 partial sum during pipeline clearing.
      acc_reg <= acc_reg;
      ovf_reg <= ovf_reg;

    end else if (enable) begin

      // Normal MAC accumulation.
      acc_reg <= sum_next;
      ovf_reg <= ovf_reg | ovf_detect;

    end

  end


  // -------------------------------------------------------------------------
  // Outputs
  // -------------------------------------------------------------------------

  assign acc_out  = acc_reg;
  assign overflow = ovf_reg;

endmodule
