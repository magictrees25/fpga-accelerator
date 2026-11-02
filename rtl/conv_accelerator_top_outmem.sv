`timescale 1ns / 1ps

// Adds synthesizable output storage to the previously PASSed
// conv_accelerator_top.  The older top and its testbench remain unchanged.
// Outputs are stored as packed 8xINT32 words, one per (M, N-tile).
module conv_accelerator_top_outmem
  import pkg_accelerator::*;
#(
  parameter int H_IN=5,
  parameter int W_IN=5,
  parameter int C_IN=3,
  parameter int KH=3,
  parameter int KW=3,
  parameter int C_OUT=4,
  parameter int STRIDE=1,
  parameter int PADDING=0,
  parameter int ADDR_WIDTH=16
)(
  input logic clk,
  input logic rst_n,
  input logic start,

  // Preload inputs before starting the computation.
  input logic fmap_wr_en,
  input logic [ADDR_WIDTH-1:0] fmap_wr_addr,
  input logic [ACT_WIDTH-1:0] fmap_wr_data,
  input logic weight_wr_en,
  input logic [ADDR_WIDTH-1:0] weight_wr_addr,
  input logic [WGT_WIDTH*ARRAY_COLS-1:0] weight_wr_data,

  // Synchronous output-RAM readback.  Request at clk edge; rd_data/valid
  // update after that edge and are sampled by the host next cycle.
  input  logic out_rd_en,
  input  logic [ADDR_WIDTH-1:0] out_rd_addr,
  output logic [ACC_WIDTH*ARRAY_COLS-1:0] out_rd_data,
  output logic [ARRAY_COLS-1:0] out_rd_mask,
  output logic out_rd_valid,

  // Debug/observation ports: current GEMM write transaction.
  output logic write_valid,
  output logic [ADDR_WIDTH-1:0] write_addr,
  output logic [ADDR_WIDTH-1:0] write_m_addr,
  output logic [ADDR_WIDTH-1:0] write_n_base,
  output logic [ARRAY_COLS-1:0] write_mask,
  output logic [ACC_WIDTH*ARRAY_COLS-1:0] write_data,
  output logic busy,
  output logic done,
  output logic any_overflow
);
  localparam int H_OUT = (H_IN + 2*PADDING - KH)/STRIDE + 1;
  localparam int W_OUT = (W_IN + 2*PADDING - KW)/STRIDE + 1;
  localparam int N_TILES = (C_OUT + ARRAY_COLS - 1)/ARRAY_COLS;
  localparam int OUT_WORDS = H_OUT*W_OUT*N_TILES;

  // Reuse the existing tested computational/data-supply hierarchy.
  conv_accelerator_top #(
    .H_IN(H_IN), .W_IN(W_IN), .C_IN(C_IN),
    .KH(KH), .KW(KW), .C_OUT(C_OUT),
    .STRIDE(STRIDE), .PADDING(PADDING), .ADDR_WIDTH(ADDR_WIDTH)
  ) u_compute (
    .clk(clk), .rst_n(rst_n), .start(start),
    .fmap_wr_en(fmap_wr_en), .fmap_wr_addr(fmap_wr_addr),
    .fmap_wr_data(fmap_wr_data),
    .weight_wr_en(weight_wr_en), .weight_wr_addr(weight_wr_addr),
    .weight_wr_data(weight_wr_data),
    .write_valid(write_valid), .write_addr(write_addr),
    .write_m_addr(write_m_addr), .write_n_base(write_n_base),
    .write_mask(write_mask), .write_data(write_data),
    .busy(busy), .done(done), .any_overflow(any_overflow)
  );

  output_bram #(
    .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(ACC_WIDTH),
    .LANES(ARRAY_COLS), .DEPTH(OUT_WORDS)
  ) u_out_mem (
    .clk(clk), .rst_n(rst_n),
    .wr_en(write_valid), .wr_addr(write_addr),
    .wr_data(write_data), .wr_mask(write_mask),
    .rd_en(out_rd_en), .rd_addr(out_rd_addr),
    .rd_data(out_rd_data), .rd_mask(out_rd_mask),
    .rd_valid(out_rd_valid)
  );
endmodule
