`timescale 1ns / 1ps

// Synthesizable convolution compute/data-supply top (bring-up baseline).
// Runs one convolution layer on a fixed configuration per elaboration:
//     M = H_OUT*W_OUT, N=C_OUT, K=KH*KW*C_IN.
// 1-clock synchronous BRAM read latency; one K beat per FEED clock.
// Activation HWC layout and weight (N-tile,K) packed-word layout.
// External preload writes must complete before 'start' is asserted.
// Output results are exposed as write_*; an output RAM can be added later.
// No AXI/DMA/backpressure, no dynamic per-layer reconfiguration yet.
module conv_accelerator_top
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

  // Preload activation SRAM/BRAM; addresses follow flattened HWC order.
  input logic fmap_wr_en,
  input logic [ADDR_WIDTH-1:0] fmap_wr_addr,
  input logic [ACT_WIDTH-1:0] fmap_wr_data,

  // Preload weight SRAM/BRAM; one word holds ARRAY_COLS weights.
  // word_addr = n_tile_idx * K_LEN + k_idx.
  input logic weight_wr_en,
  input logic [ADDR_WIDTH-1:0] weight_wr_addr,
  input logic [WGT_WIDTH*ARRAY_COLS-1:0] weight_wr_data,

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
  localparam int H_OUT=(H_IN+2*PADDING-KH)/STRIDE+1;
  localparam int W_OUT=(W_IN+2*PADDING-KW)/STRIDE+1;
  localparam int M_LEN=H_OUT*W_OUT;
  localparam int N_LEN=C_OUT;
  localparam int K_LEN=KH*KW*C_IN;
  localparam int N_TILES=(C_OUT+ARRAY_COLS-1)/ARRAY_COLS;

  // Explicit inter-module wiring (no hierarchical signals such as dut.clear_acc).
  logic [ADDR_WIDTH-1:0] m_addr, n_base, k_addr, fetch_k_addr;
  logic [ADDR_WIDTH-1:0] fmap_addr, weight_word_addr;
  logic addr_valid, bram_data_valid, wgt_data_valid;
  logic clear_acc, feed_valid;
  logic signed [ACT_WIDTH-1:0] act_data;
  logic [WGT_WIDTH*ARRAY_COLS-1:0] wgt_row_packed;

  gemm_mn_controller_mem #(.ADDR_WIDTH(ADDR_WIDTH)) u_ctrl (
    .clk(clk), .rst_n(rst_n), .start(start),
    .m_len(ADDR_WIDTH'(M_LEN)), .n_len(ADDR_WIDTH'(N_LEN)),
    .k_len(ADDR_WIDTH'(K_LEN)),
    .m_addr(m_addr), .n_base(n_base), .k_addr(k_addr),
    .act_data(act_data), .wgt_row_packed(wgt_row_packed),
    .mem_clear_acc(clear_acc), .mem_feed_valid(feed_valid),
    .write_valid(write_valid), .write_addr(write_addr),
    .write_m_addr(write_m_addr), .write_n_base(write_n_base),
    .write_mask(write_mask), .write_data(write_data),
    .busy(busy), .done(done), .any_overflow(any_overflow)
  );

  conv_bram_reader #(
    .H_IN(H_IN), .W_IN(W_IN), .C_IN(C_IN),
    .KH(KH), .KW(KW), .STRIDE(STRIDE), .PADDING(PADDING),
    .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(ACT_WIDTH)
  ) u_act_reader (
    .clk(clk), .rst_n(rst_n),
    .m_addr(m_addr), .k_addr(k_addr),
    .clear_acc(clear_acc), .feed_valid(feed_valid),
    .wr_en(fmap_wr_en), .wr_addr(fmap_wr_addr), .wr_data(fmap_wr_data),
    .act_data(act_data), .fetch_k_addr(fetch_k_addr),
    .fmap_addr(fmap_addr), .addr_valid(addr_valid),
    .data_valid(bram_data_valid)
  );

  conv_weight_bram_reader #(
    .K_LEN(K_LEN), .N_TILES(N_TILES), .ADDR_WIDTH(ADDR_WIDTH)
  ) u_weight_reader (
    .clk(clk), .rst_n(rst_n),
    .n_base(n_base), .fetch_k_addr(fetch_k_addr),
    .clear_acc(clear_acc), .feed_valid(feed_valid),
    .wr_en(weight_wr_en), .wr_addr(weight_wr_addr), .wr_data(weight_wr_data),
    .wgt_row_packed(wgt_row_packed),
    .weight_word_addr(weight_word_addr), .data_valid(wgt_data_valid)
  );
endmodule
