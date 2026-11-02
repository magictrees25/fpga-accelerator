`timescale 1ns / 1ps

// P=1 layer2.0.downsample.0: local input 8x8x64, kernel 1x1/S2/P0.
// Local output 4x4x8: M=16, N=8, K=64, N_TILES=1.
// Global input 56x56x64 -> output 28x28x128: TB schedules 7x7x16=784 Tiles.
// Global origins/channel base are TB/software metadata, not RTL ports.
// HWC activation RAM: 4096 bytes. Weight RAM: 64 packed 8xINT8 words.
// Output RAM: 16 packed 8xINT32 words plus 16 lane masks.
// Preload while idle; start one clock; wait done; read all 128 INT32 lanes.
// Existing PE, MAC, P1 Controller and Reader modules are reused unchanged.
module fpga_system_top_layer2_downsample_tile_p1
  import pkg_accelerator::*;
#(
    parameter int ADDR_WIDTH = 16
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start,

    input  logic                  fmap_wr_en,
    input  logic [ADDR_WIDTH-1:0] fmap_wr_addr,
    input  logic [ACT_WIDTH-1:0]  fmap_wr_data,

    input  logic                             weight_wr_en,
    input  logic [ADDR_WIDTH-1:0]            weight_wr_addr,
    input  logic [WGT_WIDTH*ARRAY_COLS-1:0]  weight_wr_data,

    input  logic                     out_rd_en,
    input  logic [ADDR_WIDTH-1:0]    out_rd_addr,
    input  logic [$clog2(ARRAY_COLS)-1:0] out_rd_lane_sel,
    output logic signed [ACC_WIDTH-1:0] out_rd_data,
    output logic                     out_rd_lane_valid,
    output logic                     out_rd_valid,

    output logic busy,
    output logic done,
    output logic any_overflow
);
    fpga_system_top #(
        .H_IN(8), .W_IN(8), .C_IN(64),
        .KH(1), .KW(1), .C_OUT(8),
        .STRIDE(2), .PADDING(0),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_baseline (
        .clk(clk), .rst_n(rst_n), .start(start),
        .fmap_wr_en(fmap_wr_en),
        .fmap_wr_addr(fmap_wr_addr),
        .fmap_wr_data(fmap_wr_data),
        .weight_wr_en(weight_wr_en),
        .weight_wr_addr(weight_wr_addr),
        .weight_wr_data(weight_wr_data),
        .out_rd_en(out_rd_en),
        .out_rd_addr(out_rd_addr),
        .out_rd_lane_sel(out_rd_lane_sel),
        .out_rd_data(out_rd_data),
        .out_rd_lane_valid(out_rd_lane_valid),
        .out_rd_valid(out_rd_valid),
        .busy(busy), .done(done), .any_overflow(any_overflow)
    );
endmodule
