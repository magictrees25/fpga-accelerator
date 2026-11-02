`timescale 1ns / 1ps

// P1 ResNet18 conv1 Tile: halo input 21x21x3, kernel 7x7, stride 2, LOCAL P0.
// Global input 224x224x3, S2/P3 -> 112x112x64; output Tile 8x8x8.
// Loader origin = (2*global_oy0-3, 2*global_ox0-3), out-of-image bytes zero.
// M=64,N=8,K=147,N_TILES=1. Existing PE/MAC/controllers/readers unchanged.
// K144..146 use PE rows 0..2; no padded K147..151 computation is requested.
// HWC activation: 1323x8 bit; weight: 147x64 bit; output: 64x256 + 64x8 mask.
// Weight word k: k=(ky*7+kx)*3+c, lane weights occupy lane*8 +: 8.
// Global 14x14x8 scheduling and output stitching are performed by TB/software.
module fpga_system_top_conv1_7x7_tile_p1
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
        .H_IN(21), .W_IN(21), .C_IN(3),
        .KH(7), .KW(7), .C_OUT(8),
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
