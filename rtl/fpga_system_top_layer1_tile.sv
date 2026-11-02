`timescale 1ns / 1ps

// Fixed local Tile for ResNet18 layer1.0.conv1: output 8x8x8.
// Input 10x10x64 includes halo, kernel 3x3, stride 1, LOCAL padding 0.
// Global layer padding is 1; loader/TB supplies zeros outside the 56x56 image.
// M=64, N=8, K=576, N_TILES=1. Existing PE/controllers are unchanged.
// Activation: 6400 x 8 bit; weights: 576 x 64 bit; output: 64 x 256 bit
// plus 64 x 8-bit lane masks. Logical total 13120 bytes.
// Preload HWC activations and weights before start. Weight word k contains
// 8 output channels at bits lane*8 +: 8, k=(ky*3+kx)*64+ci.
// Output word m=oy*8+ox contains 8 INT32 lanes; host reads one lane per request.
// Global spatial origin/output-channel base are loader metadata, not RTL ports.
// Functional simulation is performed by the user; no AXI/DMA is added.
module fpga_system_top_layer1_tile
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
        .H_IN(10), .W_IN(10), .C_IN(64),
        .KH(3), .KW(3), .C_OUT(8),
        .STRIDE(1), .PADDING(0),
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
