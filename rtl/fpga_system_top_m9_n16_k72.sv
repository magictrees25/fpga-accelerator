`timescale 1ns / 1ps

// Configuration wrapper for the M=9, N=16, K=72 convolution experiment.
// Input HWC: 5x5x8. Kernel: 3x3, stride=1, padding=0. Output: 3x3x16.
// Keeps fpga_system_top's legacy defaults and its narrow external interface.
// This is synthesizable RTL only; functional simulation is pending.
//
// Derived by the existing parameter chain:
//   M_LEN=9, K_LEN=72, N_TILES=2
//   activation RAM: 200 x 8 bits
//   weight RAM:     144 x 64 bits; word=ntile*72+k
//   output RAM:      18 x 256 bits, plus 18 x 8-bit lane masks
//
// Preload all activation and weight words BEFORE pulsing start.
// Weight lane l at word t*72+k represents B[k,t*8+l] in bits l*8 +: 8.
// Output word m*2+t, lane l represents C[m,t*8+l], signed INT32.
// Use the existing 32-bit lane read interface after done.
//
// This wrapper adds one hierarchy level:
//   u_baseline.u_core.u_compute (compute hierarchy)
//   u_baseline.u_core.u_out_mem (output memory)
// The original fpga_system_top hierarchy itself is unchanged.
// ADDR_WIDTH=16 retains the existing pin widths and covers this configuration.
module fpga_system_top_m9_n16_k72
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
        .H_IN(5), .W_IN(5), .C_IN(8),
        .KH(3), .KW(3), .C_OUT(16),
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
