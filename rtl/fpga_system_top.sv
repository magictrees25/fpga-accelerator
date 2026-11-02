`timescale 1ns / 1ps

// Interim FPGA-facing wrapper for the verified convolution baseline.
// Keep the 256-bit GEMM-result write bus INTERNAL; read one 32-bit lane
// at a time from the output BRAM instead of using 256 external pins.
// This is a bring-up interface, NOT an AXI interface or board pin plan.
module fpga_system_top
  import pkg_accelerator::*;
#(
    parameter int H_IN       = 5,
    parameter int W_IN       = 5,
    parameter int C_IN       = 3,
    parameter int KH         = 3,
    parameter int KW         = 3,
    parameter int C_OUT      = 4,
    parameter int STRIDE     = 1,
    parameter int PADDING    = 0,
    parameter int ADDR_WIDTH = 16
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start,

    // Input feature map: byte-wise preload.
    input  logic                  fmap_wr_en,
    input  logic [ADDR_WIDTH-1:0] fmap_wr_addr,
    input  logic [ACT_WIDTH-1:0]  fmap_wr_data,

    // Weight memory: one packed 8xINT8 word per write.
    input  logic                             weight_wr_en,
    input  logic [ADDR_WIDTH-1:0]            weight_wr_addr,
    input  logic [WGT_WIDTH*ARRAY_COLS-1:0]  weight_wr_data,

    // Output memory: request ONE lane (32-bit result) from one packed word.
    // Read address + lane are sampled on the SAME posedge as out_rd_en.
    // out_rd_valid and corresponding data update just after that posedge.
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
    // Intermediate signals stay inside the FPGA and use no package pins.
    logic [ACC_WIDTH*ARRAY_COLS-1:0] packed_rd_data;
    logic [ARRAY_COLS-1:0]           packed_rd_mask;
    logic                            packed_rd_valid;
    logic [$clog2(ARRAY_COLS)-1:0]   lane_sel_q;

    conv_accelerator_top_outmem #(
        .H_IN(H_IN), .W_IN(W_IN), .C_IN(C_IN),
        .KH(KH), .KW(KW), .C_OUT(C_OUT),
        .STRIDE(STRIDE), .PADDING(PADDING),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_core (
        .clk(clk), .rst_n(rst_n), .start(start),
        .fmap_wr_en(fmap_wr_en),
        .fmap_wr_addr(fmap_wr_addr),
        .fmap_wr_data(fmap_wr_data),
        .weight_wr_en(weight_wr_en),
        .weight_wr_addr(weight_wr_addr),
        .weight_wr_data(weight_wr_data),
        .out_rd_en(out_rd_en),
        .out_rd_addr(out_rd_addr),
        .out_rd_data(packed_rd_data),
        .out_rd_mask(packed_rd_mask),
        .out_rd_valid(packed_rd_valid),
        // Internal diagnostic write bus: no top-level IO pins.
        .write_valid(),
        .write_addr(),
        .write_m_addr(),
        .write_n_base(),
        .write_mask(),
        .write_data(),
        .busy(busy), .done(done), .any_overflow(any_overflow)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            lane_sel_q <= '0;
        else if (out_rd_en)
            lane_sel_q <= out_rd_lane_sel;
    end

    // BRAM's 256-bit registered word is selected internally after read.
    // Lane selection is registered with the read request, so it cannot
    // accidentally refer to the next request's lane.
    assign out_rd_data       = $signed(packed_rd_data[lane_sel_q*ACC_WIDTH +: ACC_WIDTH]);
    assign out_rd_valid      = packed_rd_valid;
    assign out_rd_lane_valid = packed_rd_valid && packed_rd_mask[lane_sel_q];
endmodule
