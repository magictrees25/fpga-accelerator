`timescale 1ns / 1ps

// P2 fixed local Tile: 10x10x64 halo input -> 8x8x8 output, K576.
// Same external preload and lane-read interface as the P1 Tile wrapper.
// Only the internal data supply/compute is P2. Local padding remains zero.
module fpga_system_top_layer1_tile_p2 #(
  parameter int ADDR_WIDTH=16
)(
  input logic clk,rst_n,start,
  input logic fmap_wr_en,
  input logic [ADDR_WIDTH-1:0] fmap_wr_addr,
  input logic [7:0] fmap_wr_data,
  input logic weight_wr_en,
  input logic [ADDR_WIDTH-1:0] weight_wr_addr,
  input logic [63:0] weight_wr_data,
  input logic out_rd_en,
  input logic [ADDR_WIDTH-1:0] out_rd_addr,
  input logic [2:0] out_rd_lane_sel,
  output logic signed [31:0] out_rd_data,
  output logic out_rd_lane_valid,out_rd_valid,
  output logic busy,done,any_overflow
);
  logic write_valid;
  logic [ADDR_WIDTH-1:0] write_addr,write_m_addr,write_n_base;
  logic [7:0] write_mask;
  logic [255:0] write_data,packed_rd_data;
  logic [7:0] packed_rd_mask;
  logic packed_rd_valid;
  logic [2:0] lane_sel_q;

  conv_accelerator_top_p2 #(.ADDR_WIDTH(ADDR_WIDTH)) u_compute (
    .clk(clk),.rst_n(rst_n),.start(start),
    .fmap_wr_en(fmap_wr_en),.fmap_wr_addr(fmap_wr_addr),.fmap_wr_data(fmap_wr_data),
    .weight_wr_en(weight_wr_en),.weight_wr_addr(weight_wr_addr),.weight_wr_data(weight_wr_data),
    .write_valid(write_valid),.write_addr(write_addr),
    .write_m_addr(write_m_addr),.write_n_base(write_n_base),
    .write_mask(write_mask),.write_data(write_data),
    .busy(busy),.done(done),.any_overflow(any_overflow)
  );
  output_bram #(.ADDR_WIDTH(ADDR_WIDTH),.DATA_WIDTH(32),.LANES(8),.DEPTH(64)) u_out_mem (
    .clk(clk),.rst_n(rst_n),
    .wr_en(write_valid),.wr_addr(write_addr),.wr_data(write_data),.wr_mask(write_mask),
    .rd_en(out_rd_en),.rd_addr(out_rd_addr),
    .rd_data(packed_rd_data),.rd_mask(packed_rd_mask),.rd_valid(packed_rd_valid)
  );
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) lane_sel_q<=0;
    else if (out_rd_en) lane_sel_q<=out_rd_lane_sel;
  assign out_rd_data=$signed(packed_rd_data[lane_sel_q*32 +: 32]);
  assign out_rd_valid=packed_rd_valid;
  assign out_rd_lane_valid=packed_rd_valid && packed_rd_mask[lane_sel_q];
endmodule

