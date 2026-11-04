`timescale 1ns / 1ps

// Fixed P2 integration: only activation Reader address generation is optimized.
// Weight Reader, K/MN Controllers and array are reused unchanged.
module conv_accelerator_top_p2_opt
  import pkg_accelerator::*;
#(parameter int ADDR_WIDTH=16)(
  input logic clk,rst_n,start,
  input logic fmap_wr_en,
  input logic [ADDR_WIDTH-1:0] fmap_wr_addr,
  input logic [7:0] fmap_wr_data,
  input logic weight_wr_en,
  input logic [ADDR_WIDTH-1:0] weight_wr_addr,
  input logic [63:0] weight_wr_data,
  output logic write_valid,
  output logic [ADDR_WIDTH-1:0] write_addr,write_m_addr,write_n_base,
  output logic [7:0] write_mask,
  output logic [255:0] write_data,
  output logic busy,done,any_overflow
);
  localparam int M_LEN=64,N_LEN=8,K_LEN=576;
  logic [ADDR_WIDTH-1:0] m_addr,n_base,k_base,fetch_k_base;
  logic pair_req_en,clear_acc,feed_valid;
  logic [15:0] act_pair_packed;
  logic [127:0] wgt_pair_packed;
  logic [1:0] act_rsp_valid,wgt_rsp_valid;

  gemm_mn_controller_mem_p2 #(.ADDR_WIDTH(ADDR_WIDTH)) u_ctrl (
    .clk(clk),.rst_n(rst_n),.start(start),
    .m_len(ADDR_WIDTH'(M_LEN)),.n_len(ADDR_WIDTH'(N_LEN)),.k_len(ADDR_WIDTH'(K_LEN)),
    .m_addr(m_addr),.n_base(n_base),.k_base(k_base),.fetch_k_base(fetch_k_base),
    .pair_req_en(pair_req_en),
    .act_pair_packed(act_pair_packed),.act_rsp_valid(act_rsp_valid),
    .wgt_pair_packed(wgt_pair_packed),.wgt_rsp_valid(wgt_rsp_valid),
    .mem_clear_acc(clear_acc),.mem_feed_valid(feed_valid),
    .write_valid(write_valid),.write_addr(write_addr),
    .write_m_addr(write_m_addr),.write_n_base(write_n_base),
    .write_mask(write_mask),.write_data(write_data),
    .busy(busy),.done(done),.any_overflow(any_overflow)
  );
  conv_bram_reader_p2_opt #(.ADDR_WIDTH(ADDR_WIDTH)) u_act_reader (
    .clk(clk),.rst_n(rst_n),.m_addr(m_addr),
    .pair_req_en(pair_req_en),.fetch_k_base(fetch_k_base),
    .wr_en(fmap_wr_en),.wr_addr(fmap_wr_addr),.wr_data(fmap_wr_data),
    .act_pair_packed(act_pair_packed),.act_rsp_valid(act_rsp_valid)
  );
  conv_weight_bram_reader_p2_opt #(.ADDR_WIDTH(ADDR_WIDTH)) u_weight_reader (  //conv_weight_bram_reader_p2 #(.ADDR_WIDTH(ADDR_WIDTH)) u_weight_reader (
    .clk(clk),.rst_n(rst_n),.n_base(n_base),
    .pair_req_en(pair_req_en),.fetch_k_base(fetch_k_base),
    .wr_en(weight_wr_en),.wr_addr(weight_wr_addr),.wr_data(weight_wr_data),
    .wgt_pair_packed(wgt_pair_packed),.wgt_rsp_valid(wgt_rsp_valid)
  );
  // synthesis translate_off
  always @(posedge clk) if (rst_n && busy && (fmap_wr_en || weight_wr_en))
    $fatal(1,"conv_accelerator_top_p2_opt: preload while busy");
  // synthesis translate_on
endmodule


