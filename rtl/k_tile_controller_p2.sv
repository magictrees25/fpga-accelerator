`timescale 1ns / 1ps

// Two K elements per FEED; one-cycle synchronous paired Reader responses.
// k_len counts ELEMENTS, not pairs. Positive even K only, no backpressure.
module k_tile_controller_p2
  import pkg_accelerator::*;
#(
  parameter int K_ADDR_WIDTH=16,
  parameter int DRAIN_CYCLES=13
)(
  input logic clk, rst_n, start,
  input logic [K_ADDR_WIDTH-1:0] k_len,
  input logic [15:0] act_pair_packed,
  input logic [1:0] act_rsp_valid,
  input logic [127:0] wgt_pair_packed,
  input logic [1:0] wgt_rsp_valid,
  output logic pair_req_en,
  output logic [K_ADDR_WIDTH-1:0] fetch_k_base, k_base,
  output logic array_enable, feed_valid, clear_acc, valid_reset, data_valid_in,
  output logic [63:0] act_in_packed,
  output logic [127:0] wgt_pair_in_packed,
  input logic [255:0] acc_out_packed,
  output logic busy, done,
  output logic [255:0] result_packed
);
  typedef enum logic [2:0] {S_IDLE,S_CLEAR,S_FEED,S_DRAIN,S_DONE} state_t;
  state_t state;
  logic [K_ADDR_WIDTH-1:0] k_idx, k_len_saved;
  logic [$clog2(DRAIN_CYCLES+1)-1:0] drain_cnt;
  logic [K_ADDR_WIDTH:0] next_k;
  assign next_k={1'b0,k_idx}+2;
  assign k_base=k_idx;
  assign busy=(state==S_CLEAR || state==S_FEED || state==S_DRAIN);
  assign done=(state==S_DONE);
  assign feed_valid=(state==S_FEED);
  always_comb begin
    array_enable=(state==S_CLEAR || state==S_FEED || state==S_DRAIN);
    clear_acc=(state==S_CLEAR);
    valid_reset=clear_acc;
    data_valid_in=feed_valid && (k_idx==0);
    pair_req_en=0;
    fetch_k_base=k_idx;
    act_in_packed='0;
    wgt_pair_in_packed='0;
    if (state==S_CLEAR) begin
      pair_req_en=1;
      fetch_k_base=0;
    end else if (state==S_FEED) begin
      // Response for CURRENT pair; never use fetch_k_base for injection.
      act_in_packed[(int'(k_idx)%8)*8 +: 8]=act_pair_packed[7:0];
      act_in_packed[((int'(k_idx)+1)%8)*8 +: 8]=act_pair_packed[15:8];
      wgt_pair_in_packed=wgt_pair_packed;
      if (next_k < {1'b0,k_len_saved}) begin
        pair_req_en=1;
        fetch_k_base=next_k[K_ADDR_WIDTH-1:0];
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state<=S_IDLE; k_idx<=0; k_len_saved<=0; drain_cnt<=0; result_packed<=0;
    end else case (state)
      S_IDLE: if (start && k_len!=0 && !k_len[0]) begin
        k_len_saved<=k_len; k_idx<=0; state<=S_CLEAR;
      end
      S_CLEAR: begin k_idx<=0; drain_cnt<=0; state<=S_FEED; end
      S_FEED: begin
        if (next_k >= {1'b0,k_len_saved}) begin drain_cnt<=0; state<=S_DRAIN; end
        else k_idx<=next_k[K_ADDR_WIDTH-1:0];
      end
      S_DRAIN: begin
        if (drain_cnt==DRAIN_CYCLES-1) begin
          result_packed<=acc_out_packed;
          state<=S_DONE;
        end else drain_cnt<=drain_cnt+1'b1;
      end
      S_DONE: state<=S_IDLE;
      default: state<=S_IDLE;
    endcase
  end
  // synthesis translate_off
  initial begin
    if (K_ADDR_WIDTH<2 || K_ADDR_WIDTH>30 || DRAIN_CYCLES<13)
      $fatal(1,"k_tile_controller_p2: unsupported width or unsafe DRAIN");
  end
  always @(posedge clk) if (rst_n) begin
    if (state==S_IDLE && start &&
        ($isunknown(k_len) || k_len==0 || k_len[0]))
      $fatal(1,"k_tile_controller_p2 requires positive even k_len");
    if (feed_valid && (act_rsp_valid!==2'b11 || wgt_rsp_valid!==2'b11))
      $fatal(1,"k_tile_controller_p2 missing paired response at K=%0d",k_idx);
  end
  // synthesis translate_on
endmodule

