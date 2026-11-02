`timescale 1ns / 1ps

// P2 M/N controller: same outer scheduling, paired K supply internally.
// First version N=8 only; synthetic controller TB also exercises short even K.
// mem_clear_acc/mem_feed_valid are observation outputs, not Reader requests.
module gemm_mn_controller_mem_p2
  import pkg_accelerator::*;
#(
  parameter int ADDR_WIDTH = 16
)(
  input logic clk,
  input logic rst_n,
  input logic start,
  input logic [ADDR_WIDTH-1:0] m_len,
  input logic [ADDR_WIDTH-1:0] n_len,
  input logic [ADDR_WIDTH-1:0] k_len,

  output logic [ADDR_WIDTH-1:0] m_addr,
  output logic [ADDR_WIDTH-1:0] n_base,
  output logic [ADDR_WIDTH-1:0] k_base, fetch_k_base,
  output logic pair_req_en,
  input logic [15:0] act_pair_packed,
  input logic [1:0] act_rsp_valid,
  input logic [127:0] wgt_pair_packed,
  input logic [1:0] wgt_rsp_valid,

  // Explicit ports: used by synchronous memory prefetch modules.
  output logic mem_clear_acc,
  output logic mem_feed_valid,

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

  typedef enum logic [2:0] {G_IDLE,G_START_K,G_WAIT_K,G_WRITE,G_DONE} g_state_t;
  g_state_t state;
  logic [ADDR_WIDTH-1:0] m_idx, n_tile_idx, write_word_idx;
  logic [ADDR_WIDTH-1:0] m_len_saved, n_len_saved, k_len_saved, n_tiles_saved;
  logic k_start, k_done, k_busy;
  logic array_enable, feed_valid, clear_acc, valid_reset, data_valid_in;
  logic data_valid_out;
  logic [ACT_WIDTH*ARRAY_ROWS-1:0] act_in_packed;
  logic [127:0] wgt_pair_in_packed;
  logic [ACC_WIDTH*ARRAY_COLS-1:0] acc_out_packed, k_result_packed;

  assign m_addr = m_idx;
  assign n_base = n_tile_idx * ARRAY_COLS;
  assign k_start = (state == G_START_K);
  assign write_valid = (state == G_WRITE);
  assign write_addr = write_word_idx;
  assign write_m_addr = m_idx;
  assign write_n_base = n_base;
  assign write_data = k_result_packed;
  assign busy = (state == G_START_K || state == G_WAIT_K || state == G_WRITE);
  assign done = (state == G_DONE);
  assign mem_clear_acc = clear_acc;
  assign mem_feed_valid = feed_valid;

  always_comb begin
    write_mask = '0;
    for (int lane=0; lane<ARRAY_COLS; lane++)
      if (int'(n_base)+lane < int'(n_len_saved))
        write_mask[lane] = 1'b1;
  end

  k_tile_controller_p2 #(.K_ADDR_WIDTH(ADDR_WIDTH)) u_kctrl (
    .clk(clk), .rst_n(rst_n),
    .start(k_start), .k_len(k_len_saved),
    .k_base(k_base), .fetch_k_base(fetch_k_base), .pair_req_en(pair_req_en),
    .act_pair_packed(act_pair_packed), .act_rsp_valid(act_rsp_valid),
    .wgt_pair_packed(wgt_pair_packed), .wgt_rsp_valid(wgt_rsp_valid),
    .array_enable(array_enable), .feed_valid(feed_valid),
    .clear_acc(clear_acc), .valid_reset(valid_reset),
    .data_valid_in(data_valid_in), .act_in_packed(act_in_packed),
    .wgt_pair_in_packed(wgt_pair_in_packed), .acc_out_packed(acc_out_packed),
    .busy(k_busy), .done(k_done), .result_packed(k_result_packed)
  );

  systolic_array_p2 u_array (
    .clk(clk), .rst_n(rst_n),
    .enable(array_enable), .clear_acc(clear_acc), .valid_reset(valid_reset),
    .act_in_packed(act_in_packed), .wgt_pair_in_packed(wgt_pair_in_packed),
    .data_valid_in(data_valid_in), .data_valid_out(data_valid_out),
    .acc_out_packed(acc_out_packed), .any_overflow(any_overflow)
  );

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= G_IDLE;
      m_idx <= '0;
      n_tile_idx <= '0;
      write_word_idx <= '0;
      m_len_saved <= '0;
      n_len_saved <= '0;
      k_len_saved <= '0;
      n_tiles_saved <= '0;
    end else begin
      case (state)
        G_IDLE: begin
          if (start && m_len != 0 && n_len != 0 && k_len != 0) begin
            m_len_saved <= m_len;
            n_len_saved <= n_len;
            k_len_saved <= k_len;
            n_tiles_saved <= (int'(n_len)+ARRAY_COLS-1) / ARRAY_COLS;
            m_idx <= '0;
            n_tile_idx <= '0;
            write_word_idx <= '0;
            state <= G_START_K;
          end
        end
        G_START_K: state <= G_WAIT_K;
        G_WAIT_K: if (k_done) state <= G_WRITE;
        G_WRITE: begin
          write_word_idx <= write_word_idx + 1'b1;
          if (n_tile_idx == n_tiles_saved - 1'b1) begin
            if (m_idx == m_len_saved - 1'b1)
              state <= G_DONE;
            else begin
              m_idx <= m_idx + 1'b1;
              n_tile_idx <= '0;
              state <= G_START_K;
            end
          end else begin
            n_tile_idx <= n_tile_idx + 1'b1;
            state <= G_START_K;
          end
        end
        G_DONE: state <= G_IDLE;
        default: state <= G_IDLE;
      endcase
    end
  end
  // synthesis translate_off
  always @(posedge clk) if (rst_n && state==G_IDLE && start) begin
    if ($isunknown({m_len,n_len,k_len}) || m_len==0 || m_len>64 ||
        n_len!=8 || k_len==0 || k_len[0])
      $fatal(1,"gemm_mn_controller_mem_p2: require 1<=M<=64, N=8, positive even K");
  end
  // synthesis translate_on
endmodule

