`timescale 1ns / 1ps

// GEMM scheduler (M and N tiles), built on the verified K controller.
// One start pulse computes C[M x N] using a fixed 8x8 KxN PE array.
//   - Each K-controller run computes C[m, n_base +: up to 8].
//   - N is processed in tiles of 8 columns; M positions are serialized.
//   - K length is handled inside k_tile_controller (including K > 8).
//
// This bring-up version uses combinational (zero-latency) read ports:
//    A[m_addr][k_addr] -> act_data
//    B[k_addr][n_base + lane] -> wgt_row_packed[lane]
// External output memory samples write_* at the rising edge when write_valid=1.
// write_addr counts consecutive 8-element output words in row-major order.
// write_mask is meaningful on a partial final N tile.
module gemm_mn_controller
  import pkg_accelerator::*;
#(
  parameter int ADDR_WIDTH = 16
)(
  input  logic clk,
  input  logic rst_n,
  input  logic start,
  input  logic [ADDR_WIDTH-1:0] m_len,
  input  logic [ADDR_WIDTH-1:0] n_len,
  input  logic [ADDR_WIDTH-1:0] k_len,

  // Input RAM read-address interface (combinational data returned this cycle)
  output logic [ADDR_WIDTH-1:0] m_addr,
  output logic [ADDR_WIDTH-1:0] n_base,
  output logic [ADDR_WIDTH-1:0] k_addr,
  input  logic signed [ACT_WIDTH-1:0] act_data,
  input  logic [WGT_WIDTH*ARRAY_COLS-1:0] wgt_row_packed,

  // Output RAM write interface: up to 8 outputs per write request
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

  typedef enum logic [2:0] {
    G_IDLE,
    G_START_K,
    G_WAIT_K,
    G_WRITE,
    G_DONE
  } g_state_t;

  g_state_t state;
  logic [ADDR_WIDTH-1:0] m_idx;
  logic [ADDR_WIDTH-1:0] n_tile_idx;
  logic [ADDR_WIDTH-1:0] write_word_idx;
  logic [ADDR_WIDTH-1:0] m_len_saved, n_len_saved, k_len_saved;
  logic [ADDR_WIDTH-1:0] n_tiles_saved;

  logic k_start, k_done, k_busy;
  logic array_enable, feed_valid, clear_acc, valid_reset, data_valid_in;
  logic data_valid_out;
  logic [ACT_WIDTH*ARRAY_ROWS-1:0] act_in_packed;
  logic signed [WGT_WIDTH-1:0] wgt_in [ARRAY_COLS];
  logic [ACC_WIDTH*ARRAY_COLS-1:0] acc_out_packed;
  logic [ACC_WIDTH*ARRAY_COLS-1:0] k_result_packed;

  assign m_addr       = m_idx;
  assign n_base       = n_tile_idx * ARRAY_COLS;
  assign k_start      = (state == G_START_K);
  assign write_valid  = (state == G_WRITE);
  assign write_addr   = write_word_idx;
  assign write_m_addr = m_idx;
  assign write_n_base = n_base;
  assign write_data   = k_result_packed;
  assign busy         = (state == G_START_K || state == G_WAIT_K || state == G_WRITE);
  assign done         = (state == G_DONE);

  // For N=10, the final N-tile writes only lanes 0 and 1, for example.
  always_comb begin
    write_mask = '0;
    for (int lane = 0; lane < ARRAY_COLS; lane = lane + 1)
      if ((int'(n_base) + lane) < int'(n_len_saved))
        write_mask[lane] = 1'b1;
  end

  // Reuse the already-tested K controller unchanged.
  k_tile_controller #(.K_ADDR_WIDTH(ADDR_WIDTH)) u_kctrl (
    .clk(clk), .rst_n(rst_n),
    .start(k_start), .k_len(k_len_saved),
    .k_addr(k_addr), .act_data(act_data), .wgt_row_packed(wgt_row_packed),
    .array_enable(array_enable), .feed_valid(feed_valid),
    .clear_acc(clear_acc), .valid_reset(valid_reset),
    .data_valid_in(data_valid_in), .act_in_packed(act_in_packed),
    .wgt_in(wgt_in), .acc_out_packed(acc_out_packed),
    .busy(k_busy), .done(k_done), .result_packed(k_result_packed)
  );

  systolic_array u_array (
    .clk(clk), .rst_n(rst_n),
    .enable(array_enable), .clear_acc(clear_acc), .valid_reset(valid_reset),
    .act_in_packed(act_in_packed), .wgt_in(wgt_in),
    .data_valid_in(data_valid_in), .data_valid_out(data_valid_out),
    .acc_out_packed(acc_out_packed), .any_overflow(any_overflow)
  );

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state          <= G_IDLE;
      m_idx          <= '0;
      n_tile_idx     <= '0;
      write_word_idx <= '0;
      m_len_saved    <= '0;
      n_len_saved    <= '0;
      k_len_saved    <= '0;
      n_tiles_saved  <= '0;
    end else begin
      case (state)
        G_IDLE: begin
          if (start && m_len != 0 && n_len != 0 && k_len != 0) begin
            m_len_saved    <= m_len;
            n_len_saved    <= n_len;
            k_len_saved    <= k_len;
            n_tiles_saved  <= (int'(n_len) + ARRAY_COLS - 1) / ARRAY_COLS;
            m_idx          <= '0;
            n_tile_idx     <= '0;
            write_word_idx <= '0;
            state          <= G_START_K;
          end
        end
        G_START_K: state <= G_WAIT_K;  // pulse to K controller, once per (M,N tile)
        G_WAIT_K: begin
          if (k_done) state <= G_WRITE;
        end
        G_WRITE: begin
          // Current C[m, n_base .. n_base+7] is sampled by external RAM.
          // Only after that edge do we advance M/N and restart K (which clears PE).
          write_word_idx <= write_word_idx + 1'b1;
          if (n_tile_idx == n_tiles_saved - 1'b1) begin
            if (m_idx == m_len_saved - 1'b1)
              state <= G_DONE;
            else begin
              m_idx      <= m_idx + 1'b1;
              n_tile_idx <= '0;
              state      <= G_START_K;
            end
          end else begin
            n_tile_idx <= n_tile_idx + 1'b1;
            state      <= G_START_K;
          end
        end
        G_DONE: state <= G_IDLE;
        default: state <= G_IDLE;
      endcase
    end
  end
endmodule
