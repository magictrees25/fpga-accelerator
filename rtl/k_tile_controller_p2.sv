
`timescale 1ns / 1ps

// P5-B2-5A: K Controller timing optimization.
//
// Optimization:
//   1. Replace combinational k_idx + 2 with registered next_fetch_k.
//   2. Replace next_k < k_len_saved with pairs_remaining != 0.
//   3. Preserve the original one-cycle paired Reader response protocol.
//   4. Preserve the original FEED / DRAIN cycle schedule.
//
// k_len counts ELEMENTS, not pairs.
// Positive even K only. No backpressure.

module k_tile_controller_p2
  import pkg_accelerator::*;
#(
  parameter int K_ADDR_WIDTH = 16,
  parameter int DRAIN_CYCLES = 13
)(
  input logic clk,
  input logic rst_n,
  input logic start,

  input logic [K_ADDR_WIDTH-1:0] k_len,

  input logic [15:0] act_pair_packed,
  input logic [1:0] act_rsp_valid,

  input logic [127:0] wgt_pair_packed,
  input logic [1:0] wgt_rsp_valid,

  output logic pair_req_en,
  output logic [K_ADDR_WIDTH-1:0] fetch_k_base,
  output logic [K_ADDR_WIDTH-1:0] k_base,

  output logic array_enable,
  output logic feed_valid,
  output logic clear_acc,
  output logic valid_reset,
  output logic data_valid_in,

  output logic [63:0] act_in_packed,
  output logic [127:0] wgt_pair_in_packed,

  input logic [255:0] acc_out_packed,

  output logic busy,
  output logic done,
  output logic [255:0] result_packed
);

  typedef enum logic [2:0] {
    S_IDLE,
    S_CLEAR,
    S_FEED,
    S_DRAIN,
    S_DONE
  } state_t;

  state_t state;

  logic [K_ADDR_WIDTH-1:0] k_idx;
  logic [K_ADDR_WIDTH-1:0] k_len_saved;

  // Registered next request address.
  logic [K_ADDR_WIDTH-1:0] next_fetch_k;

  // Number of K-pairs remaining AFTER the current pair.
  logic [K_ADDR_WIDTH-2:0] pairs_remaining;

  logic [$clog2(DRAIN_CYCLES+1)-1:0] drain_cnt;

  assign k_base = k_idx;

  assign busy =
      (state == S_CLEAR) ||
      (state == S_FEED)  ||
      (state == S_DRAIN);

  assign done = (state == S_DONE);

  assign feed_valid = (state == S_FEED);

  // ----------------------------------------------------------
  // Combinational control
  // ----------------------------------------------------------

  always_comb begin

    array_enable = 1'b0;
    clear_acc = 1'b0;
    valid_reset = 1'b0;
    data_valid_in = 1'b0;

    pair_req_en = 1'b0;
    fetch_k_base = k_idx;

    act_in_packed = '0;
    wgt_pair_in_packed = '0;

    array_enable =
        (state == S_CLEAR) ||
        (state == S_FEED)  ||
        (state == S_DRAIN);

    clear_acc = (state == S_CLEAR);
    valid_reset = clear_acc;

    data_valid_in =
        feed_valid && (k_idx == 0);

    // First K-pair request.
    if (state == S_CLEAR) begin

      pair_req_en = 1'b1;
      fetch_k_base = '0;

    end

    // Current pair response + next pair request.
    else if (state == S_FEED) begin

      act_in_packed[
        (int'(k_idx) % 8)*8 +: 8
      ] = act_pair_packed[7:0];

      act_in_packed[
        ((int'(k_idx)+1) % 8)*8 +: 8
      ] = act_pair_packed[15:8];

      wgt_pair_in_packed = wgt_pair_packed;

      // No wide increment-and-compare path.
      if (pairs_remaining != 0) begin

        pair_req_en = 1'b1;
        fetch_k_base = next_fetch_k;

      end

    end

  end

  // ----------------------------------------------------------
  // Sequential control
  // ----------------------------------------------------------

  always_ff @(posedge clk or negedge rst_n) begin

    if (!rst_n) begin

      state <= S_IDLE;

      k_idx <= '0;
      k_len_saved <= '0;

      next_fetch_k <= '0;
      pairs_remaining <= '0;

      drain_cnt <= '0;
      result_packed <= '0;

    end

    else begin

      case (state)

        S_IDLE: begin

          if (start && k_len != 0 && !k_len[0]) begin

            k_len_saved <= k_len;

            k_idx <= '0;

            state <= S_CLEAR;

          end

        end

        S_CLEAR: begin

          k_idx <= '0;

          // First pair is requested in S_CLEAR.
          // The next request will start at K=2.
          next_fetch_k <= K_ADDR_WIDTH'(2);

          // Total pairs = k_len / 2.
          // One pair is already requested in S_CLEAR.
          pairs_remaining <=
              k_len_saved[K_ADDR_WIDTH-1:1] - 1'b1;

          drain_cnt <= '0;

          state <= S_FEED;

        end

        S_FEED: begin

          if (pairs_remaining != 0) begin

            // Advance to the next pair.
            k_idx <= next_fetch_k;

            next_fetch_k <=
                next_fetch_k + K_ADDR_WIDTH'(2);

            pairs_remaining <=
                pairs_remaining - 1'b1;

          end

          else begin

            drain_cnt <= '0;

            state <= S_DRAIN;

          end

        end

        S_DRAIN: begin

          if (drain_cnt == DRAIN_CYCLES-1) begin

            result_packed <= acc_out_packed;

            state <= S_DONE;

          end

          else begin

            drain_cnt <= drain_cnt + 1'b1;

          end

        end

        S_DONE: begin

          state <= S_IDLE;

        end

        default: begin

          state <= S_IDLE;

        end

      endcase

    end

  end

  // ----------------------------------------------------------
  // Simulation checks
  // ----------------------------------------------------------

  // synthesis translate_off

  initial begin

    if (K_ADDR_WIDTH < 2 ||
        K_ADDR_WIDTH > 30 ||
        DRAIN_CYCLES < 13)

      $fatal(1,
        "k_tile_controller_p2: unsupported width or unsafe DRAIN");

  end

  always @(posedge clk) begin

    if (rst_n) begin

      if (state == S_IDLE && start &&
          ($isunknown(k_len) ||
           k_len == 0 ||
           k_len[0]))

        $fatal(1,
          "k_tile_controller_p2 requires positive even k_len");

      if (feed_valid &&
          (act_rsp_valid !== 2'b11 ||
           wgt_rsp_valid !== 2'b11))

        $fatal(1,
          "k_tile_controller_p2 missing paired response at K=%0d",
          k_idx);

      if (state == S_FEED &&
          pair_req_en &&
          (fetch_k_base >= k_len_saved ||
           fetch_k_base[0]))

        $fatal(1,
          "k_tile_controller_p2 invalid next K request");

    end

  end

  // synthesis translate_on

endmodule
