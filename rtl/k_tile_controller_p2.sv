
`timescale 1ns / 1ps

// ===========================================================================
// Module: k_tile_controller_p2.sv
//
// P5-B2-5A + P6-A update:
//
// P5-B2-5A:
//   1. Registered next_fetch_k.
//   2. pairs_remaining counter.
//   3. Preserve original paired Reader response protocol.
//   4. Preserve original FEED / DRAIN schedule.
//
// P6-A:
//   1. Separate accumulator clear and pipeline clear.
//   2. clear_acc_on_start determines whether this K-Tile
//      starts a new accumulation or continues a previous one.
//   3. clear_pipe is asserted whenever entering S_CLEAR.
//   4. valid_reset follows clear_pipe.
//
// K length counts ELEMENTS, not pairs.
// Positive even K only. No backpressure.
//
// ===========================================================================

module k_tile_controller_p2
  import pkg_accelerator::*;
#(
  parameter int K_ADDR_WIDTH = 16,
  parameter int DRAIN_CYCLES = 13
)(
  input logic clk,
  input logic rst_n,
  input logic start,

  // P6-A: whether this K-Tile starts a new accumulation.
  // 1: clear INT32 accumulator.
  // 0: preserve previous INT32 partial sum.
  input logic clear_acc_on_start,

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

  // Independent clear controls
  output logic clear_acc,
  output logic clear_pipe,

  output logic valid_reset,
  output logic data_valid_in,

  output logic [63:0] act_in_packed,
  output logic [127:0] wgt_pair_in_packed,

  input logic [255:0] acc_out_packed,

  output logic busy,
  output logic done,
  output logic [255:0] result_packed
);

  // -------------------------------------------------------------------------
  // State Machine
  // -------------------------------------------------------------------------

  typedef enum logic [2:0] {
    S_IDLE,
    S_CLEAR,
    S_FEED,
    S_DRAIN,
    S_DONE
  } state_t;

  state_t state;


  // -------------------------------------------------------------------------
  // Internal Registers
  // -------------------------------------------------------------------------

  logic [K_ADDR_WIDTH-1:0] k_idx;
  logic [K_ADDR_WIDTH-1:0] k_len_saved;

  // P6-A: save accumulator clear decision at start.
  logic clear_acc_saved;

  // Registered next request address.
  logic [K_ADDR_WIDTH-1:0] next_fetch_k;

  // Number of K-pairs remaining after current pair.
  logic [K_ADDR_WIDTH-2:0] pairs_remaining;

  logic [$clog2(DRAIN_CYCLES+1)-1:0] drain_cnt;


  // -------------------------------------------------------------------------
  // Basic Outputs
  // -------------------------------------------------------------------------

  assign k_base = k_idx;

  assign busy =
      (state == S_CLEAR) ||
      (state == S_FEED)  ||
      (state == S_DRAIN);

  assign done = (state == S_DONE);

  assign feed_valid = (state == S_FEED);


  // -------------------------------------------------------------------------
  // Combinational Control
  // -------------------------------------------------------------------------

  always_comb begin

    array_enable = 1'b0;

    clear_acc = 1'b0;
    clear_pipe = 1'b0;
    valid_reset = 1'b0;

    data_valid_in = 1'b0;

    pair_req_en = 1'b0;
    fetch_k_base = k_idx;

    act_in_packed = '0;
    wgt_pair_in_packed = '0;


    // Array operates during CLEAR, FEED and DRAIN.
    array_enable =
        (state == S_CLEAR) ||
        (state == S_FEED)  ||
        (state == S_DRAIN);


    // ---------------------------------------------------------
    // P6-A: Independent Clear Control
    // ---------------------------------------------------------

    // Every K-Tile must clear its pipeline.
    clear_pipe = (state == S_CLEAR);

    // Only the first K-Tile clears the INT32 accumulator.
    clear_acc =
        (state == S_CLEAR) && clear_acc_saved;

    // Reset valid tracking at the beginning of every K-Tile.
    valid_reset = clear_pipe;


    // First valid indication.
    data_valid_in =
        feed_valid && (k_idx == 0);


    // ---------------------------------------------------------
    // First K-pair request
    // ---------------------------------------------------------

    if (state == S_CLEAR) begin

      pair_req_en = 1'b1;
      fetch_k_base = '0;

    end


    // ---------------------------------------------------------
    // Current pair response + next pair request
    // ---------------------------------------------------------

    else if (state == S_FEED) begin

      act_in_packed[
        (int'(k_idx) % 8)*8 +: 8
      ] = act_pair_packed[7:0];

      act_in_packed[
        ((int'(k_idx)+1) % 8)*8 +: 8
      ] = act_pair_packed[15:8];

      wgt_pair_in_packed = wgt_pair_packed;


      // Request next pair if any remain.
      if (pairs_remaining != 0) begin

        pair_req_en = 1'b1;
        fetch_k_base = next_fetch_k;

      end

    end

  end


  // -------------------------------------------------------------------------
  // Sequential Control
  // -------------------------------------------------------------------------

  always_ff @(posedge clk or negedge rst_n) begin

    if (!rst_n) begin

      state <= S_IDLE;

      k_idx <= '0;
      k_len_saved <= '0;

      clear_acc_saved <= 1'b0;

      next_fetch_k <= '0;
      pairs_remaining <= '0;

      drain_cnt <= '0;
      result_packed <= '0;

    end

    else begin

      case (state)


        // -----------------------------------------------------
        // IDLE
        // -----------------------------------------------------

        S_IDLE: begin

          if (start && k_len != 0 && !k_len[0]) begin

            k_len_saved <= k_len;

            k_idx <= '0;

            // Save whether this K-Tile should clear the accumulator.
            clear_acc_saved <= clear_acc_on_start;

            state <= S_CLEAR;

          end

        end


        // -----------------------------------------------------
        // CLEAR
        // -----------------------------------------------------

        S_CLEAR: begin

          k_idx <= '0;

          // First K-pair is requested in S_CLEAR.
          // Next request starts at K=2.
          next_fetch_k <= K_ADDR_WIDTH'(2);

          // Total pairs = k_len / 2.
          // One pair has already been requested.
          pairs_remaining <=
              k_len_saved[K_ADDR_WIDTH-1:1] - 1'b1;

          drain_cnt <= '0;

          state <= S_FEED;

        end


        // -----------------------------------------------------
        // FEED
        // -----------------------------------------------------

        S_FEED: begin

          if (pairs_remaining != 0) begin

            // Advance to the next K-pair.
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


        // -----------------------------------------------------
        // DRAIN
        // -----------------------------------------------------

        S_DRAIN: begin

          if (drain_cnt == DRAIN_CYCLES-1) begin

            result_packed <= acc_out_packed;

            state <= S_DONE;

          end

          else begin

            drain_cnt <= drain_cnt + 1'b1;

          end

        end


        // -----------------------------------------------------
        // DONE
        // -----------------------------------------------------

        S_DONE: begin

          state <= S_IDLE;

        end


        default: begin

          state <= S_IDLE;

        end

      endcase

    end

  end


  // -------------------------------------------------------------------------
  // Simulation Checks
  // -------------------------------------------------------------------------

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

      // Validate K length and clear control at transaction start.
      if (state == S_IDLE && start) begin

        if ($isunknown({k_len, clear_acc_on_start}) ||
            k_len == 0 ||
            k_len[0])

          $fatal(1,
            "k_tile_controller_p2 requires positive even k_len and known clear control");

      end


      // Reader response protocol.
      if (feed_valid &&
          (act_rsp_valid !== 2'b11 ||
           wgt_rsp_valid !== 2'b11))

        $fatal(1,
          "k_tile_controller_p2 missing paired response at K=%0d",
          k_idx);


      // Validate next K request.
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
