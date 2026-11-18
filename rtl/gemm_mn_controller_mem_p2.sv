
`timescale 1ns / 1ps

// P2 M/N controller: same outer scheduling, paired K supply internally.
//
// P5-B2-5A:
//   Preserve original M/N scheduling and synchronous memory interface.
//
// P6-A:
//   1. Connect independent clear_acc and clear_pipe controls.
//   2. clear_acc_on_start = 1'b1 for every new M/N output tile.
//   3. Route clear_pipe from K Controller to systolic_array_p2.
//   4. Preserve original P5 single-K-Tile behavior.
//
// P6-B2-4:
//   K_TILE_SIZE=0 (default): original single-K invocation and timing.
//   K_TILE_SIZE>0: automatically partition total k_len into even K segments.
//   Each M/N output keeps its INT32 sums across segments and writes ONCE.
//   k_base stays local to a segment; fetch_k_base is the global Reader index.
//   All data must already be accessible through that global index. This is
//   not a Host/preload handshake or a larger-Cin convolution address mapper.
//   Existing supported shape is retained: 1<=M<=64, N=8, positive even K.
//
// mem_clear_acc/mem_feed_valid are observation outputs,
// not Reader requests.

module gemm_mn_controller_mem_p2
  import pkg_accelerator::*;
#(
  parameter int ADDR_WIDTH = 16,
  parameter int K_TILE_SIZE = 0
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

  // -------------------------------------------------------------------------
  // Outer M/N State Machine
  // -------------------------------------------------------------------------

  typedef enum logic [2:0] {
    G_IDLE,
    G_START_K,
    G_WAIT_K,
    G_WRITE,
    G_DONE
  } g_state_t;

  g_state_t state;

  logic [ADDR_WIDTH-1:0] m_idx, n_tile_idx, write_word_idx;
  logic [ADDR_WIDTH-1:0] m_len_saved, n_len_saved;
  logic [ADDR_WIDTH-1:0] k_len_saved, n_tiles_saved;

  localparam bit USE_K_TILING = (K_TILE_SIZE != 0);
  localparam logic [ADDR_WIDTH-1:0] K_TILE_CAP = ADDR_WIDTH'(K_TILE_SIZE);

  // k_remaining includes the current segment. Register the segment length so
  // the K Controller samples a stable length/base at each G_START_K.
  logic [ADDR_WIDTH-1:0] k_tile_base_reg, k_remaining, k_segment_len;
  logic [ADDR_WIDTH-1:0] k_after_segment;
  assign k_after_segment = k_remaining - k_segment_len;

  function automatic logic [ADDR_WIDTH-1:0] segment_length(
    input logic [ADDR_WIDTH-1:0] remaining
  );
    if (USE_K_TILING && remaining > K_TILE_CAP)
      segment_length = K_TILE_CAP;
    else
      segment_length = remaining;
  endfunction

  // -------------------------------------------------------------------------
  // K Controller Signals
  // -------------------------------------------------------------------------

  logic k_start, k_done, k_busy;

  logic array_enable, feed_valid;
  logic clear_acc, clear_pipe, valid_reset;
  logic data_valid_in, data_valid_out;

  logic [ACT_WIDTH*ARRAY_ROWS-1:0] act_in_packed;
  logic [127:0] wgt_pair_in_packed;

  logic [ACC_WIDTH*ARRAY_COLS-1:0] acc_out_packed;
  logic [ACC_WIDTH*ARRAY_COLS-1:0] k_result_packed;

  // -------------------------------------------------------------------------
  // Basic Outputs
  // -------------------------------------------------------------------------

  assign m_addr = m_idx;
  assign n_base = n_tile_idx * ARRAY_COLS;

  assign k_start = (state == G_START_K);

  assign write_valid = (state == G_WRITE);
  assign write_addr = write_word_idx;
  assign write_m_addr = m_idx;
  assign write_n_base = n_base;
  assign write_data = k_result_packed;

  assign busy =
      (state == G_START_K ||
       state == G_WAIT_K  ||
       state == G_WRITE);

  assign done = (state == G_DONE);

  assign mem_clear_acc = clear_acc;
  assign mem_feed_valid = feed_valid;

  // -------------------------------------------------------------------------
  // Output Channel Write Mask
  // -------------------------------------------------------------------------

  always_comb begin

    write_mask = '0;

    for (int lane = 0; lane < ARRAY_COLS; lane++) begin

      if (int'(n_base) + lane < int'(n_len_saved))
        write_mask[lane] = 1'b1;

    end

  end

  // -------------------------------------------------------------------------
  // K Tile Controller
  //
  // First segment of each M/N output starts a new accumulation. Subsequent
  // segments clear only the pipeline, with global fetch and local PE indexing.
  // -------------------------------------------------------------------------

  k_tile_controller_p2 #(
    .K_ADDR_WIDTH(ADDR_WIDTH),
    .USE_K_TILE_BASE(USE_K_TILING)
  ) u_kctrl (

    .clk(clk),
    .rst_n(rst_n),

    .start(k_start),
    .clear_acc_on_start(!USE_K_TILING || k_tile_base_reg == 0),

    .k_len(USE_K_TILING ? k_segment_len : k_len_saved),
    .k_tile_base(k_tile_base_reg),

    .k_base(k_base),
    .fetch_k_base(fetch_k_base),
    .pair_req_en(pair_req_en),

    .act_pair_packed(act_pair_packed),
    .act_rsp_valid(act_rsp_valid),

    .wgt_pair_packed(wgt_pair_packed),
    .wgt_rsp_valid(wgt_rsp_valid),

    .array_enable(array_enable),
    .feed_valid(feed_valid),

    .clear_acc(clear_acc),
    .clear_pipe(clear_pipe),
    .valid_reset(valid_reset),

    .data_valid_in(data_valid_in),

    .act_in_packed(act_in_packed),
    .wgt_pair_in_packed(wgt_pair_in_packed),

    .acc_out_packed(acc_out_packed),

    .busy(k_busy),
    .done(k_done),
    .result_packed(k_result_packed)

  );

  // -------------------------------------------------------------------------
  // Systolic Array
  // -------------------------------------------------------------------------

  systolic_array_p2 u_array (

    .clk(clk),
    .rst_n(rst_n),

    .enable(array_enable),

    .clear_acc(clear_acc),
    .clear_pipe(clear_pipe),
    .valid_reset(valid_reset),

    .act_in_packed(act_in_packed),
    .wgt_pair_in_packed(wgt_pair_in_packed),

    .data_valid_in(data_valid_in),
    .data_valid_out(data_valid_out),

    .acc_out_packed(acc_out_packed),
    .any_overflow(any_overflow)

  );

  // -------------------------------------------------------------------------
  // Outer M/N Scheduling
  // Original P5 scheduling preserved when K_TILE_SIZE=0. A continuation uses
  // G_WAIT_K -> G_START_K: the inner controller leaves S_DONE for S_IDLE on
  // that same edge, and accepts the new start on the following clock.
  // -------------------------------------------------------------------------

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
      k_tile_base_reg <= '0;
      k_remaining <= '0;
      k_segment_len <= '0;

    end else begin

      case (state)

        G_IDLE: begin

          if (start && m_len != 0 && n_len != 0 && k_len != 0) begin

            m_len_saved <= m_len;
            n_len_saved <= n_len;
            k_len_saved <= k_len;
            k_tile_base_reg <= '0;
            k_remaining <= USE_K_TILING ? k_len : '0;
            k_segment_len <= USE_K_TILING ? segment_length(k_len) : '0;

            n_tiles_saved <=
                (int'(n_len) + ARRAY_COLS - 1) / ARRAY_COLS;

            m_idx <= '0;
            n_tile_idx <= '0;
            write_word_idx <= '0;

            state <= G_START_K;

          end

        end

        G_START_K: begin

          state <= G_WAIT_K;

        end

        G_WAIT_K: begin

          if (k_done) begin
            if (USE_K_TILING && k_remaining > k_segment_len) begin
              k_tile_base_reg <= k_tile_base_reg + k_segment_len;
              k_remaining <= k_after_segment;
              k_segment_len <= segment_length(k_after_segment);
              state <= G_START_K;
            end else begin
              // Never expose an intermediate K-segment snapshot as a write.
              state <= G_WRITE;
            end
          end

        end

        G_WRITE: begin

          write_word_idx <= write_word_idx + 1'b1;
          // The next M/N output starts at K=0 and clears its accumulators.
          k_tile_base_reg <= '0;
          k_remaining <= USE_K_TILING ? k_len_saved : '0;
          k_segment_len <= USE_K_TILING ? segment_length(k_len_saved) : '0;

          if (n_tile_idx == n_tiles_saved - 1'b1) begin

            if (m_idx == m_len_saved - 1'b1) begin

              state <= G_DONE;

            end else begin

              m_idx <= m_idx + 1'b1;
              n_tile_idx <= '0;

              state <= G_START_K;

            end

          end else begin

            n_tile_idx <= n_tile_idx + 1'b1;

            state <= G_START_K;

          end

        end

        G_DONE: begin

          state <= G_IDLE;

        end

        default: begin

          state <= G_IDLE;

        end

      endcase

    end

  end

  // -------------------------------------------------------------------------
  // Simulation Checks
  // -------------------------------------------------------------------------

  // synthesis translate_off

  initial begin
    if (ADDR_WIDTH < 2 || ADDR_WIDTH > 30 || K_TILE_SIZE < 0 ||
        (USE_K_TILING && (K_TILE_SIZE < 2 || (K_TILE_SIZE % 2) != 0 ||
                         K_TILE_SIZE > ((64'd1 << ADDR_WIDTH) - 1))))
      $fatal(1, "gemm_mn_controller_mem_p2: K_TILE_SIZE must be 0 or a representable positive even size");
  end

  always @(posedge clk) begin
    if (rst_n && USE_K_TILING) begin
      if (k_start && (k_segment_len == 0 || k_segment_len[0] ||
                      k_segment_len > k_remaining ||
                      k_segment_len > K_TILE_CAP ||
                      ({1'b0,k_tile_base_reg} + {1'b0,k_remaining}) !=
                          {1'b0,k_len_saved}))
        $fatal(1, "gemm_mn_controller_mem_p2: invalid K segment bookkeeping");
      if (write_valid && k_remaining != k_segment_len)
        $fatal(1, "gemm_mn_controller_mem_p2: attempted intermediate K write");
    end
  end

  always @(posedge clk) begin

    if (rst_n && state == G_IDLE && start) begin

      if ($isunknown({m_len, n_len, k_len}) ||
          m_len == 0 ||
          m_len > 64 ||
          n_len != 8 ||
          k_len == 0 ||
          k_len[0])

        $fatal(1,
          "gemm_mn_controller_mem_p2: require 1<=M<=64, N=8, positive even K");

    end

  end

  // synthesis translate_on

endmodule
