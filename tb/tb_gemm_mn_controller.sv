`timescale 1ns / 1ps

// Automatic M/N/K GEMM regression test for the verified controller hierarchy.
//
//   A: M=16, K=27            A[m][k] = m + k + 1
//   B: K=27, N=16            B[k][n] = ((k % 3) + 1) * (n + 1)
//   C: M=16, N=16            C[m][n] = (774 + 54*m) * (n + 1)
//
// The 8x8 PE array computes 8 output channels per K-controller run:
//   N_TILES = 2, total K-controller runs = 16*2 = 32.
// Each run feeds K=27 input beats. Expected total = 864 K beats.
// Each output RAM write stores eight 32-bit elements (256 bits).
//
// IMPORTANT: This testbench models zero-latency, COMBINATIONAL A/B reads.
//             A real synchronous BRAM interface needs read-latency handling.
module tb_gemm_mn_controller;
  import pkg_accelerator::*;

  localparam int M = 16;
  localparam int N = 16;
  localparam int K = 27;
  localparam int NTILES = (N + ARRAY_COLS - 1) / ARRAY_COLS;
  localparam int EXPECTED_WRITES = M * NTILES;
  localparam int EXPECTED_FEEDS  = EXPECTED_WRITES * K;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst_n, start;
  logic [15:0] m_len, n_len, k_len;
  logic [15:0] m_addr, n_base, k_addr;
  logic signed [ACT_WIDTH-1:0] act_data;
  logic [WGT_WIDTH*ARRAY_COLS-1:0] wgt_row_packed;
  logic write_valid, busy, done, any_overflow;
  logic [15:0] write_addr, write_m_addr, write_n_base;
  logic [ARRAY_COLS-1:0] write_mask;
  logic [ACC_WIDTH*ARRAY_COLS-1:0] write_data;

  // Simulation-only result storage.
  logic signed [ACC_WIDTH-1:0] c_mem [0:M-1][0:N-1];
  integer writes_seen, clears_seen, feeds_seen;
  integer got_val, expected_val;

  // DUT interface is identical to the original M=2,N=16,K=9 testbench.
  gemm_mn_controller dut (
    .clk(clk), .rst_n(rst_n), .start(start),
    .m_len(m_len), .n_len(n_len), .k_len(k_len),
    .m_addr(m_addr), .n_base(n_base), .k_addr(k_addr),
    .act_data(act_data), .wgt_row_packed(wgt_row_packed),
    .write_valid(write_valid), .write_addr(write_addr),
    .write_m_addr(write_m_addr), .write_n_base(write_n_base),
    .write_mask(write_mask), .write_data(write_data),
    .busy(busy), .done(done), .any_overflow(any_overflow)
  );

  // Ideal combinational A/B read ports. No explicit A/B arrays are needed:
  // data is generated directly from the requested indices.
  // A[m][k] = m + k + 1         (max 42, fits signed INT8)
  // B[k][n] = ((k%3)+1)*(n+1)  (max 48, fits signed INT8)
  always_comb begin
    act_data = '0;
    wgt_row_packed = '0;

    if (int'(m_addr) < M && int'(k_addr) < K) begin
      act_data = ACT_WIDTH'(int'(m_addr) + int'(k_addr) + 1);
      for (int n = 0; n < ARRAY_COLS; n = n + 1) begin
        if (int'(n_base) + n < N) begin
          wgt_row_packed[n*WGT_WIDTH +: WGT_WIDTH] =
              WGT_WIDTH'(((int'(k_addr) % 3) + 1) *
                         (int'(n_base) + n + 1));
        end
      end
    end
  end

  // Each G_WRITE pulse represents one 256-bit output word.
  // Check addresses, row/tile order, lane masks, and every numeric result.
  always @(posedge clk) begin
    if (!rst_n) begin
      writes_seen = 0;
      clears_seen = 0;
      feeds_seen  = 0;
    end else begin
      if (dut.clear_acc) begin
        clears_seen = clears_seen + 1;
        if (dut.feed_valid)
          $fatal(1, "FAIL: clear_acc asserted during K feed");
      end

      if (dut.feed_valid)
        feeds_seen = feeds_seen + 1;

      if (any_overflow === 1'b1)
        $fatal(1, "FAIL: PE accumulator overflow");

      if (write_valid) begin
        if (writes_seen >= EXPECTED_WRITES)
          $fatal(1, "FAIL: too many output writes");

        if (write_addr !== 16'(writes_seen))
          $fatal(1, "FAIL: write_addr=%0d, expected=%0d",
                 write_addr, writes_seen);

        if (write_m_addr !== 16'(writes_seen / NTILES))
          $fatal(1, "FAIL: write_m_addr=%0d, expected=%0d",
                 write_m_addr, writes_seen / NTILES);

        if (write_n_base !== 16'((writes_seen % NTILES) * ARRAY_COLS))
          $fatal(1, "FAIL: write_n_base=%0d, expected=%0d",
                 write_n_base, (writes_seen % NTILES) * ARRAY_COLS);

        if (write_mask !== {ARRAY_COLS{1'b1}})
          $fatal(1, "FAIL: unexpected write_mask=%b", write_mask);

        $write("SAVE addr=%0d M%0d N%0d..N%0d:",
               write_addr, write_m_addr,
               write_n_base, write_n_base + ARRAY_COLS - 1);

        for (int lane = 0; lane < ARRAY_COLS; lane = lane + 1) begin
          got_val = $signed(write_data[lane*ACC_WIDTH +: ACC_WIDTH]);
          expected_val = (774 + 54 * int'(write_m_addr)) *
                         (int'(write_n_base) + lane + 1);

          if (got_val !== expected_val)
            $fatal(1, "FAIL: C[%0d][%0d]=%0d expected=%0d",
                   write_m_addr, int'(write_n_base) + lane,
                   got_val, expected_val);

          c_mem[int'(write_m_addr)][int'(write_n_base) + lane] = got_val;
          $write(" %0d", got_val);
        end
        $display("");
        writes_seen = writes_seen + 1;
      end
    end
  end

  initial begin
    rst_n = 1'b0;
    start = 1'b0;
    m_len = 16'(M);
    n_len = 16'(N);
    k_len = 16'(K);

    repeat (3) @(negedge clk);
    rst_n = 1'b1;

    @(negedge clk); start = 1'b1;
    @(negedge clk); start = 1'b0;

    wait (done === 1'b1);
    #1;

    if (writes_seen !== EXPECTED_WRITES)
      $fatal(1, "FAIL: writes_seen=%0d, expected=%0d",
             writes_seen, EXPECTED_WRITES);

    if (clears_seen !== EXPECTED_WRITES)
      $fatal(1, "FAIL: clear pulses=%0d, expected=%0d",
             clears_seen, EXPECTED_WRITES);

    if (feeds_seen !== EXPECTED_FEEDS)
      $fatal(1, "FAIL: K beats=%0d, expected=%0d",
             feeds_seen, EXPECTED_FEEDS);

    for (int m = 0; m < M; m = m + 1) begin
      for (int n = 0; n < N; n = n + 1) begin
        expected_val = (774 + 54*m) * (n + 1);
        if (c_mem[m][n] !== expected_val)
          $fatal(1, "FAIL: stored C[%0d][%0d]=%0d, expected=%0d",
                 m, n, c_mem[m][n], expected_val);
      end
    end

    if (any_overflow !== 1'b0)
      $fatal(1, "FAIL: overflow at end of test");

    $display("PASS: M=16 N=16 K=27; 32 writes; 32 clears; 864 K beats; 256/256 outputs correct");
    $finish;
  end

  initial begin
    repeat (4000) @(posedge clk);
    $fatal(1, "TIMEOUT: GEMM MN controller did not finish");
  end
endmodule
