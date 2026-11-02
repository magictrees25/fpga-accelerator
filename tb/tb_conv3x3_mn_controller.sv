`timescale 1ns / 1ps

// 3x3 convolution regression using the existing GEMM M/N/K controller.
// Feature map : 5 x 5 x 3 (HWC)
// Kernel      : 3 x 3 x 3 x 4 (Kh, Kw, Cin, Cout)
// Stride=1, padding=0 -> output 3 x 3 x 4
// GEMM equivalent: M=9, N=4, K=27.
//
// This testbench does ON-THE-FLY im2col: it does NOT store A[9][27].
// Source memories respond combinationally to (m_addr,k_addr,n_base).
// Synchronous BRAM read latency is NOT modeled here.
module tb_conv3x3_mn_controller;
  import pkg_accelerator::*;

  localparam int HIN  = 5;
  localparam int WIN  = 5;
  localparam int CIN  = 3;
  localparam int KH   = 3;
  localparam int KW   = 3;
  localparam int COUT = 4;
  localparam int HOUT = HIN - KH + 1;
  localparam int WOUT = WIN - KW + 1;
  localparam int M    = HOUT * WOUT;
  localparam int N    = COUT;
  localparam int K    = KH * KW * CIN;
  localparam int NTILES          = (N + ARRAY_COLS - 1) / ARRAY_COLS;
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

  // Actual input feature map and kernel, NOT pre-expanded im2col matrices.
  logic signed [ACT_WIDTH-1:0] fmap [0:HIN-1][0:WIN-1][0:CIN-1];
  logic signed [WGT_WIDTH-1:0] kernel [0:KH-1][0:KW-1][0:CIN-1][0:COUT-1];
  integer golden [0:HOUT-1][0:WOUT-1][0:COUT-1];
  logic signed [ACC_WIDTH-1:0] c_mem [0:M-1][0:N-1];
  integer writes_seen, clears_seen, feeds_seen;
  integer got_val, expected_val;

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

  // ON-THE-FLY im2col (HWC flattening order):
  //   m -> (out_y = m/3, out_x = m%3)
  //   k -> (ky = k/9, kx = (k/3)%3, ci = k%3)
  //   A[m][k] = fmap[out_y+ky][out_x+kx][ci]
  //   B[k][n] = kernel[ky][kx][ci][n]
  // For COUT=4, the upper four PE columns receive zero weights.
  always_comb begin : comb_source
    integer out_y, out_x, ky, kx, ci;
    act_data      = '0;
    wgt_row_packed = '0;
    out_y = 0;
    out_x = 0;
    ky = 0;
    kx = 0;
    ci = 0;

    if (int'(m_addr) < M && int'(k_addr) < K) begin
      out_y = int'(m_addr) / WOUT;
      out_x = int'(m_addr) % WOUT;
      ky = int'(k_addr) / (KW*CIN);
      kx = (int'(k_addr) / CIN) % KW;
      ci = int'(k_addr) % CIN;

      act_data = fmap[out_y + ky][out_x + kx][ci];
      for (int lane = 0; lane < ARRAY_COLS; lane = lane + 1) begin
        if (int'(n_base) + lane < N)
          wgt_row_packed[lane*WGT_WIDTH +: WGT_WIDTH] =
              kernel[ky][kx][ci][int'(n_base) + lane];
      end
    end
  end

  // Separate golden model: direct 3x3 convolution with six nested loops,
  // independent of the above m/k address mapping.
  initial begin : init_data
    for (int y = 0; y < HIN; y++) begin
      for (int x = 0; x < WIN; x++) begin
        for (int c = 0; c < CIN; c++) begin
          fmap[y][x][c] = ACT_WIDTH'(1 + y*WIN + x + 25*c);
        end
      end
    end

    for (int ky = 0; ky < KH; ky++) begin
      for (int kx = 0; kx < KW; kx++) begin
        for (int c = 0; c < CIN; c++) begin
          for (int n = 0; n < N; n++) begin
            kernel[ky][kx][c][n] = WGT_WIDTH'(((((ky*KW+kx)*CIN+c) + 2*n) % 7) - 3);
          end
        end
      end
    end

    for (int oy = 0; oy < HOUT; oy++) begin
      for (int ox = 0; ox < WOUT; ox++) begin
        for (int n = 0; n < N; n++) begin
          golden[oy][ox][n] = 0;
          for (int ky = 0; ky < KH; ky++) begin
            for (int kx = 0; kx < KW; kx++) begin
              for (int c = 0; c < CIN; c++) begin
                golden[oy][ox][n] = golden[oy][ox][n]
                  + int'($signed(fmap[oy+ky][ox+kx][c]))
                  * int'($signed(kernel[ky][kx][c][n]));
              end
            end
          end
        end
      end
    end
  end

  // One 256-bit write per output spatial position; only lanes 0..3 valid.
  always @(posedge clk) begin : monitor
    if (!rst_n) begin
      writes_seen = 0;
      clears_seen = 0;
      feeds_seen  = 0;
    end else begin
      if (dut.clear_acc) begin
        clears_seen = clears_seen + 1;
        if (dut.feed_valid)
          $fatal(1, "FAIL: clear_acc overlaps feed_valid");
      end
      if (dut.feed_valid)
        feeds_seen = feeds_seen + 1;
      if (any_overflow === 1'b1)
        $fatal(1, "FAIL: accumulator overflow");

      if (write_valid) begin
        if (writes_seen >= EXPECTED_WRITES)
          $fatal(1, "FAIL: excessive output writes");
        if (write_addr !== 16'(writes_seen))
          $fatal(1, "FAIL: write_addr=%0d expected=%0d",write_addr,writes_seen);
        if (write_m_addr !== 16'(writes_seen))
          $fatal(1, "FAIL: write_m_addr=%0d expected=%0d",write_m_addr,writes_seen);
        if (write_n_base !== 16'd0)
          $fatal(1, "FAIL: unexpected N tile %0d", write_n_base);
        if (write_mask !== 8'b0000_1111)
          $fatal(1, "FAIL: write_mask=%b expected=00001111", write_mask);

        $write("SAVE addr=%0d output(%0d,%0d):", write_addr,
               int'(write_m_addr)/WOUT, int'(write_m_addr)%WOUT);
        for (int lane = 0; lane < ARRAY_COLS; lane++) begin
          got_val = $signed(write_data[lane*ACC_WIDTH +: ACC_WIDTH]);
          if (lane < N) begin
            expected_val = golden[int'(write_m_addr)/WOUT]
                                   [int'(write_m_addr)%WOUT][lane];
            if (got_val !== expected_val)
              $fatal(1, "FAIL: C[oy=%0d][ox=%0d][oc=%0d] got=%0d expected=%0d",
                     int'(write_m_addr)/WOUT, int'(write_m_addr)%WOUT,
                     lane, got_val, expected_val);
            c_mem[int'(write_m_addr)][lane] = got_val;
            $write(" %0d", got_val);
          end else begin
            if (got_val !== 0)
              $fatal(1, "FAIL: inactive output lane=%0d got=%0d",lane,got_val);
          end
        end
        $display("");
        writes_seen = writes_seen + 1;
      end
    end
  end

  initial begin : drive
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
      $fatal(1,"FAIL: expected %0d writes, got %0d", EXPECTED_WRITES, writes_seen);
    if (clears_seen !== EXPECTED_WRITES)
      $fatal(1,"FAIL: expected %0d clear pulses, got %0d", EXPECTED_WRITES, clears_seen);
    if (feeds_seen !== EXPECTED_FEEDS)
      $fatal(1,"FAIL: expected %0d K beats, got %0d",EXPECTED_FEEDS,feeds_seen);
    for (int m=0; m<M; m++) begin
      for (int n=0; n<N; n++) begin
        if (c_mem[m][n] !== golden[m/WOUT][m%WOUT][n])
          $fatal(1,"FAIL: stored result M=%0d N=%0d",m,n);
      end
    end
    if (any_overflow !== 1'b0)
      $fatal(1,"FAIL: overflow at end");

    $display("PASS: conv 5x5x3 with 3x3x3x4 kernels; GEMM M=9 N=4 K=27; 9 writes; 243 K beats; 36/36 outputs correct");
    $finish;
  end

  initial begin : timeout
    repeat (2000) @(posedge clk);
    $fatal(1,"TIMEOUT: convolution regression");
  end
endmodule
