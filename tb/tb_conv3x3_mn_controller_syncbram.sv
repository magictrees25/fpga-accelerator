`timescale 1ns / 1ps

// 3x3 convolution regression using the existing GEMM M/N/K controller.
// Feature map : 5 x 5 x 3 (HWC)
// Kernel      : 3 x 3 x 3 x 4 (Kh, Kw, Cin, Cout)
// Stride=1, padding=0 -> output 3 x 3 x 4
// GEMM equivalent: M=9, N=4, K=27.
//
// RTL conv_addr_gen plus conv_bram_reader performs ONE-CYCLE-AHEAD
// synchronous BRAM reads; no pre-expanded im2col A[9][27].
// Convolution weights remain combinational in this bring-up stage.
module tb_conv3x3_mn_controller_syncbram;
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
  logic [15:0] fmap_addr, fetch_k_addr;
  logic addr_valid, bram_data_valid;
  logic bram_wr_en;
  logic [15:0] bram_wr_addr;
  logic [ACT_WIDTH-1:0] bram_wr_data;
  logic [WGT_WIDTH*ARRAY_COLS-1:0] wgt_row_packed;
  logic write_valid, busy, done, any_overflow;
  logic [15:0] write_addr, write_m_addr, write_n_base;
  logic [ARRAY_COLS-1:0] write_mask;
  logic [ACC_WIDTH*ARRAY_COLS-1:0] write_data;

  // Actual input feature map and kernel, NOT pre-expanded im2col matrices.
  logic signed [ACT_WIDTH-1:0] fmap [0:HIN-1][0:WIN-1][0:CIN-1];
  // HWC layout in 1-D memory: addr = (y*WIN + x)*CIN + c.
  logic signed [ACT_WIDTH-1:0] fmap_flat [0:HIN*WIN*CIN-1];
  integer expected_addr_map [0:M-1][0:K-1];
  logic signed [WGT_WIDTH-1:0] kernel [0:KH-1][0:KW-1][0:CIN-1][0:COUT-1];
  integer golden [0:HOUT-1][0:WOUT-1][0:COUT-1];
  logic signed [ACC_WIDTH-1:0] c_mem [0:M-1][0:N-1];
  integer writes_seen, clears_seen, feeds_seen, addresses_checked;
  integer expected_fmap_addr;
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

  // Reusable RTL path: lookahead K address -> conv address generator ->
  // registered/synchronous feature-map RAM -> act_data.
  // In this TB we tap the K controller's internal clear_acc/feed_valid.
  // A future synthesizable top-level should wire these as explicit ports.
  conv_bram_reader #(
    .H_IN(HIN), .W_IN(WIN), .C_IN(CIN),
    .KH(KH), .KW(KW), .STRIDE(1), .PADDING(0),
    .ADDR_WIDTH(16), .DATA_WIDTH(ACT_WIDTH)
  ) u_reader (
    .clk(clk), .rst_n(rst_n), .m_addr(m_addr), .k_addr(k_addr),
    .clear_acc(dut.clear_acc), .feed_valid(dut.feed_valid),
    .wr_en(bram_wr_en), .wr_addr(bram_wr_addr), .wr_data(bram_wr_data),
    .act_data(act_data), .fetch_k_addr(fetch_k_addr),
    .fmap_addr(fmap_addr), .addr_valid(addr_valid),
    .data_valid(bram_data_valid)
  );

  // Weights still read combinationally. Only activation memory models
  // a synchronous BRAM in this step.
  always_comb begin : comb_weight_source
    integer ky, kx, ci;
    wgt_row_packed = '0;
    ky = 0;
    kx = 0;
    ci = 0;
    if (int'(k_addr) < K) begin
      ky = int'(k_addr) / (KW*CIN);
      kx = (int'(k_addr) / CIN) % KW;
      ci = int'(k_addr) % CIN;
      for (int lane = 0; lane < ARRAY_COLS; lane++) begin
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
          fmap_flat[(y*WIN + x)*CIN + c] = fmap[y][x][c];
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

    // Independent reference table for all 9*27 activation addresses.
    for (int oy = 0; oy < HOUT; oy++) begin
      for (int ox = 0; ox < WOUT; ox++) begin
        for (int ky = 0; ky < KH; ky++) begin
          for (int kx = 0; kx < KW; kx++) begin
            for (int c = 0; c < CIN; c++) begin
              expected_addr_map[oy*WOUT + ox][(ky*KW + kx)*CIN + c] =
                  ((oy + ky)*WIN + ox + kx)*CIN + c;
            end
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
      addresses_checked = 0;
    end else begin
      if (dut.clear_acc) begin
        clears_seen = clears_seen + 1;
        if (dut.feed_valid)
          $fatal(1, "FAIL: clear_acc overlaps feed_valid");
        if (fetch_k_addr !== 16'd0)
          $fatal(1, "FAIL: CLEAR should prefetch K0, got %0d", fetch_k_addr);
      end
      if (dut.feed_valid) begin
        feeds_seen = feeds_seen + 1;
        if (int'(m_addr) >= M || int'(k_addr) >= K)
          $fatal(1, "FAIL: out-of-range m_addr=%0d k_addr=%0d during FEED",
                 m_addr, k_addr);
        // Current data must correspond to CURRENT k, not the requested next k.
        expected_fmap_addr = expected_addr_map[int'(m_addr)][int'(k_addr)];
        if (!bram_data_valid)
          $fatal(1, "FAIL: no BRAM data at M=%0d K=%0d", m_addr, k_addr);
        if (act_data !== fmap_flat[expected_fmap_addr])
          $fatal(1, "FAIL: BRAM data mismatched M=%0d K=%0d got=%0d expected=%0d",
                 m_addr, k_addr, $signed(act_data),
                 $signed(fmap_flat[expected_fmap_addr]));
        // Concurrent request should be for next K; when K=26, last repeats.
        if (fetch_k_addr !== 16'((int'(k_addr) < K-1) ? int'(k_addr)+1 : int'(k_addr)))
          $fatal(1, "FAIL: wrong lookahead K addr current=%0d fetch=%0d", k_addr, fetch_k_addr);
        expected_fmap_addr = expected_addr_map[int'(m_addr)][int'(fetch_k_addr)];
        if (addr_valid !== 1'b1 || fmap_addr !== 16'(expected_fmap_addr))
          $fatal(1, "FAIL: BRAM read request M=%0d fetch_K=%0d addr=%0d expected=%0d",
                 m_addr, fetch_k_addr, fmap_addr, expected_fmap_addr);
        addresses_checked = addresses_checked + 1;
      end
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
    bram_wr_en   = 1'b0;
    bram_wr_addr = '0;
    bram_wr_data = '0;

    // Preload 75 bytes through the write PORT (not hierarchical force).
    // Stay in reset until loading finishes.
    for (int addr = 0; addr < HIN*WIN*CIN; addr++) begin
      @(negedge clk);
      bram_wr_addr = 16'(addr);
      bram_wr_data = fmap_flat[addr];
      bram_wr_en = 1'b1;
    end
    @(negedge clk);
    bram_wr_en = 1'b0;

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
    if (addresses_checked !== EXPECTED_FEEDS)
      $fatal(1,"FAIL: expected %0d address checks, got %0d",EXPECTED_FEEDS,addresses_checked);
    for (int m=0; m<M; m++) begin
      for (int n=0; n<N; n++) begin
        if (c_mem[m][n] !== golden[m/WOUT][m%WOUT][n])
          $fatal(1,"FAIL: stored result M=%0d N=%0d",m,n);
      end
    end
    if (any_overflow !== 1'b0)
      $fatal(1,"FAIL: overflow at end");

    $display("PASS: sync BRAM + one-cycle prefetch; 5x5x3/3x3x3x4; M=9 N=4 K=27; 243/243 FEED beats; 9 writes; 36/36 outputs correct");
    $finish;
  end

  initial begin : timeout
    repeat (2300) @(posedge clk);
    $fatal(1,"TIMEOUT: convolution regression");
  end
endmodule
