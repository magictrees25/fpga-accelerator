`timescale 1ns / 1ps

// Full convolution integration test:
//   Input : 5x5x8 (HWC), INT8
//   Kernel: 3x3x8x16, INT8
//   Output: 3x3x16, INT32
//   GEMM  : M=9, N=16, K=72; 8x8 PE; two N-tiles.
//
// This TB instantiates the existing output-BRAM integration top. It does not
// modify the PE, K/M/N controller, BRAM readers, or any existing testbench.
// Golden outputs are computed by direct ky/kx/channel convolution loops,
// independent of DUT address-generation signals.
module tb_conv5x5_c8_n16_k72_dualntile;
  import pkg_accelerator::*;

  localparam int H_IN = 5;
  localparam int W_IN = 5;
  localparam int C_IN = 8;
  localparam int KH = 3;
  localparam int KW = 3;
  localparam int C_OUT = 16;
  localparam int STRIDE = 1;
  localparam int PADDING = 0;

  localparam int H_OUT = (H_IN + 2*PADDING - KH)/STRIDE + 1;
  localparam int W_OUT = (W_IN + 2*PADDING - KW)/STRIDE + 1;
  localparam int M = H_OUT * W_OUT;
  localparam int N = C_OUT;
  localparam int K = KH * KW * C_IN;
  localparam int N_TILES = (N + ARRAY_COLS - 1) / ARRAY_COLS;
  localparam int INPUT_WORDS = H_IN * W_IN * C_IN;
  localparam int WEIGHT_WORDS = N_TILES * K;
  localparam int OUTPUT_WORDS = M * N_TILES;
  localparam int EXPECTED_FEEDS = M * N_TILES * K;
  localparam int EXPECTED_WRITES = M * N_TILES;
  localparam int ADDR_WIDTH = 16;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst_n;
  logic start;

  logic fmap_wr_en;
  logic [ADDR_WIDTH-1:0] fmap_wr_addr;
  logic [ACT_WIDTH-1:0] fmap_wr_data;

  logic weight_wr_en;
  logic [ADDR_WIDTH-1:0] weight_wr_addr;
  logic [WGT_WIDTH*ARRAY_COLS-1:0] weight_wr_data;

  logic out_rd_en;
  logic [ADDR_WIDTH-1:0] out_rd_addr;
  logic [ACC_WIDTH*ARRAY_COLS-1:0] out_rd_data;
  logic [ARRAY_COLS-1:0] out_rd_mask;
  logic out_rd_valid;

  logic write_valid;
  logic [ADDR_WIDTH-1:0] write_addr;
  logic [ADDR_WIDTH-1:0] write_m_addr;
  logic [ADDR_WIDTH-1:0] write_n_base;
  logic [ARRAY_COLS-1:0] write_mask;
  logic [ACC_WIDTH*ARRAY_COLS-1:0] write_data;
  logic busy;
  logic done;
  logic any_overflow;

  // Existing complete compute + output-BRAM integration.
  conv_accelerator_top_outmem #(
    .H_IN(H_IN), .W_IN(W_IN), .C_IN(C_IN),
    .KH(KH), .KW(KW), .C_OUT(C_OUT),
    .STRIDE(STRIDE), .PADDING(PADDING),
    .ADDR_WIDTH(ADDR_WIDTH)
  ) dut (
    .clk(clk), .rst_n(rst_n), .start(start),
    .fmap_wr_en(fmap_wr_en), .fmap_wr_addr(fmap_wr_addr),
    .fmap_wr_data(fmap_wr_data),
    .weight_wr_en(weight_wr_en), .weight_wr_addr(weight_wr_addr),
    .weight_wr_data(weight_wr_data),
    .out_rd_en(out_rd_en), .out_rd_addr(out_rd_addr),
    .out_rd_data(out_rd_data), .out_rd_mask(out_rd_mask),
    .out_rd_valid(out_rd_valid),
    .write_valid(write_valid), .write_addr(write_addr),
    .write_m_addr(write_m_addr), .write_n_base(write_n_base),
    .write_mask(write_mask), .write_data(write_data),
    .busy(busy), .done(done), .any_overflow(any_overflow)
  );

  // Read-only hierarchical probes for protocol/data-alignment assertions.
  wire [ADDR_WIDTH-1:0] m_addr;
  wire [ADDR_WIDTH-1:0] n_base;
  wire [ADDR_WIDTH-1:0] k_addr;
  wire [ADDR_WIDTH-1:0] fetch_k_addr;
  wire [ADDR_WIDTH-1:0] fmap_addr;
  wire [ADDR_WIDTH-1:0] weight_word_addr;
  wire addr_valid;
  wire bram_data_valid;
  wire wgt_data_valid;
  wire signed [ACT_WIDTH-1:0] act_data;
  wire [WGT_WIDTH*ARRAY_COLS-1:0] wgt_row_packed;
  wire clear_acc;
  wire feed_valid;

  assign m_addr = dut.u_compute.m_addr;
  assign n_base = dut.u_compute.n_base;
  assign k_addr = dut.u_compute.k_addr;
  assign fetch_k_addr = dut.u_compute.fetch_k_addr;
  assign fmap_addr = dut.u_compute.fmap_addr;
  assign weight_word_addr = dut.u_compute.weight_word_addr;
  assign addr_valid = dut.u_compute.addr_valid;
  assign bram_data_valid = dut.u_compute.bram_data_valid;
  assign wgt_data_valid = dut.u_compute.wgt_data_valid;
  assign act_data = dut.u_compute.act_data;
  assign wgt_row_packed = dut.u_compute.wgt_row_packed;
  assign clear_acc = dut.u_compute.clear_acc;
  assign feed_valid = dut.u_compute.feed_valid;

  logic signed [ACT_WIDTH-1:0] fmap [0:H_IN-1][0:W_IN-1][0:C_IN-1];
  logic signed [ACT_WIDTH-1:0] fmap_flat [0:INPUT_WORDS-1];
  logic signed [WGT_WIDTH-1:0] kernel [0:KH-1][0:KW-1][0:C_IN-1][0:N-1];
  logic signed [ACC_WIDTH-1:0] golden [0:M-1][0:N-1];

  integer write_hits [0:OUTPUT_WORDS-1];
  integer total_feeds;
  integer total_clears;
  integer total_writes;
  integer total_readbacks;
  integer total_lane_checks;
  integer got_value;
  integer expected_value;
  integer expected_address;
  integer expected_m;
  integer expected_nt;
  integer expected_k;
  integer expected_fetch_k;
  integer signed init_value;
  integer signed init_weight;
  integer signed golden_sum;
  longint signed golden_checksum;
  longint signed golden_weighted_checksum;

  logic [WGT_WIDTH*ARRAY_COLS-1:0] expected_weight_word;

  // Independent HWC address reference for PADDING=0.
  // This function is only used by the TB assertions, not by the golden MACs.
  function automatic integer reference_fmap_addr(input integer m, input integer k);
    integer out_y;
    integer out_x;
    integer ker_y;
    integer ker_x;
    integer channel;
    integer in_y;
    integer in_x;
    begin
      out_y = m / W_OUT;
      out_x = m % W_OUT;
      ker_y = k / (KW*C_IN);
      ker_x = (k / C_IN) % KW;
      channel = k % C_IN;
      in_y = out_y*STRIDE + ker_y - PADDING;
      in_x = out_x*STRIDE + ker_x - PADDING;
      reference_fmap_addr = (in_y*W_IN + in_x)*C_IN + channel;
    end
  endfunction

  // Expected 8-lane weight word for the CURRENT K and CURRENT N-tile.
  // BRAM's registered output is compared against this current-K reference.
  always_comb begin : make_expected_weight_word
    integer ky;
    integer kx;
    integer ci;
    expected_weight_word = '0;
    ky = 0;
    kx = 0;
    ci = 0;
    if (int'(k_addr) < K) begin
      ky = int'(k_addr) / (KW*C_IN);
      kx = (int'(k_addr) / C_IN) % KW;
      ci = int'(k_addr) % C_IN;
      for (int lane = 0; lane < ARRAY_COLS; lane++) begin
        if (int'(n_base) + lane < N)
          expected_weight_word[lane*WGT_WIDTH +: WGT_WIDTH] =
              kernel[ky][kx][ci][int'(n_base)+lane];
      end
    end
  end

  // ------------------------------------------------------------------------
  // Independent deterministic input/weight initialization and golden model.
  // Dataset A is the full-layer M9/N16/K72 acceptance vector set:
  // fmap = (((y*5+x)*3 + c*7) % 31) - 15
  // w    = (((k + 3*n) % 13) - 6) + (n - 8)
  // ------------------------------------------------------------------------
  initial begin : initialize_and_make_golden
    for (int y = 0; y < H_IN; y++) begin
      for (int x = 0; x < W_IN; x++) begin
        for (int c = 0; c < C_IN; c++) begin
          init_value = (((y*W_IN+x)*3 + c*7) % 31) - 15;
          fmap[y][x][c] = ACT_WIDTH'(init_value);
          fmap_flat[(y*W_IN+x)*C_IN+c] = ACT_WIDTH'(init_value);
        end
      end
    end

    for (int ky = 0; ky < KH; ky++) begin
      for (int kx = 0; kx < KW; kx++) begin
        for (int c = 0; c < C_IN; c++) begin
          for (int n = 0; n < N; n++) begin
            init_value = (ky*KW+kx)*C_IN+c;
            init_weight = ((init_value + 3*n) % 13) - 6 + (n - 8);
            kernel[ky][kx][c][n] = WGT_WIDTH'(init_weight);
          end
        end
      end
    end

    // Golden convolution: direct ky/kx/channel loops. No DUT address signal
    // or DUT address-generator result participates in this calculation.
    golden_checksum = 0;
    golden_weighted_checksum = 0;
    for (int oy = 0; oy < H_OUT; oy++) begin
      for (int ox = 0; ox < W_OUT; ox++) begin
        for (int n = 0; n < N; n++) begin
          golden_sum = 0;
          for (int ky = 0; ky < KH; ky++) begin
            for (int kx = 0; kx < KW; kx++) begin
              for (int c = 0; c < C_IN; c++) begin
                golden_sum = golden_sum
                  + int'($signed(fmap[oy*STRIDE+ky-PADDING][ox*STRIDE+kx-PADDING][c]))
                  * int'($signed(kernel[ky][kx][c][n]));
              end
            end
          end
          golden[oy*W_OUT+ox][n] = ACC_WIDTH'(golden_sum);
          golden_checksum = golden_checksum + golden_sum;
          golden_weighted_checksum = golden_weighted_checksum
            + (longint'(oy*W_OUT+ox+1) * longint'(n+1) * longint'(golden_sum));
        end
      end
    end

    if (golden_checksum !== 64'sd810)
      $fatal(1, "TB DATA ERROR: golden checksum=%0d expected=810", golden_checksum);
    if (golden_weighted_checksum !== -64'sd114975)
      $fatal(1, "TB DATA ERROR: weighted checksum=%0d expected=-114975",
             golden_weighted_checksum);
    if (golden[0][0] !== 32'sd485 || golden[4][15] !== -32'sd192 ||
        golden[8][15] !== 32'sd113)
      $fatal(1, "TB DATA ERROR: golden anchor mismatch");

    // Address-map anchors independently verify the HWC/K decode convention.
    if (reference_fmap_addr(0,0) != 0 ||
        reference_fmap_addr(0,7) != 7 ||
        reference_fmap_addr(0,8) != 8 ||
        reference_fmap_addr(0,71) != 103 ||
        reference_fmap_addr(4,14) != 62 ||
        reference_fmap_addr(8,71) != 199)
      $fatal(1, "TB DATA ERROR: reference HWC address anchors mismatch");

    $display("INFO: configuration M=%0d N=%0d K=%0d N_TILES=%0d", M, N, K, N_TILES);
    $display("INFO: input=%0d INT8 bytes, weights=%0d packed 64-bit words, output=%0d packed words",
             INPUT_WORDS, WEIGHT_WORDS, OUTPUT_WORDS);
    $display("INFO: independent golden checksum=%0d weighted_checksum=%0d",
             golden_checksum, golden_weighted_checksum);
  end

  // ------------------------------------------------------------------------
  // Monitor the real controller/BRAM/PE data path and output writes.
  // Expected task order: M0/N0, M0/N8, M1/N0, M1/N8, ... M8/N8.
  // ------------------------------------------------------------------------
  always @(posedge clk) begin : monitor_compute
    if (!rst_n) begin
      total_feeds = 0;
      total_clears = 0;
      total_writes = 0;
      for (int i = 0; i < OUTPUT_WORDS; i++)
        write_hits[i] = 0;
    end else begin
      if (clear_acc && feed_valid)
        $fatal(1, "FAIL: CLEAR overlaps FEED");

      if (clear_acc) begin
        expected_m = total_clears / N_TILES;
        expected_nt = total_clears % N_TILES;
        if (total_clears >= EXPECTED_WRITES)
          $fatal(1, "FAIL: excessive CLEAR pulses");
        if (m_addr !== ADDR_WIDTH'(expected_m) ||
            n_base !== ADDR_WIDTH'(expected_nt*ARRAY_COLS) ||
            k_addr !== ADDR_WIDTH'(0))
          $fatal(1, "FAIL: CLEAR order got M=%0d Nbase=%0d K=%0d expected M=%0d Nbase=%0d K=0",
                 m_addr, n_base, k_addr, expected_m, expected_nt*ARRAY_COLS);
        if (fetch_k_addr !== ADDR_WIDTH'(0) ||
            weight_word_addr !== ADDR_WIDTH'(expected_nt*K))
          $fatal(1, "FAIL: CLEAR prefetch address Ntile=%0d fetchK=%0d weightAddr=%0d",
                 expected_nt, fetch_k_addr, weight_word_addr);
        expected_address = reference_fmap_addr(expected_m, 0);
        if (addr_valid !== 1'b1 || fmap_addr !== ADDR_WIDTH'(expected_address))
          $fatal(1, "FAIL: CLEAR activation prefetch M=%0d got addr=%0d valid=%b expected=%0d",
                 expected_m, fmap_addr, addr_valid, expected_address);
        $display("CLEAR: M=%0d N-tile=%0d Nbase=%0d weight_base=%0d",
                 expected_m, expected_nt, expected_nt*ARRAY_COLS, expected_nt*K);
        total_clears = total_clears + 1;
      end

      if (feed_valid) begin
        expected_m = total_feeds / (N_TILES*K);
        expected_nt = (total_feeds / K) % N_TILES;
        expected_k = total_feeds % K;
        expected_fetch_k = (expected_k < K-1) ? expected_k+1 : expected_k;

        if (total_feeds >= EXPECTED_FEEDS)
          $fatal(1, "FAIL: excessive FEED beats");
        if (m_addr !== ADDR_WIDTH'(expected_m) ||
            n_base !== ADDR_WIDTH'(expected_nt*ARRAY_COLS) ||
            k_addr !== ADDR_WIDTH'(expected_k))
          $fatal(1, "FAIL: FEED order beat=%0d got M=%0d Nbase=%0d K=%0d expected M=%0d Nbase=%0d K=%0d",
                 total_feeds, m_addr, n_base, k_addr,
                 expected_m, expected_nt*ARRAY_COLS, expected_k);
        if (fetch_k_addr !== ADDR_WIDTH'(expected_fetch_k))
          $fatal(1, "FAIL: lookahead K mismatch M=%0d Nbase=%0d K=%0d fetch=%0d expected=%0d",
                 m_addr, n_base, k_addr, fetch_k_addr, expected_fetch_k);

        if (!bram_data_valid)
          $fatal(1, "FAIL: activation BRAM data invalid M=%0d Nbase=%0d K=%0d",
                 m_addr, n_base, k_addr);
        expected_address = reference_fmap_addr(expected_m, expected_k);
        if (act_data !== fmap_flat[expected_address])
          $fatal(1, "FAIL: activation mismatch M=%0d Nbase=%0d K=%0d addr=%0d got=%0d expected=%0d",
                 expected_m, expected_nt*ARRAY_COLS, expected_k, expected_address,
                 $signed(act_data), $signed(fmap_flat[expected_address]));

        if (!wgt_data_valid)
          $fatal(1, "FAIL: weight BRAM data invalid M=%0d Nbase=%0d K=%0d",
                 m_addr, n_base, k_addr);
        if (wgt_row_packed !== expected_weight_word)
          $fatal(1, "FAIL: weight mismatch M=%0d Nbase=%0d K=%0d got=%h expected=%h",
                 m_addr, n_base, k_addr, wgt_row_packed, expected_weight_word);
        if (weight_word_addr !== ADDR_WIDTH'(expected_nt*K+expected_fetch_k))
          $fatal(1, "FAIL: weight lookahead address M=%0d Ntile=%0d K=%0d got=%0d expected=%0d",
                 m_addr, expected_nt, k_addr, weight_word_addr,
                 expected_nt*K+expected_fetch_k);

        expected_address = reference_fmap_addr(expected_m, expected_fetch_k);
        if (addr_valid !== 1'b1 || fmap_addr !== ADDR_WIDTH'(expected_address))
          $fatal(1, "FAIL: activation lookahead address M=%0d K=%0d got=%0d valid=%b expected=%0d",
                 expected_m, expected_k, fmap_addr, addr_valid, expected_address);

        total_feeds = total_feeds + 1;
      end

      if (write_valid) begin
        expected_m = total_writes / N_TILES;
        expected_nt = total_writes % N_TILES;
        if (total_writes >= EXPECTED_WRITES)
          $fatal(1, "FAIL: excessive output writes");
        if (write_addr !== ADDR_WIDTH'(total_writes))
          $fatal(1, "FAIL: write_addr=%0d expected=%0d", write_addr, total_writes);
        if (write_m_addr !== ADDR_WIDTH'(expected_m) ||
            write_n_base !== ADDR_WIDTH'(expected_nt*ARRAY_COLS))
          $fatal(1, "FAIL: write order addr=%0d got M=%0d Nbase=%0d expected M=%0d Nbase=%0d",
                 write_addr, write_m_addr, write_n_base,
                 expected_m, expected_nt*ARRAY_COLS);
        if (write_mask !== {ARRAY_COLS{1'b1}})
          $fatal(1, "FAIL: write mask addr=%0d got=%b expected=0xFF",
                 write_addr, write_mask);
        if (write_hits[int'(write_addr)] != 0)
          $fatal(1, "FAIL: duplicate output write address=%0d", write_addr);

        for (int lane = 0; lane < ARRAY_COLS; lane++) begin
          got_value = $signed(write_data[lane*ACC_WIDTH +: ACC_WIDTH]);
          expected_value = $signed(golden[expected_m][expected_nt*ARRAY_COLS+lane]);
          if (got_value !== expected_value)
            $fatal(1, "FAIL: output mismatch M=%0d N=%0d got=%0d expected=%0d",
                   expected_m, expected_nt*ARRAY_COLS+lane,
                   got_value, expected_value);
        end
        write_hits[int'(write_addr)] = write_hits[int'(write_addr)] + 1;
        $display("WRITE: addr=%0d M=%0d Nbase=%0d lanes=[%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d]",
                 write_addr, expected_m, expected_nt*ARRAY_COLS,
                 $signed(write_data[0*ACC_WIDTH +: ACC_WIDTH]),
                 $signed(write_data[1*ACC_WIDTH +: ACC_WIDTH]),
                 $signed(write_data[2*ACC_WIDTH +: ACC_WIDTH]),
                 $signed(write_data[3*ACC_WIDTH +: ACC_WIDTH]),
                 $signed(write_data[4*ACC_WIDTH +: ACC_WIDTH]),
                 $signed(write_data[5*ACC_WIDTH +: ACC_WIDTH]),
                 $signed(write_data[6*ACC_WIDTH +: ACC_WIDTH]),
                 $signed(write_data[7*ACC_WIDTH +: ACC_WIDTH]));
        total_writes = total_writes + 1;
      end

      if (any_overflow === 1'b1)
        $fatal(1, "FAIL: PE accumulator overflow detected");
    end
  end

  // ------------------------------------------------------------------------
  // Preload memories, start the layer, check counters, and read back all
  // 18 packed output words (8 lanes each = 144 independent INT32 checks).
  // ------------------------------------------------------------------------
  initial begin : drive_and_check
    rst_n = 1'b0;
    start = 1'b0;
    fmap_wr_en = 1'b0;
    fmap_wr_addr = '0;
    fmap_wr_data = '0;
    weight_wr_en = 1'b0;
    weight_wr_addr = '0;
    weight_wr_data = '0;
    out_rd_en = 1'b0;
    out_rd_addr = '0;
    total_readbacks = 0;
    total_lane_checks = 0;

    if (ARRAY_ROWS != 8 || ARRAY_COLS != 8)
      $fatal(1, "TB CONFIG ERROR: expected 8x8 PE array");
    if (M != 9 || N != 16 || K != 72 || N_TILES != 2)
      $fatal(1, "TB CONFIG ERROR: expected M9/N16/K72/two N-tiles");

    // Load 144 packed weight words: address = Ntile*K + K-index.
    for (int nt = 0; nt < N_TILES; nt++) begin
      for (int k = 0; k < K; k++) begin
        @(negedge clk);
        weight_wr_addr = ADDR_WIDTH'(nt*K+k);
        weight_wr_data = '0;
        for (int lane = 0; lane < ARRAY_COLS; lane++) begin
          weight_wr_data[lane*WGT_WIDTH +: WGT_WIDTH] =
              kernel[k/(KW*C_IN)][(k/C_IN)%KW][k%C_IN][nt*ARRAY_COLS+lane];
        end
        weight_wr_en = 1'b1;
      end
    end
    @(negedge clk);
    weight_wr_en = 1'b0;

    // Load 200 activation bytes in flattened HWC order.
    for (int addr = 0; addr < INPUT_WORDS; addr++) begin
      @(negedge clk);
      fmap_wr_addr = ADDR_WIDTH'(addr);
      fmap_wr_data = fmap_flat[addr];
      fmap_wr_en = 1'b1;
    end
    @(negedge clk);
    fmap_wr_en = 1'b0;

    // Keep compute in reset until both synchronous BRAMs are fully loaded.
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    start = 1'b1;
    @(negedge clk);
    start = 1'b0;

    wait (done === 1'b1);
    #1;

    if (total_clears !== EXPECTED_WRITES)
      $fatal(1, "FAIL: CLEAR count=%0d expected=%0d", total_clears, EXPECTED_WRITES);
    if (total_feeds !== EXPECTED_FEEDS)
      $fatal(1, "FAIL: FEED count=%0d expected=%0d", total_feeds, EXPECTED_FEEDS);
    if (total_writes !== EXPECTED_WRITES)
      $fatal(1, "FAIL: write count=%0d expected=%0d", total_writes, EXPECTED_WRITES);
    for (int addr = 0; addr < OUTPUT_WORDS; addr++) begin
      if (write_hits[addr] !== 1)
        $fatal(1, "FAIL: output word address=%0d hit count=%0d", addr, write_hits[addr]);
    end
    if (any_overflow !== 1'b0)
      $fatal(1, "FAIL: overflow flag not clear at completion");

    // Read each packed word through the actual synchronous output-BRAM port.
    for (int addr = 0; addr < OUTPUT_WORDS; addr++) begin
      @(negedge clk);
      out_rd_en = 1'b1;
      out_rd_addr = ADDR_WIDTH'(addr);
      @(posedge clk);
      #1;
      if (out_rd_valid !== 1'b1)
        $fatal(1, "FAIL: output BRAM rd_valid missing at addr=%0d", addr);
      if (out_rd_mask !== {ARRAY_COLS{1'b1}})
        $fatal(1, "FAIL: output BRAM mask addr=%0d got=%b expected=0xFF",
               addr, out_rd_mask);

      expected_m = addr / N_TILES;
      expected_nt = addr % N_TILES;
      for (int lane = 0; lane < ARRAY_COLS; lane++) begin
        got_value = $signed(out_rd_data[lane*ACC_WIDTH +: ACC_WIDTH]);
        expected_value = $signed(golden[expected_m][expected_nt*ARRAY_COLS+lane]);
        if (got_value !== expected_value)
          $fatal(1, "FAIL: output BRAM readback addr=%0d M=%0d N=%0d got=%0d expected=%0d",
                 addr, expected_m, expected_nt*ARRAY_COLS+lane,
                 got_value, expected_value);
        total_lane_checks = total_lane_checks + 1;
      end
      total_readbacks = total_readbacks + 1;
    end

    @(negedge clk);
    out_rd_en = 1'b0;
    @(posedge clk);
    #1;
    if (out_rd_valid !== 1'b0)
      $fatal(1, "FAIL: output BRAM rd_valid did not deassert");

    if (total_readbacks !== OUTPUT_WORDS)
      $fatal(1, "FAIL: output word readbacks=%0d expected=%0d",
             total_readbacks, OUTPUT_WORDS);
    if (total_lane_checks !== M*N)
      $fatal(1, "FAIL: output lane checks=%0d expected=%0d",
             total_lane_checks, M*N);

    $display("PASS: 5x5x8 / 3x3x8x16 convolution; M=9 N=16 K=72; N_TILES=2");
    $display("PASS: CLEAR=%0d FEED=%0d WRITE=%0d; output BRAM readbacks=%0d; INT32 lanes checked=%0d/%0d",
             total_clears, total_feeds, total_writes, total_readbacks,
             total_lane_checks, M*N);
    $display("PASS: all 144 outputs match independent direct-convolution golden model");
    $display("PASS: golden checksum=%0d weighted_checksum=%0d; no PE overflow",
             golden_checksum, golden_weighted_checksum);
    $finish;
  end

  initial begin : timeout_watchdog
    repeat (10000) @(posedge clk);
    $fatal(1, "TIMEOUT: M9/N16/K72 dual-N-tile convolution test did not finish");
  end
endmodule