`timescale 1ns / 1ps

// Small 1x1 convolution system regression; existing PE/controller RTL unchanged.
// Input: 8x8x16 HWC, kernel: 1x1, Cout=16, stride=2, padding=0.
// Output: 4x4x16. GEMM M=16,N=16,K=16; two N-tiles.
// Activation memory: 1024 bytes. Weights: 32 packed 8xINT8 words.
// Output memory: 32 packed 8xINT32 words. Total FEED beats: 512.
// Spatial samples are input (0,0),(0,2),...,(6,6), all 16 input channels.
// fmap[y][x][c]=((y*8+x+7*c)%127)-63, signed INT8.
// kernel[0][0][c][n]=((3*c+5*n)%17)-8, signed INT8.
// Independent six-loop convolution golden; three 16-element anchor rows.
// Drive preload/start/read ports on negedge; check FEED/writes before NBA;
// compare synchronous read data at posedge+#1. No forced memory initialization.
// User runs Vivado. This file has not been compiled or simulated here.
module tb_fpga_system_top_conv1x1_s2;
  import pkg_accelerator::*;

  localparam int HIN=8, WIN=8, CIN=16, KH=1, KW=1, COUT=16;
  localparam int STRIDE=2, PADDING=0;
  localparam int HOUT=(HIN+2*PADDING-KH)/STRIDE+1;
  localparam int WOUT=(WIN+2*PADDING-KW)/STRIDE+1;
  localparam int M=HOUT*WOUT, N=COUT, K=KH*KW*CIN;
  localparam int ADDR_WIDTH=16;
  localparam int LANE_SEL_WIDTH=$clog2(ARRAY_COLS);
  localparam int NTILES=(N+ARRAY_COLS-1)/ARRAY_COLS;
  localparam int FMAP_WORDS=HIN*WIN*CIN;
  localparam int WEIGHT_WORDS=NTILES*K;
  localparam int EXPECTED_WRITES=M*NTILES;
  localparam int EXPECTED_FEEDS=EXPECTED_WRITES*K;
  localparam int EXPECTED_OUTPUTS=M*N;
  localparam int TIMEOUT_CYCLES=10000;

  logic clk=1'b0;
  always #5 clk=~clk;

  logic rst_n, start;
  logic fmap_wr_en, weight_wr_en;
  logic [ADDR_WIDTH-1:0] fmap_wr_addr, weight_wr_addr;
  logic [ACT_WIDTH-1:0] fmap_wr_data;
  logic [WGT_WIDTH*ARRAY_COLS-1:0] weight_wr_data;
  logic out_rd_en;
  logic [ADDR_WIDTH-1:0] out_rd_addr;
  logic [LANE_SEL_WIDTH-1:0] out_rd_lane_sel;
  logic signed [ACC_WIDTH-1:0] out_rd_data;
  logic out_rd_valid, out_rd_lane_valid;
  logic busy, done, any_overflow;

  // Explicit 1x1 configuration of the existing top. PE RTL is unchanged.
  fpga_system_top #(
    .H_IN(HIN), .W_IN(WIN), .C_IN(CIN), .KH(KH), .KW(KW), .C_OUT(N),
    .STRIDE(STRIDE), .PADDING(PADDING), .ADDR_WIDTH(ADDR_WIDTH)
  ) dut (
    .clk(clk), .rst_n(rst_n), .start(start),
    .fmap_wr_en(fmap_wr_en), .fmap_wr_addr(fmap_wr_addr),
    .fmap_wr_data(fmap_wr_data),
    .weight_wr_en(weight_wr_en), .weight_wr_addr(weight_wr_addr),
    .weight_wr_data(weight_wr_data),
    .out_rd_en(out_rd_en), .out_rd_addr(out_rd_addr),
    .out_rd_lane_sel(out_rd_lane_sel), .out_rd_data(out_rd_data),
    .out_rd_lane_valid(out_rd_lane_valid), .out_rd_valid(out_rd_valid),
    .busy(busy), .done(done), .any_overflow(any_overflow)
  );

  // Read-only probes: names checked against the actual RTL instance tree.
  wire clear_acc, feed_valid, addr_valid, bram_data_valid, wgt_data_valid;
  wire act_rd_en, wgt_rd_en, write_valid;
  wire [ADDR_WIDTH-1:0] m_addr, n_base, k_addr, fetch_k_addr;
  wire [ADDR_WIDTH-1:0] fmap_addr, weight_word_addr;
  wire signed [ACT_WIDTH-1:0] act_data;
  wire [WGT_WIDTH*ARRAY_COLS-1:0] wgt_row_packed;
  wire [ADDR_WIDTH-1:0] write_addr, write_m_addr, write_n_base;
  wire [ARRAY_COLS-1:0] write_mask;
  wire [ACC_WIDTH*ARRAY_COLS-1:0] write_data;

  assign clear_acc=dut.u_core.u_compute.clear_acc;
  assign feed_valid=dut.u_core.u_compute.feed_valid;
  assign m_addr=dut.u_core.u_compute.m_addr;
  assign n_base=dut.u_core.u_compute.n_base;
  assign k_addr=dut.u_core.u_compute.k_addr;
  assign fetch_k_addr=dut.u_core.u_compute.fetch_k_addr;
  assign fmap_addr=dut.u_core.u_compute.fmap_addr;
  assign weight_word_addr=dut.u_core.u_compute.weight_word_addr;
  assign act_data=dut.u_core.u_compute.act_data;
  assign wgt_row_packed=dut.u_core.u_compute.wgt_row_packed;
  assign addr_valid=dut.u_core.u_compute.addr_valid;
  assign bram_data_valid=dut.u_core.u_compute.bram_data_valid;
  assign wgt_data_valid=dut.u_core.u_compute.wgt_data_valid;
  assign act_rd_en=dut.u_core.u_compute.u_act_reader.rd_en;
  assign wgt_rd_en=dut.u_core.u_compute.u_weight_reader.rd_en;
  assign write_valid=dut.u_core.write_valid;
  assign write_addr=dut.u_core.write_addr;
  assign write_m_addr=dut.u_core.write_m_addr;
  assign write_n_base=dut.u_core.write_n_base;
  assign write_mask=dut.u_core.write_mask;
  assign write_data=dut.u_core.write_data;

  logic signed [7:0] fmap[0:HIN-1][0:WIN-1][0:CIN-1];
  logic signed [7:0] kernel[0:KH-1][0:KW-1][0:CIN-1][0:N-1];
  logic signed [31:0] golden[0:HOUT-1][0:WOUT-1][0:N-1];
  logic signed [31:0] known_first[0:N-1], known_middle[0:N-1], known_last[0:N-1];

  integer writes_seen, clears_seen, feeds_seen, done_seen;
  integer write_outputs_checked, readbacks_checked;
  integer round_feeds;
  integer feeds_by_tile[0:NTILES-1], writes_by_tile[0:NTILES-1];
  integer requests_by_tile[0:NTILES-1];
  integer weight_request_hits[0:WEIGHT_WORDS-1];
  integer word_write_hits[0:EXPECTED_WRITES-1];
  integer lane_read_hits[0:EXPECTED_WRITES-1][0:ARRAY_COLS-1];
  logic round_open, run_started, completion_observed;

  task automatic prepare_data_and_golden;
    integer value;
    integer sum;
    begin
      for (int y=0; y<HIN; y++)
        for (int x=0; x<WIN; x++)
          for (int c=0; c<CIN; c++) begin
            // Bounded signed data; reusing 1+y*8+x+7*c would exceed INT8.
            value=((y*WIN+x+7*c)%127)-63;
            if (value < -128 || value > 127)
              $fatal(1,"FAIL: activation outside signed INT8");
            fmap[y][x][c]=8'(value);
          end

      for (int ky=0; ky<KH; ky++)
        for (int kx=0; kx<KW; kx++)
          for (int c=0; c<CIN; c++)
            for (int n=0; n<N; n++) begin
              value=((3*c+5*n)%17)-8;
              if (value < -128 || value > 127)
                $fatal(1,"FAIL: weight outside signed INT8");
              kernel[ky][kx][c][n]=8'(value);
            end

      // Independent standard convolution: no DUT signals or address helper.
      for (int oy=0; oy<HOUT; oy++)
        for (int ox=0; ox<WOUT; ox++)
          for (int n=0; n<N; n++) begin
            sum=0;
            for (int ky=0; ky<KH; ky++)
              for (int kx=0; kx<KW; kx++)
                for (int c=0; c<CIN; c++)
                  sum=sum
                    + int'($signed(fmap[oy*STRIDE+ky][ox*STRIDE+kx][c]))
                    * int'($signed(kernel[ky][kx][c][n]));
            golden[oy][ox][n]=sum;
          end

      // Three independent arithmetic anchors; all M*N results are checked.
      known_first='{777,651,-665,-315,511,147,-574,371,-231,0,-126,224,-616,210,679,-399};
      known_middle='{204,-808,764,-316,-376,-11,-3,-488,183,1143,-413,-949,1099,495,-466,-271};
      known_last='{199,-295,-738,485,943,-707,-555,1263,293,-762,-15,-33,0,-460,882,-564};
      for (int n=0; n<N; n++) begin
        if (golden[0][0][n] !== known_first[n])
          $fatal(1,"FAIL: golden first N%0d got=%0d expected=%0d",
                 n,golden[0][0][n],known_first[n]);
        if (golden[(M/2)/WOUT][(M/2)%WOUT][n] !== known_middle[n])
          $fatal(1,"FAIL: golden middle N%0d got=%0d expected=%0d",
                 n,golden[(M/2)/WOUT][(M/2)%WOUT][n],known_middle[n]);
        if (golden[HOUT-1][WOUT-1][n] !== known_last[n])
          $fatal(1,"FAIL: golden last N%0d got=%0d expected=%0d",
                 n,golden[HOUT-1][WOUT-1][n],known_last[n]);
      end
      $display("INFO: independent signed 1x1 convolution golden matches 48 arithmetic anchors");
    end
  endtask

  // Check the request present BEFORE the edge, separately from data returned
  // by the PREVIOUS edge. Called for CLEAR(K0) and every FEED lookahead.
  task automatic check_request(input integer em, input integer ent,
                               input integer ek);
    integer ky, kx, ci, expected_fmap, expected_weight;
    begin
      ky=ek/(KW*CIN);
      kx=(ek/CIN)%KW;
      ci=ek%CIN;
      expected_fmap=(((em/WOUT)*STRIDE+ky)*WIN+(em%WOUT)*STRIDE+kx)*CIN+ci;
      expected_weight=ent*K+ek;
      if (act_rd_en !== 1'b1 || wgt_rd_en !== 1'b1)
        $fatal(1,"FAIL: missing BRAM request M=%0d tile=%0d fetchK=%0d",em,ent,ek);
      if (fetch_k_addr !== ADDR_WIDTH'(ek))
        $fatal(1,"FAIL: fetchK=%0d expected=%0d M=%0d tile=%0d",
               fetch_k_addr,ek,em,ent);
      if (addr_valid !== 1'b1 || fmap_addr !== ADDR_WIDTH'(expected_fmap))
        $fatal(1,"FAIL: activation request M=%0d fetchK=%0d addr=%0d expected=%0d",
               em,ek,fmap_addr,expected_fmap);
      if (weight_word_addr !== ADDR_WIDTH'(expected_weight))
        $fatal(1,"FAIL: weight request tile=%0d fetchK=%0d addr=%0d expected=%0d",
               ent,ek,weight_word_addr,expected_weight);
      requests_by_tile[ent]=requests_by_tile[ent]+1;
      weight_request_hits[expected_weight]=weight_request_hits[expected_weight]+1;
    end
  endtask

  // Transaction checks use TB-owned expected indices, not DUT indices to
  // select a convenient golden value. No #delay here: check what RTL consumes.
  always @(posedge clk) begin : monitor
    integer em, ent, ek, ky, kx, ci;
    integer expected_value, got_value;
    if (!rst_n) begin
      writes_seen=0;
      clears_seen=0;
      feeds_seen=0;
      done_seen=0;
      write_outputs_checked=0;
      round_feeds=0;
      round_open=1'b0;
      for (int t=0; t<NTILES; t++) begin
        feeds_by_tile[t]=0;
        writes_by_tile[t]=0;
        requests_by_tile[t]=0;
      end
      for (int a=0; a<WEIGHT_WORDS; a++) weight_request_hits[a]=0;
      for (int a=0; a<EXPECTED_WRITES; a++) word_write_hits[a]=0;
    end else begin
      if ($isunknown({clear_acc,feed_valid,write_valid,busy,done,any_overflow}))
        $fatal(1,"FAIL: unknown control/status after reset");
      if (any_overflow !== 1'b0)
        $fatal(1,"FAIL: accumulator overflow");
      if ((clear_acc || feed_valid || write_valid || done) && !run_started)
        $fatal(1,"FAIL: transaction before start");
      if (completion_observed && (clear_acc || feed_valid || write_valid))
        $fatal(1,"FAIL: extra compute/write transaction after done");
      if ((clear_acc || feed_valid || write_valid) && busy !== 1'b1)
        $fatal(1,"FAIL: active transaction while busy is not asserted");

      em=writes_seen/NTILES;
      ent=writes_seen%NTILES;

      if (clear_acc) begin
        if (feed_valid || write_valid || round_open || writes_seen >= EXPECTED_WRITES)
          $fatal(1,"FAIL: invalid/overlapping CLEAR round=%0d",writes_seen);
        if (clears_seen !== writes_seen || feeds_seen !== writes_seen*K)
          $fatal(1,"FAIL: CLEAR round/count sequence");
        if (m_addr !== ADDR_WIDTH'(em) || n_base !== ADDR_WIDTH'(ent*ARRAY_COLS)
            || k_addr !== ADDR_WIDTH'(0))
          $fatal(1,"FAIL: CLEAR coordinates M=%0d Nbase=%0d K=%0d expected M=%0d tile=%0d",
                 m_addr,n_base,k_addr,em,ent);
        check_request(em,ent,0);
        round_open=1'b1;
        round_feeds=0;
        clears_seen=clears_seen+1;
      end

      if (feed_valid) begin
        if (!round_open || write_valid || round_feeds >= K || feeds_seen >= EXPECTED_FEEDS)
          $fatal(1,"FAIL: unexpected/excess FEED round=%0d Kcount=%0d",writes_seen,round_feeds);
        ek=round_feeds;
        if (m_addr !== ADDR_WIDTH'(em) || n_base !== ADDR_WIDTH'(ent*ARRAY_COLS)
            || k_addr !== ADDR_WIDTH'(ek))
          $fatal(1,"FAIL: FEED sequence got M=%0d Nbase=%0d K=%0d expected M=%0d tile=%0d K=%0d",
                 m_addr,n_base,k_addr,em,ent,ek);
        if (bram_data_valid !== 1'b1 || wgt_data_valid !== 1'b1)
          $fatal(1,"FAIL: missing aligned A/B data M=%0d tile=%0d K=%0d",em,ent,ek);
        ky=ek/(KW*CIN);
        kx=(ek/CIN)%KW;
        ci=ek%CIN;
        expected_value=int'($signed(fmap[(em/WOUT)*STRIDE+ky][(em%WOUT)*STRIDE+kx][ci]));
        got_value=int'($signed(act_data));
        if (got_value !== expected_value)
          $fatal(1,"FAIL: activation M=%0d tile=%0d K=%0d got=%0d expected=%0d",
                 em,ent,ek,got_value,expected_value);
        for (int lane=0; lane<ARRAY_COLS; lane++) begin
          expected_value=int'($signed(kernel[ky][kx][ci][ent*ARRAY_COLS+lane]));
          got_value=int'($signed(wgt_row_packed[lane*WGT_WIDTH +: WGT_WIDTH]));
          if (got_value !== expected_value)
            $fatal(1,"FAIL: weight alignment M=%0d tile=%0d K=%0d lane=%0d got=%0d expected=%0d",
                   em,ent,ek,lane,got_value,expected_value);
        end
        check_request(em,ent,(ek < K-1) ? ek+1 : ek);
        round_feeds=round_feeds+1;
        feeds_seen=feeds_seen+1;
        feeds_by_tile[ent]=feeds_by_tile[ent]+1;
      end else if (round_open && round_feeds < K) begin
        // CLEAR itself is allowed; the following 16 FEED beats are continuous.
        if (!clear_acc)
          $fatal(1,"FAIL: FEED gap before all 16 beats M=%0d tile=%0d count=%0d",
                 em,ent,round_feeds);
      end

      if (write_valid) begin
        if (!round_open || round_feeds !== K || writes_seen >= EXPECTED_WRITES)
          $fatal(1,"FAIL: early/extra write addr=%0d Kcount=%0d",write_addr,round_feeds);
        if (write_addr !== ADDR_WIDTH'(writes_seen)
            || write_m_addr !== ADDR_WIDTH'(em)
            || write_n_base !== ADDR_WIDTH'(ent*ARRAY_COLS))
          $fatal(1,"FAIL: write sequence addr=%0d M=%0d Nbase=%0d expected addr=%0d M=%0d tile=%0d",
                 write_addr,write_m_addr,write_n_base,writes_seen,em,ent);
        if (write_mask !== {ARRAY_COLS{1'b1}})
          $fatal(1,"FAIL: write mask=%h expected=FF",write_mask);
        if (word_write_hits[writes_seen] !== 0)
          $fatal(1,"FAIL: duplicate output word %0d",writes_seen);
        for (int lane=0; lane<ARRAY_COLS; lane++) begin
          expected_value=golden[em/WOUT][em%WOUT][ent*ARRAY_COLS+lane];
          got_value=$signed(write_data[lane*ACC_WIDTH +: ACC_WIDTH]);
          if (got_value !== expected_value)
            $fatal(1,"FAIL: output write M=%0d N=%0d got=%0d expected=%0d",
                   em,ent*ARRAY_COLS+lane,got_value,expected_value);
          write_outputs_checked=write_outputs_checked+1;
        end
        $display("INFO: WRITE addr=%0d M=%0d N=%0d..%0d mask=%h checked",
                 writes_seen,em,ent*ARRAY_COLS,ent*ARRAY_COLS+ARRAY_COLS-1,write_mask);
        word_write_hits[writes_seen]=word_write_hits[writes_seen]+1;
        writes_by_tile[ent]=writes_by_tile[ent]+1;
        writes_seen=writes_seen+1;
        round_open=1'b0;
      end

      if (done) begin
        if (busy !== 1'b0 || round_open || writes_seen !== EXPECTED_WRITES
            || feeds_seen !== EXPECTED_FEEDS || done_seen !== 0)
          $fatal(1,"FAIL: premature/repeated done writes=%0d feeds=%0d",writes_seen,feeds_seen);
        done_seen=done_seen+1;
      end
    end
  end

  task automatic check_compute_counts;
    integer expected_hits;
    begin
      if (writes_seen !== EXPECTED_WRITES || clears_seen !== EXPECTED_WRITES
          || feeds_seen !== EXPECTED_FEEDS || write_outputs_checked !== EXPECTED_OUTPUTS
          || round_open !== 1'b0)
        $fatal(1,"FAIL: compute totals clear=%0d feed=%0d write=%0d outputs=%0d",
               clears_seen,feeds_seen,writes_seen,write_outputs_checked);
      for (int t=0; t<NTILES; t++) begin
        if (feeds_by_tile[t] !== M*K || writes_by_tile[t] !== M
            || requests_by_tile[t] !== M*(K+1))
          $fatal(1,"FAIL: tile %0d coverage feeds=%0d writes=%0d requests=%0d",
                 t,feeds_by_tile[t],writes_by_tile[t],requests_by_tile[t]);
        for (int k=0; k<K; k++) begin
          // CLEAR requests K0; FEED requests K1..K15; final FEED repeats K15.
          expected_hits=(k == K-1) ? 2*M : M;
          if (weight_request_hits[t*K+k] !== expected_hits)
            $fatal(1,"FAIL: weight address %0d requests=%0d expected=%0d",
                   t*K+k,weight_request_hits[t*K+k],expected_hits);
        end
      end
      for (int a=0; a<EXPECTED_WRITES; a++)
        if (word_write_hits[a] !== 1)
          $fatal(1,"FAIL: output word %0d write count=%0d",a,word_write_hits[a]);
    end
  endtask

  initial begin : drive
    integer ky, kx, ci, em, ent;
    integer expected_value, got_value;
    rst_n=1'b0;
    start=1'b0;
    run_started=1'b0;
    completion_observed=1'b0;
    fmap_wr_en=1'b0;
    fmap_wr_addr='0;
    fmap_wr_data='0;
    weight_wr_en=1'b0;
    weight_wr_addr='0;
    weight_wr_data='0;
    out_rd_en=1'b0;
    out_rd_addr='0;
    out_rd_lane_sel='0;
    readbacks_checked=0;
    for (int a=0; a<EXPECTED_WRITES; a++)
      for (int lane=0; lane<ARRAY_COLS; lane++) lane_read_hits[a][lane]=0;

    if (ARRAY_ROWS != 8 || ARRAY_COLS != 8 || ACT_WIDTH != 8
        || WGT_WIDTH != 8 || ACC_WIDTH != 32)
      $fatal(1,"FAIL: this regression requires 8x8 PE and INT8/INT32 widths");
    if (dut.H_IN != HIN || dut.W_IN != WIN
        || dut.C_IN != CIN || dut.C_OUT != N
        || dut.KH != KH || dut.KW != KW
        || dut.STRIDE != STRIDE || dut.PADDING != PADDING)
      $fatal(1,"FAIL: DUT convolution configuration differs from 1x1/S2/P0 M16/N16/K16");
    if (dut.u_core.u_compute.M_LEN != M
        || dut.u_core.u_compute.N_LEN != N
        || dut.u_core.u_compute.K_LEN != K
        || dut.u_core.u_compute.N_TILES != NTILES)
      $fatal(1,"FAIL: DUT derived dimensions differ from test configuration");
    if (dut.u_core.u_compute.u_act_reader.u_mem.DEPTH < FMAP_WORDS
        || dut.u_core.u_compute.u_weight_reader.u_mem.DEPTH < WEIGHT_WORDS
        || dut.u_core.u_out_mem.DEPTH < EXPECTED_WRITES)
      $fatal(1,"FAIL: insufficient DUT BRAM depth");
    prepare_data_and_golden();

    // Same preload/reset convention as the legacy TB: RAM writes work
    // while the compute logic stays in reset. No hierarchical preload.
    for (int t=0; t<NTILES; t++)
      for (int k=0; k<K; k++) begin
        ky=k/(KW*CIN);
        kx=(k/CIN)%KW;
        ci=k%CIN;
        @(negedge clk);
        weight_wr_addr=ADDR_WIDTH'(t*K+k);
        weight_wr_data='0;
        for (int lane=0; lane<ARRAY_COLS; lane++)
          weight_wr_data[lane*WGT_WIDTH +: WGT_WIDTH]=kernel[ky][kx][ci][t*ARRAY_COLS+lane];
        weight_wr_en=1'b1;
      end
    @(negedge clk);
    weight_wr_en=1'b0;

    for (int y=0; y<HIN; y++)
      for (int x=0; x<WIN; x++)
        for (int c=0; c<CIN; c++) begin
          @(negedge clk);
          fmap_wr_addr=ADDR_WIDTH'((y*WIN+x)*CIN+c);
          fmap_wr_data=fmap[y][x][c];
          fmap_wr_en=1'b1;
        end
    @(negedge clk);
    fmap_wr_en=1'b0;

    repeat (3) @(negedge clk);
    rst_n=1'b1;
    @(negedge clk);
    run_started=1'b1;
    start=1'b1;
    @(negedge clk);
    start=1'b0;
    $display("INFO: started 1x1 stride=2 padding=0 M=16 N=16 K=16; expecting 32 rounds and 512 FEED beats");

    wait (done === 1'b1);
    #1;
    completion_observed=1'b1;
    check_compute_counts();
    if (busy !== 1'b0 || any_overflow !== 1'b0)
      $fatal(1,"FAIL: bad completion status");

    // Read EVERY stored word/lane through the external 32-bit interface.
    // Word address and lane are driven together, then sampled after NBA.
    for (int a=0; a<EXPECTED_WRITES; a++) begin
      em=a/NTILES;
      ent=a%NTILES;
      for (int lane=0; lane<ARRAY_COLS; lane++) begin
        @(negedge clk);
        out_rd_en=1'b1;
        out_rd_addr=ADDR_WIDTH'(a);
        out_rd_lane_sel=LANE_SEL_WIDTH'(lane);
        @(posedge clk);
        #1;
        if (out_rd_valid !== 1'b1 || out_rd_lane_valid !== 1'b1)
          $fatal(1,"FAIL: missing read valid addr=%0d lane=%0d",a,lane);
        expected_value=golden[em/WOUT][em%WOUT][ent*ARRAY_COLS+lane];
        got_value=$signed(out_rd_data);
        if (got_value !== expected_value)
          $fatal(1,"FAIL: BRAM read addr=%0d lane=%0d M=%0d N=%0d got=%0d expected=%0d",
                 a,lane,em,ent*ARRAY_COLS+lane,got_value,expected_value);
        lane_read_hits[a][lane]=lane_read_hits[a][lane]+1;
        readbacks_checked=readbacks_checked+1;
      end
    end
    @(negedge clk);
    out_rd_en=1'b0;
    @(posedge clk);
    #1;
    if (out_rd_valid !== 1'b0 || out_rd_lane_valid !== 1'b0)
      $fatal(1,"FAIL: output read-valid did not deassert");

    // Continue observing briefly to reject extra writes or repeated done.
    repeat (3) @(negedge clk);
    check_compute_counts();
    if (done_seen !== 1 || busy !== 1'b0 || done !== 1'b0 || any_overflow !== 1'b0)
      $fatal(1,"FAIL: final status/done count done_seen=%0d",done_seen);
    if (readbacks_checked !== EXPECTED_OUTPUTS)
      $fatal(1,"FAIL: readback count=%0d expected=%0d",readbacks_checked,EXPECTED_OUTPUTS);
    for (int a=0; a<EXPECTED_WRITES; a++)
      for (int lane=0; lane<ARRAY_COLS; lane++)
        if (lane_read_hits[a][lane] !== 1)
          $fatal(1,"FAIL: missing/duplicate read addr=%0d lane=%0d",a,lane);

    $display("INFO: both N-tiles checked; weight addresses 0..31 covered; stride-2 HWC requests checked");
    $display("INFO: 256/256 write-side outputs and 256/256 synchronous lane readbacks checked");
    $display("PASS: 1x1 stride=2 padding=0, M=16 N=16 K=16, 512 K beats, 32 writes, 256/256 outputs and readbacks correct");
    $finish;
  end

  initial begin : timeout
    repeat (TIMEOUT_CYCLES) @(posedge clk);
    $fatal(1,"FAIL: TIMEOUT after %0d cycles; done=%b busy=%b feeds=%0d writes=%0d readbacks=%0d",
           TIMEOUT_CYCLES,done,busy,feeds_seen,writes_seen,readbacks_checked);
  end
endmodule
