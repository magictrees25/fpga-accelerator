`timescale 1ns / 1ps

// ResNet18 layer1.0.conv1 local output Tile 8x8x8 regression.
// THREE independent simulation DUTs: interior, top-left, bottom-right.
// Each: local input 10x10x64, kernel 3x3, stride=1, padding=0,
// M=64 N=8 K=576 N_TILES=1, 36864 FEED beats, 64 writes, 512 results.
// Global layer is 56x56x64 -> 56x56x64 with padding=1.
// Loader supplies halo/zero padding. Golden uses global sources independently.
// Synthetic signed INT8 data, not trained/quantized ResNet weights.
// No PE/controller changes, no AXI DMA. User compiles/runs Vivado.
// Simulation Top: tb_fpga_system_top_layer1_tile
module tb_fpga_system_top_layer1_tile;
  wire [2:0] finished;
  tb_layer1_tile_case #(.CASE_ID(0), .OY0(16), .OX0(24), .OC0(24),
    .EXPECTED_PAD_WORDS(0), .EXPECTED_PAD_FEEDS(0))
    interior (.finished(finished[0]));
  tb_layer1_tile_case #(.CASE_ID(1), .OY0(0), .OX0(0), .OC0(0),
    .EXPECTED_PAD_WORDS(1216), .EXPECTED_PAD_FEEDS(3008))
    top_left (.finished(finished[1]));
  tb_layer1_tile_case #(.CASE_ID(2), .OY0(48), .OX0(48), .OC0(56),
    .EXPECTED_PAD_WORDS(1216), .EXPECTED_PAD_FEEDS(3008))
    bottom_right (.finished(finished[2]));

  initial begin
    wait (finished === 3'b111);
    $display("PASS: layer1.0.conv1 local Tile 8x8x8, 3/3 cases, 110592 K beats, 192 writes, 1536/1536 outputs and readbacks correct");
    $finish;
  end
endmodule

module tb_layer1_tile_case #(
  parameter int CASE_ID=0,
  parameter int OY0=16, OX0=24, OC0=24,
  parameter int EXPECTED_PAD_WORDS=0, EXPECTED_PAD_FEEDS=0
)(output logic finished);
  import pkg_accelerator::*;

  localparam int HIN=10, WIN=10, CIN=64, KH=3, KW=3, COUT=8;
  localparam int STRIDE=1, PADDING=0;
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
  localparam int TIMEOUT_CYCLES=100000;

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

  // Fixed configuration wrapper; same baseline PE/controller RTL.
  fpga_system_top_layer1_tile #(.ADDR_WIDTH(ADDR_WIDTH)) dut (
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

  assign clear_acc=dut.u_baseline.u_core.u_compute.clear_acc;
  assign feed_valid=dut.u_baseline.u_core.u_compute.feed_valid;
  assign m_addr=dut.u_baseline.u_core.u_compute.m_addr;
  assign n_base=dut.u_baseline.u_core.u_compute.n_base;
  assign k_addr=dut.u_baseline.u_core.u_compute.k_addr;
  assign fetch_k_addr=dut.u_baseline.u_core.u_compute.fetch_k_addr;
  assign fmap_addr=dut.u_baseline.u_core.u_compute.fmap_addr;
  assign weight_word_addr=dut.u_baseline.u_core.u_compute.weight_word_addr;
  assign act_data=dut.u_baseline.u_core.u_compute.act_data;
  assign wgt_row_packed=dut.u_baseline.u_core.u_compute.wgt_row_packed;
  assign addr_valid=dut.u_baseline.u_core.u_compute.addr_valid;
  assign bram_data_valid=dut.u_baseline.u_core.u_compute.bram_data_valid;
  assign wgt_data_valid=dut.u_baseline.u_core.u_compute.wgt_data_valid;
  assign act_rd_en=dut.u_baseline.u_core.u_compute.u_act_reader.rd_en;
  assign wgt_rd_en=dut.u_baseline.u_core.u_compute.u_weight_reader.rd_en;
  assign write_valid=dut.u_baseline.u_core.write_valid;
  assign write_addr=dut.u_baseline.u_core.write_addr;
  assign write_m_addr=dut.u_baseline.u_core.write_m_addr;
  assign write_n_base=dut.u_baseline.u_core.write_n_base;
  assign write_mask=dut.u_baseline.u_core.write_mask;
  assign write_data=dut.u_baseline.u_core.write_data;

  logic signed [7:0] fmap[0:HIN-1][0:WIN-1][0:CIN-1];
  logic signed [7:0] kernel[0:KH-1][0:KW-1][0:CIN-1][0:N-1];
  logic signed [31:0] golden[0:HOUT-1][0:WOUT-1][0:N-1];
  logic signed [31:0] known_first[0:N-1], known_middle[0:N-1], known_last[0:N-1];

  logic signed [7:0] global_fmap[0:55][0:55][0:CIN-1];
  logic signed [7:0] global_kernel[0:KH-1][0:KW-1][0:CIN-1][0:63];
  integer padded_words, padded_feeds;
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
    integer value, gy, gx, sum;
    begin
      if (OY0 < 0 || OX0 < 0 || OY0+HOUT > 56 || OX0+WOUT > 56
          || OC0 < 0 || OC0+N > 64 || OC0%8 != 0)
        $fatal(1,"%m FAIL: invalid global Tile origin/channel base");

      // Synthetic signed INT8 data at REAL layer dimensions, not trained weights.
      for (int y=0; y<56; y++)
        for (int x=0; x<56; x++)
          for (int c=0; c<CIN; c++) begin
            value=((17*y+11*x+7*c)%127)-63;
            global_fmap[y][x][c]=8'(value);
          end
      for (int ky=0; ky<KH; ky++)
        for (int kx=0; kx<KW; kx++)
          for (int c=0; c<CIN; c++)
            for (int n=0; n<64; n++) begin
              value=((13*ky+7*kx+3*c+5*n)%19)-9;
              global_kernel[ky][kx][c][n]=8'(value);
            end

      // Loader: extract the 10x10 halo from the global input, padding in TB.
      // The DUT receives valid local addresses even for these stored zeros.
      padded_words=0;
      for (int iy=0; iy<HIN; iy++)
        for (int ix=0; ix<WIN; ix++)
          for (int c=0; c<CIN; c++) begin
            gy=OY0-1+iy;
            gx=OX0-1+ix;
            if (gy < 0 || gy >= 56 || gx < 0 || gx >= 56) begin
              fmap[iy][ix][c]='0;
              padded_words=padded_words+1;
            end else
              fmap[iy][ix][c]=global_fmap[gy][gx][c];
          end
      if (padded_words !== EXPECTED_PAD_WORDS)
        $fatal(1,"%m FAIL: extracted padding words=%0d expected=%0d",
               padded_words,EXPECTED_PAD_WORDS);
      for (int ky=0; ky<KH; ky++)
        for (int kx=0; kx<KW; kx++)
          for (int c=0; c<CIN; c++)
            for (int n=0; n<N; n++)
              kernel[ky][kx][c][n]=global_kernel[ky][kx][c][OC0+n];

      // Independent GLOBAL convolution, stride=1, padding=1.
      // Never reads local fmap/kernel or DUT address/data to form golden.
      for (int oy=0; oy<HOUT; oy++)
        for (int ox=0; ox<WOUT; ox++)
          for (int n=0; n<N; n++) begin
            sum=0;
            for (int ky=0; ky<KH; ky++)
              for (int kx=0; kx<KW; kx++)
                for (int c=0; c<CIN; c++) begin
                  gy=OY0+oy+ky-1;
                  gx=OX0+ox+kx-1;
                  if (gy >= 0 && gy < 56 && gx >= 0 && gx < 56)
                    sum=sum + int'($signed(global_fmap[gy][gx][c]))
                            * int'($signed(global_kernel[ky][kx][c][OC0+n]));
                end
            golden[oy][ox][n]=sum;
          end

      // Independently calculated numerical anchors for M0, M32, M63.
      case (CASE_ID)
        0: begin
          known_first='{-2037,671,1346,273,-1636,-296,702,1453};
          known_middle='{2382,1205,-2518,-1700,4229,-1071,-2837,755};
          known_last='{1205,2382,-1514,-812,2569,-453,-2297,1122};
        end
        1: begin
          known_first='{2453,1672,-5949,3378,2122,-312,-2442,-1038};
          known_middle='{-1786,-3739,3390,-1565,1023,-2697,2646,370};
          known_last='{345,-3325,548,2844,67,-3812,1049,1616};
        end
        2: begin
          known_first='{3481,947,-827,-2582,1325,2667,-2204,-2591};
          known_middle='{-520,-1483,-52,2348,758,-2105,-579,1118};
          known_last='{1813,2329,-7111,3684,1635,-34,-3185,1074};
        end
        default: $fatal(1,"%m FAIL: no arithmetic anchors for CASE_ID=%0d",CASE_ID);
      endcase
      for (int n=0; n<N; n++) begin
        if (golden[0][0][n] !== known_first[n]
            || golden[4][0][n] !== known_middle[n]
            || golden[7][7][n] !== known_last[n])
          $fatal(1,"%m FAIL: global golden anchor mismatch case=%0d lane=%0d",CASE_ID,n);
      end
      $display("INFO: case=%0d global output origin=(%0d,%0d) channels=%0d..%0d padding_words=%0d; 24 anchors correct",
               CASE_ID,OY0,OX0,OC0,OC0+N-1,padded_words);
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
        $fatal(1,"%m FAIL: missing BRAM request M=%0d tile=%0d fetchK=%0d",em,ent,ek);
      if (fetch_k_addr !== ADDR_WIDTH'(ek))
        $fatal(1,"%m FAIL: fetchK=%0d expected=%0d M=%0d tile=%0d",
               fetch_k_addr,ek,em,ent);
      if (addr_valid !== 1'b1 || fmap_addr !== ADDR_WIDTH'(expected_fmap))
        $fatal(1,"%m FAIL: activation request M=%0d fetchK=%0d addr=%0d expected=%0d",
               em,ek,fmap_addr,expected_fmap);
      if (weight_word_addr !== ADDR_WIDTH'(expected_weight))
        $fatal(1,"%m FAIL: weight request tile=%0d fetchK=%0d addr=%0d expected=%0d",
               ent,ek,weight_word_addr,expected_weight);
      requests_by_tile[ent]=requests_by_tile[ent]+1;
      weight_request_hits[expected_weight]=weight_request_hits[expected_weight]+1;
    end
  endtask

  // Transaction checks use TB-owned expected indices, not DUT indices to
  // select a convenient golden value. No #delay here: check what RTL consumes.
  always @(posedge clk) begin : monitor
    integer em, ent, ek, ky, kx, ci, gy, gx;
    integer expected_value, got_value;
    if (!rst_n) begin
      writes_seen=0;
      clears_seen=0;
      feeds_seen=0;
      padded_feeds=0;
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
        $fatal(1,"%m FAIL: unknown control/status after reset");
      if (any_overflow !== 1'b0)
        $fatal(1,"%m FAIL: accumulator overflow");
      if ((clear_acc || feed_valid || write_valid || done) && !run_started)
        $fatal(1,"%m FAIL: transaction before start");
      if (completion_observed && (clear_acc || feed_valid || write_valid))
        $fatal(1,"%m FAIL: extra compute/write transaction after done");
      if ((clear_acc || feed_valid || write_valid) && busy !== 1'b1)
        $fatal(1,"%m FAIL: active transaction while busy is not asserted");

      em=writes_seen/NTILES;
      ent=writes_seen%NTILES;

      if (clear_acc) begin
        if (feed_valid || write_valid || round_open || writes_seen >= EXPECTED_WRITES)
          $fatal(1,"%m FAIL: invalid/overlapping CLEAR round=%0d",writes_seen);
        if (clears_seen !== writes_seen || feeds_seen !== writes_seen*K)
          $fatal(1,"%m FAIL: CLEAR round/count sequence");
        if (m_addr !== ADDR_WIDTH'(em) || n_base !== ADDR_WIDTH'(ent*ARRAY_COLS)
            || k_addr !== ADDR_WIDTH'(0))
          $fatal(1,"%m FAIL: CLEAR coordinates M=%0d Nbase=%0d K=%0d expected M=%0d tile=%0d",
                 m_addr,n_base,k_addr,em,ent);
        check_request(em,ent,0);
        round_open=1'b1;
        round_feeds=0;
        clears_seen=clears_seen+1;
      end

      if (feed_valid) begin
        if (!round_open || write_valid || round_feeds >= K || feeds_seen >= EXPECTED_FEEDS)
          $fatal(1,"%m FAIL: unexpected/excess FEED round=%0d Kcount=%0d",writes_seen,round_feeds);
        ek=round_feeds;
        if (m_addr !== ADDR_WIDTH'(em) || n_base !== ADDR_WIDTH'(ent*ARRAY_COLS)
            || k_addr !== ADDR_WIDTH'(ek))
          $fatal(1,"%m FAIL: FEED sequence got M=%0d Nbase=%0d K=%0d expected M=%0d tile=%0d K=%0d",
                 m_addr,n_base,k_addr,em,ent,ek);
        if (bram_data_valid !== 1'b1 || wgt_data_valid !== 1'b1)
          $fatal(1,"%m FAIL: missing aligned A/B data M=%0d tile=%0d K=%0d",em,ent,ek);
        ky=ek/(KW*CIN);
        kx=(ek/CIN)%KW;
        ci=ek%CIN;
        // Reference the global source, independently of the local preload.
        gy=OY0+(em/WOUT)+ky-1;
        gx=OX0+(em%WOUT)+kx-1;
        if (gy < 0 || gy >= 56 || gx < 0 || gx >= 56) begin
          expected_value=0;
          padded_feeds=padded_feeds+1;
        end else
          expected_value=int'($signed(global_fmap[gy][gx][ci]));
        got_value=int'($signed(act_data));
        if (got_value !== expected_value)
          $fatal(1,"%m FAIL: activation M=%0d tile=%0d K=%0d got=%0d expected=%0d",
                 em,ent,ek,got_value,expected_value);
        for (int lane=0; lane<ARRAY_COLS; lane++) begin
          expected_value=int'($signed(global_kernel[ky][kx][ci][OC0+ent*ARRAY_COLS+lane]));
          got_value=int'($signed(wgt_row_packed[lane*WGT_WIDTH +: WGT_WIDTH]));
          if (got_value !== expected_value)
            $fatal(1,"%m FAIL: weight alignment M=%0d tile=%0d K=%0d lane=%0d got=%0d expected=%0d",
                   em,ent,ek,lane,got_value,expected_value);
        end
        check_request(em,ent,(ek < K-1) ? ek+1 : ek);
        round_feeds=round_feeds+1;
        feeds_seen=feeds_seen+1;
        feeds_by_tile[ent]=feeds_by_tile[ent]+1;
      end else if (round_open && round_feeds < K) begin
        // CLEAR itself is allowed; the following 576 FEED beats are continuous.
        if (!clear_acc)
          $fatal(1,"%m FAIL: FEED gap before all 576 beats M=%0d tile=%0d count=%0d",
                 em,ent,round_feeds);
      end

      if (write_valid) begin
        if (!round_open || round_feeds !== K || writes_seen >= EXPECTED_WRITES)
          $fatal(1,"%m FAIL: early/extra write addr=%0d Kcount=%0d",write_addr,round_feeds);
        if (write_addr !== ADDR_WIDTH'(writes_seen)
            || write_m_addr !== ADDR_WIDTH'(em)
            || write_n_base !== ADDR_WIDTH'(ent*ARRAY_COLS))
          $fatal(1,"%m FAIL: write sequence addr=%0d M=%0d Nbase=%0d expected addr=%0d M=%0d tile=%0d",
                 write_addr,write_m_addr,write_n_base,writes_seen,em,ent);
        if (write_mask !== {ARRAY_COLS{1'b1}})
          $fatal(1,"%m FAIL: write mask=%h expected=FF",write_mask);
        if (word_write_hits[writes_seen] !== 0)
          $fatal(1,"%m FAIL: duplicate output word %0d",writes_seen);
        for (int lane=0; lane<ARRAY_COLS; lane++) begin
          expected_value=golden[em/WOUT][em%WOUT][ent*ARRAY_COLS+lane];
          got_value=$signed(write_data[lane*ACC_WIDTH +: ACC_WIDTH]);
          if (got_value !== expected_value)
            $fatal(1,"%m FAIL: output write M=%0d N=%0d got=%0d expected=%0d",
                   em,ent*ARRAY_COLS+lane,got_value,expected_value);
          write_outputs_checked=write_outputs_checked+1;
        end
        if ((writes_seen%8) == 7)
          $display("INFO: case=%0d checked output row=%0d (global y=%0d), last word=%0d",
                   CASE_ID,em/WOUT,OY0+em/WOUT,writes_seen);
        word_write_hits[writes_seen]=word_write_hits[writes_seen]+1;
        writes_by_tile[ent]=writes_by_tile[ent]+1;
        writes_seen=writes_seen+1;
        round_open=1'b0;
      end

      if (done) begin
        if (busy !== 1'b0 || round_open || writes_seen !== EXPECTED_WRITES
            || feeds_seen !== EXPECTED_FEEDS || done_seen !== 0)
          $fatal(1,"%m FAIL: premature/repeated done writes=%0d feeds=%0d",writes_seen,feeds_seen);
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
        $fatal(1,"%m FAIL: compute totals clear=%0d feed=%0d write=%0d outputs=%0d",
               clears_seen,feeds_seen,writes_seen,write_outputs_checked);
      if (padded_feeds !== EXPECTED_PAD_FEEDS)
        $fatal(1,"%m FAIL: padded FEED count=%0d expected=%0d",padded_feeds,EXPECTED_PAD_FEEDS);
      for (int t=0; t<NTILES; t++) begin
        if (feeds_by_tile[t] !== M*K || writes_by_tile[t] !== M
            || requests_by_tile[t] !== M*(K+1))
          $fatal(1,"%m FAIL: tile %0d coverage feeds=%0d writes=%0d requests=%0d",
                 t,feeds_by_tile[t],writes_by_tile[t],requests_by_tile[t]);
        for (int k=0; k<K; k++) begin
          // CLEAR requests K0; FEED requests K1..K575; final FEED repeats K575.
          expected_hits=(k == K-1) ? 2*M : M;
          if (weight_request_hits[t*K+k] !== expected_hits)
            $fatal(1,"%m FAIL: weight address %0d requests=%0d expected=%0d",
                   t*K+k,weight_request_hits[t*K+k],expected_hits);
        end
      end
      for (int a=0; a<EXPECTED_WRITES; a++)
        if (word_write_hits[a] !== 1)
          $fatal(1,"%m FAIL: output word %0d write count=%0d",a,word_write_hits[a]);
    end
  endtask

  initial begin : drive
    integer ky, kx, ci, em, ent;
    integer expected_value, got_value;
    finished=1'b0;
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
      $fatal(1,"%m FAIL: this regression requires 8x8 PE and INT8/INT32 widths");
    if (dut.u_baseline.H_IN != HIN || dut.u_baseline.W_IN != WIN
        || dut.u_baseline.C_IN != CIN || dut.u_baseline.C_OUT != N
        || dut.u_baseline.KH != KH || dut.u_baseline.KW != KW
        || dut.u_baseline.STRIDE != STRIDE || dut.u_baseline.PADDING != PADDING)
      $fatal(1,"%m FAIL: DUT convolution configuration differs from local 3x3/S1/P0 M64/N8/K576");
    if (dut.u_baseline.u_core.u_compute.M_LEN != M
        || dut.u_baseline.u_core.u_compute.N_LEN != N
        || dut.u_baseline.u_core.u_compute.K_LEN != K
        || dut.u_baseline.u_core.u_compute.N_TILES != NTILES)
      $fatal(1,"%m FAIL: DUT derived dimensions differ from test configuration");
    if (dut.u_baseline.u_core.u_compute.u_act_reader.u_mem.DEPTH < FMAP_WORDS
        || dut.u_baseline.u_core.u_compute.u_weight_reader.u_mem.DEPTH < WEIGHT_WORDS
        || dut.u_baseline.u_core.u_out_mem.DEPTH < EXPECTED_WRITES)
      $fatal(1,"%m FAIL: insufficient DUT BRAM depth");
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
    $display("INFO: case=%0d started M=64 N=8 K=576; expecting 64 rounds and 36864 FEED beats",CASE_ID);

    wait (done === 1'b1);
    #1;
    completion_observed=1'b1;
    check_compute_counts();
    if (busy !== 1'b0 || any_overflow !== 1'b0)
      $fatal(1,"%m FAIL: bad completion status");

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
          $fatal(1,"%m FAIL: missing read valid addr=%0d lane=%0d",a,lane);
        expected_value=golden[em/WOUT][em%WOUT][ent*ARRAY_COLS+lane];
        got_value=$signed(out_rd_data);
        if (got_value !== expected_value)
          $fatal(1,"%m FAIL: BRAM read addr=%0d lane=%0d M=%0d N=%0d got=%0d expected=%0d",
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
      $fatal(1,"%m FAIL: output read-valid did not deassert");

    // Continue observing briefly to reject extra writes or repeated done.
    repeat (3) @(negedge clk);
    check_compute_counts();
    if (done_seen !== 1 || busy !== 1'b0 || done !== 1'b0 || any_overflow !== 1'b0)
      $fatal(1,"%m FAIL: final status/done count done_seen=%0d",done_seen);
    if (readbacks_checked !== EXPECTED_OUTPUTS)
      $fatal(1,"%m FAIL: readback count=%0d expected=%0d",readbacks_checked,EXPECTED_OUTPUTS);
    for (int a=0; a<EXPECTED_WRITES; a++)
      for (int lane=0; lane<ARRAY_COLS; lane++)
        if (lane_read_hits[a][lane] !== 1)
          $fatal(1,"%m FAIL: missing/duplicate read addr=%0d lane=%0d",a,lane);

    $display("CASE PASS: id=%0d layer1.0.conv1 origin=(%0d,%0d) channels=%0d..%0d, M=64 N=8 K=576, 36864 K beats, 64 writes, 512/512 outputs and readbacks correct, padded_feeds=%0d",
             CASE_ID,OY0,OX0,OC0,OC0+N-1,padded_feeds);
    finished=1'b1;
  end

  initial begin : timeout
    repeat (TIMEOUT_CYCLES) @(posedge clk);
    if (finished !== 1'b1) $fatal(1,"%m FAIL: TIMEOUT after %0d cycles; done=%b busy=%b feeds=%0d writes=%0d readbacks=%0d",
           TIMEOUT_CYCLES,done,busy,feeds_seen,writes_seen,readbacks_checked);
  end
endmodule
