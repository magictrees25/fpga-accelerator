`timescale 1ns / 1ps

// ResNet18 conv1 P=1: global 224x224x3, 7x7/S2/P3 -> 112x112x64.
// One DUT/reset, local halo input 21x21x3, 7x7/S2/P0 -> 8x8x8.
// Local M64/N8/K147, global 14x14x8=1568 logical Tiles.
// FULL_LAYER=0: top-left Tile/channels 0..7 (512 outputs).
// FULL_LAYER=1: every global position/channel (802816 outputs).
// Global six-loop signed convolution golden is independent of Tile/DUT mapping.
// K tail: 18 complete groups plus K144..146, rows 0..2 only, then 13 drain clocks.
// Boundary zeros are preloaded by the TB, so local BRAM addresses remain valid.
// Synthetic signed INT8 inputs/weights; not a trained-image accuracy evaluation.
// User runs HDL tools. No compilation, simulation or synthesis at delivery.
module tb_conv1_7x7_p1_common #(
  parameter bit FULL_LAYER = 1'b0
);
  import pkg_accelerator::*;

  localparam int HIN=21, WIN=21, CIN=3, KH=7, KW=7, COUT=8;
  localparam int STRIDE=2, PADDING=0; // Local input already contains halo.
  localparam int GLOBAL_PADDING=3;
  localparam int DRAIN_EXPECTED=13, K_TAIL_START=144;
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
  localparam int TILE_ROWS_RUN=FULL_LAYER ? 14 : 1;
  localparam int TILE_COLS_RUN=FULL_LAYER ? 14 : 1;
  localparam int CHANNEL_GROUPS_RUN=FULL_LAYER ? 8 : 1;
  localparam int NUM_TILES=TILE_ROWS_RUN*TILE_COLS_RUN*CHANNEL_GROUPS_RUN;
  localparam int REGION_H=TILE_ROWS_RUN*HOUT;
  localparam int REGION_W=TILE_COLS_RUN*WOUT;
  localparam int REGION_N=CHANNEL_GROUPS_RUN*N;
  localparam int GLOBAL_H=112, GLOBAL_W=112, GLOBAL_N=64;
  localparam int GLOBAL_IN_H=224, GLOBAL_IN_W=224;
  localparam int GLOBAL_WORDS=GLOBAL_H*GLOBAL_W*GLOBAL_N;
  localparam int TOTAL_OUTPUTS=REGION_H*REGION_W*REGION_N;
  localparam int TIMEOUT_CYCLES=NUM_TILES*15000+1000;
  localparam int TILE_TIMEOUT_CYCLES=20000;
  localparam logic signed [31:0] UNWRITTEN=32'h5a55a55a;
  integer CASE_ID, OY0, OX0, OC0;
  integer PREV_OY0, PREV_OX0, PREV_OC0;
  logic rearm_checks, reset_released, finished;

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
  fpga_system_top_conv1_7x7_tile_p1 #(.ADDR_WIDTH(ADDR_WIDTH)) dut (
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
  wire array_enable;
  wire [ACT_WIDTH*ARRAY_ROWS-1:0] pe_act_in_packed;
  assign array_enable=dut.u_baseline.u_core.u_compute.u_ctrl.array_enable;
  assign pe_act_in_packed=dut.u_baseline.u_core.u_compute.u_ctrl.act_in_packed;
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
  logic signed [31:0] golden[0:GLOBAL_H-1][0:GLOBAL_W-1][0:GLOBAL_N-1];
  logic signed [31:0] stitched[0:GLOBAL_WORDS-1];
  integer scatter_hits[0:GLOBAL_WORDS-1];
  integer scatter_owner[0:GLOBAL_WORDS-1];
  logic signed [31:0] previous_output[0:EXPECTED_WRITES-1][0:ARRAY_COLS-1];
  logic [WGT_WIDTH*ARRAY_COLS-1:0] previous_weights[0:WEIGHT_WORDS-1];
  integer tile_hits[0:NUM_TILES-1];
  integer total_starts, total_dones, total_feeds, total_writes, total_clears;
  integer total_fmap_loads, total_weight_loads, changed_weight_words;
  integer total_read_requests, total_padded_loads, total_padded_feeds;
  integer total_tail_feeds, total_drain_cycles;
  integer total_readbacks, tiles_completed, changed_outputs_total;
  integer output_address_hits[0:EXPECTED_WRITES-1];
  logic start_q;

  logic signed [7:0] global_fmap[0:GLOBAL_IN_H-1][0:GLOBAL_IN_W-1][0:CIN-1];
  logic signed [7:0] global_kernel[0:KH-1][0:KW-1][0:CIN-1][0:GLOBAL_N-1];
  integer writes_seen, clears_seen, feeds_seen, done_seen;
  integer write_outputs_checked, readbacks_checked;
  integer round_feeds, round_tail_feeds, round_drain_cycles;
  integer row_beats[0:ARRAY_ROWS-1];
  integer padded_feeds;
  integer feeds_by_tile[0:NTILES-1], writes_by_tile[0:NTILES-1];
  integer requests_by_tile[0:NTILES-1];
  integer weight_request_hits[0:WEIGHT_WORDS-1];
  integer word_write_hits[0:EXPECTED_WRITES-1];
  integer lane_read_hits[0:EXPECTED_WRITES-1][0:ARRAY_COLS-1];
  logic round_open, run_started, completion_observed;

  // Compute the requested output region directly from the FULL global input.
  // Full-layer mode computes all 112x112x64 outputs; single mode computes 8x8x8.
  // No Tile origin, Tile buffer, flattened BRAM address or DUT signal is used.
  task automatic prepare_global_golden;
    integer value, iy, ix;
    integer signed sum;
    begin
      for (int y=0; y<GLOBAL_IN_H; y++)
        for (int x=0; x<GLOBAL_IN_W; x++)
          for (int c=0; c<CIN; c++)
            global_fmap[y][x][c]=8'(((17*y+11*x+7*c)%256)-128);
      for (int ky=0; ky<KH; ky++)
        for (int kx=0; kx<KW; kx++)
          for (int c=0; c<CIN; c++)
            for (int n=0; n<GLOBAL_N; n++)
              global_kernel[ky][kx][c][n]=8'(((13*ky+7*kx+3*c+5*n)%256)-128);
      for (int oy=0; oy<REGION_H; oy++)
        for (int ox=0; ox<REGION_W; ox++)
          for (int n=0; n<REGION_N; n++) begin
            sum=0;
            for (int ky=0; ky<KH; ky++)
              for (int kx=0; kx<KW; kx++)
                for (int c=0; c<CIN; c++) begin
                  iy=oy*STRIDE+ky-GLOBAL_PADDING;
                  ix=ox*STRIDE+kx-GLOBAL_PADDING;
                  if (iy >= 0 && iy < GLOBAL_IN_H && ix >= 0 && ix < GLOBAL_IN_W)
                    sum=sum + int'($signed(global_fmap[iy][ix][c]))
                            * int'($signed(global_kernel[ky][kx][c][n]));
                end
            golden[oy][ox][n]=sum;
          end
      // Independent arithmetic anchors; guards avoid uncomputed single-mode cells.
      if (golden[0][0][0] !== 151272)
        $fatal(1,"%m FAIL: golden anchor y=0 x=0 n=0");
      if (golden[0][0][7] !== 18552)
        $fatal(1,"%m FAIL: golden anchor y=0 x=0 n=7");
      if (golden[7][7][7] !== 111132)
        $fatal(1,"%m FAIL: golden anchor y=7 x=7 n=7");
      if (FULL_LAYER) begin
        if (golden[0][111][8] !== 3030)
          $fatal(1,"%m FAIL: golden anchor y=0 x=111 n=8");
        if (golden[111][0][16] !== 84510)
          $fatal(1,"%m FAIL: golden anchor y=111 x=0 n=16");
        if (golden[111][111][63] !== 145468)
          $fatal(1,"%m FAIL: golden anchor y=111 x=111 n=63");
        if (golden[55][55][31] !== -303180)
          $fatal(1,"%m FAIL: golden anchor y=55 x=55 n=31");
        if (golden[104][104][56] !== -282123)
          $fatal(1,"%m FAIL: golden anchor y=104 x=104 n=56");
      end
      $display("INFO: independent global 7x7/S2/P3 convolution golden: %0d outputs",TOTAL_OUTPUTS);
    end
  endtask

  task automatic prepare_tile;
    integer gy, gx;
    begin
      for (int iy=0; iy<HIN; iy++)
        for (int ix=0; ix<WIN; ix++)
          for (int c=0; c<CIN; c++) begin
            gy=OY0*STRIDE-GLOBAL_PADDING+iy;
            gx=OX0*STRIDE-GLOBAL_PADDING+ix;
            if (gy < 0 || gy >= GLOBAL_IN_H || gx < 0 || gx >= GLOBAL_IN_W)
              fmap[iy][ix][c]='0;
            else
              fmap[iy][ix][c]=global_fmap[gy][gx][c];
          end
      for (int ky=0; ky<KH; ky++)
        for (int kx=0; kx<KW; kx++)
          for (int c=0; c<CIN; c++)
            for (int n=0; n<N; n++)
              kernel[ky][kx][c][n]=global_kernel[ky][kx][c][OC0+n];
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
    if (!rst_n || rearm_checks) begin
      writes_seen=0;
      clears_seen=0;
      feeds_seen=0;
      done_seen=0;
      write_outputs_checked=0;
      round_feeds=0; round_tail_feeds=0; round_drain_cycles=0;
      padded_feeds=0;
      for (int row=0; row<ARRAY_ROWS; row++) row_beats[row]=0;
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
      if (act_rd_en !== (clear_acc || feed_valid)
          || wgt_rd_en !== (clear_acc || feed_valid))
        $fatal(1,"%m FAIL: unexpected/missing BRAM read enable");
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
        round_feeds=0; round_tail_feeds=0; round_drain_cycles=0;
        for (int row=0; row<ARRAY_ROWS; row++) row_beats[row]=0;
        if (array_enable !== 1'b1 || pe_act_in_packed !== '0)
          $fatal(1,"%m FAIL: invalid CLEAR inputs");
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
        gy=(OY0+em/WOUT)*STRIDE+ky-GLOBAL_PADDING;
        gx=(OX0+em%WOUT)*STRIDE+kx-GLOBAL_PADDING;
        if (gy < 0 || gy >= GLOBAL_IN_H || gx < 0 || gx >= GLOBAL_IN_W) begin
          expected_value=0;
          padded_feeds=padded_feeds+1;
        end else expected_value=int'($signed(global_fmap[gy][gx][ci]));
        got_value=int'($signed(act_data));
        if (got_value !== expected_value)
          $fatal(1,"%m FAIL: activation M=%0d tile=%0d K=%0d got=%0d expected=%0d",
                 em,ent,ek,got_value,expected_value);
        if (array_enable !== 1'b1)
          $fatal(1,"%m FAIL: array disabled during FEED");
        // No round-up to K152: last group uses only rows 0,1,2.
        for (int row=0; row<ARRAY_ROWS; row++) begin
          got_value=int'($signed(pe_act_in_packed[row*ACT_WIDTH +: ACT_WIDTH]));
          if (got_value !== ((row == ek%ARRAY_ROWS) ? expected_value : 0))
            $fatal(1,"%m FAIL: PE row injection K=%0d row=%0d got=%0d",ek,row,got_value);
        end
        row_beats[ek%ARRAY_ROWS]=row_beats[ek%ARRAY_ROWS]+1;
        if (ek >= K_TAIL_START) round_tail_feeds=round_tail_feeds+1;
        for (int lane=0; lane<ARRAY_COLS; lane++) begin
          expected_value=int'($signed(global_kernel[ky][kx][ci][OC0+ent*ARRAY_COLS+lane]));
          got_value=int'($signed(wgt_row_packed[lane*WGT_WIDTH +: WGT_WIDTH]));
          if (got_value !== expected_value)
            $fatal(1,"%m FAIL: weight alignment M=%0d tile=%0d K=%0d lane=%0d got=%0d expected=%0d",
                   em,ent,ek,lane,got_value,expected_value);
        end
        for (int lane=0; lane<ARRAY_COLS; lane++)
          if (dut.u_baseline.u_core.u_compute.u_ctrl.wgt_in[lane]
              !== $signed(wgt_row_packed[lane*WGT_WIDTH +: WGT_WIDTH]))
            $fatal(1,"%m FAIL: Controller-to-PE weight mismatch K=%0d lane=%0d",ek,lane);
        check_request(em,ent,(ek < K-1) ? ek+1 : ek);
        round_feeds=round_feeds+1;
        feeds_seen=feeds_seen+1;
        feeds_by_tile[ent]=feeds_by_tile[ent]+1;
      end else if (round_open && round_feeds < K) begin
        // CLEAR itself is allowed; the following 147 FEED beats are continuous.
        if (!clear_acc)
          $fatal(1,"%m FAIL: FEED gap before all 147 beats M=%0d tile=%0d count=%0d",
                 em,ent,round_feeds);
      end

      // Count exactly 13 zero-injection drain cycles, independently of state encoding.
      if (array_enable && !clear_acc && !feed_valid) begin
        if (!round_open || round_feeds !== K || round_drain_cycles >= DRAIN_EXPECTED
            || k_addr !== ADDR_WIDTH'(K-1) || pe_act_in_packed !== '0)
          $fatal(1,"%m FAIL: drain sequence/tail K=%0d feeds=%0d drain=%0d",k_addr,round_feeds,round_drain_cycles);
        for (int lane=0; lane<ARRAY_COLS; lane++)
          if (dut.u_baseline.u_core.u_compute.u_ctrl.wgt_in[lane] !== '0)
            $fatal(1,"%m FAIL: nonzero injected weight during drain");
        round_drain_cycles=round_drain_cycles+1;
      end

      if (write_valid) begin
        if (round_tail_feeds !== 3 || round_drain_cycles !== DRAIN_EXPECTED)
          $fatal(1,"%m FAIL: K147 tail/drain count tail=%0d drain=%0d",round_tail_feeds,round_drain_cycles);
        for (int row=0; row<ARRAY_ROWS; row++)
          if (row_beats[row] !== ((row < 3) ? 19 : 18))
            $fatal(1,"%m FAIL: K147 row coverage row=%0d beats=%0d",row,row_beats[row]);
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
          expected_value=golden[OY0+em/WOUT][OX0+em%WOUT][OC0+ent*ARRAY_COLS+lane];
          got_value=$signed(write_data[lane*ACC_WIDTH +: ACC_WIDTH]);
          if (got_value !== expected_value)
            $fatal(1,"%m FAIL: output write M=%0d N=%0d got=%0d expected=%0d",
                   em,ent*ARRAY_COLS+lane,got_value,expected_value);
          write_outputs_checked=write_outputs_checked+1;
        end
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
      for (int t=0; t<NTILES; t++) begin
        if (feeds_by_tile[t] !== M*K || writes_by_tile[t] !== M
            || requests_by_tile[t] !== M*(K+1))
          $fatal(1,"%m FAIL: tile %0d coverage feeds=%0d writes=%0d requests=%0d",
                 t,feeds_by_tile[t],writes_by_tile[t],requests_by_tile[t]);
        for (int k=0; k<K; k++) begin
          // CLEAR requests K0; FEED requests K1..K146; final FEED repeats K146.
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


  // Lifetime monitor: these counters are NEVER cleared between Tiles.
  // rearm_checks resets only the per-Tile TB scoreboard, never the DUT.
  always @(posedge clk) begin : lifetime_monitor
    integer idx, iy, ix, ci, ky, kx, gy, gx;
    integer expected_value;
    logic [WGT_WIDTH*ARRAY_COLS-1:0] expected_word;
    if (!rst_n) begin
      if (reset_released)
        $fatal(1,"%m FAIL: DUT reset was asserted between Tiles");
      total_starts=0; total_dones=0; total_feeds=0;
      total_writes=0; total_clears=0;
      total_fmap_loads=0; total_weight_loads=0;
      changed_weight_words=0;
      total_read_requests=0;
      total_padded_loads=0; total_padded_feeds=0;
      total_tail_feeds=0; total_drain_cycles=0;
      start_q=1'b0;
      for (int a=0; a<EXPECTED_WRITES; a++) output_address_hits[a]=0;
    end else begin
      if ($isunknown({start,busy,done,clear_acc,feed_valid,write_valid,
                      fmap_wr_en,weight_wr_en,out_rd_en,any_overflow}))
        $fatal(1,"%m FAIL: unknown control/status tile=%0d",CASE_ID);
      if (any_overflow)
        $fatal(1,"%m FAIL: overflow tile=%0d",CASE_ID);
      if (rearm_checks && (busy || done || clear_acc || feed_valid || write_valid || start))
        $fatal(1,"%m FAIL: unexpected activity during Tile loading tile=%0d",CASE_ID);
      if ((fmap_wr_en || weight_wr_en) && (!rearm_checks || busy || done || start))
        $fatal(1,"%m FAIL: preload overlaps computation tile=%0d",CASE_ID);
      if (fmap_wr_en && weight_wr_en)
        $fatal(1,"%m FAIL: unexpected simultaneous preload ports");

      if (weight_wr_en) begin
        idx=total_weight_loads%WEIGHT_WORDS;
        if (total_weight_loads/WEIGHT_WORDS !== CASE_ID
            || weight_wr_addr !== ADDR_WIDTH'(idx))
          $fatal(1,"%m FAIL: weight preload order tile=%0d addr=%0d expected=%0d",
                 CASE_ID,weight_wr_addr,idx);
        ky=idx/(KW*CIN); kx=(idx/CIN)%KW; ci=idx%CIN;
        expected_word='0;
        for (int lane=0; lane<ARRAY_COLS; lane++)
          expected_word[lane*WGT_WIDTH +: WGT_WIDTH]=global_kernel[ky][kx][ci][OC0+lane];
        if (weight_wr_data !== expected_word)
          $fatal(1,"%m FAIL: wrong channel-group weight preload tile=%0d addr=%0d",CASE_ID,idx);
        if (CASE_ID > 0) begin
          if (weight_wr_data === previous_weights[idx])
            $fatal(1,"%m FAIL: weight word not changed on channel-group switch tile=%0d addr=%0d",CASE_ID,idx);
          changed_weight_words=changed_weight_words+1;
        end
        previous_weights[idx]=weight_wr_data;
        total_weight_loads=total_weight_loads+1;
      end
      if (fmap_wr_en) begin
        idx=total_fmap_loads%FMAP_WORDS;
        if (total_fmap_loads/FMAP_WORDS !== CASE_ID
            || fmap_wr_addr !== ADDR_WIDTH'(idx))
          $fatal(1,"%m FAIL: activation preload order tile=%0d addr=%0d expected=%0d",CASE_ID,fmap_wr_addr,idx);
        iy=idx/(WIN*CIN); ix=(idx/CIN)%WIN; ci=idx%CIN;
        gy=OY0*STRIDE-GLOBAL_PADDING+iy; gx=OX0*STRIDE-GLOBAL_PADDING+ix;
        if (gy < 0 || gy >= GLOBAL_IN_H || gx < 0 || gx >= GLOBAL_IN_W) begin
          expected_value=0;
          total_padded_loads=total_padded_loads+1;
        end else expected_value=int'($signed(global_fmap[gy][gx][ci]));
        if (int'($signed(fmap_wr_data)) !== expected_value)
          $fatal(1,"%m FAIL: activation preload tile=%0d addr=%0d",CASE_ID,idx);
        total_fmap_loads=total_fmap_loads+1;
      end
      if (start) begin
        if (start_q || busy || done || total_starts !== CASE_ID
            || total_dones !== CASE_ID || tiles_completed !== CASE_ID
            || total_fmap_loads !== (CASE_ID+1)*FMAP_WORDS
            || total_weight_loads !== (CASE_ID+1)*WEIGHT_WORDS)
          $fatal(1,"%m FAIL: start pulse/restart/preload sequence tile=%0d starts=%0d dones=%0d",
                 CASE_ID,total_starts,total_dones);
        total_starts=total_starts+1;
      end
      start_q=start;
      if (clear_acc) total_clears=total_clears+1;
      if (feed_valid) begin
        total_feeds=total_feeds+1;
        if (int'(k_addr) >= K_TAIL_START) total_tail_feeds=total_tail_feeds+1;
        gy=(OY0+int'(m_addr)/WOUT)*STRIDE+int'(k_addr)/(KW*CIN)-GLOBAL_PADDING;
        gx=(OX0+int'(m_addr)%WOUT)*STRIDE+(int'(k_addr)/CIN)%KW-GLOBAL_PADDING;
        if (gy < 0 || gy >= GLOBAL_IN_H || gx < 0 || gx >= GLOBAL_IN_W)
          total_padded_feeds=total_padded_feeds+1;
      end
      if (array_enable && !clear_acc && !feed_valid)
        total_drain_cycles=total_drain_cycles+1;
      if (clear_acc || feed_valid) total_read_requests=total_read_requests+1;
      if (write_valid) begin
        if ($isunknown(write_addr) || int'(write_addr) >= EXPECTED_WRITES)
          $fatal(1,"%m FAIL: output address outside reused local cache");
        if (output_address_hits[int'(write_addr)] !== CASE_ID)
          $fatal(1,"%m FAIL: cache overwrite generation tile=%0d addr=%0d previous_writes=%0d",
                 CASE_ID,write_addr,output_address_hits[int'(write_addr)]);
        output_address_hits[int'(write_addr)]=output_address_hits[int'(write_addr)]+1;
        total_writes=total_writes+1;
      end
      if (done) begin
        if (total_dones !== CASE_ID || total_starts !== CASE_ID+1)
          $fatal(1,"%m FAIL: repeated/missing done tile=%0d",CASE_ID);
        total_dones=total_dones+1;
      end
    end
  end

  // Also detect a reset pulse shorter than a clock period after initial release.
  always @(negedge rst_n)
    if (reset_released === 1'b1)
      $fatal(1,"%m FAIL: inter-Tile reset is forbidden in this regression");

  task automatic load_tile;
    integer ky, kx, ci;
    begin
      for (int k=0; k<K; k++) begin
        ky=k/(KW*CIN); kx=(k/CIN)%KW; ci=k%CIN;
        @(negedge clk);
        weight_wr_addr=ADDR_WIDTH'(k);
        weight_wr_data='0;
        for (int lane=0; lane<ARRAY_COLS; lane++)
          weight_wr_data[lane*WGT_WIDTH +: WGT_WIDTH]=kernel[ky][kx][ci][lane];
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
      if (total_fmap_loads !== (CASE_ID+1)*FMAP_WORDS
          || total_weight_loads !== (CASE_ID+1)*WEIGHT_WORDS)
        $fatal(1,"%m FAIL: incomplete preload tile=%0d",CASE_ID);
    end
  endtask

  task automatic read_and_scatter;
    integer gy, gx, gn, global_addr, expected_value, got_value;
    integer changed_outputs, expected_changed;
    begin
      changed_outputs=0;
      expected_changed=0;
      for (int a=0; a<EXPECTED_WRITES; a++)
        for (int lane=0; lane<ARRAY_COLS; lane++) begin
          @(negedge clk);
          out_rd_en=1'b1;
          out_rd_addr=ADDR_WIDTH'(a);
          out_rd_lane_sel=LANE_SEL_WIDTH'(lane);
          @(posedge clk);
          #1;
          if (out_rd_valid !== 1'b1 || out_rd_lane_valid !== 1'b1)
            $fatal(1,"%m FAIL: missing read valid tile=%0d addr=%0d lane=%0d",CASE_ID,a,lane);
          gy=OY0+a/WOUT; gx=OX0+a%WOUT; gn=OC0+lane;
          expected_value=golden[gy][gx][gn];
          if (CASE_ID > 0 && expected_value !== golden[PREV_OY0+a/WOUT][PREV_OX0+a%WOUT][PREV_OC0+lane])
            expected_changed=expected_changed+1;
          got_value=$signed(out_rd_data);
          if (got_value !== expected_value)
            $fatal(1,"%m FAIL: readback tile=%0d addr=%0d lane=%0d global=(%0d,%0d,%0d) got=%0d expected=%0d",
                   CASE_ID,a,lane,gy,gx,gn,got_value,expected_value);
          if (CASE_ID > 0 && got_value !== previous_output[a][lane])
            changed_outputs=changed_outputs+1;
          previous_output[a][lane]=got_value;
          // Full-layer HWC pitch is 112x64, even for the single-Tile test.
          global_addr=(gy*GLOBAL_W+gx)*GLOBAL_N+gn;
          if (gy < 0 || gy >= REGION_H || gx < 0 || gx >= REGION_W
              || gn < 0 || gn >= REGION_N || global_addr < 0 || global_addr >= GLOBAL_WORDS)
            $fatal(1,"%m FAIL: scatter outside selected global region");
          if (scatter_hits[global_addr] !== 0)
            $fatal(1,"%m FAIL: duplicate scatter global_addr=%0d tile=%0d old_owner=%0d",
                   global_addr,CASE_ID,scatter_owner[global_addr]);
          stitched[global_addr]=got_value;
          scatter_hits[global_addr]=scatter_hits[global_addr]+1;
          scatter_owner[global_addr]=CASE_ID;
          lane_read_hits[a][lane]=lane_read_hits[a][lane]+1;
          readbacks_checked=readbacks_checked+1;
          total_readbacks=total_readbacks+1;
        end
      @(negedge clk);
      out_rd_en=1'b0;
      @(posedge clk);
      #1;
      if (out_rd_valid !== 1'b0 || out_rd_lane_valid !== 1'b0)
        $fatal(1,"%m FAIL: read-valid did not deassert");
      // Compare change coverage against previous GLOBAL golden coordinates,
      // not a value inferred from the DUT or cached output.
      if (CASE_ID > 0 && expected_changed == 0)
        $fatal(1,"%m FAIL: stimulus does not exercise output-cache replacement");
      if (changed_outputs !== expected_changed)
        $fatal(1,"%m FAIL: stale-cache/change coverage tile=%0d changed=%0d expected=%0d",
               CASE_ID,changed_outputs,expected_changed);
      changed_outputs_total=changed_outputs_total+changed_outputs;
    end
  endtask

  task automatic check_global_scatter;
    integer addr, expected_owner, compared;
    longint signed checksum;
    begin
      compared=0; checksum=0;
      // Independent global-coordinate traversal, not Tile iteration.
      // Also verify untouched positions of the entire full-layer buffer.
      for (int y=0; y<GLOBAL_H; y++)
        for (int x=0; x<GLOBAL_W; x++)
          for (int n=0; n<GLOBAL_N; n++) begin
            addr=(y*GLOBAL_W+x)*GLOBAL_N+n;
            if (y < REGION_H && x < REGION_W && n < REGION_N) begin
              expected_owner=((y/HOUT)*TILE_COLS_RUN+x/WOUT)*CHANNEL_GROUPS_RUN+n/N;
              if (scatter_hits[addr] !== 1 || scatter_owner[addr] !== expected_owner)
                $fatal(1,"%m FAIL: scatter coverage/owner global=(%0d,%0d,%0d) hits=%0d owner=%0d expected_owner=%0d",
                       y,x,n,scatter_hits[addr],scatter_owner[addr],expected_owner);
              if (stitched[addr] !== golden[y][x][n])
                $fatal(1,"%m FAIL: assembled output global=(%0d,%0d,%0d) got=%0d expected=%0d",
                       y,x,n,stitched[addr],golden[y][x][n]);
              compared=compared+1;
              checksum=checksum+stitched[addr];
            end else if (scatter_hits[addr] !== 0 || scatter_owner[addr] !== -1
                         || stitched[addr] !== UNWRITTEN)
              $fatal(1,"%m FAIL: write outside requested global region addr=%0d",addr);
          end
      if (compared !== TOTAL_OUTPUTS || checksum !== (FULL_LAYER ? 64'sd412681344 : -64'sd5442128))
        $fatal(1,"%m FAIL: full scatter count/checksum count=%0d checksum=%0d",compared,checksum);
    end
  endtask

  initial begin : drive
    integer wait_cycles;
    rst_n=1'b0; reset_released=1'b0; rearm_checks=1'b1; finished=1'b0;
    start=1'b0; run_started=1'b0; completion_observed=1'b0;
    fmap_wr_en=1'b0; fmap_wr_addr='0; fmap_wr_data='0;
    weight_wr_en=1'b0; weight_wr_addr='0; weight_wr_data='0;
    out_rd_en=1'b0; out_rd_addr='0; out_rd_lane_sel='0;
    CASE_ID=0; OY0=0; OX0=0; OC0=0;
    PREV_OY0=0; PREV_OX0=0; PREV_OC0=0;
    tiles_completed=0; total_readbacks=0; changed_outputs_total=0;
    readbacks_checked=0;
    for (int t=0; t<NUM_TILES; t++) tile_hits[t]=0;
    for (int a=0; a<GLOBAL_WORDS; a++) begin
      stitched[a]=UNWRITTEN; scatter_hits[a]=0; scatter_owner[a]=-1;
    end

    if (ARRAY_ROWS != 8 || ARRAY_COLS != 8 || ACT_WIDTH != 8
        || WGT_WIDTH != 8 || ACC_WIDTH != 32)
      $fatal(1,"%m FAIL: test requires 8x8 PE and INT8/INT32");
    if (dut.u_baseline.H_IN != HIN || dut.u_baseline.W_IN != WIN
        || dut.u_baseline.C_IN != CIN || dut.u_baseline.C_OUT != N
        || dut.u_baseline.KH != KH || dut.u_baseline.KW != KW
        || dut.u_baseline.STRIDE != STRIDE || dut.u_baseline.PADDING != PADDING
        || dut.u_baseline.ADDR_WIDTH != ADDR_WIDTH
        || dut.u_baseline.u_core.u_compute.M_LEN != M
        || dut.u_baseline.u_core.u_compute.N_LEN != N
        || dut.u_baseline.u_core.u_compute.K_LEN != K
        || dut.u_baseline.u_core.u_compute.N_TILES != NTILES)
      $fatal(1,"%m FAIL: P1 conv1 7x7 Tile DUT configuration mismatch");
    if (dut.u_baseline.u_core.u_compute.u_act_reader.u_mem.DEPTH != FMAP_WORDS
        || dut.u_baseline.u_core.u_compute.u_weight_reader.u_mem.DEPTH != WEIGHT_WORDS
        || dut.u_baseline.u_core.u_out_mem.DEPTH != EXPECTED_WRITES)
      $fatal(1,"%m FAIL: P1 conv1 7x7 Tile memory layout mismatch");
    if (dut.u_baseline.u_core.u_compute.u_ctrl.u_kctrl.DRAIN_CYCLES != DRAIN_EXPECTED)
      $fatal(1,"%m FAIL: unexpected K Controller drain configuration");
    prepare_global_golden();

    repeat (3) @(negedge clk);
    rst_n=1'b1;
    reset_released=1'b1;
    for (int tile_y=0; tile_y<TILE_ROWS_RUN; tile_y++)
      for (int tile_x=0; tile_x<TILE_COLS_RUN; tile_x++)
        for (int channel_group=0; channel_group<CHANNEL_GROUPS_RUN; channel_group++) begin
          @(negedge clk);
          if (busy !== 1'b0 || done !== 1'b0 || start !== 1'b0)
            $fatal(1,"%m FAIL: DUT not idle before next Tile");
          CASE_ID=(tile_y*TILE_COLS_RUN+tile_x)*CHANNEL_GROUPS_RUN+channel_group;
          OY0=tile_y*HOUT; OX0=tile_x*WOUT; OC0=channel_group*N;
          rearm_checks=1'b1;
          run_started=1'b0; completion_observed=1'b0;
          readbacks_checked=0;
          for (int a=0; a<EXPECTED_WRITES; a++)
            for (int lane=0; lane<ARRAY_COLS; lane++) lane_read_hits[a][lane]=0;
          if (tile_hits[CASE_ID] !== 0)
            $fatal(1,"%m FAIL: duplicate Tile scheduling");
          prepare_tile();
          load_tile();
          // Rearm only the TB scoreboard. rst_n stays high across every run.
          rearm_checks=1'b0;
          @(negedge clk);
          run_started=1'b1;
          start=1'b1;
          @(negedge clk);
          start=1'b0;
          $display("INFO: START tile=%0d origin=(%0d,%0d) channels=%0d..%0d reset_count=1",
                   CASE_ID,OY0,OX0,OC0,OC0+N-1);

          wait_cycles=0;
          while (done !== 1'b1 && wait_cycles < TILE_TIMEOUT_CYCLES) begin
            @(posedge clk);
            #1;
            wait_cycles=wait_cycles+1;
          end
          if (done !== 1'b1)
            $fatal(1,"%m FAIL: Tile timeout tile=%0d cycles=%0d feeds=%0d writes=%0d",
                   CASE_ID,wait_cycles,feeds_seen,writes_seen);
          completion_observed=1'b1;
          check_compute_counts();
          if (busy !== 1'b0 || any_overflow !== 1'b0)
            $fatal(1,"%m FAIL: invalid completion tile=%0d",CASE_ID);
          read_and_scatter();
          repeat (3) @(negedge clk);
          check_compute_counts();
          if (done_seen !== 1 || busy !== 1'b0 || done !== 1'b0
              || total_dones !== CASE_ID+1 || readbacks_checked !== EXPECTED_OUTPUTS)
            $fatal(1,"%m FAIL: Tile completion/readback count tile=%0d",CASE_ID);
          for (int a=0; a<EXPECTED_WRITES; a++)
            for (int lane=0; lane<ARRAY_COLS; lane++)
              if (lane_read_hits[a][lane] !== 1)
                $fatal(1,"%m FAIL: missing/duplicate read tile=%0d addr=%0d lane=%0d",CASE_ID,a,lane);
          tile_hits[CASE_ID]=tile_hits[CASE_ID]+1;
          tiles_completed=tiles_completed+1;
          PREV_OY0=OY0; PREV_OX0=OX0; PREV_OC0=OC0;
          $display("TILE PASS: P1 conv1 7x7 id=%0d origin=(%0d,%0d) channels=%0d..%0d, 9408 K FEED cycles, 64 writes, 512/512 outputs read and stitched, padded_feeds=%0d",
                   CASE_ID,OY0,OX0,OC0,OC0+N-1,padded_feeds);
        end

    repeat (5) @(negedge clk);
    check_global_scatter();
    if (tiles_completed !== NUM_TILES || total_starts !== NUM_TILES
        || total_dones !== NUM_TILES || total_readbacks !== TOTAL_OUTPUTS
        || total_feeds !== NUM_TILES*EXPECTED_FEEDS
        || total_writes !== NUM_TILES*EXPECTED_WRITES
        || total_clears !== NUM_TILES*EXPECTED_WRITES
        || total_fmap_loads !== NUM_TILES*FMAP_WORDS
        || total_weight_loads !== NUM_TILES*WEIGHT_WORDS
        || changed_weight_words !== (NUM_TILES-1)*WEIGHT_WORDS
        || total_read_requests !== NUM_TILES*M*(K+1)
        || changed_outputs_total !== (FULL_LAYER ? 802279 : 0)
        || total_padded_loads !== (FULL_LAYER ? 69960 : 351)
        || total_padded_feeds !== (FULL_LAYER ? 224928 : 1296)
        || total_tail_feeds !== NUM_TILES*M*3
        || total_drain_cycles !== NUM_TILES*M*DRAIN_EXPECTED
        || rst_n !== 1'b1 || busy !== 1'b0 || done !== 1'b0 || any_overflow !== 1'b0)
      $fatal(1,"%m FAIL: lifetime counts tiles=%0d starts=%0d done=%0d feeds=%0d writes=%0d reads=%0d",
             tiles_completed,total_starts,total_dones,total_feeds,total_writes,total_readbacks);
    for (int t=0; t<NUM_TILES; t++)
      if (tile_hits[t] !== 1) $fatal(1,"%m FAIL: Tile coverage id=%0d",t);
    for (int a=0; a<EXPECTED_WRITES; a++)
      if (output_address_hits[a] !== NUM_TILES)
        $fatal(1,"%m FAIL: reused output cache word=%0d generations=%0d",a,output_address_hits[a]);
    finished=1'b1;
    $display("INFO: one DUT, one reset, %0d starts/dones; %0d weight-group switches, %0d changed cached results checked",
             total_starts,NUM_TILES-1,changed_outputs_total);
    $display("INFO: global HWC scatter pitch=112x64; %0d positions written exactly once; %0d memory requests per Reader",
             TOTAL_OUTPUTS,total_read_requests);
    $display("INFO: padding load bytes=%0d, padding K FEED=%0d, K144..146 tail FEED=%0d, drain clocks=%0d",
             total_padded_loads,total_padded_feeds,total_tail_feeds,total_drain_cycles);
    $display("PASS: P1 ResNet18 conv1 7x7 S2 P3, %0d Tile, %0d writes, %0d K FEED cycles, %0d/%0d INT32 outputs and global stitching correct",
             NUM_TILES,total_writes,total_feeds,total_readbacks,TOTAL_OUTPUTS);
    $finish;
  end

  initial begin : timeout
    repeat (TIMEOUT_CYCLES) @(posedge clk);
    if (finished !== 1'b1)
      $fatal(1,"%m FAIL: global TIMEOUT tile=%0d completed=%0d starts=%0d done=%0d feeds=%0d writes=%0d",
             CASE_ID,tiles_completed,total_starts,total_dones,total_feeds,total_writes);
  end
endmodule
