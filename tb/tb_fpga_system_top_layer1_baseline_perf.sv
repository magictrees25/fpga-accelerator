`timescale 1ns / 1ps

// Baseline performance instrumentation of the PASS full-layer 392-Tile TB.
// Adds only passive measurements/CSV reporting; all functional checks remain.
// Outputs layer1_baseline_tiles.csv and layer1_baseline_summary.csv at runtime.
// Cycles are sampled from DUT transactions, not inferred from wall-clock runtime.
// Utilization = logical MAC work / (64 PE * measured cycles), NOT toggle activity.
// Preload/read payloads are on present BRAM ports, NOT measured DDR bandwidth.
// Source-state estimates for comparison ONLY: 37952 start-to-done cycles/Tile,
// 14877184 compute cycles/layer, 12.14165% dense compute utilization.
// No latency is hardcoded as a passing requirement; measure the actual run.
// Simulation Top: tb_fpga_system_top_layer1_baseline_perf
// ONE existing fpga_system_top_layer1_tile DUT, reset ONCE.
// Global input/output 56x56x64; all y/x=0..55, channels=0..63.
// Tile order: 7 spatial rows -> 7 spatial columns -> 8 channel groups.
// Local 10x10x64 includes halo, 3x3/S1/P0 -> 8x8x8; M64/N8/K576.
// TB loads, starts, waits, reads, scatters and reuses the SAME memories.
// Independent six-loop global convolution golden uses global padding=1.
// Synthetic signed INT8 data, INT32 results; no trained-weight accuracy test.
// Only TB is new. No RTL scheduler, PE/controller edits, or AXI DMA.
// Local N_TILES remains 1: each global channel group replaces words 0..575.
// Expected totals: 392 starts/dones, 14450688 FEED beats, 25088 writes, 200704 reads.
// Global scatter uses full-layer HWC pitch 56x64; every position written once.
// Add to Simulation Sources; set this module as Simulation Top; use Run All.
// Global timeout 400 ms, per-compute timeout 600 us (10 ns clock).
// User runs Vivado; this file has not been compiled/simulated here.
module tb_fpga_system_top_layer1_baseline_perf;
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
  localparam int REGION_H=56, REGION_W=56, REGION_N=64;
  localparam int TILES_Y=REGION_H/HOUT, TILES_X=REGION_W/WOUT;
  localparam int CHANNEL_GROUPS=REGION_N/N;
  localparam int NUM_TILES=TILES_Y*TILES_X*CHANNEL_GROUPS;
  localparam int GLOBAL_H=56, GLOBAL_W=56, GLOBAL_N=64;
  localparam int GLOBAL_WORDS=GLOBAL_H*GLOBAL_W*GLOBAL_N;
  localparam int TOTAL_OUTPUTS=REGION_H*REGION_W*REGION_N;
  localparam int TIMEOUT_CYCLES=40000000;
  localparam int TILE_TIMEOUT_CYCLES=60000;
  localparam logic signed [31:0] UNWRITTEN=32'h5a55a55a;
  integer CASE_ID, OY0, OX0, OC0;
  integer top_tiles, bottom_tiles, left_tiles, right_tiles;
  integer corner_tiles, edge_tiles, interior_tiles;
  integer EXPECTED_PAD_WORDS, EXPECTED_PAD_FEEDS;
  logic rearm_checks, reset_released, finished;


  // Performance instrumentation: read-only probes, no DUT control changes.
  // Timing convention: start accepted at rising edge s, done asserted AFTER
  // edge d (sampled at #1); latency=d-s. Count phases at edges s+1..d.
  // End-to-end: first accepted preload through last read response, inclusive.
  // This excludes reset/Golden generation/final TB checks, includes host gaps.
  // Counts refer to payloads at existing BRAM ports, NOT AXI/DDR transactions.
  localparam real PERF_CLOCK_NS=10.0; // matches always #5 clk, NOT measured Fmax
  localparam int P_COMPUTE=0, P_CLEAR=1, P_FEED=2, P_DRAIN=3;
  localparam int P_LAUNCH=4, P_KDONE=5, P_WRITE=6, P_OTHER=7, P_ENABLED=8;
  localparam int P_ALOAD=9, P_WLOAD=10, P_RREQ=11, P_RRET=12;
  localparam int P_PADLOAD=13, P_AREPEAT=14, P_WREPEAT=15, P_ANEW=16, P_WNEW=17;
  localparam int P_COUNTS=18, P_START=18, P_DONE=19, P_FIRSTLOAD=20;
  localparam int P_LASTREAD=21, P_FIRSTREAD=22, P_FIELDS=23;
  localparam int PERF_INPUT_WORDS=GLOBAL_H*GLOBAL_W*CIN;
  localparam int PERF_WEIGHT_WORDS=CHANNEL_GROUPS*WEIGHT_WORDS;
  localparam longint unsigned PERF_DENSE_MACS=115605504;
  localparam int PERF_PE_COUNT=ARRAY_ROWS*ARRAY_COLS;
  wire perf_array_enable=dut.u_baseline.u_core.u_compute.u_ctrl.array_enable;
  wire perf_k_start=dut.u_baseline.u_core.u_compute.u_ctrl.k_start;
  wire perf_k_done=dut.u_baseline.u_core.u_compute.u_ctrl.k_done;
  // Actual k_tile_controller asserts enable in CLEAR, FEED, DRAIN only.
  wire perf_drain=perf_array_enable && !clear_acc && !feed_valid;
  longint unsigned perf_tile[0:NUM_TILES-1][0:P_FIELDS-1];
  longint unsigned perf_sum[0:P_COUNTS-1];
  integer perf_input_hits[0:PERF_INPUT_WORDS-1];
  integer perf_weight_hits[0:PERF_WEIGHT_WORDS-1];
  logic perf_tile_done[0:NUM_TILES-1];
  logic perf_computing, perf_session_started, perf_session_finished;
  longint unsigned perf_tick, perf_first_tick, perf_last_tick;
  longint unsigned perf_session_cycles, perf_host_gap, perf_rows;
  integer perf_tile_fd, perf_summary_fd;
  string perf_tile_path="layer1_baseline_tiles.csv";
  string perf_summary_path="layer1_baseline_summary.csv";

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
  logic signed [31:0] golden[0:REGION_H-1][0:REGION_W-1][0:REGION_N-1];
  logic signed [31:0] stitched[0:GLOBAL_WORDS-1];
  integer scatter_hits[0:GLOBAL_WORDS-1];
  integer scatter_owner[0:GLOBAL_WORDS-1];
  logic signed [31:0] previous_output[0:EXPECTED_WRITES-1][0:ARRAY_COLS-1];
  logic [WGT_WIDTH*ARRAY_COLS-1:0] previous_weights[0:WEIGHT_WORDS-1];
  integer tile_hits[0:NUM_TILES-1];
  integer total_starts, total_dones, total_feeds, total_writes, total_clears;
  integer total_fmap_loads, total_weight_loads, changed_weight_words;
  integer total_padded_loads, total_padded_feeds;
  integer total_readbacks, tiles_completed, changed_outputs_total;
  integer output_address_hits[0:EXPECTED_WRITES-1];
  logic start_q;

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

  // Compute the FULL output directly from the FULL input, without Tile loops.
  // No Tile origin, Tile buffer, flattened BRAM address or DUT signal is used.
  task automatic prepare_global_golden;
    integer value, iy, ix;
    integer signed sum;
    begin
      $display("INFO: computing independent full-layer 56x56x64 Golden at time zero; please wait");
      for (int y=0; y<GLOBAL_H; y++)
        for (int x=0; x<GLOBAL_W; x++)
          for (int c=0; c<CIN; c++)
            global_fmap[y][x][c]=8'(((17*y+11*x+7*c)%127)-63);
      for (int ky=0; ky<KH; ky++)
        for (int kx=0; kx<KW; kx++)
          for (int c=0; c<CIN; c++)
            for (int n=0; n<GLOBAL_N; n++)
              global_kernel[ky][kx][c][n]=8'(((13*ky+7*kx+3*c+5*n)%19)-9);
      for (int oy=0; oy<REGION_H; oy++)
        for (int ox=0; ox<REGION_W; ox++)
          for (int n=0; n<REGION_N; n++) begin
            sum=0;
            for (int ky=0; ky<KH; ky++)
              for (int kx=0; kx<KW; kx++)
                for (int c=0; c<CIN; c++) begin
                  iy=oy+ky-1;
                  ix=ox+kx-1;
                  if (iy >= 0 && iy < GLOBAL_H && ix >= 0 && ix < GLOBAL_W)
                    sum=sum + int'($signed(global_fmap[iy][ix][c]))
                            * int'($signed(global_kernel[ky][kx][c][n]));
                end
            golden[oy][ox][n]=sum;
          end
      // Independent arithmetic anchors: corners, seams, interior, all channel groups.
      if (golden[0][0][0] !== 2453)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=0 n=0");
      if (golden[0][0][7] !== -1038)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=0 n=7");
      if (golden[0][0][8] !== 5610)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=0 n=8");
      if (golden[0][0][15] !== 7629)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=0 n=15");
      if (golden[0][0][31] !== 2163)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=0 n=31");
      if (golden[0][0][32] !== -3045)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=0 n=32");
      if (golden[0][0][55] !== -3167)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=0 n=55");
      if (golden[0][0][63] !== -2442)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=0 n=63");
      if (golden[0][55][0] !== -1390)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=55 n=0");
      if (golden[0][55][7] !== -3519)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=55 n=7");
      if (golden[0][55][8] !== -1877)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=55 n=8");
      if (golden[0][55][15] !== -5564)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=55 n=15");
      if (golden[0][55][31] !== 1879)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=55 n=31");
      if (golden[0][55][32] !== 1716)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=55 n=32");
      if (golden[0][55][55] !== 551)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=55 n=55");
      if (golden[0][55][63] !== 2059)
        $fatal(1,"%m FAIL: independent golden anchor y=0 x=55 n=63");
      if (golden[55][0][0] !== -3882)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=0 n=0");
      if (golden[55][0][7] !== -6168)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=0 n=7");
      if (golden[55][0][8] !== 2924)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=0 n=8");
      if (golden[55][0][15] !== -2497)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=0 n=15");
      if (golden[55][0][31] !== -1425)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=0 n=31");
      if (golden[55][0][32] !== 6736)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=0 n=32");
      if (golden[55][0][55] !== 3128)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=0 n=55");
      if (golden[55][0][63] !== 2714)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=0 n=63");
      if (golden[55][55][0] !== 2329)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=55 n=0");
      if (golden[55][55][7] !== 6074)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=55 n=7");
      if (golden[55][55][8] !== -5019)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=55 n=8");
      if (golden[55][55][15] !== -1445)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=55 n=15");
      if (golden[55][55][31] !== -3088)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=55 n=31");
      if (golden[55][55][32] !== -3921)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=55 n=32");
      if (golden[55][55][55] !== 765)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=55 n=55");
      if (golden[55][55][63] !== 1074)
        $fatal(1,"%m FAIL: independent golden anchor y=55 x=55 n=63");
      if (golden[7][7][0] !== 345)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=7 n=0");
      if (golden[7][7][7] !== 1616)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=7 n=7");
      if (golden[7][7][8] !== 359)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=7 n=8");
      if (golden[7][7][15] !== 3530)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=7 n=15");
      if (golden[7][7][31] !== -3947)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=7 n=31");
      if (golden[7][7][32] !== -3456)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=7 n=32");
      if (golden[7][7][55] !== -2195)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=7 n=55");
      if (golden[7][7][63] !== 1049)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=7 n=63");
      if (golden[7][8][0] !== 3168)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=8 n=0");
      if (golden[7][8][7] !== 3135)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=8 n=7");
      if (golden[7][8][8] !== 2126)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=8 n=8");
      if (golden[7][8][15] !== 1371)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=8 n=15");
      if (golden[7][8][31] !== 598)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=8 n=31");
      if (golden[7][8][32] !== -867)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=8 n=32");
      if (golden[7][8][55] !== -1863)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=8 n=55");
      if (golden[7][8][63] !== -1879)
        $fatal(1,"%m FAIL: independent golden anchor y=7 x=8 n=63");
      if (golden[8][7][0] !== 1452)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=7 n=0");
      if (golden[8][7][7] !== 904)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=7 n=7");
      if (golden[8][7][8] !== 215)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=7 n=8");
      if (golden[8][7][15] !== -1169)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=7 n=15");
      if (golden[8][7][31] !== 2038)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=7 n=31");
      if (golden[8][7][32] !== 1007)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=7 n=32");
      if (golden[8][7][55] !== -1578)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=7 n=55");
      if (golden[8][7][63] !== 738)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=7 n=63");
      if (golden[8][8][0] !== -2837)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=8 n=0");
      if (golden[8][8][7] !== -1514)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=8 n=7");
      if (golden[8][8][8] !== -812)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=8 n=8");
      if (golden[8][8][15] !== -2947)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=8 n=15");
      if (golden[8][8][31] !== 1122)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=8 n=31");
      if (golden[8][8][32] !== 1691)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=8 n=32");
      if (golden[8][8][55] !== 1421)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=8 n=55");
      if (golden[8][8][63] !== 2382)
        $fatal(1,"%m FAIL: independent golden anchor y=8 x=8 n=63");
      if (golden[27][28][0] !== -1290)
        $fatal(1,"%m FAIL: independent golden anchor y=27 x=28 n=0");
      if (golden[27][28][7] !== -4474)
        $fatal(1,"%m FAIL: independent golden anchor y=27 x=28 n=7");
      if (golden[27][28][8] !== 2074)
        $fatal(1,"%m FAIL: independent golden anchor y=27 x=28 n=8");
      if (golden[27][28][15] !== -407)
        $fatal(1,"%m FAIL: independent golden anchor y=27 x=28 n=15");
      if (golden[27][28][31] !== 3680)
        $fatal(1,"%m FAIL: independent golden anchor y=27 x=28 n=31");
      if (golden[27][28][32] !== 1108)
        $fatal(1,"%m FAIL: independent golden anchor y=27 x=28 n=32");
      if (golden[27][28][55] !== 2258)
        $fatal(1,"%m FAIL: independent golden anchor y=27 x=28 n=55");
      if (golden[27][28][63] !== -2016)
        $fatal(1,"%m FAIL: independent golden anchor y=27 x=28 n=63");
      if (golden[48][48][0] !== 947)
        $fatal(1,"%m FAIL: independent golden anchor y=48 x=48 n=0");
      if (golden[48][48][7] !== 4223)
        $fatal(1,"%m FAIL: independent golden anchor y=48 x=48 n=7");
      if (golden[48][48][8] !== 2259)
        $fatal(1,"%m FAIL: independent golden anchor y=48 x=48 n=8");
      if (golden[48][48][15] !== 2514)
        $fatal(1,"%m FAIL: independent golden anchor y=48 x=48 n=15");
      if (golden[48][48][31] !== -334)
        $fatal(1,"%m FAIL: independent golden anchor y=48 x=48 n=31");
      if (golden[48][48][32] !== -2868)
        $fatal(1,"%m FAIL: independent golden anchor y=48 x=48 n=32");
      if (golden[48][48][55] !== -4815)
        $fatal(1,"%m FAIL: independent golden anchor y=48 x=48 n=55");
      if (golden[48][48][63] !== -2591)
        $fatal(1,"%m FAIL: independent golden anchor y=48 x=48 n=63");
      $display("INFO: 200704 global Golden results prepared; 80 arithmetic anchors correct");
    end
  endtask

  task automatic prepare_tile;
    integer gy, gx;
    begin
      padded_words=0;
      for (int iy=0; iy<HIN; iy++)
        for (int ix=0; ix<WIN; ix++)
          for (int c=0; c<CIN; c++) begin
            gy=OY0-1+iy;
            gx=OX0-1+ix;
            if (gy < 0 || gy >= GLOBAL_H || gx < 0 || gx >= GLOBAL_W) begin
              fmap[iy][ix][c]='0;
              padded_words=padded_words+1;
            end else fmap[iy][ix][c]=global_fmap[gy][gx][c];
          end
      if (padded_words !== EXPECTED_PAD_WORDS)
        $fatal(1,"%m FAIL: tile=%0d padded preload=%0d expected=%0d",
               CASE_ID,padded_words,EXPECTED_PAD_WORDS);
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
      total_padded_loads=0; total_padded_feeds=0;
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
        gy=OY0+iy-1; gx=OX0+ix-1;
        if (gy < 0 || gy >= GLOBAL_H || gx < 0 || gx >= GLOBAL_W) begin
          expected_value=0;
          total_padded_loads=total_padded_loads+1;
        end else expected_value=int'($signed(global_fmap[gy][gx][ci]));
        if (int'($signed(fmap_wr_data)) !== expected_value)
          $fatal(1,"%m FAIL: activation/halo preload tile=%0d addr=%0d",CASE_ID,idx);
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
        gy=OY0+int'(m_addr)/WOUT+int'(k_addr)/(KW*CIN)-1;
        gx=OX0+int'(m_addr)%WOUT+(int'(k_addr)/CIN)%KW-1;
        if (gy < 0 || gy >= GLOBAL_H || gx < 0 || gx >= GLOBAL_W)
          total_padded_feeds=total_padded_feeds+1;
      end
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
    integer prev_oy, prev_ox, prev_oc, prev_id;
    begin
      changed_outputs=0; expected_changed=0;
      prev_oy=0; prev_ox=0; prev_oc=0;
      if (CASE_ID > 0) begin
        prev_id=CASE_ID-1;
        prev_oy=(prev_id/(TILES_X*CHANNEL_GROUPS))*HOUT;
        prev_ox=((prev_id/CHANNEL_GROUPS)%TILES_X)*WOUT;
        prev_oc=(prev_id%CHANNEL_GROUPS)*N;
      end
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
          got_value=$signed(out_rd_data);
          if (got_value !== expected_value)
            $fatal(1,"%m FAIL: readback tile=%0d addr=%0d lane=%0d global=(%0d,%0d,%0d) got=%0d expected=%0d",
                   CASE_ID,a,lane,gy,gx,gn,got_value,expected_value);
          // Expected cache changes come from two GLOBAL Golden locations,
          // independently of previous DUT output / previous_output.
          if (CASE_ID > 0
              && expected_value !== golden[prev_oy+a/WOUT][prev_ox+a%WOUT][prev_oc+lane])
            expected_changed=expected_changed+1;
          if (CASE_ID > 0 && got_value !== previous_output[a][lane])
            changed_outputs=changed_outputs+1;
          previous_output[a][lane]=got_value;
          // Full-layer HWC pitch: 56 pixels per row, 64 channels per pixel.
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
      if (CASE_ID > 0 && expected_changed == 0)
        $fatal(1,"%m FAIL: test data did not distinguish successive cache generations");
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
      // Every full-layer position must have exactly one write and correct owner.
      for (int y=0; y<GLOBAL_H; y++)
        for (int x=0; x<GLOBAL_W; x++)
          for (int n=0; n<GLOBAL_N; n++) begin
            addr=(y*GLOBAL_W+x)*GLOBAL_N+n;
            if (y < REGION_H && x < REGION_W && n < REGION_N) begin
              expected_owner=((y/HOUT)*TILES_X+x/WOUT)*CHANNEL_GROUPS+n/N;
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
      if (compared !== TOTAL_OUTPUTS || checksum !== 64'sd7795)
        $fatal(1,"%m FAIL: full scatter count/checksum count=%0d checksum=%0d",compared,checksum);
    end
  endtask


  task automatic perf_count(input integer field);
    begin
      perf_tile[CASE_ID][field]=perf_tile[CASE_ID][field]+1;
      perf_sum[field]=perf_sum[field]+1;
    end
  endtask

  always @(posedge clk) begin : performance_monitor
    integer idx, iy, ix, gy, gx, ci, global_idx, stages, selected_phase;
    logic count_compute;
    if (!rst_n) begin
      perf_tick=0; perf_first_tick=0; perf_last_tick=0;
      perf_session_cycles=0; perf_host_gap=0;
      perf_computing=1'b0; perf_session_started=1'b0; perf_session_finished=1'b0;
      for (int t=0; t<NUM_TILES; t++) begin
        perf_tile_done[t]=1'b0;
        for (int f=0; f<P_FIELDS; f++) perf_tile[t][f]=0;
      end
      for (int f=0; f<P_COUNTS; f++) perf_sum[f]=0;
      for (int a=0; a<PERF_INPUT_WORDS; a++) perf_input_hits[a]=0;
      for (int a=0; a<PERF_WEIGHT_WORDS; a++) perf_weight_hits[a]=0;
    end else begin
      perf_tick=perf_tick+1;
      // Save the OLD transaction state; the acceptance edge itself is excluded.
      count_compute=perf_computing;
      if (start) begin
        if (perf_computing || perf_tile_done[CASE_ID])
          $fatal(1,"%m FAIL: performance start overlap/duplicate tile=%0d",CASE_ID);
        perf_tile[CASE_ID][P_START]=perf_tick;
        perf_computing=1'b1;
      end
      if (count_compute) begin
        perf_count(P_COMPUTE);
        stages=int'(clear_acc)+int'(feed_valid)+int'(perf_drain)
              +int'(perf_k_start)+int'(perf_k_done)+int'(write_valid);
        if ($isunknown({perf_array_enable,perf_drain,perf_k_start,perf_k_done})
            || stages > 1)
          $fatal(1,"%m FAIL: overlapping/unknown compute phases");
        if (clear_acc) selected_phase=P_CLEAR;
        else if (feed_valid) selected_phase=P_FEED;
        else if (perf_drain) selected_phase=P_DRAIN;
        else if (perf_k_start) selected_phase=P_LAUNCH;
        else if (perf_k_done) selected_phase=P_KDONE;
        else if (write_valid) selected_phase=P_WRITE;
        else selected_phase=P_OTHER;
        perf_count(selected_phase);
        if (perf_array_enable && !clear_acc) perf_count(P_ENABLED);
      end
      if (!perf_session_started && (fmap_wr_en || weight_wr_en)) begin
        perf_first_tick=perf_tick;
        perf_session_started=1'b1;
      end
      if (perf_session_started && !perf_session_finished) begin
        perf_session_cycles=perf_session_cycles+1;
        if (int'(fmap_wr_en)+int'(weight_wr_en)+int'(out_rd_en)+int'(count_compute) > 1)
          $fatal(1,"%m FAIL: performance phase overlap; serial baseline changed");
        if (!fmap_wr_en && !weight_wr_en && !out_rd_en && !count_compute)
          perf_host_gap=perf_host_gap+1;
      end
      if ((fmap_wr_en || weight_wr_en) && perf_tile[CASE_ID][P_FIRSTLOAD] == 0)
        perf_tile[CASE_ID][P_FIRSTLOAD]=perf_tick;

      if (fmap_wr_en) begin
        perf_count(P_ALOAD);
        idx=int'(fmap_wr_addr);
        iy=idx/(WIN*CIN); ix=(idx/CIN)%WIN; ci=idx%CIN;
        gy=OY0+iy-1; gx=OX0+ix-1;
        if (gy < 0 || gy >= GLOBAL_H || gx < 0 || gx >= GLOBAL_W)
          perf_count(P_PADLOAD);
        else begin
          global_idx=(gy*GLOBAL_W+gx)*CIN+ci;
          if (perf_input_hits[global_idx] == 0) perf_count(P_ANEW);
          else perf_count(P_AREPEAT);
          perf_input_hits[global_idx]=perf_input_hits[global_idx]+1;
        end
      end
      if (weight_wr_en) begin
        perf_count(P_WLOAD);
        // Unique packed word identity: global output group and K, not local addr.
        global_idx=(OC0/ARRAY_COLS)*WEIGHT_WORDS+int'(weight_wr_addr);
        if (perf_weight_hits[global_idx] == 0) perf_count(P_WNEW);
        else perf_count(P_WREPEAT);
        perf_weight_hits[global_idx]=perf_weight_hits[global_idx]+1;
      end
      if (out_rd_en) begin
        perf_count(P_RREQ);
        if (perf_tile[CASE_ID][P_FIRSTREAD] == 0)
          perf_tile[CASE_ID][P_FIRSTREAD]=perf_tick;
      end

      // Observe NBA-updated done/read-valid, separate from pre-edge phase counts.
      #1;
      if (perf_computing && done === 1'b1) begin
        perf_tile[CASE_ID][P_DONE]=perf_tick;
        perf_tile_done[CASE_ID]=1'b1;
        perf_computing=1'b0;
        if (perf_tile[CASE_ID][P_DONE]-perf_tile[CASE_ID][P_START]
            !== perf_tile[CASE_ID][P_COMPUTE])
          $fatal(1,"%m FAIL: performance start-to-done accounting mismatch");
      end
      if (out_rd_valid === 1'b1) begin
        if (out_rd_lane_valid !== 1'b1)
          $fatal(1,"%m FAIL: performance saw invalid output lane");
        perf_count(P_RRET);
        perf_tile[CASE_ID][P_LASTREAD]=perf_tick;
        if (perf_sum[P_RRET] == TOTAL_OUTPUTS) begin
          perf_last_tick=perf_tick;
          perf_session_finished=1'b1;
        end
      end
    end
  end

  task automatic perf_open_files;
    begin
      perf_rows=0;
      // Files are written by the user's simulation in its working directory.
      // A new simulation run replaces these two CSVs.
      perf_tile_fd=$fopen(perf_tile_path,"w");
      perf_summary_fd=$fopen(perf_summary_path,"w");
      if (perf_tile_fd == 0 || perf_summary_fd == 0)
        $fatal(1,"%m FAIL: cannot create performance CSV files");
      $fdisplay(perf_tile_fd,"tile,oy,ox,oc,start_cycle,done_cycle,compute_cycles,clear_cycles,feed_cycles,drain_cycles,launch_cycles,kdone_cycles,output_write_cycles,compute_other_cycles,array_enabled_nonclear_cycles,activation_load_cycles,weight_load_cycles,read_request_cycles,read_response_cycles,read_span_cycles,first_load_cycle,last_read_cycle,tile_window_cycles,tile_host_gap_cycles,activation_bytes,padding_load_bytes,activation_first_seen_bytes,activation_reload_bytes,weight_bytes,weight_first_seen_bytes,weight_reload_bytes,output_read_bytes,dense_mac_count,dense_compute_util_pct");
      $fdisplay(perf_summary_fd,"metric,value,unit");
      $display("PERF: CSV outputs: %s and %s (simulation working directory)",perf_tile_path,perf_summary_path);
    end
  endtask

  task automatic perf_report_tile;
    longint unsigned phases, window_cycles, gap_cycles, read_span;
    real utilization;
    begin
      phases=perf_tile[CASE_ID][P_CLEAR]+perf_tile[CASE_ID][P_FEED]
            +perf_tile[CASE_ID][P_DRAIN]+perf_tile[CASE_ID][P_LAUNCH]
            +perf_tile[CASE_ID][P_KDONE]+perf_tile[CASE_ID][P_WRITE]
            +perf_tile[CASE_ID][P_OTHER];
      if (!perf_tile_done[CASE_ID] || perf_computing || phases !== perf_tile[CASE_ID][P_COMPUTE]
          || perf_tile[CASE_ID][P_CLEAR] !== clears_seen
          || perf_tile[CASE_ID][P_FEED] !== feeds_seen
          || perf_tile[CASE_ID][P_WRITE] !== writes_seen
          || perf_tile[CASE_ID][P_ALOAD] !== FMAP_WORDS
          || perf_tile[CASE_ID][P_WLOAD] !== WEIGHT_WORDS
          || perf_tile[CASE_ID][P_RREQ] !== EXPECTED_OUTPUTS
          || perf_tile[CASE_ID][P_RRET] !== readbacks_checked
          || perf_tile[CASE_ID][P_PADLOAD] !== EXPECTED_PAD_WORDS
          || perf_rows !== CASE_ID)
        $fatal(1,"%m FAIL: performance counters disagree with functional scoreboard tile=%0d",CASE_ID);
      window_cycles=perf_tile[CASE_ID][P_LASTREAD]-perf_tile[CASE_ID][P_FIRSTLOAD]+1;
      read_span=perf_tile[CASE_ID][P_LASTREAD]-perf_tile[CASE_ID][P_FIRSTREAD]+1;
      if (window_cycles < phases+perf_tile[CASE_ID][P_ALOAD]
                           +perf_tile[CASE_ID][P_WLOAD]+perf_tile[CASE_ID][P_RREQ])
        $fatal(1,"%m FAIL: negative performance host gap");
      gap_cycles=window_cycles-phases-perf_tile[CASE_ID][P_ALOAD]
                 -perf_tile[CASE_ID][P_WLOAD]-perf_tile[CASE_ID][P_RREQ];
      utilization=100.0*real'(EXPECTED_OUTPUTS*K)/(real'(PERF_PE_COUNT)*real'(phases));
      $fdisplay(perf_tile_fd,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0.8f",
        CASE_ID,
        OY0,
        OX0,
        OC0,
        perf_tile[CASE_ID][P_START],
        perf_tile[CASE_ID][P_DONE],
        phases,
        perf_tile[CASE_ID][P_CLEAR],
        perf_tile[CASE_ID][P_FEED],
        perf_tile[CASE_ID][P_DRAIN],
        perf_tile[CASE_ID][P_LAUNCH],
        perf_tile[CASE_ID][P_KDONE],
        perf_tile[CASE_ID][P_WRITE],
        perf_tile[CASE_ID][P_OTHER],
        perf_tile[CASE_ID][P_ENABLED],
        perf_tile[CASE_ID][P_ALOAD],
        perf_tile[CASE_ID][P_WLOAD],
        perf_tile[CASE_ID][P_RREQ],
        perf_tile[CASE_ID][P_RRET],
        read_span,
        perf_tile[CASE_ID][P_FIRSTLOAD],
        perf_tile[CASE_ID][P_LASTREAD],
        window_cycles,
        gap_cycles,
        perf_tile[CASE_ID][P_ALOAD]*(ACT_WIDTH/8),
        perf_tile[CASE_ID][P_PADLOAD]*(ACT_WIDTH/8),
        perf_tile[CASE_ID][P_ANEW]*(ACT_WIDTH/8),
        perf_tile[CASE_ID][P_AREPEAT]*(ACT_WIDTH/8),
        perf_tile[CASE_ID][P_WLOAD]*(WGT_WIDTH*ARRAY_COLS/8),
        perf_tile[CASE_ID][P_WNEW]*(WGT_WIDTH*ARRAY_COLS/8),
        perf_tile[CASE_ID][P_WREPEAT]*(WGT_WIDTH*ARRAY_COLS/8),
        perf_tile[CASE_ID][P_RRET]*(ACC_WIDTH/8),
        EXPECTED_OUTPUTS*K,
        utilization);
      $fflush(perf_tile_fd);
      $display("PERF TILE %0d: start_to_done=%0d clear=%0d feed=%0d drain=%0d launch=%0d kdone=%0d write=%0d other=%0d read=%0d PE_dense_util=%0.4f%%",
               CASE_ID,phases,perf_tile[CASE_ID][P_CLEAR],perf_tile[CASE_ID][P_FEED],
               perf_tile[CASE_ID][P_DRAIN],perf_tile[CASE_ID][P_LAUNCH],
               perf_tile[CASE_ID][P_KDONE],perf_tile[CASE_ID][P_WRITE],
               perf_tile[CASE_ID][P_OTHER],read_span,utilization);
      perf_rows=perf_rows+1;
    end
  endtask

  task automatic perf_metric_i(input string name, input longint unsigned value, input string unit_name);
    $fdisplay(perf_summary_fd,"%s,%0d,%s",name,value,unit_name);
  endtask
  task automatic perf_metric_r(input string name, input real value, input string unit_name);
    $fdisplay(perf_summary_fd,"%s,%0.8f,%s",name,value,unit_name);
  endtask

  task automatic perf_report_summary;
    longint unsigned compute_cycles, elapsed_cycles, total_phases, dense_from_feeds;
    longint unsigned act_bytes, weight_bytes, output_bytes, unique_input, unique_weight;
    longint unsigned actual_real_input, spatial_pass_input, input_n_repeats, input_halo_repeats;
    longint unsigned tile_window_sum, total_read_span, min_compute, max_compute;
    longint unsigned hist8, hist16, hist32;
    real compute_util, e2e_util, feed_util, array_enable_util;
    begin
      compute_cycles=perf_sum[P_COMPUTE];
      elapsed_cycles=perf_session_cycles;
      total_phases=perf_sum[P_CLEAR]+perf_sum[P_FEED]+perf_sum[P_DRAIN]
                   +perf_sum[P_LAUNCH]+perf_sum[P_KDONE]+perf_sum[P_WRITE]+perf_sum[P_OTHER];
      dense_from_feeds=perf_sum[P_FEED]*ARRAY_COLS;
      if (!perf_session_finished || perf_computing || perf_rows !== NUM_TILES
          || elapsed_cycles !== perf_last_tick-perf_first_tick+1
          || total_phases !== compute_cycles || dense_from_feeds !== PERF_DENSE_MACS
          || perf_sum[P_FEED] !== total_feeds || perf_sum[P_WRITE] !== total_writes
          || perf_sum[P_CLEAR] !== total_clears || perf_sum[P_ALOAD] !== total_fmap_loads
          || perf_sum[P_WLOAD] !== total_weight_loads || perf_sum[P_RRET] !== total_readbacks
          || perf_sum[P_RREQ] !== total_readbacks || perf_sum[P_PADLOAD] !== total_padded_loads
          || elapsed_cycles !== compute_cycles+perf_sum[P_ALOAD]+perf_sum[P_WLOAD]
                                +perf_sum[P_RREQ]+perf_host_gap)
        $fatal(1,"%m FAIL: full-layer performance accounting mismatch");
      if (perf_sum[P_ALOAD] !== perf_sum[P_PADLOAD]+perf_sum[P_ANEW]+perf_sum[P_AREPEAT]
          || perf_sum[P_WLOAD] !== perf_sum[P_WNEW]+perf_sum[P_WREPEAT])
        $fatal(1,"%m FAIL: memory reuse accounting mismatch");
      hist8=0; hist16=0; hist32=0;
      for (int a=0; a<PERF_INPUT_WORDS; a++) begin
        case (perf_input_hits[a])
          8: hist8=hist8+1;
          16: hist16=hist16+1;
          32: hist32=hist32+1;
          default: $fatal(1,"%m FAIL: input identity load coverage addr=%0d hits=%0d",a,perf_input_hits[a]);
        endcase
      end
      for (int a=0; a<PERF_WEIGHT_WORDS; a++)
        if (perf_weight_hits[a] !== TILES_Y*TILES_X)
          $fatal(1,"%m FAIL: weight identity load coverage addr=%0d hits=%0d",a,perf_weight_hits[a]);

      act_bytes=perf_sum[P_ALOAD]*(ACT_WIDTH/8);
      weight_bytes=perf_sum[P_WLOAD]*(WGT_WIDTH*ARRAY_COLS/8);
      output_bytes=perf_sum[P_RRET]*(ACC_WIDTH/8);
      unique_input=perf_sum[P_ANEW]*(ACT_WIDTH/8);
      unique_weight=perf_sum[P_WNEW]*(WGT_WIDTH*ARRAY_COLS/8);
      actual_real_input=(perf_sum[P_ALOAD]-perf_sum[P_PADLOAD])*(ACT_WIDTH/8);
      spatial_pass_input=actual_real_input/CHANNEL_GROUPS;
      input_n_repeats=actual_real_input-spatial_pass_input;
      input_halo_repeats=spatial_pass_input-unique_input;
      if (unique_input !== PERF_INPUT_WORDS || unique_weight !== KH*KW*CIN*GLOBAL_N
          || actual_real_input%CHANNEL_GROUPS != 0
          || input_n_repeats+input_halo_repeats !== perf_sum[P_AREPEAT]*(ACT_WIDTH/8))
        $fatal(1,"%m FAIL: derived reuse decomposition mismatch");
      tile_window_sum=0; total_read_span=0; min_compute=perf_tile[0][P_COMPUTE]; max_compute=0;
      for (int t=0; t<NUM_TILES; t++) begin
        tile_window_sum=tile_window_sum+perf_tile[t][P_LASTREAD]-perf_tile[t][P_FIRSTLOAD]+1;
        total_read_span=total_read_span+perf_tile[t][P_LASTREAD]-perf_tile[t][P_FIRSTREAD]+1;
        if (perf_tile[t][P_COMPUTE] < min_compute) min_compute=perf_tile[t][P_COMPUTE];
        if (perf_tile[t][P_COMPUTE] > max_compute) max_compute=perf_tile[t][P_COMPUTE];
      end
      if (elapsed_cycles < tile_window_sum)
        $fatal(1,"%m FAIL: overlapping Tile end-to-end windows");

      // Dense layer MACs include padded positions; excludes arithmetic zero
      // bubbles used to transport data through the physical array.
      // Non-padding MACs exclude only structural border padding, not zeros
      // in the synthetic tensors. These are logical utilization estimates,
      // not measured DSP toggle activity, synthesis utilization or board Fmax.
      feed_util=100.0*real'(PERF_DENSE_MACS)/(real'(PERF_PE_COUNT)*real'(perf_sum[P_FEED]));
      compute_util=100.0*real'(PERF_DENSE_MACS)/(real'(PERF_PE_COUNT)*real'(compute_cycles));
      e2e_util=100.0*real'(PERF_DENSE_MACS)/(real'(PERF_PE_COUNT)*real'(elapsed_cycles));
      array_enable_util=100.0*real'(PERF_DENSE_MACS)/(real'(PERF_PE_COUNT)*real'(perf_sum[P_ENABLED]));
      perf_metric_i("tiles",NUM_TILES,"tiles");
      perf_metric_i("start_count",total_starts,"events");
      perf_metric_i("done_count",total_dones,"events");
      perf_metric_i("pe_count",PERF_PE_COUNT,"PE");
      perf_metric_i("dense_mac_count",PERF_DENSE_MACS,"MAC");
      perf_metric_i("nonpadding_mac_count",PERF_DENSE_MACS-total_padded_feeds*ARRAY_COLS,"MAC");
      perf_metric_i("padding_mac_count",total_padded_feeds*ARRAY_COLS,"MAC");
      perf_metric_i("sum_start_to_done_cycles",compute_cycles,"cycles");
      perf_metric_i("tile_compute_min_cycles",min_compute,"cycles");
      perf_metric_i("tile_compute_max_cycles",max_compute,"cycles");
      perf_metric_i("clear_cycles",perf_sum[P_CLEAR],"cycles");
      perf_metric_i("feed_cycles",perf_sum[P_FEED],"cycles");
      perf_metric_i("drain_cycles",perf_sum[P_DRAIN],"cycles");
      perf_metric_i("launch_cycles",perf_sum[P_LAUNCH],"cycles");
      perf_metric_i("kdone_cycles",perf_sum[P_KDONE],"cycles");
      perf_metric_i("output_bram_write_cycles",perf_sum[P_WRITE],"cycles");
      perf_metric_i("compute_other_cycles",perf_sum[P_OTHER],"cycles");
      perf_metric_i("array_enabled_nonclear_cycles",perf_sum[P_ENABLED],"cycles");
      perf_metric_i("activation_preload_cycles",perf_sum[P_ALOAD],"cycles");
      perf_metric_i("weight_preload_cycles",perf_sum[P_WLOAD],"cycles");
      perf_metric_i("output_read_request_cycles",perf_sum[P_RREQ],"cycles");
      perf_metric_i("output_read_response_cycles",perf_sum[P_RRET],"cycles");
      perf_metric_i("sum_tile_read_span_cycles",total_read_span,"cycles");
      perf_metric_i("e2e_first_preload_cycle",perf_first_tick,"cycle_index");
      perf_metric_i("e2e_last_read_cycle",perf_last_tick,"cycle_index");
      perf_metric_i("e2e_cycles",elapsed_cycles,"cycles");
      perf_metric_i("host_gap_cycles",perf_host_gap,"cycles");
      perf_metric_i("sum_tile_windows_cycles",tile_window_sum,"cycles");
      perf_metric_i("between_tile_gap_cycles",elapsed_cycles-tile_window_sum,"cycles");
      perf_metric_i("activation_preload_bytes",act_bytes,"bytes");
      perf_metric_i("padding_preload_bytes",perf_sum[P_PADLOAD]*(ACT_WIDTH/8),"bytes");
      perf_metric_i("real_activation_preload_bytes",actual_real_input,"bytes");
      perf_metric_i("unique_input_bytes",unique_input,"bytes");
      perf_metric_i("repeated_real_input_bytes",actual_real_input-unique_input,"bytes");
      perf_metric_i("derived_single_channel_group_real_input_bytes",spatial_pass_input,"bytes");
      perf_metric_i("derived_input_N_group_repeat_bytes",input_n_repeats,"bytes");
      perf_metric_i("derived_input_halo_repeat_bytes",input_halo_repeats,"bytes");
      perf_metric_i("activation_block_loads",NUM_TILES,"loads");
      perf_metric_i("distinct_spatial_input_blocks",TILES_Y*TILES_X,"blocks");
      perf_metric_i("repeated_activation_block_loads",NUM_TILES-TILES_Y*TILES_X,"loads");
      perf_metric_i("input_identities_loaded_8_times",hist8,"bytes");
      perf_metric_i("input_identities_loaded_16_times",hist16,"bytes");
      perf_metric_i("input_identities_loaded_32_times",hist32,"bytes");
      perf_metric_i("weight_preload_bytes",weight_bytes,"bytes");
      perf_metric_i("unique_weight_bytes",unique_weight,"bytes");
      perf_metric_i("repeated_weight_bytes",weight_bytes-unique_weight,"bytes");
      perf_metric_i("weight_block_loads",NUM_TILES,"loads");
      perf_metric_i("distinct_weight_blocks",CHANNEL_GROUPS,"blocks");
      perf_metric_i("repeated_weight_block_loads",NUM_TILES-CHANNEL_GROUPS,"loads");
      perf_metric_i("weight_loads_per_identity",TILES_Y*TILES_X,"loads");
      perf_metric_i("output_read_bytes",output_bytes,"bytes");
      perf_metric_i("output_bram_write_bytes",perf_sum[P_WRITE]*(ACC_WIDTH*ARRAY_COLS/8),"bytes");
      perf_metric_i("output_mask_write_bytes",perf_sum[P_WRITE]*(ARRAY_COLS/8),"bytes");
      perf_metric_i("host_payload_bytes",act_bytes+weight_bytes+output_bytes,"bytes");
      perf_metric_r("tb_clock_period_ns",PERF_CLOCK_NS,"ns");
      perf_metric_r("compute_latency_at_tb_clock_ms",real'(compute_cycles)*PERF_CLOCK_NS/1.0e6,"ms");
      perf_metric_r("e2e_latency_at_tb_clock_ms",real'(elapsed_cycles)*PERF_CLOCK_NS/1.0e6,"ms");
      perf_metric_r("dense_util_feed_pct",feed_util,"percent");
      perf_metric_r("dense_util_compute_pct",compute_util,"percent");
      perf_metric_r("dense_util_array_enabled_nonclear_pct",array_enable_util,"percent");
      perf_metric_r("dense_util_e2e_pct",e2e_util,"percent");
      perf_metric_r("nonpadding_util_compute_pct",100.0*real'(PERF_DENSE_MACS-total_padded_feeds*ARRAY_COLS)/(real'(PERF_PE_COUNT)*real'(compute_cycles)),"percent");
      perf_metric_r("dense_mac_per_compute_cycle",real'(PERF_DENSE_MACS)/real'(compute_cycles),"MAC_per_cycle");
      perf_metric_r("dense_mac_per_e2e_cycle",real'(PERF_DENSE_MACS)/real'(elapsed_cycles),"MAC_per_cycle");
      perf_metric_r("feed_share_compute_pct",100.0*real'(perf_sum[P_FEED])/real'(compute_cycles),"percent");
      perf_metric_r("drain_share_compute_pct",100.0*real'(perf_sum[P_DRAIN])/real'(compute_cycles),"percent");
      perf_metric_r("compute_share_e2e_pct",100.0*real'(compute_cycles)/real'(elapsed_cycles),"percent");
      perf_metric_r("preload_share_e2e_pct",100.0*real'(perf_sum[P_ALOAD]+perf_sum[P_WLOAD])/real'(elapsed_cycles),"percent");
      perf_metric_r("readback_share_e2e_pct",100.0*real'(perf_sum[P_RREQ])/real'(elapsed_cycles),"percent");
      perf_metric_r("host_gap_share_e2e_pct",100.0*real'(perf_host_gap)/real'(elapsed_cycles),"percent");
      perf_metric_r("real_input_load_amplification",real'(actual_real_input)/real'(unique_input),"ratio");
      perf_metric_r("weight_load_amplification",real'(weight_bytes)/real'(unique_weight),"ratio");
      $fdisplay(perf_summary_fd,"functional_status,PASS,text");
      $fdisplay(perf_summary_fd,"measurement_scope,simulation_only_not_board_or_DDR,text");
      $fdisplay(perf_summary_fd,"compute_window,start_accept_edge_exclusive_to_done_assert_edge_inclusive,text");
      $fdisplay(perf_summary_fd,"e2e_window,first_preload_edge_to_last_read_response_edge_inclusive,text");
      $fdisplay(perf_summary_fd,"utilization_basis,logical_dense_MACs_over_64_PE_cycle_capacity,text");
      $fclose(perf_tile_fd);
      $fclose(perf_summary_fd);
      $display("PERF SUMMARY: compute=%0d cycles (%0.6f ms at TB clock), first_load_to_last_read=%0d cycles (%0.6f ms)",
               compute_cycles,real'(compute_cycles)*PERF_CLOCK_NS/1.0e6,
               elapsed_cycles,real'(elapsed_cycles)*PERF_CLOCK_NS/1.0e6);
      $display("PERF UTIL: dense_MACs=%0d, FEED=%0.4f%%, compute=%0.4f%%, end_to_end=%0.4f%%",
               PERF_DENSE_MACS,feed_util,compute_util,e2e_util);
      $display("PERF TRAFFIC: activation=%0d B (padding=%0d B), weights=%0d B, readback=%0d B; existing BRAM ports, NOT DDR",
               act_bytes,perf_sum[P_PADLOAD]*(ACT_WIDTH/8),weight_bytes,output_bytes);
      $display("PERF REUSE: unique_input=%0d B, repeated_real_input=%0d B; unique_weights=%0d B, repeated_weights=%0d B",
               unique_input,actual_real_input-unique_input,unique_weight,weight_bytes-unique_weight);
      $display("PERF CSV READY: %s; %s",perf_tile_path,perf_summary_path);
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
    EXPECTED_PAD_WORDS=1216; EXPECTED_PAD_FEEDS=3008;
    tiles_completed=0; total_readbacks=0; changed_outputs_total=0;
    readbacks_checked=0;
    top_tiles=0; bottom_tiles=0; left_tiles=0; right_tiles=0;
    corner_tiles=0; edge_tiles=0; interior_tiles=0;
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
      $fatal(1,"%m FAIL: existing Tile DUT configuration mismatch");
    if (dut.u_baseline.u_core.u_compute.u_act_reader.u_mem.DEPTH != FMAP_WORDS
        || dut.u_baseline.u_core.u_compute.u_weight_reader.u_mem.DEPTH != WEIGHT_WORDS
        || dut.u_baseline.u_core.u_out_mem.DEPTH != EXPECTED_WRITES)
      $fatal(1,"%m FAIL: existing Tile memory layout mismatch");
    if (REGION_H != GLOBAL_H || REGION_W != GLOBAL_W || REGION_N != GLOBAL_N
        || REGION_H%HOUT != 0 || REGION_W%WOUT != 0 || REGION_N%N != 0
        || NUM_TILES != 392 || TOTAL_OUTPUTS != 200704)
      $fatal(1,"%m FAIL: incomplete full-layer Tile configuration");
    perf_open_files();
    prepare_global_golden();

    repeat (3) @(negedge clk);
    rst_n=1'b1;
    reset_released=1'b1;
    for (int tile_y=0; tile_y<TILES_Y; tile_y++)
      for (int tile_x=0; tile_x<TILES_X; tile_x++)
        for (int channel_group=0; channel_group<CHANNEL_GROUPS; channel_group++) begin
          @(negedge clk);
          if (busy !== 1'b0 || done !== 1'b0 || start !== 1'b0)
            $fatal(1,"%m FAIL: DUT not idle before next Tile");
          CASE_ID=(tile_y*TILES_X+tile_x)*CHANNEL_GROUPS+channel_group;
          OY0=tile_y*HOUT; OX0=tile_x*WOUT; OC0=channel_group*N;
          rearm_checks=1'b1;
          run_started=1'b0; completion_observed=1'b0;
          readbacks_checked=0;
          for (int a=0; a<EXPECTED_WRITES; a++)
            for (int lane=0; lane<ARRAY_COLS; lane++) lane_read_hits[a][lane]=0;
          if ((tile_y == 0 || tile_y == TILES_Y-1)
              && (tile_x == 0 || tile_x == TILES_X-1)) begin
            EXPECTED_PAD_WORDS=1216; EXPECTED_PAD_FEEDS=3008;
          end else if (tile_y == 0 || tile_y == TILES_Y-1
                       || tile_x == 0 || tile_x == TILES_X-1) begin
            EXPECTED_PAD_WORDS=640; EXPECTED_PAD_FEEDS=1536;
          end else begin
            EXPECTED_PAD_WORDS=0; EXPECTED_PAD_FEEDS=0;
          end
          if (tile_hits[CASE_ID] !== 0)
            $fatal(1,"%m FAIL: duplicate Tile scheduling");
          prepare_tile();
          load_tile();
          // Rearm only the TB scoreboard. rst_n stays high for all 392 runs.
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
          if (tile_y == 0) top_tiles=top_tiles+1;
          if (tile_y == TILES_Y-1) bottom_tiles=bottom_tiles+1;
          if (tile_x == 0) left_tiles=left_tiles+1;
          if (tile_x == TILES_X-1) right_tiles=right_tiles+1;
          if (EXPECTED_PAD_WORDS == 1216) corner_tiles=corner_tiles+1;
          else if (EXPECTED_PAD_WORDS == 640) edge_tiles=edge_tiles+1;
          else interior_tiles=interior_tiles+1;
          perf_report_tile();
          $display("TILE PASS: id=%0d origin=(%0d,%0d) channels=%0d..%0d, 36864 K beats, 64 writes, 512/512 outputs read and stitched, padded_feeds=%0d",
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
        || total_padded_loads !== 141312 || total_padded_feeds !== 342016
        || changed_outputs_total !== 200087
        || top_tiles !== 56 || bottom_tiles !== 56 || left_tiles !== 56 || right_tiles !== 56
        || corner_tiles !== 32 || edge_tiles !== 160 || interior_tiles !== 200
        || rst_n !== 1'b1 || busy !== 1'b0 || done !== 1'b0 || any_overflow !== 1'b0)
      $fatal(1,"%m FAIL: lifetime counts tiles=%0d starts=%0d done=%0d feeds=%0d writes=%0d reads=%0d",
             tiles_completed,total_starts,total_dones,total_feeds,total_writes,total_readbacks);
    for (int t=0; t<NUM_TILES; t++)
      if (tile_hits[t] !== 1) $fatal(1,"%m FAIL: Tile coverage id=%0d",t);
    for (int a=0; a<EXPECTED_WRITES; a++)
      if (output_address_hits[a] !== NUM_TILES)
        $fatal(1,"%m FAIL: reused output cache word=%0d generations=%0d",a,output_address_hits[a]);
    perf_report_summary();
    finished=1'b1;
    $display("INFO: one DUT, one reset, 392 starts/dones; 391 weight switches, 200087 changed cached results checked");
    $display("INFO: global HWC scatter pitch=56x64; 200704 positions written exactly once; all four borders checked");
    $display("PASS: layer1.0.conv1 full layer, 392 sequential Tiles, 14450688 K beats, 25088 writes, 200704/200704 INT32 outputs and global stitching correct");
    $finish;
  end

  initial begin : timeout
    repeat (TIMEOUT_CYCLES) @(posedge clk);
    if (finished !== 1'b1)
      $fatal(1,"%m FAIL: global TIMEOUT tile=%0d completed=%0d starts=%0d done=%0d feeds=%0d writes=%0d",
             CASE_ID,tiles_completed,total_starts,total_dones,total_feeds,total_writes);
  end
endmodule
