`timescale 1ns / 1ps

// Padding/stride system regression. Only this testbench file is new.
// Keep all existing RTL and earlier testbenches unchanged.
// All cases use a 5x5x8 input, 3x3 kernel, 16 output channels, two N-tiles.
// S1/P1: M=25, 3600 FEED beats (896 zero-padding), 50 writes, 400 outputs.
// S2/P0: M= 4,  576 FEED beats (  0 zero-padding),  8 writes,  64 outputs.
// S2/P1: M= 9, 1296 FEED beats (512 zero-padding), 18 writes, 144 outputs.
// Each case checks every A/B beat, request address, write, and lane readback.
// For padded CURRENT K: bram_data_valid=0, act_data=0, feed_valid stays 1.
// The NEXT K request valid is checked separately from the returned data.
// Three independent DUT instances run concurrently in simulation only;
// this does not alter the FPGA architecture or add any RTL top-level pins.
//
// Select tb_fpga_system_top_padding_stride as Simulation Top.
// Run until final suite PASS or $fatal. Global bound: 10000 clocks=100 us.
// This file has not been simulated by the authoring assistant.
module tb_fpga_system_top_padding_stride;
  logic clk=1'b0;
  always #5 clk=~clk;
  wire pass_s1p1, pass_s2p0, pass_s2p1;

  tb_fpga_padding_stride_case #(
    .STRIDE(1), .PADDING(1), .EXPECTED_PAD_FEEDS(896)
  ) case_s1p1 (.clk(clk), .passed(pass_s1p1));
  tb_fpga_padding_stride_case #(
    .STRIDE(2), .PADDING(0), .EXPECTED_PAD_FEEDS(0)
  ) case_s2p0 (.clk(clk), .passed(pass_s2p0));
  tb_fpga_padding_stride_case #(
    .STRIDE(2), .PADDING(1), .EXPECTED_PAD_FEEDS(512)
  ) case_s2p1 (.clk(clk), .passed(pass_s2p1));

  initial begin : finish_suite
    wait (pass_s1p1 === 1'b1 && pass_s2p0 === 1'b1 && pass_s2p1 === 1'b1);
    #1;
    $display("PASS: padding/stride suite, 3/3 cases, 5472 K beats, 76 writes, 608/608 outputs and readbacks correct");
    $finish;
  end

  initial begin : timeout
    repeat (10000) @(posedge clk);
    $fatal(1,"FAIL: TIMEOUT padding/stride suite; S1P1=%b S2P0=%b S2P1=%b",
           pass_s1p1,pass_s2p0,pass_s2p1);
  end
endmodule

// A complete independent DUT and scoreboard for one static configuration.
// Same negedge-drive / posedge-monitor / posedge+#1 readback timing as the
// previously user-verified M9/N16/K72 test. This helper never calls $finish.
module tb_fpga_padding_stride_case #(
  parameter int STRIDE=1,
  parameter int PADDING=1,
  parameter int EXPECTED_PAD_FEEDS=896
)(
  input logic clk,
  output logic passed
);
  import pkg_accelerator::*;

  localparam int HIN=5, WIN=5, CIN=8, KH=3, KW=3, COUT=16;
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

  // The earlier M9 wrapper fixes stride=1,padding=0. This case explicitly
  // configures the existing parameterized top; no RTL change is needed.
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
  integer reference_padded_feeds;
  integer padded_feeds_seen, real_feeds_seen;
  integer pad_to_real_transitions, real_to_pad_transitions;
  logic previous_real, have_previous_feed;

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
    integer sum, iy, ix;
    begin
      reference_padded_feeds=0;
      for (int y=0; y<HIN; y++)
        for (int x=0; x<WIN; x++)
          for (int c=0; c<CIN; c++) begin
            value=1+y*5+x+7*c;
            if (value < -128 || value > 127)
              $fatal(1,"FAIL (%m): activation outside signed INT8");
            fmap[y][x][c]=8'(value);
          end

      for (int ky=0; ky<KH; ky++)
        for (int kx=0; kx<KW; kx++)
          for (int c=0; c<CIN; c++)
            for (int n=0; n<N; n++) begin
              value=(n-7)+(ky-1)+(kx-1)+((c%3)-1);
              if (value < -128 || value > 127)
                $fatal(1,"FAIL (%m): weight outside signed INT8");
              kernel[ky][kx][c][n]=8'(value);
            end

      // Independent standard convolution: no DUT signals or address helper.
      for (int oy=0; oy<HOUT; oy++)
        for (int ox=0; ox<WOUT; ox++)
          for (int n=0; n<N; n++) begin
            sum=0;
            for (int ky=0; ky<KH; ky++)
              for (int kx=0; kx<KW; kx++)
                for (int c=0; c<CIN; c++) begin
                  iy=oy*STRIDE-PADDING+ky;
                  ix=ox*STRIDE-PADDING+kx;
                  // Standard zero padding: NEVER index fmap out of bounds.
                  if (iy >= 0 && iy < HIN && ix >= 0 && ix < WIN)
                    sum=sum
                      + int'($signed(fmap[iy][ix][c]))
                      * int'($signed(kernel[ky][kx][c][n]));
                  else if (n == 0)
                    reference_padded_feeds=reference_padded_feeds+NTILES;
                end
            golden[oy][ox][n]=sum;
          end

      // Arithmetic reference anchors computed independently of DUT logic.
      if (STRIDE == 1 && PADDING == 1) begin
        known_first='{-5496,-4584,-3672,-2760,-1848,-936,-24,888,1800,2712,3624,4536,5448,6360,7272,8184};
        known_middle='{-18855,-16155,-13455,-10755,-8055,-5355,-2655,45,2745,5445,8145,10845,13545,16245,18945,21645};
        known_last='{-12000,-10512,-9024,-7536,-6048,-4560,-3072,-1584,-96,1392,2880,4368,5856,7344,8832,10320};
      end
      else if (STRIDE == 2 && PADDING == 0) begin
        known_first='{-15777,-13509,-11241,-8973,-6705,-4437,-2169,99,2367,4635,6903,9171,11439,13707,15975,18243};
        known_middle='{-20907,-17919,-14931,-11943,-8955,-5967,-2979,9,2997,5985,8973,11961,14949,17937,20925,23913};
        known_last='{-21933,-18801,-15669,-12537,-9405,-6273,-3141,-9,3123,6255,9387,12519,15651,18783,21915,25047};
      end
      else if (STRIDE == 2 && PADDING == 1) begin
        known_first='{-5496,-4584,-3672,-2760,-1848,-936,-24,888,1800,2712,3624,4536,5448,6360,7272,8184};
        known_middle='{-18855,-16155,-13455,-10755,-8055,-5355,-2655,45,2745,5445,8145,10845,13545,16245,18945,21645};
        known_last='{-12000,-10512,-9024,-7536,-6048,-4560,-3072,-1584,-96,1392,2880,4368,5856,7344,8832,10320};
      end
      else
        $fatal(1,"FAIL (%m): unsupported test configuration");
      if (reference_padded_feeds !== EXPECTED_PAD_FEEDS)
        $fatal(1,"FAIL (%m): golden padding count=%0d expected=%0d",
               reference_padded_feeds,EXPECTED_PAD_FEEDS);
      for (int n=0; n<N; n++) begin
        if (golden[0][0][n] !== known_first[n])
          $fatal(1,"FAIL (%m): golden first N%0d got=%0d expected=%0d",
                 n,golden[0][0][n],known_first[n]);
        if (golden[(M/2)/WOUT][(M/2)%WOUT][n] !== known_middle[n])
          $fatal(1,"FAIL (%m): golden middle N%0d got=%0d expected=%0d",
                 n,golden[(M/2)/WOUT][(M/2)%WOUT][n],known_middle[n]);
        if (golden[HOUT-1][WOUT-1][n] !== known_last[n])
          $fatal(1,"FAIL (%m): golden last N%0d got=%0d expected=%0d",
                 n,golden[HOUT-1][WOUT-1][n],known_last[n]);
      end
      $display("INFO (%m): stride=%0d padding=%0d M=%0d; 48 golden anchors checked",
               STRIDE,PADDING,M);
    end
  endtask

  // Check the request present BEFORE the edge, separately from data returned
  // by the PREVIOUS edge. Called for CLEAR(K0) and every FEED lookahead.
  task automatic check_request(input integer em, input integer ent,
                               input integer ek);
    integer ky, kx, ci, iy, ix, expected_fmap, expected_weight;
    logic expected_real;
    begin
      ky=ek/(KW*CIN);
      kx=(ek/CIN)%KW;
      ci=ek%CIN;
      iy=(em/WOUT)*STRIDE-PADDING+ky;
      ix=(em%WOUT)*STRIDE-PADDING+kx;
      expected_real=(iy >= 0 && iy < HIN && ix >= 0 && ix < WIN);
      // Current addr-gen contract returns a harmless address zero for padding.
      expected_fmap=expected_real ? (iy*WIN+ix)*CIN+ci : 0;
      expected_weight=ent*K+ek;
      if (act_rd_en !== 1'b1 || wgt_rd_en !== 1'b1)
        $fatal(1,"FAIL (%m): missing BRAM request M=%0d tile=%0d fetchK=%0d",em,ent,ek);
      if (fetch_k_addr !== ADDR_WIDTH'(ek))
        $fatal(1,"FAIL (%m): fetchK=%0d expected=%0d M=%0d tile=%0d",
               fetch_k_addr,ek,em,ent);
      if (addr_valid !== expected_real || fmap_addr !== ADDR_WIDTH'(expected_fmap))
        $fatal(1,"FAIL (%m): activation request M=%0d fetchK=%0d addr=%0d expected=%0d",
               em,ek,fmap_addr,expected_fmap);
      if (weight_word_addr !== ADDR_WIDTH'(expected_weight))
        $fatal(1,"FAIL (%m): weight request tile=%0d fetchK=%0d addr=%0d expected=%0d",
               ent,ek,weight_word_addr,expected_weight);
      requests_by_tile[ent]=requests_by_tile[ent]+1;
      weight_request_hits[expected_weight]=weight_request_hits[expected_weight]+1;
    end
  endtask

  // Transaction checks use TB-owned expected indices, not DUT indices to
  // select a convenient golden value. No #delay here: check what RTL consumes.
  always @(posedge clk) begin : monitor
    integer em, ent, ek, ky, kx, ci, iy, ix;
    logic expected_real;
    integer expected_value, got_value;
    if (!rst_n) begin
      writes_seen=0;
      clears_seen=0;
      feeds_seen=0;
      done_seen=0;
      write_outputs_checked=0;
      padded_feeds_seen=0;
      real_feeds_seen=0;
      pad_to_real_transitions=0;
      real_to_pad_transitions=0;
      previous_real=1'b0;
      have_previous_feed=1'b0;
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
        $fatal(1,"FAIL (%m): unknown control/status after reset");
      if (any_overflow !== 1'b0)
        $fatal(1,"FAIL (%m): accumulator overflow");
      if ((clear_acc || feed_valid || write_valid || done) && !run_started)
        $fatal(1,"FAIL (%m): transaction before start");
      if (completion_observed && (clear_acc || feed_valid || write_valid))
        $fatal(1,"FAIL (%m): extra compute/write transaction after done");
      if ((clear_acc || feed_valid || write_valid) && busy !== 1'b1)
        $fatal(1,"FAIL (%m): active transaction while busy is not asserted");

      em=writes_seen/NTILES;
      ent=writes_seen%NTILES;

      if (clear_acc) begin
        if (feed_valid || write_valid || round_open || writes_seen >= EXPECTED_WRITES)
          $fatal(1,"FAIL (%m): invalid/overlapping CLEAR round=%0d",writes_seen);
        if (clears_seen !== writes_seen || feeds_seen !== writes_seen*K)
          $fatal(1,"FAIL (%m): CLEAR round/count sequence");
        if (m_addr !== ADDR_WIDTH'(em) || n_base !== ADDR_WIDTH'(ent*ARRAY_COLS)
            || k_addr !== ADDR_WIDTH'(0))
          $fatal(1,"FAIL (%m): CLEAR coordinates M=%0d Nbase=%0d K=%0d expected M=%0d tile=%0d",
                 m_addr,n_base,k_addr,em,ent);
        check_request(em,ent,0);
        round_open=1'b1;
        round_feeds=0;
        have_previous_feed=1'b0;
        clears_seen=clears_seen+1;
      end

      if (feed_valid) begin
        if (!round_open || write_valid || round_feeds >= K || feeds_seen >= EXPECTED_FEEDS)
          $fatal(1,"FAIL (%m): unexpected/excess FEED round=%0d Kcount=%0d",writes_seen,round_feeds);
        ek=round_feeds;
        if (m_addr !== ADDR_WIDTH'(em) || n_base !== ADDR_WIDTH'(ent*ARRAY_COLS)
            || k_addr !== ADDR_WIDTH'(ek))
          $fatal(1,"FAIL (%m): FEED sequence got M=%0d Nbase=%0d K=%0d expected M=%0d tile=%0d K=%0d",
                 m_addr,n_base,k_addr,em,ent,ek);
        if (wgt_data_valid !== 1'b1)
          $fatal(1,"FAIL (%m): missing weight data M=%0d tile=%0d K=%0d",em,ent,ek);
        ky=ek/(KW*CIN);
        kx=(ek/CIN)%KW;
        ci=ek%CIN;
        iy=(em/WOUT)*STRIDE-PADDING+ky;
        ix=(em%WOUT)*STRIDE-PADDING+kx;
        expected_real=(iy >= 0 && iy < HIN && ix >= 0 && ix < WIN);
        if (bram_data_valid !== expected_real)
          $fatal(1,"FAIL (%m): CURRENT K activation valid M=%0d tile=%0d K=%0d valid=%b expected=%b",
                 em,ent,ek,bram_data_valid,expected_real);
        expected_value=0;
        if (expected_real) begin
          expected_value=int'($signed(fmap[iy][ix][ci]));
          real_feeds_seen=real_feeds_seen+1;
        end else begin
          // Padding is a zero-valued FEED beat, not a stalled/missing beat.
          padded_feeds_seen=padded_feeds_seen+1;
        end
        if (have_previous_feed) begin
          if (!previous_real && expected_real)
            pad_to_real_transitions=pad_to_real_transitions+1;
          if (previous_real && !expected_real)
            real_to_pad_transitions=real_to_pad_transitions+1;
        end
        previous_real=expected_real;
        have_previous_feed=1'b1;
        got_value=int'($signed(act_data));
        if (got_value !== expected_value)
          $fatal(1,"FAIL (%m): activation M=%0d tile=%0d K=%0d got=%0d expected=%0d",
                 em,ent,ek,got_value,expected_value);
        for (int lane=0; lane<ARRAY_COLS; lane++) begin
          expected_value=int'($signed(kernel[ky][kx][ci][ent*ARRAY_COLS+lane]));
          got_value=int'($signed(wgt_row_packed[lane*WGT_WIDTH +: WGT_WIDTH]));
          if (got_value !== expected_value)
            $fatal(1,"FAIL (%m): weight alignment M=%0d tile=%0d K=%0d lane=%0d got=%0d expected=%0d",
                   em,ent,ek,lane,got_value,expected_value);
        end
        check_request(em,ent,(ek < K-1) ? ek+1 : ek);
        round_feeds=round_feeds+1;
        feeds_seen=feeds_seen+1;
        feeds_by_tile[ent]=feeds_by_tile[ent]+1;
      end else if (round_open && round_feeds < K) begin
        // CLEAR itself is allowed; the following 72 FEED beats are continuous.
        if (!clear_acc)
          $fatal(1,"FAIL (%m): FEED gap before all 72 beats M=%0d tile=%0d count=%0d",
                 em,ent,round_feeds);
      end

      if (write_valid) begin
        if (!round_open || round_feeds !== K || writes_seen >= EXPECTED_WRITES)
          $fatal(1,"FAIL (%m): early/extra write addr=%0d Kcount=%0d",write_addr,round_feeds);
        if (write_addr !== ADDR_WIDTH'(writes_seen)
            || write_m_addr !== ADDR_WIDTH'(em)
            || write_n_base !== ADDR_WIDTH'(ent*ARRAY_COLS))
          $fatal(1,"FAIL (%m): write sequence addr=%0d M=%0d Nbase=%0d expected addr=%0d M=%0d tile=%0d",
                 write_addr,write_m_addr,write_n_base,writes_seen,em,ent);
        if (write_mask !== {ARRAY_COLS{1'b1}})
          $fatal(1,"FAIL (%m): write mask=%h expected=FF",write_mask);
        if (word_write_hits[writes_seen] !== 0)
          $fatal(1,"FAIL (%m): duplicate output word %0d",writes_seen);
        for (int lane=0; lane<ARRAY_COLS; lane++) begin
          expected_value=golden[em/WOUT][em%WOUT][ent*ARRAY_COLS+lane];
          got_value=$signed(write_data[lane*ACC_WIDTH +: ACC_WIDTH]);
          if (got_value !== expected_value)
            $fatal(1,"FAIL (%m): output write M=%0d N=%0d got=%0d expected=%0d",
                   em,ent*ARRAY_COLS+lane,got_value,expected_value);
          write_outputs_checked=write_outputs_checked+1;
        end
        $display("INFO (%m): WRITE addr=%0d M=%0d N=%0d..%0d mask=%h checked",
                 writes_seen,em,ent*ARRAY_COLS,ent*ARRAY_COLS+ARRAY_COLS-1,write_mask);
        word_write_hits[writes_seen]=word_write_hits[writes_seen]+1;
        writes_by_tile[ent]=writes_by_tile[ent]+1;
        writes_seen=writes_seen+1;
        round_open=1'b0;
      end

      if (done) begin
        if (busy !== 1'b0 || round_open || writes_seen !== EXPECTED_WRITES
            || feeds_seen !== EXPECTED_FEEDS || done_seen !== 0)
          $fatal(1,"FAIL (%m): premature/repeated done writes=%0d feeds=%0d",writes_seen,feeds_seen);
        done_seen=done_seen+1;
      end
    end
  end

  task automatic check_compute_counts;
    integer expected_hits;
    begin
      if (padded_feeds_seen !== EXPECTED_PAD_FEEDS
          || padded_feeds_seen !== reference_padded_feeds
          || real_feeds_seen !== EXPECTED_FEEDS-EXPECTED_PAD_FEEDS)
        $fatal(1,"FAIL (%m): padding/real FEED totals padded=%0d real=%0d expected padded=%0d",
               padded_feeds_seen,real_feeds_seen,EXPECTED_PAD_FEEDS);
      if (PADDING > 0 &&
          (pad_to_real_transitions == 0 || real_to_pad_transitions == 0))
        $fatal(1,"FAIL (%m): padding/real transitions not both covered");
      if (writes_seen !== EXPECTED_WRITES || clears_seen !== EXPECTED_WRITES
          || feeds_seen !== EXPECTED_FEEDS || write_outputs_checked !== EXPECTED_OUTPUTS
          || round_open !== 1'b0)
        $fatal(1,"FAIL (%m): compute totals clear=%0d feed=%0d write=%0d outputs=%0d",
               clears_seen,feeds_seen,writes_seen,write_outputs_checked);
      for (int t=0; t<NTILES; t++) begin
        if (feeds_by_tile[t] !== M*K || writes_by_tile[t] !== M
            || requests_by_tile[t] !== M*(K+1))
          $fatal(1,"FAIL (%m): tile %0d coverage feeds=%0d writes=%0d requests=%0d",
                 t,feeds_by_tile[t],writes_by_tile[t],requests_by_tile[t]);
        for (int k=0; k<K; k++) begin
          // CLEAR requests K0; FEED requests K1..K71; final FEED repeats K71.
          expected_hits=(k == K-1) ? 2*M : M;
          if (weight_request_hits[t*K+k] !== expected_hits)
            $fatal(1,"FAIL (%m): weight address %0d requests=%0d expected=%0d",
                   t*K+k,weight_request_hits[t*K+k],expected_hits);
        end
      end
      for (int a=0; a<EXPECTED_WRITES; a++)
        if (word_write_hits[a] !== 1)
          $fatal(1,"FAIL (%m): output word %0d write count=%0d",a,word_write_hits[a]);
    end
  endtask

  initial begin : drive
    integer ky, kx, ci, em, ent;
    integer expected_value, got_value;
    passed=1'b0;
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
      $fatal(1,"FAIL (%m): this regression requires 8x8 PE and INT8/INT32 widths");
    if (dut.H_IN != HIN || dut.W_IN != WIN
        || dut.C_IN != CIN || dut.C_OUT != N
        || dut.KH != KH || dut.KW != KW
        || dut.STRIDE != STRIDE || dut.PADDING != PADDING)
      $fatal(1,"FAIL (%m): DUT convolution configuration differs from test case");
    if (dut.u_core.u_compute.M_LEN != M
        || dut.u_core.u_compute.N_LEN != N
        || dut.u_core.u_compute.K_LEN != K
        || dut.u_core.u_compute.N_TILES != NTILES)
      $fatal(1,"FAIL (%m): DUT derived dimensions differ from test configuration");
    if (dut.u_core.u_compute.u_act_reader.u_mem.DEPTH < FMAP_WORDS
        || dut.u_core.u_compute.u_weight_reader.u_mem.DEPTH < WEIGHT_WORDS
        || dut.u_core.u_out_mem.DEPTH < EXPECTED_WRITES)
      $fatal(1,"FAIL (%m): insufficient DUT BRAM depth");
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
    $display("INFO (%m): started S=%0d P=%0d M=%0d N=%0d K=%0d; expecting %0d rounds, %0d FEED beats",
             STRIDE,PADDING,M,N,K,EXPECTED_WRITES,EXPECTED_FEEDS);

    wait (done === 1'b1);
    #1;
    completion_observed=1'b1;
    check_compute_counts();
    if (busy !== 1'b0 || any_overflow !== 1'b0)
      $fatal(1,"FAIL (%m): bad completion status");

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
          $fatal(1,"FAIL (%m): missing read valid addr=%0d lane=%0d",a,lane);
        expected_value=golden[em/WOUT][em%WOUT][ent*ARRAY_COLS+lane];
        got_value=$signed(out_rd_data);
        if (got_value !== expected_value)
          $fatal(1,"FAIL (%m): BRAM read addr=%0d lane=%0d M=%0d N=%0d got=%0d expected=%0d",
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
      $fatal(1,"FAIL (%m): output read-valid did not deassert");

    // Continue observing briefly to reject extra writes or repeated done.
    repeat (3) @(negedge clk);
    check_compute_counts();
    if (done_seen !== 1 || busy !== 1'b0 || done !== 1'b0 || any_overflow !== 1'b0)
      $fatal(1,"FAIL (%m): final status/done count done_seen=%0d",done_seen);
    if (readbacks_checked !== EXPECTED_OUTPUTS)
      $fatal(1,"FAIL (%m): readback count=%0d expected=%0d",readbacks_checked,EXPECTED_OUTPUTS);
    for (int a=0; a<EXPECTED_WRITES; a++)
      for (int lane=0; lane<ARRAY_COLS; lane++)
        if (lane_read_hits[a][lane] !== 1)
          $fatal(1,"FAIL (%m): missing/duplicate read addr=%0d lane=%0d",a,lane);

    $display("INFO (%m): both N-tiles and weight addresses 0..143 checked; padding-to-real=%0d real-to-padding=%0d",
             pad_to_real_transitions,real_to_pad_transitions);
    $display("CASE PASS: stride=%0d padding=%0d M=%0d N=%0d K=%0d, %0d K beats (%0d padded), %0d writes, %0d/%0d outputs and readbacks correct",
             STRIDE,PADDING,M,N,K,feeds_seen,padded_feeds_seen,writes_seen,
             readbacks_checked,EXPECTED_OUTPUTS);
    passed=1'b1;
  end

endmodule
