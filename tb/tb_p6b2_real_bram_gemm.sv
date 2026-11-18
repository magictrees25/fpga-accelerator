`timescale 1ns / 1ps

// P6-B2-5: REAL Activation/Weight BRAM Readers -> automatic K scheduler -> PE
// -> REAL Output BRAM. One DUT, one reset, two complete preload/start/read jobs.
// Local convolution: 9x9x64 input, 3x3/S2/P0 -> 4x4x8, M16 N8 K576.
// K_TILE_SIZE=64 creates 9 internal segments for each M; only FINAL sums write.
// No Reader response model, forced internals, manual K starts or hidden offsets.
// Final Golden is an independent six-loop convolution over source arrays.
module tb_p6b2_real_bram_gemm;
  localparam int TILE_SIZE = 64;
  logic clk = 1'b0;
  logic finished;
  always #5 clk = ~clk;
  logic rst_n, start;
  logic [15:0] m_len, n_len, k_len;
  logic [15:0] m_addr, n_base, k_base, fetch_k_base;
  logic pair_req_en, mem_clear_acc, mem_feed_valid;
  logic [15:0] act_pair_packed;
  logic [127:0] wgt_pair_packed;
  logic [1:0] act_rsp_valid, wgt_rsp_valid;
  logic write_valid, busy, done, any_overflow;
  logic [15:0] write_addr, write_m_addr, write_n_base;
  logic [7:0] write_mask;
  logic [255:0] write_data;
  logic [15:0] rsp_m, rsp_n, rsp_k;

  bit active, segment_inflight, job_inflight;
  integer run_id, expected_m_count, expected_k_count, segment_count, segment_cap;
  integer writes, requests, feeds, drains, pipe_clears, acc_clears;
  integer k_starts, k_dones, host_dones, checked_outputs;
  integer expected_segment, segment_requests, segment_feeds, segment_drains;
  integer segment_clears, segment_firsts;
  integer total_requests, total_feeds, total_drains, total_writes;
  integer total_k_starts, total_k_dones, total_outputs, total_host_dones;
  logic [255:0] last_snapshot;

  integer signed golden [0:15][0:7];
  integer signed partial [0:15][0:8][0:7];
  integer signed pe_partial [0:15][0:8][0:7][0:7];
  integer signed running [0:7];
  integer signed running_pe [0:7][0:7];
  logic [255:0] captured [0:15];
  bit seen [0:15];

  logic signed [7:0] fmap [0:8][0:8][0:63];
  logic signed [7:0] kernel [0:2][0:2][0:63][0:7];
  logic act_wr_en, weight_wr_en;
  logic [15:0] act_wr_addr, weight_wr_addr;
  logic [7:0] act_wr_data;
  logic [63:0] weight_wr_data;
  logic fmap_rd_en, fmap_rd_valid;
  logic [15:0] fmap_rd_addr, fmap_rd_data;
  logic out_rd_en, out_rd_valid;
  logic [15:0] out_rd_addr;
  logic [255:0] out_rd_data;
  logic [7:0] out_rd_mask;
  logic prior_req;
  integer act_loads, weight_loads, act_responses, weight_responses, readbacks;
  integer total_act_loads, total_weight_loads, total_readbacks;
  integer total_act_responses, total_weight_responses;
  integer expected_changed, changed_readbacks;
  integer signed previous_result [0:15][0:7];

  gemm_mn_controller_mem_p2 #(
    .ADDR_WIDTH(16), .K_TILE_SIZE(TILE_SIZE)
  ) dut (
    .clk(clk), .rst_n(rst_n), .start(start),
    .m_len(m_len), .n_len(n_len), .k_len(k_len),
    .m_addr(m_addr), .n_base(n_base), .k_base(k_base), .fetch_k_base(fetch_k_base),
    .pair_req_en(pair_req_en), .act_pair_packed(act_pair_packed),
    .act_rsp_valid(act_rsp_valid), .wgt_pair_packed(wgt_pair_packed),
    .wgt_rsp_valid(wgt_rsp_valid), .mem_clear_acc(mem_clear_acc),
    .mem_feed_valid(mem_feed_valid), .write_valid(write_valid),
    .write_addr(write_addr), .write_m_addr(write_m_addr),
    .write_n_base(write_n_base), .write_mask(write_mask), .write_data(write_data),
    .busy(busy), .done(done), .any_overflow(any_overflow)
  );

  // These helpers are used only for beat/partial-sum checks. The FINAL
  // convolution Golden below indexes y/x/ky/kx/c directly, without these maps.
  function automatic integer signed aval(input integer job, input integer m, input integer k);
    aval = int'($signed(fmap[(m/4)*2 + k/192][(m%4)*2 + (k/64)%3][k%64]));
  endfunction
  function automatic integer signed bval(input integer job, input integer k, input integer n);
    bval = int'($signed(kernel[k/192][(k/64)%3][k%64][n]));
  endfunction
  function automatic integer signed feature_pattern(
    input integer job, input integer y, input integer x, input integer c
  );
    integer v;
    begin
      v=((17*y+11*x+7*c+37+53*job)%255)-127;
      if(y==0 && x==0 && c==0) v=(job==0)?-128:127;
      if(y==8 && x==8 && c==63) v=(job==0)?127:-128;
      feature_pattern=v;
    end
  endfunction
  function automatic integer signed weight_pattern(
    input integer job, input integer ky, input integer kx,
    input integer c, input integer n
  );
    integer v;
    begin
      v=((13*ky+7*kx+3*c+5*n+41+13*job)%31)-15;
      if(ky==0 && kx==0 && c==0 && n==0) v=(job==0)?-128:127;
      if(ky==2 && kx==2 && c==63 && n==7) v=(job==0)?127:-128;
      weight_pattern=v;
    end
  endfunction

  function automatic integer seg_length(input integer seg);
    integer remaining;
    begin
      remaining = expected_k_count - seg*segment_cap;
      seg_length = (remaining > segment_cap) ? segment_cap : remaining;
    end
  endfunction

  conv_bram_reader_p4_3x3s2 #(
    .ADDR_WIDTH(16), .EXTERNAL_ACT_MEM(1'b1)
  ) dut_act_reader (
    .clk(clk), .rst_n(rst_n), .m_addr(m_addr),
    .pair_req_en(pair_req_en), .fetch_k_base(fetch_k_base),
    .wr_en(1'b0), .wr_addr(16'd0), .wr_data(8'd0),
    .fmap_rd_data(fmap_rd_data), .fmap_rd_en(fmap_rd_en), .fmap_rd_addr(fmap_rd_addr),
    .act_pair_packed(act_pair_packed), .act_rsp_valid(act_rsp_valid)
  );

  conv_activation_bram_shared_p5 #(
    .ADDR_WIDTH(16), .MAX_BYTES(6400)
  ) dut_act_bram (
    .clk(clk), .rst_n(rst_n), .wr_en(act_wr_en),
    .wr_addr(act_wr_addr), .wr_data(act_wr_data),
    .rd_en(fmap_rd_en), .rd_addr(fmap_rd_addr),
    .rd_data(fmap_rd_data), .rd_valid(fmap_rd_valid)
  );

  conv_weight_bram_reader_p5_shared #(
    .ADDR_WIDTH(16), .K_MAX(576)
  ) dut_weight_reader (
    .clk(clk), .rst_n(rst_n), .n_base(n_base),
    // The Reader owns the FULL memory span, never the inner segment length.
    .k_len(16'd576), .pair_req_en(pair_req_en), .fetch_k_base(fetch_k_base),
    .wr_en(weight_wr_en), .wr_addr(weight_wr_addr), .wr_data(weight_wr_data),
    .wgt_pair_packed(wgt_pair_packed), .wgt_rsp_valid(wgt_rsp_valid)
  );

  output_bram #(
    .ADDR_WIDTH(16), .DATA_WIDTH(32), .LANES(8), .DEPTH(16)
  ) dut_output (
    .clk(clk), .rst_n(rst_n), .wr_en(write_valid), .wr_addr(write_addr),
    .wr_data(write_data), .wr_mask(write_mask),
    .rd_en(out_rd_en), .rd_addr(out_rd_addr), .rd_data(out_rd_data),
    .rd_mask(out_rd_mask), .rd_valid(out_rd_valid)
  );

  // Observation tags only: no assignment to Reader response/data signals.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      prior_req<=0; rsp_m<=0; rsp_n<=0; rsp_k<=0;
    end else begin
      prior_req<=pair_req_en;
      if(pair_req_en) begin
        rsp_m<=m_addr; rsp_n<=n_base; rsp_k<=fetch_k_base;
      end
    end
  end

  // Check actual host write strobes, banking and one-cycle response timing.
  always @(posedge clk) begin : memory_monitor
    integer y,x,c,k,ky,kx;
    logic [63:0] expected_weight_word;
    if(rst_n) begin
      if(act_rsp_valid !== {2{prior_req}} || wgt_rsp_valid !== {2{prior_req}} ||
         fmap_rd_valid !== prior_req || fmap_rd_en !== pair_req_en ||
         dut_weight_reader.bank_rd_en !== {2{pair_req_en}})
        $fatal(1,"FAIL: real Reader/BRAM enable or one-cycle response-valid mismatch");
      if(act_rsp_valid==2'b11) begin act_responses++; total_act_responses++; end
      if(wgt_rsp_valid==2'b11) begin weight_responses++; total_weight_responses++; end
      if((act_wr_en || weight_wr_en) && (busy || pair_req_en || write_valid || done || out_rd_en))
        $fatal(1,"FAIL: host preload overlaps computation/readback");
      if(act_wr_en) begin
        y=act_loads/(9*64); x=(act_loads/64)%9; c=act_loads%64;
        if(act_loads>=5184 || act_wr_addr !== 16'(act_loads) ||
           act_wr_data !== fmap[y][x][c] ||
           dut_act_bram.bank_wr_addr !== 16'(act_loads/2) ||
           dut_act_bram.bank_wr_en !== ((act_loads%2)==0 ? 2'b01 : 2'b10))
          $fatal(1,"FAIL: activation preload sequence/data/bank");
        act_loads++; total_act_loads++;
      end
      if(weight_wr_en) begin
        k=weight_loads; ky=k/192; kx=(k/64)%3; c=k%64;
        for(int n=0;n<8;n++) expected_weight_word[n*8 +: 8]=kernel[ky][kx][c][n];
        if(weight_loads>=576 || weight_wr_addr !== 16'(weight_loads) ||
           weight_wr_data !== expected_weight_word ||
           dut_weight_reader.bank_wr_addr !== 16'(weight_loads/2) ||
           dut_weight_reader.bank_wr_en !== ((weight_loads%2)==0 ? 2'b01 : 2'b10))
          $fatal(1,"FAIL: weight preload sequence/data/bank");
        weight_loads++; total_weight_loads++;
      end
      if(out_rd_en && (busy || write_valid || int'(out_rd_addr)>=16))
        $fatal(1,"FAIL: Output BRAM read/write overlap or read address");
      if(write_valid && int'(write_addr)>=16)
        $fatal(1,"FAIL: Output BRAM write address exceeds depth");
    end
  end

  task automatic preload_memories;
    integer addr,k;
    begin
      for(int y=0;y<9;y++)
        for(int x=0;x<9;x++)
          for(int c=0;c<64;c++) begin
            addr=(y*9+x)*64+c;
            @(negedge clk);
            act_wr_en=1; act_wr_addr=16'(addr); act_wr_data=fmap[y][x][c];
          end
      @(negedge clk); act_wr_en=0;
      for(int ky=0;ky<3;ky++)
        for(int kx=0;kx<3;kx++)
          for(int c=0;c<64;c++) begin
            k=(ky*3+kx)*64+c;
            @(negedge clk);
            weight_wr_en=1; weight_wr_addr=16'(k);
            for(int n=0;n<8;n++) weight_wr_data[n*8 +: 8]=kernel[ky][kx][c][n];
          end
      @(negedge clk); weight_wr_en=0;
      if(act_loads!=5184 || weight_loads!=576)
        $fatal(1,"FAIL: actual host preload strobe counts");
    end
  endtask

  task automatic read_output_memory;
    integer m;
    integer signed got;
    begin
      // Permuted read order distinguishes address-dependent BRAM data from
      // a stale final snapshot. Each of the 16 words is requested once.
      for(int index=0;index<16;index++) begin
        m=(index*5+3)%16;
        @(negedge clk); out_rd_en=1; out_rd_addr=16'(m);
        @(posedge clk); #1;
        if(out_rd_valid !== 1'b1 || out_rd_mask !== 8'hff ||
           out_rd_data !== captured[m])
          $fatal(1,"FAIL: synchronous Output BRAM word/mask at M=%0d",m);
        for(int n=0;n<8;n++) begin
          got=int'($signed(out_rd_data[n*32 +: 32]));
          if(got !== golden[m][n])
            $fatal(1,"FAIL: Output BRAM M=%0d N=%0d got=%0d expected=%0d",m,n,got,golden[m][n]);
          if(run_id>0 && golden[m][n] != previous_result[m][n]) begin
            if(got === previous_result[m][n]) $fatal(1,"FAIL: stale output from previous job");
            changed_readbacks++;
          end
          previous_result[m][n]=got;
          readbacks++; total_readbacks++;
        end
        @(negedge clk); out_rd_en=0;
        @(posedge clk); #1;
        if(out_rd_valid !== 1'b0) $fatal(1,"FAIL: Output BRAM valid did not deassert");
      end
      if(readbacks!=128 || changed_readbacks!=expected_changed)
        $fatal(1,"FAIL: readback/changed-output coverage counts");
    end
  endtask

  // All protocol checks sample pre-NBA signals, as the synchronous DUT does.
  always @(posedge clk) begin : monitor
    integer base, length, local_k, global_k, ky, kx, ci, iy, ix, want_bank_addr;
    logic [63:0] want_act;
    logic [127:0] want_weights;
    if (rst_n) begin
      if ($isunknown({busy, done, write_valid, pair_req_en, mem_clear_acc,
                      mem_feed_valid, dut.k_start, dut.k_done, dut.clear_pipe}) ||
          any_overflow !== 1'b0)
        $fatal(1, "FAIL TILE_SIZE=%0d: unknown control or overflow", TILE_SIZE);
      if (!active) begin
        if (busy || done || write_valid || pair_req_en || dut.k_start || dut.k_done)
          $fatal(1, "FAIL: activity outside an externally launched GEMM job");
      end else begin
        base = expected_segment * segment_cap;
        length = seg_length(expected_segment);
        if (job_inflight && !done && busy !== 1'b1)
          $fatal(1, "FAIL: busy dropped between K segments or M positions");
        if (dut.valid_reset !== dut.clear_pipe || mem_clear_acc !== dut.clear_acc)
          $fatal(1, "FAIL: clear/valid observation wiring");

        if (dut.k_start) begin
          if (segment_inflight || expected_segment >= segment_count ||
              writes >= expected_m_count || dut.k_busy || dut.k_done || write_valid ||
              m_addr !== 16'(writes) || n_base !== 16'd0 ||
              dut.u_kctrl.k_len !== 16'(length) ||
              dut.u_kctrl.k_tile_base !== 16'(base) ||
              dut.u_kctrl.clear_acc_on_start !== (expected_segment == 0))
            $fatal(1, "FAIL TILE_SIZE=%0d job=%0d M=%0d seg=%0d: K start/base/length/clear",
                   TILE_SIZE, run_id, writes, expected_segment);
          segment_inflight = 1'b1;
          job_inflight = 1'b1;
          k_starts++; total_k_starts++;
        end

        if (dut.clear_pipe) begin
          if (!segment_inflight || segment_clears != 0 || segment_requests != 0 ||
              segment_feeds != 0 || segment_drains != 0 ||
              mem_clear_acc !== (expected_segment == 0) ||
              !dut.array_enable || !pair_req_en || mem_feed_valid ||
              k_base !== 16'd0 || dut.act_in_packed !== 64'd0 ||
              dut.wgt_pair_in_packed !== 128'd0)
            $fatal(1, "FAIL: segment CLEAR ordering or partial-sum clear decision");
          segment_clears++; pipe_clears++;
        end
        if (mem_clear_acc) begin
          if (!dut.clear_pipe) $fatal(1, "FAIL: accumulator clear outside pipeline clear");
          acc_clears++;
        end

        if (pair_req_en) begin
          if (!segment_inflight || !(dut.clear_pipe || mem_feed_valid) ||
              segment_requests >= length/2 || m_addr !== 16'(writes) ||
              n_base !== 16'd0 || fetch_k_base !== 16'(base + 2*segment_requests))
            $fatal(1, "FAIL TILE_SIZE=%0d job=%0d M=%0d seg=%0d req=%0d gotK=%0d",
                   TILE_SIZE, run_id, writes, expected_segment, segment_requests, fetch_k_base);
          global_k=base+2*segment_requests;
          ky=global_k/192; kx=(global_k/64)%3; ci=global_k%64;
          iy=(writes/4)*2+ky; ix=(writes%4)*2+kx;
          want_bank_addr=((iy*9+ix)*64+ci)/2;
          if(fmap_rd_addr !== 16'(want_bank_addr) ||
             dut_weight_reader.bank_rd_addr[0] !== 16'(global_k/2) ||
             dut_weight_reader.bank_rd_addr[1] !== 16'(global_k/2))
            $fatal(1,"FAIL: real activation/weight BRAM bank read address M=%0d K=%0d",writes,global_k);
          segment_requests++; requests++; total_requests++;
        end

        if (mem_feed_valid) begin
          local_k = 2*segment_feeds;
          global_k = base + local_k;
          if (!segment_inflight || segment_feeds >= length/2 ||
              segment_clears != 1 || !dut.array_enable || dut.clear_pipe ||
              k_base !== 16'(local_k) || m_addr !== 16'(writes) || n_base !== 16'd0 ||
              rsp_k !== 16'(global_k) || rsp_m !== 16'(writes) || rsp_n !== 16'd0 ||
              dut_act_reader.rsp_k_base !== 16'(global_k) ||
              dut_act_reader.rsp_m_addr !== 16'(writes) || fmap_rd_valid !== 1'b1 ||
              act_rsp_valid !== 2'b11 || wgt_rsp_valid !== 2'b11)
            $fatal(1, "FAIL: local K / global response tag / FEED ordering");
          if (pair_req_en !== (local_k + 2 < length) ||
              dut.data_valid_in !== (local_k == 0))
            $fatal(1, "FAIL: lookahead request or first-valid schedule");
          want_act = '0;
          want_act[(local_k%8)*8 +: 8] = 8'(aval(run_id,writes,global_k));
          want_act[((local_k+1)%8)*8 +: 8] = 8'(aval(run_id,writes,global_k+1));
          for (int n = 0; n < 8; n++) begin
            want_weights[n*8 +: 8] = 8'(bval(run_id,global_k,n));
            want_weights[(8+n)*8 +: 8] = 8'(bval(run_id,global_k+1,n));
          end
          if (act_pair_packed !== {8'(aval(run_id,writes,global_k+1)),
                                   8'(aval(run_id,writes,global_k))} ||
              wgt_pair_packed !== want_weights || dut.act_in_packed !== want_act ||
              dut.wgt_pair_in_packed !== want_weights)
            $fatal(1, "FAIL: signed paired data or PE row mapping");
          if (dut.data_valid_in) segment_firsts++;
          segment_feeds++; feeds++; total_feeds++;
        end

        if (dut.array_enable && !mem_feed_valid && !dut.clear_pipe) begin
          if (!segment_inflight || segment_feeds != length/2 || pair_req_en ||
              dut.data_valid_in || dut.act_in_packed !== 64'd0 ||
              dut.wgt_pair_in_packed !== 128'd0 || k_base !== 16'(length-2))
            $fatal(1, "FAIL: DRAIN protocol");
          segment_drains++; drains++; total_drains++;
        end

        if (dut.k_done) begin
          if (!segment_inflight || segment_requests != length/2 ||
              segment_feeds != length/2 || segment_drains != 13 ||
              segment_clears != 1 || segment_firsts != 1 || write_valid)
            $fatal(1, "FAIL: K done counts or early write");
          for (int n = 0; n < 8; n++) begin
            if ($signed(dut.k_result_packed[n*32 +: 32]) !== partial[writes][expected_segment][n] ||
                $signed(dut.acc_out_packed[n*32 +: 32]) !== partial[writes][expected_segment][n])
              $fatal(1, "FAIL TILE_SIZE=%0d job=%0d M=%0d seg=%0d N=%0d: cumulative snapshot",
                     TILE_SIZE, run_id, writes, expected_segment, n);
            for (int r = 0; r < 8; r++)
              if (dut.u_array.pe_acc[r][n] !== pe_partial[writes][expected_segment][r][n])
                $fatal(1, "FAIL: intermediate PE[%0d][%0d] partial sum", r,n);
          end
          last_snapshot = dut.k_result_packed;
          k_dones++; total_k_dones++;
          expected_segment++;
          segment_requests = 0; segment_feeds = 0; segment_drains = 0;
          segment_clears = 0; segment_firsts = 0;
          segment_inflight = 1'b0;
        end

        if (write_valid) begin
          if (segment_inflight || expected_segment != segment_count ||
              writes >= expected_m_count || k_dones != (writes+1)*segment_count ||
              write_addr !== 16'(writes) || write_m_addr !== 16'(writes) ||
              write_n_base !== 16'd0 || write_mask !== 8'hff ||
              write_data !== last_snapshot || !busy || pair_req_en || mem_feed_valid)
            $fatal(1, "FAIL TILE_SIZE=%0d job=%0d: premature write or write metadata", TILE_SIZE, run_id);
          if (seen[writes]) $fatal(1, "FAIL: duplicate output address");
          seen[writes] = 1'b1;
          captured[writes] = write_data;
          for (int n = 0; n < 8; n++) begin
            if ($signed(write_data[n*32 +: 32]) !== golden[writes][n])
              $fatal(1, "FAIL TILE_SIZE=%0d job=%0d M=%0d N=%0d got=%0d expected=%0d",
                     TILE_SIZE,run_id,writes,n,$signed(write_data[n*32 +: 32]),golden[writes][n]);
            checked_outputs++; total_outputs++;
          end
          writes++; total_writes++;
          expected_segment = 0;
        end

        if (done) begin
          if (!job_inflight || writes != expected_m_count || busy || write_valid ||
              dut.k_start || dut.k_done || pair_req_en)
            $fatal(1, "FAIL: job done before all final outputs, or extended done pulse");
          host_dones++; total_host_dones++;
          job_inflight = 1'b0;
        end
      end
    end
  end

  // Every segment clears the pipelines, but only a new M output clears sums.
  // All 64 accumulator values are checked immediately after that clear edge.
  for (genvar r = 0; r < 8; r++) begin : check_row
    for (genvar n = 0; n < 8; n++) begin : check_col
      always @(posedge clk) begin : check_clear
        integer signed expected_before;
        if (rst_n && active && dut.clear_pipe) begin
          expected_before = (expected_segment == 0) ? 0 :
                            pe_partial[writes][expected_segment-1][r][n];
          #1;
          if (dut.u_array.pe_acc[r][n] !== expected_before ||
              dut.u_array.gen_row[r].gen_col[n].u_pe.act_reg !== 8'sd0 ||
              dut.u_array.gen_row[r].gen_col[n].u_pe.wgt_reg !== 8'sd0 ||
              dut.u_array.gen_row[r].gen_col[n].u_pe.mul_reg !== 16'sd0)
            $fatal(1, "FAIL: PE clear/preservation at segment boundary r=%0d n=%0d",r,n);
        end
      end
    end
  end

  task automatic run_job(input integer id, input integer m_count, input integer k_count);
    integer waited, gk, row_idx, m_index;
    integer signed a, w;
    begin
      @(negedge clk);
      if (busy !== 1'b0 || done !== 1'b0)
        $fatal(1, "FAIL: previous GEMM job did not become idle");
      run_id = id; expected_m_count = m_count; expected_k_count = k_count;
      segment_cap = (TILE_SIZE == 0) ? k_count : TILE_SIZE;
      segment_count = (k_count + segment_cap - 1) / segment_cap;
      if (m_count > 16 || segment_count > 9)
        $fatal(1, "FAIL: scenario exceeds TB reference-array bounds");
      writes=0; requests=0; feeds=0; drains=0; pipe_clears=0; acc_clears=0;
      k_starts=0; k_dones=0; host_dones=0; checked_outputs=0;
      expected_segment=0; segment_requests=0; segment_feeds=0; segment_drains=0;
      segment_clears=0; segment_firsts=0; last_snapshot='0;
      segment_inflight=0; job_inflight=0;
      act_loads=0; weight_loads=0; act_responses=0; weight_responses=0; readbacks=0;
      expected_changed=0; changed_readbacks=0;
      if(m_count!=16 || k_count!=576) $fatal(1,"FAIL: real Reader configuration requires M16 K576");
      for(int y=0;y<9;y++)
        for(int x=0;x<9;x++)
          for(int c=0;c<64;c++) fmap[y][x][c]=8'(feature_pattern(id,y,x,c));
      for(int ky=0;ky<3;ky++)
        for(int kx=0;kx<3;kx++)
          for(int c=0;c<64;c++)
            for(int n=0;n<8;n++) kernel[ky][kx][c][n]=8'(weight_pattern(id,ky,kx,c,n));
      for (int m=0; m<16; m++) begin seen[m]=0; captured[m]='x; end

      // Independent FINAL Golden: STANDARD 3x3 convolution with stride=2,
      // local padding=0. Does not use DUT addresses, K decoding, aval or bval.
      for(int oy=0;oy<4;oy++)
        for(int ox=0;ox<4;ox++)
          for(int n=0;n<8;n++) begin
            m_index=oy*4+ox;
            golden[m_index][n]=0;
            for(int ky=0;ky<3;ky++)
              for(int kx=0;kx<3;kx++)
                for(int c=0;c<64;c++) begin
                  a=int'($signed(fmap[oy*2+ky][ox*2+kx][c]));
                  w=int'($signed(kernel[ky][kx][c][n]));
                  golden[m_index][n]+=a*w;
                end
            if(id>0 && golden[m_index][n]!=previous_result[m_index][n]) expected_changed++;
          end
      if(id>0 && expected_changed!=128)
        $fatal(1,"FAIL: overwrite stimulus must change all 128 results");
      // Additional boundary references; no DUT counter, address or result used.
      for (int m=0; m<m_count; m++) begin
        for (int n=0; n<8; n++) begin
          running[n]=0;
          for (int r=0; r<8; r++) running_pe[r][n]=0;
        end
        for (int seg=0; seg<segment_count; seg++) begin
          for (int local_k=0; local_k<seg_length(seg); local_k++) begin
            gk=seg*segment_cap+local_k;
            row_idx=local_k%8;
            a=aval(id,m,gk);
            for (int n=0; n<8; n++) begin
              w=bval(id,gk,n);
              running[n] += a*w;
              running_pe[row_idx][n] += a*w;
            end
          end
          for (int n=0; n<8; n++) begin
            partial[m][seg][n]=running[n];
            for (int r=0; r<8; r++) pe_partial[m][seg][r][n]=running_pe[r][n];
          end
        end
        for (int n=0; n<8; n++)
          if (running[n] !== golden[m][n]) $fatal(1, "FAIL: internal Golden consistency");
      end

      preload_memories();
      // The stores above are external writes only, not direct BRAM assignments.
      @(negedge clk);
      m_len=16'(m_count); n_len=16'd8; k_len=16'(k_count);
      active=1; start=1;
      @(negedge clk);
      start=0;
      // Total M/N/K must be saved at the external start, not re-read for a
      // later K segment or M position. Invalid live values expose such reuse.
      m_len=0; n_len=0; k_len=0;
      waited=0;
      while (done !== 1'b1 && waited < 20000) begin
        @(posedge clk); #2; waited++;
      end
      if (done !== 1'b1)
        $fatal(1, "FAIL TILE_SIZE=%0d job=%0d: GEMM done timeout",TILE_SIZE,id);
      // Keep monitoring through the full done pulse and return to IDLE.
      repeat (3) @(negedge clk);
      if (writes != m_count || checked_outputs != m_count*8 ||
          requests != m_count*k_count/2 || feeds != requests ||
          k_starts != m_count*segment_count || k_dones != k_starts ||
          pipe_clears != k_starts || acc_clears != m_count || drains != 13*k_starts ||
          host_dones != 1 || busy !== 1'b0 || done !== 1'b0 || segment_inflight || job_inflight)
        $fatal(1, "FAIL TILE_SIZE=%0d job=%0d: aggregate counts",TILE_SIZE,id);
      // Check capture coverage, then independently read the REAL Output BRAM.
      for (int m=0; m<16; m++) begin
        if (seen[m] !== (m<m_count)) $fatal(1, "FAIL: output address coverage");
        if (m<m_count)
          for (int n=0; n<8; n++)
            if ($signed(captured[m][n*32 +: 32]) !== golden[m][n])
              $fatal(1, "FAIL: captured final output differs from full Golden");
      end
      if(act_responses!=4608 || weight_responses!=4608)
        $fatal(1,"FAIL: real Reader return counts");
      read_output_memory();
      active=0;
      $display("CASE PASS: K_TILE_SIZE=%0d job=%0d M=%0d N=8 K=%0d segments=%0d pairs=%0d writes=%0d outputs=%0d/%0d",
               TILE_SIZE,id,m_count,k_count,k_starts,requests,writes,checked_outputs,m_count*8);
      $display("  REAL BRAM: A loads=%0d bytes, W loads=%0d words, A/W responses=%0d/%0d, readbacks=%0d/128, changed=%0d",
               act_loads,weight_loads,act_responses,weight_responses,readbacks,changed_readbacks);
    end
  endtask

  initial begin
    finished=0; rst_n=0; start=0; m_len=0; n_len=8; k_len=0;
    active=0; segment_inflight=0; job_inflight=0; run_id=0;
    act_wr_en=0; act_wr_addr=0; act_wr_data=0;
    weight_wr_en=0; weight_wr_addr=0; weight_wr_data=0;
    out_rd_en=0; out_rd_addr=0;
    act_loads=0; weight_loads=0; act_responses=0; weight_responses=0; readbacks=0;
    total_act_loads=0; total_weight_loads=0; total_readbacks=0;
    total_act_responses=0; total_weight_responses=0;
    total_requests=0; total_feeds=0; total_drains=0; total_writes=0;
    total_k_starts=0; total_k_dones=0; total_outputs=0; total_host_dones=0;
    expected_changed=0; changed_readbacks=0;
    for(int m=0;m<16;m++)for(int n=0;n<8;n++)previous_result[m][n]=0;
    repeat (4) @(negedge clk);
    rst_n=1;
    run_job(0,16,576);
    run_job(1,16,576); // reload both memories; NO reset of PE, controller or RAM
    if(total_requests!=9216 || total_feeds!=9216 || total_writes!=32 ||
       total_outputs!=256 || total_readbacks!=256 || total_host_dones!=2 ||
       total_k_starts!=288 || total_k_dones!=288 || total_drains!=3744 ||
       total_act_responses!=9216 || total_weight_responses!=9216 ||
       total_act_loads!=10368 || total_weight_loads!=1152)
      $fatal(1,"FAIL: lifetime real-BRAM integration counts");
    finished=1;
    $display("PASS: P6-B2-5 REAL BRAM + AUTOMATIC K-TILE GEMM + PE + OUTPUT BRAM");
    $display("  One DUT, one reset, 2 jobs; each M=16 N=8 K=576, 144 K segments");
    $display("  9216 requests/FEED, 288 K starts/dones, 32 FINAL writes");
    $display("  256/256 INT32 convolution outputs and 256/256 synchronous BRAM readbacks correct");
    $display("  128 changed cached outputs checked; all request banks, partial sums and M transitions checked");
    $finish;
  end
  initial begin
    repeat(100000) @(posedge clk);
    $fatal(1,"FAIL: TIMEOUT tb_p6b2_real_bram_gemm");
  end
endmodule
