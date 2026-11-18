`timescale 1ns / 1ps

// P6-B2-4: automatic outer K-segment scheduling, M advancement and FINAL writes.
// Testbench drives only one external start per GEMM job. It never drives the
// inner K Controller or its Tile base. Reader is a one-clock synthetic model,
// addressed solely by DUT m_addr/n_base/fetch_k_base (no hidden Tile offset).
// Real BRAM / convolution Reader integration belongs to P6-B2-5.
// Sources: pkg_accelerator, mac_unit, systolic_array_p2, k_tile_controller_p2,
//          gemm_mn_controller_mem_p2, this file (all .sv).
module tb_p6b2_gemm_ktile_scheduler;
  logic clk = 1'b0;
  logic tiled_finished, legacy_finished;
  always #5 clk = ~clk;

  p6b2_gemm_ktile_case #(.TILE_SIZE(64)) u_tiled (
    .clk(clk), .finished(tiled_finished)
  );
  p6b2_gemm_ktile_case #(.TILE_SIZE(0)) u_legacy (
    .clk(clk), .finished(legacy_finished)
  );

  initial begin
    wait (tiled_finished === 1'b1 && legacy_finished === 1'b1);
    $display("PASS: P6-B2-4 AUTOMATIC K-TILE SCHEDULING AND FINAL WRITEBACK");
    $display("  Tiled: 7 jobs, 166 K starts/dones, 5103 requests/FEED, 27 final writes, 216/216 outputs");
    $display("  Legacy: 7 jobs, 27 K starts/dones, 5103 requests/FEED, 27 final writes, 216/216 outputs");
    $display("  Main tiled M16 N8 K576: 144 K segments, 4608 pairs, 16 writes, 128/128 outputs");
    $display("  Tail K=2, short K, config latching, PE partial sums, M reset, repeated starts checked");
    $display("  One reset per DUT; synchronous Reader MODEL; no real BRAM integration claim");
    $finish;
  end
  initial begin
    repeat (100000) @(posedge clk);
    $fatal(1, "FAIL: P6-B2-4 global timeout");
  end
endmodule

module p6b2_gemm_ktile_case #(
  parameter int TILE_SIZE = 64
)(
  input logic clk,
  output logic finished
);
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

  function automatic integer signed aval(input integer job, input integer m, input integer k);
    aval = ((job*53 + m*31 + k*17 + 19) % 256) - 128;
  endfunction
  function automatic integer signed bval(input integer job, input integer k, input integer n);
    bval = ((job*71 + k*29 + n*43 + 7) % 251) - 125;
  endfunction
  function automatic integer seg_length(input integer seg);
    integer remaining;
    begin
      remaining = expected_k_count - seg*segment_cap;
      seg_length = (remaining > segment_cap) ? segment_cap : remaining;
    end
  endfunction

  // Synchronous response, using ACTUAL request indices. No TB Tile selection.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      act_pair_packed <= '0; wgt_pair_packed <= '0;
      act_rsp_valid <= '0; wgt_rsp_valid <= '0;
      rsp_m <= '0; rsp_n <= '0; rsp_k <= '0;
    end else begin
      act_rsp_valid <= {2{pair_req_en}};
      wgt_rsp_valid <= {2{pair_req_en}};
      if (pair_req_en) begin
        rsp_m <= m_addr; rsp_n <= n_base; rsp_k <= fetch_k_base;
        for (int p = 0; p < 2; p++) begin
          act_pair_packed[p*8 +: 8] <= 8'(aval(run_id, int'(m_addr), int'(fetch_k_base)+p));
          for (int n = 0; n < 8; n++)
            wgt_pair_packed[(p*8+n)*8 +: 8] <=
                8'(bval(run_id, int'(fetch_k_base)+p, int'(n_base)+n));
        end
      end else begin
        act_pair_packed <= '0;
        wgt_pair_packed <= '0;
      end
    end
  end

  // All protocol checks sample pre-NBA signals, as the synchronous DUT does.
  always @(posedge clk) begin : monitor
    integer base, length, local_k, global_k;
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
          segment_requests++; requests++; total_requests++;
        end

        if (mem_feed_valid) begin
          local_k = 2*segment_feeds;
          global_k = base + local_k;
          if (!segment_inflight || segment_feeds >= length/2 ||
              segment_clears != 1 || !dut.array_enable || dut.clear_pipe ||
              k_base !== 16'(local_k) || m_addr !== 16'(writes) || n_base !== 16'd0 ||
              rsp_k !== 16'(global_k) || rsp_m !== 16'(writes) || rsp_n !== 16'd0 ||
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
    integer waited, gk, row_idx;
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
      for (int m=0; m<16; m++) begin seen[m]=0; captured[m]='x; end

      // Final Golden: direct full dot product, independent of Tile scheduling.
      for (int m=0; m<m_count; m++) begin
        for (int n=0; n<8; n++) begin
          golden[m][n]=0;
          for (int k=0; k<k_count; k++)
            golden[m][n] += aval(id,m,k)*bval(id,k,n);
        end
      end
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
      // Capture storage is TB-only, not output BRAM. Check each output address
      // once and ensure unused slots were never written.
      for (int m=0; m<16; m++) begin
        if (seen[m] !== (m<m_count)) $fatal(1, "FAIL: output address coverage");
        if (m<m_count)
          for (int n=0; n<8; n++)
            if ($signed(captured[m][n*32 +: 32]) !== golden[m][n])
              $fatal(1, "FAIL: captured final output differs from full Golden");
      end
      active=0;
      $display("CASE PASS: K_TILE_SIZE=%0d job=%0d M=%0d N=8 K=%0d segments=%0d pairs=%0d writes=%0d outputs=%0d/%0d",
               TILE_SIZE,id,m_count,k_count,k_starts,requests,writes,checked_outputs,m_count*8);
    end
  endtask

  initial begin
    finished=0; rst_n=0; start=0; m_len=0; n_len=8; k_len=0;
    active=0; segment_inflight=0; job_inflight=0; run_id=0;
    total_requests=0; total_feeds=0; total_drains=0; total_writes=0;
    total_k_starts=0; total_k_dones=0; total_outputs=0; total_host_dones=0;
    repeat (4) @(negedge clk);
    rst_n=1;
    run_job(0, 1, 128);  // two K segments
    run_job(1,16, 576);  // main: 16 M x 9 segments, 16 FINAL writes
    run_job(2, 3, 130);  // 64+64+2, partial last row group
    run_job(3, 2,  10);  // shorter than cap, wraps PE row 7 -> 0
    run_job(4, 1,  64);  // exactly one segment, no spurious extra start
    run_job(5, 2,  66);  // 64+2
    run_job(6, 2, 128);  // restart earlier shape with different signed data
    if (total_requests != 5103 || total_feeds != 5103 || total_writes != 27 ||
        total_outputs != 216 || total_host_dones != 7 ||
        total_k_starts != ((TILE_SIZE==0) ? 27 : 166) ||
        total_k_dones != total_k_starts || total_drains != 13*total_k_starts)
      $fatal(1, "FAIL TILE_SIZE=%0d: lifetime counts",TILE_SIZE);
    finished=1;
  end
endmodule
