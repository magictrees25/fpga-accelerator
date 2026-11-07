`timescale 1ns / 1ps

// ============================================================================
// P3: Full 49 M-Tile x 8 N-Tile (392-Tile) functional/performance testbench.
//
// Scope:
//   - All 49 spatial M tiles, selected in row-major order.
//   - Each M tile covers an 8x8 output region.
//   - Each M tile executes all 8 N tiles (8 output channels per N tile).
//   - Each run computes M=64, N=8, K=576.
//   - Total runs: 49 M tiles * 8 N tiles = 392.
//   - Input patch per M tile: 10x10x64, including one-pixel halo.
//   - Golden model uses global 56x56x64 feature-map coordinates.
//
// The RTL is unchanged. This testbench drives the existing P3 single-tile
// interface repeatedly from the outer testbench.
//
// M-tile order (row-major over the complete 7x7 grid):
//   mt=0..6:   tile_y=0, tile_x=0..6
//   mt=7..13:  tile_y=1, tile_x=0..6
//   ...
//   mt=42..48: tile_y=6, tile_x=0..6
//
// The testbench verifies:
//   1. Activation Line Buffer request/address/data.
//   2. Activation BRAM row-prefetch addresses and Line Buffer writes.
//   3. Weight BRAM request/address/data.
//   4. K-pair feed, drain, launch, K-done and output-write sequencing.
//   5. Every output against an independent direct-convolution Golden Model.
// ============================================================================

// Keeps original 392-Tile verification and adds P2.1-compatible CSV counters.
// CSV data are TB-observed cycles and port events, NOT board/DDR measurements.
module tb_layer1_p3_perf;

  localparam int ADDR_WIDTH = 16;
  localparam int M_LEN = 64;
  localparam int K_LEN = 576;
  localparam int PAIRS_PER_M = K_LEN / 2;
  localparam int PAIRS_PER_TILE = M_LEN * PAIRS_PER_M;

  localparam int M_TILE_COUNT = 49;
  localparam int N_TILE_COUNT = 8;
  localparam int TOTAL_RUNS = M_TILE_COUNT * N_TILE_COUNT;

  localparam int OUTPUTS_PER_RUN = 512;
  localparam int COMPUTE_CYCLES_PER_RUN = 19520;
  // Includes activation/weight preload, output readback and compute time.
  localparam int SIM_TIMEOUT_CYCLES = 12000000;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst_n, start;
  logic fmap_wr_en, weight_wr_en, out_rd_en;
  logic [ADDR_WIDTH-1:0] fmap_wr_addr, weight_wr_addr, out_rd_addr;
  logic [7:0] fmap_wr_data;
  logic [63:0] weight_wr_data;
  logic [2:0] out_rd_lane_sel;

  logic signed [31:0] out_rd_data;
  logic out_rd_lane_valid, out_rd_valid;
  logic busy, done, any_overflow;

  fpga_system_top_layer1_tile_p3 #(
    .ADDR_WIDTH(ADDR_WIDTH)
  ) dut (.*);

  // ---------------- Hierarchical observation points ----------------

  wire clear_acc    = dut.u_compute.clear_acc;
  wire feed_valid   = dut.u_compute.feed_valid;
  wire pair_req_en  = dut.u_compute.pair_req_en;
  wire [15:0] k_base       = dut.u_compute.k_base;
  wire [15:0] fetch_k_base = dut.u_compute.fetch_k_base;
  wire [15:0] m_addr       = dut.u_compute.m_addr;
  wire [15:0] n_base       = dut.u_compute.n_base;

  wire [15:0] act_pair = dut.u_compute.act_pair_packed;
  wire [127:0] wgt_pair = dut.u_compute.wgt_pair_packed;
  wire [1:0] act_valid = dut.u_compute.act_rsp_valid;
  wire [1:0] wgt_valid = dut.u_compute.wgt_rsp_valid;

  wire k_start = dut.u_compute.u_ctrl.k_start;
  wire k_done  = dut.u_compute.u_ctrl.k_done;
  wire array_enable = dut.u_compute.u_ctrl.array_enable;
  wire drain = array_enable && !clear_acc && !feed_valid;

  wire write_valid = dut.write_valid;
  wire [15:0] write_addr = dut.write_addr;
  wire [7:0] write_mask = dut.write_mask;
  wire [255:0] write_data = dut.write_data;

  // ---------------- Independent Golden Model ----------------

  logic signed [7:0] global_fmap [0:55][0:55][0:63];
  logic signed [7:0] global_kernel [0:2][0:2][0:63][0:63];
  logic signed [31:0] golden [0:7][0:7][0:63];

  integer k_hits [0:575];

  integer active_m_tile;
  integer active_n_tile;

  function automatic integer activation(input integer m, input integer k);
    integer tile_y, tile_x;
    integer gy, gx, c;
    begin
      tile_y = active_m_tile / 7;
      tile_x = active_m_tile % 7;

      gy = tile_y*8 + m/8 + k/192 - 1;
      gx = tile_x*8 + m%8 + (k/64)%3 - 1;
      c  = k % 64;

      if (gy < 0 || gy >= 56 || gx < 0 || gx >= 56)
        activation = 0;
      else
        activation = int'($signed(global_fmap[gy][gx][c]));
    end
  endfunction

  function automatic integer weight_value(input integer k, input integer n);
    begin
      weight_value =
        int'($signed(global_kernel[k/192][(k/64)%3][k%64][n]));
    end
  endfunction

  // Initialize the global feature map and all 64 output-channel kernels once.
  task automatic prepare_global_data;
    begin
      for (int y=0; y<56; y++)
        for (int x=0; x<56; x++)
          for (int c=0; c<64; c++)
            global_fmap[y][x][c] =
              8'(((17*y + 11*x + 7*c) % 127) - 63);

      for (int ky=0; ky<3; ky++)
        for (int kx=0; kx<3; kx++)
          for (int c=0; c<64; c++)
            for (int n=0; n<64; n++)
              global_kernel[ky][kx][c][n] =
                8'(((13*ky + 7*kx + 3*c + 5*n) % 19) - 9);
    end
  endtask

  // Recalculate the 8x8x64 golden outputs for the currently selected M tile.
  task automatic prepare_golden_for_current_m_tile;
    integer tile_y, tile_x;
    integer gy, gx;
    integer signed sum;
    begin
      tile_y = active_m_tile / 7;
      tile_x = active_m_tile % 7;

      $display("INFO: Build Golden Model for M-Tile %0d (tile_y=%0d, tile_x=%0d)",
               active_m_tile, tile_y, tile_x);

      for (int oy=0; oy<8; oy++)
        for (int ox=0; ox<8; ox++)
          for (int n=0; n<64; n++) begin
            sum = 0;
            for (int ky=0; ky<3; ky++)
              for (int kx=0; kx<3; kx++)
                for (int c=0; c<64; c++) begin
                  gy = tile_y*8 + oy + ky - 1;
                  gx = tile_x*8 + ox + kx - 1;

                  if (gy >= 0 && gy < 56 && gx >= 0 && gx < 56)
                    sum += int'($signed(global_fmap[gy][gx][c])) *
                           int'($signed(global_kernel[ky][kx][c][n]));
                end

            golden[oy][ox][n] = sum;
          end

      // Preserve the original single-tile Golden Model anchor check.
      if (active_m_tile == 0) begin
        if (golden[0][0][0] !== 2453 || golden[7][7][7] !== 1616)
          $fatal(1, "FAIL: Golden Model anchor mismatch on M-Tile 0");
      end
    end
  endtask

  // ---------------- P3 address and prefetch checks ----------------

  task automatic check_activation_request(
    input integer em,
    input integer ek
  );
    integer input_row, input_col, expected_lb_addr;
    begin
      input_row = em/8 + ek/192;
      input_col = em%8 + (ek/64)%3;

      // Four-row circular buffer:
      // slot = input_row % 4
      // address = slot*320 + input_col*32 + channel_pair
      expected_lb_addr =
        ((input_row % 4) * 10 + input_col) * 32 + ((ek % 64) / 2);

      if (dut.u_compute.u_act_reader.lb_rd_en !== 1'b1)
        $fatal(1,
          "FAIL: P3 Line Buffer read enable low MT=%0d M=%0d K=%0d",
          active_m_tile, em, ek);

      if (dut.u_compute.u_act_reader.lb_rd_addr !== 11'(expected_lb_addr))
        $fatal(1,
          "FAIL: P3 Line Buffer address MT=%0d M=%0d K=%0d got=%0d expected=%0d",
          active_m_tile, em, ek,
          dut.u_compute.u_act_reader.lb_rd_addr, expected_lb_addr);

      // Weight Reader remains the P2.1 implementation.
      if (dut.u_compute.u_weight_reader.bank_rd_en !== 2'b11 ||
          dut.u_compute.u_weight_reader.bank_rd_addr[0] !== 16'(ek/2) ||
          dut.u_compute.u_weight_reader.bank_rd_addr[1] !== 16'(ek/2))
        $fatal(1,
          "FAIL: Weight Reader request/address MT=%0d M=%0d K=%0d",
          active_m_tile, em, ek);
    end
  endtask

  // ---------------- Counters ----------------

  integer fmap_load_count, fmap_expected_write_count, fmap_total_write_count;
  integer weight_load_count;

  integer request_count, line_buffer_read_count;
  integer prefetch_trigger_count, prefetch_read_count, prefetch_write_count;

  integer feed_pair_count, clear_count, drain_count;
  integer launch_count, kdone_count, output_write_count;
  integer round_pairs, round_drains;
  integer readback_count;

  integer m_idx, k_idx, got_value, expected_value;
  integer idx, row_idx, col_idx, expected_addr;
  integer waited;

  integer completed_m_tiles, completed_mn_runs;

  integer total_clear_count, total_feed_pair_count, total_drain_count;
  integer total_launch_count, total_kdone_count, total_output_write_count;

  longint unsigned run_compute_cycles;

  logic round_open, computing, start_q, done_q, finished;
  logic [255:0] result_snapshot;
  logic [63:0] expected_injection;

  longint unsigned tick_count, start_tick, done_tick, compute_cycles;

  // ---------------- Performance measurements ----------------
  string tile_csv = "layer1_p3_tiles.csv";
  string summary_csv = "layer1_p3_summary.csv";
  integer tile_fd, summary_fd;

  longint unsigned first_preload_tick, last_read_tick;
  longint unsigned first_tile_preload_tick, last_tile_read_tick;
  longint unsigned starts_seen, dones_seen;
  longint unsigned total_weight_load_words;
  longint unsigned total_read_requests, total_read_responses;
  longint unsigned total_padding_preload_bytes;
  longint unsigned total_line_buffer_init_bytes;

  integer tile_activation_load_bytes, tile_weight_load_words;
  integer tile_read_requests, tile_read_responses;

  // P2.1 uses the same 10-ns TB clock and the same compute / E2E edges.
  task automatic csv_i(input string name,
                       input longint unsigned value,
                       input string units);
    $fdisplay(summary_fd, "%s,%0d,%s", name, value, units);
  endtask

  task automatic csv_r(input string name, input real value,
                       input string units);
    $fdisplay(summary_fd, "%s,%0.8f,%s", name, value, units);
  endtask

  task automatic emit_tile_csv;
    longint unsigned tile_e2e;
    integer serial_id;
    begin
      serial_id = active_m_tile*N_TILE_COUNT + active_n_tile;
      if (first_tile_preload_tick == 0 ||
          last_tile_read_tick < first_tile_preload_tick ||
          tile_activation_load_bytes !=
            ((active_n_tile == 0) ? 6400 : 2560) ||
          tile_weight_load_words != 576 ||
          tile_read_requests != OUTPUTS_PER_RUN ||
          tile_read_responses != OUTPUTS_PER_RUN)
        $fatal(1, "FAIL: P3 tile performance accounting MT=%0d NT=%0d",
               active_m_tile, active_n_tile);

      tile_e2e = last_tile_read_tick - first_tile_preload_tick + 1;
      $fdisplay(tile_fd,
        "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
        serial_id, (active_m_tile/7)*8, (active_m_tile%7)*8,
        active_n_tile*8, start_tick, done_tick, run_compute_cycles,
        tile_activation_load_bytes, tile_weight_load_words,
        tile_read_requests, tile_read_responses,
        tile_activation_load_bytes, 8*tile_weight_load_words,
        4*tile_read_responses, first_tile_preload_tick,
        last_tile_read_tick, tile_e2e);
    end
  endtask

  task automatic emit_summary_csv;
    longint unsigned e2e_cycles, gap_cycles, macs, host_payload;
    begin
      if (first_preload_tick == 0 || last_read_tick < first_preload_tick)
        $fatal(1, "FAIL: E2E measurement boundaries");
      e2e_cycles = last_read_tick - first_preload_tick + 1;
      macs = longint'(TOTAL_RUNS) * OUTPUTS_PER_RUN * K_LEN;
      host_payload = longint'(fmap_total_write_count) +
                     total_weight_load_words*8 +
                     total_read_responses*4;

      if (e2e_cycles <
          compute_cycles + longint'(fmap_total_write_count) +
          total_weight_load_words + total_read_requests)
        $fatal(1, "FAIL: P3 E2E phase accounting");
      gap_cycles = e2e_cycles - compute_cycles -
                   longint'(fmap_total_write_count) -
                   total_weight_load_words - total_read_requests;

      csv_i("tiles", TOTAL_RUNS, "tiles");
      csv_i("start_count", starts_seen, "events");
      csv_i("done_count", dones_seen, "events");
      csv_i("pe_count", 64, "PE");
      csv_i("k_parallel", 2, "K_per_pair");
      $fdisplay(summary_fd, "variant,P3_LINE_BUFFER,text");
      csv_i("dense_mac_count", macs, "MAC");
      csv_i("sum_start_to_done_cycles", compute_cycles, "cycles");
      csv_i("tile_compute_min_cycles", COMPUTE_CYCLES_PER_RUN, "cycles");
      csv_i("tile_compute_max_cycles", COMPUTE_CYCLES_PER_RUN, "cycles");
      csv_i("feed_pair_cycles", total_feed_pair_count, "cycles");
      csv_i("valid_k_elements", total_feed_pair_count*2, "K_elements");
      csv_i("pair_request_cycles", request_count, "cycles");
      csv_i("clear_cycles", total_clear_count, "cycles");
      csv_i("drain_cycles", total_drain_count, "cycles");
      csv_i("launch_cycles", total_launch_count, "cycles");
      csv_i("kdone_cycles", total_kdone_count, "cycles");
      csv_i("output_bram_write_cycles", total_output_write_count, "cycles");
      csv_i("compute_other_cycles", 0, "cycles");
      csv_i("array_enabled_nonclear_cycles",
            total_feed_pair_count+total_drain_count, "cycles");
      csv_i("activation_preload_cycles", fmap_total_write_count, "cycles");
      csv_i("weight_preload_cycles", total_weight_load_words, "cycles");
      csv_i("output_read_request_cycles", total_read_requests, "cycles");
      csv_i("output_read_response_cycles", total_read_responses, "cycles");
      csv_i("e2e_first_preload_cycle", first_preload_tick, "cycle_index");
      csv_i("e2e_last_read_cycle", last_read_tick, "cycle_index");
      csv_i("e2e_cycles", e2e_cycles, "cycles");
      csv_i("host_gap_cycles", gap_cycles, "cycles");
      csv_i("activation_preload_bytes", fmap_total_write_count, "bytes");
      csv_i("padding_preload_bytes", total_padding_preload_bytes, "bytes");
      csv_i("real_activation_preload_bytes",
            fmap_total_write_count-total_padding_preload_bytes, "bytes");
      csv_i("weight_preload_bytes", total_weight_load_words*8, "bytes");
      csv_i("output_read_bytes", total_read_responses*4, "bytes");
      csv_i("output_bram_write_bytes", total_output_write_count*32, "bytes");
      csv_i("host_payload_bytes", host_payload, "bytes");

      // P3-specific measured hardware read and prefetch activity.
      csv_i("activation_bram_pair_reads", prefetch_read_count, "pairs");
      csv_i("line_buffer_pair_reads", line_buffer_read_count, "pairs");
      csv_i("line_buffer_prefetch_pair_writes",
            prefetch_write_count, "pairs");
      csv_i("line_buffer_initial_preload_bytes",
            total_line_buffer_init_bytes, "bytes");
      csv_i("bram_prefetch_rows", prefetch_trigger_count, "rows");
      csv_i("weight_bram_pair_reads", request_count, "pairs");

      csv_r("tb_clock_period_ns", 10.0, "ns");
      csv_r("compute_latency_at_tb_clock_ms",
            real'(compute_cycles)*10.0/1.0e6, "ms");
      csv_r("e2e_latency_at_tb_clock_ms",
            real'(e2e_cycles)*10.0/1.0e6, "ms");
      csv_r("dense_util_compute_pct",
            100.0*real'(macs)/(64.0*real'(compute_cycles)), "percent");
      csv_r("dense_util_e2e_pct",
            100.0*real'(macs)/(64.0*real'(e2e_cycles)), "percent");

      $fdisplay(summary_fd, "functional_status,PASS,text");
      $fdisplay(summary_fd, "run_status,COMPLETE,text");
      $fdisplay(summary_fd,
        "measurement_scope,simulation_only_not_board_or_DDR,text");
      $fdisplay(summary_fd,
        "compute_window,start_accept_edge_exclusive_to_done_assert_edge_inclusive,text");
      $fdisplay(summary_fd,
        "e2e_window,first_preload_edge_to_last_read_response_edge_inclusive,text");
      $fdisplay(summary_fd,
        "host_loading_policy,full_patch_per_M_tile_then_four_row_restore_for_later_N_tiles,text");
      $fdisplay(summary_fd,
        "resource_and_timing_status,external_implementation_reports,text");
      $fclose(tile_fd);
      $fclose(summary_fd);

      $display("CSV READY: %s ; %s (simulation working directory)",
               tile_csv, summary_csv);
      $display("PERF: P3 compute=%0d cycles, E2E=%0d cycles",
               compute_cycles, e2e_cycles);
      $display("PERF: BRAM activation pair reads=%0d, Line Buffer pair reads=%0d",
               prefetch_read_count, line_buffer_read_count);
      $display("PERF: TB 100MHz accounting only; not board/DDR latency");
    end
  endtask

  // ---------------- Main monitor ----------------

  always @(posedge clk) begin : monitor
    bit count_compute;
    integer em, ek, p, n;
    integer local_y, local_x, global_y, global_x;

    if (!rst_n) begin
      tick_count = 0;
      computing = 0;
      start_q = 0;
      done_q = 0;
    end else begin
      tick_count = tick_count + 1;
      count_compute = computing;

      if ($isunknown({clear_acc, feed_valid, pair_req_en,
                      write_valid, busy, done, any_overflow}) ||
          any_overflow !== 1'b0)
        $fatal(1, "FAIL: Unknown control or arithmetic overflow");

      if ((clear_acc || feed_valid || write_valid || k_done || drain) &&
          !computing)
        $fatal(1, "FAIL: Compute transaction outside start/done window");

      if (start) begin
        if (start_q || busy || done || computing ||
            fmap_load_count != fmap_expected_write_count ||
            weight_load_count != 576)
          $fatal(1,
            "FAIL: Start before preload completion or while busy MT=%0d NT=%0d",
            active_m_tile, active_n_tile);

        computing = 1;
        start_tick = tick_count;
        starts_seen = starts_seen + 1;

        // Per-run counters are reset for each of the 64 M/N tile runs.
        clear_count = 0;
        feed_pair_count = 0;
        drain_count = 0;
        launch_count = 0;
        kdone_count = 0;
        output_write_count = 0;
        round_pairs = 0;
        round_drains = 0;
        round_open = 0;
        run_compute_cycles = 0;
      end
      start_q = start;

      if (count_compute) begin
        compute_cycles = compute_cycles + 1;
        run_compute_cycles = run_compute_cycles + 1;
      end

      // P2.1-comparable E2E first preload edge (not start of simulation).
      if (fmap_wr_en || weight_wr_en) begin
        if (first_preload_tick == 0)
          first_preload_tick = tick_count;
        if (first_tile_preload_tick == 0)
          first_tile_preload_tick = tick_count;
      end

      if (int'(count_compute)+int'(fmap_wr_en)+
          int'(weight_wr_en)+int'(out_rd_en) > 1)
        $fatal(1, "FAIL: overlapping P3 host/compute phases");

      // Count byte writes into first four Line Buffer rows.
      if (dut.u_compute.u_act_reader.preload_lb_accept)
        total_line_buffer_init_bytes = total_line_buffer_init_bytes + 1;

      // Host-side weight preload checking.
      if (weight_wr_en) begin
        if (weight_wr_addr !== 16'(weight_load_count) ||
            weight_load_count >= 576)
          $fatal(1, "FAIL: Weight preload address/order");

        weight_load_count = weight_load_count + 1;
        total_weight_load_words = total_weight_load_words + 1;
        tile_weight_load_words = tile_weight_load_words + 1;
      end

      // Host-side activation preload checking.
      if (fmap_wr_en) begin
        if (fmap_wr_addr !== 16'(fmap_load_count) ||
            fmap_load_count >= fmap_expected_write_count)
          $fatal(1,
            "FAIL: Activation preload address/order MT=%0d addr=%0d count=%0d expected_limit=%0d",
            active_m_tile, fmap_wr_addr, fmap_load_count,
            fmap_expected_write_count);

        fmap_load_count = fmap_load_count + 1;
        fmap_total_write_count = fmap_total_write_count + 1;
        tile_activation_load_bytes = tile_activation_load_bytes + 1;

        // Global image border padding accounting, identical semantics
        // to P2.1 padding_preload_bytes.
        local_y = int'($unsigned(fmap_wr_addr)) / 640;
        local_x = (int'($unsigned(fmap_wr_addr)) / 64) % 10;
        global_y = (active_m_tile/7)*8 + local_y - 1;
        global_x = (active_m_tile%7)*8 + local_x - 1;
        if (global_y < 0 || global_y >= 56 ||
            global_x < 0 || global_x >= 56)
          total_padding_preload_bytes = total_padding_preload_bytes + 1;
      end

      // P3 Line Buffer read requests are driven by pair_req_en.
      if (pair_req_en) begin
        check_activation_request(int'(m_addr), int'(fetch_k_base));
        request_count = request_count + 1;
      end

      if (dut.u_compute.u_act_reader.lb_rd_en)
        line_buffer_read_count = line_buffer_read_count + 1;

      // BRAM reads are row prefetches, not per-K compute requests.
      if (dut.u_compute.u_act_reader.prefetch_trigger)
        prefetch_trigger_count = prefetch_trigger_count + 1;

      if (dut.u_compute.u_act_reader.bank_rd_en === 2'b11) begin
        idx = prefetch_read_count % 1920;
        row_idx = 4 + idx/320;
        col_idx = idx%320;
        expected_addr = row_idx*320 + col_idx;

        if (dut.u_compute.u_act_reader.prefetch_row !== 4'(row_idx) ||
            dut.u_compute.u_act_reader.prefetch_index !== 9'(col_idx) ||
            dut.u_compute.u_act_reader.bank_rd_addr[0] !== 16'(expected_addr) ||
            dut.u_compute.u_act_reader.bank_rd_addr[1] !== 16'(expected_addr))
          $fatal(1,
            "FAIL: Activation BRAM prefetch address/order MT=%0d NT=%0d",
            active_m_tile, active_n_tile);

        prefetch_read_count = prefetch_read_count + 1;
      end else if (dut.u_compute.u_act_reader.bank_rd_en !== 2'b00) begin
        $fatal(1, "FAIL: Activation BRAM banks not read together");
      end

      // Both Line Buffer banks are written together for prefetched rows.
      if (dut.u_compute.u_act_reader.lb_wr_en === 2'b11) begin
        idx = prefetch_write_count % 1920;
        row_idx = 4 + idx/320;
        col_idx = idx%320;
        expected_addr = (row_idx%4)*320 + col_idx;

        if (dut.u_compute.u_act_reader.lb_wr_addr !== 11'(expected_addr))
          $fatal(1,
            "FAIL: Line Buffer prefetch write address/order MT=%0d NT=%0d",
            active_m_tile, active_n_tile);

        prefetch_write_count = prefetch_write_count + 1;
      end

      em = int'(m_addr);

      if (clear_acc) begin
        if (round_open || output_write_count >= 64 ||
            clear_count != output_write_count ||
            feed_pair_count != output_write_count*288 ||
            k_base !== 16'd0)
          $fatal(1, "FAIL: K round CLEAR sequence");

        round_open = 1;
        round_pairs = 0;
        round_drains = 0;
        clear_count = clear_count + 1;
        total_clear_count = total_clear_count + 1;
      end

      if (round_open && !clear_acc && round_pairs < 288 && !feed_valid)
        $fatal(1, "FAIL: Bubble detected in paired K feed");

      if (feed_valid) begin
        ek = 2*round_pairs;

        if (!round_open || round_pairs >= 288 ||
            k_base !== 16'(ek) || m_addr !== 16'(em) || n_base !== 0 ||
            act_valid !== 2'b11 || wgt_valid !== 2'b11 ||
            dut.u_compute.u_act_reader.rsp_k_base !== 16'(ek) ||
            dut.u_compute.u_act_reader.rsp_m_addr !== 16'(em) ||
            dut.u_compute.u_weight_reader.rsp_k_base !== 16'(ek) ||
            dut.u_compute.u_weight_reader.rsp_n_base !== 0)
          $fatal(1,
            "FAIL: FEED index/response tag mismatch MT=%0d NT=%0d M=%0d K=%0d",
            active_m_tile, active_n_tile, em, ek);

        expected_injection = 64'b0;

        for (p=0; p<2; p=p+1) begin
          expected_value = activation(em, ek+p);
          got_value = int'($signed(act_pair[p*8 +: 8]));

          if (got_value !== expected_value)
            $fatal(1,
              "FAIL: Activation data MT=%0d NT=%0d M=%0d K=%0d got=%0d expected=%0d",
              active_m_tile, active_n_tile, em, ek+p,
              got_value, expected_value);

          expected_injection[((ek+p)%8)*8 +: 8] = 8'(expected_value);
          k_hits[ek+p] = k_hits[ek+p] + 1;

          for (n=0; n<8; n=n+1) begin
            expected_value = weight_value(ek+p, active_n_tile*8+n);
            got_value = int'($signed(wgt_pair[(p*8+n)*8 +: 8]));

            if (got_value !== expected_value)
              $fatal(1,
                "FAIL: Weight data MT=%0d NT=%0d M=%0d K=%0d N=%0d got=%0d expected=%0d",
                active_m_tile, active_n_tile, em, ek+p, n,
                got_value, expected_value);
          end
        end

        if (dut.u_compute.u_ctrl.act_in_packed !== expected_injection ||
            dut.u_compute.u_ctrl.wgt_pair_in_packed !== wgt_pair)
          $fatal(1, "FAIL: K injection mapping");

        round_pairs = round_pairs + 1;
        feed_pair_count = feed_pair_count + 1;
        total_feed_pair_count = total_feed_pair_count + 1;
      end

      if (!pair_req_en &&
          dut.u_compute.u_weight_reader.bank_rd_en !== 2'b00)
        $fatal(1, "FAIL: Weight BRAM read request leaked");

      if (drain) begin
        if (!round_open || round_pairs != 288 ||
            dut.u_compute.u_ctrl.act_in_packed !== 0 ||
            dut.u_compute.u_ctrl.wgt_pair_in_packed !== 0)
          $fatal(1, "FAIL: DRAIN boundary");

        round_drains = round_drains + 1;
        drain_count = drain_count + 1;
        total_drain_count = total_drain_count + 1;
      end

      if (k_start) begin
        launch_count = launch_count + 1;
        total_launch_count = total_launch_count + 1;
      end

      if (k_done) begin
        if (!round_open || round_pairs != 288 || round_drains != 13)
          $fatal(1, "FAIL: K_DONE phase count M=%0d", em);

        result_snapshot = dut.u_compute.u_ctrl.k_result_packed;

        for (n=0; n<8; n=n+1)
          if (int'($signed(result_snapshot[n*32 +: 32])) !==
              golden[em/8][em%8][active_n_tile*8+n])
            $fatal(1,
              "FAIL: Golden result MT=%0d NT=%0d M=%0d N=%0d",
              active_m_tile, active_n_tile, em, n);

        kdone_count = kdone_count + 1;
        total_kdone_count = total_kdone_count + 1;
      end

      if (write_valid) begin
        if (!round_open || round_pairs != 288 || round_drains != 13 ||
            output_write_count >= 64 ||
            write_addr !== 16'(em) ||
            dut.write_m_addr !== 16'(em) ||
            dut.write_n_base !== 0 ||
            write_mask !== 8'hff ||
            write_data !== result_snapshot)
          $fatal(1,
            "FAIL: Output BRAM write metadata/data MT=%0d NT=%0d",
            active_m_tile, active_n_tile);

        for (n=0; n<8; n=n+1)
          if (int'($signed(write_data[n*32 +: 32])) !==
              golden[em/8][em%8][active_n_tile*8+n])
            $fatal(1,
              "FAIL: Output write Golden mismatch MT=%0d NT=%0d M=%0d N=%0d",
              active_m_tile, active_n_tile, em, n);

        output_write_count = output_write_count + 1;
        total_output_write_count = total_output_write_count + 1;
        round_open = 0;
        round_pairs = 0;
        round_drains = 0;
      end

      if (out_rd_en) begin
        total_read_requests = total_read_requests + 1;
        tile_read_requests = tile_read_requests + 1;
      end

      #1;
      if (out_rd_valid) begin
        if (out_rd_lane_valid !== 1'b1)
          $fatal(1, "FAIL: P3 output read lane invalid during PERF");
        total_read_responses = total_read_responses + 1;
        tile_read_responses = tile_read_responses + 1;
        last_read_tick = tick_count;
        last_tile_read_tick = tick_count;
      end

      if (done && done_q)
        $fatal(1, "FAIL: done wider than one clock");

      if (done) begin
        if (!computing || output_write_count != 64)
          $fatal(1, "FAIL: Unexpected done");

        computing = 0;
        done_tick = tick_count;
        dones_seen = dones_seen + 1;
      end

      done_q = done;
    end
  end

  // ---------------- Test stimulus tasks ----------------

  task automatic load_activation_patch;
    integer tile_y, tile_x;
    integer gy, gx;
    begin
      tile_y = active_m_tile / 7;
      tile_x = active_m_tile % 7;

      // Load this M tile's 10x10x64 patch, including its one-pixel halo.
      // The same patch is shared by all eight N tiles.
      for (int y=0; y<10; y++)
        for (int x=0; x<10; x++)
          for (int c=0; c<64; c++) begin
            @(negedge clk);
            fmap_wr_en = 1'b1;
            fmap_wr_addr = 16'((y*10+x)*64+c);

            gy = tile_y*8 + y - 1;
            gx = tile_x*8 + x - 1;

            if (gy < 0 || gy >= 56 || gx < 0 || gx >= 56)
              fmap_wr_data = 8'd0;
            else
              fmap_wr_data = global_fmap[gy][gx][c];
          end

      @(negedge clk);
      fmap_wr_en = 1'b0;
    end
  endtask

  task automatic reload_initial_four_rows;
    integer tile_y, tile_x;
    integer y, x, c, gy, gx;
    begin
      tile_y = active_m_tile / 7;
      tile_x = active_m_tile % 7;

      // After a complete M traversal, the circular Line Buffer contains
      // rows 6..9. Restore rows 0..3 before restarting M=0 for the next N tile.
      for (y=0; y<4; y=y+1)
        for (x=0; x<10; x=x+1)
          for (c=0; c<64; c=c+1) begin
            @(negedge clk);
            fmap_wr_en = 1'b1;
            fmap_wr_addr = 16'((y*10+x)*64+c);

            gy = tile_y*8 + y - 1;
            gx = tile_x*8 + x - 1;

            if (gy < 0 || gy >= 56 || gx < 0 || gx >= 56)
              fmap_wr_data = 8'd0;
            else
              fmap_wr_data = global_fmap[gy][gx][c];
          end

      @(negedge clk);
      fmap_wr_en = 1'b0;
    end
  endtask

  task automatic load_weights_for_n_tile(input integer n_tile);
    begin
      // Each 64-bit word carries 8 consecutive output-channel weights.
      for (int k=0; k<576; k++) begin
        @(negedge clk);
        weight_wr_en = 1'b1;
        weight_wr_addr = 16'(k);

        for (int n=0; n<8; n++)
          weight_wr_data[n*8 +: 8] =
            8'(weight_value(k, n_tile*8+n));
      end

      @(negedge clk);
      weight_wr_en = 1'b0;
    end
  endtask

  task automatic read_and_check_outputs(input integer n_tile);
    integer got, expected;
    begin
      for (int m=0; m<64; m++)
        for (int n=0; n<8; n++) begin
          @(negedge clk);
          out_rd_en = 1'b1;
          out_rd_addr = 16'(m);
          out_rd_lane_sel = 3'(n);

          @(posedge clk);
          #2;

          if (out_rd_valid !== 1'b1 || out_rd_lane_valid !== 1'b1)
            $fatal(1,
              "FAIL: Output BRAM read valid MT=%0d NT=%0d M=%0d N=%0d",
              active_m_tile, n_tile, m, n);

          got = int'($signed(out_rd_data));
          expected = golden[m/8][m%8][n_tile*8+n];

          if (got !== expected)
            $fatal(1,
              "FAIL: Output BRAM data MT=%0d NT=%0d M=%0d N=%0d got=%0d expected=%0d",
              active_m_tile, n_tile, m, n, got, expected);

          readback_count = readback_count + 1;
        end

      @(negedge clk);
      out_rd_en = 1'b0;
      @(posedge clk);
      #2;

      if (out_rd_valid !== 1'b0)
        $fatal(1, "FAIL: Output BRAM stale read-valid");
    end
  endtask

  // ---------------- Main sequence: 49 M tiles x 8 N tiles ----------------

  initial begin : stimulus
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
    out_rd_lane_sel = '0;

    active_m_tile = 0;
    active_n_tile = 0;

    first_preload_tick = 0;
    last_read_tick = 0;
    first_tile_preload_tick = 0;
    last_tile_read_tick = 0;
    starts_seen = 0;
    dones_seen = 0;
    total_weight_load_words = 0;
    total_read_requests = 0;
    total_read_responses = 0;
    total_padding_preload_bytes = 0;
    total_line_buffer_init_bytes = 0;
    tile_activation_load_bytes = 0;
    tile_weight_load_words = 0;
    tile_read_requests = 0;
    tile_read_responses = 0;

    if ($value$plusargs("P3_TILES_CSV=%s", tile_csv)) begin end
    if ($value$plusargs("P3_SUMMARY_CSV=%s", summary_csv)) begin end
    tile_fd = $fopen(tile_csv, "w");
    summary_fd = $fopen(summary_csv, "w");
    if (tile_fd == 0 || summary_fd == 0)
      $fatal(1, "FAIL: cannot open P3 CSV output files");

    $fdisplay(tile_fd,
      "tile,oy,ox,oc,start_cycle,done_cycle,compute_cycles,activation_load_cycles,weight_load_cycles,read_request_cycles,read_response_cycles,activation_bytes,weight_bytes,output_read_bytes,first_load_cycle,last_read_cycle,e2e_cycles");
    $fdisplay(summary_fd, "metric,value,unit");
    $fdisplay(summary_fd, "initial_status,STARTED,text");
    $fflush(summary_fd);

    completed_m_tiles = 0;
    completed_mn_runs = 0;

    fmap_load_count = 0;
    fmap_expected_write_count = 6400;
    fmap_total_write_count = 0;
    weight_load_count = 0;

    request_count = 0;
    line_buffer_read_count = 0;
    prefetch_trigger_count = 0;
    prefetch_read_count = 0;
    prefetch_write_count = 0;

    feed_pair_count = 0;
    clear_count = 0;
    drain_count = 0;
    launch_count = 0;
    kdone_count = 0;
    output_write_count = 0;
    round_pairs = 0;
    round_drains = 0;
    round_open = 0;
    readback_count = 0;

    total_clear_count = 0;
    total_feed_pair_count = 0;
    total_drain_count = 0;
    total_launch_count = 0;
    total_kdone_count = 0;
    total_output_write_count = 0;

    result_snapshot = '0;
    expected_injection = '0;

    tick_count = 0;
    start_tick = 0;
    done_tick = 0;
    compute_cycles = 0;
    run_compute_cycles = 0;
    finished = 0;

    for (int k=0; k<576; k++)
      k_hits[k] = 0;

    prepare_global_data();

    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    $display("======================================================");
    $display("P3: Full 49 M-Tile x 8 N-Tile (392-Tile) Functional Verification");
    $display("Each run: M=64, N=8, K=576");
    $display("M tiles: 0..48; N tiles: 0..7; total runs: 392");
    $display("======================================================");

    for (int mt=0; mt<M_TILE_COUNT; mt++) begin
      active_m_tile = mt;
      active_n_tile = 0;
      first_tile_preload_tick = 0;
      last_tile_read_tick = 0;
      tile_activation_load_bytes = 0;
      tile_weight_load_words = 0;
      tile_read_requests = 0;
      tile_read_responses = 0;

      $display("");
      $display("######################################################");
      $display("INFO: Start M-Tile %0d (tile_y=%0d, tile_x=%0d)",
               mt, mt/7, mt%7);
      $display("######################################################");

      // Rebuild the golden outputs for this global 8x8 output region.
      prepare_golden_for_current_m_tile();

      // Load a new 10x10x64 activation patch for this M tile.
      @(negedge clk);
      fmap_load_count = 0;
      fmap_expected_write_count = 10*10*64;

      $display("INFO: Load activation patch for M-Tile %0d", mt);
      load_activation_patch();

      if (fmap_load_count != 6400)
        $fatal(1,
          "FAIL: M-Tile %0d activation preload count=%0d expected=6400",
          mt, fmap_load_count);

      // Each M tile executes all eight N tiles.
      for (int nt=0; nt<N_TILE_COUNT; nt++) begin
        active_n_tile = nt;

        // N=0 already contains this M tile's full activation preload.
        // N>0 will begin with a fresh four-row restore.
        if (nt > 0) begin
          first_tile_preload_tick = 0;
          last_tile_read_tick = 0;
          tile_activation_load_bytes = 0;
          tile_weight_load_words = 0;
          tile_read_requests = 0;
          tile_read_responses = 0;
        end

        // After a complete M traversal, the circular Line Buffer no longer
        // contains rows 0..3. Restore them before restarting at local M=0.
        if (nt > 0) begin
          @(negedge clk);
          fmap_load_count = 0;
          fmap_expected_write_count = 4*10*64;

          $display("INFO: Restore initial four Line Buffer rows for M-Tile %0d N-Tile %0d",
                   mt, nt);

          reload_initial_four_rows();

          if (fmap_load_count != fmap_expected_write_count)
            $fatal(1,
              "FAIL: Four-row restore count MT=%0d NT=%0d got=%0d expected=%0d",
              mt, nt, fmap_load_count, fmap_expected_write_count);
        end

        // Reset only the TB's weight-preload address counter.
        @(negedge clk);
        weight_load_count = 0;

        $display("-----------------------------------------------");
        $display("INFO: M-Tile %0d, N-Tile %0d: output channels %0d..%0d",
                 mt, nt, nt*8, nt*8+7);

        load_weights_for_n_tile(nt);

        if (weight_load_count != 576)
          $fatal(1,
            "FAIL: Weight preload count MT=%0d NT=%0d got=%0d expected=576",
            mt, nt, weight_load_count);

        repeat (2) @(negedge clk);

        $display("INFO: Start M-Tile %0d N-Tile %0d computation", mt, nt);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        waited = 0;
        while (done !== 1'b1 && waited < 30000) begin
          @(posedge clk);
          #2;
          waited = waited + 1;
        end

        if (done !== 1'b1)
          $fatal(1, "TIMEOUT: P3 M-Tile %0d N-Tile %0d compute", mt, nt);

        repeat (3) @(negedge clk);

        // Output BRAM has 8 lanes. Read back this N tile before the next
        // N tile overwrites the same 64 local M addresses.
        read_and_check_outputs(nt);

        if (clear_count != 64 ||
            feed_pair_count != PAIRS_PER_TILE ||
            request_count != (mt*N_TILE_COUNT + nt + 1)*PAIRS_PER_TILE ||
            line_buffer_read_count !=
              (mt*N_TILE_COUNT + nt + 1)*PAIRS_PER_TILE ||
            drain_count != 64*13 ||
            launch_count != 64 ||
            kdone_count != 64 ||
            output_write_count != 64)
          $fatal(1,
            "FAIL: Per-run event count MT=%0d NT=%0d",
            mt, nt);

        if (run_compute_cycles != COMPUTE_CYCLES_PER_RUN)
          $fatal(1,
            "FAIL: Compute cycles MT=%0d NT=%0d got=%0d expected=%0d",
            mt, nt, run_compute_cycles, COMPUTE_CYCLES_PER_RUN);

        if (readback_count !=
            (mt*N_TILE_COUNT + nt + 1)*OUTPUTS_PER_RUN)
          $fatal(1,
            "FAIL: Readback count MT=%0d NT=%0d got=%0d expected=%0d",
            mt, nt, readback_count,
            (mt*N_TILE_COUNT + nt + 1)*OUTPUTS_PER_RUN);

        emit_tile_csv();
        completed_mn_runs = completed_mn_runs + 1;

        $display("PASS: M-Tile %0d N-Tile %0d channels %0d..%0d",
                 mt, nt, nt*8, nt*8+7);
        $display("PASS: 512 output values checked for M-Tile %0d N-Tile %0d",
                 mt, nt);
        $display("INFO: M-Tile %0d N-Tile %0d compute cycles = %0d",
                 mt, nt, run_compute_cycles);
      end

      completed_m_tiles = completed_m_tiles + 1;
      $display("PASS: M-Tile %0d completed all 8 N-Tiles", mt);
    end

    // ---------------- Final aggregate assertions ----------------

    if (completed_m_tiles != M_TILE_COUNT)
      $fatal(1, "FAIL: completed M tiles=%0d expected=%0d",
             completed_m_tiles, M_TILE_COUNT);

    if (completed_mn_runs != TOTAL_RUNS)
      $fatal(1, "FAIL: completed M/N runs=%0d expected=%0d",
             completed_mn_runs, TOTAL_RUNS);

    if (fmap_total_write_count !=
        M_TILE_COUNT*(6400 + 7*4*10*64))
      $fatal(1,
        "FAIL: total activation writes=%0d expected=%0d",
        fmap_total_write_count,
        M_TILE_COUNT*(6400 + 7*4*10*64));

    if (request_count != TOTAL_RUNS*PAIRS_PER_TILE ||
        line_buffer_read_count != TOTAL_RUNS*PAIRS_PER_TILE ||
        prefetch_trigger_count != TOTAL_RUNS*6 ||
        prefetch_read_count != TOTAL_RUNS*1920 ||
        prefetch_write_count != TOTAL_RUNS*1920)
      $fatal(1,
        "FAIL: Aggregate activation traffic/request counters");

    if (total_clear_count != TOTAL_RUNS*64 ||
        total_feed_pair_count != TOTAL_RUNS*PAIRS_PER_TILE ||
        total_drain_count != TOTAL_RUNS*64*13 ||
        total_launch_count != TOTAL_RUNS*64 ||
        total_kdone_count != TOTAL_RUNS*64 ||
        total_output_write_count != TOTAL_RUNS*64 ||
        readback_count != TOTAL_RUNS*OUTPUTS_PER_RUN)
      $fatal(1, "FAIL: Aggregate compute/output event counts");

    for (int k=0; k<576; k++)
      if (k_hits[k] != TOTAL_RUNS*64)
        $fatal(1,
          "FAIL: K coverage k=%0d hits=%0d expected=%0d",
          k, k_hits[k], TOTAL_RUNS*64);

    if (round_open || computing || busy !== 1'b0 ||
        done !== 1'b0 || any_overflow !== 1'b0)
      $fatal(1, "FAIL: Final DUT state");

    if (compute_cycles != TOTAL_RUNS*COMPUTE_CYCLES_PER_RUN)
      $fatal(1,
        "FAIL: Aggregate compute cycles=%0d expected=%0d",
        compute_cycles, TOTAL_RUNS*COMPUTE_CYCLES_PER_RUN);

    // Final independently counted performance assertions.
    if (starts_seen != TOTAL_RUNS ||
        dones_seen != TOTAL_RUNS ||
        total_weight_load_words != TOTAL_RUNS*K_LEN ||
        total_read_requests != TOTAL_RUNS*OUTPUTS_PER_RUN ||
        total_read_responses != TOTAL_RUNS*OUTPUTS_PER_RUN ||
        total_line_buffer_init_bytes != TOTAL_RUNS*4*10*64)
      $fatal(1, "FAIL: P3 PERF global counter mismatch");

    emit_summary_csv();

    $display("======================================================");
    $display("PASS: P3 full 49 M-Tile x 8 N-Tile (392-Tile) functional verification");
    $display("PASS: %0d/%0d output values match Golden Model",
             TOTAL_RUNS*OUTPUTS_PER_RUN,
             TOTAL_RUNS*OUTPUTS_PER_RUN);
    $display("PASS: M tiles completed = %0d", completed_m_tiles);
    $display("PASS: M/N tile runs completed = %0d", completed_mn_runs);
    $display("PASS: Activation Line Buffer reads = %0d",
             line_buffer_read_count);
    $display("PASS: BRAM prefetch operations = %0d rows",
             prefetch_trigger_count);
    $display("PASS: Activation BRAM pair reads = %0d",
             prefetch_read_count);
    $display("PASS: Line Buffer pair writes = %0d",
             prefetch_write_count);
    $display("PASS: Compute cycles = %0d", compute_cycles);
    $display("PASS: 49 M-Tiles x 8 N-Tiles = 392 runs");
    $display("======================================================");

    finished = 1'b1;
    $finish;
  end

  // Total simulation includes host-side patch/weight loading and output reads.
  initial begin
    repeat (SIM_TIMEOUT_CYCLES) @(posedge clk);
    if (!finished)
      $fatal(1, "TIMEOUT: P3 full 392-Tile performance testbench");
  end

endmodule
