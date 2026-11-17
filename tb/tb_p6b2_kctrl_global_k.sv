`timescale 1ns / 1ps

// P6-B2-3: K Controller + unchanged P2 array, synchronous Reader MODEL.
// The model uses fetch_k_base directly; it NEVER supplies a hidden Tile offset.
// This is not the later real-BRAM/GEMM integration test (P6-B2-5).
// Sources: pkg_accelerator.sv, mac_unit.sv, systolic_array_p2.sv,
//          k_tile_controller_p2.sv, this file.
// Simulation Top: tb_p6b2_kctrl_global_k
module tb_p6b2_kctrl_global_k;
  logic clk = 1'b0;
  logic global_finished, legacy_finished;
  always #5 clk = ~clk;

  p6b2_kctrl_global_k_case #(.GLOBAL_MODE(1'b1)) u_global (
    .clk(clk), .finished(global_finished)
  );
  p6b2_kctrl_global_k_case #(.GLOBAL_MODE(1'b0)) u_legacy (
    .clk(clk), .finished(legacy_finished)
  );

  initial begin
    wait (global_finished === 1'b1 && legacy_finished === 1'b1);
    $display("PASS: P6-B2-3 GLOBAL K ADDRESS AND INT32 ACCUMULATION");
    $display("  Main global case: 9 Tiles, 288 requests, 288 FEED, 117 DRAIN");
    $display("  Main global case: 9 pipeline clears, 1 accumulator clear, 72/72 outputs");
    $display("  Extra global cases: non-8-aligned base, K=2/10, last address 65535");
    $display("  Legacy mode: 9 Tiles with unconnected-value base input ignored");
    $display("  All requests, local K, paired data, 64 PE sums and snapshots checked");
    $finish;
  end

  initial begin
    repeat (10000) @(posedge clk);
    $fatal(1, "FAIL: P6-B2-3 global watchdog timeout");
  end
endmodule

module p6b2_kctrl_global_k_case #(
  parameter bit GLOBAL_MODE = 1'b1
)(
  input logic clk,
  output logic finished
);
  localparam int K_ADDR_WIDTH = 16;
  localparam int DRAIN_CYCLES = 13;
  logic rst_n, start, clear_acc_on_start;
  logic [15:0] k_len, cfg_base;
  wire [15:0] k_tile_base = GLOBAL_MODE ? cfg_base : 16'hzzzz;
  logic [15:0] act_pair_packed;
  logic [1:0] act_rsp_valid, wgt_rsp_valid;
  logic [127:0] wgt_pair_packed;
  logic pair_req_en;
  logic [15:0] fetch_k_base, k_base;
  logic array_enable, feed_valid, clear_acc, clear_pipe, valid_reset;
  logic data_valid_in, data_valid_out, any_overflow, busy, done;
  logic [63:0] act_in_packed;
  logic [127:0] wgt_pair_in_packed;
  logic [255:0] acc_out_packed, result_packed;

  integer response_tag;
  bit monitor_enable;
  integer active_id, expected_base, expected_length;
  bit expected_clear;
  integer requests, feeds, drains, pipe_clears, acc_clears, first_valids;
  integer total_requests, total_feeds, total_drains;
  integer total_pipe_clears, total_acc_clears, total_starts, total_dones;
  integer checked_outputs, completed_tiles;
  integer signed golden [0:7];
  integer signed golden_pe [0:7][0:7];
  integer signed before_pe [0:7][0:7];

  // Independent scalar data definition: full signed INT8 range, both signs.
  function automatic integer signed activation_value(input integer k);
    activation_value = ((k * 37 + 19) % 256) - 128;
  endfunction
  function automatic integer signed weight_value(input integer k, input integer n);
    weight_value = ((k * 29 + n * 43 + 7) % 251) - 125;
  endfunction

  k_tile_controller_p2 #(
    .K_ADDR_WIDTH(K_ADDR_WIDTH), .DRAIN_CYCLES(DRAIN_CYCLES),
    .USE_K_TILE_BASE(GLOBAL_MODE)
  ) dut_kctrl (
    .clk(clk), .rst_n(rst_n), .start(start),
    .clear_acc_on_start(clear_acc_on_start), .k_len(k_len),
    .k_tile_base(k_tile_base),
    .act_pair_packed(act_pair_packed), .act_rsp_valid(act_rsp_valid),
    .wgt_pair_packed(wgt_pair_packed), .wgt_rsp_valid(wgt_rsp_valid),
    .pair_req_en(pair_req_en), .fetch_k_base(fetch_k_base), .k_base(k_base),
    .array_enable(array_enable), .feed_valid(feed_valid),
    .clear_acc(clear_acc), .clear_pipe(clear_pipe), .valid_reset(valid_reset),
    .data_valid_in(data_valid_in), .act_in_packed(act_in_packed),
    .wgt_pair_in_packed(wgt_pair_in_packed), .acc_out_packed(acc_out_packed),
    .busy(busy), .done(done), .result_packed(result_packed)
  );

  systolic_array_p2 dut_array (
    .clk(clk), .rst_n(rst_n), .enable(array_enable),
    .clear_acc(clear_acc), .clear_pipe(clear_pipe), .valid_reset(valid_reset),
    .act_in_packed(act_in_packed), .wgt_pair_in_packed(wgt_pair_in_packed),
    .data_valid_in(data_valid_in), .data_valid_out(data_valid_out),
    .acc_out_packed(acc_out_packed), .any_overflow(any_overflow)
  );

  // One-cycle, no-backpressure Reader model. The ONLY data address source is
  // the actual DUT request. Scoreboards below use independently counted K.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      act_pair_packed <= '0;
      wgt_pair_packed <= '0;
      act_rsp_valid <= '0;
      wgt_rsp_valid <= '0;
      response_tag <= -1;
    end else begin
      act_rsp_valid <= pair_req_en ? 2'b11 : 2'b00;
      wgt_rsp_valid <= pair_req_en ? 2'b11 : 2'b00;
      if (pair_req_en) begin
        response_tag <= int'(fetch_k_base);
        act_pair_packed[7:0] <= 8'(activation_value(int'(fetch_k_base)));
        act_pair_packed[15:8] <= 8'(activation_value(int'(fetch_k_base) + 1));
        for (int n = 0; n < 8; n++) begin
          wgt_pair_packed[n*8 +: 8] <= 8'(weight_value(int'(fetch_k_base), n));
          wgt_pair_packed[(8+n)*8 +: 8] <=
              8'(weight_value(int'(fetch_k_base) + 1, n));
        end
      end
    end
  end

  // Observe pre-NBA signals, exactly those sampled by the array and Reader.
  always @(posedge clk) begin : monitor
    integer local_k, global_k;
    logic [63:0] expected_act;
    logic [127:0] expected_weights;
    if (rst_n) begin
      if ($isunknown({pair_req_en, feed_valid, clear_pipe, clear_acc,
                      valid_reset, array_enable, busy, done, data_valid_in}))
        $fatal(1, "FAIL mode=%0d: unknown controller output", GLOBAL_MODE);
      if (start) total_starts++;
      if (done) total_dones++;
      if (any_overflow !== 1'b0)
        $fatal(1, "FAIL mode=%0d: unexpected overflow", GLOBAL_MODE);

      if (monitor_enable) begin
        if (valid_reset !== clear_pipe)
          $fatal(1, "FAIL: valid_reset must follow clear_pipe");
        if (busy !== array_enable)
          $fatal(1, "FAIL: busy/array_enable mismatch");
        if (clear_acc && !clear_pipe)
          $fatal(1, "FAIL: accumulator clear outside pipeline clear");

        if (clear_pipe) begin
          if (clear_acc !== expected_clear || feed_valid || !pair_req_en ||
              k_base !== 16'd0 || act_in_packed !== 64'd0 ||
              wgt_pair_in_packed !== 128'd0)
            $fatal(1, "FAIL mode=%0d tile=%0d: CLEAR protocol", GLOBAL_MODE, active_id);
          pipe_clears++;
          total_pipe_clears++;
        end
        if (clear_acc) begin
          acc_clears++;
          total_acc_clears++;
        end

        if (pair_req_en) begin
          if (!(clear_pipe || feed_valid) || requests >= expected_length/2 ||
              fetch_k_base !== 16'(expected_base + 2*requests))
            $fatal(1,
              "FAIL mode=%0d tile=%0d request=%0d got=%0d expected=%0d",
              GLOBAL_MODE, active_id, requests, fetch_k_base, expected_base + 2*requests);
          requests++;
          total_requests++;
        end

        if (feed_valid) begin
          local_k = 2*feeds;
          global_k = expected_base + local_k;
          if (feeds >= expected_length/2 || k_base !== 16'(local_k) ||
              act_rsp_valid !== 2'b11 || wgt_rsp_valid !== 2'b11 ||
              response_tag !== global_k || clear_pipe || !array_enable)
            $fatal(1, "FAIL mode=%0d tile=%0d: local K / response tag / FEED", GLOBAL_MODE, active_id);
          if (pair_req_en !== (local_k + 2 < expected_length))
            $fatal(1, "FAIL: missing lookahead or extra final-pair request");
          if (data_valid_in !== (local_k == 0))
            $fatal(1, "FAIL: first-valid marker must use local K=0");

          expected_act = '0;
          expected_act[(local_k%8)*8 +: 8] = 8'(activation_value(global_k));
          expected_act[((local_k+1)%8)*8 +: 8] = 8'(activation_value(global_k+1));
          for (int n = 0; n < 8; n++) begin
            expected_weights[n*8 +: 8] = 8'(weight_value(global_k, n));
            expected_weights[(8+n)*8 +: 8] = 8'(weight_value(global_k+1, n));
          end
          if (act_pair_packed !== {8'(activation_value(global_k+1)),
                                   8'(activation_value(global_k))} ||
              wgt_pair_packed !== expected_weights ||
              act_in_packed !== expected_act || wgt_pair_in_packed !== expected_weights)
            $fatal(1, "FAIL mode=%0d tile=%0d localK=%0d: aligned signed data / PE row", GLOBAL_MODE, active_id, local_k);
          if (data_valid_in) first_valids++;
          feeds++;
          total_feeds++;
        end

        if (array_enable && !feed_valid && !clear_pipe) begin
          if (feeds != expected_length/2 || pair_req_en || data_valid_in ||
              act_in_packed !== 64'd0 || wgt_pair_in_packed !== 128'd0 ||
              k_base !== 16'(expected_length-2))
            $fatal(1, "FAIL: DRAIN data/control or local K changed");
          drains++;
          total_drains++;
        end
      end else if (pair_req_en || array_enable || done)
        $fatal(1, "FAIL: unexpected activity outside a launched transaction");
    end
  end

  // Check each physical PE after the clear edge, including preservation of
  // the preceding Tile's sum. Existing P6 hierarchy names are used verbatim.
  for (genvar r = 0; r < 8; r++) begin : check_row
    for (genvar n = 0; n < 8; n++) begin : check_col
      always @(posedge clk) begin
        if (rst_n && clear_pipe) begin
          #1;
          if (dut_array.gen_row[r].gen_col[n].u_pe.act_reg !== 8'sd0 ||
              dut_array.gen_row[r].gen_col[n].u_pe.wgt_reg !== 8'sd0 ||
              dut_array.gen_row[r].gen_col[n].u_pe.mul_reg !== 16'sd0 ||
              dut_array.pe_acc[r][n] !== before_pe[r][n])
            $fatal(1, "FAIL mode=%0d tile=%0d: pipeline clear / preserved PE[%0d][%0d]",
                   GLOBAL_MODE, active_id, r, n);
        end
      end
    end
  end
  for (genvar p = 0; p < 2; p++) begin : check_stream
    for (genvar n = 1; n < 8; n++) begin : check_weight_col
      for (genvar s = 0; s < n; s++) begin : check_skew
        always @(posedge clk) begin
          if (rst_n && clear_pipe) begin
            #1;
            if (dut_array.gen_stream[p].gen_column[n].gen_delayed.skew[s] !== 8'sd0)
              $fatal(1, "FAIL: weight skew did not clear");
          end
        end
      end
    end
  end

  task automatic run_tile(input integer id, input integer base,
                          input integer length, input bit new_sum);
    integer waited;
    integer signed a, w;
    logic [255:0] held_result;
    begin
      @(negedge clk);
      if (busy !== 1'b0 || done !== 1'b0)
        $fatal(1, "FAIL: previous Tile did not return to IDLE");
      active_id = id;
      expected_base = GLOBAL_MODE ? base : 0;
      expected_length = length;
      expected_clear = new_sum;
      requests = 0; feeds = 0; drains = 0;
      pipe_clears = 0; acc_clears = 0; first_valids = 0;

      // Scalar Golden uses the test scenario, never DUT requests or outputs.
      // The per-PE Golden separately distributes each LOCAL K to a row.
      for (int n = 0; n < 8; n++) begin
        if (new_sum) golden[n] = 0;
        for (int r = 0; r < 8; r++) begin
          if (new_sum) golden_pe[r][n] = 0;
          before_pe[r][n] = golden_pe[r][n];
        end
      end
      for (int k = 0; k < length; k++) begin
        a = activation_value(expected_base + k);
        for (int n = 0; n < 8; n++) begin
          w = weight_value(expected_base + k, n);
          golden[n] += a*w;
          golden_pe[k%8][n] += a*w;
        end
      end

      cfg_base = 16'(base);
      k_len = 16'(length);
      clear_acc_on_start = new_sum;
      monitor_enable = 1'b1;
      start = 1'b1;
      @(negedge clk);
      start = 1'b0;
      // Deliberately perturb live configuration after accepted start. CLEAR's
      // first request and all later requests must use the saved configuration.
      cfg_base = 16'hffff;
      k_len = 16'd2;
      clear_acc_on_start = !new_sum;

      if (clear_pipe !== 1'b1 || clear_acc !== new_sum)
        $fatal(1, "FAIL: CLEAR/accumulator decision was not latched at start");
      @(posedge clk);
      #1;
      if (dut_array.valid_cnt !== 5'd0 || dut_array.counting !== 1'b0)
        $fatal(1, "FAIL: valid tracking did not clear");

      waited = 0;
      while (done !== 1'b1 && waited < 1000) begin
        @(posedge clk);
        #1;
        waited++;
      end
      if (done !== 1'b1)
        $fatal(1, "FAIL mode=%0d tile=%0d: done timeout", GLOBAL_MODE, id);
      if (requests != length/2 || feeds != length/2 || drains != DRAIN_CYCLES ||
          pipe_clears != 1 || acc_clears != int'(new_sum) || first_valids != 1)
        $fatal(1,
          "FAIL mode=%0d tile=%0d: counts req=%0d feed=%0d drain=%0d pipe=%0d acc=%0d first=%0d",
          GLOBAL_MODE, id, requests, feeds, drains, pipe_clears, acc_clears, first_valids);
      for (int n = 0; n < 8; n++) begin
        if ($signed(result_packed[n*32 +: 32]) !== golden[n] ||
            $signed(acc_out_packed[n*32 +: 32]) !== golden[n])
          $fatal(1, "FAIL mode=%0d tile=%0d N=%0d got=%0d expected=%0d",
                 GLOBAL_MODE, id, n, $signed(result_packed[n*32 +: 32]), golden[n]);
        for (int r = 0; r < 8; r++) begin
          if (dut_array.pe_acc[r][n] !== golden_pe[r][n])
            $fatal(1, "FAIL mode=%0d tile=%0d PE[%0d][%0d]", GLOBAL_MODE, id, r, n);
        end
        checked_outputs++;
      end
      held_result = result_packed;
      // Sample the complete done pulse, then check its deassertion and IDLE.
      @(posedge clk);
      #1;
      completed_tiles++;
      if (done !== 1'b0 || busy !== 1'b0 || total_dones != completed_tiles)
        $fatal(1, "FAIL: done pulse count/width or IDLE return");
      monitor_enable = 1'b0;
      repeat (2) begin
        @(posedge clk);
        #1;
        if (result_packed !== held_result || acc_out_packed !== held_result)
          $fatal(1, "FAIL: partial sums or result snapshot changed while idle");
      end
      $display("TILE PASS: global_mode=%0d id=%0d base=%0d K=%0d req/feed=%0d drain=%0d outputs=8/8",
               GLOBAL_MODE, id, expected_base, length, requests, drains);
    end
  endtask

  initial begin
    finished = 1'b0; rst_n = 1'b0; start = 1'b0;
    cfg_base = '0; k_len = 16'd64; clear_acc_on_start = 1'b0;
    monitor_enable = 1'b0;
    active_id = -1; expected_base = 0; expected_length = 64; expected_clear = 0;
    requests = 0; feeds = 0; drains = 0;
    pipe_clears = 0; acc_clears = 0; first_valids = 0;
    total_requests = 0; total_feeds = 0; total_drains = 0;
    total_pipe_clears = 0; total_acc_clears = 0;
    total_starts = 0; total_dones = 0; completed_tiles = 0; checked_outputs = 0;
    for (int n = 0; n < 8; n++) begin
      golden[n] = 0;
      for (int r = 0; r < 8; r++) begin
        golden_pe[r][n] = 0;
        before_pe[r][n] = 0;
      end
    end
    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    // One reset only. Main acceptance case: nine consecutive 64-element Tiles.
    for (int tile = 0; tile < 9; tile++)
      run_tile(tile, tile*64, 64, (tile == 0));
    if (total_requests != 288 || total_feeds != 288 || total_drains != 117 ||
        total_pipe_clears != 9 || total_acc_clears != 1 || total_starts != 9 ||
        total_dones != 9 || completed_tiles != 9 || checked_outputs != 72)
      $fatal(1, "FAIL: nine-Tile aggregate counts");
    $display("CASE PASS: global_mode=%0d nine Tiles, 288 requests/FEED, 117 DRAIN, 72/72 outputs", GLOBAL_MODE);

    if (GLOBAL_MODE) begin
      // Distinguish local-row mapping from global-row mapping, check single
      // pair, partial last row group, continuation, and the address ceiling.
      run_tile(9, 6, 2, 1'b1);
      run_tile(10, 14, 10, 1'b0);
      run_tile(11, 65534, 2, 1'b1);
      if (total_requests != 295 || total_feeds != 295 || total_drains != 156 ||
          total_pipe_clears != 12 || total_acc_clears != 3 || total_starts != 12 ||
          total_dones != 12 || completed_tiles != 12 || checked_outputs != 96)
        $fatal(1, "FAIL: additional global boundary-case counts");
    end
    finished = 1'b1;
  end
endmodule
