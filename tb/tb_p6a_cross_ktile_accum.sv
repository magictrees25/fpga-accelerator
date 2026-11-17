`timescale 1ns / 1ps

// ============================================================================
// Testbench: tb_p6a_cross_ktile_accum.sv
//
// P6-A extended functional test:
//   M = 1, N = 8, K_TOTAL = 128
//   Two K-Tiles, each containing K = 64 elements (32 paired Reader requests).
//
// Tile 0:
//   clear_acc_on_start = 1
//   Clear accumulator + pipeline, then accumulate K[0:63].
//
// Tile 1:
//   clear_acc_on_start = 0
//   Clear pipeline only, preserve INT32 partial sums, then accumulate K[64:127].
//
// Data:
//   Signed INT8 activations and weights. Deterministic patterns include
//   positive and negative values. Golden values are calculated independently
//   from the RTL datapath.
//
// Checks:
//   1. 32 Reader pair requests per K-Tile, addresses 0,2,...,62.
//   2. 32 FEED cycles and 13 DRAIN cycles per K-Tile.
//   3. One clear_pipe pulse per K-Tile; clear_acc only on Tile 0.
//   4. All 64 PE accumulators survive Tile 1 clear_pipe.
//   5. All 64 PE accumulators and 8 INT32 outputs match Golden.
//   6. MAC input/multiply pipeline registers and weight skew are cleared.
//   7. No unexpected INT32 overflow.
//
// This testbench directly instantiates k_tile_controller_p2 and
// systolic_array_p2. It does not instantiate the M/N controller or BRAM.
// ============================================================================

module tb_p6a_cross_ktile_accum;

  localparam int K_ADDR_WIDTH = 16;
  localparam int K_TILE       = 64;
  localparam int K_TOTAL      = 128;
  localparam int K_PAIRS      = K_TILE / 2;
  localparam int DRAIN_CYCLES = 13;

  logic clk;
  logic rst_n;
  logic start;
  logic clear_acc_on_start;
  logic [K_ADDR_WIDTH-1:0] k_len;

  // Synchronous Reader response interface
  logic [15:0]  act_pair_packed;
  logic [1:0]   act_rsp_valid;
  logic [127:0] wgt_pair_packed;
  logic [1:0]   wgt_rsp_valid;

  // K Controller request interface
  logic pair_req_en;
  logic [K_ADDR_WIDTH-1:0] fetch_k_base;
  logic [K_ADDR_WIDTH-1:0] k_base;

  // K Controller -> systolic array
  logic array_enable;
  logic feed_valid;
  logic clear_acc;
  logic clear_pipe;
  logic valid_reset;
  logic data_valid_in;
  logic [63:0] act_in_packed;
  logic [127:0] wgt_pair_in_packed;

  // Systolic array outputs
  logic [255:0] acc_out_packed;
  logic data_valid_out;
  logic any_overflow;

  // K Controller outputs
  logic busy;
  logic done;
  logic [255:0] result_packed;

  // Reader model selects global K range:
  // active_tile=0 -> global K 0..63
  // active_tile=1 -> global K 64..127
  integer active_tile;

  // Per-Tile protocol counters
  integer request_count;
  integer feed_count;
  integer drain_count;
  integer pipe_clear_count;
  integer acc_clear_count;
  integer valid_reset_count;
  integer first_valid_count;
  integer expected_fetch_base;
  logic monitor_enable;

  // Independent Golden model
  integer signed golden_tile0 [0:7];
  integer signed golden_total [0:7];
  integer signed golden_pe_tile0 [0:7][0:7];
  integer signed golden_pe_total [0:7][0:7];

  integer negative_activation_count;
  integer negative_weight_count;
  integer negative_product_count;


  // --------------------------------------------------------------------------
  // Clock
  // --------------------------------------------------------------------------

  initial clk = 1'b0;
  always #5 clk = ~clk;


  // --------------------------------------------------------------------------
  // Deterministic signed INT8 stimulus
  // --------------------------------------------------------------------------

  function automatic integer signed activation_value(input integer global_k);
    integer tmp;
    begin
      tmp = (global_k * 7 + 3) % 15;
      activation_value = tmp - 7;  // range: -7..+7
    end
  endfunction


  function automatic integer signed weight_value(
    input integer global_k,
    input integer out_channel
  );
    integer tmp;
    begin
      tmp = (global_k * 5 + out_channel * 3 + 1) % 11;
      weight_value = tmp - 5;      // range: -5..+5
    end
  endfunction


  // --------------------------------------------------------------------------
  // DUT: K-Tile controller
  // --------------------------------------------------------------------------

  k_tile_controller_p2 #(
    .K_ADDR_WIDTH (K_ADDR_WIDTH),
    .DRAIN_CYCLES (DRAIN_CYCLES)
  ) dut_kctrl (
    .clk                  (clk),
    .rst_n                (rst_n),
    .start                (start),
    .clear_acc_on_start   (clear_acc_on_start),
    .k_len                (k_len),

    .act_pair_packed      (act_pair_packed),
    .act_rsp_valid        (act_rsp_valid),
    .wgt_pair_packed      (wgt_pair_packed),
    .wgt_rsp_valid        (wgt_rsp_valid),

    .pair_req_en          (pair_req_en),
    .fetch_k_base         (fetch_k_base),
    .k_base               (k_base),

    .array_enable         (array_enable),
    .feed_valid           (feed_valid),
    .clear_acc            (clear_acc),
    .clear_pipe           (clear_pipe),
    .valid_reset          (valid_reset),
    .data_valid_in        (data_valid_in),

    .act_in_packed        (act_in_packed),
    .wgt_pair_in_packed   (wgt_pair_in_packed),

    .acc_out_packed       (acc_out_packed),

    .busy                 (busy),
    .done                 (done),
    .result_packed        (result_packed)
  );


  // --------------------------------------------------------------------------
  // DUT: 8x8 systolic array
  // --------------------------------------------------------------------------

  systolic_array_p2 dut_array (
    .clk                  (clk),
    .rst_n                (rst_n),
    .enable               (array_enable),

    .clear_acc            (clear_acc),
    .clear_pipe           (clear_pipe),
    .valid_reset          (valid_reset),

    .act_in_packed        (act_in_packed),
    .wgt_pair_in_packed   (wgt_pair_in_packed),

    .data_valid_in        (data_valid_in),
    .data_valid_out       (data_valid_out),

    .acc_out_packed       (acc_out_packed),
    .any_overflow         (any_overflow)
  );


  // --------------------------------------------------------------------------
  // One-cycle synchronous Reader model
  //
  // pair_req_en requests two adjacent K elements:
  //   fetch_k_base and fetch_k_base+1.
  //
  // Tile 1 uses a global offset of 64 while the K Controller's local address
  // restarts at zero. Since K_TILE=64 is divisible by 8, PE row mapping remains
  // aligned across the K-Tile boundary.
  // --------------------------------------------------------------------------

  always_ff @(posedge clk or negedge rst_n) begin : reader_model
    integer global_k_base;

    if (!rst_n) begin
      act_pair_packed <= '0;
      act_rsp_valid   <= 2'b00;
      wgt_pair_packed <= '0;
      wgt_rsp_valid   <= 2'b00;
    end
    else begin
      act_rsp_valid <= 2'b00;
      wgt_rsp_valid <= 2'b00;

      if (pair_req_en) begin
        global_k_base = active_tile * K_TILE + int'(fetch_k_base);

        // Two adjacent signed INT8 activations.
        act_pair_packed[7:0]  <= 8'(activation_value(global_k_base));
        act_pair_packed[15:8] <= 8'(activation_value(global_k_base + 1));

        // Stream 0 carries the even K element; stream 1 carries the odd K
        // element. Each stream supplies one signed INT8 weight per output.
        for (int n = 0; n < 8; n++) begin
          wgt_pair_packed[n*8 +: 8] <=
              8'(weight_value(global_k_base, n));

          wgt_pair_packed[(8+n)*8 +: 8] <=
              8'(weight_value(global_k_base + 1, n));
        end

        act_rsp_valid <= 2'b11;
        wgt_rsp_valid <= 2'b11;
      end
    end
  end


  // --------------------------------------------------------------------------
  // Protocol and control monitors
  // --------------------------------------------------------------------------

  always @(posedge clk) begin
    if (!rst_n) begin
      request_count       = 0;
      feed_count          = 0;
      drain_count         = 0;
      pipe_clear_count    = 0;
      acc_clear_count     = 0;
      valid_reset_count   = 0;
      first_valid_count   = 0;
      expected_fetch_base = 0;
    end
    else if (monitor_enable) begin

      if (clear_pipe) begin
        pipe_clear_count = pipe_clear_count + 1;

        if (!array_enable)
          $fatal(1, "FAIL: clear_pipe asserted while array_enable=0");

        if (!valid_reset)
          $fatal(1, "FAIL: valid_reset must follow clear_pipe");
      end

      if (clear_acc) begin
        acc_clear_count = acc_clear_count + 1;

        if (!clear_pipe)
          $fatal(1, "FAIL: clear_acc asserted without clear_pipe");
      end

      if (valid_reset)
        valid_reset_count = valid_reset_count + 1;

      if (pair_req_en) begin
        if (int'(fetch_k_base) != expected_fetch_base) begin
          $fatal(1,
            "FAIL: Tile %0d request address got=%0d expected=%0d",
            active_tile, int'(fetch_k_base), expected_fetch_base);
        end

        if (expected_fetch_base >= K_TILE)
          $fatal(1, "FAIL: Tile %0d requested beyond K_TILE", active_tile);

        request_count       = request_count + 1;
        expected_fetch_base = expected_fetch_base + 2;
      end

      if (feed_valid) begin
        feed_count = feed_count + 1;

        if (!array_enable)
          $fatal(1, "FAIL: FEED asserted while array_enable=0");

        if (act_rsp_valid !== 2'b11 ||
            wgt_rsp_valid !== 2'b11)
          $fatal(1,
            "FAIL: missing synchronous Reader response at local K=%0d",
            int'(k_base));

        if (data_valid_in)
          first_valid_count = first_valid_count + 1;
      end

      // With this controller, array_enable=1, feed_valid=0 and
      // clear_pipe=0 identifies the DRAIN state.
      if (array_enable && !feed_valid && !clear_pipe)
        drain_count = drain_count + 1;

    end
  end


  // --------------------------------------------------------------------------
  // Generated checks: every PE input/multiply pipeline register must clear.
  // These checks run one time unit after the active clock edge so nonblocking
  // assignments in mac_unit have taken effect.
  // --------------------------------------------------------------------------

  for (genvar r = 0; r < 8; r++) begin : gen_check_pe_pipe_row
    for (genvar c = 0; c < 8; c++) begin : gen_check_pe_pipe_col
      always @(posedge clk) begin
        if (rst_n && clear_pipe) begin
          #1;
          if (dut_array.gen_row[r].gen_col[c].u_pe.act_reg !== 8'sd0)
            $fatal(1,
              "FAIL: PE[%0d][%0d] act_reg not cleared", r, c);

          if (dut_array.gen_row[r].gen_col[c].u_pe.wgt_reg !== 8'sd0)
            $fatal(1,
              "FAIL: PE[%0d][%0d] wgt_reg not cleared", r, c);

          if (dut_array.gen_row[r].gen_col[c].u_pe.mul_reg !== 16'sd0)
            $fatal(1,
              "FAIL: PE[%0d][%0d] mul_reg not cleared", r, c);
        end
      end
    end
  end


  // Generated checks: every registered weight-skew element must clear.
  for (genvar p = 0; p < 2; p++) begin : gen_check_weight_stream
    for (genvar c = 1; c < 8; c++) begin : gen_check_weight_column
      for (genvar s = 0; s < c; s++) begin : gen_check_weight_skew
        always @(posedge clk) begin
          if (rst_n && clear_pipe) begin
            #1;
            if (dut_array.gen_stream[p].gen_column[c].gen_delayed.skew[s]
                !== 8'sd0)
              $fatal(1,
                "FAIL: weight skew stream=%0d col=%0d stage=%0d not cleared",
                p, c, s);
          end
        end
      end
    end
  end


  // --------------------------------------------------------------------------
  // Helper: launch one K-Tile
  // --------------------------------------------------------------------------

  task automatic launch_tile(
    input integer tile_id,
    input logic   clear_acc_flag
  );
    begin
      // The previous transaction must have left S_DONE and returned to IDLE.
      wait (done === 1'b0);
      @(negedge clk);

      active_tile        = tile_id;
      clear_acc_on_start = clear_acc_flag;
      k_len              = K_TILE[K_ADDR_WIDTH-1:0];

      request_count       = 0;
      feed_count          = 0;
      drain_count         = 0;
      pipe_clear_count    = 0;
      acc_clear_count     = 0;
      valid_reset_count   = 0;
      first_valid_count   = 0;
      expected_fetch_base = 0;

      monitor_enable = 1'b1;
      start          = 1'b1;

      @(negedge clk);
      start = 1'b0;
    end
  endtask


  // --------------------------------------------------------------------------
  // Helper: check all 64 PE INT32 accumulators against independent Golden.
  // k_limit=64 checks Tile 0; k_limit=128 checks Tile 0 + Tile 1.
  // --------------------------------------------------------------------------

  task automatic check_pe_matrix(
    input integer k_limit,
    input string  stage_name
  );
    integer signed got;
    integer signed expected;
    begin
      for (int r = 0; r < 8; r++) begin
        for (int c = 0; c < 8; c++) begin
          // k_limit=0 is used immediately after Tile 0 clear:
          // every PE accumulator must be zero.
          if (k_limit == 0)
            expected = 0;
          else if (k_limit == K_TILE)
            expected = golden_pe_tile0[r][c];
          else if (k_limit == K_TOTAL)
            expected = golden_pe_total[r][c];
          else
            $fatal(1, "FAIL: unsupported PE Golden limit=%0d", k_limit);

          got = int'($signed(dut_array.pe_acc[r][c]));

          if (got !== expected) begin
            $fatal(1,
              "FAIL: %s PE[%0d][%0d] got=%0d expected=%0d",
              stage_name, r, c, got, expected);
          end
        end
      end

      $display("PASS: %s, all 64 PE accumulators match Golden",
               stage_name);
    end
  endtask


  // --------------------------------------------------------------------------
  // Helper: check 8 packed INT32 outputs and overflow status.
  // check_tile0=1 checks the first 64 K elements; otherwise all 128.
  // --------------------------------------------------------------------------

  task automatic check_outputs(
    input logic  check_tile0,
    input string stage_name
  );
    integer signed got_result;
    integer signed got_live;
    integer signed expected;
    begin
      for (int n = 0; n < 8; n++) begin
        if (check_tile0)
          expected = golden_tile0[n];
        else
          expected = golden_total[n];

        got_result = int'($signed(result_packed[n*32 +: 32]));
        got_live   = int'($signed(acc_out_packed[n*32 +: 32]));

        if (got_result !== expected) begin
          $fatal(1,
            "FAIL: %s result[%0d] got=%0d expected=%0d",
            stage_name, n, got_result, expected);
        end

        if (got_live !== expected) begin
          $fatal(1,
            "FAIL: %s acc_out[%0d] got=%0d expected=%0d",
            stage_name, n, got_live, expected);
        end
      end

      if (any_overflow !== 1'b0)
        $fatal(1, "FAIL: %s unexpected accumulator overflow", stage_name);

      $display("PASS: %s, all 8 INT32 outputs match Golden",
               stage_name);
    end
  endtask


  // --------------------------------------------------------------------------
  // Helper: verify exact per-Tile controller activity.
  // --------------------------------------------------------------------------

  task automatic check_tile_counters(
    input integer expected_clear_acc,
    input string  stage_name
  );
    begin
      if (request_count != K_PAIRS)
        $fatal(1,
          "FAIL: %s Reader requests=%0d expected=%0d",
          stage_name, request_count, K_PAIRS);

      if (feed_count != K_PAIRS)
        $fatal(1,
          "FAIL: %s FEED cycles=%0d expected=%0d",
          stage_name, feed_count, K_PAIRS);

      if (drain_count != DRAIN_CYCLES)
        $fatal(1,
          "FAIL: %s DRAIN cycles=%0d expected=%0d",
          stage_name, drain_count, DRAIN_CYCLES);

      if (pipe_clear_count != 1)
        $fatal(1,
          "FAIL: %s clear_pipe pulses=%0d expected=1",
          stage_name, pipe_clear_count);

      if (acc_clear_count != expected_clear_acc)
        $fatal(1,
          "FAIL: %s clear_acc pulses=%0d expected=%0d",
          stage_name, acc_clear_count, expected_clear_acc);

      if (valid_reset_count != 1)
        $fatal(1,
          "FAIL: %s valid_reset pulses=%0d expected=1",
          stage_name, valid_reset_count);

      if (first_valid_count != 1)
        $fatal(1,
          "FAIL: %s data_valid_in first pulses=%0d expected=1",
          stage_name, first_valid_count);

      if (expected_fetch_base != K_TILE)
        $fatal(1,
          "FAIL: %s final requested K address=%0d expected=%0d",
          stage_name, expected_fetch_base - 2, K_TILE - 2);

      $display(
        "PASS: %s counters: requests=%0d FEED=%0d DRAIN=%0d clear_pipe=%0d clear_acc=%0d",
        stage_name, request_count, feed_count, drain_count,
        pipe_clear_count, acc_clear_count);
    end
  endtask


  // --------------------------------------------------------------------------
  // Main test
  // --------------------------------------------------------------------------

  initial begin : main_test
    integer signed a;
    integer signed w;

    rst_n              = 1'b0;
    start              = 1'b0;
    clear_acc_on_start = 1'b0;
    k_len              = K_TILE[K_ADDR_WIDTH-1:0];

    // Reader response signals are driven exclusively by reader_model.

    active_tile        = 0;
    monitor_enable     = 1'b0;

    request_count       = 0;
    feed_count          = 0;
    drain_count         = 0;
    pipe_clear_count    = 0;
    acc_clear_count     = 0;
    valid_reset_count   = 0;
    first_valid_count   = 0;
    expected_fetch_base = 0;

    negative_activation_count = 0;
    negative_weight_count     = 0;
    negative_product_count    = 0;

    // Build independent Golden values for all 128 K elements.
    for (int n = 0; n < 8; n++) begin
      golden_tile0[n] = 0;
      golden_total[n] = 0;
    end

    for (int r = 0; r < 8; r++) begin
      for (int n = 0; n < 8; n++) begin
        golden_pe_tile0[r][n] = 0;
        golden_pe_total[r][n] = 0;
      end
    end

    for (int k = 0; k < K_TOTAL; k++) begin
      a = activation_value(k);

      if (a < 0)
        negative_activation_count++;

      for (int n = 0; n < 8; n++) begin
        w = weight_value(k, n);

        if (w < 0)
          negative_weight_count++;

        if ((a * w) < 0)
          negative_product_count++;

        golden_total[n] = golden_total[n] + a * w;
        golden_pe_total[k % 8][n] =
            golden_pe_total[k % 8][n] + a * w;

        if (k < K_TILE) begin
          golden_tile0[n] = golden_tile0[n] + a * w;
          golden_pe_tile0[k % 8][n] =
              golden_pe_tile0[k % 8][n] + a * w;
        end
      end
    end

    if (negative_activation_count == 0 ||
        negative_weight_count == 0 ||
        negative_product_count == 0)
      $fatal(1, "FAIL: signed stimulus does not cover negative values");

    $display("Golden stimulus: negative A=%0d, negative W=%0d, negative products=%0d",
      negative_activation_count, negative_weight_count, negative_product_count);

    repeat (4) @(negedge clk);
    rst_n = 1'b1;


    // ========================================================================
    // Tile 0: K = 0..63, clear accumulator and pipeline
    // ========================================================================

    $display("\n--- P6-A EXTENDED TEST: Tile 0, K=0..63 ---");

    launch_tile(0, 1'b1);

    wait (clear_pipe === 1'b1);
    if (clear_acc !== 1'b1)
      $fatal(1, "FAIL: Tile 0 must assert clear_acc with clear_pipe");

    @(posedge clk);
    #1;

    check_pe_matrix(0, "Tile 0 after accumulator clear");

    if (dut_array.valid_cnt !== 5'd0 || dut_array.counting !== 1'b0)
      $fatal(1, "FAIL: array valid tracking did not reset at Tile 0 boundary");

    wait (done === 1'b1);
    #1;

    check_tile_counters(1, "Tile 0");
    check_pe_matrix(K_TILE, "Tile 0 completed");
    check_outputs(1'b1, "Tile 0 partial result");

    monitor_enable = 1'b0;


    // ========================================================================
    // Tile 1: K = 64..127, clear pipeline only and preserve accumulators
    // ========================================================================

    $display("\n--- P6-A EXTENDED TEST: Tile 1, K=64..127 ---");

    launch_tile(1, 1'b0);

    wait (clear_pipe === 1'b1);
    if (clear_acc !== 1'b0)
      $fatal(1, "FAIL: Tile 1 must not assert clear_acc");

    @(posedge clk);
    #1;

    // Critical P6-A assertion: all 64 partial sums from Tile 0 must survive.
    check_pe_matrix(K_TILE, "Tile 1 after clear_pipe (partial sums preserved)");

    // Pipeline and weight-skew registers are checked by generated assertions.
    if (dut_array.valid_cnt !== 5'd0 || dut_array.counting !== 1'b0)
      $fatal(1, "FAIL: array valid tracking did not reset at Tile 1 boundary");

    wait (done === 1'b1);
    #1;

    check_tile_counters(0, "Tile 1");
    check_pe_matrix(K_TOTAL, "Tile 1 completed");
    check_outputs(1'b0, "Final K=128 result");

    monitor_enable = 1'b0;


    // ========================================================================
    // Final summary
    // ========================================================================

    $display("\n==============================================================");
    $display("PASS: P6-A CROSS-K-TILE INT32 ACCUMULATION");
    $display("  M=1, N=8, K_TOTAL=128");
    $display("  K-Tile 0: K=64, clear accumulator + pipeline");
    $display("  K-Tile 1: K=64, clear pipeline only");
    $display("  64/64 PE accumulators match independent Golden");
    $display("  8/8 INT32 outputs match independent Golden");
    $display("  Signed INT8 activation/weight patterns exercised");
    $display("  Each Tile: 32 requests, 32 FEED cycles, 13 DRAIN cycles");
    $display("  Tile 1 clear_pipe preserved every Tile 0 partial sum");
    $display("==============================================================\n");

    $finish;
  end


  // --------------------------------------------------------------------------
  // Watchdog
  // --------------------------------------------------------------------------

  initial begin
    repeat (5000) @(posedge clk);
    $fatal(1, "TIMEOUT: tb_p6a_cross_ktile_accum");
  end


  // --------------------------------------------------------------------------
  // Waveform dump
  // --------------------------------------------------------------------------

  initial begin
    $dumpfile("tb_p6a_cross_ktile_accum.vcd");
    $dumpvars(0, tb_p6a_cross_ktile_accum);
  end

endmodule
