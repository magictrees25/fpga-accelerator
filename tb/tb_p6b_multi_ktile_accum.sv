`timescale 1ns / 1ps

// ============================================================================
// Testbench: tb_p6b_multi_ktile_accum.sv
//
// P6-B1 nine-K-Tile stress test:
//   M = 1, N = 8, K_TOTAL = 576
//   Nine K-Tiles, each containing K = 64 elements (32 paired Reader requests).
//
// Each K-Tile restarts its local K address at zero. The Reader model applies
// active_tile*64 to generate the corresponding global activation/weight data.
// Tile 0 clears the INT32 accumulators; Tiles 1..8 preserve the cumulative
// partial sums while clearing the input/multiply/weight-skew pipeline.
//
// Checks: exact request/FEED/DRAIN counts, clear pulses, all 64 PE accumulators
// and all 8 packed INT32 outputs after every Tile, signed INT8 data, and overflow.
//
// This testbench directly instantiates k_tile_controller_p2 and
// systolic_array_p2. It does not instantiate the M/N controller or BRAM.
// ============================================================================

module tb_p6b_multi_ktile_accum;

  localparam int K_ADDR_WIDTH = 16;
  localparam int K_TILE       = 64;
  localparam int K_TILE_COUNT = 9;
  localparam int K_TOTAL      = K_TILE * K_TILE_COUNT;
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

  // Reader model selects the global K range for the current local K-Tile.
  // active_tile=0..8 maps local K=0..63 to global K=0..575.
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

  // Independent cumulative Golden model, snapshotted after each K-Tile.
  integer signed golden_output_by_tile [0:K_TILE_COUNT-1][0:7];
  integer signed golden_pe_by_tile [0:K_TILE_COUNT-1][0:7][0:7];
  integer signed running_golden [0:7];
  integer signed running_pe [0:7][0:7];

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
  // Each Tile applies active_tile*K_TILE while the K Controller's local address
  // restarts at zero. K_TILE=64 is divisible by 8, so PE row mapping remains
  // aligned across all nine K-Tile boundaries.
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
    input integer tile_index,
    input string  stage_name
  );
    integer signed got;
    integer signed expected;
    begin
      for (int r = 0; r < 8; r++) begin
        for (int c = 0; c < 8; c++) begin
          if (tile_index < 0)
            expected = 0;
          else if (tile_index >= K_TILE_COUNT)
            $fatal(1, "FAIL: invalid Golden Tile index=%0d", tile_index);
          else
            expected = golden_pe_by_tile[tile_index][r][c];

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
    input integer tile_index,
    input string  stage_name
  );
    integer signed got_result;
    integer signed got_live;
    integer signed expected;
    begin
      if (tile_index < 0 || tile_index >= K_TILE_COUNT)
        $fatal(1, "FAIL: invalid output Golden Tile index=%0d", tile_index);

      for (int n = 0; n < 8; n++) begin
        expected = golden_output_by_tile[tile_index][n];

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
    integer global_k;

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

    if ((K_TILE % 8) != 0)
      $fatal(1, "FAIL: K_TILE must be divisible by 8 for PE-row alignment");

    // Initialize cumulative Golden state.
    for (int n = 0; n < 8; n++) begin
      running_golden[n] = 0;
      for (int tile = 0; tile < K_TILE_COUNT; tile++)
        golden_output_by_tile[tile][n] = 0;
    end

    for (int r = 0; r < 8; r++) begin
      for (int n = 0; n < 8; n++) begin
        running_pe[r][n] = 0;
        for (int tile = 0; tile < K_TILE_COUNT; tile++)
          golden_pe_by_tile[tile][r][n] = 0;
      end
    end

    // Build an independent cumulative Golden result.
    // Since K_TILE=64 is divisible by 8, local K-row mapping is aligned
    // with global K-row mapping at every K-Tile boundary.
    for (int tile = 0; tile < K_TILE_COUNT; tile++) begin
      for (int local_k = 0; local_k < K_TILE; local_k++) begin
        global_k = tile * K_TILE + local_k;
        a = activation_value(global_k);

        if (a < 0)
          negative_activation_count++;

        for (int n = 0; n < 8; n++) begin
          w = weight_value(global_k, n);

          if (w < 0)
            negative_weight_count++;

          if ((a * w) < 0)
            negative_product_count++;

          running_golden[n] =
              running_golden[n] + a * w;

          running_pe[local_k % 8][n] =
              running_pe[local_k % 8][n] + a * w;
        end
      end

      // Snapshot the cumulative result at the end of this K-Tile.
      for (int n = 0; n < 8; n++)
        golden_output_by_tile[tile][n] = running_golden[n];

      for (int r = 0; r < 8; r++) begin
        for (int n = 0; n < 8; n++)
          golden_pe_by_tile[tile][r][n] = running_pe[r][n];
      end
    end

    if (negative_activation_count == 0 ||
        negative_weight_count == 0 ||
        negative_product_count == 0)
      $fatal(1, "FAIL: signed stimulus does not cover negative values");

    $display(
      "Golden stimulus: negative A=%0d, negative W=%0d, negative products=%0d",
      negative_activation_count, negative_weight_count, negative_product_count);

    $display(
      "P6-B1 configuration: M=1 N=8 K_TOTAL=%0d K_TILE=%0d K_TILE_COUNT=%0d",
      K_TOTAL, K_TILE, K_TILE_COUNT);

    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    // Run nine consecutive K-Tiles without resetting the DUT.
    for (int tile = 0; tile < K_TILE_COUNT; tile++) begin
      $display(
        "\n--- P6-B1: Tile %0d, global K=%0d..%0d ---",
        tile, tile * K_TILE, (tile + 1) * K_TILE - 1);

      // Only Tile 0 clears the INT32 accumulators.
      // Every Tile clears the input/multiply/weight-skew pipeline.
      launch_tile(tile, (tile == 0));

      wait (clear_pipe === 1'b1);

      if (clear_acc !== (tile == 0))
        $fatal(1,
          "FAIL: Tile %0d clear_acc=%0b expected=%0b",
          tile, clear_acc, (tile == 0));

      @(posedge clk);
      #1;

      // At each boundary, verify that the previous cumulative partial sum
      // survived clear_pipe. Tile 0 must start from zero.
      if (tile == 0)
        check_pe_matrix(-1, "Tile 0 after accumulator clear");
      else
        check_pe_matrix(tile - 1,
          $sformatf("Tile %0d boundary: previous partial sums preserved", tile));

      if (dut_array.valid_cnt !== 5'd0 ||
          dut_array.counting !== 1'b0)
        $fatal(1,
          "FAIL: array valid tracking did not reset at Tile %0d boundary",
          tile);

      wait (done === 1'b1);
      #1;

      check_tile_counters((tile == 0), $sformatf("Tile %0d", tile));
      check_pe_matrix(tile, $sformatf("Tile %0d completed", tile));
      check_outputs(tile, $sformatf("Cumulative result after Tile %0d", tile));

      monitor_enable = 1'b0;
    end

    // Final summary.
    $display("\n==============================================================");
    $display("PASS: P6-B1 NINE-K-TILE INT32 ACCUMULATION");
    $display("  M=1, N=8, K_TOTAL=%0d", K_TOTAL);
    $display("  K-Tiles=%0d, K per Tile=%0d", K_TILE_COUNT, K_TILE);
    $display("  Reader requests=%0d, FEED cycles=%0d",
             K_PAIRS * K_TILE_COUNT, K_PAIRS * K_TILE_COUNT);
    $display("  DRAIN cycles=%0d", DRAIN_CYCLES * K_TILE_COUNT);
    $display("  clear_pipe pulses=%0d, clear_acc pulses=1",
             K_TILE_COUNT);
    $display("  All 64 PE accumulators checked after every K-Tile");
    $display("  All 8 cumulative INT32 outputs checked after every K-Tile");
    $display("  No DUT reset between K-Tiles");
    $display("==============================================================\n");

    $finish;
  end


  // --------------------------------------------------------------------------
  // Watchdog
  // --------------------------------------------------------------------------

  initial begin
    repeat (10000) @(posedge clk);
    $fatal(1, "TIMEOUT: tb_p6b_multi_ktile_accum");
  end


  // --------------------------------------------------------------------------
  // Waveform dump
  // --------------------------------------------------------------------------

  initial begin
    $dumpfile("tb_p6b_multi_ktile_accum.vcd");
    $dumpvars(0, tb_p6b_multi_ktile_accum);
  end

endmodule
