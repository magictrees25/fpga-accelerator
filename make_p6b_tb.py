from pathlib import Path

# Run this script from the repository root.
# It copies the known-good P6-A testbench into a new P6-B1 testbench,
# then generalizes the test from 2 K-Tiles to 9 K-Tiles.
src_path = Path("tb") / "tb_p6a_cross_ktile_accum.sv"
out_path = Path("tb") / "tb_p6b_multi_ktile_accum.sv"

if not src_path.exists():
    raise SystemExit(
        f"Cannot find {src_path}. Run this script from the repository root."
    )
if out_path.exists():
    raise SystemExit(
        f"{out_path} already exists. Back it up or remove it before rerunning."
    )

text = src_path.read_text(encoding="utf-8")

def replace_once(old: str, new: str, label: str) -> None:
    global text
    if old not in text:
        raise SystemExit(f"Could not locate expected source block: {label}")
    text = text.replace(old, new, 1)

def replace_task(task_name: str, replacement: str) -> None:
    global text
    start_marker = f"  task automatic {task_name}("
    start = text.find(start_marker)
    if start < 0:
        raise SystemExit(f"Could not find task {task_name}")
    end_marker = "  endtask"
    end = text.find(end_marker, start)
    if end < 0:
        raise SystemExit(f"Could not find endtask for {task_name}")
    end += len(end_marker)
    text = text[:start] + replacement.rstrip() + text[end:]

# New module and local parameters.
replace_once(
    "module tb_p6a_cross_ktile_accum;",
    "module tb_p6b_multi_ktile_accum;",
    "module name",
)

# Replace the P6-A two-Tile header with the P6-B1 scope.
header_start_marker = "// P6-A extended functional test:\n"
header_end_marker = "// This testbench directly instantiates k_tile_controller_p2 and\n"
header_start = text.find(header_start_marker)
header_end = text.find(header_end_marker, header_start)
if header_start < 0 or header_end < 0:
    raise SystemExit("Could not locate the P6-A testbench header")
new_header = (
    "// P6-B1 nine-K-Tile stress test:\n"
    "//   M = 1, N = 8, K_TOTAL = 576\n"
    "//   Nine K-Tiles, each containing K = 64 elements (32 paired Reader requests).\n"
    "//\n"
    "// Each K-Tile restarts its local K address at zero. The Reader model applies\n"
    "// active_tile*64 to generate the corresponding global activation/weight data.\n"
    "// Tile 0 clears the INT32 accumulators; Tiles 1..8 preserve the cumulative\n"
    "// partial sums while clearing the input/multiply/weight-skew pipeline.\n"
    "//\n"
    "// Checks: exact request/FEED/DRAIN counts, clear pulses, all 64 PE accumulators\n"
    "// and all 8 packed INT32 outputs after every Tile, signed INT8 data, and overflow.\n"
    "//\n"
)
text = text[:header_start] + new_header + text[header_end:]
old_constants = (
    "  localparam int K_ADDR_WIDTH = 16;\n"
    "  localparam int K_TILE       = 64;\n"
    "  localparam int K_TOTAL      = 128;\n"
    "  localparam int K_PAIRS      = K_TILE / 2;\n"
    "  localparam int DRAIN_CYCLES = 13;"
)
new_constants = (
    "  localparam int K_ADDR_WIDTH = 16;\n"
    "  localparam int K_TILE       = 64;\n"
    "  localparam int K_TILE_COUNT = 9;\n"
    "  localparam int K_TOTAL      = K_TILE * K_TILE_COUNT;\n"
    "  localparam int K_PAIRS      = K_TILE / 2;\n"
    "  localparam int DRAIN_CYCLES = 13;"
)
replace_once(old_constants, new_constants, "local parameters")

# Replace the two-Tile Golden arrays with cumulative snapshots after every Tile.
start_marker = "  // Independent Golden model\n"
end_marker = "  integer negative_activation_count;\n"
start = text.find(start_marker)
end = text.find(end_marker, start)
if start < 0 or end < 0:
    raise SystemExit("Could not locate the P6-A Golden-model declarations")
new_golden_declarations = (
    "  // Independent cumulative Golden model, snapshotted after each K-Tile.\n"
    "  integer signed golden_output_by_tile [0:K_TILE_COUNT-1][0:7];\n"
    "  integer signed golden_pe_by_tile [0:K_TILE_COUNT-1][0:7][0:7];\n"
    "  integer signed running_golden [0:7];\n"
    "  integer signed running_pe [0:7][0:7];\n\n"
)
text = text[:start] + new_golden_declarations + text[end:]

# Generalize the PE accumulator checker to compare any completed Tile.
replace_task(
    "check_pe_matrix",
    r'''  task automatic check_pe_matrix(
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
  endtask'''
)

# Generalize the packed output checker to compare any completed Tile.
replace_task(
    "check_outputs",
    r'''  task automatic check_outputs(
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
  endtask'''
)

# Replace the P6-A two-Tile main sequence with a nine-Tile loop.
main_start_marker = "  initial begin : main_test\n"
watchdog_marker = "  // --------------------------------------------------------------------------\n  // Watchdog"
main_start = text.find(main_start_marker)
main_end = text.find(watchdog_marker, main_start)
if main_start < 0 or main_end < 0:
    raise SystemExit("Could not locate the P6-A main test block")

new_main = r'''  initial begin : main_test
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
'''

text = text[:main_start] + new_main + "\n\n" + text[main_end:]

# Update file-specific labels and waveform names.
text = text.replace("tb_p6a_cross_ktile_accum", "tb_p6b_multi_ktile_accum")
text = text.replace("P6-A EXTENDED TEST", "P6-B1 NINE-K-TILE TEST")
text = text.replace("P6-A extended functional test", "P6-B1 nine-K-Tile stress test")
text = text.replace("Two K-Tiles, each containing K = 64 elements", "Nine K-Tiles, each containing K = 64 elements")
text = text.replace("M = 1, N = 8, K_TOTAL = 128", "M = 1, N = 8, K_TOTAL = 576")
text = text.replace("repeat (5000)", "repeat (10000)")

# Refresh the global-K offset explanation above the Reader model.
text = text.replace(
    "  // Tile 1 uses a global offset of 64 while the K Controller's local address\n"
    "  // restarts at zero. Since K_TILE=64 is divisible by 8, PE row mapping remains\n"
    "  // aligned across the K-Tile boundary.\n",
    "  // Each Tile applies active_tile*K_TILE while the K Controller's local address\n"
    "  // restarts at zero. K_TILE=64 is divisible by 8, so PE row mapping remains\n"
    "  // aligned across all nine K-Tile boundaries.\n"
)

# Refresh the Reader-model explanatory comment.
text = text.replace(
    "  // Reader model selects global K range:\n"
    "  // active_tile=0 -> global K 0..63\n"
    "  // active_tile=1 -> global K 64..127\n"
    "  integer active_tile;",
    "  // Reader model selects the global K range for the current local K-Tile.\n"
    "  // active_tile=0..8 maps local K=0..63 to global K=0..575.\n"
    "  integer active_tile;"
)

out_path.write_text(text, encoding="utf-8")
print(f"Created: {out_path}")
print("P6-A source file was not modified.")
print("Next: add tb/tb_p6b_multi_ktile_accum.sv as the Vivado Simulation Top.")
