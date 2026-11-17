`timescale 1ns / 1ps

// ============================================================================
// Testbench: tb_p6a_cross_ktile_accum.sv
//
// P6-A objective:
//   Verify that every K-Tile clears the MAC pipeline, while only a new
//   M/N output tile clears the INT32 accumulators.
//
// Test sequence:
//   Tile 0: clear_acc_on_start = 1, weight = 10 + n
//   Tile 1: clear_acc_on_start = 0, weight = 20 + n
//           The previous INT32 partial sums must survive clear_pipe.
//   Tile 2: clear_acc_on_start = 1, weight = 3 + n
//           The previous accumulated result must be cleared.
//
// K_LEN = 8 elements = 4 paired Reader requests per K-Tile.
// Activation values are K+1, so PE row r receives activation r+1.
// The sum of the eight activation values is 1+2+...+8 = 36.
//
// Expected output per column n:
//   Tile 0 result = 36 * (10 + n)
//   Tile 1 result = 36 * ((10+n) + (20+n)) = 36 * (30 + 2*n)
//   Tile 2 result = 36 * (3 + n)
//
// This testbench instantiates the K controller and systolic array directly.
// A one-cycle synchronous Reader model supplies paired responses.
// ============================================================================

module tb_p6a_cross_ktile_accum;

  localparam int K_ADDR_WIDTH = 16;
  localparam int K_LEN        = 8;
  localparam int DRAIN_CYCLES = 13;

  logic clk;
  logic rst_n;
  logic start;
  logic clear_acc_on_start;
  logic [K_ADDR_WIDTH-1:0] k_len;

  // --------------------------------------------------------------------------
  // Reader interface
  // --------------------------------------------------------------------------

  logic [15:0]  act_pair_packed;
  logic [1:0]   act_rsp_valid;

  logic [127:0] wgt_pair_packed;
  logic [1:0]   wgt_rsp_valid;

  logic pair_req_en;
  logic [K_ADDR_WIDTH-1:0] fetch_k_base;
  logic [K_ADDR_WIDTH-1:0] k_base;

  // Selects the test data set returned by the Reader model:
  //   0 = first K-Tile
  //   1 = continuation K-Tile
  //   2 = new output tile (clear previous accumulation)
  integer active_tile;

  // --------------------------------------------------------------------------
  // K controller <-> systolic array interface
  // --------------------------------------------------------------------------

  logic array_enable;
  logic feed_valid;
  logic clear_acc;
  logic clear_pipe;
  logic valid_reset;
  logic data_valid_in;
  logic [63:0] act_in_packed;
  logic [127:0] wgt_pair_in_packed;

  logic [255:0] acc_out_packed;
  logic data_valid_out;
  logic any_overflow;

  logic busy;
  logic done;
  logic [255:0] result_packed;

  integer pipe_clear_count;
  integer acc_clear_count;
  integer request_count;
  integer expected_fetch_base;


  // --------------------------------------------------------------------------
  // Clock
  // --------------------------------------------------------------------------

  initial clk = 1'b0;
  always #5 clk = ~clk;


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
  // The K controller requests K pairs:
  //   K=0, 2, 4, 6
  //
  // The response is registered on the request edge and is consumed by the
  // controller during the following FEED cycle.
  // --------------------------------------------------------------------------

  always_ff @(posedge clk or negedge rst_n) begin
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

        // Return the two activation values addressed by this K-pair.
        // K=0,2,4,6 returns (1,2), (3,4), (5,6), (7,8).
        act_pair_packed[7:0]  <= 8'(int'(fetch_k_base) + 1);
        act_pair_packed[15:8] <= 8'(int'(fetch_k_base) + 2);

        // The two independent weight streams receive the same value here.
        // Stream 0 occupies bits [63:0]; stream 1 occupies bits [127:64].
        for (int n = 0; n < 8; n++) begin
          case (active_tile)
            0: begin
              wgt_pair_packed[n*8 +: 8]       <= 8'(10 + n);
              wgt_pair_packed[(8+n)*8 +: 8]   <= 8'(10 + n);
            end

            1: begin
              wgt_pair_packed[n*8 +: 8]       <= 8'(20 + n);
              wgt_pair_packed[(8+n)*8 +: 8]   <= 8'(20 + n);
            end

            default: begin
              wgt_pair_packed[n*8 +: 8]       <= 8'(3 + n);
              wgt_pair_packed[(8+n)*8 +: 8]   <= 8'(3 + n);
            end
          endcase
        end

        // The two activation and two weight responses are both valid.
        act_rsp_valid <= 2'b11;
        wgt_rsp_valid <= 2'b11;
      end
    end
  end


  // --------------------------------------------------------------------------
  // Control and request monitors
  // --------------------------------------------------------------------------

  always @(posedge clk) begin
    if (!rst_n) begin
      pipe_clear_count    = 0;
      acc_clear_count     = 0;
      request_count       = 0;
      expected_fetch_base = 0;
    end
    else begin

      if (clear_pipe) begin
        pipe_clear_count = pipe_clear_count + 1;

        if (!array_enable)
          $fatal(1, "FAIL: clear_pipe asserted while array_enable=0");
      end

      if (clear_acc) begin
        acc_clear_count = acc_clear_count + 1;

        if (!clear_pipe)
          $fatal(1, "FAIL: clear_acc asserted without clear_pipe");
      end

      // A new K-Tile starts a fresh Reader address sequence at K=0.
      if (start && !busy) begin
        request_count       = 0;
        expected_fetch_base = 0;
      end
      else if (pair_req_en) begin

        if (int'(fetch_k_base) != expected_fetch_base) begin
          $fatal(1,
            "FAIL: unexpected K request address, got=%0d expected=%0d",
            int'(fetch_k_base), expected_fetch_base);
        end

        request_count       = request_count + 1;
        expected_fetch_base = expected_fetch_base + 2;
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
      // DONE is a one-cycle state. Wait until the controller has returned
      // to IDLE before presenting the next start pulse.
      wait (done == 1'b0);
      @(negedge clk);

      active_tile        = tile_id;
      clear_acc_on_start = clear_acc_flag;
      start              = 1'b1;

      @(negedge clk);
      start = 1'b0;
    end
  endtask


  // --------------------------------------------------------------------------
  // Helper: check all 64 PE accumulators
  //
  // Expected PE value is (row+1) * (base_weight + weight_step*column).
  // --------------------------------------------------------------------------

  task automatic check_pe_matrix(
    input integer base_weight,
    input integer weight_step,
    input string  stage_name
  );
    integer got;
    integer expected;
    begin
      for (int r = 0; r < 8; r++) begin
        for (int c = 0; c < 8; c++) begin

          got = int'($signed(dut_array.pe_acc[r][c]));
          expected = (r + 1) * (base_weight + weight_step * c);

          if (got != expected) begin
            $fatal(1,
              "FAIL: %s PE[%0d][%0d] got=%0d expected=%0d",
              stage_name, r, c, got, expected);
          end

        end
      end

      $display("PASS: %s, all 64 PE accumulators correct",
               stage_name);
    end
  endtask


  // --------------------------------------------------------------------------
  // Helper: check the 8 packed INT32 output values
  //
  // result_case:
  //   0 = after Tile 0
  //   1 = after Tile 0 + Tile 1
  //   2 = after Tile 2 has cleared the previous accumulation
  // --------------------------------------------------------------------------

  task automatic check_result(input integer result_case);
    integer got;
    integer expected;
    begin
      for (int n = 0; n < 8; n++) begin

        case (result_case)
          0: expected = 36 * (10 + n);
          1: expected = 36 * (30 + 2*n);
          default: expected = 36 * (3 + n);
        endcase

        got = int'($signed(result_packed[n*32 +: 32]));

        if (got != expected) begin
          $fatal(1,
            "FAIL: result[%0d] got=%0d expected=%0d case=%0d",
            n, got, expected, result_case);
        end

        // The live array output must agree with the registered K result.
        got = int'($signed(acc_out_packed[n*32 +: 32]));

        if (got != expected) begin
          $fatal(1,
            "FAIL: acc_out[%0d] got=%0d expected=%0d case=%0d",
            n, got, expected, result_case);
        end

      end

      if (any_overflow !== 1'b0)
        $fatal(1, "FAIL: unexpected accumulator overflow");

      $display("PASS: result case %0d, all 8 INT32 outputs correct",
               result_case);
    end
  endtask


  // --------------------------------------------------------------------------
  // Main test sequence
  // --------------------------------------------------------------------------

  initial begin
    rst_n                = 1'b0;
    start                = 1'b0;
    clear_acc_on_start   = 1'b0;
    k_len                = K_LEN;

    active_tile          = 0;

    repeat (4) @(negedge clk);
    rst_n = 1'b1;


    // ========================================================================
    // Tile 0: first K-Tile of a new output
    // clear_pipe = 1, clear_acc = 1
    // ========================================================================

    $display("\n--- P6-A TEST: Tile 0, clear accumulator ---");

    launch_tile(0, 1'b1);

    wait (clear_pipe && clear_acc);
    @(posedge clk);
    #1;

    check_pe_matrix(0, 0, "Tile 0 after clear_acc");

    wait (done === 1'b1);
    #1;

    check_result(0);
    check_pe_matrix(10, 1, "Tile 0 completed");

    if (request_count != K_LEN/2)
      $fatal(1, "FAIL: Tile 0 request_count=%0d expected=%0d",
             request_count, K_LEN/2);


    // ========================================================================
    // Tile 1: continuation of the same output
    // clear_pipe = 1, clear_acc = 0
    // The PE partial sums must remain unchanged during S_CLEAR.
    // ========================================================================

    $display("\n--- P6-A TEST: Tile 1, preserve accumulator ---");

    launch_tile(1, 1'b0);

    wait (clear_pipe && !clear_acc);
    @(posedge clk);
    #1;

    check_pe_matrix(10, 1, "Tile 1 after clear_pipe");

    wait (done === 1'b1);
    #1;

    check_result(1);
    check_pe_matrix(30, 2, "Tile 1 completed");

    if (request_count != K_LEN/2)
      $fatal(1, "FAIL: Tile 1 request_count=%0d expected=%0d",
             request_count, K_LEN/2);


    // ========================================================================
    // Tile 2: start a new output tile
    // clear_pipe = 1, clear_acc = 1
    // The previous Tile 0 + Tile 1 sum must be removed.
    // ========================================================================

    $display("\n--- P6-A TEST: Tile 2, clear previous output ---");

    launch_tile(2, 1'b1);

    wait (clear_pipe && clear_acc);
    @(posedge clk);
    #1;

    check_pe_matrix(0, 0, "Tile 2 after clear_acc");

    wait (done === 1'b1);
    #1;

    check_result(2);
    check_pe_matrix(3, 1, "Tile 2 completed");

    if (request_count != K_LEN/2)
      $fatal(1, "FAIL: Tile 2 request_count=%0d expected=%0d",
             request_count, K_LEN/2);


    // ------------------------------------------------------------------------
    // Final control checks
    // ------------------------------------------------------------------------

    if (pipe_clear_count != 3)
      $fatal(1,
        "FAIL: clear_pipe count=%0d expected=3",
        pipe_clear_count);

    if (acc_clear_count != 2)
      $fatal(1,
        "FAIL: clear_acc count=%0d expected=2",
        acc_clear_count);

    $display("\n==================================================");
    $display("PASS: P6-A cross-K-Tile accumulation");
    $display("  K-Tile 0: clear accumulator");
    $display("  K-Tile 1: preserve partial sums across clear_pipe");
    $display("  K-Tile 2: clear previous output accumulation");
    $display("  clear_pipe pulses = %0d", pipe_clear_count);
    $display("  clear_acc  pulses = %0d", acc_clear_count);
    $display("  all 64 PE accumulators and 8 outputs verified");
    $display("==================================================\n");

    $finish;
  end


  // --------------------------------------------------------------------------
  // Watchdog
  // --------------------------------------------------------------------------

  initial begin
    repeat (2000) @(posedge clk);
    $fatal(1, "TIMEOUT: tb_p6a_cross_ktile_accum");
  end


  // --------------------------------------------------------------------------
  // Optional waveform dump
  // --------------------------------------------------------------------------

  initial begin
    $dumpfile("tb_p6a_cross_ktile_accum.vcd");
    $dumpvars(0, tb_p6a_cross_ktile_accum);
  end

endmodule
