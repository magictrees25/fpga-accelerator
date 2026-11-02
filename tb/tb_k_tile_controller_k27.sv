`timescale 1ns / 1ps

// Verification of the existing k_tile_controller + systolic_array.
// Task: M=1, N=8, K=27, i.e. four logical K tiles (8+8+8+3).
//
// A[k]   = k + 1                      (k=0..26)
// B[k,n] = ((k % 3) + 1) * (n + 1)    (n=0..7)
// C[n]   = 774 * (n+1)
//
// A and B are provided by an ideal combinational ROM in this TB.
// No edits to k_tile_controller.sv, mac_unit.sv, or systolic_array.sv.

module tb_k_tile_controller_k27;
  import pkg_accelerator::*;

  localparam int TEST_K = 27;
  localparam int EXPECTED_C0 = 774;

  logic clk = 1'b0;
  always #5 clk = ~clk;  // 100 MHz

  logic rst_n;
  logic start;
  logic [15:0] k_len;
  logic [15:0] k_addr;
  logic signed [ACT_WIDTH-1:0] act_data;
  logic [WGT_WIDTH*ARRAY_COLS-1:0] wgt_row_packed;

  logic array_enable;
  logic feed_valid;
  logic clear_acc;
  logic valid_reset;
  logic data_valid_in;
  logic [ACT_WIDTH*ARRAY_ROWS-1:0] act_in_packed;
  logic signed [WGT_WIDTH-1:0] wgt_in [ARRAY_COLS];
  logic [ACC_WIDTH*ARRAY_COLS-1:0] acc_out_packed;
  logic [ACC_WIDTH*ARRAY_COLS-1:0] result_packed;
  logic data_valid_out;
  logic any_overflow;
  logic busy;
  logic done;

  k_tile_controller u_kctrl (
    .clk(clk), .rst_n(rst_n), .start(start), .k_len(k_len),
    .k_addr(k_addr), .act_data(act_data), .wgt_row_packed(wgt_row_packed),
    .array_enable(array_enable), .feed_valid(feed_valid),
    .clear_acc(clear_acc), .valid_reset(valid_reset),
    .data_valid_in(data_valid_in), .act_in_packed(act_in_packed),
    .wgt_in(wgt_in), .acc_out_packed(acc_out_packed),
    .busy(busy), .done(done), .result_packed(result_packed)
  );

  systolic_array dut (
    .clk(clk), .rst_n(rst_n), .enable(array_enable),
    .clear_acc(clear_acc), .valid_reset(valid_reset),
    .act_in_packed(act_in_packed), .wgt_in(wgt_in),
    .data_valid_in(data_valid_in), .data_valid_out(data_valid_out),
    .acc_out_packed(acc_out_packed), .any_overflow(any_overflow)
  );

  // Combinational A/B reference data source.
  // K=27 corresponds to e.g. a 3x3 convolution with 3 input channels.
  always_comb begin
    act_data = '0;
    wgt_row_packed = '0;
    if (k_addr < TEST_K) begin
      act_data = $signed(k_addr + 16'd1);
      for (int n = 0; n < ARRAY_COLS; n = n + 1) begin
        wgt_row_packed[n*WGT_WIDTH +: WGT_WIDTH] =
          ((k_addr % 3) + 1) * (n + 1);
      end
    end
  end

  integer beats_seen;
  integer clear_seen;
  integer got;
  integer exp_val;

  // Check exactly one feed beat for each k, and no clear inside the feed.
  always @(posedge clk) begin
    if (!rst_n) begin
      beats_seen = 0;
      clear_seen = 0;
    end else begin
      if (clear_acc) begin
        clear_seen = clear_seen + 1;
        if (feed_valid)
          $fatal(1, "FAIL: clear_acc asserted during FEED");
      end
      if (feed_valid) begin
        if (clear_acc)
          $fatal(1, "FAIL: clear_acc asserted while feeding K%0d", k_addr);
        if (k_addr !== beats_seen)
          $fatal(1, "FAIL: k_addr=%0d, expected=%0d", k_addr, beats_seen);
        if (k_addr % ARRAY_ROWS == 0)
          $display("K-TILE %0d starts at K%0d, PE row 0", k_addr / ARRAY_ROWS, k_addr);
        $display("FEED: K%0d -> PE row %0d", k_addr, k_addr % ARRAY_ROWS);
        beats_seen = beats_seen + 1;
      end
    end
  end

  initial begin
    rst_n = 1'b0;
    start = 1'b0;
    k_len = TEST_K;
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    start = 1'b1;
    @(negedge clk);
    start = 1'b0;

    wait (done === 1'b1);
    #1; // sample after nonblocking assignments

    if (beats_seen !== TEST_K)
      $fatal(1, "FAIL: fed %0d K elements, expected %0d", beats_seen, TEST_K);
    if (clear_seen !== 1)
      $fatal(1, "FAIL: clear_acc seen %0d times (expected exactly 1)", clear_seen);

    $display("----- Automatic K=27 results -----");
    for (int n = 0; n < ARRAY_COLS; n = n + 1) begin
      got = $signed(result_packed[n*ACC_WIDTH +: ACC_WIDTH]);
      exp_val = EXPECTED_C0 * (n + 1);
      $display("C[%0d] = %0d (expected %0d)", n, got, exp_val);
      if (got !== exp_val)
        $fatal(1, "FAIL: C[%0d]=%0d, expected %0d", n, got, exp_val);
    end

    // PE row 0 gets K0, K8, K16, K24:
    // 1*1 + 9*3 + 17*2 + 25*1 = 87 for column 0.
    if ($signed(dut.gen_row[0].gen_col[0].u_pe.acc_reg) !== 32'sd87)
      $fatal(1, "FAIL: PE[0][0] expected 87, got %0d",
             $signed(dut.gen_row[0].gen_col[0].u_pe.acc_reg));

    // Same activation sequence; column 7 weights are 8x column 0.
    if ($signed(dut.gen_row[0].gen_col[7].u_pe.acc_reg) !== 32'sd696)
      $fatal(1, "FAIL: PE[0][7] expected 696, got %0d",
             $signed(dut.gen_row[0].gen_col[7].u_pe.acc_reg));

    if (any_overflow !== 1'b0)
      $fatal(1, "FAIL: unexpected overflow");

    $display("PASS: automatic K=27 (4 logical K tiles), C=[774,1548,2322,3096,3870,4644,5418,6192]");
    $finish;
  end

  initial begin
    repeat (300) @(posedge clk);
    $fatal(1, "TIMEOUT: K=27 controller did not assert done");
  end
endmodule
