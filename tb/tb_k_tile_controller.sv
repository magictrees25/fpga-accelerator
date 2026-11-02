`timescale 1ns / 1ps

// Integration TB for k_tile_controller + original 8x8 systolic_array.
// M=1, N=8, K=9. Testbench supplies only RAM-like data, start and k_len;
// the controller schedules all 9 beats, the clear, the drain and the done.
//
// A[k]   = k+1
// B[k,n] = (k+1)*(n+1)  -- unlike earlier all-ones-per-column tests,
//                         weights now change with K, checking address order.
// C[n]   = (1^2 + 2^2 + ... + 9^2)*(n+1) = 285*(n+1).
module tb_k_tile_controller;
  import pkg_accelerator::*;

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
    .clear_acc(clear_acc),
    .valid_reset(valid_reset), .data_valid_in(data_valid_in),
    .act_in_packed(act_in_packed), .wgt_in(wgt_in),
    .acc_out_packed(acc_out_packed),
    .busy(busy), .done(done), .result_packed(result_packed)
  );

  systolic_array dut (
    .clk(clk), .rst_n(rst_n), .enable(array_enable),
    .clear_acc(clear_acc), .valid_reset(valid_reset),
    .act_in_packed(act_in_packed), .wgt_in(wgt_in),
    .data_valid_in(data_valid_in), .data_valid_out(data_valid_out),
    .acc_out_packed(acc_out_packed), .any_overflow(any_overflow)
  );

  // Simplified combinational A/B ROM.
  // For a real synchronous BRAM, the controller must handle read latency.
  always_comb begin
    act_data = '0;
    wgt_row_packed = '0;
    if (k_addr < 9) begin
      act_data = $signed(k_addr + 16'd1);
      for (int n = 0; n < ARRAY_COLS; n = n + 1)
        wgt_row_packed[n*WGT_WIDTH +: WGT_WIDTH] =
          (k_addr + 16'd1) * (n + 1);
    end
  end

  integer got;
  integer exp_val;
  integer n;
  integer beats_seen;

  // Assert that exactly K=9 beats are sent in the correct order.
  always @(posedge clk) begin
    if (!rst_n) begin
      beats_seen = 0;
    end else if (feed_valid) begin
      if (k_addr !== beats_seen)
        $fatal(1, "FAIL: K address %0d, expected %0d", k_addr, beats_seen);
      $display("FEED: K%0d -> PE row %0d", k_addr, k_addr % ARRAY_ROWS);
      beats_seen = beats_seen + 1;
    end
  end

  initial begin
    rst_n = 1'b0;
    start = 1'b0;
    k_len = 16'd9;
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    start = 1'b1;
    @(negedge clk);
    start = 1'b0;

    // Wait for the controller to finish on its own.
    wait (done === 1'b1);
    #1;  // after nonblocking result snapshot update

    if (beats_seen !== 9)
      $fatal(1, "FAIL: sent %0d K beats, expected 9", beats_seen);

    $display("----- Auto K=9 results -----");
    for (n = 0; n < ARRAY_COLS; n = n + 1) begin
      got = $signed(result_packed[n*ACC_WIDTH +: ACC_WIDTH]);
      exp_val = 285*(n+1);
      $display("C[%0d] = %0d (expected %0d)", n, got, exp_val);
      if (got !== exp_val)
        $fatal(1, "FAIL: channel %0d result %0d != %0d", n, got, exp_val);
    end

    // PE row 0 accumulated both k=0 and k=8, so (1^2+9^2)=82.
    if ($signed(dut.gen_row[0].gen_col[0].u_pe.acc_reg) !== 32'sd82)
      $fatal(1, "FAIL: PE[0][0] expected 82");
    if (any_overflow !== 1'b0)
      $fatal(1, "FAIL: unexpected overflow");

    $display("PASS: automatic K=9 scheduling, 8 columns correct, C=[285,570,855,1140,1425,1710,1995,2280]");
    $finish;
  end

  initial begin
    repeat (200) @(posedge clk);
    $fatal(1, "TIMEOUT: controller did not assert done");
  end
endmodule
