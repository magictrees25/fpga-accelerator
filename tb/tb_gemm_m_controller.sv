`timescale 1ns / 1ps

// TEST 1: automatic M scheduling with K=9 and 8 output channels.
// One top-level start pulse; NO manually generated M switches/clear pulses.
// A = [ 1..9; 10..18 ]  (2x9)
// B[k,n] = n+1          (9x8)
// Expected C rows: [45,90,...,360], [126,252,...,1008].
module tb_gemm_m_controller;
  import pkg_accelerator::*;
  localparam int M = 2;
  localparam int K = 9;
  localparam int N = ARRAY_COLS;

  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst_n, start;
  logic [15:0] m_len, k_len;
  logic [15:0] m_addr, k_addr;
  logic signed [ACT_WIDTH-1:0] act_data;
  logic [WGT_WIDTH*ARRAY_COLS-1:0] wgt_row_packed;
  logic write_valid, busy, done, any_overflow;
  logic [15:0] write_addr;
  logic [ACC_WIDTH*ARRAY_COLS-1:0] write_data;

  // C output RAM in the TB.  In hardware these pins can connect to BRAM.
  logic signed [ACC_WIDTH-1:0] c_memory [0:M-1][0:N-1];
  integer rows_written;
  integer clears_seen;
  integer beats_seen;
  integer expected_val;
  integer got_val;

  gemm_m_controller dut (
    .clk(clk), .rst_n(rst_n), .start(start),
    .m_len(m_len), .k_len(k_len),
    .m_addr(m_addr), .k_addr(k_addr),
    .act_data(act_data), .wgt_row_packed(wgt_row_packed),
    .write_valid(write_valid), .write_addr(write_addr),
    .write_data(write_data), .busy(busy), .done(done),
    .any_overflow(any_overflow)
  );

  // Ideal combinational A/B RAMs: no read latency, for first-stage bring-up.
  always_comb begin
    act_data      = '0;
    wgt_row_packed = '0;
    if (m_addr < M && k_addr < K) begin
      act_data = 8'(m_addr * K + k_addr + 1);
      for (int n = 0; n < N; n = n + 1)
        wgt_row_packed[n*WGT_WIDTH +: WGT_WIDTH] = 8'(n + 1);
    end
  end

  // Check that the RTL owns the scheduling, and check each write as it occurs.
  always @(posedge clk) begin
    if (!rst_n) begin
      rows_written = 0;
      clears_seen  = 0;
      beats_seen   = 0;
    end else begin
      if (dut.clear_acc) begin
        clears_seen = clears_seen + 1;
        if (dut.feed_valid)
          $fatal(1, "FAIL: clear while feeding K");
      end
      if (dut.feed_valid) begin
        beats_seen = beats_seen + 1;
        if (dut.clear_acc)
          $fatal(1, "FAIL: clear asserted during K feed");
      end
      if (write_valid) begin
        if (write_addr !== rows_written)
          $fatal(1, "FAIL: output row order %0d expected %0d", write_addr, rows_written);
        $display("SAVE C row M%0d:", write_addr);
        for (int n = 0; n < N; n = n + 1) begin
          got_val = $signed(write_data[n*ACC_WIDTH +: ACC_WIDTH]);
          expected_val = ((write_addr == 0) ? 45 : 126) * (n + 1);
          if (got_val !== expected_val)
            $fatal(1, "FAIL: C[%0d][%0d]=%0d expected=%0d", write_addr,n,got_val,expected_val);
          c_memory[write_addr][n] = got_val;
          $write(" %0d", got_val);
        end
        $display("");
        rows_written = rows_written + 1;
      end
    end
  end

  initial begin
    rst_n = 0;
    start = 0;
    m_len = M;
    k_len = K;
    repeat (3) @(negedge clk);
    rst_n = 1;
    @(negedge clk);
    start = 1;
    @(negedge clk);
    start = 0;

    wait (done === 1'b1);
    #1;
    if (rows_written !== M)
      $fatal(1, "FAIL: saved %0d rows, expected %0d", rows_written, M);
    if (clears_seen !== M)
      $fatal(1, "FAIL: clear count %0d expected %0d", clears_seen, M);
    if (beats_seen !== M*K)
      $fatal(1, "FAIL: feed count %0d expected %0d", beats_seen, M*K);

    for (int m = 0; m < M; m = m + 1)
      for (int n = 0; n < N; n = n + 1) begin
        expected_val = (m == 0 ? 45 : 126) * (n + 1);
        if (c_memory[m][n] !== expected_val)
          $fatal(1, "FAIL: stored C[%0d][%0d] mismatch", m,n);
      end

    if ($signed(dut.u_array.gen_row[0].gen_col[0].u_pe.acc_reg) !== 32'sd28)
      $fatal(1, "FAIL: PE[0][0] should be 28 after second M calculation");
    if (any_overflow !== 1'b0)
      $fatal(1, "FAIL: accumulator overflow");

    $display("PASS: automatic M=2 N=8 K=9, two C rows saved; clears=2, feeds=18");
    $finish;
  end

  initial begin
    repeat (400) @(posedge clk);
    $fatal(1, "TIMEOUT: M controller did not finish");
  end
endmodule
