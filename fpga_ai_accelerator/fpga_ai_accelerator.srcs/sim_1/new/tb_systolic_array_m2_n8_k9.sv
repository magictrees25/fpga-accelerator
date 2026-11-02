`timescale 1ns / 1ps

// Two-output-position test for taitashaw/fpga-ai-accelerator.
// GEMM: M=2, N=8, K=9
//   A[0,:] = [ 1, 2, 3, 4, 5, 6, 7, 8, 9]   sum= 45
//   A[1,:] = [10,11,12,13,14,15,16,17,18]   sum=126
//   B[k,n] = n+1 for all k=0..8, n=0..7
// Expected:
//   C[0,:] = [ 45,  90, 135, 180, 225, 270, 315, 360]
//   C[1,:] = [126, 252, 378, 504, 630, 756, 882,1008]
//
// What this TB verifies:
//   (1) K=9 wraps around the eight PE rows without clearing the sum.
//   (2) All eight output columns produce the expected results.
//   (3) The first output row is saved BEFORE clearing accumulators.
//   (4) clear_acc pulses for one rising edge between M positions.
//   (5) The second output row starts from zero, not from M=0's sums.
//
// NOTE: Scheduling and output storage are provided by this TESTBENCH.
//       The existing RTL does not yet automatically iterate over M.
module tb_systolic_array_m2_n8_k9;
    import pkg_accelerator::*;

    logic clk;
    logic rst_n;
    logic enable;
    logic clear_acc;
    logic valid_reset;
    logic data_valid_in;
    logic data_valid_out;
    logic [ACT_WIDTH*ARRAY_ROWS-1:0] act_in_packed;
    logic signed [WGT_WIDTH-1:0] wgt_in [ARRAY_COLS];
    logic [ACC_WIDTH*ARRAY_COLS-1:0] acc_out_packed;
    logic any_overflow;

    integer saved_C [0:1][0:ARRAY_COLS-1];
    integer m_phase;  // 0=M0, 1=clear, 2=M1; handy for Vivado waveforms
    integer edge_count;

    initial clk = 1'b0;
    always #5 clk = ~clk;  // 10 ns period

    systolic_array dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .enable        (enable),
        .clear_acc     (clear_acc),
        .valid_reset   (valid_reset),
        .act_in_packed (act_in_packed),
        .wgt_in        (wgt_in),
        .data_valid_in (data_valid_in),
        .data_valid_out(data_valid_out),
        .acc_out_packed(acc_out_packed),
        .any_overflow  (any_overflow)
    );

    // Trace current column 0 and 7. The #1 observes registered updates.
    always @(posedge clk) begin
        if (rst_n && enable) begin
            edge_count = edge_count + 1;
            #1;
            $display("edge=%0d phase=%0d clear=%0b C0=%0d C7=%0d valid=%0b",
                     edge_count, m_phase, clear_acc,
                     $signed(acc_out_packed[0*ACC_WIDTH +: ACC_WIDTH]),
                     $signed(acc_out_packed[7*ACC_WIDTH +: ACC_WIDTH]),
                     data_valid_out);
        end
    end

    // Drives one GEMM output position (one A row) across 9 active beats.
    // Called at a falling clock edge. Each beat is sampled at the next
    // rising edge; @(negedge clk) advances to the next beat.
    task automatic drive_m_row(input integer row_index);
        integer kk;
        integer nn;
        integer activation;
        begin
            for (kk = 0; kk < 9; kk = kk + 1) begin
                activation = (row_index == 0) ? (kk + 1) : (kk + 10);
                act_in_packed = '0;
                act_in_packed[(kk % ARRAY_ROWS)*ACT_WIDTH +: ACT_WIDTH] =
                    activation;
                for (nn = 0; nn < ARRAY_COLS; nn = nn + 1)
                    wgt_in[nn] = nn + 1;
                data_valid_in = (kk == 0);
                @(negedge clk);
            end

            // Supply zero bubbles to drain the registered PE/skew pipeline.
            act_in_packed = '0;
            for (nn = 0; nn < ARRAY_COLS; nn = nn + 1)
                wgt_in[nn] = '0;
            data_valid_in = 1'b0;
        end
    endtask

    // Verify all eight output columns, and save the calculated C row.
    task automatic check_and_save_row(input integer row_index);
        integer nn;
        integer expected;
        integer got;
        integer expected_pe00;
        begin
            for (nn = 0; nn < ARRAY_COLS; nn = nn + 1) begin
                expected = (row_index == 0 ? 45 : 126) * (nn + 1);
                got = $signed(acc_out_packed[nn*ACC_WIDTH +: ACC_WIDTH]);
                $display("C[%0d][%0d] = %0d (expected %0d)",
                         row_index, nn, got, expected);
                if (got !== expected)
                    $fatal(1, "FAIL: C[%0d][%0d]=%0d, expected %0d",
                           row_index, nn, got, expected);
                saved_C[row_index][nn] = got;
            end

            expected_pe00 = (row_index == 0) ? 10 : 28;
            if ($signed(dut.gen_row[0].gen_col[0].u_pe.acc_reg)
                    !== expected_pe00)
                $fatal(1, "FAIL: row %0d PE[0][0] acc, expected %0d",
                       row_index, expected_pe00);

            if (any_overflow !== 1'b0)
                $fatal(1, "FAIL: unexpected overflow in M=%0d", row_index);
        end
    endtask

    integer nn;
    initial begin
        rst_n         = 1'b0;
        enable        = 1'b0;
        clear_acc     = 1'b0;
        valid_reset   = 1'b0;
        data_valid_in = 1'b0;
        act_in_packed = '0;
        m_phase       = -1;
        edge_count    = 0;
        for (nn = 0; nn < ARRAY_COLS; nn = nn + 1)
            wgt_in[nn] = '0;

        repeat (3) @(negedge clk);
        rst_n  = 1'b1;
        enable = 1'b1;

        // ----- First output position, M=0 -----
        m_phase = 0;
        $display("----- Drive M=0 -----");
        drive_m_row(0);
        repeat (11) @(posedge clk); // 9 active beats + 11 drain beats
        #2;                          // settle NBA updates and column sums
        $display("----- Verify and store M=0 BEFORE clearing -----");
        check_and_save_row(0);

        // ----- Clear PE accumulators and the weight-skew pipeline -----
        // Must only clear after the last useful result was accumulated.
        @(negedge clk);
        m_phase  = 1;
        clear_acc = 1'b1;
        @(posedge clk);
        #2;
        if (acc_out_packed !== '0)
            $fatal(1, "FAIL: clear_acc did not reset column sums");
        $display("CLEAR: all eight column sums returned to 0");
        @(negedge clk);
        clear_acc = 1'b0;

        // ----- Second output position, M=1 -----
        m_phase = 2;
        $display("----- Drive M=1 -----");
        drive_m_row(1);
        repeat (11) @(posedge clk);
        #2;
        $display("----- Verify and store M=1 -----");
        check_and_save_row(1);

        // Saved M=0 results should survive the PE accumulator clear.
        for (nn = 0; nn < ARRAY_COLS; nn = nn + 1) begin
            if (saved_C[0][nn] !== 45 * (nn + 1))
                $fatal(1, "FAIL: stored first row corrupted, N=%0d", nn);
            if (saved_C[1][nn] !== 126 * (nn + 1))
                $fatal(1, "FAIL: stored second row corrupted, N=%0d", nn);
        end

        $display("PASS: M=2 N=8 K=9; first C row [45,90,...,360], second [126,252,...,1008]");
        $finish;
    end
endmodule
