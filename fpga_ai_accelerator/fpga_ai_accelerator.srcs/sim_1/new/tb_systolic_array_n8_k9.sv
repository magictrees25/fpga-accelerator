`timescale 1ns / 1ps

// Full-column test for taitashaw/fpga-ai-accelerator's 8x8 systolic_array.
// GEMM: M=1, N=8, K=9
// A = [1,2,3,4,5,6,7,8,9]
// B[k][n] = n+1 (all nine rows of column n have the same weight).
// Expected C = [45,90,135,180,225,270,315,360].
//
// The 9th K element reuses PE row 0. Accumulators are NOT cleared
// between K-tiles. Weight skew and activation forwarding are exercised
// across all eight columns.
module tb_systolic_array_n8_k9;
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

    initial clk = 1'b0;
    always #5 clk = ~clk;   // 10-ns period

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

    integer k;
    integer n;
    integer expected;
    integer got;
    integer edge_count = 0;

    always @(posedge clk) begin
        if (rst_n && enable) begin
            edge_count = edge_count + 1;
            #1; // after nonblocking register updates
            $display("edge=%0d C0=%0d C7=%0d valid=%0b",
                     edge_count,
                     $signed(acc_out_packed[0*ACC_WIDTH +: ACC_WIDTH]),
                     $signed(acc_out_packed[7*ACC_WIDTH +: ACC_WIDTH]),
                     data_valid_out);
        end
    end

    initial begin
        rst_n         = 1'b0;
        enable        = 1'b0;
        clear_acc     = 1'b0;
        valid_reset   = 1'b0;
        data_valid_in = 1'b0;
        act_in_packed = '0;
        for (n = 0; n < ARRAY_COLS; n = n + 1)
            wgt_in[n] = '0;

        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        enable = 1'b1;

        // Nine input beats: row 0..7, then reuse row 0 for k=8.
        // Row k%8 is active on beat k; other row inputs are zero.
        // Column n's weight is (n+1) on every beat (one weight/col/beat).
        for (k = 0; k < 9; k = k + 1) begin
            act_in_packed = '0;
            act_in_packed[(k % ARRAY_ROWS)*ACT_WIDTH +: ACT_WIDTH] = k + 1;
            for (n = 0; n < ARRAY_COLS; n = n + 1)
                wgt_in[n] = n + 1;
            data_valid_in = (k == 0);
            @(negedge clk); // beat k was sampled at the preceding posedge
        end

        // Drain the array with zero inputs. Last valid activation is
        // captured by PE[0][7] on active edge 16 and accumulated on 18.
        act_in_packed = '0;
        data_valid_in = 1'b0;
        for (n = 0; n < ARRAY_COLS; n = n + 1)
            wgt_in[n] = '0;

        repeat (11) @(posedge clk); // through active edge 20
        #2;

        $display("----- Final results -----");
        for (n = 0; n < ARRAY_COLS; n = n + 1) begin
            expected = 45 * (n + 1);
            got = $signed(acc_out_packed[n*ACC_WIDTH +: ACC_WIDTH]);
            $display("C[%0d] = %0d (expected %0d)", n, got, expected);
            if (got !== expected)
                $fatal(1, "FAIL: column %0d got %0d, expected %0d",
                       n, got, expected);
        end

        if ($signed(dut.gen_row[0].gen_col[0].u_pe.acc_reg) !== 32'sd10)
            $fatal(1, "FAIL: PE[0][0] did not accumulate 1+9");
        if ($signed(dut.gen_row[0].gen_col[7].u_pe.acc_reg) !== 32'sd80)
            $fatal(1, "FAIL: PE[0][7] did not accumulate (1+9)*8");
        if (any_overflow !== 1'b0)
            $fatal(1, "FAIL: unexpected overflow");

        $display("PASS: M=1 N=8 K=9; C=[45,90,135,180,225,270,315,360]");
        $finish;
    end
endmodule
