`timescale 1ns / 1ps

// K=9 dot-product test for taitashaw/fpga-ai-accelerator.
// A = [1 2 3 4 5 6 7 8 9]
// B = [1 1 1 1 1 1 1 1 1]^T
// Expected output at column 0: 45; all other columns: 0.
// This tests accumulation across an 8-row boundary (K-tile 0 + K-tile 1).
module tb_systolic_array_k9;
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
    always #5 clk = ~clk;  // 10 ns period

    systolic_array dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .enable         (enable),
        .clear_acc      (clear_acc),
        .valid_reset    (valid_reset),
        .act_in_packed  (act_in_packed),
        .wgt_in         (wgt_in),
        .data_valid_in  (data_valid_in),
        .data_valid_out (data_valid_out),
        .acc_out_packed (acc_out_packed),
        .any_overflow   (any_overflow)
    );

    integer k;
    integer c;
    integer edge_count = 0;

    always @(posedge clk) begin
        if (rst_n && enable) begin
            edge_count = edge_count + 1;
            #1;
            $display("edge=%0d | PE00.acc=%0d | col_sum[0]=%0d | valid=%0b",
                     edge_count,
                     $signed(dut.gen_row[0].gen_col[0].u_pe.acc_reg),
                     $signed(acc_out_packed[0 +: ACC_WIDTH]),
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
        for (c = 0; c < ARRAY_COLS; c = c + 1)
            wgt_in[c] = '0;

        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        enable = 1'b1;

        // Nine input beats. The first eight use rows 0..7;
        // beat 9 reuses row 0, while PE accumulators remain uncleared.
        // Present all inputs on falling edges, ahead of sampling rising edges.
        for (k = 0; k < 9; k = k + 1) begin
            act_in_packed = '0;
            act_in_packed[(k % ARRAY_ROWS)*ACT_WIDTH +: ACT_WIDTH] = k + 1;
            wgt_in[0] = 8'sd1;
            data_valid_in = (k == 0);
            @(negedge clk);
        end

        // No new valid data after beat 9. Drain the 3-stage MAC pipeline.
        act_in_packed = '0;
        data_valid_in = 1'b0;
        for (c = 0; c < ARRAY_COLS; c = c + 1)
            wgt_in[c] = '0;

        // Last useful beat is captured at active edge 9;
        // its result is accumulated at active edge 11.
        repeat (2) @(posedge clk);
        #2; // settle nonblocking assignments and combinational column sum

        if ($signed(dut.gen_row[0].gen_col[0].u_pe.acc_reg) !== 32'sd10)
            $fatal(1, "FAIL: row-0 PE accumulated %0d, expected 10 (1 + 9)",
                   $signed(dut.gen_row[0].gen_col[0].u_pe.acc_reg));

        if ($signed(acc_out_packed[0 +: ACC_WIDTH]) !== 32'sd45)
            $fatal(1, "FAIL: column-0 sum %0d, expected 45",
                   $signed(acc_out_packed[0 +: ACC_WIDTH]));

        for (c = 1; c < ARRAY_COLS; c = c + 1) begin
            if (acc_out_packed[c*ACC_WIDTH +: ACC_WIDTH] !== 32'd0)
                $fatal(1, "FAIL: unexpected nonzero result in column %0d", c);
        end

        if (any_overflow !== 1'b0)
            $fatal(1, "FAIL: unexpected PE overflow");

        $display("PASS: K=9 result = 45; PE[0][0] acc = 10 (1 + 9)");
        $finish;
    end
endmodule
