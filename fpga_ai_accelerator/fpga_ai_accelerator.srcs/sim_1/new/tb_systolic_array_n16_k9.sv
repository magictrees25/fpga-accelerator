`timescale 1ns / 1ps

// N-Tiling verification for taitashaw/fpga-ai-accelerator.
// GEMM: M=1, N=16, K=9
//   A[0,k] = k+1, k=0..8
//   B[k,n] = n+1, k=0..8, n=0..15
//   C[0,n] = 45*(n+1)
//
// The DUT has 8 PE columns. The TB executes two N-tiles:
//   tile 0 -> output channels N0..N7
//   tile 1 -> output channels N8..N15
// Within each N-tile K=9 exercises PE-row reuse (K0 and K8 on row 0).
// The TB saves the first N-tile BEFORE pulsing clear_acc, then checks
// that the second N-tile starts from zero and both groups are correct.
//
// IMPORTANT: Tile scheduling / result storage here belong to this TB,
// not an automatic tiling controller within systolic_array.sv.

module tb_systolic_array_n16_k9;
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

    // Convenient variables to observe in Vivado's waveform window.
    integer n_phase;                    // 0: N0..N7, 1: clear, 2: N8..N15
    integer saved_C [0:15];
    integer edge_count;

    initial clk = 1'b0;
    always #5 clk = ~clk;  // 100 MHz test clock

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

    // Debug trace, sampled after the registers update.
    always @(posedge clk) begin
        if (rst_n && enable) begin
            edge_count = edge_count + 1;
            #1;
            $display("edge=%0d phase=%0d clear=%0b col0=%0d col7=%0d valid=%0b",
                edge_count, n_phase, clear_acc,
                $signed(acc_out_packed[0*ACC_WIDTH +: ACC_WIDTH]),
                $signed(acc_out_packed[7*ACC_WIDTH +: ACC_WIDTH]),
                data_valid_out);
        end
    end

    // Drive a complete K=9 dot-product for the chosen group of 8 channels.
    // Start on a falling clock edge so inputs are stable before next posedge.
    task automatic drive_n_tile(input integer tile_index);
        integer k;
        integer col;
        integer physical_weight;
        begin
            for (k = 0; k < 9; k = k + 1) begin
                act_in_packed = '0;
                // K=9 => the last element reuses PE row 0.
                act_in_packed[(k % ARRAY_ROWS)*ACT_WIDTH +: ACT_WIDTH] = k + 1;

                for (col = 0; col < ARRAY_COLS; col = col + 1) begin
                    physical_weight = tile_index*ARRAY_COLS + col + 1;
                    wgt_in[col] = physical_weight;
                end
                data_valid_in = (k == 0);
                @(negedge clk);
            end

            // Flush the input and weight-skew pipelines with zeros.
            act_in_packed = '0;
            data_valid_in = 1'b0;
            for (col = 0; col < ARRAY_COLS; col = col + 1)
                wgt_in[col] = '0;
        end
    endtask

    // Check one group of 8 channels and save their final results.
    task automatic check_and_save_tile(input integer tile_index);
        integer col;
        integer channel;
        integer expected;
        integer actual;
        integer expected_pe00;
        integer expected_pe07;
        begin
            for (col = 0; col < ARRAY_COLS; col = col + 1) begin
                channel  = tile_index*ARRAY_COLS + col;
                expected = 45*(channel + 1);
                actual   = $signed(acc_out_packed[col*ACC_WIDTH +: ACC_WIDTH]);
                $display("C[0][%0d] = %0d (expected %0d)",
                         channel, actual, expected);
                if (actual !== expected)
                    $fatal(1, "FAIL: N=%0d actual=%0d expected=%0d",
                           channel, actual, expected);
                saved_C[channel] = actual;
            end

            // PE row 0 accumulates k=0 and k=8: (1+9)*weight.
            expected_pe00 = 10*(tile_index*ARRAY_COLS + 1);
            expected_pe07 = 10*(tile_index*ARRAY_COLS + 8);
            if ($signed(dut.gen_row[0].gen_col[0].u_pe.acc_reg)
                    !== expected_pe00)
                $fatal(1, "FAIL: PE[0][0] acc expected %0d", expected_pe00);
            if ($signed(dut.gen_row[0].gen_col[7].u_pe.acc_reg)
                    !== expected_pe07)
                $fatal(1, "FAIL: PE[0][7] acc expected %0d", expected_pe07);
            if (any_overflow !== 1'b0)
                $fatal(1, "FAIL: unexpected overflow in N-tile %0d", tile_index);
        end
    endtask

    integer i;
    initial begin
        rst_n         = 1'b0;
        enable        = 1'b0;
        clear_acc     = 1'b0;
        valid_reset   = 1'b0;
        data_valid_in = 1'b0;
        act_in_packed = '0;
        n_phase       = -1;
        edge_count    = 0;
        for (i = 0; i < ARRAY_COLS; i = i + 1)
            wgt_in[i] = '0;
        for (i = 0; i < 16; i = i + 1)
            saved_C[i] = 0;

        repeat (3) @(negedge clk);
        rst_n  = 1'b1;
        enable = 1'b1;

        // ------------ N-tile 0: channels 0..7 ------------
        n_phase = 0;
        $display("===== N-tile 0: N0..N7 =====");
        drive_n_tile(0);
        repeat (11) @(posedge clk);
        #2;
        check_and_save_tile(0);

        // Important: save the first eight results before clearing.
        @(negedge clk);
        n_phase  = 1;
        clear_acc = 1'b1;
        @(posedge clk);
        #2;
        if (acc_out_packed !== '0)
            $fatal(1, "FAIL: clear_acc did not zero all column sums");
        $display("CLEAR: 8 PE-column sums are zero");
        @(negedge clk);
        clear_acc = 1'b0;

        // ------------ N-tile 1: channels 8..15 ------------
        n_phase = 2;
        $display("===== N-tile 1: N8..N15 =====");
        drive_n_tile(1);
        repeat (11) @(posedge clk);
        #2;
        check_and_save_tile(1);

        $display("===== Final 16 outputs (two stored N-tiles) =====");
        for (i = 0; i < 16; i = i + 1) begin
            $display("C[0][%0d]=%0d", i, saved_C[i]);
            if (saved_C[i] !== 45*(i+1))
                $fatal(1, "FAIL: saved_C[%0d] got %0d expected %0d",
                       i, saved_C[i], 45*(i+1));
        end

        $display("PASS: M=1 N=16 K=9; both N-tiles correct (45,90,...,720)");
        $finish;
    end
endmodule
