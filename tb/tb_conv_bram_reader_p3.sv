
`timescale 1ns / 1ps

module tb_conv_bram_reader_p3;

    localparam int H_IN = 10;
    localparam int W_IN = 10;
    localparam int C_IN = 64;

    localparam int M_COUNT = 64;
    localparam int K_COUNT = 576;
    localparam int ADDR_WIDTH = 16;

    localparam int TOTAL_BYTES =
        H_IN * W_IN * C_IN;

    localparam int PAIRS_PER_ROW =
        W_IN * (C_IN / 2);

    logic clk;
    logic rst_n;

    logic [15:0] m_addr;
    logic pair_req_en;
    logic [15:0] fetch_k_base;

    logic wr_en;
    logic [15:0] wr_addr;
    logic [7:0] wr_data;

    logic [15:0] act_pair_packed;
    logic [1:0] act_rsp_valid;

    integer i;
    integer m;
    integer k;

    integer kernel_index;
    integer ky;
    integer kx;
    integer ci;

    integer expected_row;
    integer expected_col;

    integer byte_addr0;
    integer byte_addr1;

    integer checked_pairs;

    integer prefetch_trigger_count;
    integer prefetch_read_count;
    integer prefetch_write_count;

    logic [7:0] expected_even;
    logic [7:0] expected_odd;

    // ========================================================
    // 1. Clock
    // ========================================================

    initial clk = 0;
    always #5 clk = ~clk;

    // ========================================================
    // 2. Generate deterministic activation data
    // ========================================================

    function automatic logic [7:0] fmap_pattern(
        input integer addr
    );

        integer temp;

        begin
            temp = (
                addr * 73 +
                (addr >> 4) * 19 +
                8'h31
            ) & 255;

            fmap_pattern = temp[7:0];
        end

    endfunction

    // ========================================================
    // 3. DUT
    // ========================================================

    conv_bram_reader_p3 #(
        .H_IN(H_IN),
        .W_IN(W_IN),
        .C_IN(C_IN),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) dut (

        .clk(clk),
        .rst_n(rst_n),

        .m_addr(m_addr),
        .pair_req_en(pair_req_en),
        .fetch_k_base(fetch_k_base),

        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_data(wr_data),

        .act_pair_packed(act_pair_packed),
        .act_rsp_valid(act_rsp_valid)

    );

    // ========================================================
    // 4. Monitor prefetch activity
    // ========================================================

    always @(posedge clk) begin

        if (rst_n) begin

            if (dut.prefetch_trigger)
                prefetch_trigger_count =
                    prefetch_trigger_count + 1;

            if (dut.prefetch_issue)
                prefetch_read_count =
                    prefetch_read_count + 1;

            if (dut.prefetch_pending)
                prefetch_write_count =
                    prefetch_write_count + 1;

        end

    end

    // ========================================================
    // 5. Main test
    // ========================================================

    initial begin

        rst_n = 0;

        m_addr = 0;
        pair_req_en = 0;
        fetch_k_base = 0;

        wr_en = 0;
        wr_addr = 0;
        wr_data = 0;

        checked_pairs = 0;

        prefetch_trigger_count = 0;
        prefetch_read_count = 0;
        prefetch_write_count = 0;

        // ----------------------------------------------------
        // Reset
        // ----------------------------------------------------

        $display("====================================");
        $display("P3 Activation Reader Verification");
        $display("====================================");

        repeat (3) @(negedge clk);

        rst_n = 1;

        // ----------------------------------------------------
        // TEST 1: Load complete activation feature map
        // ----------------------------------------------------

        $display("------------------------------------");
        $display("TEST 1: Activation BRAM preload");
        $display("------------------------------------");

        for (i = 0; i < TOTAL_BYTES; i = i + 1) begin

            @(negedge clk);

            wr_en = 1;
            wr_addr = 16'(i);
            wr_data = fmap_pattern(i);

            @(posedge clk);
            #1;

        end

        @(negedge clk);
        wr_en = 0;

        $display("PASS: %0d activation bytes loaded",
                 TOTAL_BYTES);

        $display("PASS: Initial 4-row Line Buffer preload completed");

        // ----------------------------------------------------
        // TEST 2: Verify M/K address mapping and data
        // ----------------------------------------------------

        $display("------------------------------------");
        $display("TEST 2: M/K sliding-window data check");
        $display("------------------------------------");

        for (m = 0; m < M_COUNT; m = m + 1) begin

            for (k = 0; k < K_COUNT; k = k + 2) begin

                // Drive one K-pair request
                @(negedge clk);

                m_addr = 16'(m);
                fetch_k_base = 16'(k);
                pair_req_en = 1;

                // Synchronous response
                @(posedge clk);
                #1;

                // Calculate expected input coordinates
                kernel_index = k / 64;

                ky = kernel_index / 3;
                kx = kernel_index % 3;

                ci = k % 64;

                expected_row = m / 8 + ky;
                expected_col = m % 8 + kx;

                // Original HWC byte address
                byte_addr0 =
                    ((expected_row * W_IN + expected_col)
                    * C_IN) + ci;

                byte_addr1 = byte_addr0 + 1;

                expected_even = fmap_pattern(byte_addr0);
                expected_odd  = fmap_pattern(byte_addr1);

                // Check valid
                if (act_rsp_valid !== 2'b11) begin

                    $fatal(1,
                        "FAIL: invalid response M=%0d K=%0d valid=%b",
                        m, k, act_rsp_valid
                    );

                end

                // Check even channel
                if (act_pair_packed[7:0] !== expected_even) begin

                    $fatal(1,
                        "FAIL: EVEN M=%0d K=%0d got=%h expected=%h",
                        m, k,
                        act_pair_packed[7:0],
                        expected_even
                    );

                end

                // Check odd channel
                if (act_pair_packed[15:8] !== expected_odd) begin

                    $fatal(1,
                        "FAIL: ODD M=%0d K=%0d got=%h expected=%h",
                        m, k,
                        act_pair_packed[15:8],
                        expected_odd
                    );

                end

                checked_pairs = checked_pairs + 1;

            end

            if ((m % 8) == 7) begin

                $display(
                    "PASS: Output row %0d completed",
                    m / 8
                );

            end

        end

        // Stop requests
        @(negedge clk);

        pair_req_en = 0;

        repeat (3) @(posedge clk);
        #1;

        // ----------------------------------------------------
        // TEST 3: Verify prefetch sequence
        // ----------------------------------------------------

        $display("------------------------------------");
        $display("TEST 3: BRAM row prefetch");
        $display("------------------------------------");

        if (prefetch_trigger_count != 6)
            $fatal(1,
                "FAIL: prefetch trigger count=%0d expected=6",
                prefetch_trigger_count
            );

        if (prefetch_read_count != 6 * PAIRS_PER_ROW)
            $fatal(1,
                "FAIL: BRAM read count=%0d expected=%0d",
                prefetch_read_count,
                6 * PAIRS_PER_ROW
            );

        if (prefetch_write_count != 6 * PAIRS_PER_ROW)
            $fatal(1,
                "FAIL: Line Buffer write count=%0d expected=%0d",
                prefetch_write_count,
                6 * PAIRS_PER_ROW
            );

        if (dut.prefetch_busy !== 1'b0)
            $fatal(1, "FAIL: prefetch engine still busy");

        if (dut.prefetch_pending !== 1'b0)
            $fatal(1, "FAIL: pending BRAM response remains");

        $display("PASS: 6 prefetch operations completed");

        $display("PASS: %0d BRAM pair reads",
                 prefetch_read_count);

        $display("PASS: %0d Line Buffer pair writes",
                 prefetch_write_count);

        // ----------------------------------------------------
        // TEST 4: Final result
        // ----------------------------------------------------

        if (checked_pairs != M_COUNT * (K_COUNT / 2))
            $fatal(1, "FAIL: incorrect checked pair count");

        if (act_rsp_valid !== 2'b00)
            $fatal(1, "FAIL: response valid did not clear");

        $display("====================================");
        $display("PASS: P3 Activation Reader");
        $display("PASS: Activation BRAM preload");
        $display("PASS: Initial 4-row Line Buffer preload");
        $display("PASS: M/K address mapping");
        $display("PASS: Sliding Window data");
        $display("PASS: BRAM row prefetch");
        $display("PASS: Line Buffer data refill");
        $display("====================================");

        $display("TOTAL CHECKED PAIRS = %0d",
                 checked_pairs);

        $display("TOTAL CHECKED K ELEMENTS = %0d",
                 checked_pairs * 2);

        $display("TOTAL PREFETCHED ROWS = %0d", 6);

        $display("====================================");

        $finish;

    end

endmodule
