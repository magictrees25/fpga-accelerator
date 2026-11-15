`timescale 1ns / 1ps

module tb_conv_bram_reader_p3_shared;

    localparam int H_IN = 10;
    localparam int W_IN = 10;
    localparam int C_IN = 64;

    localparam int M_COUNT = 64;
    localparam int K_COUNT = 576;
    localparam int ADDR_WIDTH = 16;

    localparam int TOTAL_BYTES = H_IN * W_IN * C_IN;
    localparam int PAIRS_PER_ROW = W_IN * (C_IN / 2);
    localparam int EXPECTED_PREFETCH_PAIRS = 6 * PAIRS_PER_ROW;

    logic clk;
    logic rst_n;

    logic [15:0] m_addr;
    logic pair_req_en;
    logic [15:0] fetch_k_base;

    logic wr_en;
    logic [15:0] wr_addr;
    logic [7:0] wr_data;

    logic [15:0] fmap_rd_data;
    logic fmap_rd_valid;
    logic fmap_rd_en;
    logic [ADDR_WIDTH-1:0] fmap_rd_addr;

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
    integer shared_read_count;
    integer external_addr_check_count;
    integer expected_pair_addr;

    logic [7:0] expected_even;
    logic [7:0] expected_odd;

    // --------------------------------------------------------
    // Clock
    // --------------------------------------------------------

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // --------------------------------------------------------
    // Deterministic activation data pattern
    // Keep identical to the original P3 local-mode testbench.
    // --------------------------------------------------------

    function automatic logic [7:0] fmap_pattern(input integer addr);
        integer temp;
        begin
            temp = (addr * 73 + (addr >> 4) * 19 + 8'h31) & 255;
            fmap_pattern = temp[7:0];
        end
    endfunction

    // --------------------------------------------------------
    // P3 Reader: external shared-memory mode
    // --------------------------------------------------------

    conv_bram_reader_p3 #(
        .H_IN(H_IN),
        .W_IN(W_IN),
        .C_IN(C_IN),
        .ADDR_WIDTH(ADDR_WIDTH),
        .EXTERNAL_ACT_MEM(1'b1)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),

        .m_addr(m_addr),
        .pair_req_en(pair_req_en),
        .fetch_k_base(fetch_k_base),

        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_data(wr_data),

        .fmap_rd_data(fmap_rd_data),
        .fmap_rd_valid(fmap_rd_valid),
        .fmap_rd_en(fmap_rd_en),
        .fmap_rd_addr(fmap_rd_addr),

        .act_pair_packed(act_pair_packed),
        .act_rsp_valid(act_rsp_valid)
    );

    // --------------------------------------------------------
    // Shared Activation BRAM
    // Host writes are broadcast to the shared BRAM.
    // P3 also sees the same writes to initialize its 4-row LB.
    // --------------------------------------------------------

    conv_activation_bram_shared_p5 #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .MAX_BYTES(TOTAL_BYTES)
    ) u_shared_act_mem (
        .clk(clk),
        .rst_n(rst_n),

        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_data(wr_data),

        .rd_en(fmap_rd_en),
        .rd_addr(fmap_rd_addr),
        .rd_data(fmap_rd_data),
        .rd_valid(fmap_rd_valid)
    );

    // --------------------------------------------------------
    // Monitor external prefetch requests and Line Buffer writes
    // --------------------------------------------------------

    always @(posedge clk) begin
        if (rst_n) begin

            if (dut.prefetch_trigger)
                prefetch_trigger_count = prefetch_trigger_count + 1;

            if (dut.prefetch_issue)
                prefetch_read_count = prefetch_read_count + 1;

            if (fmap_rd_en)
                shared_read_count = shared_read_count + 1;

            // Count actual full-pair Line Buffer writes, not just
            // the delayed request tag.
            if (dut.lb_wr_en == 2'b11)
                prefetch_write_count = prefetch_write_count + 1;

            // Shared BRAM address is a pair address:
            // row * 320 + pair_index_within_row.
            if (fmap_rd_en) begin
                expected_pair_addr =
                    dut.prefetch_row * PAIRS_PER_ROW + dut.prefetch_index;

                if (fmap_rd_addr !== ADDR_WIDTH'(expected_pair_addr)) begin
                    $fatal(1,
                        "FAIL: shared address mismatch row=%0d index=%0d expected=%0d actual=%0d",
                        dut.prefetch_row,
                        dut.prefetch_index,
                        expected_pair_addr,
                        fmap_rd_addr
                    );
                end

                external_addr_check_count = external_addr_check_count + 1;
            end

            // Both response-valid pipelines must describe the same
            // one-cycle response from the shared Activation BRAM.
            if (fmap_rd_valid !== dut.prefetch_pending) begin
                $fatal(1,
                    "FAIL: shared rd_valid/prefetch_pending mismatch shared=%b pending=%b",
                    fmap_rd_valid,
                    dut.prefetch_pending
                );
            end

        end
    end

    // --------------------------------------------------------
    // Main test
    // --------------------------------------------------------

    initial begin

        rst_n = 1'b0;

        m_addr = '0;
        pair_req_en = 1'b0;
        fetch_k_base = '0;

        wr_en = 1'b0;
        wr_addr = '0;
        wr_data = '0;

        checked_pairs = 0;
        prefetch_trigger_count = 0;
        prefetch_read_count = 0;
        prefetch_write_count = 0;
        shared_read_count = 0;
        external_addr_check_count = 0;
        expected_pair_addr = 0;

        $display("========================================");
        $display("P3 Shared Activation BRAM Verification");
        $display("========================================");

        repeat (3) @(negedge clk);
        rst_n = 1'b1;

        // ----------------------------------------------------
        // TEST 1: Load the complete activation feature map.
        // The same writes feed Shared BRAM and P3's initial 4-row LB.
        // ----------------------------------------------------

        $display("----------------------------------------");
        $display("TEST 1: Shared Activation BRAM preload");
        $display("----------------------------------------");

        for (i = 0; i < TOTAL_BYTES; i = i + 1) begin
            @(negedge clk);
            wr_en = 1'b1;
            wr_addr = ADDR_WIDTH'(i);
            wr_data = fmap_pattern(i);

            @(posedge clk);
            #1;
        end

        @(negedge clk);
        wr_en = 1'b0;
        wr_addr = '0;
        wr_data = '0;

        repeat (2) @(posedge clk);
        #1;

        $display("PASS: %0d activation bytes loaded into Shared BRAM", TOTAL_BYTES);
        $display("PASS: Initial 4-row Line Buffer preload completed");

        // ----------------------------------------------------
        // TEST 2: Verify M/K sliding-window data from the Line Buffer.
        // Prefetches for input rows 4..9 occur during this scan.
        // ----------------------------------------------------

        $display("----------------------------------------");
        $display("TEST 2: M/K sliding-window data check");
        $display("----------------------------------------");

        for (m = 0; m < M_COUNT; m = m + 1) begin
            for (k = 0; k < K_COUNT; k = k + 2) begin

                @(negedge clk);
                m_addr = 16'(m);
                fetch_k_base = 16'(k);
                pair_req_en = 1'b1;

                @(posedge clk);
                #1;

                kernel_index = k / 64;
                ky = kernel_index / 3;
                kx = kernel_index % 3;
                ci = k % 64;

                expected_row = m / 8 + ky;
                expected_col = m % 8 + kx;

                byte_addr0 =
                    ((expected_row * W_IN + expected_col) * C_IN) + ci;
                byte_addr1 = byte_addr0 + 1;

                expected_even = fmap_pattern(byte_addr0);
                expected_odd  = fmap_pattern(byte_addr1);

                if (act_rsp_valid !== 2'b11) begin
                    $fatal(1,
                        "FAIL: invalid response M=%0d K=%0d valid=%b",
                        m, k, act_rsp_valid
                    );
                end

                if (act_pair_packed[7:0] !== expected_even) begin
                    $fatal(1,
                        "FAIL: EVEN M=%0d K=%0d got=%h expected=%h",
                        m, k, act_pair_packed[7:0], expected_even
                    );
                end

                if (act_pair_packed[15:8] !== expected_odd) begin
                    $fatal(1,
                        "FAIL: ODD M=%0d K=%0d got=%h expected=%h",
                        m, k, act_pair_packed[15:8], expected_odd
                    );
                end

                checked_pairs = checked_pairs + 1;

            end

            if ((m % 8) == 7) begin
                $display("PASS: Output row %0d completed", m / 8);
            end
        end

        // Stop compute requests and allow any final response/write to drain.
        @(negedge clk);
        pair_req_en = 1'b0;
        repeat (3) @(posedge clk);
        #1;

        // ----------------------------------------------------
        // TEST 3: Verify external shared-BRAM prefetch sequence.
        // ----------------------------------------------------

        $display("----------------------------------------");
        $display("TEST 3: External Shared BRAM row prefetch");
        $display("----------------------------------------");

        if (prefetch_trigger_count != 6)
            $fatal(1,
                "FAIL: prefetch trigger count=%0d expected=6",
                prefetch_trigger_count
            );

        if (prefetch_read_count != EXPECTED_PREFETCH_PAIRS)
            $fatal(1,
                "FAIL: P3 prefetch issue count=%0d expected=%0d",
                prefetch_read_count,
                EXPECTED_PREFETCH_PAIRS
            );

        if (shared_read_count != EXPECTED_PREFETCH_PAIRS)
            $fatal(1,
                "FAIL: Shared BRAM read count=%0d expected=%0d",
                shared_read_count,
                EXPECTED_PREFETCH_PAIRS
            );

        if (prefetch_write_count != EXPECTED_PREFETCH_PAIRS)
            $fatal(1,
                "FAIL: Line Buffer full-pair write count=%0d expected=%0d",
                prefetch_write_count,
                EXPECTED_PREFETCH_PAIRS
            );

        if (external_addr_check_count != EXPECTED_PREFETCH_PAIRS)
            $fatal(1,
                "FAIL: external address check count=%0d expected=%0d",
                external_addr_check_count,
                EXPECTED_PREFETCH_PAIRS
            );

        if (dut.prefetch_busy !== 1'b0)
            $fatal(1, "FAIL: prefetch engine still busy");

        if (dut.prefetch_pending !== 1'b0)
            $fatal(1, "FAIL: pending BRAM response remains");

        if (fmap_rd_valid !== 1'b0)
            $fatal(1, "FAIL: Shared BRAM response-valid did not clear");

        if (checked_pairs != M_COUNT * (K_COUNT / 2))
            $fatal(1, "FAIL: incorrect checked pair count=%0d", checked_pairs);

        if (act_rsp_valid !== 2'b00)
            $fatal(1, "FAIL: response valid did not clear");

        $display("PASS: 6 prefetch operations completed");
        $display("PASS: %0d P3 prefetch pair requests", prefetch_read_count);
        $display("PASS: %0d Shared BRAM pair reads", shared_read_count);
        $display("PASS: %0d Line Buffer pair writes", prefetch_write_count);
        $display("PASS: %0d external pair addresses checked", external_addr_check_count);

        $display("========================================");
        $display("PASS: P3 Shared Activation BRAM Reader");
        $display("PASS: Shared Activation BRAM preload");
        $display("PASS: Initial 4-row Line Buffer preload");
        $display("PASS: M/K address mapping");
        $display("PASS: Sliding Window data");
        $display("PASS: External BRAM row prefetch");
        $display("PASS: Line Buffer data refill");
        $display("========================================");

        $display("TOTAL CHECKED PAIRS = %0d", checked_pairs);
        $display("TOTAL CHECKED K ELEMENTS = %0d", checked_pairs * 2);
        $display("TOTAL PREFETCHED ROWS = %0d", prefetch_trigger_count);
        $display("TOTAL SHARED BRAM PAIR READS = %0d", shared_read_count);
        $display("========================================");

        $finish;

    end

    // --------------------------------------------------------
    // Watchdog
    // --------------------------------------------------------

    initial begin
        repeat (30000) @(posedge clk);
        $fatal(1, "TIMEOUT: P3 Shared Activation BRAM TB");
    end

endmodule
