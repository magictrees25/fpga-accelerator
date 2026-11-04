
`timescale 1ns / 1ps

module tb_conv_weight_bram_reader_p2_opt;

    // ============================================================
    // Parameters: fixed P2 configuration
    // ============================================================
    localparam int K_LEN = 576;
    localparam int N_TILES = 1;
    localparam int ADDR_WIDTH = 16;
    localparam int N = 8;
    localparam int PAIR_COUNT = K_LEN / 2;

    // ============================================================
    // Clock and common inputs
    // ============================================================
    logic clk = 0;
    always #5 clk = ~clk;

    logic rst_n;
    logic [ADDR_WIDTH-1:0] n_base;
    logic pair_req_en;
    logic [ADDR_WIDTH-1:0] fetch_k_base;

    logic wr_en;
    logic [ADDR_WIDTH-1:0] wr_addr;
    logic [63:0] wr_data;

    // ============================================================
    // Original Reader outputs
    // ============================================================
    logic [127:0] ref_wgt_pair;
    logic [1:0] ref_rsp_valid;

    // ============================================================
    // Optimized Reader outputs
    // ============================================================
    logic [127:0] opt_wgt_pair;
    logic [1:0] opt_rsp_valid;

    // ============================================================
    // Golden Reference
    // Each K word contains 8 INT8 weights
    // ============================================================
    logic [63:0] expected_word [0:K_LEN-1];

    integer checked_count;

    // ============================================================
    // Generate test weights
    // Include -128, 0 and 127 boundary values
    // ============================================================
    function automatic logic [7:0] make_weight_byte(
        input integer k,
        input integer n
    );
        integer value;
        begin
            if (k == 0 && n == 0)
                value = -128;
            else if (k == K_LEN-1 && n == N-1)
                value = 127;
            else if (((k*7 + n*13) % 23) == 0)
                value = 0;
            else
                value = ((k*37 + n*19 + 11) % 256) - 128;

            make_weight_byte = value[7:0];
        end
    endfunction

    // ============================================================
    // DUT 1: Original P2 Weight Reader
    // ============================================================
    conv_weight_bram_reader_p2 #(
        .K_LEN(K_LEN),
        .N_TILES(N_TILES),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_ref (
        .clk(clk),
        .rst_n(rst_n),
        .n_base(n_base),
        .pair_req_en(pair_req_en),
        .fetch_k_base(fetch_k_base),
        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_data(wr_data),
        .wgt_pair_packed(ref_wgt_pair),
        .wgt_rsp_valid(ref_rsp_valid)
    );

    // ============================================================
    // DUT 2: Optimized P2 Weight Reader
    // ============================================================
    conv_weight_bram_reader_p2_opt #(
        .K_LEN(K_LEN),
        .N_TILES(N_TILES),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_opt (
        .clk(clk),
        .rst_n(rst_n),
        .n_base(n_base),
        .pair_req_en(pair_req_en),
        .fetch_k_base(fetch_k_base),
        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_data(wr_data),
        .wgt_pair_packed(opt_wgt_pair),
        .wgt_rsp_valid(opt_rsp_valid)
    );

    // ============================================================
    // Check idle / invalid response
    // ============================================================
    task automatic check_idle(input string phase);
        begin
            if (ref_rsp_valid !== 2'b00 ||
                opt_rsp_valid !== 2'b00)
                $fatal(1, "FAIL %s: response valid not cleared", phase);

            if (ref_wgt_pair !== 128'b0 ||
                opt_wgt_pair !== 128'b0)
                $fatal(1, "FAIL %s: output not zero when invalid", phase);

            if (u_ref.bank_rd_en !== 2'b00 ||
                u_opt.bank_rd_en !== 2'b00)
                $fatal(1, "FAIL %s: BRAM read enable not cleared", phase);
        end
    endtask

    // ============================================================
    // Check one K-pair response against both DUTs and Golden Model
    //
    // K even -> Bank0
    // K+1 odd -> Bank1
    //
    // Output:
    // [63:0]   = weight[K][0:7]
    // [127:64] = weight[K+1][0:7]
    // ============================================================
    task automatic check_pair(
        input integer k,
        input string phase
    );
        logic [127:0] expected_pair;
        begin
            expected_pair = {
                expected_word[k+1],
                expected_word[k]
            };

            // Both Readers must respond in the same cycle.
            if (ref_rsp_valid !== 2'b11)
                $fatal(1,
                    "FAIL %s: original valid=%b K=%0d",
                    phase, ref_rsp_valid, k);

            if (opt_rsp_valid !== 2'b11)
                $fatal(1,
                    "FAIL %s: optimized valid=%b K=%0d",
                    phase, opt_rsp_valid, k);

            // Independent Golden Reference comparison.
            if (ref_wgt_pair !== expected_pair)
                $fatal(1,
                    "FAIL %s: original data mismatch K=%0d",
                    phase, k);

            if (opt_wgt_pair !== expected_pair)
                $fatal(1,
                    "FAIL %s: optimized data mismatch K=%0d",
                    phase, k);

            // Direct comparison between the two DUTs.
            if (ref_wgt_pair !== opt_wgt_pair)
                $fatal(1,
                    "FAIL %s: original/optimized mismatch K=%0d",
                    phase, k);

            // Check request legality.
            if (u_ref.request_legal !== 1'b1 ||
                u_opt.request_legal !== 1'b1)
                $fatal(1,
                    "FAIL %s: request_legal is low K=%0d",
                    phase, k);

            // Check request acceptance.
            if (u_ref.pair_accept !== 1'b1 ||
                u_opt.pair_accept !== 1'b1)
                $fatal(1,
                    "FAIL %s: pair_accept is low K=%0d",
                    phase, k);

            // Both banks must be enabled.
            if (u_ref.bank_rd_en !== 2'b11 ||
                u_opt.bank_rd_en !== 2'b11)
                $fatal(1,
                    "FAIL %s: bank read enable mismatch K=%0d",
                    phase, k);

            // Both banks read address K/2.
            if (u_ref.bank_rd_addr[0] !== 16'(k/2) ||
                u_ref.bank_rd_addr[1] !== 16'(k/2) ||
                u_opt.bank_rd_addr[0] !== 16'(k/2) ||
                u_opt.bank_rd_addr[1] !== 16'(k/2))
                $fatal(1,
                    "FAIL %s: bank address mismatch K=%0d",
                    phase, k);

            // Response tags must also agree.
            if (u_ref.rsp_k_base !== 16'(k) ||
                u_opt.rsp_k_base !== 16'(k) ||
                u_ref.rsp_n_base !== 16'd0 ||
                u_opt.rsp_n_base !== 16'd0)
                $fatal(1,
                    "FAIL %s: response tag mismatch K=%0d",
                    phase, k);

            checked_count = checked_count + 1;
        end
    endtask

    // ============================================================
    // Main Test
    // ============================================================
    integer k;
    integer n;

    initial begin

        // --------------------------------------------------------
        // Initialize
        // --------------------------------------------------------
        rst_n = 0;
        n_base = 0;
        pair_req_en = 0;
        fetch_k_base = 0;

        wr_en = 0;
        wr_addr = 0;
        wr_data = 0;

        checked_count = 0;

        // --------------------------------------------------------
        // Generate Golden Reference data
        // --------------------------------------------------------
        for (k = 0; k < K_LEN; k = k + 1) begin
            for (n = 0; n < N; n = n + 1) begin
                expected_word[k][n*8 +: 8] =
                    make_weight_byte(k, n);
            end
        end

        // --------------------------------------------------------
        // Reset
        // --------------------------------------------------------
        repeat (3) @(posedge clk);
        #1;
        check_idle("reset");

        @(negedge clk);
        rst_n = 1;

        // --------------------------------------------------------
        // Step 1: Preload all 576 weight words
        // Both DUTs receive identical write transactions.
        // --------------------------------------------------------
        $display("INFO: Start weight BRAM preload");

        for (k = 0; k < K_LEN; k = k + 1) begin
            @(negedge clk);

            wr_en = 1;
            wr_addr = 16'(k);
            wr_data = expected_word[k];
        end

        @(negedge clk);
        wr_en = 0;
        wr_addr = 0;
        wr_data = 0;

        @(posedge clk);
        #1;
        check_idle("after preload");

        $display("PASS: 576 weight words loaded");

        // --------------------------------------------------------
        // Step 2: Continuous ascending K-pair reads
        //
        // K = 0, 2, 4, ... 574
        // One request per clock cycle.
        // --------------------------------------------------------
        $display("INFO: Start continuous ascending K-pair reads");

        for (k = 0; k < K_LEN; k = k + 2) begin

            @(negedge clk);

            pair_req_en = 1;
            n_base = 0;
            fetch_k_base = 16'(k);
            wr_en = 0;

            @(posedge clk);
            #1;

            check_pair(k, "ascending");
        end

        // Stop requests and check response clearing.
        @(negedge clk);
        pair_req_en = 0;
        fetch_k_base = 16'hffff;
        n_base = 16'hffff;

        @(posedge clk);
        #1;
        check_idle("ascending end");

        $display("PASS: 288 continuous ascending K-pair reads");

        // --------------------------------------------------------
        // Step 3: Descending K-pair reads with idle bubbles
        //
        // K = 574, 572, ... 0
        // One idle cycle between requests.
        // --------------------------------------------------------
        $display("INFO: Start descending K-pair reads with bubbles");

        for (k = K_LEN-2; k >= 0; k = k - 2) begin

            @(negedge clk);

            pair_req_en = 1;
            n_base = 0;
            fetch_k_base = 16'(k);

            @(posedge clk);
            #1;

            check_pair(k, "descending");

            // Insert one idle cycle.
            @(negedge clk);

            pair_req_en = 0;
            fetch_k_base = 16'hffff;
            n_base = 16'hffff;

            @(posedge clk);
            #1;

            check_idle("bubble");
        end

        $display("PASS: 288 descending K-pair reads with bubbles");

        // --------------------------------------------------------
        // Step 4: Asynchronous reset
        // BRAM contents should remain available after reset.
        // --------------------------------------------------------
        @(negedge clk);

        rst_n = 0;
        pair_req_en = 0;
        n_base = 0;
        fetch_k_base = 0;

        #1;
        check_idle("asynchronous reset");

        repeat (2) @(posedge clk);

        @(negedge clk);
        rst_n = 1;

        // Read again after reset to verify BRAM contents.
        @(negedge clk);
        pair_req_en = 1;
        n_base = 0;
        fetch_k_base = 0;

        @(posedge clk);
        #1;

        check_pair(0, "post-reset");

        @(negedge clk);
        pair_req_en = 0;
        fetch_k_base = 16'hffff;
        n_base = 16'hffff;

        @(posedge clk);
        #1;
        check_idle("post-reset idle");

        // --------------------------------------------------------
        // Final result
        // --------------------------------------------------------
        if (checked_count != 577)
            $fatal(1,
                "FAIL: checked_count=%0d, expected=577",
                checked_count);

        $display("----------------------------------------------");
        $display("PASS: P2 Weight Reader comparison completed");
        $display("PASS: Original and optimized outputs identical");
        $display("PASS: All 288 legal K-pairs checked ascending");
        $display("PASS: All 288 legal K-pairs checked descending");
        $display("PASS: Continuous and bubble reads verified");
        $display("PASS: Reset and BRAM data retention verified");
        $display("PASS: Golden Reference comparison successful");
        $display("TOTAL CHECKED PAIRS = %0d", checked_count);
        $display("----------------------------------------------");

        $finish;
    end

    // ------------------------------------------------------------
    // Simulation watchdog
    // ------------------------------------------------------------
    initial begin
        #100000;
        $fatal(1, "TIMEOUT: Weight Reader test did not finish");
    end

endmodule
