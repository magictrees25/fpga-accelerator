
`timescale 1ns / 1ps

// ============================================================================
// Testbench: tb_p6b2_reader_global_k
// Project:   ResNet-18 FPGA Convolution Accelerator
//
// P6-B2-2:
//   Verify real Activation BRAM Reader and Weight BRAM Reader
//   with global K addressing.
//
// Configuration:
//   M       = 16
//   N       = 8
//   K       = 576
//   K-Tile  = 64
//
// Tests:
//   1. Preload real shared Activation BRAM.
//   2. Preload real shared Weight BRAM.
//   3. Read all 576 K elements using 288 pair requests.
//   4. Repeat for all 16 M positions.
//   5. Verify Activation BRAM address mapping.
//   6. Verify signed INT8 activation and weight data.
//   7. Verify synchronous BRAM response valid timing.
//   8. Check all K-Tile boundaries.
//
// No RTL modification.
// ============================================================================

module tb_p6b2_reader_global_k;

    // ------------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------------

    localparam int ADDR_WIDTH = 16;

    localparam int H_IN = 9;
    localparam int W_IN = 9;
    localparam int C_IN = 64;

    localparam int M_COUNT = 16;
    localparam int N_COUNT = 8;

    localparam int K_TOTAL = 576;
    localparam int K_PAIRS = K_TOTAL / 2;

    localparam int K_TILE = 64;
    localparam int K_TILE_COUNT = K_TOTAL / K_TILE;

    localparam int ACT_MAX_BYTES = 6400;

    // ------------------------------------------------------------------------
    // Clock and reset
    // ------------------------------------------------------------------------

    logic clk;
    logic rst_n;

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------------------
    // Activation Reader interface
    // ------------------------------------------------------------------------

    logic [ADDR_WIDTH-1:0] m_addr;
    logic pair_req_en;
    logic [ADDR_WIDTH-1:0] fetch_k_base;

    logic [15:0] act_pair_packed;
    logic [1:0] act_rsp_valid;

    logic [15:0] fmap_rd_data;
    logic fmap_rd_valid;
    logic fmap_rd_en;
    logic [ADDR_WIDTH-1:0] fmap_rd_addr;

    // Activation BRAM host write interface
    logic act_wr_en;
    logic [ADDR_WIDTH-1:0] act_wr_addr;
    logic [7:0] act_wr_data;

    // ------------------------------------------------------------------------
    // Weight Reader interface
    // ------------------------------------------------------------------------

    logic [ADDR_WIDTH-1:0] n_base;
    logic [ADDR_WIDTH-1:0] k_len;

    logic weight_wr_en;
    logic [ADDR_WIDTH-1:0] weight_wr_addr;
    logic [63:0] weight_wr_data;

    logic [127:0] wgt_pair_packed;
    logic [1:0] wgt_rsp_valid;

    // ------------------------------------------------------------------------
    // Counters
    // ------------------------------------------------------------------------

    integer activation_write_count;
    integer weight_write_count;
    integer request_count;
    integer activation_response_count;
    integer weight_response_count;

    integer finished;

    // ------------------------------------------------------------------------
    // Real P4 3x3/S2 Activation Reader
    // ------------------------------------------------------------------------

    conv_bram_reader_p4_3x3s2 #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .EXTERNAL_ACT_MEM(1'b1)
    ) dut_act_reader (
        .clk(clk),
        .rst_n(rst_n),

        .m_addr(m_addr),
        .pair_req_en(pair_req_en),
        .fetch_k_base(fetch_k_base),

        .wr_en(1'b0),
        .wr_addr('0),
        .wr_data('0),

        .fmap_rd_data(fmap_rd_data),
        .fmap_rd_en(fmap_rd_en),
        .fmap_rd_addr(fmap_rd_addr),

        .act_pair_packed(act_pair_packed),
        .act_rsp_valid(act_rsp_valid)
    );

    // ------------------------------------------------------------------------
    // Real shared Activation BRAM
    // ------------------------------------------------------------------------

    conv_activation_bram_shared_p5 #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .MAX_BYTES(ACT_MAX_BYTES)
    ) dut_act_bram (
        .clk(clk),
        .rst_n(rst_n),

        .wr_en(act_wr_en),
        .wr_addr(act_wr_addr),
        .wr_data(act_wr_data),

        .rd_en(fmap_rd_en),
        .rd_addr(fmap_rd_addr),

        .rd_data(fmap_rd_data),
        .rd_valid(fmap_rd_valid)
    );

    // ------------------------------------------------------------------------
    // Real P5 shared Weight BRAM Reader
    // ------------------------------------------------------------------------

    conv_weight_bram_reader_p5_shared #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .K_MAX(K_TOTAL)
    ) dut_weight_reader (
        .clk(clk),
        .rst_n(rst_n),

        .n_base(n_base),
        .k_len(k_len),

        .pair_req_en(pair_req_en),
        .fetch_k_base(fetch_k_base),

        .wr_en(weight_wr_en),
        .wr_addr(weight_wr_addr),
        .wr_data(weight_wr_data),

        .wgt_pair_packed(wgt_pair_packed),
        .wgt_rsp_valid(wgt_rsp_valid)
    );

    // ------------------------------------------------------------------------
    // Golden activation pattern
    // ------------------------------------------------------------------------

    function automatic integer signed activation_pattern(
        input integer y,
        input integer x,
        input integer c
    );

        integer v;

        begin

            v = ((17*y + 11*x + 7*c + 37) % 255) - 127;

            if ((y == 0) && (x == 0) && (c == 0))
                v = -128;

            if ((y == 8) && (x == 8) && (c == 63))
                v = 127;

            return v;

        end

    endfunction

    // ------------------------------------------------------------------------
    // Golden weight pattern
    // ------------------------------------------------------------------------

    function automatic integer signed weight_pattern(
        input integer global_k,
        input integer n
    );

        integer kernel_pixel;
        integer channel;
        integer ky;
        integer kx;
        integer v;

        begin

            kernel_pixel = global_k / C_IN;
            channel      = global_k % C_IN;

            ky = kernel_pixel / 3;
            kx = kernel_pixel % 3;

            v = ((13*ky + 7*kx + 3*channel + 5*n + 41) % 31) - 15;

            if ((global_k == 0) && (n == 0))
                v = -128;

            if ((global_k == 575) && (n == 7))
                v = 127;

            return v;

        end

    endfunction

    // ------------------------------------------------------------------------
    // Preload Activation BRAM
    //
    // HWC layout:
    //   byte_addr = (y * W_IN + x) * C_IN + channel
    //
    // 9 * 9 * 64 = 5184 INT8 bytes.
    // ------------------------------------------------------------------------

    task automatic preload_activation;

        integer addr;

        begin

            $display("");
            $display("========================================");
            $display("P6-B2: Preloading Activation BRAM");
            $display("========================================");

            activation_write_count = 0;

            for (int y = 0; y < H_IN; y++) begin

                for (int x = 0; x < W_IN; x++) begin

                    for (int c = 0; c < C_IN; c++) begin

                        addr = (y * W_IN + x) * C_IN + c;

                        @(negedge clk);

                        act_wr_en   = 1'b1;
                        act_wr_addr = ADDR_WIDTH'(addr);
                        act_wr_data = 8'(activation_pattern(y, x, c));

                        activation_write_count++;

                    end

                end

            end

            @(negedge clk);

            act_wr_en = 1'b0;

            if (activation_write_count != H_IN*W_IN*C_IN)
                $fatal(1, "FAIL: Activation preload count mismatch");

            $display(
                "PASS: Activation BRAM preload, %0d bytes",
                activation_write_count
            );

        end

    endtask

    // ------------------------------------------------------------------------
    // Preload Weight BRAM
    //
    // One address stores 8 output-channel INT8 weights.
    //
    // wr_addr = global K
    // wr_data = 8 * INT8 weights
    //
    // Total:
    //   576 K addresses
    //   8 weights per address
    // ------------------------------------------------------------------------

    task automatic preload_weight;

        begin

            $display("");
            $display("========================================");
            $display("P6-B2: Preloading Weight BRAM");
            $display("========================================");

            weight_write_count = 0;

            for (int k = 0; k < K_TOTAL; k++) begin

                @(negedge clk);

                weight_wr_en   = 1'b1;
                weight_wr_addr = ADDR_WIDTH'(k);

                for (int n = 0; n < N_COUNT; n++) begin

                    weight_wr_data[n*8 +: 8] =
                        8'(weight_pattern(k, n));

                end

                weight_write_count++;

            end

            @(negedge clk);

            weight_wr_en = 1'b0;

            if (weight_write_count != K_TOTAL)
                $fatal(1, "FAIL: Weight preload count mismatch");

            $display(
                "PASS: Weight BRAM preload, %0d K addresses",
                weight_write_count
            );

        end

    endtask

    // ------------------------------------------------------------------------
    // Check one real Activation/Weight pair request
    //
    // Input:
    //   m_index = output spatial position
    //   k_base  = global even K address
    //
    // Output:
    //   Verify Activation BRAM address and returned INT8 pair.
    //   Verify Weight BRAM returned INT8 pair.
    // ------------------------------------------------------------------------

    task automatic check_pair(
        input integer m_index,
        input integer k_base
    );

        integer global_k;
        integer kernel_pixel;

        integer ky;
        integer kx;

        integer channel;

        integer input_row;
        integer input_col;

        integer pixel_index;
        integer expected_act_addr;

        integer expected_act;
        integer expected_wgt;

        integer got_act;
        integer got_wgt;

        begin

            // ---------------------------------------------------------------
            // Calculate expected Activation BRAM address
            // ---------------------------------------------------------------

            global_k = k_base;

            kernel_pixel = global_k / C_IN;

            ky = kernel_pixel / 3;
            kx = kernel_pixel % 3;

            input_row = ((m_index >> 2) & 3) * 2 + ky;
            input_col = (m_index & 3) * 2 + kx;

            pixel_index = input_row * W_IN + input_col;

            // Each pixel stores 64 channels.
            // One pair read accesses two adjacent INT8 channels.
            expected_act_addr =
                pixel_index * (C_IN / 2) +
                ((k_base % C_IN) / 2);

            // ---------------------------------------------------------------
            // Drive one pair request
            // ---------------------------------------------------------------

            @(negedge clk);

            m_addr       = ADDR_WIDTH'(m_index);
            fetch_k_base = ADDR_WIDTH'(k_base);
            pair_req_en  = 1'b1;

            // Check combinational request and address before BRAM clock edge.
            #1;

            if (fmap_rd_en !== 1'b1)
                $fatal(
                    1,
                    "FAIL: Activation read enable M=%0d K=%0d",
                    m_index, k_base
                );

            if (fmap_rd_addr !== ADDR_WIDTH'(expected_act_addr))
                $fatal(
                    1,
                    "FAIL: Activation address M=%0d K=%0d got=%0d expected=%0d",
                    m_index,
                    k_base,
                    fmap_rd_addr,
                    expected_act_addr
                );

            // ---------------------------------------------------------------
            // Synchronous BRAM response
            // ---------------------------------------------------------------

            @(posedge clk);
            #1;

            if (act_rsp_valid !== 2'b11)
                $fatal(
                    1,
                    "FAIL: Activation response valid M=%0d K=%0d valid=%b",
                    m_index, k_base, act_rsp_valid
                );

            if (wgt_rsp_valid !== 2'b11)
                $fatal(
                    1,
                    "FAIL: Weight response valid M=%0d K=%0d valid=%b",
                    m_index, k_base, wgt_rsp_valid
                );

            if (fmap_rd_valid !== 1'b1)
                $fatal(
                    1,
                    "FAIL: Shared Activation BRAM response valid M=%0d K=%0d",
                    m_index, k_base
                );

            // ---------------------------------------------------------------
            // Verify two activation INT8 values
            // ---------------------------------------------------------------

            for (int p = 0; p < 2; p++) begin

                global_k = k_base + p;

                kernel_pixel = global_k / C_IN;

                ky = kernel_pixel / 3;
                kx = kernel_pixel % 3;

                channel = global_k % C_IN;

                input_row = ((m_index >> 2) & 3) * 2 + ky;
                input_col = (m_index & 3) * 2 + kx;

                expected_act =
                    activation_pattern(input_row, input_col, channel);

                got_act = $signed(act_pair_packed[p*8 +: 8]);

                if (got_act !== expected_act)
                    $fatal(
                        1,
                        "FAIL: Activation data M=%0d K=%0d p=%0d got=%0d expected=%0d",
                        m_index,
                        k_base,
                        p,
                        got_act,
                        expected_act
                    );

            end

            // ---------------------------------------------------------------
            // Verify two K words, each containing 8 weight INT8 values
            // ---------------------------------------------------------------

            for (int p = 0; p < 2; p++) begin

                global_k = k_base + p;

                for (int n = 0; n < N_COUNT; n++) begin

                    expected_wgt = weight_pattern(global_k, n);

                    got_wgt =
                        $signed(wgt_pair_packed[p*64 + n*8 +: 8]);

                    if (got_wgt !== expected_wgt)
                        $fatal(
                            1,
                            "FAIL: Weight data M=%0d K=%0d N=%0d got=%0d expected=%0d",
                            m_index,
                            global_k,
                            n,
                            got_wgt,
                            expected_wgt
                        );

                end

            end

            request_count++;
            activation_response_count++;
            weight_response_count++;

            // ---------------------------------------------------------------
            // Release request and verify valid returns low
            // ---------------------------------------------------------------

            @(negedge clk);

            pair_req_en = 1'b0;

            @(posedge clk);
            #1;

            if (act_rsp_valid !== 2'b00)
                $fatal(1, "FAIL: Activation valid did not clear");

            if (wgt_rsp_valid !== 2'b00)
                $fatal(1, "FAIL: Weight valid did not clear");

            if (fmap_rd_valid !== 1'b0)
                $fatal(1, "FAIL: Shared BRAM valid did not clear");

        end

    endtask

    // ------------------------------------------------------------------------
    // Main test
    // ------------------------------------------------------------------------

    initial begin : main_test

        integer expected_requests;

        rst_n = 1'b0;

        m_addr = '0;
        pair_req_en = 1'b0;
        fetch_k_base = '0;

        n_base = '0;
        k_len = ADDR_WIDTH'(K_TOTAL);

        act_wr_en = 1'b0;
        act_wr_addr = '0;
        act_wr_data = '0;

        weight_wr_en = 1'b0;
        weight_wr_addr = '0;
        weight_wr_data = '0;

        activation_write_count = 0;
        weight_write_count = 0;
        request_count = 0;
        activation_response_count = 0;
        weight_response_count = 0;

        finished = 0;

        if ((K_TOTAL % K_TILE) != 0)
            $fatal(1, "FAIL: K_TOTAL must be divisible by K_TILE");

        if (K_TILE_COUNT != 9)
            $fatal(1, "FAIL: Expected 9 K-Tiles");

        $display("");
        $display("================================================");
        $display("P6-B2-2 REAL BRAM READER GLOBAL K TEST");
        $display("================================================");

        $display(
            "Configuration: M=%0d N=%0d K=%0d K_TILE=%0d",
            M_COUNT, N_COUNT, K_TOTAL, K_TILE
        );

        repeat (4) @(negedge clk);

        rst_n = 1'b1;

        // ---------------------------------------------------------------
        // Preload actual BRAMs
        // ---------------------------------------------------------------

        preload_activation();

        preload_weight();

        // ---------------------------------------------------------------
        // Verify all M positions, all K-Tiles, and all K-pair requests.
        // K addresses passed to the readers are GLOBAL addresses.
        // ---------------------------------------------------------------

        expected_requests = M_COUNT * K_PAIRS;

        for (int m = 0; m < M_COUNT; m++) begin

            $display("");
            $display("------------------------------------------------");
            $display("P6-B2: M position %0d / %0d", m, M_COUNT-1);
            $display("------------------------------------------------");

            for (int tile = 0; tile < K_TILE_COUNT; tile++) begin

                $display(
                    "  M=%0d Tile=%0d global K=%0d..%0d",
                    m,
                    tile,
                    tile * K_TILE,
                    (tile + 1) * K_TILE - 1
                );

                for (int pair_idx = 0;
                     pair_idx < (K_TILE / 2);
                     pair_idx++) begin

                    check_pair(
                        m,
                        tile * K_TILE + pair_idx * 2
                    );

                end

                $display(
                    "  PASS: M=%0d Tile=%0d, 32 K-pair requests",
                    m,
                    tile
                );

            end

        end

        // ---------------------------------------------------------------
        // Final counters and summary
        // ---------------------------------------------------------------

        if (request_count != expected_requests)
            $fatal(
                1,
                "FAIL: request count got=%0d expected=%0d",
                request_count,
                expected_requests
            );

        if (activation_response_count != expected_requests)
            $fatal(
                1,
                "FAIL: activation response count got=%0d expected=%0d",
                activation_response_count,
                expected_requests
            );

        if (weight_response_count != expected_requests)
            $fatal(
                1,
                "FAIL: weight response count got=%0d expected=%0d",
                weight_response_count,
                expected_requests
            );

        if (activation_write_count != H_IN * W_IN * C_IN)
            $fatal(1, "FAIL: final Activation preload count mismatch");

        if (weight_write_count != K_TOTAL)
            $fatal(1, "FAIL: final Weight preload count mismatch");

        finished = 1;

        $display("");
        $display("================================================");
        $display("PASS: P6-B2-2 REAL BRAM READER GLOBAL K TEST");
        $display("  M positions                  = %0d", M_COUNT);
        $display("  K elements per M             = %0d", K_TOTAL);
        $display("  K-Tiles per M                = %0d", K_TILE_COUNT);
        $display("  K-pair requests per M        = %0d", K_PAIRS);
        $display("  Total K-pair requests        = %0d", request_count);
        $display("  Activation responses         = %0d",
                 activation_response_count);
        $display("  Weight responses             = %0d",
                 weight_response_count);
        $display("  Activation bytes preloaded   = %0d",
                 activation_write_count);
        $display("  Weight K addresses preloaded = %0d",
                 weight_write_count);
        $display("  All M positions and K-Tile boundaries checked");
        $display("================================================");
        $display("");

        $finish;

    end

    // ------------------------------------------------------------------------
    // Watchdog
    // ------------------------------------------------------------------------

    initial begin : watchdog
        repeat (100000) @(posedge clk);

        if (finished != 1)
            $fatal(
                1,
                "FAIL: TIMEOUT tb_p6b2_reader_global_k"
            );
    end

endmodule
