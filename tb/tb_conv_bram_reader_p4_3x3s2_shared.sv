
`timescale 1ns / 1ps

module tb_conv_bram_reader_p4_3x3s2_shared;

    localparam int H = 9;
    localparam int W = 9;
    localparam int C = 64;

    localparam int PATCH_BYTES = H*W*C;
    localparam int MAX_BYTES = 6400;

    localparam int M = 16;
    localparam int K = 576;
    localparam int K_PAIRS = K/2;

    localparam int EPOCHS = 2;
    localparam int PAIRS_PER_EPOCH = M*K_PAIRS;

    logic clk = 0;
    always #5 clk = ~clk;

    logic rst_n;

    // Host write interface
    logic wr_en;
    logic [15:0] wr_addr;
    logic [7:0] wr_data;

    // Reader request
    logic pair_req_en;
    logic [15:0] m_addr;
    logic [15:0] fetch_k_base;

    // Shared BRAM interface
    logic [15:0] fmap_rd_data;
    logic fmap_rd_en;
    logic [15:0] fmap_rd_addr;
    logic fmap_rd_valid;

    // Reader output
    logic [15:0] act_pair_packed;
    logic [1:0] act_rsp_valid;

    logic [7:0] golden [0:PATCH_BYTES-1];

    bit finished = 0;

    // ------------------------------------------------
    // P4 3x3/S2 Reader: external memory mode
    // ------------------------------------------------

    conv_bram_reader_p4_3x3s2 #(
        .ADDR_WIDTH(16),
        .EXTERNAL_ACT_MEM(1'b1)
    ) u_reader (
        .clk(clk),
        .rst_n(rst_n),

        .m_addr(m_addr),
        .pair_req_en(pair_req_en),
        .fetch_k_base(fetch_k_base),

        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_data(wr_data),

        .fmap_rd_data(fmap_rd_data),
        .fmap_rd_en(fmap_rd_en),
        .fmap_rd_addr(fmap_rd_addr),

        .act_pair_packed(act_pair_packed),
        .act_rsp_valid(act_rsp_valid)
    );

    // ------------------------------------------------
    // Shared Activation BRAM
    // ------------------------------------------------

    conv_activation_bram_shared_p5 #(
        .ADDR_WIDTH(16),
        .MAX_BYTES(MAX_BYTES)
    ) u_shared_mem (
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

    // ------------------------------------------------
    // Golden data pattern
    // ------------------------------------------------

    function automatic [7:0] pattern(
        input integer addr,
        input integer epoch
    );

        integer v;

        begin
            v = (addr*37 + epoch*53 + 19) & 255;
            pattern = v[7:0];
        end

    endfunction

    // ------------------------------------------------
    // Load 9x9x64 activation patch
    // ------------------------------------------------

    task automatic load_epoch(input integer epoch);

        integer a;

        begin

            for (a=0; a<PATCH_BYTES; a=a+1) begin

                @(negedge clk);

                wr_en = 1'b1;
                wr_addr = 16'(a);
                wr_data = pattern(a,epoch);

                golden[a] = pattern(a,epoch);

            end

            @(negedge clk);

            wr_en = 1'b0;
            wr_addr = '0;
            wr_data = '0;

            repeat (2) @(negedge clk);

            $display(
                "INFO: epoch=%0d loaded %0d activation bytes",
                epoch,PATCH_BYTES
            );

        end

    endtask

    // ------------------------------------------------
    // Scan all M/K pairs
    // ------------------------------------------------

    task automatic scan_epoch(input integer epoch);

        integer idx;
        integer m,k;

        integer oy,ox;
        integer kernel_pixel;
        integer ky,kx,ci;

        integer input_row,input_col;
        integer byte_addr,pair_addr;

        integer i;

        logic [15:0] expected_data;
        logic [15:0] expected_addr;

        begin

            for (i=0; i<PAIRS_PER_EPOCH; i=i+1) begin

                // Permute all 4608 requests.
                // gcd(73,4608)=1, so every pair is visited once.

                idx = (73*i + 19*epoch) % PAIRS_PER_EPOCH;

                m = idx/K_PAIRS;
                k = 2*(idx%K_PAIRS);

                oy = m/4;
                ox = m%4;

                kernel_pixel = k/64;

                ky = kernel_pixel/3;
                kx = kernel_pixel%3;

                ci = k%64;

                input_row = 2*oy + ky;
                input_col = 2*ox + kx;

                // HWC byte address
                byte_addr =
                    ((input_row*W + input_col)*C) + ci;

                // Two INT8 values form one pair.
                pair_addr = byte_addr/2;

                expected_addr = 16'(pair_addr);

                @(negedge clk);

                pair_req_en = 1'b1;
                m_addr = 16'(m);
                fetch_k_base = 16'(k);

                #1;

                // Check external memory request.

                if (fmap_rd_en !== 1'b1)
                    $fatal(1,
                        "FAIL: external rd_en epoch=%0d M=%0d K=%0d",
                        epoch,m,k
                    );

                if (fmap_rd_addr !== expected_addr)
                    $fatal(1,
                        "FAIL: address epoch=%0d M=%0d K=%0d expected=%0d actual=%0d",
                        epoch,m,k,expected_addr,fmap_rd_addr
                    );

                // Synchronous BRAM response

                @(posedge clk);
                #1;

                expected_data = {
                    golden[byte_addr+1],
                    golden[byte_addr]
                };

                if (act_rsp_valid !== 2'b11)
                    $fatal(1,
                        "FAIL: Reader valid epoch=%0d M=%0d K=%0d",
                        epoch,m,k
                    );

                if (fmap_rd_valid !== 1'b1)
                    $fatal(1,
                        "FAIL: Shared BRAM valid epoch=%0d M=%0d K=%0d",
                        epoch,m,k
                    );

                if (act_pair_packed !== expected_data)
                    $fatal(1,
                        "FAIL: data epoch=%0d M=%0d K=%0d expected=%h actual=%h",
                        epoch,m,k,expected_data,act_pair_packed
                    );

            end

            // Stop requests.

            @(negedge clk);

            pair_req_en = 1'b0;
            m_addr = '0;
            fetch_k_base = '0;

            @(posedge clk);
            #1;

            if (act_rsp_valid !== 2'b00)
                $fatal(1,"FAIL: stale Reader valid");

            if (fmap_rd_valid !== 1'b0)
                $fatal(1,"FAIL: stale Shared BRAM valid");

            if (act_pair_packed !== 16'b0)
                $fatal(1,"FAIL: stale activation output");

            $display(
                "PASS: epoch=%0d, %0d/%0d activation pairs through shared BRAM",
                epoch,PAIRS_PER_EPOCH,PAIRS_PER_EPOCH
            );

        end

    endtask

    // ------------------------------------------------
    // Main test
    // ------------------------------------------------

    integer e;

    initial begin

        rst_n = 0;

        wr_en = 0;
        wr_addr = 0;
        wr_data = 0;

        pair_req_en = 0;
        m_addr = 0;
        fetch_k_base = 0;

        repeat (4) @(negedge clk);

        rst_n = 1;

        for (e=0; e<EPOCHS; e=e+1) begin

            load_epoch(e);

            scan_epoch(e);

        end

        finished = 1;

        $display("----------------------------------------");
        $display("PASS: P5-B2 P4 3x3/S2 Shared Activation BRAM");
        $display("PASS: 2 epochs");
        $display("PASS: 9216/9216 activation pairs");
        $display("PASS: 18432/18432 INT8 values");
        $display("PASS: Shared BRAM address mapping");
        $display("PASS: One-cycle synchronous read");
        $display("----------------------------------------");

        $finish;

    end

    // ------------------------------------------------
    // Watchdog
    // ------------------------------------------------

    initial begin

        repeat (30000) @(posedge clk);

        if (!finished)
            $fatal(1,"TIMEOUT: P4 3x3/S2 shared BRAM TB");

    end

endmodule
