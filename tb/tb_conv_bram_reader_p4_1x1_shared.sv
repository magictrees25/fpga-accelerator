`timescale 1ns / 1ps

module tb_conv_bram_reader_p4_1x1_shared;

    localparam int INPUT_BYTES = 4096;
    localparam int MAX_BYTES   = 6400;
    localparam int PAIRS       = 512;
    localparam int EPOCHS      = 2;

    logic clk = 0;
    always #5 clk = ~clk;

    logic rst_n;

    logic wr_en;
    logic [15:0] wr_addr;
    logic [7:0] wr_data;

    logic pair_req_en;
    logic [15:0] m_addr;
    logic [15:0] fetch_k_base;

    logic [15:0] fmap_rd_data;
    logic fmap_rd_en;
    logic [15:0] fmap_rd_addr;
    logic fmap_rd_valid;

    logic [15:0] act_pair_packed;
    logic [1:0] act_rsp_valid;

    logic [7:0] golden [0:INPUT_BYTES-1];

    bit finished = 0;

    // ------------------------------------------------
    // P4 1x1 Reader: external memory mode
    // ------------------------------------------------

    conv_bram_reader_p4_1x1 #(
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
    // Golden pattern
    // ------------------------------------------------

    function automatic [7:0] pattern(
        input integer addr,
        input integer epoch
    );

        integer v;

        begin
            v = (addr * 37 + epoch * 53 + 19) & 255;
            pattern = v[7:0];
        end

    endfunction

    // ------------------------------------------------
    // Preload activation feature map
    // ------------------------------------------------

    task automatic write_epoch(input integer epoch);

        integer i;

        begin

            for (i = 0; i < INPUT_BYTES; i = i + 1) begin

                @(negedge clk);

                wr_en   = 1'b1;
                wr_addr = i;
                wr_data = pattern(i, epoch);

                golden[i] = pattern(i, epoch);

                @(posedge clk);
                #1;

            end

            @(negedge clk);

            wr_en   = 1'b0;
            wr_addr = 0;
            wr_data = 0;

        end

    endtask

    // ------------------------------------------------
    // Read all M/K pairs through shared BRAM
    // ------------------------------------------------

    task automatic read_epoch(input integer epoch);

        integer idx;
        integer m;
        integer k;
        integer oy;
        integer ox;
        integer byte_addr;
        integer pair_addr;

        logic [15:0] expected_data;
        logic [15:0] expected_addr;

        begin

            for (idx = 0; idx < PAIRS; idx = idx + 1) begin

                m = idx / 32;
                k = (idx % 32) * 2;

                oy = m / 4;
                ox = m % 4;

                // Original HWC input address.
                byte_addr =
                    ((2 * oy * 8 + 2 * ox) * 64) + k;

                pair_addr = byte_addr / 2;

                @(negedge clk);

                pair_req_en = 1'b1;
                m_addr = m;
                fetch_k_base = k;

                #1;

                expected_addr = pair_addr;

                // Check address generation.
                if (fmap_rd_en !== 1'b1)
                    $fatal(1, "FAIL: external read enable");

                if (fmap_rd_addr !== expected_addr)
                    $fatal(
                        1,
                        "FAIL: address epoch=%0d M=%0d K=%0d expected=%0d actual=%0d",
                        epoch, m, k, expected_addr, fmap_rd_addr
                    );

                @(posedge clk);
                #1;

                expected_data = {
                    golden[byte_addr + 1],
                    golden[byte_addr]
                };

                // Check both valid signals.
                if (act_rsp_valid !== 2'b11)
                    $fatal(1, "FAIL: Reader response valid");

                if (fmap_rd_valid !== 1'b1)
                    $fatal(1, "FAIL: Shared BRAM response valid");

                // Check returned INT8 pair.
                if (act_pair_packed !== expected_data)
                    $fatal(
                        1,
                        "FAIL: epoch=%0d M=%0d K=%0d expected=%h actual=%h",
                        epoch, m, k,
                        expected_data, act_pair_packed
                    );

            end

            @(negedge clk);

            pair_req_en = 1'b0;
            m_addr = 0;
            fetch_k_base = 0;

            @(posedge clk);
            #1;

            if (act_rsp_valid !== 2'b00 ||
                fmap_rd_valid !== 1'b0 ||
                act_pair_packed !== 16'b0)
                $fatal(1, "FAIL: stale response after request");

            $display(
                "PASS: epoch=%0d, 512/512 pairs through shared BRAM",
                epoch
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

        for (e = 0; e < EPOCHS; e = e + 1) begin

            write_epoch(e);

            read_epoch(e);

        end

        finished = 1;

        $display("----------------------------------------");
        $display("PASS: P5-B2 P4 1x1 Shared Activation BRAM");
        $display("PASS: 2 epochs");
        $display("PASS: 1024/1024 activation pairs");
        $display("PASS: 2048/2048 INT8 values");
        $display("PASS: Shared BRAM address mapping");
        $display("PASS: One-cycle read response");
        $display("----------------------------------------");

        $finish;

    end

    // ------------------------------------------------
    // Watchdog
    // ------------------------------------------------

    initial begin

        repeat (15000) @(posedge clk);

        if (!finished)
            $fatal(1, "TIMEOUT: P4 1x1 shared BRAM TB");

    end

endmodule