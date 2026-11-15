`timescale 1ns / 1ps

module tb_conv_activation_bram_shared_p5;

    localparam int MAX_BYTES  = 6400;
    localparam int BANK_DEPTH = MAX_BYTES / 2;
    localparam int ADDR_WIDTH = 16;
    localparam int EPOCHS     = 2;

    logic clk = 0;
    always #5 clk = ~clk;

    logic rst_n;

    logic wr_en;
    logic [15:0] wr_addr;
    logic [7:0] wr_data;

    logic rd_en;
    logic [15:0] rd_addr;
    logic [15:0] rd_data;
    logic rd_valid;

    logic [7:0] golden [0:MAX_BYTES-1];

    bit finished = 0;

    // ------------------------------------------------
    // DUT
    // ------------------------------------------------

    conv_activation_bram_shared_p5 #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .MAX_BYTES(MAX_BYTES)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),

        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_data(wr_data),

        .rd_en(rd_en),
        .rd_addr(rd_addr),

        .rd_data(rd_data),
        .rd_valid(rd_valid)
    );

    // ------------------------------------------------
    // Test data pattern
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
    // Write complete activation memory
    // ------------------------------------------------

    task automatic write_epoch(input integer epoch);

        integer i;

        begin

            for (i = 0; i < MAX_BYTES; i = i + 1) begin

                @(negedge clk);

                wr_en   = 1'b1;
                wr_addr = 16'(i);
                wr_data = pattern(i, epoch);

                golden[i] = pattern(i, epoch);

                @(posedge clk);
                #1;

            end

            @(negedge clk);

            wr_en   = 1'b0;
            wr_addr = '0;
            wr_data = '0;

        end

    endtask

    // ------------------------------------------------
    // Read and verify all activation pairs
    // ------------------------------------------------

    task automatic read_epoch(input integer epoch);

        integer i;
        logic [15:0] expected;

        begin

            for (i = 0; i < BANK_DEPTH; i = i + 1) begin

                @(negedge clk);

                rd_en   = 1'b1;
                rd_addr = 16'(i);

                @(posedge clk);
                #1;

                expected = {
                    golden[2*i+1],
                    golden[2*i]
                };

                if (rd_valid !== 1'b1)
                    $fatal(
                        1,
                        "FAIL: rd_valid epoch=%0d pair=%0d",
                        epoch, i
                    );

                if (rd_data !== expected)
                    $fatal(
                        1,
                        "FAIL: epoch=%0d pair=%0d expected=%h actual=%h",
                        epoch, i, expected, rd_data
                    );

            end

            @(negedge clk);

            rd_en   = 1'b0;
            rd_addr = '0;

            @(posedge clk);
            #1;

            if (rd_valid !== 1'b0)
                $fatal(1, "FAIL: rd_valid did not clear");

        end

    endtask

    // ------------------------------------------------
    // Main test
    // ------------------------------------------------

    integer e;

    initial begin

        rst_n   = 0;

        wr_en   = 0;
        wr_addr = 0;
        wr_data = 0;

        rd_en   = 0;
        rd_addr = 0;

        repeat (4) @(negedge clk);

        rst_n = 1;

        for (e = 0; e < EPOCHS; e = e + 1) begin

            // Write all 6400 bytes
            write_epoch(e);

            // Read and verify all 3200 pairs
            read_epoch(e);

            $display(
                "PASS: epoch=%0d, 6400 bytes written, 3200 pairs checked",
                e
            );

        end

        finished = 1;

        $display("------------------------------------");
        $display("PASS: P5-B2 Shared Activation BRAM");
        $display("PASS: Two independent 8-bit banks");
        $display("PASS: 6400/6400 bytes written per epoch");
        $display("PASS: 3200/3200 pairs read per epoch");
        $display("PASS: One-cycle synchronous read");
        $display("------------------------------------");

        $finish;

    end

    // ------------------------------------------------
    // Watchdog
    // ------------------------------------------------

    initial begin

        repeat (25000) @(posedge clk);

        if (!finished)
            $fatal(1, "TIMEOUT: Shared Activation BRAM TB");

    end

endmodule