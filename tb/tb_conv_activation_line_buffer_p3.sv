
`timescale 1ns / 1ps

module tb_conv_activation_line_buffer_p3;

    localparam int ROWS = 4;
    localparam int W_IN = 10;
    localparam int C_IN = 64;

    localparam int DEPTH =
        ROWS * W_IN * (C_IN / 2);

    localparam int ADDR_WIDTH = 11;

    logic clk;
    logic rst_n;

    logic [1:0] wr_en;
    logic [ADDR_WIDTH-1:0] wr_addr;
    logic [15:0] wr_data;

    logic rd_en;
    logic [ADDR_WIDTH-1:0] rd_addr;

    logic [15:0] rd_data;
    logic rd_valid;

    logic [7:0] expected_even [0:DEPTH-1];
    logic [7:0] expected_odd  [0:DEPTH-1];

    integer i;
    integer checked;

    // ========================================================
    // 1. Clock
    // ========================================================

    initial clk = 0;
    always #5 clk = ~clk;

    // ========================================================
    // 2. DUT
    // ========================================================

    conv_activation_line_buffer_p3 #(
        .ROWS(ROWS),
        .W_IN(W_IN),
        .C_IN(C_IN),
        .ADDR_WIDTH(ADDR_WIDTH)
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

    // ========================================================
    // 3. Expected data patterns
    // ========================================================

    function automatic logic [7:0] even_pattern(
        input integer index
    );

        even_pattern = 8'(index * 17 + 8'h31);

    endfunction

    function automatic logic [7:0] odd_pattern(
        input integer index
    );

        odd_pattern = 8'(index * 29 + 8'h57);

    endfunction

    // ========================================================
    // 4. Write task
    // ========================================================

    task automatic write_word(
        input integer address,
        input logic [1:0] bank_enable,
        input logic [15:0] data
    );

        begin

            @(negedge clk);

            wr_en = bank_enable;
            wr_addr = ADDR_WIDTH'(address);
            wr_data = data;

            @(posedge clk);
            #1;

        end

    endtask

    // ========================================================
    // 5. Read and verify task
    //
    // One-cycle synchronous read.
    // ========================================================

    task automatic check_read(
        input integer address
    );

        logic [15:0] expected_word;

        begin

            expected_word = {
                expected_odd[address],
                expected_even[address]
            };

            // Issue read request
            @(negedge clk);

            rd_en = 1'b1;
            rd_addr = ADDR_WIDTH'(address);

            // Response after one clock edge
            @(posedge clk);
            #1;

            if (rd_valid !== 1'b1)
                $fatal(1,
                    "FAIL: rd_valid not asserted, addr=%0d",
                    address
                );

            if (rd_data !== expected_word)
                $fatal(1,
                    "FAIL: addr=%0d got=%h expected=%h",
                    address,
                    rd_data,
                    expected_word
                );

            // Stop issuing reads
            @(negedge clk);

            rd_en = 1'b0;

            @(posedge clk);
            #1;

            if (rd_valid !== 1'b0)
                $fatal(1,
                    "FAIL: rd_valid did not clear"
                );

            // Output data should hold when rd_en=0
            if (rd_data !== expected_word)
                $fatal(1,
                    "FAIL: output data did not hold"
                );

            checked++;

        end

    endtask

    // ========================================================
    // 6. Main test
    // ========================================================

    initial begin

        rst_n = 0;

        wr_en = 0;
        wr_addr = 0;
        wr_data = 0;

        rd_en = 0;
        rd_addr = 0;

        checked = 0;

        repeat (3) @(posedge clk);

        @(negedge clk);
        rst_n = 1;

        // ----------------------------------------------------
        // TEST 1: Fill all 1280 addresses
        // ----------------------------------------------------

        $display("------------------------------------");
        $display("TEST 1: Fill Line Buffer");
        $display("------------------------------------");

        for (i = 0; i < DEPTH; i++) begin

            expected_even[i] = even_pattern(i);
            expected_odd[i]  = odd_pattern(i);

            write_word(
                i,
                2'b11,
                {
                    expected_odd[i],
                    expected_even[i]
                }
            );

        end

        @(negedge clk);
        wr_en = 0;

        $display("PASS: 1280 addresses written");

        // ----------------------------------------------------
        // TEST 2: Independent Bank 0 write
        // ----------------------------------------------------

        $display("TEST 2: Independent Bank 0 write");

        expected_even[17] = 8'hA5;

        write_word(
            17,
            2'b01,
            16'h00A5
        );

        // ----------------------------------------------------
        // TEST 3: Independent Bank 1 write
        // ----------------------------------------------------

        $display("TEST 3: Independent Bank 1 write");

        expected_odd[19] = 8'h5A;

        write_word(
            19,
            2'b10,
            16'h5A00
        );

        // ----------------------------------------------------
        // TEST 4: Simultaneous dual-bank write
        // ----------------------------------------------------

        $display("TEST 4: Simultaneous dual-bank write");

        expected_even[21] = 8'h3C;
        expected_odd[21]  = 8'hC3;

        write_word(
            21,
            2'b11,
            16'hC33C
        );

        @(negedge clk);
        wr_en = 0;

        $display("PASS: Independent and dual-bank writes");

        // ----------------------------------------------------
        // TEST 5: Read all addresses
        // ----------------------------------------------------

        $display("------------------------------------");
        $display("TEST 5: Read all 1280 addresses");
        $display("------------------------------------");

        for (i = 0; i < DEPTH; i++) begin

            check_read(i);

        end

        // ----------------------------------------------------
        // Final result
        // ----------------------------------------------------

        if (checked != DEPTH)
            $fatal(1, "FAIL: incorrect checked count");

        $display("------------------------------------");
        $display("PASS: P3 Line Buffer verification");
        $display("PASS: Bank 0 and Bank 1");
        $display("PASS: Independent bank writes");
        $display("PASS: Simultaneous bank writes");
        $display("PASS: One-cycle synchronous read");
        $display("PASS: Output data hold");
        $display("TOTAL CHECKED = %0d", checked);
        $display("------------------------------------");

        $finish;

    end

endmodule
