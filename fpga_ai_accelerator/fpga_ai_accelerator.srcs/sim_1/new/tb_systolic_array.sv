
`timescale 1ns / 1ps

// Minimal self-checking testbench for
// taitashaw/fpga-ai-accelerator/rtl/systolic_array.sv
// Expected matrix: [2 4] * [[3 5], [6 7]] = [30 38]
module tb_systolic_array;
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

    // 10 ns clock period = 100 MHz
    initial clk = 1'b0;
    always #5 clk = ~clk;

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

    integer i;
    integer cycle_count = 0;

    // Watch the 3-stage MAC pipeline and both output columns.
    always @(posedge clk) begin
        if (rst_n && enable) begin
            cycle_count = cycle_count + 1;
            #1;  // wait until nonblocking assignments have updated
            $display("edge=%0d | PE00 act=%0d mul=%0d acc=%0d | C0=%0d C1=%0d",
                     cycle_count,
                     $signed(dut.gen_row[0].gen_col[0].u_pe.act_reg),
                     $signed(dut.gen_row[0].gen_col[0].u_pe.mul_reg),
                     $signed(dut.gen_row[0].gen_col[0].u_pe.acc_reg),
                     $signed(acc_out_packed[0*ACC_WIDTH +: ACC_WIDTH]),
                     $signed(acc_out_packed[1*ACC_WIDTH +: ACC_WIDTH]));
        end
    end

    initial begin
        $dumpfile("tb_systolic_array.vcd");
        $dumpvars(0, tb_systolic_array);

        // Reset and initialization.
        rst_n         = 1'b0;
        enable        = 1'b0;
        clear_acc     = 1'b0;
        valid_reset   = 1'b0;
        data_valid_in = 1'b0;
        act_in_packed = '0;
        for (i = 0; i < ARRAY_COLS; i = i + 1)
            wgt_in[i] = '0;

        repeat (3) @(negedge clk);
        rst_n         = 1'b1;
        enable        = 1'b1;
        data_valid_in = 1'b1;

        // Edge 1: PE[0][0] captures 2*3;
        // column 1 skew register captures weight 5.
        act_in_packed[0*ACT_WIDTH +: ACT_WIDTH] = 8'sd2;
        wgt_in[0] = 8'sd3;
        wgt_in[1] = 8'sd5;

        @(negedge clk);
        // Edge 2: PE[1][0] captures 4*6;
        // PE[0][1] captures 2*5 (through activation + weight delays).
        act_in_packed = '0;
        act_in_packed[1*ACT_WIDTH +: ACT_WIDTH] = 8'sd4;
        wgt_in[0] = 8'sd6;
        wgt_in[1] = 8'sd7;
        data_valid_in = 1'b0;

        @(negedge clk);
        // Flush the pipeline with zero inputs; PE[1][1] captures 4*7 at edge 3.
        act_in_packed = '0;
        for (i = 0; i < ARRAY_COLS; i = i + 1)
            wgt_in[i] = '0;

        // From just before edge 3, wait for edges 3, 4, 5.
        repeat (3) @(posedge clk);
        #1;  // allow registered outputs and combinational column sums to settle

        if ($signed(acc_out_packed[0*ACC_WIDTH +: ACC_WIDTH]) !== 32'sd30 ||
            $signed(acc_out_packed[1*ACC_WIDTH +: ACC_WIDTH]) !== 32'sd38)
            $fatal(1, "FAIL: C0=%0d C1=%0d (expected 30, 38)",
                   $signed(acc_out_packed[0*ACC_WIDTH +: ACC_WIDTH]),
                   $signed(acc_out_packed[1*ACC_WIDTH +: ACC_WIDTH]));

        for (i = 2; i < ARRAY_COLS; i = i + 1) begin
            if (acc_out_packed[i*ACC_WIDTH +: ACC_WIDTH] !== 32'd0)
                $fatal(1, "FAIL: unexpected nonzero result in column %0d", i);
        end

        if (any_overflow !== 1'b0)
            $fatal(1, "FAIL: unexpected PE overflow");

        $display("PASS: matrix multiplication result = [30, 38]");
        $finish;
    end
endmodule
