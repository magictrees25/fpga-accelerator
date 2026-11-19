`timescale 1ns / 1ps
module tb_p7_requant_residual;
    logic clk = 0;
    always #5 clk = ~clk;
    logic rst_n, in_valid, in_residual_en, out_valid;
    logic signed [31:0] in_acc, in_bias, in_multiplier;
    logic signed [7:0] in_skip, out_data;
    logic [119:0] vectors [0:255];
    logic [7:0] expected_now, previous_expected;
    bit previous_valid;
    int checked;
    string data_dir;
    requant_residual_p7 dut (.*);

    function automatic bit exists(input string path);
        int fd;
        begin
            fd = $fopen(path, "r");
            exists = (fd != 0);
            if (fd != 0) $fclose(fd);
        end
    endfunction

    always @(posedge clk) begin : score
        bit accepted;
        logic [7:0] captured_expected;
        accepted = in_valid;
        captured_expected = expected_now;
        if (!rst_n) begin
            previous_valid = 0;
            previous_expected = 0;
        end else begin
            #2;
            if (out_valid !== previous_valid) $fatal(1, "P7 post: pipeline valid/bubble mismatch");
            if (previous_valid) begin
                if (out_data !== previous_expected)
                    $fatal(1, "P7 post: vector=%0d got=%0d want=%0d", checked, out_data, previous_expected);
                checked++;
            end
            previous_valid = accepted;
            previous_expected = captured_expected;
        end
    end

    initial begin
        rst_n=0; in_valid=0; in_acc=0; in_bias=0; in_multiplier=0;
        in_skip=0; in_residual_en=0; expected_now=0; checked=0;
        if (!$value$plusargs("P7_DATA_DIR=%s", data_dir)) begin
            data_dir="reference/p7a_layer1_0";
            if (!exists({data_dir,"/post_vectors.mem"}))
                data_dir="../../../../../reference/p7a_layer1_0";
        end
        if (!exists({data_dir,"/post_vectors.mem"}))
            $fatal(1, "P7: missing post_vectors.mem; set +P7_DATA_DIR=<absolute directory>");
        $readmemh({data_dir,"/post_vectors.mem"}, vectors);
        for (int i=0;i<256;i++)
            if ($isunknown(vectors[i])) $fatal(1, "P7: incomplete vector file i=%0d", i);
        repeat(4) @(negedge clk);
        rst_n=1;
        for (int i=0;i<256;i++) begin
            @(negedge clk);
            in_valid=1;
            in_residual_en=vectors[i][112];
            in_skip=vectors[i][111:104];
            in_multiplier=vectors[i][103:72];
            in_bias=vectors[i][71:40];
            in_acc=vectors[i][39:8];
            expected_now=vectors[i][7:0];
            if ((i%7)==3) begin
                @(negedge clk);
                in_valid=0;
            end
        end
        @(negedge clk); in_valid=0;
        repeat(4) @(negedge clk);
        if (checked != 256) $fatal(1, "P7 post: missing results %0d/256", checked);
        $display("PASS: P7-A requant/residual 256/256 vectors, signed rounding, bias, saturation and pipeline bubbles");
        $finish;
    end
    initial begin #100000; $fatal(1, "P7 post: timeout"); end
endmodule
