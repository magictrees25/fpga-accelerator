`timescale 1ns / 1ps
module tb_p7b_requant_branch;
    logic clk=0;
    always #5 clk=~clk;
    logic rst_n,in_valid,in_residual_en,in_relu_en,out_valid,out_overflow;
    logic signed [31:0] in_acc,in_bias,in_multiplier,in_skip,out_data;
    logic [167:0] vectors[0:255];
    logic [31:0] expected_now,previous_expected;
    bit previous_valid;
    int checked;
    string data_dir;
    requant_branch_p7b dut(.*);
    function automatic bit exists(input string path);
        int fd;
        begin fd=$fopen(path,"r");exists=(fd!=0);if(fd!=0)$fclose(fd);end
    endfunction
    always @(posedge clk) begin : score
        bit accepted;
        logic [31:0] captured;
        accepted=in_valid;captured=expected_now;
        if(!rst_n)begin previous_valid=0;previous_expected=0;end
        else begin
            #2;
            if(out_valid!==previous_valid) $fatal(1,"P7-B post valid/bubble mismatch");
            if(previous_valid)begin
                if(out_data!==previous_expected || out_overflow!==0)
                    $fatal(1,"P7-B post vector=%0d got=%0d want=%0d overflow=%b",checked,out_data,$signed(previous_expected),out_overflow);
                checked++;
            end
            previous_valid=accepted;previous_expected=captured;
        end
    end
    initial begin
        rst_n=0;in_valid=0;in_acc=0;in_bias=0;in_multiplier=0;in_skip=0;
        in_residual_en=0;in_relu_en=0;expected_now=0;checked=0;
        if(!$value$plusargs("P7B_DATA_DIR=%s",data_dir))begin
            data_dir="../../../../../reference/p7b_layer2_0";
            if(!exists({data_dir,"/post_vectors.mem"}))data_dir="reference/p7b_layer2_0";
        end
        if(!exists({data_dir,"/post_vectors.mem"}))$fatal(1,"P7-B: set +P7B_DATA_DIR=<data directory>");
        $readmemh({data_dir,"/post_vectors.mem"},vectors);
        for(int i=0;i<256;i++)if($isunknown(vectors[i]))$fatal(1,"P7-B incomplete post vectors");
        repeat(4)@(negedge clk);rst_n=1;
        for(int i=0;i<256;i++)begin
            @(negedge clk);in_valid=1;
            in_residual_en=vectors[i][160];in_relu_en=vectors[i][161];
            in_skip=vectors[i][159:128];in_multiplier=vectors[i][127:96];
            in_bias=vectors[i][95:64];in_acc=vectors[i][63:32];expected_now=vectors[i][31:0];
            if(i%7==3)begin @(negedge clk);in_valid=0;end
        end
        @(negedge clk);in_valid=0;
        repeat(4)@(negedge clk);
        if(checked!=256)$fatal(1,"P7-B missing post outputs");
        $display("PASS: P7-B signed shortcut/requant 256/256 vectors, no premature ReLU, signed rounding and valid bubbles");
        $finish;
    end
    initial begin #100000;$fatal(1,"P7-B post timeout");end
endmodule
