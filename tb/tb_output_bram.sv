`timescale 1ns / 1ps

// Standalone smoke-test: verifies synchronous read, channel mask,
// zero padding, and multi-address storage without the accelerator.
module tb_output_bram;
    localparam int LANES=8;
    localparam int WIDTH=32;
    logic clk=0;
    always #5 clk=~clk;

    logic rst_n;
    logic wr_en;
    logic [3:0] wr_addr, rd_addr;
    logic [LANES*WIDTH-1:0] wr_data, rd_data;
    logic [LANES-1:0] wr_mask, rd_mask;
    logic rd_en, rd_valid;
    integer expected;

    output_bram #(.ADDR_WIDTH(4), .DATA_WIDTH(WIDTH),
                  .LANES(LANES), .DEPTH(9)) dut (
      .clk(clk), .rst_n(rst_n),
      .wr_en(wr_en), .wr_addr(wr_addr),
      .wr_data(wr_data), .wr_mask(wr_mask),
      .rd_en(rd_en), .rd_addr(rd_addr),
      .rd_data(rd_data), .rd_mask(rd_mask), .rd_valid(rd_valid)
    );

    initial begin
        rst_n=0;
        wr_en=0; wr_addr=0; wr_data='0; wr_mask='0;
        rd_en=0; rd_addr=0;
        repeat(2) @(negedge clk);
        rst_n=1;

        // Store only first four lanes at address 0.
        @(negedge clk);
        wr_en=1; wr_addr=0; wr_mask=8'h0f;
        for(int i=0;i<LANES;i++) wr_data[i*WIDTH +: WIDTH] = 32'(100+i);
        @(negedge clk);
        // Store all 8 lanes at address 1.
        wr_addr=1; wr_mask=8'hff;
        for(int i=0;i<LANES;i++) wr_data[i*WIDTH +: WIDTH] = 32'(200+i);
        @(negedge clk);
        wr_en=0;

        // Read address 0, after output register updates on rising edge.
        rd_en=1; rd_addr=0;
        @(posedge clk); #1;
        if (!rd_valid || rd_mask !== 8'h0f)
            $fatal(1,"FAIL: addr0 valid/mask");
        for(int i=0;i<LANES;i++) begin
            expected=(i<4)?(100+i):0;
            if($signed(rd_data[i*WIDTH +: WIDTH]) !== expected)
                $fatal(1,"FAIL: addr0 lane%0d",i);
        end
        @(negedge clk);
        rd_addr=1;
        @(posedge clk); #1;
        if (!rd_valid || rd_mask !== 8'hff)
            $fatal(1,"FAIL: addr1 valid/mask");
        for(int i=0;i<LANES;i++)
            if($signed(rd_data[i*WIDTH +: WIDTH]) !== (200+i))
                $fatal(1,"FAIL: addr1 lane%0d",i);
        @(negedge clk);
        rd_en=0;
        @(posedge clk); #1;
        if (rd_valid !== 1'b0)
            $fatal(1,"FAIL: rd_valid did not drop");
        $display("PASS: output_bram standalone; 2 writes; 2 reads; mask + zero padding correct");
        $finish;
    end
    initial begin
        #2000;
        $fatal(1,"TIMEOUT: output_bram standalone");
    end
endmodule
