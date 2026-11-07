`timescale 1ns / 1ps

// P4-A activation-reader-only verification: NO compute array or weight reader.
// Tests two full preloads with different signed input data, and checks all
// 16 M positions x 32 K-pairs in a different permutation per epoch.
// Drives requests at negedge and samples after the following posedge +1ns.
module tb_conv_bram_reader_p4_1x1;
    localparam int H=8, W=8, C=64, M=16, K=64;
    localparam int EPOCHS=2, PAIRS_PER_EPOCH=M*(K/2);

    logic clk=1'b0;
    always #5 clk=~clk;
    logic rst_n, pair_req_en, wr_en;
    logic [15:0] m_addr, fetch_k_base, wr_addr;
    logic [7:0] wr_data;
    logic [15:0] act_pair_packed;
    logic [1:0] act_rsp_valid;

    conv_bram_reader_p4_1x1 dut (
        .clk(clk), .rst_n(rst_n),
        .m_addr(m_addr), .pair_req_en(pair_req_en),
        .fetch_k_base(fetch_k_base),
        .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data),
        .act_pair_packed(act_pair_packed), .act_rsp_valid(act_rsp_valid)
    );

    // Independent golden source: HWC map, NOT generated from DUT address.
    logic signed [7:0] golden_fmap [0:7][0:7][0:63];
    integer hits [0:1][0:15][0:31];
    integer epoch_active, accepted_count, response_count, idle_checks;
    integer last_m, last_k, last_epoch;
    bit finished;

    function automatic integer golden_value(
        input integer y, x, c, epoch
    );
        // Covers positive and negative INT8 values; differs by epoch.
        return ((17*y + 11*x + 7*c + 37*epoch) % 255) - 127;
    endfunction

    // Scoreboard: count each request and check its newly registered result.
    // The two BRAMs and response-valid register all sample the same posedge.
    always @(posedge clk) begin : scoreboard
        bit accepted;
        integer em, ek, ee, pair_addr, oy, ox;
        integer want0, want1;
        accepted = rst_n && pair_req_en && !wr_en;
        em = int'($unsigned(m_addr));
        ek = int'($unsigned(fetch_k_base));
        ee = epoch_active;

        if (accepted) begin
            if (em<0 || em>=M || ek<0 || ek>=K || (ek%2)!=0)
                $fatal(1, "FAIL invalid stimulus M=%0d K=%0d", em, ek);
            oy = em / 4;
            ox = em % 4;
            pair_addr = (((2*oy)*W + (2*ox))*C + ek) / 2;
            if (dut.bank_rd_en !== 2'b11 ||
                dut.bank_rd_addr[0] !== 16'(pair_addr) ||
                dut.bank_rd_addr[1] !== 16'(pair_addr))
                $fatal(1, "FAIL address epoch=%0d M=%0d K=%0d want=%0d got=%0d",
                    ee, em, ek, pair_addr, dut.bank_rd_addr[0]);
            hits[ee][em][ek/2] = hits[ee][em][ek/2] + 1;
            accepted_count = accepted_count + 1;
        end else if (!wr_en && dut.bank_rd_en !== 2'b00) begin
            $fatal(1, "FAIL: BRAM read enable during idle");
        end

        #1;
        if (accepted) begin
            if (act_rsp_valid !== 2'b11 ||
                dut.rsp_m_addr !== 16'(em) ||
                dut.rsp_k_base !== 16'(ek))
                $fatal(1, "FAIL response valid/tag epoch=%0d M=%0d K=%0d",
                    ee, em, ek);
            want0 = int'($signed(golden_fmap[2*(em/4)][2*(em%4)][ek]));
            want1 = int'($signed(golden_fmap[2*(em/4)][2*(em%4)][ek+1]));
            if (int'($signed(act_pair_packed[7:0])) !== want0 ||
                int'($signed(act_pair_packed[15:8])) !== want1)
                $fatal(1,
                    "FAIL epoch=%0d M=%0d K=%0d got=(%0d,%0d) want=(%0d,%0d)",
                    ee, em, ek,
                    int'($signed(act_pair_packed[7:0])),
                    int'($signed(act_pair_packed[15:8])), want0, want1);
            response_count = response_count + 1;
            last_m = em;
            last_k = ek;
            last_epoch = ee;
        end else if (act_rsp_valid !== 2'b00 || act_pair_packed !== 16'b0) begin
            $fatal(1, "FAIL stale output when no valid request");
        end else begin
            idle_checks = idle_checks + 1;
        end
    end

    task automatic load_epoch(input integer e);
        integer a, y, x, c;
        begin
            @(negedge clk);
            rst_n=0;
            wr_en=0;
            pair_req_en=0;
            epoch_active=e;
            for (y=0; y<H; y++)
                for (x=0; x<W; x++)
                    for (c=0; c<C; c++) begin
                        a=(y*W+x)*C+c;
                        golden_fmap[y][x][c]=8'(golden_value(y,x,c,e));
                        @(negedge clk);
                        wr_en=1;
                        wr_addr=16'(a);
                        wr_data=golden_fmap[y][x][c];
                    end
            @(negedge clk);
            wr_en=0;
            rst_n=1;
            repeat (2) @(negedge clk);
        end
    endtask

    task automatic scan_epoch(input integer e);
        integer idx, m, k;
        begin
            // 73 is coprime with 512 -> a permutation of all M/K pairs.
            for (int i=0; i<PAIRS_PER_EPOCH; i++) begin
                idx=(73*i + 19*e) % PAIRS_PER_EPOCH;
                m=idx/32;
                k=2*(idx%32);
                @(negedge clk);
                pair_req_en=1;
                m_addr=16'(m);
                fetch_k_base=16'(k);
            end
            @(negedge clk);
            pair_req_en=0;
            repeat (3) @(negedge clk);
        end
    endtask

    initial begin : stimulus
        rst_n=0;
        wr_en=0;
        wr_addr=0;
        wr_data=0;
        pair_req_en=0;
        m_addr=0;
        fetch_k_base=0;
        epoch_active=0;
        accepted_count=0;
        response_count=0;
        idle_checks=0;
        last_m=0;
        last_k=0;
        last_epoch=0;
        finished=0;
        for (int e=0; e<EPOCHS; e++)
            for (int m=0; m<M; m++)
                for (int k=0; k<K/2; k++)
                    hits[e][m][k]=0;

        repeat (3) @(negedge clk);
        for (int e=0; e<EPOCHS; e++) begin
            load_epoch(e);
            scan_epoch(e);
            if (accepted_count != (e+1)*PAIRS_PER_EPOCH ||
                response_count != (e+1)*PAIRS_PER_EPOCH)
                $fatal(1, "FAIL epoch counts epoch=%0d req=%0d rsp=%0d",
                       e, accepted_count, response_count);
            for (int m=0; m<M; m++)
                for (int k=0; k<K/2; k++)
                    if (hits[e][m][k]!=1)
                        $fatal(1, "FAIL coverage epoch=%0d M=%0d Kpair=%0d hits=%0d",
                               e,m,k,hits[e][m][k]);
            $display("PASS: epoch=%0d 512/512 activation pairs checked",e);
        end

        if (idle_checks == 0)
            $fatal(1,"FAIL no idle cycles observed");
        finished=1;
        $display("PASS: P4-A 1x1/S2 Reader, 2 epochs, %0d/%0d pairs, %0d INT8 values",
                 response_count,EPOCHS*PAIRS_PER_EPOCH,2*response_count);
        $finish;
    end

    initial begin : watchdog
        repeat (15000) @(posedge clk);
        if (!finished)
            $fatal(1,"TIMEOUT P4-A Reader TB");
    end
endmodule
