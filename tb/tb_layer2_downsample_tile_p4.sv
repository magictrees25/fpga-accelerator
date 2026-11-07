`timescale 1ns / 1ps

// P4-B: COMPLETE local Tile, 1x1 convolution, stride2, padding0.
// One DUT, one reset, two consecutive epochs (different activation/weight data).
// Input: 8x8x64, Output: 4x4x8 (M16,N8,K64), 128 INT32 outputs/epoch.
// Golden reference uses independent spatial/channel multiply-and-sum loops.
// No assertion of FPGA board timing; 10 ns is the simulation clock only.
module tb_layer2_downsample_tile_p4;
    localparam int H=8, W=8, C=64, OH=4, OW=4, N=8;
    localparam int M=OH*OW, K=C, EPOCHS=2;
    localparam int ACT_BYTES=H*W*C, WEIGHT_WORDS=K;
    localparam int EXPECT_FEED=M*(K/2);
    localparam int EXPECT_CYCLES=M*(1 + K/2 + 13 + 1 + 1 + 1);

    logic clk=1'b0;
    always #5 clk=~clk;
    logic rst_n,start,fmap_wr_en,weight_wr_en,out_rd_en;
    logic [15:0] fmap_wr_addr,weight_wr_addr,out_rd_addr;
    logic [7:0] fmap_wr_data;
    logic [63:0] weight_wr_data;
    logic [2:0] out_rd_lane_sel;
    logic signed [31:0] out_rd_data;
    logic out_rd_valid,out_rd_lane_valid,busy,done,any_overflow;

    fpga_system_top_layer2_downsample_tile_p4 dut (.*);

    logic signed [7:0] fmap[0:H-1][0:W-1][0:C-1];
    logic signed [7:0] weights[0:K-1][0:N-1];
    logic signed [31:0] golden[0:M-1][0:N-1];

    integer epoch;
    integer reads_checked, writes_checked, feeds_checked, reqs_checked;
    integer clears_checked, drains_checked, launches_checked, kdones_checked;
    integer next_feed_m,next_feed_k, cycles_checked;
    integer accepted_starts, completed_runs;
    integer tick, start_tick, done_tick;
    bit computing, done_q, finished;

    function automatic integer a_pattern(input integer y,x,c,e);
        integer v;
        begin
            v = ((17*y+11*x+7*c+37*e)%255)-127;
            if (y==0 && x==0 && c==0) v=-128;
            if (y==6 && x==6 && c==63) v=127;
            return v;
        end
    endfunction

    function automatic integer w_pattern(input integer c,n,e);
        integer v;
        begin
            v = ((13*c+5*n+41*e)%31)-15;
            if(c==0 && n==0) v=-128;
            if(c==63 && n==7) v=127;
            return v;
        end
    endfunction

    task automatic prepare_reference(input integer e);
        integer sum;
        begin
            for(int y=0;y<H;y++)
                for(int x=0;x<W;x++)
                    for(int c=0;c<C;c++)
                        fmap[y][x][c]=8'(a_pattern(y,x,c,e));
            for(int c=0;c<C;c++)
                for(int n=0;n<N;n++)
                    weights[c][n]=8'(w_pattern(c,n,e));
            for(int oy=0;oy<OH;oy++)
                for(int ox=0;ox<OW;ox++)
                    for(int n=0;n<N;n++) begin
                        sum=0;
                        for(int c=0;c<C;c++)
                            sum += int'($signed(fmap[oy*2][ox*2][c])) *
                                   int'($signed(weights[c][n]));
                        golden[oy*OW+ox][n]=sum;
                    end
        end
    endtask

    task automatic load_epoch;
        integer addr;
        begin
            // All inputs are driven at falling edges; no port changes at posedge.
            for(int y=0;y<H;y++)
                for(int x=0;x<W;x++)
                    for(int c=0;c<C;c++) begin
                        addr=(y*W+x)*C+c;
                        @(negedge clk);
                        fmap_wr_en=1;
                        fmap_wr_addr=16'(addr);
                        fmap_wr_data=fmap[y][x][c];
                    end
            @(negedge clk);
            fmap_wr_en=0;
            for(int c=0;c<C;c++) begin
                @(negedge clk);
                weight_wr_en=1;
                weight_wr_addr=16'(c);
                for(int n=0;n<N;n++)
                    weight_wr_data[n*8 +: 8]=8'(weights[c][n]);
            end
            @(negedge clk);
            weight_wr_en=0;
        end
    endtask

    // Functional checks at pre-NBA edge: exactly the values consumed by PE.
    always @(posedge clk) begin : monitor
        bit was_computing;
        integer want_a0,want_a1,got_a0,got_a1,got_w;
        integer m,k;
        if (!rst_n) begin
            tick=0;
            computing=0;
            done_q=0;
        end else begin
            tick++;
            was_computing=computing;
            if (start) begin
                if (computing || busy || done ||
                    accepted_starts!=epoch || completed_runs!=epoch)
                    $fatal(1,"FAIL: P4 improper launch epoch=%0d",epoch);
                accepted_starts++;
                start_tick=tick;
                cycles_checked=0;
                computing=1;
            end
            if (was_computing) cycles_checked++;
            if ($isunknown({busy,done,any_overflow}) || any_overflow !== 1'b0)
                $fatal(1,"FAIL: P4 unknown/overflow");
            if (int'(was_computing)+int'(fmap_wr_en)+
                int'(weight_wr_en)+int'(out_rd_en)>1)
                $fatal(1,"FAIL: overlapping host/compute phases");

            if (dut.u_compute.clear_acc) begin
                if(!computing || next_feed_k!=0 ||
                   clears_checked!=writes_checked)
                    $fatal(1,"FAIL: clear sequence epoch=%0d",epoch);
                clears_checked++;
            end
            if (dut.u_compute.u_ctrl.k_start) launches_checked++;
            if (dut.u_compute.u_ctrl.k_done) kdones_checked++;
            if (dut.u_compute.u_ctrl.array_enable &&
                !dut.u_compute.clear_acc && !dut.u_compute.feed_valid)
                drains_checked++;

            if (dut.u_compute.pair_req_en) begin
                if(!computing ||
                   dut.u_compute.u_act_reader.bank_rd_en !== 2'b11 ||
                   dut.u_compute.u_weight_reader.bank_rd_en !== 2'b11)
                    $fatal(1,"FAIL: P4 pair request/bank enables");
                reqs_checked++;
            end
            if (dut.u_compute.feed_valid) begin
                m=next_feed_m;
                k=next_feed_k;
                if(!computing || m>=M || k>=K ||
                   dut.u_compute.m_addr !== 16'(m) ||
                   dut.u_compute.k_base !== 16'(k) ||
                   dut.u_compute.n_base !== 16'd0 ||
                   dut.u_compute.act_rsp_valid !== 2'b11 ||
                   dut.u_compute.wgt_rsp_valid !== 2'b11 ||
                   dut.u_compute.u_act_reader.rsp_m_addr !== 16'(m) ||
                   dut.u_compute.u_act_reader.rsp_k_base !== 16'(k) ||
                   dut.u_compute.u_weight_reader.rsp_k_base !== 16'(k))
                    $fatal(1,"FAIL: P4 feed control/tag M=%0d K=%0d",m,k);
                want_a0=int'($signed(fmap[2*(m/OW)][2*(m%OW)][k]));
                want_a1=int'($signed(fmap[2*(m/OW)][2*(m%OW)][k+1]));
                got_a0=int'($signed(dut.u_compute.act_pair_packed[7:0]));
                got_a1=int'($signed(dut.u_compute.act_pair_packed[15:8]));
                if (got_a0!==want_a0 || got_a1!==want_a1)
                    $fatal(1,"FAIL: P4 activation M=%0d K=%0d got=%0d,%0d want=%0d,%0d",
                           m,k,got_a0,got_a1,want_a0,want_a1);
                for (int p=0;p<2;p++)
                    for (int n=0;n<N;n++) begin
                        got_w=int'($signed(
                            dut.u_compute.wgt_pair_packed[(p*8+n)*8 +: 8]));
                        if (got_w !== int'($signed(weights[k+p][n])))
                            $fatal(1,"FAIL: P4 weight K=%0d n=%0d",k+p,n);
                    end
                feeds_checked++;
                if(k==K-2) begin
                    next_feed_k=0;
                    next_feed_m++;
                end else next_feed_k=k+2;
            end
            if (dut.write_valid) begin
                m=writes_checked;
                if(!computing || m>=M ||
                   dut.write_addr!==16'(m) ||
                   dut.write_m_addr!==16'(m) ||
                   dut.write_n_base!==16'd0 ||
                   dut.write_mask!==8'hff)
                    $fatal(1,"FAIL: P4 output write metadata M=%0d",m);
                for(int n=0;n<N;n++) begin
                    if(int'($signed(dut.write_data[n*32 +: 32])) !==
                       int'($signed(golden[m][n])))
                        $fatal(1,"FAIL: P4 write M=%0d n=%0d got=%0d want=%0d",
                            m,n,$signed(dut.write_data[n*32 +: 32]),golden[m][n]);
                end
                writes_checked++;
            end
            #1;
            if(done && done_q) $fatal(1,"FAIL: P4 done wider than one cycle");
            if(done) begin
                if(!computing || writes_checked!=M ||
                   feeds_checked!=EXPECT_FEED || reqs_checked!=EXPECT_FEED ||
                   clears_checked!=M || drains_checked!=M*13 ||
                   launches_checked!=M || kdones_checked!=M ||
                   next_feed_m!=M || next_feed_k!=0)
                    $fatal(1,"FAIL: P4 phase totals at done epoch=%0d",epoch);
                computing=0;
                done_tick=tick;
                completed_runs++;
                if (done_tick-start_tick != cycles_checked ||
                    cycles_checked!=EXPECT_CYCLES)
                    $fatal(1,"FAIL: P4 compute cycles got=%0d expected=%0d",
                           cycles_checked,EXPECT_CYCLES);
            end
            done_q=done;
        end
    end

    task automatic read_and_verify;
        integer got,want;
        begin
            for(int m=0;m<M;m++)
                for(int n=0;n<N;n++) begin
                    @(negedge clk);
                    out_rd_en=1;
                    out_rd_addr=16'(m);
                    out_rd_lane_sel=3'(n);
                    @(posedge clk);
                    #2;
                    got=int'($signed(out_rd_data));
                    want=int'($signed(golden[m][n]));
                    if(out_rd_valid!==1'b1 || out_rd_lane_valid!==1'b1 ||
                       got!==want)
                        $fatal(1,"FAIL: P4 output epoch=%0d m=%0d n=%0d got=%0d want=%0d",
                               epoch,m,n,got,want);
                    reads_checked++;
                end
            @(negedge clk);
            out_rd_en=0;
            @(posedge clk);
            #2;
            if(out_rd_valid!==1'b0)
                $fatal(1,"FAIL: P4 stale output valid");
        end
    endtask

    initial begin : stimulus
        integer waited;
        rst_n=0;start=0;
        fmap_wr_en=0;fmap_wr_addr=0;fmap_wr_data=0;
        weight_wr_en=0;weight_wr_addr=0;weight_wr_data=0;
        out_rd_en=0;out_rd_addr=0;out_rd_lane_sel=0;
        epoch=0;finished=0;accepted_starts=0;completed_runs=0;
        reads_checked=0;writes_checked=0;feeds_checked=0;reqs_checked=0;
        clears_checked=0;drains_checked=0;launches_checked=0;kdones_checked=0;
        next_feed_m=0;next_feed_k=0;cycles_checked=0;
        tick=0;start_tick=0;done_tick=0;computing=0;done_q=0;

        repeat(4) @(negedge clk);
        rst_n=1;
        for(int e=0;e<EPOCHS;e++) begin
            epoch=e;
            reads_checked=0;writes_checked=0;feeds_checked=0;reqs_checked=0;
            clears_checked=0;drains_checked=0;launches_checked=0;kdones_checked=0;
            next_feed_m=0;next_feed_k=0;cycles_checked=0;
            prepare_reference(e);
            load_epoch();
            @(negedge clk);
            if(busy || done || computing)
                $fatal(1,"FAIL: P4 not idle before start epoch=%0d",e);
            start=1;
            @(negedge clk);
            start=0;
            waited=0;
            while (done!==1'b1 && waited < 2000) begin
                @(posedge clk);
                #2;
                waited++;
            end
            if(done!==1'b1)
                $fatal(1,"TIMEOUT: P4 wait done epoch=%0d",e);
            read_and_verify();
            if(reads_checked != M*N || writes_checked!=M ||
               feeds_checked!=EXPECT_FEED || completed_runs!=e+1)
                $fatal(1,"FAIL: P4 epoch scoreboard e=%0d",e);
            $display("PASS: P4-B epoch=%0d 128/128 INT32 outputs, 512/512 K-pairs, %0d compute cycles",
                     e,cycles_checked);
            repeat(3) @(negedge clk);
        end
        finished=1;
        $display("PASS: P4-B 1x1/S2 complete Tile, 2 epochs, 256/256 INT32 outputs, no reset between runs");
        $finish;
    end
    initial begin : watchdog
        repeat(25000) @(posedge clk);
        if(!finished) $fatal(1,"TIMEOUT: P4-B full Tile TB");
    end
endmodule
