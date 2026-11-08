`timescale 1ns / 1ps

// P4-D3: full ResNet-18 layer2.0.conv1 3x3/S2/P1 convolution.
// Global input 56x56x64 -> global output 28x28x128.
// 7x7 M-Tiles x 16 N-Tiles = 784 computations. One DUT, no reset between tiles.
// Each tile: pre-padded 9x9x64 input -> 4x4x8 output, GEMM M=16,N=8,K=576.
// Independent global-coordinate direct-convolution Golden Model (not DUT addresses).
// Checks 3,612,672 K-pairs, 100,352 INT32 outputs, exact scatter ownership,
// halo padding at only top/left edges, reloaded weights and synchronous latency.
// Simulation clock 10ns, not board/DDR measured latency.
module tb_layer2_conv1_784tiles_p4;
    localparam int H=9, W=9, C=64, OH=4, OW=4, N=8;
    localparam int M=OH*OW, K=3*3*C;
    localparam int SPATIAL_TY=7, SPATIAL_TX=7, CHANNEL_GROUPS=16;
    localparam int NUM_M_TILES=SPATIAL_TY*SPATIAL_TX;
    localparam int TOTAL_TILES=NUM_M_TILES*CHANNEL_GROUPS;
    localparam int G_IN_H=56, G_IN_W=56, G_OUT_H=28, G_OUT_W=28, G_N=128;
    localparam int GLOBAL_WORDS=G_OUT_H*G_OUT_W*G_N;
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

    fpga_system_top_layer2_conv1_tile_p4 dut (.*);

    logic signed [7:0] fmap[0:H-1][0:W-1][0:C-1];
    logic signed [7:0] weights[0:K-1][0:N-1];
    logic signed [31:0] golden[0:M-1][0:N-1];
    logic signed [7:0] global_fmap[0:G_IN_H-1][0:G_IN_W-1][0:C-1];
    logic signed [7:0] global_kernel[0:2][0:2][0:C-1][0:G_N-1];
    logic signed [31:0] global_golden[0:G_OUT_H-1][0:G_OUT_W-1][0:G_N-1];
    logic signed [31:0] stitched[0:GLOBAL_WORDS-1];
    integer scatter_hits[0:GLOBAL_WORDS-1];
    integer scatter_owner[0:GLOBAL_WORDS-1];
    integer tile_y, tile_x, channel_group, oy0, ox0, oc0;
    integer total_reads, total_writes, total_feeds;
    integer total_act_load_bytes, total_weight_load_words;
    integer act_load_this_tile, weight_load_this_tile;
    integer pad_bytes_this_tile, total_padding_bytes;
    longint unsigned total_compute_cycles;
    integer complete_tile_count;

    integer epoch;
    integer reads_checked, writes_checked, feeds_checked, reqs_checked;
    integer clears_checked, drains_checked, launches_checked, kdones_checked;
    integer next_feed_m,next_feed_k, cycles_checked;
    integer accepted_starts, completed_runs;
    integer tick, start_tick, done_tick;
    bit computing, done_q, finished;

    // Use the same epoch=0 input/weight pattern as the PASSed P4-D2 single-
    // tile TB. n here is a *global* output channel in 0..127.
    function automatic integer a_pattern(input integer y,x,c);
        integer v;
        begin
            v=((17*y+11*x+7*c)%255)-127;
            if(y==0 && x==0 && c==0) v=-128;
            if(y==54 && x==54 && c==63) v=127;
            return v;
        end
    endfunction

    function automatic integer w_pattern(input integer ky,kx,c,n);
        integer v;
        begin
            v=((13*ky+7*kx+3*c+5*n)%31)-15;
            if(ky==0 && kx==0 && c==0 && n==0) v=-128;
            if(ky==2 && kx==2 && c==63 && n==7) v=127;
            return v;
        end
    endfunction

    // Compute entire 28x28x128 golden from GLOBAL input and GLOBAL weights.
    // No local Patch, DUT address or Tile origin is consulted here.
    task automatic prepare_global_reference;
        integer signed sum;
        integer gy,gx;
        begin
            for(int y=0;y<G_IN_H;y++)
                for(int x=0;x<G_IN_W;x++)
                    for(int c=0;c<C;c++)
                        global_fmap[y][x][c]=8'(a_pattern(y,x,c));
            for(int ky=0;ky<3;ky++)
                for(int kx=0;kx<3;kx++)
                    for(int c=0;c<C;c++)
                        for(int n=0;n<G_N;n++)
                            global_kernel[ky][kx][c][n]=8'(w_pattern(ky,kx,c,n));
            for(int oy=0;oy<G_OUT_H;oy++)
                for(int ox=0;ox<G_OUT_W;ox++)
                    for(int n=0;n<G_N;n++) begin
                        sum=0;
                        for(int ky=0;ky<3;ky++)
                            for(int kx=0;kx<3;kx++)
                                for(int c=0;c<C;c++) begin
                                    gy=2*oy+ky-1;
                                    gx=2*ox+kx-1;
                                    if(gy>=0 && gy<G_IN_H &&
                                       gx>=0 && gx<G_IN_W)
                                        sum += int'($signed(global_fmap[gy][gx][c])) *
                                               int'($signed(global_kernel[ky][kx][c][n]));
                                end
                        global_golden[oy][ox][n]=sum;
                    end
            // Anchors verified with independent Python/Numpy integer GEMM.
            if(global_golden[0][0][0] !== -32'sd3739 ||
               global_golden[0][0][7] !== -32'sd3095 ||
               global_golden[0][0][8] !== -32'sd10230 ||
               global_golden[0][0][127] !== 32'sd922 ||
               global_golden[3][3][7] !== 32'sd2951 ||
               global_golden[4][4][8] !== 32'sd5068 ||
               global_golden[12][20][63] !== 32'sd5964 ||
               global_golden[24][24][120] !== -32'sd14709 ||
               global_golden[27][27][127] !== 32'sd2046 ||
               global_golden[0][27][50] !== 32'sd1262 ||
               global_golden[27][0][100] !== 32'sd6027 ||
               global_golden[17][13][37] !== -32'sd9592)
                $fatal(1,"FAIL: P4-D3 independent global Golden anchors");
            $display("INFO: P4-D3 global 28x28x128 3x3/S2/P1 Golden prepared");
        end
    endtask

    // Local reader sees 9x9x64 padded HWC. At global top/left edges,
    // pad with zero; never dereference an out-of-range global_fmap address.
    // Output Golden stays a GLOBAL look-up, not recomputed via local BRAM.
    task automatic prepare_tile_reference;
        integer gy,gx,k,ky,kx,ci;
        integer expected_pad;
        begin
            pad_bytes_this_tile=0;
            for(int y=0;y<H;y++)
                for(int x=0;x<W;x++)
                    for(int c=0;c<C;c++) begin
                        gy=2*oy0+y-1;
                        gx=2*ox0+x-1;
                        if(gy<0 || gy>=G_IN_H || gx<0 || gx>=G_IN_W) begin
                            fmap[y][x][c]=8'd0;
                            pad_bytes_this_tile++;
                        end else
                            fmap[y][x][c]=global_fmap[gy][gx][c];
                    end
            // 9x9 patch intersects global padding only at y=-1 / x=-1.
            expected_pad=((oy0==0) ? W*C : 0) +
                         ((ox0==0) ? H*C : 0) -
                         ((oy0==0 && ox0==0) ? C : 0);
            if(pad_bytes_this_tile!=expected_pad)
                $fatal(1,"FAIL: P4-D3 tile padding origin=(%0d,%0d) got=%0d want=%0d",
                       oy0,ox0,pad_bytes_this_tile,expected_pad);
            for(int ky_i=0;ky_i<3;ky_i++)
                for(int kx_i=0;kx_i<3;kx_i++)
                    for(int c=0;c<C;c++)
                        for(int n=0;n<N;n++) begin
                            k=(ky_i*3+kx_i)*C+c;
                            weights[k][n]=global_kernel[ky_i][kx_i][c][oc0+n];
                        end
            for(int m=0;m<M;m++)
                for(int n=0;n<N;n++)
                    golden[m][n]=global_golden[oy0+m/OW][ox0+m%OW][oc0+n];
        end
    endtask

    // Preload full halo patch and all 576 packed 8-channel weight words.
    task automatic load_epoch;
        integer addr;
        begin
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
            for(int k=0;k<K;k++) begin
                @(negedge clk);
                weight_wr_en=1;
                weight_wr_addr=16'(k);
                for(int n=0;n<N;n++)
                    weight_wr_data[n*8 +: 8]=8'(weights[k][n]);
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
                if(act_load_this_tile != ACT_BYTES ||
                   weight_load_this_tile != WEIGHT_WORDS)
                    $fatal(1,"FAIL: preload counts before start tile=%0d",epoch);
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
            if (fmap_wr_en) begin
                total_act_load_bytes++;
                act_load_this_tile++;
                if (fmap_wr_addr!==16'(act_load_this_tile-1))
                    $fatal(1,"FAIL: activation preload ordering tile=%0d",epoch);
            end
            if (weight_wr_en) begin
                total_weight_load_words++;
                weight_load_this_tile++;
                if (weight_wr_addr!==16'(weight_load_this_tile-1))
                    $fatal(1,"FAIL: weight preload ordering tile=%0d",epoch);
            end

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
                begin
                    integer ky,kx,ci;
                    ky=(k/64)/3;
                    kx=(k/64)%3;
                    ci=k%64;
                    want_a0=int'($signed(fmap[2*(m/OW)+ky][2*(m%OW)+kx][ci]));
                    want_a1=int'($signed(fmap[2*(m/OW)+ky][2*(m%OW)+kx][ci+1]));
                end
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
                    begin
                        integer gy,gx,gn,addr,expected_owner;
                        gy=oy0+m/OW;
                        gx=ox0+m%OW;
                        gn=oc0+n;
                        addr=(gy*G_OUT_W+gx)*G_N+gn;
                        expected_owner=(((gy/OH)*SPATIAL_TX)+(gx/OW))*CHANNEL_GROUPS+(gn/N);
                        if(scatter_hits[addr]!=0)
                            $fatal(1,"FAIL: duplicate global output addr=%0d old owner=%0d new=%0d",
                                   addr,scatter_owner[addr],epoch);
                        if(expected_owner!=epoch)
                            $fatal(1,"FAIL: wrong global owner addr=%0d got=%0d want=%0d",
                                   addr,epoch,expected_owner);
                        scatter_hits[addr]=1;
                        scatter_owner[addr]=epoch;
                        stitched[addr]=out_rd_data;
                    end
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

    // Verify complete 28x28x128 output coverage and Golden checksum.
    task automatic check_global_assembly;
        longint signed checksum;
        integer total;
        begin
            checksum=0;
            total=0;
            for (int oy=0;oy<G_OUT_H;oy++)
                for (int ox=0;ox<G_OUT_W;ox++)
                    for (int n=0;n<G_N;n++) begin
                        integer a,owner;
                        a=(oy*G_OUT_W+ox)*G_N+n;
                        owner=((oy/OH)*SPATIAL_TX+ox/OW)*CHANNEL_GROUPS+n/N;
                        if (scatter_hits[a]!==1 || scatter_owner[a]!==owner ||
                            stitched[a]!==global_golden[oy][ox][n])
                            $fatal(1,"FAIL: full scatter y=%0d x=%0d n=%0d hits=%0d owner=%0d expected=%0d",
                                   oy,ox,n,scatter_hits[a],scatter_owner[a],owner);
                        total++;
                        checksum+=longint'($signed(stitched[a]));
                    end
            if (total!=GLOBAL_WORDS || checksum!=-64'sd28679)
                $fatal(1,"FAIL: global count/checksum got count=%0d sum=%0d",
                       total,checksum);
            $display("PASS: P4-D3 global 28x28x128 stitched: %0d/%0d INT32 outputs, checksum=%0d",
                     total,GLOBAL_WORDS,checksum);
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
        tile_y=0;tile_x=0;channel_group=0;oy0=0;ox0=0;oc0=0;
        total_reads=0;total_writes=0;total_feeds=0;
        total_act_load_bytes=0;total_weight_load_words=0;
        act_load_this_tile=0;weight_load_this_tile=0;
        pad_bytes_this_tile=0;total_padding_bytes=0;
        total_compute_cycles=0;complete_tile_count=0;
        for(int a=0;a<GLOBAL_WORDS;a++) begin
            scatter_hits[a]=0;
            scatter_owner[a]=-1;
            stitched[a]=32'h5a55a55a;
        end

        prepare_global_reference();
        repeat(4) @(negedge clk);
        rst_n=1;
        for(int ty=0;ty<SPATIAL_TY;ty++)
            for(int tx=0;tx<SPATIAL_TX;tx++)
                for(int ng=0;ng<CHANNEL_GROUPS;ng++) begin
                    tile_y=ty; tile_x=tx; channel_group=ng;
                    epoch=(ty*SPATIAL_TX+tx)*CHANNEL_GROUPS+ng;
                    oy0=ty*OH; ox0=tx*OW; oc0=ng*N;
                    reads_checked=0;writes_checked=0;
                    feeds_checked=0;reqs_checked=0;
                    clears_checked=0;drains_checked=0;
                    launches_checked=0;kdones_checked=0;
                    next_feed_m=0;next_feed_k=0;cycles_checked=0;
                    act_load_this_tile=0;weight_load_this_tile=0;
                    if (busy!==0 || done!==0 || computing)
                        $fatal(1,"FAIL: nonidle between tiles tile=%0d",epoch);
                    prepare_tile_reference();
                    total_padding_bytes+=pad_bytes_this_tile;
                    load_epoch();
                    @(negedge clk);
                    if(busy || done || computing)
                        $fatal(1,"FAIL: P4-D3 not idle before start tile=%0d",epoch);
                    start=1;
                    @(negedge clk);
                    start=0;
                    waited=0;
                    while (done!==1'b1 && waited < 8000) begin
                        @(posedge clk);
                        #2;
                        waited++;
                    end
                    if(done!==1'b1)
                        $fatal(1,"TIMEOUT: P4-D3 wait done tile=%0d",epoch);
                    read_and_verify();
                    if(reads_checked!=M*N || writes_checked!=M ||
                       feeds_checked!=EXPECT_FEED || reqs_checked!=EXPECT_FEED ||
                       clears_checked!=M || drains_checked!=M*13 ||
                       launches_checked!=M || kdones_checked!=M ||
                       cycles_checked!=EXPECT_CYCLES ||
                       completed_runs!=epoch+1)
                        $fatal(1,"FAIL: P4-D3 tile scoreboard tile=%0d",epoch);
                    total_reads+=reads_checked;
                    total_writes+=writes_checked;
                    total_feeds+=feeds_checked;
                    total_compute_cycles+=cycles_checked;
                    complete_tile_count++;
                    // Print once per 4x4x128 spatial group (49 progress lines).
                    if(ng==CHANNEL_GROUPS-1)
                        $display("PASS: P4-D3 M-Tile=%0d/49 origin=(%0d,%0d), N=0..127, %0d runs verified, pad=%0d bytes/tile",
                                 ty*SPATIAL_TX+tx+1,oy0,ox0,complete_tile_count,pad_bytes_this_tile);
                    repeat(3) @(negedge clk);
                end
        if(complete_tile_count!=TOTAL_TILES ||
           accepted_starts!=TOTAL_TILES || completed_runs!=TOTAL_TILES ||
           total_reads!=GLOBAL_WORDS || total_writes!=TOTAL_TILES*M ||
           total_feeds!=TOTAL_TILES*EXPECT_FEED ||
           total_compute_cycles!=TOTAL_TILES*EXPECT_CYCLES ||
           total_act_load_bytes!=TOTAL_TILES*ACT_BYTES ||
           total_weight_load_words!=TOTAL_TILES*WEIGHT_WORDS ||
           total_padding_bytes!=128000)
            $fatal(1,"FAIL: P4-D3 full-layer totals: tiles=%0d results=%0d Kpairs=%0d cycles=%0d act_bytes=%0d weights=%0d padding=%0d",
                   complete_tile_count,total_reads,total_feeds,total_compute_cycles,
                   total_act_load_bytes,total_weight_load_words,total_padding_bytes);
        check_global_assembly();
        finished=1;
        $display("PASS: P4-D3 complete 3x3/S2/P1 layer2.0.conv1: 784/784 Tiles, 100352/100352 INT32, 3612672/3612672 K-pairs, %0d compute cycles, pad=%0d bytes",
                 total_compute_cycles,total_padding_bytes);
        $display("INFO: simulation-only 10ns TB clock, not implemented/DDR measured latency");
        $finish;
    end
    initial begin : watchdog
        repeat(12000000) @(posedge clk);
        if(!finished) $fatal(1,"TIMEOUT: P4-D3 784-Tile full-layer test");
    end
endmodule
