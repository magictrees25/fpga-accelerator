`timescale 1ns / 1ps

// P5-A mixed-mode regression: SIX consecutive runs, ONE shared compute array,
// ONE reset only.  Modes switch 3x3/S1 -> 1x1/S2 -> 3x3/S2 -> repeat.
// Full local patches and weights are reloaded for every run while idle.
// Independent Golden uses GLOBAL coordinates, not DUT addresses or local
// preload image.  Output, K-pair payload, writeback, and inactive reader
// requests are checked.  10-ns clock is TB-only (not implementation timing).
module tb_p5_multimode_6runs;
    localparam int RUNS=6, C=64, N=8;
    logic clk=1'b0;
    always #5 clk=~clk;
    logic rst_n,start,fmap_wr_en,weight_wr_en,out_rd_en;
    logic [1:0] cfg_mode;
    logic [15:0] fmap_wr_addr,weight_wr_addr,out_rd_addr;
    logic [7:0] fmap_wr_data;
    logic [63:0] weight_wr_data;
    logic [2:0] out_rd_lane_sel;
    logic signed [31:0] out_rd_data;
    logic out_rd_lane_valid,out_rd_valid,busy,done,any_overflow;

    fpga_system_top_p5_multimode dut (.*);

    logic signed [31:0] golden[0:63][0:7];
    int epoch, run_mode, origin_y, origin_x;
    int H, W, OH, OW, M, K, stride, padding;
    int act_load_count, weight_load_count, pad_bytes;
    int feed_count, req_count, write_count, clear_count, drain_count;
    int launch_count, kdone_count, read_count;
    int next_m,next_k,cycle_count,expected_cycles;
    int starts_seen, dones_seen, tick, start_tick, done_tick;
    bit computing, done_q, finished;
    longint signed golden_checksum;

    function automatic int activation_pattern(input int gy,gx,c,e);
        int v;
        begin
            v=((17*gy+11*gx+7*c+37*e)%255)-127;
            if (gy==0 && gx==0 && c==0) v=-128;
            if (gy==54 && gx==54 && c==63) v=127;
            return v;
        end
    endfunction

    function automatic int weight_pattern(input int ky,kx,c,n,e);
        int v;
        begin
            v=((13*ky+7*kx+3*c+5*n+41*e)%31)-15;
            if (ky==0 && kx==0 && c==0 && n==0) v=-128;
            if (c==63 && n==7 && (ky==2 || ky==0) && (kx==2 || kx==0)) v=127;
            return v;
        end
    endfunction

    function automatic int activation_at(input int gy,gx,c,e);
        if(gy<0 || gy>=56 || gx<0 || gx>=56) return 0;
        return activation_pattern(gy,gx,c,e);
    endfunction

    task automatic setup_run(input int e);
        int ky,kx,gy,gx,sum;
        begin
            run_mode=e%3;
            case(run_mode)
                0: begin H=10;W=10;OH=8;OW=8;K=576;stride=1;padding=1; end
                1: begin H=8; W=8; OH=4;OW=4;K=64; stride=2;padding=0; end
                2: begin H=9; W=9; OH=4;OW=4;K=576;stride=2;padding=1; end
                default: $fatal(1,"unexpected test mode");
            endcase
            M=OH*OW;
            golden_checksum=0;
            origin_y = (e<3) ? 0 : ((run_mode==0) ? 48 : 24);
            origin_x = (e<3) ? 0 : ((run_mode==0) ? 48 : 24);
            expected_cycles=M*(1+K/2+13+1+1+1);

            // Pure independent GLOBAL convolution; no local buffer lookup.
            for (int oy=0;oy<OH;oy++)
                for (int ox=0;ox<OW;ox++)
                    for (int n=0;n<N;n++) begin
                        sum=0;
                        for (int ky_i=0;ky_i<((K==64)?1:3);ky_i++)
                            for (int kx_i=0;kx_i<((K==64)?1:3);kx_i++)
                                for (int c=0;c<C;c++) begin
                                    gy=(origin_y+oy)*stride+ky_i-padding;
                                    gx=(origin_x+ox)*stride+kx_i-padding;
                                    sum += activation_at(gy,gx,c,e) *
                                           weight_pattern(ky_i,kx_i,c,n,e);
                                end
                        golden[oy*OW+ox][n]=sum;
                        golden_checksum+=sum;
                    end
        end
    endtask

    task automatic preload_run;
        int gy,gx,ky,kx,c,word;
        begin
            // Independent Python reference checks for each of the six cases.
            case(epoch)
                0: if(golden_checksum != -64'sd588897) $fatal(1,"P5: bad golden anchor 0");
                1: if(golden_checksum != -64'sd58390) $fatal(1,"P5: bad golden anchor 1");
                2: if(golden_checksum != -64'sd169341) $fatal(1,"P5: bad golden anchor 2");
                3: if(golden_checksum !=  64'sd1194144) $fatal(1,"P5: bad golden anchor 3");
                4: if(golden_checksum !=  64'sd129283) $fatal(1,"P5: bad golden anchor 4");
                5: if(golden_checksum != -64'sd57523) $fatal(1,"P5: bad golden anchor 5");
            endcase
            // cfg_mode is intentionally set BEFORE every preload and remains
            // stable through start/done, so only the targeted RAM is filled.
            @(negedge clk);
            cfg_mode=2'(run_mode);
            if(busy || done) $fatal(1,"P5: not idle before preload e=%0d",epoch);
            act_load_count=0;
            weight_load_count=0;
            pad_bytes=0;
            for(int y=0;y<H;y++)
                for(int x=0;x<W;x++)
                    for(int ci=0;ci<C;ci++) begin
                        gy=origin_y*stride+y-padding;
                        gx=origin_x*stride+x-padding;
                        @(negedge clk);
                        fmap_wr_en=1;
                        fmap_wr_addr=16'((y*W+x)*C+ci);
                        fmap_wr_data=8'(activation_at(gy,gx,ci,epoch));
                        act_load_count++;
                        if(gy<0 || gy>=56 || gx<0 || gx>=56) pad_bytes++;
                    end
            @(negedge clk);
            fmap_wr_en=0;
            for(int kk=0;kk<K;kk++) begin
                ky=(K==64)?0:(kk/64)/3;
                kx=(K==64)?0:(kk/64)%3;
                c=kk%64;
                @(negedge clk);
                weight_wr_en=1;
                weight_wr_addr=16'(kk);
                for(int nn=0;nn<N;nn++)
                    weight_wr_data[nn*8 +:8]=8'(weight_pattern(ky,kx,c,nn,epoch));
                weight_load_count++;
            end
            @(negedge clk);
            weight_wr_en=0;
            if(act_load_count!=H*W*C || weight_load_count!=K)
                $fatal(1,"P5: preload count mismatch e=%0d",epoch);
        end
    endtask

    // Scoreboard samples the data ACTUALLY consumed on the current edge.
    always @(posedge clk) begin : scoreboard
        bit was_computing;
        int em,ek,ky,kx,ci,gy,gx,want_a,want_w,got_a,got_w;
        if(!rst_n) begin
            tick=0; computing=0; done_q=0;
        end else begin
            tick++;
            was_computing=computing;
            if(start) begin
                if(computing || busy || done || starts_seen!=epoch || dones_seen!=epoch)
                    $fatal(1,"P5: unexpected start e=%0d",epoch);
                starts_seen++;
                computing=1;
                start_tick=tick;
                cycle_count=0;
            end
            if(was_computing) cycle_count++;
            if($isunknown({busy,done,any_overflow}) || any_overflow!==0)
                $fatal(1,"P5: unknown or overflow e=%0d",epoch);
            if(int'(was_computing)+int'(fmap_wr_en)+
               int'(weight_wr_en)+int'(out_rd_en)>1)
                $fatal(1,"P5: host / compute overlap");

            if(dut.u_compute.clear_acc) clear_count++;
            if(dut.u_compute.u_ctrl.k_start) launch_count++;
            if(dut.u_compute.u_ctrl.k_done) kdone_count++;
            if(dut.u_compute.u_ctrl.array_enable &&
               !dut.u_compute.clear_acc && !dut.u_compute.feed_valid)
                drain_count++;

            if(dut.u_compute.pair_req_en) begin
                if(!computing || dut.u_compute.active_mode !== 2'(run_mode))
                    $fatal(1,"P5: request wrong mode e=%0d",epoch);
                case(run_mode)
                    0: if(dut.u_compute.req_s1!==1'b1 ||
                          dut.u_compute.req_1x1!==1'b0 ||
                          dut.u_compute.req_s2!==1'b0 ||
                          dut.u_compute.wreq_3x3!==1'b1)
                          $fatal(1,"P5: S1 routing error");
                    1: if(dut.u_compute.req_s1!==1'b0 ||
                          dut.u_compute.req_1x1!==1'b1 ||
                          dut.u_compute.req_s2!==1'b0 ||
                          dut.u_compute.wreq_1x1!==1'b1)
                          $fatal(1,"P5: 1x1 routing error");
                    2: if(dut.u_compute.req_s1!==1'b0 ||
                          dut.u_compute.req_1x1!==1'b0 ||
                          dut.u_compute.req_s2!==1'b1 ||
                          dut.u_compute.wreq_3x3!==1'b1)
                          $fatal(1,"P5: S2 routing error");
                endcase
                // No unselected Reader may drive a physical RAM request.
                if(dut.u_compute.u_act_s1.lb_rd_en !== dut.u_compute.req_s1 ||
                   dut.u_compute.u_act_1x1.bank_rd_en !== {2{dut.u_compute.req_1x1}} ||
                   dut.u_compute.u_act_s2.bank_rd_en !== {2{dut.u_compute.req_s2}} ||
                   dut.u_compute.u_weight_shared.bank_rd_en !==
                   {2{dut.u_compute.pair_req_en}})
                    $fatal(1,"P5: unselected RAM read enable e=%0d",epoch);
                req_count++;
            end
            if(dut.u_compute.feed_valid) begin
                em=next_m;ek=next_k;
                if(!computing || em>=M || ek>=K ||
                   dut.u_compute.m_addr!==16'(em) ||
                   dut.u_compute.k_base!==16'(ek) ||
                   dut.u_compute.act_rsp_valid!==2'b11 ||
                   dut.u_compute.wgt_rsp_valid!==2'b11 ||
                   dut.u_compute.n_base!==0)
                    $fatal(1,"P5: bad FEED indices/valid e=%0d M=%0d K=%0d",epoch,em,ek);
                for(int p=0;p<2;p++) begin
                    ci=(ek+p)%64;
                    ky=(K==64)?0:((ek+p)/64)/3;
                    kx=(K==64)?0:((ek+p)/64)%3;
                    gy=(origin_y+em/OW)*stride+ky-padding;
                    gx=(origin_x+em%OW)*stride+kx-padding;
                    want_a=activation_at(gy,gx,ci,epoch);
                    got_a=int'($signed(dut.u_compute.act_pair_packed[p*8+:8]));
                    if(got_a!==want_a)
                        $fatal(1,"P5: activation e=%0d mode=%0d M=%0d K=%0d p=%0d got=%0d want=%0d",
                               epoch,run_mode,em,ek,p,got_a,want_a);
                    for(int n=0;n<N;n++) begin
                        want_w=weight_pattern(ky,kx,ci,n,epoch);
                        got_w=int'($signed(dut.u_compute.wgt_pair_packed[(p*N+n)*8+:8]));
                        if(got_w!==want_w)
                            $fatal(1,"P5: weight e=%0d mode=%0d K=%0d n=%0d got=%0d want=%0d",
                                   epoch,run_mode,ek+p,n,got_w,want_w);
                    end
                end
                feed_count++;
                if(ek==K-2) begin next_k=0;next_m++; end
                else next_k=ek+2;
            end
            if(dut.write_valid) begin
                em=write_count;
                if(!computing || em>=M ||
                   dut.write_addr!==16'(em) || dut.write_m_addr!==16'(em) ||
                   dut.write_n_base!==16'd0 || dut.write_mask!==8'hff)
                    $fatal(1,"P5: wrong write metadata epoch=%0d M=%0d",epoch,em);
                for(int n=0;n<N;n++)
                    if(int'($signed(dut.write_data[n*32+:32]))!==golden[em][n])
                        $fatal(1,"P5: output write epoch=%0d mode=%0d M=%0d N=%0d got=%0d want=%0d",
                               epoch,run_mode,em,n,$signed(dut.write_data[n*32+:32]),golden[em][n]);
                write_count++;
            end
            #1;
            if(done && done_q) $fatal(1,"P5: multi-cycle done");
            if(done) begin
                if(!computing || write_count!=M ||
                   feed_count!=M*(K/2) || req_count!=M*(K/2) ||
                   clear_count!=M || drain_count!=M*13 ||
                   launch_count!=M || kdone_count!=M ||
                   next_m!=M || next_k!=0)
                    $fatal(1,"P5: incomplete run counters e=%0d",epoch);
                computing=0;
                done_tick=tick;
                dones_seen++;
                if(done_tick-start_tick!=cycle_count ||
                   cycle_count!=expected_cycles)
                    $fatal(1,"P5: cycle count e=%0d got=%0d want=%0d",
                           epoch,cycle_count,expected_cycles);
            end
            done_q=done;
        end
    end

    task automatic check_outputs;
        int got;
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
                    if(out_rd_valid!==1'b1 || out_rd_lane_valid!==1'b1 ||
                       got!==golden[m][n])
                        $fatal(1,"P5: readback e=%0d mode=%0d M=%0d N=%0d got=%0d want=%0d",
                               epoch,run_mode,m,n,got,golden[m][n]);
                    read_count++;
                end
            @(negedge clk);
            out_rd_en=0;
            @(posedge clk);#2;
            if(out_rd_valid!==1'b0)
                $fatal(1,"P5: output read valid stuck");
        end
    endtask

    initial begin : stimulus
        int waited,total_readbacks;
        rst_n=0; start=0;cfg_mode=0;
        fmap_wr_en=0;weight_wr_en=0;out_rd_en=0;
        fmap_wr_addr=0;fmap_wr_data=0;
        weight_wr_addr=0;weight_wr_data=0;
        out_rd_addr=0;out_rd_lane_sel=0;
        epoch=0;run_mode=0;origin_y=0;origin_x=0;
        H=0;W=0;OH=0;OW=0;M=0;K=0;stride=0;padding=0;
        starts_seen=0;dones_seen=0;tick=0;start_tick=0;done_tick=0;
        computing=0;done_q=0;finished=0;total_readbacks=0;golden_checksum=0;
        repeat(4) @(negedge clk);
        rst_n=1;
        for(int e=0;e<RUNS;e++) begin
            epoch=e;
            req_count=0;feed_count=0;write_count=0;clear_count=0;
            drain_count=0;launch_count=0;kdone_count=0;read_count=0;
            next_m=0;next_k=0;cycle_count=0;
            setup_run(e);
            preload_run();
            @(negedge clk);
            if(busy || done || computing)
                $fatal(1,"P5: compute not idle e=%0d",e);
            start=1;
            @(negedge clk);
            start=0;
            waited=0;
            while(done!==1'b1 && waited<30000) begin
                @(posedge clk);#2;
                waited++;
            end
            if(done!==1'b1)
                $fatal(1,"P5: timeout e=%0d mode=%0d",e,run_mode);
            check_outputs();
            if(read_count!=M*N || write_count!=M ||
               feed_count!=M*K/2 || dones_seen!=e+1)
                $fatal(1,"P5: final per-run count mismatch e=%0d",e);
            total_readbacks+=read_count;
            $display("PASS: P5-A run=%0d mode=%0d origin=(%0d,%0d) %0d/%0d INT32, %0d/%0d K-pairs, %0d cycles, pad=%0d bytes",
                     e,run_mode,origin_y,origin_x,read_count,M*N,
                     feed_count,M*K/2,cycle_count,pad_bytes);
            $display("INFO: P5-A mode=%0d golden checksum=%0d",run_mode,golden_checksum);
            repeat(4) @(negedge clk);
        end
        if(starts_seen!=RUNS || dones_seen!=RUNS || total_readbacks!=1536)
            $fatal(1,"P5: lifetime counters");
        finished=1;
        $display("PASS: P5-A three convolution modes, 6/6 consecutive runs, 1536/1536 INT32 outputs, ONE shared PE array, NO reset between modes");
        $finish;
    end
    initial begin : watchdog
        repeat(200000) @(posedge clk);
        if(!finished) $fatal(1,"TIMEOUT: P5-A 6-run regression");
    end
endmodule
