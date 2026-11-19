`timescale 1ns / 1ps

// Real pretrained layer1.0: full conv1 -> RTL requant/ReLU -> full conv2 ->
// RTL requant/residual/ReLU. The second convolution MUST consume mid_actual,
// which is populated exclusively by synchronous DUT output RAM reads.
// Python full-tensor golden files are used only for comparisons.
module tb_fpga_system_top_p7_layer1_block;
    localparam int ELEMENTS=56*56*64, WEIGHTS=3*3*64*64;
    localparam int JOBS=2*7*7*8, PAIRS_PER_JOB=64*576/2;
    logic clk=0;
    always #5 clk=~clk;
    logic rst_n,start,residual_en;
    logic [255:0] channel_bias,channel_multiplier;
    logic fmap_wr_en,weight_wr_en,skip_wr_en,out_rd_en;
    logic [15:0] fmap_wr_addr,weight_wr_addr,out_rd_addr;
    logic [7:0] fmap_wr_data;
    logic [63:0] weight_wr_data,skip_wr_data;
    logic [5:0] skip_wr_addr;
    logic [2:0] out_rd_lane_sel;
    logic signed [7:0] out_rd_data;
    logic out_rd_valid,out_rd_lane_valid,busy,done,any_overflow;
    fpga_system_top_p7_residual_tile dut (.*);

    logic signed [7:0] input_q[0:ELEMENTS-1];
    logic signed [7:0] weight1[0:WEIGHTS-1],weight2[0:WEIGHTS-1];
    logic signed [31:0] bias1[0:63],bias2[0:63],mult1[0:63],mult2[0:63];
    logic signed [31:0] acc1[0:ELEMENTS-1],acc2[0:ELEMENTS-1];
    logic signed [7:0] golden1[0:ELEMENTS-1],golden2[0:ELEMENTS-1];
    logic signed [7:0] mid_actual[0:ELEMENTS-1],final_actual[0:ELEMENTS-1];
    bit seen1[0:ELEMENTS-1],seen2[0:ELEMENTS-1];
    logic [63:0] previous_cache[0:63];
    bit previous_cache_valid;
    int layer,origin_y,origin_x,channel_base,job;
    int req_count,feed_count,raw_count,post_count,read_count,clear_count;
    int starts_seen,dones_seen,total_pairs,total_raw,total_post,total_reads;
    int activation_loads,weight_loads,skip_loads,changed_cache_lanes;
    int padding_loads,padding_k_elements,weight_switches;
    bit computing,done_q;
    string data_dir;

    function automatic bit exists(input string path);
        int fd;
        begin
            fd=$fopen(path,"r"); exists=(fd!=0);
            if(fd!=0) $fclose(fd);
        end
    endfunction
    task automatic require_file(input string name);
        if(!exists({data_dir,"/",name})) $fatal(1,"P7 missing file: %s/%s",data_dir,name);
    endtask
    function automatic int index_hwc(input int y,x,c);
        index_hwc=(y*56+x)*64+c;
    endfunction
    function automatic logic signed [7:0] source_at(input int y,x,c);
        int idx;
        begin
            if(y<0 || y>=56 || x<0 || x>=56) source_at=0;
            else begin
                idx=index_hwc(y,x,c);
                if(layer==0) source_at=input_q[idx];
                else begin
                    if(!seen1[idx]) $fatal(1,"P7: conv2 reads unwritten conv1 output %0d",idx);
                    source_at=mid_actual[idx];
                end
            end
        end
    endfunction
    function automatic logic signed [7:0] weight_at(input int k,n);
        if(layer==0) weight_at=weight1[k*64+n];
        else weight_at=weight2[k*64+n];
    endfunction
    function automatic logic signed [31:0] expected_acc(input int idx);
        if(layer==0) expected_acc=acc1[idx]; else expected_acc=acc2[idx];
    endfunction
    function automatic logic signed [7:0] expected_output(input int idx);
        if(layer==0) expected_output=golden1[idx]; else expected_output=golden2[idx];
    endfunction

    task automatic load_job;
        int gy,gx;
        begin
            @(negedge clk);
            if(busy || done) $fatal(1,"P7: preload before idle");
            residual_en=(layer==1);
            for(int lane=0;lane<8;lane++) begin
                channel_bias[lane*32+:32]=(layer==0)?bias1[channel_base+lane]:bias2[channel_base+lane];
                channel_multiplier[lane*32+:32]=(layer==0)?mult1[channel_base+lane]:mult2[channel_base+lane];
            end
            for(int y=0;y<10;y++)
                for(int x=0;x<10;x++)
                    for(int c=0;c<64;c++) begin
                        gy=origin_y+y-1; gx=origin_x+x-1;
                        @(negedge clk);
                        fmap_wr_en=1; fmap_wr_addr=16'((y*10+x)*64+c);
                        fmap_wr_data=source_at(gy,gx,c);
                        if(gy<0 || gy>=56 || gx<0 || gx>=56) padding_loads++;
                    end
            @(negedge clk); fmap_wr_en=0;
            for(int k=0;k<576;k++) begin
                @(negedge clk); weight_wr_en=1; weight_wr_addr=16'(k);
                for(int lane=0;lane<8;lane++) weight_wr_data[lane*8+:8]=weight_at(k,channel_base+lane);
            end
            @(negedge clk); weight_wr_en=0;
            // Original block input, never conv1's output, is the identity skip.
            for(int m=0;m<64;m++) begin
                @(negedge clk); skip_wr_en=1; skip_wr_addr=6'(m);
                for(int lane=0;lane<8;lane++)
                    skip_wr_data[lane*8+:8]=input_q[index_hwc(origin_y+m/8,origin_x+m%8,channel_base+lane)];
            end
            @(negedge clk); skip_wr_en=0;
        end
    endtask

    // Sample values at the same rising edge as the actual hardware consumers.
    always @(posedge clk) if(rst_n) begin : monitor
        int m,k,ky,kx,c,gy,gx,idx;
        if($isunknown({busy,done,any_overflow}) || any_overflow!==0)
            $fatal(1,"P7: unknown status or INT32 overflow");
        if(fmap_wr_en) activation_loads++;
        if(weight_wr_en) weight_loads++;
        if(skip_wr_en) skip_loads++;
        if(start) begin
            if(busy || done || !computing) $fatal(1,"P7 illegal start");
            starts_seen++;
        end
        if(done && done_q) $fatal(1,"P7: done longer than one cycle");
        if(done) begin
            if(!computing || raw_count!=64 || post_count!=64)
                $fatal(1,"P7: early/unsolicited done");
            dones_seen++;
        end
        done_q=done;
        if(dut.u_compute.clear_acc) clear_count++;
        if(dut.u_compute.pair_req_en) begin
            m=req_count/288; k=(req_count%288)*2;
            if(!computing || req_count>=PAIRS_PER_JOB ||
               dut.u_compute.m_addr!==16'(m) || dut.u_compute.fetch_k_base!==16'(k) ||
               dut.u_compute.u_weight_shared.bank_rd_en!==2'b11 ||
               dut.u_compute.u_weight_shared.bank_rd_addr[0]!==16'(k/2) ||
               dut.u_compute.u_weight_shared.bank_rd_addr[1]!==16'(k/2))
                $fatal(1,"P7: K request/bank address mismatch job=%0d pair=%0d",job,req_count);
            req_count++;
        end
        if(dut.u_compute.feed_valid) begin
            m=feed_count/288; k=(feed_count%288)*2;
            if(!computing || feed_count>=PAIRS_PER_JOB ||
               dut.u_compute.m_addr!==16'(m) || dut.u_compute.k_base!==16'(k) ||
               dut.u_compute.act_rsp_valid!==2'b11 || dut.u_compute.wgt_rsp_valid!==2'b11)
                $fatal(1,"P7: FEED metadata mismatch job=%0d pair=%0d",job,feed_count);
            for(int p=0;p<2;p++) begin
                ky=((k+p)/64)/3; kx=((k+p)/64)%3; c=(k+p)%64;
                gy=origin_y+m/8+ky-1; gx=origin_x+m%8+kx-1;
                if(dut.u_compute.act_pair_packed[p*8+:8]!==source_at(gy,gx,c))
                    $fatal(1,"P7: FEED activation mismatch job=%0d M=%0d K=%0d",job,m,k+p);
                if(gy<0 || gy>=56 || gx<0 || gx>=56) padding_k_elements++;
                for(int lane=0;lane<8;lane++)
                    if(dut.u_compute.wgt_pair_packed[(p*8+lane)*8+:8]!==weight_at(k+p,channel_base+lane))
                        $fatal(1,"P7: weight switch/alignment mismatch job=%0d K=%0d lane=%0d",job,k+p,lane);
            end
            feed_count++; total_pairs++;
        end
        if(dut.raw_valid) begin
            m=raw_count;
            if(!computing || m>=64 || dut.raw_addr!==16'(m) || dut.raw_mask!==8'hff)
                $fatal(1,"P7 raw write address/mask mismatch");
            for(int lane=0;lane<8;lane++) begin
                idx=index_hwc(origin_y+m/8,origin_x+m%8,channel_base+lane);
                if(dut.raw_data[lane*32+:32]!==expected_acc(idx))
                    $fatal(1,"P7 INT32 mismatch layer=%0d y=%0d x=%0d c=%0d got=%0d want=%0d",
                           layer+1,origin_y+m/8,origin_x+m%8,channel_base+lane,
                           $signed(dut.raw_data[lane*32+:32]),expected_acc(idx));
            end
            raw_count++; total_raw++;
        end
        if(dut.quant_valid[0]) begin
            m=post_count;
            if(!computing || m>=64 || dut.addr_q2!==16'(m) || dut.mask_q2!==8'hff || dut.quant_valid!==8'hff)
                $fatal(1,"P7 postprocess address/mask/valid mismatch");
            for(int lane=0;lane<8;lane++) begin
                idx=index_hwc(origin_y+m/8,origin_x+m%8,channel_base+lane);
                if(dut.quant_data[lane*8+:8]!==expected_output(idx))
                    $fatal(1,"P7 postprocess mismatch layer=%0d global_idx=%0d got=%0d want=%0d",
                           layer+1,idx,$signed(dut.quant_data[lane*8+:8]),expected_output(idx));
                if(previous_cache_valid && previous_cache[m][lane*8+:8]!==dut.quant_data[lane*8+:8])
                    changed_cache_lanes++;
            end
            post_count++; total_post++;
        end
    end

    task automatic read_and_stitch;
        int idx;
        begin
            for(int m=0;m<64;m++)
                for(int lane=0;lane<8;lane++) begin
                    @(negedge clk);
                    out_rd_en=1; out_rd_addr=16'(m); out_rd_lane_sel=3'(lane);
                    @(posedge clk); #2;
                    idx=index_hwc(origin_y+m/8,origin_x+m%8,channel_base+lane);
                    if(out_rd_valid!==1 || out_rd_lane_valid!==1 || out_rd_data!==expected_output(idx))
                        $fatal(1,"P7 synchronous readback mismatch layer=%0d idx=%0d got=%0d want=%0d",
                               layer+1,idx,out_rd_data,expected_output(idx));
                    if(layer==0) begin
                        if(seen1[idx]) $fatal(1,"P7 duplicate conv1 scatter idx=%0d",idx);
                        mid_actual[idx]=out_rd_data; seen1[idx]=1;
                    end else begin
                        if(seen2[idx]) $fatal(1,"P7 duplicate final scatter idx=%0d",idx);
                        final_actual[idx]=out_rd_data; seen2[idx]=1;
                    end
                    previous_cache[m][lane*8+:8]=out_rd_data;
                    read_count++; total_reads++;
                end
            @(negedge clk); out_rd_en=0;
            @(posedge clk); #2;
            if(out_rd_valid!==0) $fatal(1,"P7 output read valid stuck");
            previous_cache_valid=1;
        end
    endtask

    initial begin : stimulus
        int waited;
        rst_n=0; start=0; residual_en=0; channel_bias=0; channel_multiplier=0;
        fmap_wr_en=0; weight_wr_en=0; skip_wr_en=0; out_rd_en=0;
        fmap_wr_addr=0; fmap_wr_data=0; weight_wr_addr=0; weight_wr_data=0;
        skip_wr_addr=0; skip_wr_data=0; out_rd_addr=0; out_rd_lane_sel=0;
        layer=0; origin_y=0; origin_x=0; channel_base=0; job=0;
        starts_seen=0; dones_seen=0; total_pairs=0; total_raw=0; total_post=0; total_reads=0;
        req_count=0; feed_count=0; raw_count=0; post_count=0; read_count=0; clear_count=0;
        activation_loads=0; weight_loads=0; skip_loads=0; changed_cache_lanes=0;
        padding_loads=0; padding_k_elements=0; weight_switches=0;
        computing=0; done_q=0; previous_cache_valid=0;
        if(!$value$plusargs("P7_DATA_DIR=%s",data_dir)) begin
            data_dir="reference/p7a_layer1_0";
            if(!exists({data_dir,"/input.mem"})) data_dir="../../../../../reference/p7a_layer1_0";
        end
        require_file("input.mem");
        require_file("weight1.mem"); require_file("weight2.mem");
        require_file("bias1.mem"); require_file("bias2.mem");
        require_file("mult1.mem"); require_file("mult2.mem");
        require_file("acc1.mem"); require_file("acc2.mem");
        require_file("output1.mem"); require_file("output2.mem");
        $readmemh({data_dir,"/input.mem"},input_q);
        $readmemh({data_dir,"/weight1.mem"},weight1); $readmemh({data_dir,"/weight2.mem"},weight2);
        $readmemh({data_dir,"/bias1.mem"},bias1); $readmemh({data_dir,"/bias2.mem"},bias2);
        $readmemh({data_dir,"/mult1.mem"},mult1); $readmemh({data_dir,"/mult2.mem"},mult2);
        $readmemh({data_dir,"/acc1.mem"},acc1); $readmemh({data_dir,"/acc2.mem"},acc2);
        $readmemh({data_dir,"/output1.mem"},golden1); $readmemh({data_dir,"/output2.mem"},golden2);
        for(int i=0;i<ELEMENTS;i++) begin
            seen1[i]=0; seen2[i]=0;
            if($isunknown({input_q[i],acc1[i],acc2[i],golden1[i],golden2[i]}) ||
               input_q[i]<0 || golden1[i]<0 || golden2[i]<0)
                $fatal(1,"P7 missing/invalid activation or golden data i=%0d",i);
        end
        for(int i=0;i<WEIGHTS;i++)
            if($isunknown({weight1[i],weight2[i]})) $fatal(1,"P7 missing weight i=%0d",i);
        for(int i=0;i<64;i++)
            if($isunknown({bias1[i],bias2[i],mult1[i],mult2[i]}) || mult1[i]<=0 || mult2[i]<=0)
                $fatal(1,"P7 missing/invalid channel params i=%0d",i);
        $display("INFO: P7-A pretrained ResNet18 layer1.0, full 56x56x64, data=%s",data_dir);
        repeat(4) @(negedge clk);
        rst_n=1;
        for(layer=0;layer<2;layer++) begin
            for(int ty=0;ty<7;ty++)
                for(int tx=0;tx<7;tx++)
                    for(int nt=0;nt<8;nt++) begin
                        @(negedge clk);
                        origin_y=ty*8; origin_x=tx*8; channel_base=nt*8;
                        req_count=0; feed_count=0; raw_count=0; post_count=0; read_count=0; clear_count=0;
                        load_job();
                        if(job>0) weight_switches++;
                        @(negedge clk); computing=1; start=1;
                        @(negedge clk); start=0;
                        // Prove parameters were latched at accepted start.
                        channel_bias='x; channel_multiplier='x; residual_en=~residual_en;
                        waited=0;
                        while(done!==1 && waited<40000) begin @(posedge clk); #2; waited++; end
                        if(done!==1) $fatal(1,"P7 tile timeout job=%0d layer=%0d",job,layer+1);
                        read_and_stitch();
                        if(req_count!=PAIRS_PER_JOB || feed_count!=PAIRS_PER_JOB || clear_count!=64 ||
                           raw_count!=64 || post_count!=64 || read_count!=512 || dones_seen!=job+1)
                            $fatal(1,"P7 tile counters job=%0d req/feed=%0d/%0d clear=%0d writes=%0d/%0d reads=%0d done=%0d",
                                   job,req_count,feed_count,clear_count,raw_count,post_count,read_count,dones_seen);
                        @(negedge clk); computing=0;
                        $display("TILE PASS: P7-A conv%0d id=%0d origin=(%0d,%0d) OC=%0d..%0d, 512 INT32 + 512 INT8/readbacks correct",
                                 layer+1,job,origin_y,origin_x,channel_base,channel_base+7);
                        job++;
                    end
            for(int i=0;i<ELEMENTS;i++) begin
                if(layer==0 && (!seen1[i] || mid_actual[i]!==golden1[i]))
                    $fatal(1,"P7 conv1 full scatter mismatch idx=%0d",i);
                if(layer==1 && (!seen2[i] || final_actual[i]!==golden2[i]))
                    $fatal(1,"P7 final full scatter mismatch idx=%0d",i);
            end
            $display("LAYER PASS: P7-A conv%0d 392 Tiles, 200704/200704 INT32 and INT8 outputs",layer+1);
        end
        if(job!=JOBS || starts_seen!=JOBS || dones_seen!=JOBS || total_pairs!=JOBS*PAIRS_PER_JOB ||
           total_raw!=JOBS*64 || total_post!=JOBS*64 || total_reads!=2*ELEMENTS ||
           activation_loads!=JOBS*6400 || weight_loads!=JOBS*576 || skip_loads!=JOBS*64 ||
           weight_switches!=JOBS-1 || changed_cache_lanes==0 ||
           padding_loads!=282624 || padding_k_elements!=684032)
            $fatal(1,"P7 full block coverage/counter mismatch");
        $display("INFO: one DUT/reset, %0d starts/dones, %0d pairs, %0d raw + %0d quant writes, %0d synchronous lane reads",
                 starts_seen,total_pairs,total_raw,total_post,total_reads);
        $display("INFO: %0d weight reload switches, %0d changed cached lanes, padding loads=%0d K-elements=%0d",
                 weight_switches,changed_cache_lanes,padding_loads,padding_k_elements);
        $display("PASS: P7-A REAL RESNET18 layer1.0 RESIDUAL BLOCK, 784 Tiles, 401408/401408 INT32 accumulators, 200704/200704 final INT8 outputs");
        $display("INFO: conv2 used DUT conv1 readbacks; RTL performed BN-folded bias, Q24 requantization, identity add and ReLU. TB scheduling; no board/accuracy claim.");
        $finish;
    end
    initial begin #1000000000; $fatal(1,"P7 full block global timeout"); end
endmodule
