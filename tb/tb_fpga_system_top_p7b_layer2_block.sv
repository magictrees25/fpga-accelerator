`timescale 1ns / 1ps
// Full pretrained layer2.0. Golden = full tensors from independent Python.
// Shortcut and conv1 actual readbacks, NEVER their golden substitutes, feed
// the final conv2/add phase. One DUT, one reset, 3 x 784 Host-scheduled tiles.
module tb_fpga_system_top_p7b_layer2_block;
    localparam int INPUTS=56*56*64,OUTPUTS=28*28*128,JOBS=3*784;
    logic clk=0;
    always #5 clk=~clk;
    logic rst_n,start;
    logic [1:0] cfg_mode;
    logic [255:0] channel_bias,channel_multiplier;
    logic load_req,load_c_tile,load_done;
    logic [3:0] load_m_addr;
    logic fmap_wr_en,weight_wr_en,skip_wr_en,out_rd_en;
    logic [15:0] fmap_wr_addr,weight_wr_addr,out_rd_addr;
    logic [7:0] fmap_wr_data;
    logic [63:0] weight_wr_data;
    logic [3:0] skip_wr_addr;
    logic [255:0] skip_wr_data;
    logic [2:0] out_rd_lane_sel;
    logic signed [31:0] out_rd_data;
    logic out_rd_valid,out_rd_lane_valid,busy,done,any_overflow;
    fpga_system_top_p7b_downsample_tile dut(.*);

    logic signed [7:0] input_q[0:INPUTS-1];
    logic signed [7:0] weight0[0:8191],weight1[0:73727],weight2[0:147455];
    logic signed [31:0] bias0[0:127],bias1[0:127],bias2[0:127];
    logic signed [31:0] mult0[0:127],mult1[0:127],mult2[0:127];
    logic signed [31:0] acc0[0:OUTPUTS-1],acc1[0:OUTPUTS-1],acc2[0:OUTPUTS-1],prefix2[0:OUTPUTS-1];
    logic signed [31:0] golden0[0:OUTPUTS-1],golden1[0:OUTPUTS-1],golden2[0:OUTPUTS-1];
    logic signed [31:0] shortcut_actual[0:OUTPUTS-1];
    logic signed [7:0] mid_actual[0:OUTPUTS-1],final_actual[0:OUTPUTS-1];
    bit seen0[0:OUTPUTS-1],seen1[0:OUTPUTS-1],seen2[0:OUTPUTS-1];
    int mode,oy,ox,nb,job,k_length,chunks;
    int segments,seg_req,seg_feed,seg_drain,raw_count,post_count,read_count,acks;
    int a_slot,w_slot,starts,dones,total_segments,total_pairs,total_raw,total_post,total_reads,total_acks;
    int pipe_clears,acc_clears,total_a_loads,total_w_loads,pad_loads,pad_elements;
    int negative_shortcuts,changed_cache,phase_reads[0:2];
    int signed pe_ref[0:7][0:7],held_pe[0:7][0:7],previous_cache[0:127];
    bit active,load_q,done_q,previous_valid;
    string data_dir;

    function automatic bit exists(input string path);
        int fd;
        begin fd=$fopen(path,"r");exists=(fd!=0);if(fd!=0)$fclose(fd);end
    endfunction
    task automatic require_file(input string name);
        if(!exists({data_dir,"/",name}))$fatal(1,"P7-B missing %s/%s",data_dir,name);
    endtask
    function automatic int idx_out(input int m,n);
        idx_out=((oy+m/4)*28+ox+m%4)*128+nb+n;
    endfunction
    function automatic bit is_pad(input int m,k);
        int y,x,h;
        begin
            h=(mode==2)?28:56;
            y=oy+m/4;x=ox+m%4;
            if(mode==0)begin y=2*y;x=2*x;end
            else begin y=y*((mode==1)?2:1)+k/192-1;x=x*((mode==1)?2:1)+(k/64)%3-1;end
            is_pad=(y<0 || x<0 || y>=h || x>=h);
        end
    endfunction
    function automatic logic signed [7:0] activation_at(input int m,ct,k);
        int y,x,c,idx;
        begin
            y=oy+m/4;x=ox+m%4;c=ct*64+k%64;
            if(mode==0)begin y=2*y;x=2*x;end
            else begin y=y*((mode==1)?2:1)+k/192-1;x=x*((mode==1)?2:1)+(k/64)%3-1;end
            if(is_pad(m,k)) activation_at=0;
            else if(mode!=2) activation_at=input_q[(y*56+x)*64+c];
            else begin
                idx=(y*28+x)*128+c;
                if(!seen1[idx])$fatal(1,"P7-B conv2 used unwritten conv1 activation");
                activation_at=mid_actual[idx];
            end
        end
    endfunction
    function automatic logic signed [7:0] weight_at(input int ct,k,n);
        if(mode==0)weight_at=weight0[k*128+nb+n];
        else if(mode==1)weight_at=weight1[k*128+nb+n];
        else weight_at=weight2[((k/64)*128+ct*64+k%64)*128+nb+n];
    endfunction
    function automatic logic signed [31:0] want_acc(input int idx);
        case(mode)0:want_acc=acc0[idx];1:want_acc=acc1[idx];default:want_acc=acc2[idx];endcase
    endfunction
    function automatic logic signed [31:0] want_post(input int idx);
        case(mode)0:want_post=golden0[idx];1:want_post=golden1[idx];default:want_post=golden2[idx];endcase
    endfunction

    always @(posedge clk) if(rst_n)begin : monitor
        int m,ct,k,idx;
        logic signed [31:0] expected;
        if($isunknown({busy,done,any_overflow}) || any_overflow!==0)$fatal(1,"P7-B status/overflow");
        if(start)begin
            if(!active || busy || done)$fatal(1,"P7-B illegal start");
            starts++;
        end
        if(done)begin
            if(done_q || !active || raw_count!=16 || post_count!=16 || segments!=16*chunks)
                $fatal(1,"P7-B early/repeated done");
            dones++;
        end
        done_q=done;
        if(load_req && !load_q)begin
            if(!active || load_m_addr!==4'(segments/chunks) || load_c_tile!==1'(segments%chunks))
                $fatal(1,"P7-B M/C request order");
            a_slot=0;w_slot=0;
            for(int r=0;r<8;r++)for(int n=0;n<8;n++)held_pe[r][n]=dut.u_array.pe_acc[r][n];
        end
        load_q=load_req;
        if(fmap_wr_en)begin
            if(!load_req || fmap_wr_addr!==16'(a_slot) ||
               fmap_wr_data!==activation_at(int'(load_m_addr),int'(load_c_tile),a_slot))
                $fatal(1,"P7-B activation preload address/data");
            if(is_pad(int'(load_m_addr),a_slot))pad_loads++;
            a_slot++;total_a_loads++;
        end
        if(weight_wr_en)begin
            if(!load_req || weight_wr_addr!==16'(w_slot))$fatal(1,"P7-B weight preload address");
            for(int n=0;n<8;n++)if(weight_wr_data[n*8+:8]!==weight_at(int'(load_c_tile),w_slot,n))
                $fatal(1,"P7-B weight packing");
            w_slot++;total_w_loads++;
        end
        if(load_done)begin
            if(!load_req || a_slot!=k_length || w_slot!=k_length)$fatal(1,"P7-B early load ack");
            for(int r=0;r<8;r++)for(int n=0;n<8;n++)
                if(dut.u_array.pe_acc[r][n]!==held_pe[r][n])$fatal(1,"P7-B PE sum changed during Host wait");
            acks++;total_acks++;
        end
        if(dut.clear_pipe)begin
            if(dut.clear_acc!==!dut.c_idx)$fatal(1,"P7-B wrong C-tile clear policy");
            pipe_clears++;
        end
        if(dut.clear_acc)begin
            for(int r=0;r<8;r++)for(int n=0;n<8;n++)pe_ref[r][n]=0;
            acc_clears++;
        end
        m=segments/chunks;ct=segments%chunks;
        if(dut.pair_req_en)begin
            k=seg_req*2;
            if(!active || m>=16 || seg_req>=k_length/2 || dut.m_idx!==4'(m) || dut.c_idx!==1'(ct) ||
               dut.fetch_k_base!==16'(k) || dut.u_activation.rd_addr!==16'(k/2) ||
               dut.u_weight.bank_rd_addr[0]!==16'(k/2) || dut.u_weight.bank_rd_addr[1]!==16'(k/2))
                $fatal(1,"P7-B local K/bank request order");
            seg_req++;
        end
        if(dut.feed_valid)begin
            k=seg_feed*2;
            if(!active || seg_feed>=k_length/2 || dut.k_base!==16'(k) || dut.act_rsp_valid!==2'b11 || dut.wgt_rsp_valid!==2'b11)
                $fatal(1,"P7-B FEED metadata");
            for(int p=0;p<2;p++)begin
                if(dut.act_pair_packed[p*8+:8]!==activation_at(m,ct,k+p))$fatal(1,"P7-B activation pair mismatch");
                if(is_pad(m,k+p))pad_elements++;
                for(int n=0;n<8;n++)begin
                    if(dut.wgt_pair_packed[(p*8+n)*8+:8]!==weight_at(ct,k+p,n))$fatal(1,"P7-B weight pair mismatch");
                    pe_ref[(k+p)%8][n]+=int'(activation_at(m,ct,k+p))*int'(weight_at(ct,k+p,n));
                end
            end
            seg_feed++;total_pairs++;
        end
        if(dut.array_enable && !dut.feed_valid && !dut.clear_pipe)begin
            if(dut.act_in_packed!==0 || dut.wgt_pair_in_packed!==0)$fatal(1,"P7-B nonzero DRAIN");
            seg_drain++;
        end
        if(dut.k_done)begin
            if(seg_req!=k_length/2 || seg_feed!=k_length/2 || seg_drain!=13 || dut.raw_valid)
                $fatal(1,"P7-B K segment counts/partial write");
            for(int n=0;n<8;n++)begin
                idx=idx_out(m,n);
                expected=(mode==2 && ct==0)?prefix2[idx]:want_acc(idx);
                if(dut.k_result_packed[n*32+:32]!==expected)$fatal(1,"P7-B cumulative K result mode=%0d M=%0d C=%0d N=%0d",mode,m,ct,n);
                for(int r=0;r<8;r++)if(dut.u_array.pe_acc[r][n]!==pe_ref[r][n])$fatal(1,"P7-B cumulative PE sums");
            end
            segments++;total_segments++;seg_req=0;seg_feed=0;seg_drain=0;
        end
        if(dut.raw_valid)begin
            if(raw_count>=16 || dut.m_idx!==4'(raw_count) || segments!=(raw_count+1)*chunks)
                $fatal(1,"P7-B raw output order/final-only write");
            for(int n=0;n<8;n++)if(dut.k_result_packed[n*32+:32]!==want_acc(idx_out(raw_count,n)))
                $fatal(1,"P7-B independent INT32 golden mismatch mode=%0d job=%0d M=%0d N=%0d",mode,job,raw_count,n);
            raw_count++;total_raw++;
        end
        if(dut.post_valid[0])begin
            if(post_count>=16 || dut.m_idx!==4'(post_count) || dut.post_valid!==8'hff || dut.u_output.wr_mask!==8'hff)
                $fatal(1,"P7-B quantized write metadata");
            for(int n=0;n<8;n++)if(dut.post_data[n*32+:32]!==want_post(idx_out(post_count,n)))
                $fatal(1,"P7-B post/add mismatch mode=%0d job=%0d M=%0d N=%0d got=%0d want=%0d",
                       mode,job,post_count,n,$signed(dut.post_data[n*32+:32]),want_post(idx_out(post_count,n)));
            post_count++;total_post++;
        end
    end
    // Check the clear edge itself, including preservation of all 64 PE sums.
    for(genvar r=0;r<8;r++)begin : check_row
        for(genvar n=0;n<8;n++)begin : check_col
            always @(posedge clk) if(rst_n && dut.clear_pipe)begin : check_clear
                integer signed before_clear;
                before_clear=dut.clear_acc?0:dut.u_array.pe_acc[r][n];
                #1;
                if(dut.u_array.pe_acc[r][n]!==before_clear)$fatal(1,"P7-B clear_acc/clear_pipe preservation");
            end
        end
    end

    task automatic load_segment(input int m,ct);
        int waited;
        begin
            waited=0;
            while(load_req!==1 && waited<2000)begin @(posedge clk);#2;waited++;end
            if(load_req!==1 || load_m_addr!==4'(m) || load_c_tile!==1'(ct))$fatal(1,"P7-B load request timeout/order");
            repeat(2+(m+ct)%5)@(negedge clk);
            for(int k=0;k<k_length;k++)begin
                @(negedge clk);fmap_wr_en=1;fmap_wr_addr=16'(k);fmap_wr_data=activation_at(m,ct,k);
            end
            @(negedge clk);fmap_wr_en=0;
            for(int k=0;k<k_length;k++)begin
                @(negedge clk);weight_wr_en=1;weight_wr_addr=16'(k);
                for(int n=0;n<8;n++)weight_wr_data[n*8+:8]=weight_at(ct,k,n);
            end
            @(negedge clk);weight_wr_en=0;
            repeat(2+m%3)@(negedge clk);
            load_done=1;
            @(negedge clk);load_done=0;
            if(load_req!==0)$fatal(1,"P7-B load ack not consumed");
        end
    endtask
    task automatic read_stitch;
        int m,n,idx;
        begin
            for(int mi=0;mi<16;mi++)for(int ni=0;ni<8;ni++)begin
                m=(mi*5+3)%16;n=(ni*3+1)%8;
                @(negedge clk);out_rd_en=1;out_rd_addr=16'(m);out_rd_lane_sel=3'(n);
                @(posedge clk);#2;
                idx=idx_out(m,n);
                if(out_rd_valid!==1 || out_rd_lane_valid!==1 || out_rd_data!==want_post(idx))
                    $fatal(1,"P7-B sync read mismatch mode=%0d idx=%0d got=%0d want=%0d",mode,idx,out_rd_data,want_post(idx));
                case(mode)
                    0:begin
                        if(seen0[idx])$fatal(1,"P7-B duplicate shortcut scatter");
                        shortcut_actual[idx]=out_rd_data;seen0[idx]=1;
                        if(out_rd_data<0)negative_shortcuts++;
                    end
                    1:begin
                        if(seen1[idx] || out_rd_data<0 || out_rd_data>127)$fatal(1,"P7-B conv1 scatter/range");
                        mid_actual[idx]=out_rd_data[7:0];seen1[idx]=1;
                    end
                    2:begin
                        if(seen2[idx] || out_rd_data<0 || out_rd_data>127)$fatal(1,"P7-B final scatter/range");
                        final_actual[idx]=out_rd_data[7:0];seen2[idx]=1;
                    end
                endcase
                if(previous_valid && out_rd_data!==previous_cache[m*8+n])changed_cache++;
                previous_cache[m*8+n]=out_rd_data;
                read_count++;total_reads++;phase_reads[mode]++;
            end
            @(negedge clk);out_rd_en=0;
            @(posedge clk);#2;
            if(out_rd_valid!==0)$fatal(1,"P7-B read valid stuck");
            previous_valid=1;
        end
    endtask

    initial begin : stimulus
        int waited,idx;
        rst_n=0;start=0;cfg_mode=0;channel_bias=0;channel_multiplier=0;
        load_done=0;fmap_wr_en=0;weight_wr_en=0;skip_wr_en=0;out_rd_en=0;
        fmap_wr_addr=0;fmap_wr_data=0;weight_wr_addr=0;weight_wr_data=0;
        skip_wr_addr=0;skip_wr_data=0;out_rd_addr=0;out_rd_lane_sel=0;
        mode=0;oy=0;ox=0;nb=0;job=0;k_length=64;chunks=1;
        segments=0;seg_req=0;seg_feed=0;seg_drain=0;raw_count=0;post_count=0;read_count=0;acks=0;
        a_slot=0;w_slot=0;starts=0;dones=0;total_segments=0;total_pairs=0;total_raw=0;total_post=0;total_reads=0;total_acks=0;
        pipe_clears=0;acc_clears=0;total_a_loads=0;total_w_loads=0;pad_loads=0;pad_elements=0;
        negative_shortcuts=0;changed_cache=0;active=0;load_q=0;done_q=0;previous_valid=0;
        for(int i=0;i<3;i++)phase_reads[i]=0;
        if(!$value$plusargs("P7B_DATA_DIR=%s",data_dir))begin
            data_dir="../../../../../reference/p7b_layer2_0";
            if(!exists({data_dir,"/input.mem"}))data_dir="reference/p7b_layer2_0";
        end
        require_file("input.mem");require_file("prefix2.mem");
        for(int i=0;i<3;i++)begin
            require_file($sformatf("weight%0d.mem",i));require_file($sformatf("bias%0d.mem",i));
            require_file($sformatf("mult%0d.mem",i));require_file($sformatf("acc%0d.mem",i));require_file($sformatf("output%0d.mem",i));
        end
        $readmemh({data_dir,"/input.mem"},input_q);$readmemh({data_dir,"/prefix2.mem"},prefix2);
        $readmemh({data_dir,"/weight0.mem"},weight0);$readmemh({data_dir,"/weight1.mem"},weight1);$readmemh({data_dir,"/weight2.mem"},weight2);
        $readmemh({data_dir,"/bias0.mem"},bias0);$readmemh({data_dir,"/bias1.mem"},bias1);$readmemh({data_dir,"/bias2.mem"},bias2);
        $readmemh({data_dir,"/mult0.mem"},mult0);$readmemh({data_dir,"/mult1.mem"},mult1);$readmemh({data_dir,"/mult2.mem"},mult2);
        $readmemh({data_dir,"/acc0.mem"},acc0);$readmemh({data_dir,"/acc1.mem"},acc1);$readmemh({data_dir,"/acc2.mem"},acc2);
        $readmemh({data_dir,"/output0.mem"},golden0);$readmemh({data_dir,"/output1.mem"},golden1);$readmemh({data_dir,"/output2.mem"},golden2);
        for(int i=0;i<INPUTS;i++)if($isunknown(input_q[i]) || input_q[i]<0)$fatal(1,"P7-B incomplete/invalid input");
        for(int i=0;i<8192;i++)if($isunknown(weight0[i]))$fatal(1,"P7-B missing W0");
        for(int i=0;i<73728;i++)if($isunknown(weight1[i]))$fatal(1,"P7-B missing W1");
        for(int i=0;i<147456;i++)if($isunknown(weight2[i]))$fatal(1,"P7-B missing W2");
        for(int i=0;i<128;i++)if($isunknown({bias0[i],bias1[i],bias2[i],mult0[i],mult1[i],mult2[i]}) ||
            mult0[i]<=0 || mult1[i]<=0 || mult2[i]<=0)$fatal(1,"P7-B invalid quantization params");
        for(int i=0;i<OUTPUTS;i++)begin
            seen0[i]=0;seen1[i]=0;seen2[i]=0;
            if($isunknown({acc0[i],acc1[i],acc2[i],prefix2[i],golden0[i],golden1[i],golden2[i]}))$fatal(1,"P7-B incomplete Golden");
        end
        repeat(4)@(negedge clk);rst_n=1;
        $display("INFO: P7-B real layer2.0, input 56x56x64 -> output 28x28x128, data=%s",data_dir);
        for(mode=0;mode<3;mode++)begin
            k_length=(mode==0)?64:576;chunks=(mode==2)?2:1;
            for(int ty=0;ty<7;ty++)for(int tx=0;tx<7;tx++)for(int nt=0;nt<16;nt++)begin
                @(negedge clk);
                if(busy || done || load_req)$fatal(1,"P7-B tile started before idle");
                oy=ty*4;ox=tx*4;nb=nt*8;
                segments=0;seg_req=0;seg_feed=0;seg_drain=0;raw_count=0;post_count=0;read_count=0;acks=0;
                cfg_mode=2'(mode);
                for(int n=0;n<8;n++)begin
                    case(mode)
                        0:begin channel_bias[n*32+:32]=bias0[nb+n];channel_multiplier[n*32+:32]=mult0[nb+n];end
                        1:begin channel_bias[n*32+:32]=bias1[nb+n];channel_multiplier[n*32+:32]=mult1[nb+n];end
                        2:begin channel_bias[n*32+:32]=bias2[nb+n];channel_multiplier[n*32+:32]=mult2[nb+n];end
                    endcase
                end
                for(int m=0;m<16;m++)begin
                    @(negedge clk);skip_wr_en=1;skip_wr_addr=4'(m);
                    for(int n=0;n<8;n++)begin
                        idx=idx_out(m,n);
                        if(mode==2 && !seen0[idx])$fatal(1,"P7-B missing actual shortcut");
                        skip_wr_data[n*32+:32]=(mode==2)?shortcut_actual[idx]:32'd0;
                    end
                end
                @(negedge clk);skip_wr_en=0;
                @(negedge clk);active=1;start=1;
                @(negedge clk);start=0;cfg_mode=2'd3;channel_bias='x;channel_multiplier='x;
                for(int m=0;m<16;m++)for(int ct=0;ct<chunks;ct++)load_segment(m,ct);
                waited=0;
                while(done!==1 && waited<2000)begin @(posedge clk);#2;waited++;end
                if(done!==1)$fatal(1,"P7-B tile completion timeout job=%0d",job);
                read_stitch();
                if(segments!=16*chunks || acks!=16*chunks || raw_count!=16 || post_count!=16 || read_count!=128 || dones!=job+1)
                    $fatal(1,"P7-B tile counters job=%0d",job);
                @(negedge clk);active=0;
                $display("TILE PASS: P7-B mode=%0d id=%0d origin=(%0d,%0d) OC=%0d..%0d, %0d chunks, 128/128 INT32 and post/readbacks",
                         mode,job,oy,ox,nb,nb+7,segments);
                job++;
            end
            for(int i=0;i<OUTPUTS;i++)begin
                if(mode==0 && (!seen0[i] || shortcut_actual[i]!==golden0[i]))$fatal(1,"P7-B full shortcut scatter");
                if(mode==1 && (!seen1[i] || int'(mid_actual[i])!==golden1[i]))$fatal(1,"P7-B full conv1 scatter");
                if(mode==2 && (!seen2[i] || int'(final_actual[i])!==golden2[i]))$fatal(1,"P7-B final full scatter");
            end
            if(phase_reads[mode]!=OUTPUTS)$fatal(1,"P7-B phase coverage");
            $display("PHASE PASS: P7-B mode=%0d, 784 Tiles, 100352/100352 outputs and readbacks",mode);
        end
        if(job!=JOBS || starts!=JOBS || dones!=JOBS || total_segments!=50176 || total_acks!=50176 ||
           total_pairs!=11239424 || total_raw!=37632 || total_post!=37632 || total_reads!=301056 ||
           pipe_clears!=50176 || acc_clears!=37632 || total_a_loads!=22478848 || total_w_loads!=22478848 ||
           negative_shortcuts==0 || changed_cache==0 || pad_loads!=850944 || pad_elements!=pad_loads)
            $fatal(1,"P7-B block totals/coverage mismatch");
        $display("INFO: one DUT/reset, 2352 starts/dones, 50176 load acks/K segments, 11239424 pair FEED, 37632 raw + 37632 post writes");
        $display("INFO: signed shortcut negatives=%0d, changed cached lanes=%0d, padding scalar loads/FEED=%0d/%0d",negative_shortcuts,changed_cache,pad_loads,pad_elements);
        $display("PASS: P7-B REAL RESNET18 layer2.0 DOWNSAMPLE RESIDUAL BLOCK, 2352 Tiles, 301056/301056 INT32 accumulators, 100352/100352 final INT8 outputs");
        $display("INFO: conv2 used DUT conv1 and signed shortcut readbacks; Host window staging, no full-network/board/accuracy claim");
        $finish;
    end
    initial begin #2000000000;$fatal(1,"P7-B global timeout");end
endmodule
