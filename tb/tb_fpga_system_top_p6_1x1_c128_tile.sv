`timescale 1ns / 1ps

// P6-C1: layer3.0.downsample.0 local Tile, true Cin=128, 1x1/S2/P0.
// Full reference input is 28x28x128, full kernel 1x1x128x256.
// Two separate 4x4x8 Tiles, one DUT/reset, two 64-channel loads per M.
// No internal force/model responses: real P4 Reader + shared BRAM + P2 PE.
module tb_fpga_system_top_p6_1x1_c128_tile;
  logic clk=0;
  always #5 clk=~clk;
  logic rst_n,start,load_req,load_c_tile,load_done;
  logic [3:0] load_m_addr;
  logic fmap_wr_en,weight_wr_en,out_rd_en;
  logic [15:0] fmap_wr_addr,weight_wr_addr,out_rd_addr;
  logic [7:0] fmap_wr_data;
  logic [63:0] weight_wr_data;
  logic [2:0] out_rd_lane_sel;
  logic signed [31:0] out_rd_data;
  logic out_rd_lane_valid,out_rd_valid,busy,done,any_overflow;
  fpga_system_top_p6_1x1_c128_tile dut (.*);

  logic signed [7:0] image_data [0:27][0:27][0:127];
  logic signed [7:0] kernel [0:127][0:255];
  integer signed golden [0:15][0:7];
  integer signed prefix [0:15][0:1][0:7];
  integer signed pe_prefix [0:15][0:1][0:7][0:7];
  integer signed pe_running [0:7][0:7];
  integer signed previous_result [0:15][0:7];
  logic [255:0] captured [0:15];
  bit seen [0:15],read_seen [0:15][0:7];
  integer job_id,origin_y,origin_x,out_channel_base;
  bit active,job_inflight,segment_inflight;
  logic prior_load,prior_req;
  integer completed_segments,load_requests,load_acks,k_starts,writes,outputs,readbacks;
  integer requests,feeds,drains,pipe_clears,acc_clears,host_dones;
  integer slot_a_loads,slot_w_loads,act_loads,weight_loads;
  integer seg_requests,seg_feeds,seg_drains,seg_clears,seg_firsts;
  integer act_responses,weight_responses,changed_readbacks,expected_changed;
  integer total_segments,total_loads,total_acks,total_requests,total_feeds,total_drains;
  integer total_writes,total_outputs,total_readbacks,total_act_loads,total_weight_loads,total_dones;
  logic [255:0] last_snapshot;

  function automatic integer pixel_byte_base(input integer m);
    pixel_byte_base=((2*(m/4))*8+2*(m%4))*64;
  endfunction
  function automatic integer signed a_value(input integer m,input integer global_c);
    a_value=int'($signed(image_data[2*(origin_y+m/4)][2*(origin_x+m%4)][global_c]));
  endfunction
  function automatic integer signed w_value(input integer global_c,input integer n);
    w_value=int'($signed(kernel[global_c][out_channel_base+n]));
  endfunction
  function automatic integer signed feature_pattern(input integer job,input integer y,input integer x,input integer c);
    if(y==0 && x==0 && c==0) feature_pattern=-128;
    else feature_pattern=((17*y+11*x+7*c+37*job+19)%256)-128;
  endfunction
  function automatic integer signed weight_pattern(input integer job,input integer c,input integer n);
    if(c==0 && n==0) weight_pattern=-128;
    else if(c==127 && n==255) weight_pattern=127;
    else weight_pattern=((3*c+5*n+53*job+41)%251)-125;
  endfunction

  // Observations, not controls. Reader response signals are RTL-driven.
  always @(posedge clk or negedge rst_n)
    if(!rst_n) begin prior_load<=0;prior_req<=0;end
    else begin prior_load<=load_req;prior_req<=dut.pair_req_en;end

  always @(posedge clk) if(rst_n) begin : monitor
    integer em,ec,local_k,gc;
    logic [63:0] want_weight_word,want_act;
    logic [127:0] want_weights;
    if($isunknown({busy,done,load_req,load_m_addr,load_c_tile,dut.k_start,dut.k_done,
                    dut.pair_req_en,dut.feed_valid,dut.clear_pipe,dut.clear_acc,dut.write_valid}) ||
       any_overflow !== 1'b0)
      $fatal(1,"FAIL: unknown control or PE overflow");
    if(dut.act_rsp_valid !== {2{prior_req}} || dut.wgt_rsp_valid !== {2{prior_req}} ||
       dut.fmap_rd_valid !== prior_req || dut.fmap_rd_en !== dut.pair_req_en ||
       dut.u_weight_reader.bank_rd_en !== {2{dut.pair_req_en}})
      $fatal(1,"FAIL: real memory response latency/enable");
    if(!active) begin
      if(busy || done || load_req || dut.pair_req_en || dut.write_valid)
        $fatal(1,"FAIL: activity outside a launched job");
    end else begin
      em=completed_segments/2; ec=completed_segments%2;
      if(job_inflight && !done && busy !== 1'b1)
        $fatal(1,"FAIL: busy dropped during load or computation");
      if(dut.act_rsp_valid==2'b11) act_responses++;
      if(dut.wgt_rsp_valid==2'b11) weight_responses++;
      if(load_req) begin
        job_inflight=1;
        if(em>=16 || load_m_addr !== 4'(em) || load_c_tile !== 1'(ec) ||
           !busy || dut.array_enable || dut.pair_req_en || dut.k_start || dut.k_done || dut.write_valid)
          $fatal(1,"FAIL: load request metadata/quiescence M=%0d Ctile=%0d",em,ec);
        if(!prior_load) begin
          load_requests++; total_loads++;
          slot_a_loads=0;slot_w_loads=0;
        end
      end
      if(fmap_wr_en) begin
        if(!load_req || slot_a_loads>=64 ||
           fmap_wr_addr !== 16'(pixel_byte_base(em)+slot_a_loads) ||
           fmap_wr_data !== 8'(a_value(em,64*ec+slot_a_loads)) ||
           dut.u_activation_shared.bank_wr_addr !== 16'((pixel_byte_base(em)+slot_a_loads)/2) ||
           dut.u_activation_shared.bank_wr_en !== ((slot_a_loads%2)==0 ? 2'b01 : 2'b10))
          $fatal(1,"FAIL: C-Tile activation preload mapping/data/bank");
        slot_a_loads++;act_loads++;total_act_loads++;
      end
      if(weight_wr_en) begin
        for(int n=0;n<8;n++) want_weight_word[n*8 +: 8]=8'(w_value(64*ec+slot_w_loads,n));
        if(!load_req || slot_w_loads>=64 || weight_wr_addr !== 16'(slot_w_loads) ||
           weight_wr_data !== want_weight_word ||
           dut.u_weight_reader.bank_wr_addr !== 16'(slot_w_loads/2) ||
           dut.u_weight_reader.bank_wr_en !== ((slot_w_loads%2)==0 ? 2'b01 : 2'b10))
          $fatal(1,"FAIL: C-Tile weight preload mapping/data/bank");
        slot_w_loads++;weight_loads++;total_weight_loads++;
      end
      if(load_done) begin
        if(!load_req || fmap_wr_en || weight_wr_en || slot_a_loads!=64 || slot_w_loads!=64 ||
           load_acks!=completed_segments)
          $fatal(1,"FAIL: premature/duplicate load acknowledgement");
        load_acks++;total_acks++;
      end
      if(dut.k_start) begin
        if(segment_inflight || em>=16 || load_req || load_acks!=completed_segments+1 ||
           load_requests!=load_acks || dut.k_busy || dut.k_done ||
           dut.m_idx !== 4'(em) || dut.c_tile_idx !== 1'(ec) ||
           dut.u_kctrl.k_len !== 16'd64 || dut.u_kctrl.clear_acc_on_start !== (ec==0))
          $fatal(1,"FAIL: compute started before Host ready or wrong chunk metadata");
        segment_inflight=1;k_starts++;
      end
      if(dut.clear_pipe) begin
        if(!segment_inflight || seg_clears!=0 || seg_requests!=0 || seg_feeds!=0 ||
           dut.clear_acc !== (ec==0) || dut.valid_reset !== 1'b1 ||
           dut.k_base !== 16'd0 || !dut.array_enable || !dut.pair_req_en)
          $fatal(1,"FAIL: first/continuing C-Tile clear controls");
        seg_clears++;pipe_clears++;
      end
      if(dut.clear_acc) begin
        if(!dut.clear_pipe) $fatal(1,"FAIL: clear_acc outside clear_pipe");
        acc_clears++;
      end
      if(dut.pair_req_en) begin
        if(!segment_inflight || seg_requests>=32 ||
           !(dut.clear_pipe || dut.feed_valid) || dut.fetch_k_base !== 16'(2*seg_requests) ||
           dut.m_addr !== 16'(em) ||
           dut.fmap_rd_addr !== 16'((pixel_byte_base(em)+2*seg_requests)/2) ||
           dut.u_weight_reader.bank_rd_addr[0] !== 16'(seg_requests) ||
           dut.u_weight_reader.bank_rd_addr[1] !== 16'(seg_requests))
          $fatal(1,"FAIL: local BRAM request after channel reload M=%0d Ctile=%0d",em,ec);
        seg_requests++;requests++;total_requests++;
      end
      if(dut.feed_valid) begin
        local_k=2*seg_feeds;gc=64*ec+local_k;
        if(!segment_inflight || seg_feeds>=32 || seg_clears!=1 || dut.k_base !== 16'(local_k) ||
           dut.u_act_reader.rsp_k_base !== 16'(local_k) || dut.u_act_reader.rsp_m_addr !== 16'(em) ||
           dut.act_rsp_valid !== 2'b11 || dut.wgt_rsp_valid !== 2'b11 ||
           dut.pair_req_en !== (local_k<62) || dut.data_valid_in !== (local_k==0))
          $fatal(1,"FAIL: C-Tile FEED tags/phase/valid");
        want_act=0;
        want_act[(local_k%8)*8 +: 8]=8'(a_value(em,gc));
        want_act[((local_k+1)%8)*8 +: 8]=8'(a_value(em,gc+1));
        for(int n=0;n<8;n++) begin
          want_weights[n*8 +: 8]=8'(w_value(gc,n));
          want_weights[(8+n)*8 +: 8]=8'(w_value(gc+1,n));
        end
        if(dut.act_pair_packed !== {8'(a_value(em,gc+1)),8'(a_value(em,gc))} ||
           dut.wgt_pair_packed !== want_weights || dut.act_in_packed !== want_act ||
           dut.wgt_pair_in_packed !== want_weights)
          $fatal(1,"FAIL: global Cin mapping or stale C-Tile data M=%0d Ctile=%0d localK=%0d",em,ec,local_k);
        if(dut.data_valid_in) seg_firsts++;
        seg_feeds++;feeds++;total_feeds++;
      end
      if(dut.array_enable && !dut.feed_valid && !dut.clear_pipe) begin
        if(!segment_inflight || seg_feeds!=32 || dut.pair_req_en || dut.data_valid_in ||
           dut.act_in_packed !== 64'd0 || dut.wgt_pair_in_packed !== 128'd0)
          $fatal(1,"FAIL: DRAIN protocol");
        seg_drains++;drains++;total_drains++;
      end
      if(dut.k_done) begin
        if(!segment_inflight || seg_requests!=32 || seg_feeds!=32 || seg_drains!=13 ||
           seg_clears!=1 || seg_firsts!=1 || dut.write_valid)
          $fatal(1,"FAIL: C-Tile completion counts or partial write");
        for(int n=0;n<8;n++) begin
          if($signed(dut.k_result_packed[n*32 +: 32]) !== prefix[em][ec][n])
            $fatal(1,"FAIL: cumulative chunk result M=%0d Ctile=%0d N=%0d",em,ec,n);
          for(int r=0;r<8;r++)
            if(dut.u_array.pe_acc[r][n] !== pe_prefix[em][ec][r][n])
              $fatal(1,"FAIL: cumulative PE partial sum");
        end
        last_snapshot=dut.k_result_packed;
        completed_segments++;total_segments++;
        segment_inflight=0;
        seg_requests=0;seg_feeds=0;seg_drains=0;seg_clears=0;seg_firsts=0;
      end
      if(dut.write_valid) begin
        if(segment_inflight || completed_segments!=2*(writes+1) || writes>=16 ||
           dut.write_addr !== 16'(writes) || dut.write_mask !== 8'hff ||
           dut.write_data !== last_snapshot || !busy || load_req || dut.pair_req_en)
          $fatal(1,"FAIL: final-only writeback ordering/metadata");
        if(seen[writes]) $fatal(1,"FAIL: duplicate final write");
        seen[writes]=1;captured[writes]=dut.write_data;
        for(int n=0;n<8;n++) begin
          if($signed(dut.write_data[n*32 +: 32]) !== golden[writes][n])
            $fatal(1,"FAIL: full Cin128 Golden job=%0d M=%0d N=%0d",job_id,writes,n);
          outputs++;total_outputs++;
        end
        writes++;total_writes++;
      end
      if(done) begin
        if(!job_inflight || writes!=16 || completed_segments!=32 || busy || load_req || dut.write_valid)
          $fatal(1,"FAIL: done before final result or repeated done");
        host_dones++;total_dones++;job_inflight=0;
      end
    end
  end

  // Preserve all 64 PE sums throughout variable-length Host load waits.
  for(genvar r=0;r<8;r++) begin : check_row
    for(genvar n=0;n<8;n++) begin : check_col
      integer signed held_sum;
      always @(posedge clk) begin : check_wait_and_clear
        integer signed want;
        if(rst_n && active && load_req) begin
          if(!prior_load) held_sum=dut.u_array.pe_acc[r][n];
          else if(dut.u_array.pe_acc[r][n] !== held_sum)
            $fatal(1,"FAIL: PE sum changed during Host load wait");
          if(load_c_tile && dut.u_array.pe_acc[r][n] !== pe_prefix[int'(load_m_addr)][0][r][n])
            $fatal(1,"FAIL: C0 sum lost before/during C1 preload");
        end
        if(rst_n && active && dut.clear_pipe) begin
          want=dut.c_tile_idx ? pe_prefix[int'(dut.m_idx)][0][r][n] : 0;
          #1;
          if(dut.u_array.pe_acc[r][n] !== want ||
             dut.u_array.gen_row[r].gen_col[n].u_pe.act_reg !== 8'sd0 ||
             dut.u_array.gen_row[r].gen_col[n].u_pe.wgt_reg !== 8'sd0 ||
             dut.u_array.gen_row[r].gen_col[n].u_pe.mul_reg !== 16'sd0)
            $fatal(1,"FAIL: pipeline clear/accumulator preservation after reload");
        end
      end
    end
  end

  task automatic serve_load(input integer m,input integer ct);
    integer waited;
    begin
      waited=0;
      while(load_req !== 1'b1 && waited<300) begin @(posedge clk);#2;waited++;end
      if(load_req !== 1'b1 || load_m_addr !== 4'(m) || load_c_tile !== 1'(ct))
        $fatal(1,"FAIL: Host timed out waiting for requested M/C chunk");
      // Variable ready delay, while PE partial sums must remain frozen.
      repeat(2+(m+ct)%5) @(negedge clk);
      for(int c=0;c<64;c++) begin
        @(negedge clk);
        fmap_wr_en=1;fmap_wr_addr=16'(pixel_byte_base(m)+c);
        fmap_wr_data=8'(a_value(m,ct*64+c));
      end
      @(negedge clk);fmap_wr_en=0;
      for(int c=0;c<64;c++) begin
        @(negedge clk);
        weight_wr_en=1;weight_wr_addr=16'(c);
        for(int n=0;n<8;n++)weight_wr_data[n*8 +: 8]=8'(w_value(ct*64+c,n));
      end
      @(negedge clk);weight_wr_en=0;
      repeat(3+(m%3)) @(negedge clk);
      if(!load_req || slot_a_loads!=64 || slot_w_loads!=64)
        $fatal(1,"FAIL: load request ended before all data were ready");
      load_done=1;
      @(negedge clk);load_done=0;
      if(load_req !== 1'b0) $fatal(1,"FAIL: load acknowledgement was not consumed");
    end
  endtask

  task automatic read_results;
    integer m,n;
    integer signed got;
    begin
      for(int mi=0;mi<16;mi++) begin
        m=(mi*5+3)%16;
        for(int ni=0;ni<8;ni++) begin
          n=(ni*3+1)%8;
          @(negedge clk);out_rd_en=1;out_rd_addr=16'(m);out_rd_lane_sel=3'(n);
          @(posedge clk);#1;
          got=int'($signed(out_rd_data));
          if(out_rd_valid !== 1'b1 || out_rd_lane_valid !== 1'b1 ||
             got !== golden[m][n] || out_rd_data !== $signed(captured[m][n*32 +: 32]) || read_seen[m][n])
            $fatal(1,"FAIL: synchronous 32-bit lane readback M=%0d N=%0d got=%0d expected=%0d",m,n,got,golden[m][n]);
          read_seen[m][n]=1;
          if(job_id>0 && golden[m][n]!=previous_result[m][n]) begin
            if(got===previous_result[m][n])$fatal(1,"FAIL: stale output from previous job");
            changed_readbacks++;
          end
          previous_result[m][n]=got;
          readbacks++;total_readbacks++;
        end
      end
      @(negedge clk);out_rd_en=0;
      @(posedge clk);#1;
      if(out_rd_valid !== 1'b0 || out_rd_lane_valid !== 1'b0 ||
         readbacks!=128 || changed_readbacks!=expected_changed)
        $fatal(1,"FAIL: readback valid/coverage/overwrite counts");
    end
  endtask

  task automatic run_job(input integer id,input integer oy0,input integer ox0,input integer n0);
    integer m,waited;
    integer signed a,w,sum;
    begin
      @(negedge clk);
      if(busy || done || load_req)$fatal(1,"FAIL: job started before IDLE");
      job_id=id;origin_y=oy0;origin_x=ox0;out_channel_base=n0;
      if(oy0<0 || oy0+4>14 || ox0<0 || ox0+4>14 || n0<0 || n0+8>256)
        $fatal(1,"FAIL: reference Tile coordinates outside target layer");
      completed_segments=0;load_requests=0;load_acks=0;k_starts=0;writes=0;outputs=0;readbacks=0;
      requests=0;feeds=0;drains=0;pipe_clears=0;acc_clears=0;host_dones=0;
      slot_a_loads=0;slot_w_loads=0;act_loads=0;weight_loads=0;
      seg_requests=0;seg_feeds=0;seg_drains=0;seg_clears=0;seg_firsts=0;
      act_responses=0;weight_responses=0;expected_changed=0;changed_readbacks=0;
      segment_inflight=0;job_inflight=0;last_snapshot=0;
      for(int y=0;y<28;y++)for(int x=0;x<28;x++)for(int c=0;c<128;c++)
        image_data[y][x][c]=8'(feature_pattern(id,y,x,c));
      for(int c=0;c<128;c++)for(int n=0;n<256;n++)kernel[c][n]=8'(weight_pattern(id,c,n));
      // Full-Cin standard 1x1 stride-2 convolution. No DUT/local BRAM indices.
      for(int oy=0;oy<4;oy++)for(int ox=0;ox<4;ox++)for(int n=0;n<8;n++)begin
        m=oy*4+ox;sum=0;
        for(int c=0;c<128;c++) begin
          a=int'($signed(image_data[(oy0+oy)*2][(ox0+ox)*2][c]));
          w=int'($signed(kernel[c][n0+n]));
          sum+=a*w;
        end
        golden[m][n]=sum;
        if(id>0 && sum!=previous_result[m][n])expected_changed++;
      end
      if(id>0 && expected_changed!=128)$fatal(1,"FAIL: second job must change all output values");
      // Independent cumulative PE references for clear/hold boundary checks.
      for(int mi=0;mi<16;mi++) begin
        seen[mi]=0;captured[mi]='x;
        for(int n=0;n<8;n++)begin
          read_seen[mi][n]=0;
          for(int r=0;r<8;r++)pe_running[r][n]=0;
        end
        for(int ct=0;ct<2;ct++)begin
          for(int c=0;c<64;c++)for(int n=0;n<8;n++)
            pe_running[c%8][n]+=a_value(mi,ct*64+c)*w_value(ct*64+c,n);
          for(int n=0;n<8;n++)begin
            prefix[mi][ct][n]=0;
            for(int r=0;r<8;r++)begin
              pe_prefix[mi][ct][r][n]=pe_running[r][n];
              prefix[mi][ct][n]+=pe_running[r][n];
            end
          end
        end
        for(int n=0;n<8;n++)if(prefix[mi][1][n] !== golden[mi][n])$fatal(1,"FAIL: reference consistency");
      end
      active=1;start=1;
      @(negedge clk);start=0;
      for(int mi=0;mi<16;mi++)for(int ct=0;ct<2;ct++)serve_load(mi,ct);
      waited=0;
      while(done !== 1'b1 && waited<300)begin @(posedge clk);#2;waited++;end
      if(done !== 1'b1)$fatal(1,"FAIL: timeout waiting for final done");
      repeat(3)@(negedge clk);
      if(load_requests!=32 || load_acks!=32 || k_starts!=32 || completed_segments!=32 ||
         requests!=1024 || feeds!=1024 || drains!=416 || pipe_clears!=32 || acc_clears!=16 ||
         act_responses!=1024 || weight_responses!=1024 || act_loads!=2048 || weight_loads!=2048 ||
         writes!=16 || outputs!=128 || host_dones!=1 || busy || done || load_req || job_inflight || segment_inflight)
        $fatal(1,"FAIL: Cin128 per-job aggregate counts");
      read_results();
      for(int mi=0;mi<16;mi++)begin
        if(!seen[mi])$fatal(1,"FAIL: missing output address");
        for(int n=0;n<8;n++)if(!read_seen[mi][n])$fatal(1,"FAIL: missing output lane readback");
      end
      active=0;
      $display("CASE PASS: P6-C1 job=%0d origin=(%0d,%0d) OC=%0d..%0d, Cin=128, 32 loads/acks, 1024 pairs, 16 writes, 128/128 outputs and readbacks",
               id,oy0,ox0,n0,n0+7);
      $display("  A preload=%0d bytes, W preload=%0d 64-bit words, changed cached results=%0d",act_loads,weight_loads,changed_readbacks);
    end
  endtask

  initial begin
    rst_n=0;start=0;load_done=0;fmap_wr_en=0;weight_wr_en=0;out_rd_en=0;
    fmap_wr_addr=0;fmap_wr_data=0;weight_wr_addr=0;weight_wr_data=0;out_rd_addr=0;out_rd_lane_sel=0;
    active=0;job_inflight=0;segment_inflight=0;
    total_segments=0;total_loads=0;total_acks=0;total_requests=0;total_feeds=0;total_drains=0;
    total_writes=0;total_outputs=0;total_readbacks=0;total_act_loads=0;total_weight_loads=0;total_dones=0;
    for(int m=0;m<16;m++)for(int n=0;n<8;n++)previous_result[m][n]=0;
    repeat(4)@(negedge clk);rst_n=1;
    run_job(0,0,0,0);
    run_job(1,8,4,248);
    if(total_segments!=64 || total_loads!=64 || total_acks!=64 || total_requests!=2048 ||
       total_feeds!=2048 || total_drains!=832 || total_writes!=32 || total_outputs!=256 ||
       total_readbacks!=256 || total_act_loads!=4096 || total_weight_loads!=4096 || total_dones!=2)
      $fatal(1,"FAIL: lifetime Cin128 counts");
    $display("PASS: P6-C1 CIN128 TWO-C-TILE RELOAD AND INT32 ACCUMULATION");
    $display("  One DUT/reset, 2 jobs, 64 load handshakes, 2048 paired FEED, 32 final writes");
    $display("  256/256 full-Cin convolution outputs and 256/256 synchronous lane readbacks correct");
    $display("  PE partial sums preserved during Host waits; 128 changed cached outputs checked");
    $finish;
  end
  initial begin repeat(100000)@(posedge clk);$fatal(1,"FAIL: P6-C1 global timeout");end
endmodule
