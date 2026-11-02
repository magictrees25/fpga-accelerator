`timescale 1ns / 1ps

// Shared functional/performance harness. Select a wrapper as Simulation Top.
// All configurations use ONE P2 DUT and ONE reset. Golden uses full 56x56x64
// input, direct convolution with global pad1, independent of DUT/Tile addresses.
// CSVs describe TB-driven simulation at 10ns, NOT implemented/board/DDR speed.
module tb_layer1_p2_common #(
  parameter int REGION_H=8, REGION_W=8, REGION_N=8,
  parameter bit WRITE_CSV=0
);
  localparam int TY=REGION_H/8,TX=REGION_W/8,NG=REGION_N/8;
  localparam int TILES=TY*TX*NG,OUTPUTS=REGION_H*REGION_W*REGION_N;
  localparam int GLOBAL_WORDS=56*56*64;
  localparam int PAIRS_PER_TILE=64*288;
  localparam logic signed [31:0] SENTINEL=32'h5a55a55a;
  logic clk=0;
  always #5 clk=~clk;
  logic rst_n,start,fmap_wr_en,weight_wr_en,out_rd_en;
  logic [15:0] fmap_wr_addr,weight_wr_addr,out_rd_addr;
  logic [7:0] fmap_wr_data;
  logic [63:0] weight_wr_data;
  logic [2:0] out_rd_lane_sel;
  logic signed [31:0] out_rd_data;
  logic out_rd_valid,out_rd_lane_valid,busy,done,any_overflow;
  fpga_system_top_layer1_tile_p2 dut (.*);

  wire clear_acc=dut.u_compute.clear_acc;
  wire feed_valid=dut.u_compute.feed_valid;
  wire pair_req_en=dut.u_compute.pair_req_en;
  wire [15:0] k_base=dut.u_compute.k_base;
  wire [15:0] fetch_k_base=dut.u_compute.fetch_k_base;
  wire [15:0] m_addr=dut.u_compute.m_addr;
  wire [15:0] n_base=dut.u_compute.n_base;
  wire [15:0] act_pair=dut.u_compute.act_pair_packed;
  wire [127:0] wgt_pair=dut.u_compute.wgt_pair_packed;
  wire [1:0] act_valid=dut.u_compute.act_rsp_valid;
  wire [1:0] wgt_valid=dut.u_compute.wgt_rsp_valid;
  wire k_start=dut.u_compute.u_ctrl.k_start;
  wire k_done=dut.u_compute.u_ctrl.k_done;
  wire array_enable=dut.u_compute.u_ctrl.array_enable;
  wire drain=array_enable && !clear_acc && !feed_valid;
  wire write_valid=dut.write_valid;
  wire [15:0] write_addr=dut.write_addr;
  wire [255:0] write_data=dut.write_data;
  wire [7:0] write_mask=dut.write_mask;

  logic signed [7:0] global_fmap[0:55][0:55][0:63];
  logic signed [7:0] global_kernel[0:2][0:2][0:63][0:63];
  logic signed [31:0] golden[0:REGION_H-1][0:REGION_W-1][0:REGION_N-1];
  logic signed [31:0] stitched[0:GLOBAL_WORDS-1];
  integer scatter_hits[0:GLOBAL_WORDS-1],scatter_owner[0:GLOBAL_WORDS-1];
  integer cache_hits[0:63],request_hits[0:575];
  logic signed [31:0] previous_output[0:63][0:7];
  logic [63:0] previous_weights[0:575];
  integer tile_id,oy0,ox0,oc0,tiles_completed;
  integer pairs,requests,writes,clears,drains,launches,kdones,other_cycles;
  integer read_requests,read_returns,aload,wload,pad_load,pad_k;
  integer round_pairs,round_drains,read_checks;
  bit round_open,computing,done_q,start_q,reset_released,finished;
  longint unsigned tick,start_tick,done_tick,compute_cycles;
  longint unsigned first_load,last_read,first_tile_load,last_tile_read;
  longint unsigned total_compute,total_pairs,total_requests,total_writes;
  longint unsigned total_clears,total_drains,total_launches,total_kdones,total_other;
  longint unsigned total_aload,total_wload,total_rreq,total_rret,total_pad_load,total_pad_k;
  longint unsigned starts,dones,min_compute,max_compute;
  longint unsigned changed_weights,changed_outputs;
  integer top_tiles,bottom_tiles,left_tiles,right_tiles;
  logic [255:0] snapshot;
  integer tile_fd,summary_fd;
  string tile_csv="layer1_p2_tiles.csv";
  string summary_csv="layer1_p2_summary.csv";

  function automatic integer activation(input integer m,input integer k);
    integer gy,gx,c;
    begin
      gy=oy0+m/8+k/192-1;
      gx=ox0+m%8+(k/64)%3-1;
      c=k%64;
      if(gy<0 || gy>=56 || gx<0 || gx>=56) return 0;
      return int'($signed(global_fmap[gy][gx][c]));
    end
  endfunction
  function automatic bit is_padding(input integer m,input integer k);
    integer gy,gx;
    begin
      gy=oy0+m/8+k/192-1;gx=ox0+m%8+(k/64)%3-1;
      return gy<0 || gy>=56 || gx<0 || gx>=56;
    end
  endfunction
  function automatic integer weight_value(input integer k,input integer n);
    return int'($signed(global_kernel[k/192][(k/64)%3][k%64][oc0+n]));
  endfunction

  task automatic prepare_golden;
    integer gy,gx;
    integer signed sum;
    begin
      for(int y=0;y<56;y++)
        for(int x=0;x<56;x++)
          for(int c=0;c<64;c++)
            global_fmap[y][x][c]=8'(((17*y+11*x+7*c)%127)-63);
      for(int ky=0;ky<3;ky++)
        for(int kx=0;kx<3;kx++)
          for(int c=0;c<64;c++)
            for(int n=0;n<64;n++)
              global_kernel[ky][kx][c][n]=8'(((13*ky+7*kx+3*c+5*n)%19)-9);
      $display("INFO: independent direct convolution Golden for %0d outputs",OUTPUTS);
      for(int oy=0;oy<REGION_H;oy++)
        for(int ox=0;ox<REGION_W;ox++)
          for(int n=0;n<REGION_N;n++) begin
            sum=0;
            for(int ky=0;ky<3;ky++)
              for(int kx=0;kx<3;kx++)
                for(int c=0;c<64;c++) begin
                  gy=oy+ky-1;gx=ox+kx-1;
                  if(gy>=0 && gy<56 && gx>=0 && gx<56)
                    sum+=int'($signed(global_fmap[gy][gx][c]))*
                         int'($signed(global_kernel[ky][kx][c][n]));
                end
            golden[oy][ox][n]=sum;
          end
      if(golden[0][0][0]!==2453 || golden[7][7][7]!==1616)
        $fatal(1,"FAIL independent Golden anchors");
      if(REGION_H==56 && REGION_W==56 && REGION_N==64)
        if(golden[55][55][63]!==1074) $fatal(1,"FAIL Golden bottom-right anchor");
    end
  endtask

  // Check requests BEFORE the edge against independent TB-owned M/K indices.
  task automatic check_request(input integer em,input integer ek);
    integer a;
    begin
      if(fetch_k_base!==16'(ek) || m_addr!==16'(em) || n_base!==0 ||
         dut.u_compute.u_act_reader.bank_rd_en!==2'b11 ||
         dut.u_compute.u_weight_reader.bank_rd_en!==2'b11)
        $fatal(1,"FAIL tile=%0d request control M=%0d K=%0d",tile_id,em,ek);
      for(int p=0;p<2;p++) begin
        a=((em/8+(ek+p)/192)*10+em%8+((ek+p)/64)%3)*64+(ek+p)%64;
        if(dut.u_compute.u_act_reader.fmap_addr[p]!==16'(a) ||
           dut.u_compute.u_act_reader.bank_rd_addr[p]!==16'(a/2) ||
           dut.u_compute.u_weight_reader.bank_rd_addr[p]!==16'(ek/2) ||
           a%2!=p) $fatal(1,"FAIL P2 request bank/address");
        request_hits[ek+p]++;
      end
      requests++;total_requests++;
    end
  endtask

  always @(posedge clk) begin : monitor
    bit count_compute,read_sample;
    integer req_expected,req_k,em,ek,got,want,idx,gy,gx,ci,phases;
    logic [63:0] expected_injection,expected_word;
    if(!rst_n) begin
      tick=0;computing=0;done_q=0;start_q=0;
    end else begin
      tick++;
      count_compute=computing;
      read_sample=out_rd_en;
      if($isunknown({clear_acc,feed_valid,pair_req_en,write_valid,busy,done,any_overflow}) ||
         any_overflow!==0) $fatal(1,"FAIL unknown/overflow tile=%0d",tile_id);
      if((clear_acc || feed_valid || write_valid || k_done || drain) && !computing)
        $fatal(1,"FAIL transaction outside accepted start/done window");
      if(start) begin
        if(start_q || busy || done || computing || starts!=tile_id || dones!=tile_id ||
           aload!=6400 || wload!=576 || tiles_completed!=tile_id)
          $fatal(1,"FAIL preload/start/restart sequence");
        starts++;computing=1;start_tick=tick;
      end
      start_q=start;
      if(count_compute) begin
        compute_cycles++;total_compute++;
        phases=int'(clear_acc)+int'(feed_valid)+int'(drain)+
               int'(k_start)+int'(k_done)+int'(write_valid);
        if(phases>1) $fatal(1,"FAIL overlapping compute phases");
        if(phases==0) begin other_cycles++;total_other++;end
      end
      if(int'(count_compute)+int'(fmap_wr_en)+int'(weight_wr_en)+int'(out_rd_en)>1)
        $fatal(1,"FAIL serial host/compute phases overlap");
      if((fmap_wr_en || weight_wr_en) && first_load==0) first_load=tick;
      if((fmap_wr_en || weight_wr_en) && first_tile_load==0) first_tile_load=tick;

      if(weight_wr_en) begin
        idx=wload;
        if(idx>=576 || weight_wr_addr!==16'(idx)) $fatal(1,"FAIL weight preload order");
        expected_word=0;
        for(int n=0;n<8;n++) expected_word[n*8 +: 8]=8'(weight_value(idx,n));
        if(weight_wr_data!==expected_word) $fatal(1,"FAIL loaded weight data");
        if(tile_id>0 && weight_wr_data!==previous_weights[idx]) changed_weights++;
        previous_weights[idx]=weight_wr_data;
        wload++;total_wload++;
      end
      if(fmap_wr_en) begin
        idx=aload;
        if(idx>=6400 || fmap_wr_addr!==16'(idx)) $fatal(1,"FAIL fmap preload order");
        gy=oy0+idx/640-1;gx=ox0+(idx/64)%10-1;ci=idx%64;
        if(gy<0 || gy>=56 || gx<0 || gx>=56) begin
          want=0;pad_load++;total_pad_load++;
        end else want=int'($signed(global_fmap[gy][gx][ci]));
        if(int'($signed(fmap_wr_data))!==want) $fatal(1,"FAIL loaded fmap/halo data");
        aload++;total_aload++;
      end

      em=writes;req_expected=0;req_k=0;
      if(clear_acc) begin
        if(round_open || writes>=64 || clears!=writes ||
           pairs!=writes*288 || k_base!==0 || m_addr!==16'(em))
          $fatal(1,"FAIL CLEAR sequence");
        round_open=1;round_pairs=0;round_drains=0;
        clears++;total_clears++;req_expected=1;req_k=0;
      end
      if(round_open && !clear_acc && round_pairs<288 && !feed_valid)
        $fatal(1,"FAIL bubble in paired FEED");
      if(feed_valid) begin
        ek=2*round_pairs;
        if(!round_open || round_pairs>=288 || k_base!==16'(ek) ||
           m_addr!==16'(em) || n_base!==0 || act_valid!==2'b11 || wgt_valid!==2'b11 ||
           dut.u_compute.u_act_reader.rsp_k_base!==16'(ek) ||
           dut.u_compute.u_weight_reader.rsp_k_base!==16'(ek) ||
           dut.u_compute.u_act_reader.rsp_m_addr!==16'(em) ||
           dut.u_compute.u_weight_reader.rsp_n_base!==0)
          $fatal(1,"FAIL FEED indices/response tags tile=%0d M=%0d K=%0d",tile_id,em,ek);
        expected_injection=0;
        for(int p=0;p<2;p++) begin
          want=activation(em,ek+p);
          got=int'($signed(act_pair[p*8 +: 8]));
          if(got!==want) $fatal(1,"FAIL FEED A tile=%0d M=%0d K=%0d got=%0d want=%0d",tile_id,em,ek+p,got,want);
          expected_injection[((ek+p)%8)*8 +: 8]=8'(want);
          if(is_padding(em,ek+p)) begin pad_k++;total_pad_k++;end
          for(int n=0;n<8;n++)
            if(int'($signed(wgt_pair[(p*8+n)*8 +: 8]))!==weight_value(ek+p,n))
              $fatal(1,"FAIL FEED B tile=%0d M=%0d K=%0d n=%0d",tile_id,em,ek+p,n);
        end
        if(dut.u_compute.u_ctrl.act_in_packed!==expected_injection ||
           dut.u_compute.u_ctrl.wgt_pair_in_packed!==wgt_pair)
          $fatal(1,"FAIL K injection rows/weights");
        if(round_pairs<287) begin req_expected=1;req_k=ek+2;end
        round_pairs++;pairs++;total_pairs++;
      end
      if(pair_req_en!==1'(req_expected)) $fatal(1,"FAIL request timing");
      if(pair_req_en) check_request(em,req_k);
      else if(dut.u_compute.u_act_reader.bank_rd_en!==2'b00 ||
              dut.u_compute.u_weight_reader.bank_rd_en!==2'b00)
        $fatal(1,"FAIL read request leaked");

      if(drain) begin
        if(!round_open || round_pairs!=288 ||
           dut.u_compute.u_ctrl.act_in_packed!==0 ||
           dut.u_compute.u_ctrl.wgt_pair_in_packed!==0)
          $fatal(1,"FAIL DRAIN boundary");
        round_drains++;drains++;total_drains++;
      end
      if(k_start) begin launches++;total_launches++;end
      if(k_done) begin
        if(!round_open || round_pairs!=288 || round_drains!=13)
          $fatal(1,"FAIL K done phase counts");
        snapshot=dut.u_compute.u_ctrl.k_result_packed;
        for(int n=0;n<8;n++)
          if(int'($signed(snapshot[n*32 +: 32]))!==golden[oy0+em/8][ox0+em%8][oc0+n])
            $fatal(1,"FAIL snapshot result tile=%0d M=%0d n=%0d",tile_id,em,n);
        kdones++;total_kdones++;
      end
      if(write_valid) begin
        if(!round_open || round_pairs!=288 || round_drains!=13 || writes>=64 ||
           write_addr!==16'(em) || dut.write_m_addr!==16'(em) ||
           dut.write_n_base!==0 || write_mask!==8'hff || write_data!==snapshot)
          $fatal(1,"FAIL write metadata/phase/snapshot");
        for(int n=0;n<8;n++)
          if(int'($signed(write_data[n*32 +: 32]))!==golden[oy0+em/8][ox0+em%8][oc0+n])
            $fatal(1,"FAIL write result tile=%0d M=%0d n=%0d",tile_id,em,n);
        if(cache_hits[em]!=tile_id) $fatal(1,"FAIL output cache generation");
        cache_hits[em]++;
        writes++;total_writes++;round_open=0;round_pairs=0;round_drains=0;
      end
      if(out_rd_en) begin read_requests++;total_rreq++;end

      #1;
      if(done && done_q) $fatal(1,"FAIL done wider than one clock");
      if(done) begin
        if(!computing || writes!=64 || dones!=tile_id) $fatal(1,"FAIL unexpected done");
        computing=0;dones++;done_tick=tick;
        if(done_tick-start_tick!=compute_cycles) $fatal(1,"FAIL compute time accounting");
      end
      done_q=done;
      if(out_rd_valid!==read_sample) $fatal(1,"FAIL readback latency/valid");
      if(out_rd_valid) begin
        if(out_rd_lane_valid!==1) $fatal(1,"FAIL invalid output lane");
        read_returns++;total_rret++;last_read=tick;last_tile_read=tick;
      end
    end
  end

  always @(negedge rst_n)
    if(reset_released===1) $fatal(1,"FAIL inter-Tile reset");

  task automatic load_tile;
    integer gy,gx;
    begin
      for(int k=0;k<576;k++) begin
        @(negedge clk);
        weight_wr_en=1;weight_wr_addr=16'(k);
        for(int n=0;n<8;n++) weight_wr_data[n*8 +: 8]=8'(weight_value(k,n));
      end
      @(negedge clk);weight_wr_en=0;
      for(int y=0;y<10;y++)
        for(int x=0;x<10;x++)
          for(int c=0;c<64;c++) begin
            @(negedge clk);
            fmap_wr_en=1;fmap_wr_addr=16'((y*10+x)*64+c);
            gy=oy0+y-1;gx=ox0+x-1;
            if(gy<0 || gy>=56 || gx<0 || gx>=56) fmap_wr_data=0;
            else fmap_wr_data=global_fmap[gy][gx][c];
          end
      @(negedge clk);fmap_wr_en=0;
    end
  endtask

  task automatic read_and_stitch;
    integer gy,gx,gn,addr,got,want,prev_id,py,px,pn;
    integer changed,expected_changed;
    begin
      changed=0;expected_changed=0;py=0;px=0;pn=0;
      if(tile_id>0) begin
        prev_id=tile_id-1;
        py=(prev_id/(TX*NG))*8;px=((prev_id/NG)%TX)*8;pn=(prev_id%NG)*8;
      end
      for(int m=0;m<64;m++)
        for(int n=0;n<8;n++) begin
          @(negedge clk);
          out_rd_en=1;out_rd_addr=16'(m);out_rd_lane_sel=3'(n);
          @(posedge clk);#2;
          if(out_rd_valid!==1 || out_rd_lane_valid!==1) $fatal(1,"FAIL missing output read");
          gy=oy0+m/8;gx=ox0+m%8;gn=oc0+n;
          got=int'($signed(out_rd_data));want=golden[gy][gx][gn];
          if(got!==want) $fatal(1,"FAIL readback tile=%0d m=%0d n=%0d got=%0d want=%0d",tile_id,m,n,got,want);
          if(tile_id>0) begin
            if(want!==golden[py+m/8][px+m%8][pn+n]) expected_changed++;
            if(got!==previous_output[m][n]) changed++;
          end
          previous_output[m][n]=got;
          addr=(gy*56+gx)*64+gn;
          if(scatter_hits[addr]!=0) $fatal(1,"FAIL duplicate global scatter addr=%0d",addr);
          scatter_hits[addr]++;scatter_owner[addr]=tile_id;stitched[addr]=got;
          read_checks++;
        end
      @(negedge clk);out_rd_en=0;
      @(posedge clk);#2;
      if(out_rd_valid!==0) $fatal(1,"FAIL stale output read valid");
      if(changed!=expected_changed) $fatal(1,"FAIL stale output cache comparison");
      changed_outputs+=changed;
    end
  endtask

  task automatic metric_i(input string name,input longint unsigned value,input string units);
    $fdisplay(summary_fd,"%s,%0d,%s",name,value,units);
  endtask
  task automatic metric_r(input string name,input real value,input string units);
    $fdisplay(summary_fd,"%s,%0.8f,%s",name,value,units);
  endtask

  task automatic report_tile;
    begin
      if(compute_cycles!=clears+pairs+drains+launches+kdones+writes+other_cycles ||
         clears!=64 || pairs!=PAIRS_PER_TILE || requests!=PAIRS_PER_TILE ||
         drains!=64*13 || launches!=64 || kdones!=64 || writes!=64 || other_cycles!=0 ||
         aload!=6400 || wload!=576 || read_requests!=512 || read_returns!=512 ||
         read_checks!=512 || round_open || computing || busy!==0 || done!==0)
        $fatal(1,"FAIL per-Tile counts tile=%0d pairs=%0d writes=%0d cycles=%0d",tile_id,pairs,writes,compute_cycles);
      for(int k=0;k<576;k++) if(request_hits[k]!=64) $fatal(1,"FAIL K request coverage k=%0d",k);
      if(tile_id==0 || compute_cycles<min_compute) min_compute=compute_cycles;
      if(compute_cycles>max_compute) max_compute=compute_cycles;
      if(WRITE_CSV)
        $fdisplay(tile_fd,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0.8f",
          tile_id,oy0,ox0,oc0,start_tick,done_tick,compute_cycles,clears,pairs,2*pairs,
          drains,launches,kdones,writes,other_cycles,aload,wload,read_requests,read_returns,
          aload,8*wload,4*read_returns,pad_load,pad_k,294912,first_tile_load,last_tile_read,
          100.0*294912.0/(64.0*real'(compute_cycles)));
      $display("TILE PASS: P2 id=%0d origin=(%0d,%0d) channels=%0d..%0d; %0d pairs, %0d cycles, 512/512 outputs",
        tile_id,oy0,ox0,oc0,oc0+7,pairs,compute_cycles);
    end
  endtask

  task automatic report_summary;
    longint unsigned e2e,gap,macs;
    begin
      e2e=last_read-first_load+1;
      macs=longint'(OUTPUTS)*576;
      if(e2e<total_compute+total_aload+total_wload+total_rreq)
        $fatal(1,"FAIL E2E phase accounting");
      gap=e2e-total_compute-total_aload-total_wload-total_rreq;
      if(WRITE_CSV) begin
        metric_i("tiles",TILES,"tiles");metric_i("start_count",starts,"events");
        metric_i("done_count",dones,"events");metric_i("pe_count",64,"PE");
        metric_i("k_parallel",2,"K_per_pair");
        metric_i("dense_mac_count",macs,"MAC");
        metric_i("nonpadding_mac_count",macs-total_pad_k*8,"MAC");
        metric_i("padding_mac_count",total_pad_k*8,"MAC");
        metric_i("sum_start_to_done_cycles",total_compute,"cycles");
        metric_i("tile_compute_min_cycles",min_compute,"cycles");
        metric_i("tile_compute_max_cycles",max_compute,"cycles");
        metric_i("feed_pair_cycles",total_pairs,"cycles");
        metric_i("valid_k_elements",2*total_pairs,"K_elements");
        metric_i("pair_request_cycles",total_requests,"cycles");
        metric_i("clear_cycles",total_clears,"cycles");
        metric_i("drain_cycles",total_drains,"cycles");
        metric_i("launch_cycles",total_launches,"cycles");
        metric_i("kdone_cycles",total_kdones,"cycles");
        metric_i("output_bram_write_cycles",total_writes,"cycles");
        metric_i("compute_other_cycles",total_other,"cycles");
        metric_i("array_enabled_nonclear_cycles",total_pairs+total_drains,"cycles");
        metric_i("activation_preload_cycles",total_aload,"cycles");
        metric_i("weight_preload_cycles",total_wload,"cycles");
        metric_i("output_read_request_cycles",total_rreq,"cycles");
        metric_i("output_read_response_cycles",total_rret,"cycles");
        metric_i("e2e_first_preload_cycle",first_load,"cycle_index");
        metric_i("e2e_last_read_cycle",last_read,"cycle_index");
        metric_i("e2e_cycles",e2e,"cycles");metric_i("host_gap_cycles",gap,"cycles");
        metric_i("activation_preload_bytes",total_aload,"bytes");
        metric_i("padding_preload_bytes",total_pad_load,"bytes");
        metric_i("real_activation_preload_bytes",total_aload-total_pad_load,"bytes");
        metric_i("weight_preload_bytes",total_wload*8,"bytes");
        metric_i("output_read_bytes",total_rret*4,"bytes");
        metric_i("output_bram_write_bytes",total_writes*32,"bytes");
        metric_i("host_payload_bytes",total_aload+8*total_wload+4*total_rret,"bytes");
        metric_r("tb_clock_period_ns",10.0,"ns");
        metric_r("compute_latency_at_tb_clock_ms",real'(total_compute)*10.0/1.0e6,"ms");
        metric_r("e2e_latency_at_tb_clock_ms",real'(e2e)*10.0/1.0e6,"ms");
        metric_r("dense_util_feed_pct",100.0*real'(macs)/(64.0*real'(total_pairs)),"percent");
        metric_r("dense_util_compute_pct",100.0*real'(macs)/(64.0*real'(total_compute)),"percent");
        metric_r("dense_util_e2e_pct",100.0*real'(macs)/(64.0*real'(e2e)),"percent");
        $fdisplay(summary_fd,"functional_status,PASS,text");
        $fdisplay(summary_fd,"run_status,COMPLETE,text");
        $fdisplay(summary_fd,"measurement_scope,simulation_only_not_board_or_DDR,text");
        $fdisplay(summary_fd,"compute_window,start_accept_edge_exclusive_to_done_assert_edge_inclusive,text");
        $fdisplay(summary_fd,"e2e_window,first_preload_edge_to_last_read_response_edge_inclusive,text");
        $fdisplay(summary_fd,"utilization_basis,logical_dense_MACs_over_64_PE_cycle_capacity,text");
        $fdisplay(summary_fd,"resource_and_timing_status,not_measured,text");
        $fclose(tile_fd);$fclose(summary_fd);
        $display("CSV READY: %s ; %s (simulation working directory)",tile_csv,summary_csv);
      end
      $display("PERF: measured compute=%0d cycles, E2E=%0d cycles, logical compute util=%0.4f%%",
        total_compute,e2e,100.0*real'(macs)/(64.0*real'(total_compute)));
      $display("PERF: time conversion uses TB 100MHz; no implementation/board timing claim");
    end
  endtask

  initial begin : stimulus
    integer waited,addr,owner,compared;
    longint signed checksum;
    rst_n=0;reset_released=0;finished=0;start=0;
    fmap_wr_en=0;fmap_wr_addr=0;fmap_wr_data=0;
    weight_wr_en=0;weight_wr_addr=0;weight_wr_data=0;
    out_rd_en=0;out_rd_addr=0;out_rd_lane_sel=0;
    tile_id=0;oy0=0;ox0=0;oc0=0;tiles_completed=0;
    pairs=0;requests=0;writes=0;clears=0;drains=0;launches=0;kdones=0;other_cycles=0;
    read_requests=0;read_returns=0;aload=0;wload=0;pad_load=0;pad_k=0;
    round_pairs=0;round_drains=0;round_open=0;read_checks=0;
    compute_cycles=0;start_tick=0;done_tick=0;
    first_load=0;last_read=0;first_tile_load=0;last_tile_read=0;
    total_compute=0;total_pairs=0;total_requests=0;total_writes=0;
    total_clears=0;total_drains=0;total_launches=0;total_kdones=0;total_other=0;
    total_aload=0;total_wload=0;total_rreq=0;total_rret=0;total_pad_load=0;total_pad_k=0;
    starts=0;dones=0;min_compute=0;max_compute=0;changed_weights=0;changed_outputs=0;
    top_tiles=0;bottom_tiles=0;left_tiles=0;right_tiles=0;snapshot=0;
    if(REGION_H<8 || REGION_H>56 || REGION_H%8!=0 ||
       REGION_W<8 || REGION_W>56 || REGION_W%8!=0 ||
       REGION_N<8 || REGION_N>64 || REGION_N%8!=0)
      $fatal(1,"FAIL unsupported TB region");
    for(int a=0;a<GLOBAL_WORDS;a++) begin
      stitched[a]=SENTINEL;scatter_hits[a]=0;scatter_owner[a]=-1;
    end
    for(int m=0;m<64;m++) begin
      cache_hits[m]=0;
      for(int n=0;n<8;n++) previous_output[m][n]=SENTINEL;
    end
    for(int k=0;k<576;k++) begin previous_weights[k]=0;request_hits[k]=0;end
    if(WRITE_CSV) begin
      if($value$plusargs("P2_TILES_CSV=%s",tile_csv)) begin end
      if($value$plusargs("P2_SUMMARY_CSV=%s",summary_csv)) begin end
      tile_fd=$fopen(tile_csv,"w");summary_fd=$fopen(summary_csv,"w");
      if(tile_fd==0 || summary_fd==0) $fatal(1,"FAIL cannot create P2 CSV files");
      $fdisplay(tile_fd,"tile,oy,ox,oc,start_cycle,done_cycle,compute_cycles,clear_cycles,feed_pair_cycles,valid_k_elements,drain_cycles,launch_cycles,kdone_cycles,output_write_cycles,compute_other_cycles,activation_load_cycles,weight_load_cycles,read_request_cycles,read_response_cycles,activation_bytes,weight_bytes,output_read_bytes,padding_load_bytes,padding_k_elements,dense_mac_count,first_load_cycle,last_read_cycle,dense_compute_util_pct");
      $fdisplay(summary_fd,"metric,value,unit");
      $fdisplay(summary_fd,"initial_status,STARTED,text");
      $fflush(summary_fd);
    end
    prepare_golden();
    repeat(3) @(negedge clk);rst_n=1;reset_released=1;
    for(int ty=0;ty<TY;ty++)
      for(int tx=0;tx<TX;tx++)
        for(int ng=0;ng<NG;ng++) begin
          @(negedge clk);
          if(busy!==0 || done!==0 || computing) $fatal(1,"FAIL not idle between tiles");
          tile_id=(ty*TX+tx)*NG+ng;oy0=ty*8;ox0=tx*8;oc0=ng*8;
          pairs=0;requests=0;writes=0;clears=0;drains=0;launches=0;kdones=0;other_cycles=0;
          read_requests=0;read_returns=0;aload=0;wload=0;pad_load=0;pad_k=0;
          round_pairs=0;round_drains=0;round_open=0;read_checks=0;snapshot=0;
          compute_cycles=0;start_tick=0;done_tick=0;first_tile_load=0;last_tile_read=0;
          for(int k=0;k<576;k++) request_hits[k]=0;
          load_tile();
          @(negedge clk);start=1;
          @(negedge clk);start=0;
          waited=0;
          while(done!==1 && waited<30000) begin @(posedge clk);#2;waited++;end
          if(done!==1) $fatal(1,"TIMEOUT P2 Tile id=%0d pairs=%0d writes=%0d",tile_id,pairs,writes);
          read_and_stitch();
          repeat(3) @(negedge clk);
          report_tile();
          tiles_completed++;
          if(oy0==0) top_tiles++;
          if(oy0==48) bottom_tiles++;
          if(ox0==0) left_tiles++;
          if(ox0==48) right_tiles++;
        end
    repeat(3) @(negedge clk);
    compared=0;checksum=0;
    for(int y=0;y<56;y++)
      for(int x=0;x<56;x++)
        for(int n=0;n<64;n++) begin
          addr=(y*56+x)*64+n;
          if(y<REGION_H && x<REGION_W && n<REGION_N) begin
            owner=((y/8)*TX+x/8)*NG+n/8;
            if(scatter_hits[addr]!=1 || scatter_owner[addr]!=owner ||
               stitched[addr]!==golden[y][x][n])
              $fatal(1,"FAIL global HWC scatter addr=%0d",addr);
            compared++;checksum+=int'($signed(stitched[addr]));
          end else if(scatter_hits[addr]!=0 || scatter_owner[addr]!=-1 ||
                      stitched[addr]!==SENTINEL)
            $fatal(1,"FAIL write outside requested region");
        end
    if(compared!=OUTPUTS || tiles_completed!=TILES || starts!=TILES || dones!=TILES ||
       total_pairs!=TILES*PAIRS_PER_TILE || total_requests!=total_pairs ||
       total_writes!=TILES*64 || total_clears!=TILES*64 ||
       total_drains!=TILES*64*13 || total_launches!=TILES*64 ||
       total_kdones!=TILES*64 || total_other!=0 ||
       total_aload!=TILES*6400 || total_wload!=TILES*576 ||
       total_rreq!=OUTPUTS || total_rret!=OUTPUTS ||
       busy!==0 || done!==0 || any_overflow!==0)
      $fatal(1,"FAIL final lifetime counts");
    for(int m=0;m<64;m++) if(cache_hits[m]!=TILES) $fatal(1,"FAIL cache write generations");
    if(NG>1 && changed_weights!=(TILES-1)*576) $fatal(1,"FAIL weight switches");
    if(REGION_H==56 && REGION_W==56 && REGION_N==64) begin
      if(total_pad_load!=141312 || total_pad_k!=342016 || checksum!=7795 ||
         changed_outputs!=200087 || top_tiles!=56 || bottom_tiles!=56 ||
         left_tiles!=56 || right_tiles!=56)
        $fatal(1,"FAIL full-layer borders/padding/cache/checksum");
    end
    report_summary();
    finished=1;
    $display("PASS: P2 layer1.0.conv1, %0d Tiles, %0d pair FEED cycles, %0d K elements, %0d writes, %0d/%0d outputs and global stitching correct",
      TILES,total_pairs,2*total_pairs,total_writes,compared,OUTPUTS);
    $finish;
  end
  initial begin
    repeat(TILES*40000+10000) @(posedge clk);
    if(!finished) $fatal(1,"TIMEOUT P2 shared Tile TB tile=%0d",tile_id);
  end
endmodule

