`timescale 1ns / 1ps

// Combined K + M/N controller test with synchronous synthetic paired sources.
// No physical BRAM/address generator. Three starts, ONE reset, K16 and K576.
module tb_gemm_mn_controller_mem_p2;
  logic clk=0;
  always #5 clk=~clk;
  logic rst_n,start;
  logic [15:0] m_len,n_len,k_len,m_addr,n_base,k_base,fetch_k_base;
  logic pair_req_en,mem_clear_acc,mem_feed_valid;
  logic [15:0] act_pair_packed;
  logic [127:0] wgt_pair_packed;
  logic [1:0] act_rsp_valid,wgt_rsp_valid;
  logic write_valid,busy,done,any_overflow;
  logic [15:0] write_addr,write_m_addr,write_n_base;
  logic [7:0] write_mask;
  logic [255:0] write_data;
  logic [15:0] rsp_k,rsp_m;
  integer run_id,writes,feeds,requests,clears,drains,kdones,launches;
  integer round_pairs,round_drains,done_count,total_pairs,total_outputs;
  integer golden[0:1][0:7];
  bit active;
  logic [255:0] last_snapshot;
  gemm_mn_controller_mem_p2 dut (.*);

  function automatic integer aval(input integer run,input integer m,input integer k);
    return ((run*53+m*31+k*17)%256)-128;
  endfunction
  function automatic integer bval(input integer run,input integer k,input integer n);
    return ((run*71+k*29+n*13)%256)-128;
  endfunction

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      act_pair_packed<=0;wgt_pair_packed<=0;
      act_rsp_valid<=0;wgt_rsp_valid<=0;rsp_k<=0;rsp_m<=0;
    end else begin
      act_rsp_valid<={2{pair_req_en}};wgt_rsp_valid<={2{pair_req_en}};
      if(pair_req_en) begin
        rsp_k<=fetch_k_base;rsp_m<=m_addr;
        for(int p=0;p<2;p++) begin
          act_pair_packed[p*8 +: 8]<=8'(aval(run_id,int'(m_addr),int'(fetch_k_base)+p));
          for(int n=0;n<8;n++)
            wgt_pair_packed[(p*8+n)*8 +: 8]<=8'(bval(run_id,int'(fetch_k_base)+p,n));
        end
      end else begin act_pair_packed<=0;wgt_pair_packed<=0;end
    end
  end

  always @(posedge clk) if(rst_n && active) begin : monitor
    integer want_k,expected_requests;
    if($isunknown({busy,done,write_valid,mem_clear_acc,mem_feed_valid,pair_req_en,any_overflow}) ||
       any_overflow!==0) $fatal(1,"FAIL controller unknown/overflow");
    expected_requests=0;
    if(mem_clear_acc) begin
      if(round_pairs!=0 || round_drains!=0 || k_base!==16'd0 ||
         m_addr!==16'(writes) || n_base!==0)
        $fatal(1,"FAIL controller CLEAR ordering");
      clears++;
      expected_requests=1;want_k=0;
    end
    if(mem_feed_valid) begin
      if(k_base!==16'(round_pairs*2) || m_addr!==16'(writes) ||
         rsp_k!==k_base || rsp_m!==m_addr ||
         act_rsp_valid!==2'b11 || wgt_rsp_valid!==2'b11)
        $fatal(1,"FAIL controller response tag/FEED");
      for(int p=0;p<2;p++) begin
        if(int'($signed(act_pair_packed[p*8 +: 8])) !== aval(run_id,writes,round_pairs*2+p))
          $fatal(1,"FAIL controller A pair");
        for(int n=0;n<8;n++)
          if(int'($signed(wgt_pair_packed[(p*8+n)*8 +: 8])) !== bval(run_id,round_pairs*2+p,n))
            $fatal(1,"FAIL controller B pair");
      end
      if(round_pairs<int'(k_len)/2-1) begin expected_requests=1;want_k=round_pairs*2+2;end
      round_pairs++;feeds++;total_pairs++;
    end
    if(pair_req_en!==1'(expected_requests) ||
       (expected_requests && fetch_k_base!==16'(want_k)))
      $fatal(1,"FAIL controller prefetch schedule run=%0d",run_id);
    if(pair_req_en) requests++;
    if(dut.array_enable && !mem_clear_acc && !mem_feed_valid) begin
      if(pair_req_en || dut.act_in_packed!==0 || dut.wgt_pair_in_packed!==0)
        $fatal(1,"FAIL controller DRAIN injected data");
      round_drains++;drains++;
    end
    if(dut.k_start) launches++;
    if(dut.k_done) begin
      kdones++;
      if(round_pairs!=int'(k_len)/2 || round_drains!=13)
        $fatal(1,"FAIL controller dot-product phase counts");
      for(int n=0;n<8;n++)
        if(int'($signed(dut.k_result_packed[n*32 +: 32])) !== golden[writes][n])
          $fatal(1,"FAIL controller result snapshot");
      last_snapshot=dut.k_result_packed;
    end
    if(write_valid) begin
      if(write_addr!==16'(writes) || write_m_addr!==16'(writes) ||
         write_n_base!==0 || write_mask!==8'hff || write_data!==last_snapshot)
        $fatal(1,"FAIL controller output write metadata/snapshot");
      for(int n=0;n<8;n++) begin
        if(int'($signed(write_data[n*32 +: 32])) !== golden[writes][n])
          $fatal(1,"FAIL controller result run=%0d m=%0d n=%0d",run_id,writes,n);
        total_outputs++;
      end
      writes++;round_pairs=0;round_drains=0;
    end
    if(done) done_count++;
  end

  initial begin : drive
    integer wait_cycles;
    rst_n=0;start=0;m_len=0;n_len=8;k_len=0;run_id=0;active=0;
    total_pairs=0;total_outputs=0;
    repeat(3) @(negedge clk);
    rst_n=1;
    for(run_id=0;run_id<3;run_id++) begin
      @(negedge clk);
      m_len=(run_id==2)?1:2;k_len=(run_id==0)?16:576;
      writes=0;feeds=0;requests=0;clears=0;drains=0;kdones=0;launches=0;
      round_pairs=0;round_drains=0;done_count=0;last_snapshot=0;
      for(int m=0;m<int'(m_len);m++)
        for(int n=0;n<8;n++) begin
          golden[m][n]=0;
          for(int k=0;k<int'(k_len);k++) golden[m][n]+=aval(run_id,m,k)*bval(run_id,k,n);
        end
      active=1;start=1;
      @(negedge clk);start=0;
      wait_cycles=0;
      while(done!==1 && wait_cycles<2000) begin @(posedge clk);#2;wait_cycles++;end
      if(done!==1) $fatal(1,"TIMEOUT controller run=%0d",run_id);
      repeat(3) @(negedge clk);
      if(writes!=int'(m_len) || feeds!=int'(m_len)*int'(k_len)/2 ||
         requests!=feeds || clears!=writes || drains!=13*writes ||
         kdones!=writes || launches!=writes || done_count!=1 || busy!==0 || done!==0)
        $fatal(1,"FAIL controller final counts run=%0d",run_id);
      active=0;
      $display("CASE PASS: controller run=%0d M=%0d K=%0d pairs=%0d",run_id,m_len,k_len,feeds);
    end
    if(total_pairs!=880 || total_outputs!=40) $fatal(1,"FAIL controller lifetime counts");
    $display("PASS: P2 K/MN Controllers, 3 starts, 1 reset, 880 pairs, 40/40 outputs correct");
    $finish;
  end
  initial begin repeat(10000) @(posedge clk);$fatal(1,"TIMEOUT controller P2");end
endmodule

