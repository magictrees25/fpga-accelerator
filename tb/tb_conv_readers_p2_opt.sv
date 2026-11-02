`timescale 1ns / 1ps

// Optimized Reader verification, including cycle equality with original Reader. No PE, K controller, GEMM controller or convolution
// result is instantiated. User runs this test in Vivado.
// Drive at negedge; check old response at posedge BEFORE NBA, then check the
// newly registered response at posedge+#1. This detects extra/missing latency.
module tb_conv_readers_p2_opt;
    localparam int H=10, W=10, C=64, KH=3, KW=3, N=8;
    localparam int OH=8, OW=8, M=64, K=576, GROUPS=288, EPOCHS=2;
    localparam int EXPECT_SCAN_PAIRS=EPOCHS*M*GROUPS; // 36864
    localparam int EXPECT_PROBE_PAIRS=8;
    localparam int EXPECT_PAIRS=EXPECT_SCAN_PAIRS+EXPECT_PROBE_PAIRS;

    logic clk=1'b0;
    always #5 clk=~clk;
    logic rst_n, pair_req_en;
    logic [15:0] m_addr, n_base, fetch_k_base;
    logic fmap_wr_en, weight_wr_en;
    logic [15:0] fmap_wr_addr, weight_wr_addr;
    logic [7:0] fmap_wr_data;
    logic [63:0] weight_wr_data;
    logic [15:0] act_pair_packed;
    logic [127:0] wgt_pair_packed;
    logic [1:0] act_rsp_valid, wgt_rsp_valid;

    conv_bram_reader_p2_opt u_act (
        .clk(clk), .rst_n(rst_n), .m_addr(m_addr),
        .pair_req_en(pair_req_en), .fetch_k_base(fetch_k_base),
        .wr_en(fmap_wr_en), .wr_addr(fmap_wr_addr), .wr_data(fmap_wr_data),
        .act_pair_packed(act_pair_packed), .act_rsp_valid(act_rsp_valid)
    );
    // Original PASS P2 Reader, driven by the SAME requests and preload.
    // Equality is additional coverage; independent reference arrays below
    // remain the primary expected-data source.
    logic [15:0] reference_act_pair;
    logic [1:0] reference_act_valid;
    conv_bram_reader_p2 u_act_reference (
        .clk(clk), .rst_n(rst_n), .m_addr(m_addr),
        .pair_req_en(pair_req_en), .fetch_k_base(fetch_k_base),
        .wr_en(fmap_wr_en), .wr_addr(fmap_wr_addr), .wr_data(fmap_wr_data),
        .act_pair_packed(reference_act_pair), .act_rsp_valid(reference_act_valid)
    );

    conv_weight_bram_reader_p2 u_wgt (
        .clk(clk), .rst_n(rst_n), .n_base(n_base),
        .pair_req_en(pair_req_en), .fetch_k_base(fetch_k_base),
        .wr_en(weight_wr_en), .wr_addr(weight_wr_addr), .wr_data(weight_wr_data),
        .wgt_pair_packed(wgt_pair_packed), .wgt_rsp_valid(wgt_rsp_valid)
    );

    // TB reference built from spatial/channel/kernel loops, not DUT addresses.
    integer fmap [0:EPOCHS-1][0:H-1][0:W-1][0:C-1];
    integer kernel [0:EPOCHS-1][0:KH-1][0:KW-1][0:C-1][0:N-1];
    integer expected_addr [0:M-1][0:K-1];
    integer expected_act [0:EPOCHS-1][0:M-1][0:K-1];
    integer expected_wgt [0:EPOCHS-1][0:K-1][0:N-1];
    integer hits [0:EPOCHS-1][0:M-1][0:GROUPS-1];

    integer epoch;
    bit scan_request;
    bit pending_valid, pending_scan;
    integer pending_m, pending_k, pending_epoch;
    integer scan_pairs, probe_pairs, pairs_checked, act_checks, weight_checks;
    integer bursts_checked, burst_length, reset_checks, idle_checks;
    integer fmap_writes, weight_writes, halo_zero_checks, distinct_pair_checks;
    logic [255:0] act_values_seen, weight_values_seen;

    task automatic check_response(
        input bit valid_expected,
        input integer exp_epoch,
        input integer exp_m,
        input integer exp_k,
        input string sample_point
    );
        integer got, want;
        begin
            if (act_rsp_valid !== reference_act_valid ||
                act_pair_packed !== reference_act_pair)
                $fatal(1,"FAIL %s: optimized/original Reader response or latency differs",sample_point);
            if (valid_expected &&
                (u_act.rsp_k_base !== u_act_reference.rsp_k_base ||
                 u_act.rsp_m_addr !== u_act_reference.rsp_m_addr))
                $fatal(1,"FAIL %s: optimized/original Reader tag mismatch",sample_point);
            if (!valid_expected) begin
                if (act_rsp_valid !== 2'b00 || wgt_rsp_valid !== 2'b00 ||
                    act_pair_packed !== 16'b0 || wgt_pair_packed !== 128'b0)
                    $fatal(1,"FAIL %s: stale/invalid response act_v=%b wgt_v=%b",
                           sample_point,act_rsp_valid,wgt_rsp_valid);
            end else begin
                if (act_rsp_valid !== 2'b11 || wgt_rsp_valid !== 2'b11)
                    $fatal(1,"FAIL %s: missing pair epoch=%0d M=%0d K=%0d act_v=%b wgt_v=%b",
                           sample_point,exp_epoch,exp_m,exp_k,act_rsp_valid,wgt_rsp_valid);
                if (u_act.rsp_k_base !== 16'(exp_k) ||
                    u_wgt.rsp_k_base !== 16'(exp_k) ||
                    u_act.rsp_m_addr !== 16'(exp_m) ||
                    u_wgt.rsp_n_base !== 16'd0)
                    $fatal(1,"FAIL %s: request tag mismatch M=%0d K=%0d",
                           sample_point,exp_m,exp_k);
                for (int p=0;p<2;p++) begin
                    got=int'($signed(act_pair_packed[p*8 +: 8]));
                    want=expected_act[exp_epoch][exp_m][exp_k+p];
                    if (got !== want)
                        $fatal(1,"FAIL %s: activation epoch=%0d M=%0d K=%0d p=%0d got=%0d want=%0d",
                               sample_point,exp_epoch,exp_m,exp_k+p,p,got,want);
                    for (int n=0;n<N;n++) begin
                        got=int'($signed(wgt_pair_packed[(p*N+n)*8 +: 8]));
                        want=expected_wgt[exp_epoch][exp_k+p][n];
                        if (got !== want)
                            $fatal(1,"FAIL %s: weight epoch=%0d K=%0d p=%0d n=%0d got=%0d want=%0d",
                                   sample_point,exp_epoch,exp_k+p,p,n,got,want);
                    end
                end
            end
        end
    endtask

    // All scoreboard state below is single-writer, except initialization.
    // Counters survive the mid-test reset; only the pending response is flushed.
    always @(posedge clk) begin : monitor
        integer address_expected, req_m, req_k, req_epoch;
        bit req_valid, req_scan;
        logic [1:0] expected_bank_we;
        if (fmap_wr_en) begin
            expected_bank_we=fmap_wr_addr[0] ? 2'b10 : 2'b01;
            if (u_act.bank_wr_en !== expected_bank_we ||
                u_act.bank_wr_addr !== (fmap_wr_addr >> 1))
                $fatal(1,"FAIL: activation write bank/address addr=%0d",fmap_wr_addr);
            fmap_writes=fmap_writes+1;
        end else if (u_act.bank_wr_en !== 2'b00)
            $fatal(1,"FAIL: activation write enable leaked");

        if (weight_wr_en) begin
            expected_bank_we=weight_wr_addr[0] ? 2'b10 : 2'b01;
            if (u_wgt.bank_wr_en !== expected_bank_we ||
                u_wgt.bank_wr_addr !== (weight_wr_addr >> 1))
                $fatal(1,"FAIL: weight write bank/address addr=%0d",weight_wr_addr);
            weight_writes=weight_writes+1;
        end else if (u_wgt.bank_wr_en !== 2'b00)
            $fatal(1,"FAIL: weight write enable leaked");

        if (!rst_n) begin
            pending_valid=0;
            pending_scan=0;
            burst_length=0;
            #1;
            check_response(0,0,0,0,"reset");
            if (u_act.bank_rd_en !== 2'b00 || u_wgt.bank_rd_en !== 2'b00)
                $fatal(1,"FAIL: RAM read enabled during reset");
        end else begin
            // This is the response a synchronous downstream consumer sees now.
            check_response(pending_valid,pending_epoch,pending_m,pending_k,"consumer edge");
            if ($isunknown({pair_req_en,m_addr,n_base,fetch_k_base}))
                $fatal(1,"FAIL: unknown request input");
            req_valid=pair_req_en;
            req_m=int'(m_addr);
            req_k=int'(fetch_k_base);
            req_epoch=epoch;
            req_scan=scan_request;
            if (req_valid) begin
                if (req_m<0 || req_m>=M || req_k<0 || req_k>=K-1 ||
                    (req_k%2)!=0 || n_base!==16'd0 ||
                    fmap_wr_en!==1'b0 || weight_wr_en!==1'b0)
                    $fatal(1,"FAIL: test driver issued an illegal request");
                if (u_act.bank_rd_en !== 2'b11 || u_wgt.bank_rd_en !== 2'b11 ||
                    u_act.addr_valid !== 2'b11)
                    $fatal(1,"FAIL: both banks must read each valid pair");
                for (int p=0;p<2;p++) begin
                    address_expected=expected_addr[req_m][req_k+p];
                    if (u_act.bank_rd_addr[p] !== u_act.bank_rd_addr[0] ||
                        u_act.bank_rd_addr[p] !== u_act_reference.bank_rd_addr[p] ||
                        u_act.fmap_addr[p] !== u_act_reference.fmap_addr[p])
                        $fatal(1,"FAIL: shared bank address/original address mismatch");

                    if (u_act.lane_k[p] !== 16'(req_k+p) ||
                        u_act.fmap_addr[p] !== 16'(address_expected) ||
                        u_act.bank_rd_addr[p] !== 16'(address_expected/2) ||
                        address_expected%2 != p ||
                        u_wgt.bank_rd_addr[p] !== 16'((req_k+p)/2))
                        $fatal(1,"FAIL: request bank/address M=%0d K=%0d p=%0d expected_fmap=%0d",
                               req_m,req_k,p,address_expected);
                end
                if (req_scan) begin
                    if (req_k==0)
                        burst_length=1;
                    else begin
                        if (!pending_valid || !pending_scan ||
                            pending_epoch!=req_epoch || pending_m!=req_m ||
                            pending_k!=req_k-2)
                            $fatal(1,"FAIL: bubble/reorder inside 288-pair burst");
                        burst_length=burst_length+1;
                    end
                    if (hits[req_epoch][req_m][req_k/2]!=0)
                        $fatal(1,"FAIL: duplicate scan pair");
                    hits[req_epoch][req_m][req_k/2]=1;
                    if (req_k==K-2) begin
                        if (burst_length!=GROUPS)
                            $fatal(1,"FAIL: burst has %0d pairs",burst_length);
                        bursts_checked=bursts_checked+1;
                    end
                end
            end else begin
                if (u_act.bank_rd_en !== 2'b00 || u_wgt.bank_rd_en !== 2'b00)
                    $fatal(1,"FAIL: RAM read enabled with no request");
            end

            pending_valid=req_valid;
            pending_m=req_m;
            pending_k=req_k;
            pending_epoch=req_epoch;
            pending_scan=req_scan;
            #1; // allow BRAM/valid/tag nonblocking assignments to settle
            check_response(req_valid,req_epoch,req_m,req_k,"registered response");
            if (req_valid) begin
                pairs_checked=pairs_checked+1;
                if (req_scan) scan_pairs=scan_pairs+1;
                else probe_pairs=probe_pairs+1;
                act_checks=act_checks+2;
                weight_checks=weight_checks+2*N;
                if (wgt_pair_packed[63:0] !== wgt_pair_packed[127:64])
                    distinct_pair_checks=distinct_pair_checks+1;
                for (int p=0;p<2;p++) begin
                    act_values_seen[int'($signed(act_pair_packed[p*8 +: 8]))+128]=1'b1;
                    address_expected=expected_addr[req_m][req_k+p];
                    // Epoch 1 models the global top-left tile: local row/col 0 are halo.
                    if (req_epoch==1 &&
                        (address_expected/(W*C)==0 || (address_expected/C)%W==0)) begin
                        if (act_pair_packed[p*8 +: 8] !== 8'd0)
                            $fatal(1,"FAIL: preloaded halo byte is not zero");
                        halo_zero_checks=halo_zero_checks+1;
                    end
                    for (int n=0;n<N;n++)
                        weight_values_seen[int'($signed(wgt_pair_packed[(p*N+n)*8 +: 8]))+128]=1'b1;
                end
            end else idle_checks=idle_checks+1;
        end
    end

    // One call occupies exactly one request cycle. Calls with en=1 can run
    // back-to-back. Invalid idle addresses intentionally stress stale returns.
    task automatic step(
        input bit en, input integer m, input integer k, input bit scan
    );
        begin
            @(negedge clk);
            pair_req_en=en;
            m_addr=16'(m);
            fetch_k_base=16'(k);
            n_base=0;
            scan_request=scan;
            #1;
            // Changing the next request must not change the registered response.
            check_response(pending_valid,pending_epoch,pending_m,pending_k,"between edges");
            @(posedge clk);
            #2; // monitor has completed post-NBA checks
        end
    endtask

    task automatic preload(input integer data_epoch);
        integer address_to_write, k;
        begin
            epoch=data_epoch;
            // Dataset 1 reverses write order to test bank routing independently
            // from any implied stream counter. All writes still use original addresses.
            for (int i=0;i<H*W*C;i++) begin
                address_to_write=(data_epoch==0) ? i : H*W*C-1-i;
                @(negedge clk);
                pair_req_en=0;
                scan_request=0;
                fmap_wr_en=1;
                fmap_wr_addr=16'(address_to_write);
                fmap_wr_data=8'(fmap[data_epoch][address_to_write/(W*C)]
                                  [(address_to_write/C)%W][address_to_write%C]);
            end
            @(negedge clk);
            fmap_wr_en=0;
            for (int i=0;i<K;i++) begin
                k=(data_epoch==0) ? i : K-1-i;
                @(negedge clk);
                weight_wr_en=1;
                weight_wr_addr=16'(k);
                for (int n=0;n<N;n++)
                    weight_wr_data[n*8 +: 8]=8'(expected_wgt[data_epoch][k][n]);
            end
            @(negedge clk);
            weight_wr_en=0;
        end
    endtask

    initial begin : stimulus
        integer m, k;
        rst_n=0; pair_req_en=0; m_addr=0; n_base=0; fetch_k_base=0;
        fmap_wr_en=0; fmap_wr_addr=0; fmap_wr_data=0;
        weight_wr_en=0; weight_wr_addr=0; weight_wr_data=0;
        epoch=0; scan_request=0;
        pending_valid=0; pending_scan=0; pending_m=0; pending_k=0; pending_epoch=0;
        scan_pairs=0; probe_pairs=0; pairs_checked=0; act_checks=0; weight_checks=0;
        bursts_checked=0; burst_length=0; reset_checks=0; idle_checks=0;
        fmap_writes=0; weight_writes=0; halo_zero_checks=0; distinct_pair_checks=0;
        act_values_seen='0; weight_values_seen='0;

        for (int e=0;e<EPOCHS;e++) begin
            for (int y=0;y<H;y++)
                for (int x=0;x<W;x++)
                    for (int c=0;c<C;c++) begin
                        fmap[e][y][x][c]=((53*y+29*x+11*c+67*e)%256)-128;
                        if (e==1 && (y==0 || x==0))
                            fmap[e][y][x][c]=0;
                    end
            for (int ky=0;ky<KH;ky++)
                for (int kx=0;kx<KW;kx++)
                    for (int c=0;c<C;c++)
                        for (int n=0;n<N;n++) begin
                            kernel[e][ky][kx][c][n]=((31*ky+53*kx+37*c+17*n+71*e)%256)-128;
                            expected_wgt[e][(ky*KW+kx)*C+c][n]=kernel[e][ky][kx][c][n];
                        end
            for (int oy=0;oy<OH;oy++)
                for (int ox=0;ox<OW;ox++) begin
                    m=oy*OW+ox;
                    for (int q=0;q<GROUPS;q++) hits[e][m][q]=0;
                    for (int ky=0;ky<KH;ky++)
                        for (int kx=0;kx<KW;kx++)
                            for (int c=0;c<C;c++) begin
                                k=(ky*KW+kx)*C+c;
                                expected_addr[m][k]=((oy+ky)*W+ox+kx)*C+c;
                                expected_act[e][m][k]=fmap[e][oy+ky][ox+kx][c];
                            end
                end
        end

        repeat (3) @(negedge clk);
        rst_n=1;
        for (int e=0;e<EPOCHS;e++) begin
            preload(e);
            $display("INFO: epoch=%0d loaded via original byte/64-bit ports; scan 64 M x 288 pairs",e);
            for (int mi=0;mi<M;mi++)
                for (int q=0;q<GROUPS;q++)
                    step(1,mi,2*q,1);
            step(0,65535,65535,0); // flush last response, no illegal RAM access
            step(0,63,573,0);     // odd K must be ignored when request is disabled

            // Gapped, non-monotonic requests: no hidden auto-increment allowed.
            step(1,63,574,0);
            step(0,65535,65535,0);
            step(1,0,0,0);
            step(0,0,1,0);
            step(1,17,192,0);
            step(0,0,0,0);
            $display("CASE CHECKED: epoch=%0d, 18432 consecutive scan pairs + 3 probes",e);
        end

        // Reset with a VALID response present; asynchronous invalidation must
        // remove that response. RAM contents must survive control reset.
        step(1,63,574,0);
        @(negedge clk);
        rst_n=0;
        pair_req_en=0;
        scan_request=0;
        #1;
        check_response(0,0,0,0,"asynchronous reset");
        reset_checks=reset_checks+1;
        repeat (3) @(negedge clk);
        rst_n=1;
        step(1,0,0,0);
        step(0,65535,65535,0);
        step(0,0,0,0);

        if (scan_pairs!=EXPECT_SCAN_PAIRS || probe_pairs!=EXPECT_PROBE_PAIRS ||
            pairs_checked!=EXPECT_PAIRS || bursts_checked!=EPOCHS*M)
            $fatal(1,"FAIL: counts scan=%0d probe=%0d pairs=%0d bursts=%0d",
                   scan_pairs,probe_pairs,pairs_checked,bursts_checked);
        if (act_checks!=2*EXPECT_PAIRS || weight_checks!=16*EXPECT_PAIRS ||
            fmap_writes!=EPOCHS*H*W*C || weight_writes!=EPOCHS*K)
            $fatal(1,"FAIL: data/write check counts");
        for (int e=0;e<EPOCHS;e++)
            for (int mi=0;mi<M;mi++)
                for (int q=0;q<GROUPS;q++)
                    if (hits[e][mi][q]!=1)
                        $fatal(1,"FAIL: missing scan coverage e=%0d M=%0d q=%0d",e,mi,q);
        if (act_values_seen[0]!==1'b1 || act_values_seen[128]!==1'b1 ||
            act_values_seen[255]!==1'b1 || weight_values_seen[0]!==1'b1 ||
            weight_values_seen[128]!==1'b1 || weight_values_seen[255]!==1'b1)
            $fatal(1,"FAIL: signed endpoint/zero coverage incomplete");
        if (halo_zero_checks==0 || distinct_pair_checks!=EXPECT_PAIRS ||
            reset_checks!=1 || idle_checks==0)
            $fatal(1,"FAIL: halo/independent streams/reset/idle coverage incomplete");

        $display("INFO: 128 bursts x 288 pairs; bank addresses, tags, first/last pairs checked");
        $display("INFO: two datasets, reverse reload, signed -128/0/127, halo zeros, bubbles and reset checked");
        $display("PASS: P2 OPT Readers (original equivalence), 36864 scan pairs + 8 probes, 73744 activations and 589952 weights correct");
        $finish;
    end

    initial begin : watchdog
        repeat (100000) @(posedge clk);
        $fatal(1,"TIMEOUT: P2 reader verification did not finish");
    end
endmodule


