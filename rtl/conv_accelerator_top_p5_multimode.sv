`timescale 1ns / 1ps

// P5-B1: one 8x8 INT8 compute array / one K-M-N controller, 3 run-time modes.
// This is a FUNCTIONAL INTEGRATION PROTOTYPE, not a memory-optimized engine.
// All three activation-reader memories and BOTH weight-reader memories exist
// physically; preloads/requests are gated to the selected mode.  BRAM use
// will therefore exceed that of the individual fixed-mode P3/P4 designs.
// Mode 0: 3x3/S1/P1, local 10x10x64 -> 8x8x8; M64,N8,K576, P3 LB reader.
// Mode 1: 1x1/S2/P0, local 8x8x64  -> 4x4x8; M16,N8,K64, P4 direct reader.
// Mode 2: 3x3/S2/P1, local 9x9x64  -> 4x4x8; M16,N8,K576, P4 direct reader.
// P1 means GLOBAL zero halo is preloaded by host for modes 0 and 2.
// cfg_mode MUST be held stable during preload, start and compute.  Mode
// changes are allowed after done/while idle, with a FULL preload each run.
// No reset is required between different mode runs. N tiling remains HOST-led.
module conv_accelerator_top_p5_multimode #(
    parameter int ADDR_WIDTH = 16
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    start,
    input  logic [1:0]              cfg_mode,
    input  logic                    fmap_wr_en,
    input  logic [ADDR_WIDTH-1:0]   fmap_wr_addr,
    input  logic [7:0]              fmap_wr_data,
    input  logic                    weight_wr_en,
    input  logic [ADDR_WIDTH-1:0]   weight_wr_addr,
    input  logic [63:0]             weight_wr_data,
    output logic                    write_valid,
    output logic [ADDR_WIDTH-1:0]   write_addr,
    output logic [ADDR_WIDTH-1:0]   write_m_addr,
    output logic [ADDR_WIDTH-1:0]   write_n_base,
    output logic [7:0]              write_mask,
    output logic [255:0]            write_data,
    output logic                    busy,
    output logic                    done,
    output logic                    any_overflow
);
    localparam logic [1:0] MODE_3X3_S1 = 2'd0;
    localparam logic [1:0] MODE_1X1_S2 = 2'd1;
    localparam logic [1:0] MODE_3X3_S2 = 2'd2;

    logic [1:0] active_mode;
    logic [ADDR_WIDTH-1:0] cfg_m_len, cfg_k_len;
    logic [ADDR_WIDTH-1:0] m_addr, n_base, k_base, fetch_k_base;
    logic pair_req_en, clear_acc, feed_valid;
    logic [15:0] act_pair_packed;
    logic [127:0] wgt_pair_packed;
    logic [1:0] act_rsp_valid, wgt_rsp_valid;
    logic [15:0] act_s1, act_1x1, act_s2;
    logic [1:0]  av_s1, av_1x1, av_s2;
    

    logic req_s1, req_1x1, req_s2;
    logic wr_s1, wr_1x1, wr_s2;
    // Weight request mode indicators (also used by TB)
    logic wreq_3x3, wreq_1x1;

    assign cfg_m_len = (cfg_mode == MODE_3X3_S1) ? ADDR_WIDTH'(64) : ADDR_WIDTH'(16);
    assign cfg_k_len = (cfg_mode == MODE_1X1_S2) ? ADDR_WIDTH'(64) : ADDR_WIDTH'(576);

    // On start, controller captures cfg M/K, and we capture the SAME mode.
    // Reader requests begin in S_CLEAR on a subsequent clock edge.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            active_mode <= MODE_3X3_S1;
        else if (start && !busy && !done && cfg_mode != 2'd3)
            active_mode <= cfg_mode;
    end

    gemm_mn_controller_mem_p2 #(.ADDR_WIDTH(ADDR_WIDTH)) u_ctrl (
        .clk(clk), .rst_n(rst_n), .start(start),
        .m_len(cfg_m_len), .n_len(ADDR_WIDTH'(8)), .k_len(cfg_k_len),
        .m_addr(m_addr), .n_base(n_base), .k_base(k_base),
        .fetch_k_base(fetch_k_base), .pair_req_en(pair_req_en),
        .act_pair_packed(act_pair_packed), .act_rsp_valid(act_rsp_valid),
        .wgt_pair_packed(wgt_pair_packed), .wgt_rsp_valid(wgt_rsp_valid),
        .mem_clear_acc(clear_acc), .mem_feed_valid(feed_valid),
        .write_valid(write_valid), .write_addr(write_addr),
        .write_m_addr(write_m_addr), .write_n_base(write_n_base),
        .write_mask(write_mask), .write_data(write_data),
        .busy(busy), .done(done), .any_overflow(any_overflow)
    );

    // Host writes use cfg_mode while idle; compute requests use active_mode.
    assign wr_s1  = fmap_wr_en && cfg_mode == MODE_3X3_S1;
    assign wr_1x1 = fmap_wr_en && cfg_mode == MODE_1X1_S2;
    assign wr_s2  = fmap_wr_en && cfg_mode == MODE_3X3_S2;

    assign req_s1  = pair_req_en && active_mode == MODE_3X3_S1;
    assign req_1x1 = pair_req_en && active_mode == MODE_1X1_S2;
    assign req_s2  = pair_req_en && active_mode == MODE_3X3_S2;

    conv_bram_reader_p3 #(.ADDR_WIDTH(ADDR_WIDTH)) u_act_s1 (
        .clk(clk), .rst_n(rst_n), .m_addr(m_addr),
        .pair_req_en(req_s1), .fetch_k_base(fetch_k_base),
        .wr_en(wr_s1), .wr_addr(fmap_wr_addr), .wr_data(fmap_wr_data),
        .act_pair_packed(act_s1), .act_rsp_valid(av_s1)
    );
    conv_bram_reader_p4_1x1 #(.ADDR_WIDTH(ADDR_WIDTH)) u_act_1x1 (
        .clk(clk), .rst_n(rst_n), .m_addr(m_addr),
        .pair_req_en(req_1x1), .fetch_k_base(fetch_k_base),
        .wr_en(wr_1x1), .wr_addr(fmap_wr_addr), .wr_data(fmap_wr_data),
        .act_pair_packed(act_1x1), .act_rsp_valid(av_1x1)
    );
    conv_bram_reader_p4_3x3s2 #(.ADDR_WIDTH(ADDR_WIDTH)) u_act_s2 (
        .clk(clk), .rst_n(rst_n), .m_addr(m_addr),
        .pair_req_en(req_s2), .fetch_k_base(fetch_k_base),
        .wr_en(wr_s2), .wr_addr(fmap_wr_addr), .wr_data(fmap_wr_data),
        .act_pair_packed(act_s2), .act_rsp_valid(av_s2)
    );

 
    // Weight request mode indicators
    assign wreq_1x1 = pair_req_en && active_mode == MODE_1X1_S2;
    assign wreq_3x3 = pair_req_en && active_mode != MODE_1X1_S2;

    // P5-B1: One shared weight BRAM reader for all convolution modes.
    // K=64  : 1x1/S2
    // K=576 : 3x3/S1 and 3x3/S2
    conv_weight_bram_reader_p5_shared #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .K_MAX(576)
    ) u_weight_shared (
        .clk(clk),
        .rst_n(rst_n),

        .n_base(n_base),
        .k_len(cfg_k_len),

        .pair_req_en(pair_req_en),
        .fetch_k_base(fetch_k_base),

        .wr_en(weight_wr_en),
        .wr_addr(weight_wr_addr),
        .wr_data(weight_wr_data),

        .wgt_pair_packed(wgt_pair_packed),
        .wgt_rsp_valid(wgt_rsp_valid)
    );


    always_comb begin
        case (active_mode)
            MODE_3X3_S1: begin
                act_pair_packed = act_s1;
                act_rsp_valid = av_s1;
            end
            MODE_1X1_S2: begin
                act_pair_packed = act_1x1;
                act_rsp_valid = av_1x1;
            end
            MODE_3X3_S2: begin
                act_pair_packed = act_s2;
                act_rsp_valid = av_s2;
            end
            default: begin
                act_pair_packed = '0;
                act_rsp_valid = '0;
            end
        endcase
      
    end

    // synthesis translate_off
    initial if (ADDR_WIDTH != 16)
        $fatal(1, "P5-A: fixed Reader combination requires ADDR_WIDTH=16");
    always @(posedge clk) if (rst_n) begin
        if ((fmap_wr_en || weight_wr_en || start) && cfg_mode == 2'd3)
            $fatal(1,"P5-A: cfg_mode=3 is reserved");
        if (fmap_wr_en && weight_wr_en)
            $fatal(1,"P5-A: simultaneous host preload ports unsupported in TB");
        if (busy && (fmap_wr_en || weight_wr_en || start))
            $fatal(1,"P5-A: preload/start while busy");
        if (busy && cfg_mode != active_mode)
            $fatal(1,"P5-A: cfg_mode must remain constant until done");
    end
    // synthesis translate_on
endmodule
