`timescale 1ns / 1ps

// P5-B2-3: one shared Activation BRAM for all three convolution modes.
// P5-B1 compute datapath is preserved: one 8x8 INT8 array, one K-M-N
// controller, and one shared weight BRAM reader.
//
// Mode 0: 3x3/S1/P1, P3 four-row Line Buffer reader, M=64, K=576.
// Mode 1: 1x1/S2/P0, P4 direct reader, M=16, K=64.
// Mode 2: 3x3/S2/P1, P4 direct reader, M=16, K=576.
//
// The three Activation Readers are instantiated with EXTERNAL_ACT_MEM=1.
// Exactly one physical Activation BRAM (two 8-bit banks) is instantiated
// below. The selected Reader's read request is muxed to that BRAM.
// Host activation writes are routed directly to the shared BRAM. P3 also
// receives the same write stream to initialize its private four-row buffer.

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
    logic [1:0] av_s1, av_1x1, av_s2;

    logic req_s1, req_1x1, req_s2;
    logic wr_s1, wr_1x1, wr_s2;

    // Weight request mode indicators retained for the existing regression TB.
    logic wreq_3x3, wreq_1x1;

    // Per-reader shared Activation BRAM interfaces.
    logic fmap_rd_en_s1, fmap_rd_en_1x1, fmap_rd_en_s2;
    logic [ADDR_WIDTH-1:0] fmap_rd_addr_s1;
    logic [ADDR_WIDTH-1:0] fmap_rd_addr_1x1;
    logic [ADDR_WIDTH-1:0] fmap_rd_addr_s2;

    // Single physical shared Activation BRAM interface.
    logic shared_fmap_rd_en;
    logic [ADDR_WIDTH-1:0] shared_fmap_rd_addr;
    logic [15:0] shared_fmap_rd_data;
    logic shared_fmap_rd_valid;

    assign cfg_m_len = (cfg_mode == MODE_3X3_S1)
                     ? ADDR_WIDTH'(64) : ADDR_WIDTH'(16);
    assign cfg_k_len = (cfg_mode == MODE_1X1_S2)
                     ? ADDR_WIDTH'(64) : ADDR_WIDTH'(576);

    // Capture the selected mode at start. cfg_mode must remain stable
    // through preload/start/compute as in the P5-B1 implementation.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            active_mode <= MODE_3X3_S1;
        else if (start && !busy && !done && cfg_mode != 2'd3)
            active_mode <= cfg_mode;
    end

    gemm_mn_controller_mem_p2 #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_ctrl (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .m_len(cfg_m_len),
        .n_len(ADDR_WIDTH'(8)),
        .k_len(cfg_k_len),
        .m_addr(m_addr),
        .n_base(n_base),
        .k_base(k_base),
        .fetch_k_base(fetch_k_base),
        .pair_req_en(pair_req_en),
        .act_pair_packed(act_pair_packed),
        .act_rsp_valid(act_rsp_valid),
        .wgt_pair_packed(wgt_pair_packed),
        .wgt_rsp_valid(wgt_rsp_valid),
        .mem_clear_acc(clear_acc),
        .mem_feed_valid(feed_valid),
        .write_valid(write_valid),
        .write_addr(write_addr),
        .write_m_addr(write_m_addr),
        .write_n_base(write_n_base),
        .write_mask(write_mask),
        .write_data(write_data),
        .busy(busy),
        .done(done),
        .any_overflow(any_overflow)
    );

    // Host writes are directed to the selected Reader. P3 needs the write
    // stream locally to fill its Line Buffer's initial four rows; P4 readers
    // use wr_en only to block a compute request during host preload.
    assign wr_s1  = fmap_wr_en && (cfg_mode == MODE_3X3_S1);
    assign wr_1x1 = fmap_wr_en && (cfg_mode == MODE_1X1_S2);
    assign wr_s2  = fmap_wr_en && (cfg_mode == MODE_3X3_S2);

    assign req_s1  = pair_req_en && (active_mode == MODE_3X3_S1);
    assign req_1x1 = pair_req_en && (active_mode == MODE_1X1_S2);
    assign req_s2  = pair_req_en && (active_mode == MODE_3X3_S2);

    // P3: retains its four-row Line Buffer; only its backing fmap BRAM is
    // externalized to the shared Activation BRAM.
    conv_bram_reader_p3 #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .EXTERNAL_ACT_MEM(1'b1)
    ) u_act_s1 (
        .clk(clk),
        .rst_n(rst_n),
        .m_addr(m_addr),
        .pair_req_en(req_s1),
        .fetch_k_base(fetch_k_base),
        .wr_en(wr_s1),
        .wr_addr(fmap_wr_addr),
        .wr_data(fmap_wr_data),
        .fmap_rd_data(shared_fmap_rd_data),
        .fmap_rd_valid(shared_fmap_rd_valid),
        .fmap_rd_en(fmap_rd_en_s1),
        .fmap_rd_addr(fmap_rd_addr_s1),
        .act_pair_packed(act_s1),
        .act_rsp_valid(av_s1)
    );

    // P4 1x1/S2 direct reader.
    conv_bram_reader_p4_1x1 #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .EXTERNAL_ACT_MEM(1'b1)
    ) u_act_1x1 (
        .clk(clk),
        .rst_n(rst_n),
        .m_addr(m_addr),
        .pair_req_en(req_1x1),
        .fetch_k_base(fetch_k_base),
        .wr_en(wr_1x1),
        .wr_addr(fmap_wr_addr),
        .wr_data(fmap_wr_data),
        .fmap_rd_data(shared_fmap_rd_data),
        .fmap_rd_en(fmap_rd_en_1x1),
        .fmap_rd_addr(fmap_rd_addr_1x1),
        .act_pair_packed(act_1x1),
        .act_rsp_valid(av_1x1)
    );

    // P4 3x3/S2 direct reader.
    conv_bram_reader_p4_3x3s2 #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .EXTERNAL_ACT_MEM(1'b1)
    ) u_act_s2 (
        .clk(clk),
        .rst_n(rst_n),
        .m_addr(m_addr),
        .pair_req_en(req_s2),
        .fetch_k_base(fetch_k_base),
        .wr_en(wr_s2),
        .wr_addr(fmap_wr_addr),
        .wr_data(fmap_wr_data),
        .fmap_rd_data(shared_fmap_rd_data),
        .fmap_rd_en(fmap_rd_en_s2),
        .fmap_rd_addr(fmap_rd_addr_s2),
        .act_pair_packed(act_s2),
        .act_rsp_valid(av_s2)
    );

    // Select the actual BRAM request from the active convolution mode.
    // In P3 mode this is the row-prefetch request (not the Line Buffer read).
    always_comb begin
        shared_fmap_rd_en   = 1'b0;
        shared_fmap_rd_addr = '0;
        case (active_mode)
            MODE_3X3_S1: begin
                shared_fmap_rd_en   = fmap_rd_en_s1;
                shared_fmap_rd_addr = fmap_rd_addr_s1;
            end
            MODE_1X1_S2: begin
                shared_fmap_rd_en   = fmap_rd_en_1x1;
                shared_fmap_rd_addr = fmap_rd_addr_1x1;
            end
            MODE_3X3_S2: begin
                shared_fmap_rd_en   = fmap_rd_en_s2;
                shared_fmap_rd_addr = fmap_rd_addr_s2;
            end
            default: begin
                shared_fmap_rd_en   = 1'b0;
                shared_fmap_rd_addr = '0;
            end
        endcase
    end

    // The only Activation BRAM instance in the multimode compute top.
    // Byte-addressed host writes; pair-addressed synchronous reads.
    conv_activation_bram_shared_p5 #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .MAX_BYTES(6400)
    ) u_activation_shared (
        .clk(clk),
        .rst_n(rst_n),
        .wr_en(fmap_wr_en),
        .wr_addr(fmap_wr_addr),
        .wr_data(fmap_wr_data),
        .rd_en(shared_fmap_rd_en),
        .rd_addr(shared_fmap_rd_addr),
        .rd_data(shared_fmap_rd_data),
        .rd_valid(shared_fmap_rd_valid)
    );

    // P5-B1: one shared weight BRAM reader for all convolution modes.
    // K=64 for 1x1/S2; K=576 for both 3x3 modes.
    assign wreq_1x1 = pair_req_en && (active_mode == MODE_1X1_S2);
    assign wreq_3x3 = pair_req_en && (active_mode != MODE_1X1_S2);

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

    // Select the active Reader's activation response for the unchanged GEMM
    // controller / PE-array datapath.
    always_comb begin
        case (active_mode)
            MODE_3X3_S1: begin
                act_pair_packed = act_s1;
                act_rsp_valid   = av_s1;
            end
            MODE_1X1_S2: begin
                act_pair_packed = act_1x1;
                act_rsp_valid   = av_1x1;
            end
            MODE_3X3_S2: begin
                act_pair_packed = act_s2;
                act_rsp_valid   = av_s2;
            end
            default: begin
                act_pair_packed = '0;
                act_rsp_valid   = '0;
            end
        endcase
    end

    // Simulation-only protocol checks.
    // synthesis translate_off
    initial begin
        if (ADDR_WIDTH != 16)
            $fatal(1, "P5-B2: fixed Reader combination requires ADDR_WIDTH=16");
    end

    always @(posedge clk) begin
        if (rst_n) begin
            if ((fmap_wr_en || weight_wr_en || start) && cfg_mode == 2'd3)
                $fatal(1, "P5-B2: cfg_mode=3 is reserved");
            if (fmap_wr_en && weight_wr_en)
                $fatal(1, "P5-B2: simultaneous host preload ports unsupported in TB");
            if (busy && (fmap_wr_en || weight_wr_en || start))
                $fatal(1, "P5-B2: preload/start while busy");
            if (busy && cfg_mode != active_mode)
                $fatal(1, "P5-B2: cfg_mode must remain constant until done");
            if (fmap_wr_en && shared_fmap_rd_en)
                $fatal(1, "P5-B2: shared Activation BRAM read/write overlap");
            if ((int'(fmap_rd_en_s1) + int'(fmap_rd_en_1x1) +
                 int'(fmap_rd_en_s2)) > 1)
                $fatal(1, "P5-B2: more than one Activation Reader requested BRAM");
        end
    end
    // synthesis translate_on

endmodule
