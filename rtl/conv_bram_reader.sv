`timescale 1ns / 1ps

// One-cycle look-ahead activation reader for the EXISTING no-stall K controller.
// Fixed one-clock synchronous BRAM read latency.
// S_CLEAR: prefetch K0.
// S_FEED(Kk): consume Kk, while prefetching K(k+1) for next FEED beat.
// No changes to k_tile_controller / gemm_mn_controller are needed in this test.
// NOTE: valid for a continuous, one-K-per-clock FEED schedule; no backpressure.
module conv_bram_reader #(
    parameter int H_IN       = 5,
    parameter int W_IN       = 5,
    parameter int C_IN       = 3,
    parameter int KH         = 3,
    parameter int KW         = 3,
    parameter int STRIDE     = 1,
    parameter int PADDING    = 0,
    parameter int ADDR_WIDTH = 16,
    parameter int DATA_WIDTH = 8
) (
    input  logic clk,
    input  logic rst_n,
    input  logic [ADDR_WIDTH-1:0] m_addr,
    input  logic [ADDR_WIDTH-1:0] k_addr,
    input  logic clear_acc,                 // high in S_CLEAR
    input  logic feed_valid,                // high in S_FEED
    input  logic wr_en,                     // feature-map preload port
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [DATA_WIDTH-1:0] wr_data,
    output logic signed [DATA_WIDTH-1:0] act_data,
    // Debug outputs
    output logic [ADDR_WIDTH-1:0] fetch_k_addr,
    output logic [ADDR_WIDTH-1:0] fmap_addr,
    output logic addr_valid,
    output logic data_valid
);
    localparam int K_LEN = KH * KW * C_IN;
    localparam int DEPTH = H_IN * W_IN * C_IN;

    logic [DATA_WIDTH-1:0] ram_q;
    logic rd_en;
    logic rd_valid;

    // Look ahead by one beat, except on the last K of each dot product.
    always_comb begin
        fetch_k_addr = k_addr;
        if (feed_valid && (int'(k_addr) < K_LEN - 1))
            fetch_k_addr = k_addr + 1'b1;
    end

    conv_addr_gen #(
        .H_IN(H_IN), .W_IN(W_IN), .C_IN(C_IN),
        .KH(KH), .KW(KW), .STRIDE(STRIDE),
        .PADDING(PADDING), .ADDR_WIDTH(ADDR_WIDTH)
    ) u_addr (
        .m_addr(m_addr), .k_addr(fetch_k_addr),
        .fmap_addr(fmap_addr), .addr_valid(addr_valid)
    );

    // Read even for padding: addr generator returns 0; registered valid
    // then forces activation to 0 for invalid/padded positions.
    assign rd_en = clear_acc || feed_valid;

    sync_fmap_bram #(
        .DEPTH(DEPTH), .DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)
    ) u_mem (
        .clk(clk), .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data),
        .rd_en(rd_en), .rd_addr(fmap_addr), .rd_data(ram_q)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_valid <= 1'b0;
        else
            rd_valid <= rd_en && addr_valid;
    end

    assign data_valid = rd_valid;
    assign act_data   = rd_valid ? $signed(ram_q) : '0;
endmodule
