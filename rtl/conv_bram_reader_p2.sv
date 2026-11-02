`timescale 1ns / 1ps

// P=2 step 1: fixed layer1 local-Tile activation reader.
// Original HWC byte write interface; two 3200x8 banks.
// A request sampled at E updates data/valid after E; the consumer uses
// that registered response at E+1. No extra response data register.
// This first version supports ONLY the geometry checked below.
module conv_bram_reader_p2 #(
    parameter int H_IN = 10,
    parameter int W_IN = 10,
    parameter int C_IN = 64,
    parameter int KH = 3,
    parameter int KW = 3,
    parameter int STRIDE = 1,
    parameter int PADDING = 0,
    parameter int ADDR_WIDTH = 16
)(
    input  logic clk,
    input  logic rst_n,
    input  logic [ADDR_WIDTH-1:0] m_addr,
    input  logic pair_req_en,
    input  logic [ADDR_WIDTH-1:0] fetch_k_base,
    input  logic wr_en,
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [7:0] wr_data,
    output logic [15:0] act_pair_packed,
    output logic [1:0] act_rsp_valid
);
    localparam int K_LEN = KH*KW*C_IN;
    localparam int DEPTH = H_IN*W_IN*C_IN;
    localparam int BANK_DEPTH = DEPTH/2;
    localparam int M_LEN = 64;

    // Named internal observation signals; not additional top-level pins.
    logic [ADDR_WIDTH-1:0] lane_k [0:1];
    logic [ADDR_WIDTH-1:0] fmap_addr [0:1];
    logic [1:0] addr_valid;
    logic [ADDR_WIDTH-1:0] bank_rd_addr [0:1];
    logic [ADDR_WIDTH-1:0] bank_wr_addr;
    logic [1:0] bank_rd_en, bank_wr_en;
    logic [7:0] bank_q [0:1];
    logic request_legal, pair_accept;
    logic [ADDR_WIDTH-1:0] rsp_k_base, rsp_m_addr;

    assign lane_k[0] = fetch_k_base;
    assign lane_k[1] = fetch_k_base + ADDR_WIDTH'(1);
    assign bank_wr_addr = wr_addr >> 1;
    assign request_legal = !fetch_k_base[0] &&
                           (int'($unsigned(fetch_k_base)) < K_LEN-1) &&
                           (int'($unsigned(m_addr)) < M_LEN);
    // Invalid requests must be diagnosed in simulation, never read out of range.
    assign pair_accept = rst_n && pair_req_en && request_legal && !wr_en &&
                         (&addr_valid) &&
                         !fmap_addr[0][0] && fmap_addr[1][0];

    for (genvar p=0; p<2; p=p+1) begin : gen_bank
        conv_addr_gen #(
            .H_IN(H_IN), .W_IN(W_IN), .C_IN(C_IN),
            .KH(KH), .KW(KW), .STRIDE(STRIDE), .PADDING(PADDING),
            .ADDR_WIDTH(ADDR_WIDTH)
        ) u_addr (
            .m_addr(m_addr), .k_addr(lane_k[p]),
            .fmap_addr(fmap_addr[p]), .addr_valid(addr_valid[p])
        );

        assign bank_rd_addr[p] = fmap_addr[p] >> 1;
        assign bank_rd_en[p] = pair_accept;
        // Preload is permitted while reset is asserted, as in the baseline.
        assign bank_wr_en[p] = wr_en &&
                              (int'($unsigned(wr_addr)) < DEPTH) &&
                              (wr_addr[0] == (p == 1));

        sync_fmap_bram #(
            .DEPTH(BANK_DEPTH), .DATA_WIDTH(8), .ADDR_WIDTH(ADDR_WIDTH)
        ) u_mem (
            .clk(clk),
            .wr_en(bank_wr_en[p]), .wr_addr(bank_wr_addr), .wr_data(wr_data),
            .rd_en(bank_rd_en[p]), .rd_addr(bank_rd_addr[p]), .rd_data(bank_q[p])
        );

        // Packed bits preserve signed INT8 representation; consumers must
        // use $signed(act_pair_packed[p*8 +: 8]) for arithmetic.
        assign act_pair_packed[p*8 +: 8] =
            act_rsp_valid[p] ? bank_q[p] : 8'b0;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            act_rsp_valid <= '0;
            rsp_k_base <= '0;
            rsp_m_addr <= '0;
        end else begin
            act_rsp_valid <= {2{pair_accept}};
            rsp_k_base <= pair_accept ? fetch_k_base : '0;
            rsp_m_addr <= pair_accept ? m_addr : '0;
        end
    end

    // synthesis translate_off
    initial begin
        if (H_IN != 10 || W_IN != 10 || C_IN != 64 ||
            KH != 3 || KW != 3 || STRIDE != 1 || PADDING != 0)
            $fatal(1, "conv_bram_reader_p2: step 1 supports only 10x10x64, 3x3, S1, local P0");
        if (ADDR_WIDTH < 13 || ADDR_WIDTH > 30)
            $fatal(1, "conv_bram_reader_p2: ADDR_WIDTH must be 13..30");
    end

    always @(posedge clk) begin
        if (wr_en === 1'b1 &&
            ($isunknown(wr_addr) || int'($unsigned(wr_addr)) >= DEPTH))
            $fatal(1, "conv_bram_reader_p2: invalid write address");
        if (rst_n && pair_req_en === 1'b1) begin
            if ($isunknown({m_addr, fetch_k_base, wr_en}) ||
                request_legal !== 1'b1 || wr_en !== 1'b0)
                $fatal(1, "conv_bram_reader_p2: illegal/overlapping request M=%0d K=%0d", m_addr, fetch_k_base);
            if (addr_valid !== 2'b11 ||
                fmap_addr[0][0] !== 1'b0 || fmap_addr[1][0] !== 1'b1)
                $fatal(1, "conv_bram_reader_p2: address validity or bank parity failure");
        end
    end
    // synthesis translate_on
endmodule

