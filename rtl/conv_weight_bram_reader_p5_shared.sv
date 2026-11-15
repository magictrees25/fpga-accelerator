
`timescale 1ns / 1ps

// P5-B1: Shared Weight BRAM Reader
//
// One unified even/odd bank memory supports:
//   3x3 convolution: K_LEN = 576
//   1x1 convolution: K_LEN = 64
//
// Memory organization:
//   Bank 0: even K (K0, K2, K4, ...)
//   Bank 1: odd  K (K1, K3, K5, ...)
//
// Each bank:
//   DEPTH = 288
//   WORD_WIDTH = 64 bits
//
// One pair request returns two consecutive K words.
// Interface is compatible with P5-A weight reader outputs.

module conv_weight_bram_reader_p5_shared #(
    parameter int ADDR_WIDTH = 16,
    parameter int K_MAX      = 576
)(
    input  logic clk,
    input  logic rst_n,

    input  logic [ADDR_WIDTH-1:0] n_base,
    input  logic [ADDR_WIDTH-1:0] k_len,

    input  logic pair_req_en,
    input  logic [ADDR_WIDTH-1:0] fetch_k_base,

    input  logic wr_en,
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [63:0] wr_data,

    output logic [127:0] wgt_pair_packed,
    output logic [1:0] wgt_rsp_valid
);

    localparam int BANK_DEPTH = K_MAX / 2;

    logic [ADDR_WIDTH-1:0] bank_rd_addr [0:1];
    logic [ADDR_WIDTH-1:0] bank_wr_addr;

    logic [1:0] bank_rd_en;
    logic [1:0] bank_wr_en;

    logic [63:0] bank_q [0:1];

    logic wr_addr_in_range;
    logic request_legal;
    logic pair_accept;

    // ------------------------------------------------
    // Address mapping
    // ------------------------------------------------

    assign bank_wr_addr = wr_addr >> 1;

    assign wr_addr_in_range =
        (int'($unsigned(wr_addr)) <
         int'($unsigned(k_len))) &&
        (int'($unsigned(wr_addr)) < K_MAX);

    // One request reads K and K+1.
    // K must be even and both addresses must be valid.
    assign request_legal =
        (n_base == '0) &&
        !fetch_k_base[0] &&
        ((int'($unsigned(fetch_k_base)) + 1) <
         int'($unsigned(k_len))) &&
        (int'($unsigned(fetch_k_base)) < (K_MAX - 1));

    assign pair_accept =
        rst_n && pair_req_en && request_legal && !wr_en;

    // ------------------------------------------------
    // Two-bank BRAM
    // ------------------------------------------------

    for (genvar p = 0; p < 2; p = p + 1) begin : gen_bank

        assign bank_rd_addr[p] = fetch_k_base >> 1;

        assign bank_rd_en[p] = pair_accept;

        assign bank_wr_en[p] =
            wr_en &&
            wr_addr_in_range &&
            (wr_addr[0] == 1'(p));

        sync_weight_bram #(
            .DEPTH(BANK_DEPTH),
            .WORD_WIDTH(64),
            .ADDR_WIDTH(ADDR_WIDTH)
        ) u_mem (
            .clk(clk),

            .wr_en(bank_wr_en[p]),
            .wr_addr(bank_wr_addr),
            .wr_data(wr_data),

            .rd_en(bank_rd_en[p]),
            .rd_addr(bank_rd_addr[p]),
            .rd_data(bank_q[p])
        );

        assign wgt_pair_packed[p*64 +: 64] =
            wgt_rsp_valid[p] ? bank_q[p] : 64'b0;

    end

    // ------------------------------------------------
    // Response valid
    // ------------------------------------------------

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wgt_rsp_valid <= 2'b00;
        end
        else begin
            wgt_rsp_valid <= {2{pair_accept}};
        end
    end

    // ------------------------------------------------
    // Simulation checks
    // ------------------------------------------------

    // synthesis translate_off

    initial begin
        if (K_MAX != 576)
            $fatal(1, "P5-B1: K_MAX must be 576");

        if ((K_MAX % 2) != 0 ||
            ADDR_WIDTH < $clog2(K_MAX))
            $fatal(1, "P5-B1: invalid memory parameters");
    end

    always @(posedge clk) begin
        if (rst_n) begin

            if ((k_len != ADDR_WIDTH'(64)) &&
                (k_len != ADDR_WIDTH'(576)))
                $fatal(1, "P5-B1: invalid K length");

            if (wr_en) begin
                if ($isunknown({wr_addr, k_len}) ||
                    wr_addr_in_range !== 1'b1 ||
                    pair_req_en)
                    $fatal(1,
                        "P5-B1: invalid/overlapping weight write");
            end

            if (pair_req_en) begin
                if ($isunknown({
                    n_base, fetch_k_base, k_len, wr_en
                }) ||
                    request_legal !== 1'b1 ||
                    wr_en !== 1'b0)
                    $fatal(1,
                        "P5-B1: illegal weight request N=%0d K=%0d",
                        n_base, fetch_k_base);
            end

        end
    end

    // synthesis translate_on

endmodule
