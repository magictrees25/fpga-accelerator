`timescale 1ns / 1ps

// P=2 step 1: fixed K576/N_TILES1 packed-weight reader.
// Original 64-bit word writes: word k contains B[k,0..7].
// Bank0 contains even k, Bank1 odd k, each 288x64.
// pair p occupies bits p*64 +: 64, channel n occupies n*8 +: 8.
module conv_weight_bram_reader_p2_opt #(
    parameter int K_LEN = 576,
    parameter int N_TILES = 1,
    parameter int ADDR_WIDTH = 16
)(
    input  logic clk,
    input  logic rst_n,
    input  logic [ADDR_WIDTH-1:0] n_base,
    input  logic pair_req_en,
    input  logic [ADDR_WIDTH-1:0] fetch_k_base,
    input  logic wr_en,
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [63:0] wr_data,
    output logic [127:0] wgt_pair_packed,
    output logic [1:0] wgt_rsp_valid
);
    localparam int BANK_DEPTH = K_LEN/2;
    logic [ADDR_WIDTH-1:0] bank_rd_addr [0:1];
    logic [ADDR_WIDTH-1:0] bank_wr_addr;
    logic [1:0] bank_rd_en, bank_wr_en;
    logic [63:0] bank_q [0:1];
    logic request_legal, pair_accept;
    logic [ADDR_WIDTH-1:0] rsp_k_base, rsp_n_base;

    assign bank_wr_addr = wr_addr >> 1;
    assign request_legal = (n_base == 0) && !fetch_k_base[0] &&
                           (int'($unsigned(fetch_k_base)) < K_LEN-1);
    assign pair_accept = rst_n && pair_req_en && request_legal && !wr_en;

    for (genvar p=0; p<2; p=p+1) begin : gen_bank
        assign bank_rd_addr[p] = fetch_k_base >> 1;
        assign bank_rd_en[p] = pair_accept;
        assign bank_wr_en[p] = wr_en &&
                              (int'($unsigned(wr_addr)) < K_LEN) &&
                              (wr_addr[0] == (p == 1));

        sync_weight_bram #(
            .DEPTH(BANK_DEPTH), .WORD_WIDTH(64), .ADDR_WIDTH(ADDR_WIDTH)
        ) u_mem (
            .clk(clk),
            .wr_en(bank_wr_en[p]), .wr_addr(bank_wr_addr), .wr_data(wr_data),
            .rd_en(bank_rd_en[p]), .rd_addr(bank_rd_addr[p]), .rd_data(bank_q[p])
        );

        assign wgt_pair_packed[p*64 +: 64] =
            wgt_rsp_valid[p] ? bank_q[p] : 64'b0;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wgt_rsp_valid <= '0;
            rsp_k_base <= '0;
            rsp_n_base <= '0;
        end else begin
            wgt_rsp_valid <= {2{pair_accept}};
            rsp_k_base <= pair_accept ? fetch_k_base : '0;
            rsp_n_base <= pair_accept ? n_base : '0;
        end
    end

    // synthesis translate_off
    initial begin
        if (K_LEN != 576 || N_TILES != 1)
            $fatal(1, "conv_weight_bram_reader_p2: step 1 supports only K576/N_TILES1");
        if (ADDR_WIDTH < 10 || ADDR_WIDTH > 30)
            $fatal(1, "conv_weight_bram_reader_p2: ADDR_WIDTH must be 10..30");
    end

    always @(posedge clk) begin
        if (wr_en === 1'b1 &&
            ($isunknown(wr_addr) || int'($unsigned(wr_addr)) >= K_LEN))
            $fatal(1, "conv_weight_bram_reader_p2: invalid write address");
        if (rst_n && pair_req_en === 1'b1) begin
            if ($isunknown({n_base, fetch_k_base, wr_en}) ||
                request_legal !== 1'b1 || wr_en !== 1'b0)
                $fatal(1, "conv_weight_bram_reader_p2: illegal/overlapping request N=%0d K=%0d", n_base, fetch_k_base);
        end
    end
    // synthesis translate_on
endmodule

