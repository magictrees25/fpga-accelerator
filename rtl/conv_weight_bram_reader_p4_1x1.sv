`timescale 1ns / 1ps

// P4-B: K=64 packed INT8 weight reader for ResNet-18 layer2 downsample.
// One N tile (8 output channels) per run. Host loads one 64-bit word per K.
// Even/odd K stored in separate 64-bit synchronous BRAM banks (32 words each).
// A legal pair request (K=0,2,...,62) produces two 64-bit words after the
// next rising edge. Interface matches conv_weight_bram_reader_p2_opt.
module conv_weight_bram_reader_p4_1x1 #(
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
    localparam int K_LEN=64;
    localparam int BANK_DEPTH=K_LEN/2;
    logic [ADDR_WIDTH-1:0] bank_rd_addr[0:1];
    logic [ADDR_WIDTH-1:0] bank_wr_addr;
    logic [1:0] bank_rd_en, bank_wr_en;
    logic [63:0] bank_q[0:1];
    logic request_legal, pair_accept;
    // These diagnostic tags match the existing P2.1 weight-reader interface.
    logic [ADDR_WIDTH-1:0] rsp_k_base, rsp_n_base;

    assign bank_wr_addr = wr_addr >> 1;
    assign request_legal = (n_base == '0) &&
                           !(|fetch_k_base[ADDR_WIDTH-1:6]) &&
                           !fetch_k_base[0];
    assign pair_accept = rst_n && pair_req_en && request_legal && !wr_en;

    for (genvar p=0; p<2; p=p+1) begin : gen_bank
        assign bank_rd_addr[p] = fetch_k_base >> 1;
        assign bank_rd_en[p] = pair_accept;
        assign bank_wr_en[p] = wr_en &&
                               (int'($unsigned(wr_addr)) < K_LEN) &&
                               (wr_addr[0] == 1'(p));
        sync_weight_bram #(
            .DEPTH(BANK_DEPTH),
            .WORD_WIDTH(64),
            .ADDR_WIDTH(ADDR_WIDTH)
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
            wgt_rsp_valid <= 2'b00;
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
        if (ADDR_WIDTH < 7 || ADDR_WIDTH > 30)
            $fatal(1,"P4 Weight Reader: invalid ADDR_WIDTH");
    end
    always @(posedge clk) begin
        if (wr_en && ($isunknown(wr_addr) ||
                      int'($unsigned(wr_addr)) >= K_LEN))
            $fatal(1,"P4 Weight Reader: illegal preload address %0d",wr_addr);
        if (rst_n && pair_req_en) begin
            if ($isunknown({n_base,fetch_k_base,wr_en}) ||
                request_legal !== 1'b1 || wr_en)
                $fatal(1,"P4 Weight Reader: illegal request N=%0d K=%0d",
                       n_base,fetch_k_base);
        end
    end
    // synthesis translate_on
endmodule
