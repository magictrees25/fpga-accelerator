`timescale 1ns / 1ps

// P4-A: Direct BRAM activation reader for ResNet-18 layer2.0.downsample.0.
// Fixed LOCAL tile: input 8x8x64 (HWC), 1x1/S2/P0, output 4x4x8.
// GEMM per tile: M=16, N=8, K=64; two adjacent signed INT8 activations
// per request, returned together after ONE rising clock edge.
//
// Packed activation request:
//   m = oy*4 + ox, oy/ox in 0..3
//   k = channel, even in 0,2,...,62
//   in_y = 2*oy, in_x = 2*ox
//   byte_addr = ((in_y*8 + in_x)*64 + k)
//   pair_addr = byte_addr/2 = 512*oy + 64*ox + k/2
//
// All dimension constants are deliberately fixed for P4-A, not a generic
// ResNet-18 mapping yet. Host/TB schedules spatial and output-channel tiles.
module conv_bram_reader_p4_1x1 #(
    parameter int ADDR_WIDTH = 16
)(
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic [ADDR_WIDTH-1:0] m_addr,
    input  logic                  pair_req_en,
    input  logic [ADDR_WIDTH-1:0] fetch_k_base,
    input  logic                  wr_en,
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [7:0]            wr_data,
    output logic [15:0]           act_pair_packed,
    output logic [1:0]            act_rsp_valid
);
    localparam int H_IN=8, W_IN=8, C_IN=64;
    localparam int M_LEN=16, K_LEN=64;
    localparam int DEPTH=H_IN*W_IN*C_IN; // 4096 bytes
    localparam int BANK_DEPTH=DEPTH/2;   // 2048 bytes per bank

    logic [ADDR_WIDTH-1:0] bank_wr_addr;
    logic [ADDR_WIDTH-1:0] bank_rd_addr [0:1];
    logic [1:0] bank_wr_en, bank_rd_en;
    logic [7:0] bank_q [0:1];
    logic request_legal, pair_accept;
    logic [ADDR_WIDTH-1:0] rsp_k_base, rsp_m_addr;

    // Pair address bits: [10:9]=oy, [8]=0, [7:6]=ox, [5]=0,
    // [4:0]=k/2.  This exactly implements 512*oy + 64*ox + k/2.
    logic [10:0] shared_bank_addr;
    assign shared_bank_addr = {
        m_addr[3:2], 1'b0, m_addr[1:0], 1'b0, fetch_k_base[5:1]
    };

    assign request_legal =
        !(|m_addr[ADDR_WIDTH-1:4]) &&
        !(|fetch_k_base[ADDR_WIDTH-1:6]) &&
        !fetch_k_base[0];
    assign pair_accept = rst_n && pair_req_en && request_legal && !wr_en;
    assign bank_wr_addr = wr_addr >> 1;

    for (genvar p=0; p<2; p=p+1) begin : gen_act_bank
        assign bank_rd_addr[p] = ADDR_WIDTH'(shared_bank_addr);
        assign bank_rd_en[p] = pair_accept;
        assign bank_wr_en[p] = wr_en &&
            (int'($unsigned(wr_addr)) < DEPTH) &&
            (wr_addr[0] == 1'(p));

        sync_fmap_bram #(
            .DEPTH(BANK_DEPTH),
            .DATA_WIDTH(8),
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

        // A response is meaningful only when its corresponding valid bit is 1.
        assign act_pair_packed[p*8 +: 8] =
            act_rsp_valid[p] ? bank_q[p] : 8'b0;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            act_rsp_valid <= 2'b00;
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
        if (ADDR_WIDTH < 12 || ADDR_WIDTH > 30)
            $fatal(1, "P4 1x1 Reader: ADDR_WIDTH must be 12..30");
    end

    always @(posedge clk) begin
        if (wr_en && ($isunknown(wr_addr) ||
                      int'($unsigned(wr_addr)) >= DEPTH))
            $fatal(1, "P4 1x1 Reader: invalid preload address");
        if (rst_n && pair_req_en) begin
            if ($isunknown({m_addr, fetch_k_base, wr_en}) ||
                !request_legal || wr_en)
                $fatal(1, "P4 1x1 Reader: illegal/overlapping request M=%0d K=%0d",
                       m_addr, fetch_k_base);
        end
    end
    // synthesis translate_on
endmodule
