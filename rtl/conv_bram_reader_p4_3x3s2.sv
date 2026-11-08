`timescale 1ns / 1ps

// P4-D1: Paired INT8 activation BRAM reader for ResNet-18 layer2.0.conv1.
// Global convolution: 3x3 / stride 2 / padding 1, Cin=64, Cout=128.
// This reader works on ONE *already halo-padded* local HWC patch:
//   input patch 9x9x64 (including one-pixel zero padding at global edges),
//   output tile 4x4x8, GEMM M=16, N=8, K=3*3*64=576.
// In local patch coordinates, the convolution is 3x3/S2/P0:
//   m = oy*4+ox, k = (ky*3+kx)*64 + ci
//   py = 2*oy+ky, px = 2*ox+kx, ci=0..63.
//   byte_addr=((py*9+px)*64+ci)
//   pair_addr=((py*9+px)*32+ci/2), k must be even.
// The host/TB must preload ZERO for halo coordinates outside global 56x56.
// This module does NOT auto-generate global padding and has no AXI/DDR port.
// One rising-edge BRAM read latency; no backpressure; one K-pair/clock.

module conv_bram_reader_p4_3x3s2 #(
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
    localparam int H_IN=9, W_IN=9, C_IN=64;
    localparam int DEPTH=H_IN*W_IN*C_IN; // 5184 bytes
    localparam int BANK_DEPTH=DEPTH/2;   // 2592 byte-pairs

    logic [3:0] kernel_pixel;
    logic [1:0] ky, kx;
    logic kernel_valid;
    logic [3:0] input_row, input_col; // 0..8
    logic [6:0] row_times_nine, pixel_index; // 0..80
    logic [11:0] shared_bank_addr;
    logic request_legal, pair_accept;

    // Decode 3x3 kernel position; channel-pair index is k[5:1].
    always_comb begin
        kernel_pixel = fetch_k_base[9:6];
        ky = 2'd0;
        kx = 2'd0;
        kernel_valid = 1'b1;
        case (kernel_pixel)
            4'd0: begin ky=0; kx=0; end
            4'd1: begin ky=0; kx=1; end
            4'd2: begin ky=0; kx=2; end
            4'd3: begin ky=1; kx=0; end
            4'd4: begin ky=1; kx=1; end
            4'd5: begin ky=1; kx=2; end
            4'd6: begin ky=2; kx=0; end
            4'd7: begin ky=2; kx=1; end
            4'd8: begin ky=2; kx=2; end
            default: begin ky=0; kx=0; kernel_valid=1'b0; end
        endcase
    end

    // m[3:2] = output y (0..3); m[1:0] = output x (0..3).
    // Multiplication by 9 uses shifts/adds; low 5 bits concatenate K-pair.
    assign input_row = {1'b0, m_addr[3:2], 1'b0} + {2'b00,ky};
    assign input_col = {1'b0, m_addr[1:0], 1'b0} + {2'b00,kx};
    assign row_times_nine = {input_row,3'b000} + {3'b000,input_row};
    assign pixel_index = row_times_nine + {3'b000,input_col};
    assign shared_bank_addr = {pixel_index, fetch_k_base[5:1]};

    assign request_legal =
        !(|m_addr[ADDR_WIDTH-1:4]) &&
        !(|fetch_k_base[ADDR_WIDTH-1:10]) &&
        !fetch_k_base[0] && kernel_valid &&
        (input_row <= 4'd8) && (input_col <= 4'd8);
    assign pair_accept = rst_n && pair_req_en && request_legal && !wr_en;

    logic [ADDR_WIDTH-1:0] bank_wr_addr;
    logic [ADDR_WIDTH-1:0] bank_rd_addr [0:1];
    logic [1:0] bank_wr_en, bank_rd_en;
    logic [7:0] bank_q [0:1];

    // Keep P2-style request/response debug names for existing TB conventions.
    logic [ADDR_WIDTH-1:0] rsp_k_base, rsp_m_addr;
    assign bank_wr_addr = wr_addr >> 1;

    for (genvar p=0; p<2; p++) begin : gen_bank
        assign bank_rd_addr[p] = ADDR_WIDTH'(shared_bank_addr);
        assign bank_rd_en[p] = pair_accept;
        assign bank_wr_en[p] = wr_en &&
            (int'($unsigned(wr_addr)) < DEPTH) &&
            (wr_addr[0] == 1'(p));

        sync_fmap_bram #(
            .DEPTH(BANK_DEPTH), .DATA_WIDTH(8), .ADDR_WIDTH(ADDR_WIDTH)
        ) u_mem (
            .clk(clk),
            .wr_en(bank_wr_en[p]), .wr_addr(bank_wr_addr), .wr_data(wr_data),
            .rd_en(bank_rd_en[p]), .rd_addr(bank_rd_addr[p]), .rd_data(bank_q[p])
        );
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
    initial if (ADDR_WIDTH < 13 || ADDR_WIDTH > 30)
        $fatal(1,"P4-D Reader requires ADDR_WIDTH 13..30");
    always @(posedge clk) begin
        if (wr_en && ($isunknown(wr_addr) ||
                      int'($unsigned(wr_addr)) >= DEPTH))
            $fatal(1,"P4-D Reader: illegal preload address");
        if (rst_n && pair_req_en &&
            ($isunknown({m_addr,fetch_k_base,wr_en}) ||
             !request_legal || wr_en))
            $fatal(1,"P4-D Reader: illegal/overlapping request M=%0d K=%0d",
                   m_addr,fetch_k_base);
    end
    // synthesis translate_on
endmodule
