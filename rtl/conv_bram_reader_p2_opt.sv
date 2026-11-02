`timescale 1ns / 1ps

// Address-optimized fixed P2 Tile Reader. RAM layout and one-clock response
// are unchanged. Both banks share one address, computed by two small LUT
// decoders and narrow additions. No generic division/modulo/multiplication.
// Legal requests may arrive in ANY order; no implicit stream counter.
module conv_bram_reader_p2_opt #(
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
    localparam int DEPTH = H_IN*W_IN*C_IN;
    localparam int BANK_DEPTH = DEPTH/2;

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

    logic [6:0] row_pixels, window_pixel, pixel_index;
    logic [4:0] kernel_pixel_offset;
    logic kernel_valid;
    logic [11:0] shared_bank_addr;

    // m=oy*8+ox. Table computes 10*oy, with no multiplier expression.
    always_comb begin
        case (m_addr[5:3])
            3'd0: row_pixels=7'd0;
            3'd1: row_pixels=7'd10;
            3'd2: row_pixels=7'd20;
            3'd3: row_pixels=7'd30;
            3'd4: row_pixels=7'd40;
            3'd5: row_pixels=7'd50;
            3'd6: row_pixels=7'd60;
            3'd7: row_pixels=7'd70;
            default: row_pixels=7'd0;
        endcase
    end

    // k=(ky*3+kx)*64+ci. Decode nine kernel pixels to ky*10+kx.
    always_comb begin
        kernel_valid=1'b1;
        case (fetch_k_base[9:6])
            4'd0: kernel_pixel_offset=5'd0;
            4'd1: kernel_pixel_offset=5'd1;
            4'd2: kernel_pixel_offset=5'd2;
            4'd3: kernel_pixel_offset=5'd10;
            4'd4: kernel_pixel_offset=5'd11;
            4'd5: kernel_pixel_offset=5'd12;
            4'd6: kernel_pixel_offset=5'd20;
            4'd7: kernel_pixel_offset=5'd21;
            4'd8: kernel_pixel_offset=5'd22;
            default: begin
                kernel_pixel_offset=5'd0;
                kernel_valid=1'b0;
            end
        endcase
    end

    // 0<=window_pixel<=77; 0<=pixel_index<=99. Channel-pair bits are
    // appended, avoiding a carry chain through the low five address bits.
    assign window_pixel = row_pixels + {4'b0000,m_addr[2:0]};
    assign pixel_index = window_pixel + {2'b00,kernel_pixel_offset};
    assign shared_bank_addr = {pixel_index,fetch_k_base[5:1]};

    // Range checks are parallel to address arithmetic, not address muxes.
    assign request_legal = !(|m_addr[ADDR_WIDTH-1:6]) &&
                           !(|fetch_k_base[ADDR_WIDTH-1:10]) &&
                           !fetch_k_base[0] && kernel_valid;
    assign pair_accept = rst_n && pair_req_en && request_legal && !wr_en;

    // Observation names preserved for independent regression checks.
    // A legal k is even: k+1 is formed by setting bit 0, no incrementer.
    assign lane_k[0] = fetch_k_base;
    assign lane_k[1] = {fetch_k_base[ADDR_WIDTH-1:1],1'b1};
    assign addr_valid = {2{request_legal}};
    assign bank_wr_addr = wr_addr >> 1;

    for (genvar p=0; p<2; p=p+1) begin : gen_bank
        assign bank_rd_addr[p] = ADDR_WIDTH'(shared_bank_addr);
        assign fmap_addr[p] = ADDR_WIDTH'({shared_bank_addr,(p == 1)});
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
            $fatal(1, "conv_bram_reader_p2_opt: step 1 supports only 10x10x64, 3x3, S1, local P0");
        if (ADDR_WIDTH < 13 || ADDR_WIDTH > 30)
            $fatal(1, "conv_bram_reader_p2_opt: ADDR_WIDTH must be 13..30");
    end

    always @(posedge clk) begin
        if (wr_en === 1'b1 &&
            ($isunknown(wr_addr) || int'($unsigned(wr_addr)) >= DEPTH))
            $fatal(1, "conv_bram_reader_p2_opt: invalid write address");
        if (rst_n && pair_req_en === 1'b1) begin
            if ($isunknown({m_addr, fetch_k_base, wr_en}) ||
                request_legal !== 1'b1 || wr_en !== 1'b0)
                $fatal(1, "conv_bram_reader_p2_opt: illegal/overlapping request M=%0d K=%0d", m_addr, fetch_k_base);
            if (addr_valid !== 2'b11 ||
                fmap_addr[0][0] !== 1'b0 || fmap_addr[1][0] !== 1'b1)
                $fatal(1, "conv_bram_reader_p2_opt: address validity or bank parity failure");
        end
    end
    // synthesis translate_on
endmodule


