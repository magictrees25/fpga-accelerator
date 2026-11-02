`timescale 1ns / 1ps

// Optimized, combinational conv input-address generator (HWC layout).
// Compatible with the existing conv_addr_gen port list.
// The common PADDING=0 case uses a flattened address identity so that
// kernel_x and channel do not need separate divider/remainder hardware.
// Other padding configurations retain exact boundary/padding behavior.
// IMPORTANT: replace the CONTENTS of the existing conv_addr_gen.sv;
// do not compile this file alongside another module named conv_addr_gen.
module conv_addr_gen #(
    parameter int H_IN       = 5,
    parameter int W_IN       = 5,
    parameter int C_IN       = 3,
    parameter int KH         = 3,
    parameter int KW         = 3,
    parameter int STRIDE     = 1,
    parameter int PADDING    = 0,
    parameter int ADDR_WIDTH = 16
) (
    input  logic [ADDR_WIDTH-1:0] m_addr,
    input  logic [ADDR_WIDTH-1:0] k_addr,
    output logic [ADDR_WIDTH-1:0] fmap_addr,
    output logic                  addr_valid
);
    localparam int H_OUT = (H_IN + 2*PADDING - KH) / STRIDE + 1;
    localparam int W_OUT = (W_IN + 2*PADDING - KW) / STRIDE + 1;
    localparam int M_LEN = H_OUT * W_OUT;
    localparam int K_LEN = KH * KW * C_IN;
    localparam int K_ROW = KW * C_IN;
    localparam int IN_ROW_STEP  = W_IN * C_IN;
    localparam int OUT_COL_STEP = STRIDE * C_IN;

    // +1 ensures constants such as W_OUT and K_ROW fit even when powers of 2.
    localparam int MW = (M_LEN > 0) ? (($clog2(M_LEN+1) > 0) ? $clog2(M_LEN+1) : 1) : 1;
    localparam int KWID = (K_LEN > 0) ? (($clog2(K_LEN+1) > 0) ? $clog2(K_LEN+1) : 1) : 1;

    logic [MW-1:0] m_small, out_y, out_x;
    logic [KWID-1:0] k_small, ker_y, ker_rem;

    // Used only in PADDING != 0 elaboration branch.
    integer ker_x, ci;
    integer in_y, in_x;
    integer linear_addr;

    always_comb begin
        m_small   = MW'(m_addr);
        k_small   = KWID'(k_addr);
        out_y     = '0;
        out_x     = '0;
        ker_y     = '0;
        ker_rem   = '0;
        ker_x     = 0;
        ci        = 0;
        in_y      = 0;
        in_x      = 0;
        linear_addr = 0;
        fmap_addr = '0;
        addr_valid = 1'b0;

        if ((m_addr < M_LEN) && (k_addr < K_LEN)) begin
            // Narrow-operand constant division: only valid ranges are needed.
            out_y   = m_small / MW'(W_OUT);
            out_x   = m_small % MW'(W_OUT);
            ker_y   = k_small / KWID'(K_ROW);
            ker_rem = k_small % KWID'(K_ROW);

            if (PADDING == 0) begin
                // k = ker_y*(KW*C_IN) + (ker_x*C_IN + ci)
                // HWC flat address = (out_y*STRIDE + ker_y)*(W_IN*C_IN)
                //                  + out_x*(STRIDE*C_IN) + ker_rem.
                // Valid m and k imply an in-range window for PADDING=0.
                fmap_addr = ADDR_WIDTH'(
                    (out_y * STRIDE + ker_y) * IN_ROW_STEP
                    + out_x * OUT_COL_STEP + ker_rem
                );
                addr_valid = 1'b1;
            end else begin
                // Retain generic padding/stride behavior.
                ker_x = int'(ker_rem) / C_IN;
                ci    = int'(ker_rem) % C_IN;
                in_y  = int'(out_y) * STRIDE - PADDING + int'(ker_y);
                in_x  = int'(out_x) * STRIDE - PADDING + ker_x;
                if ((in_y >= 0) && (in_y < H_IN) &&
                    (in_x >= 0) && (in_x < W_IN)) begin
                    linear_addr = (in_y * W_IN + in_x) * C_IN + ci;
                    fmap_addr  = ADDR_WIDTH'(linear_addr);
                    addr_valid = 1'b1;
                end
            end
        end
    end
endmodule
