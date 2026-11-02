`timescale 1ns / 1ps

// Output feature-map storage, one word = 8 lanes x 32-bit INT32.
// Simple dual-port synchronous RAM: one write port from GEMM controller,
// one synchronous read port for the host/testbench.
//
// wr_mask[i] is the VALID OUTPUT CHANNEL flag for lane i.  During a write,
// inactive lanes are STORED AS ZERO (not left as unknown/uninitialized).
// The original per-word lane mask is stored separately and returned on read.
//
// No memory reset: read only addresses that have been written since preload/
// computation. Read-during-write to the same address is intentionally unspecified.
module output_bram #(
    parameter int ADDR_WIDTH = 16,
    parameter int DATA_WIDTH = 32,
    parameter int LANES      = 8,
    parameter int DEPTH      = 9
)(
    input  logic                         clk,
    input  logic                         rst_n,

    input  logic                         wr_en,
    input  logic [ADDR_WIDTH-1:0]        wr_addr,
    input  logic [DATA_WIDTH*LANES-1:0]  wr_data,
    input  logic [LANES-1:0]             wr_mask,

    input  logic                         rd_en,
    input  logic [ADDR_WIDTH-1:0]        rd_addr,
    output logic [DATA_WIDTH*LANES-1:0]  rd_data,
    output logic [LANES-1:0]             rd_mask,
    output logic                         rd_valid
);
    localparam int WORD_WIDTH = DATA_WIDTH * LANES;

    (* ram_style = "block" *) logic [WORD_WIDTH-1:0] mem [0:DEPTH-1];
    logic [LANES-1:0] mask_mem [0:DEPTH-1];
    logic [WORD_WIDTH-1:0] masked_word;

    // Build a full packed word: invalid channels become zero.
    // In particular N=4 produces [C0,C1,C2,C3,0,0,0,0].
    always_comb begin
        masked_word = '0;
        for (int lane = 0; lane < LANES; lane++) begin
            if (wr_mask[lane])
                masked_word[lane*DATA_WIDTH +: DATA_WIDTH] =
                    wr_data[lane*DATA_WIDTH +: DATA_WIDTH];
        end
    end

    // Synchronous write and synchronous read, with independent addresses.
    // Readback is checked only after computation ends, so no R/W collision.
    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr]      <= masked_word;
            mask_mem[wr_addr] <= wr_mask;
        end
        if (rd_en) begin
            rd_data <= mem[rd_addr];
            rd_mask <= mask_mem[rd_addr];
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_valid <= 1'b0;
        else
            rd_valid <= rd_en;
    end
endmodule
