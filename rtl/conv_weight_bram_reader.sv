`timescale 1ns / 1ps

// Synchronous weight reader paired with conv_bram_reader.
// Both use the SAME lookahead fetch_k_addr, and both have one registered
// BRAM read stage: during S_CLEAR prefetch K0; during S_FEED(Kk) use Kk
// while fetching K(k+1). Supports tiled N, with 8 weights per memory word.
//
// Memory layout (one 64-bit word per K and N tile):
//   word_addr = n_tile_idx * K_LEN + fetch_k_addr
//             = (n_base / ARRAY_COLS) * K_LEN + fetch_k_addr.
//
// Not a general stallable memory interface: assumes K Controller does not
// stall and consumes one beat per clock throughout S_FEED.
module conv_weight_bram_reader
  import pkg_accelerator::*;
#(
    parameter int K_LEN       = 27,
    parameter int N_TILES     = 1,
    parameter int ADDR_WIDTH  = 16
)(
    input  logic clk,
    input  logic rst_n,
    input  logic [ADDR_WIDTH-1:0] n_base,
    input  logic [ADDR_WIDTH-1:0] fetch_k_addr,
    input  logic clear_acc,
    input  logic feed_valid,
    input  logic wr_en,
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [WGT_WIDTH*ARRAY_COLS-1:0] wr_data,
    output logic [WGT_WIDTH*ARRAY_COLS-1:0] wgt_row_packed,
    output logic [ADDR_WIDTH-1:0] weight_word_addr,
    output logic data_valid
);
    localparam int WORD_WIDTH = WGT_WIDTH * ARRAY_COLS;
    localparam int DEPTH = K_LEN * N_TILES;
    logic [WORD_WIDTH-1:0] ram_q;
    logic rd_en;
    logic read_in_range;
    logic rd_valid;
    logic [ADDR_WIDTH-1:0] n_tile_idx;

    assign n_tile_idx = n_base / ARRAY_COLS;
    //assign weight_word_addr = n_tile_idx * K_LEN + fetch_k_addr;
	generate
    if (N_TILES == 1) begin : gen_single_tile
        assign weight_word_addr = fetch_k_addr;
    end
    else begin : gen_multi_tile
        assign weight_word_addr = n_tile_idx * K_LEN
                                + fetch_k_addr;
    end
   endgenerate
    assign read_in_range = (int'(n_tile_idx) < N_TILES) &&
                           (int'(fetch_k_addr) < K_LEN);
    assign rd_en = (clear_acc || feed_valid) && read_in_range;

    sync_weight_bram #(
        .DEPTH(DEPTH), .WORD_WIDTH(WORD_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)
    ) u_mem (
        .clk(clk), .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data),
        .rd_en(rd_en), .rd_addr(weight_word_addr), .rd_data(ram_q)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_valid <= 1'b0;
        else
            rd_valid <= rd_en;
    end

    assign data_valid = rd_valid;
    assign wgt_row_packed = rd_valid ? ram_q : '0;
endmodule
