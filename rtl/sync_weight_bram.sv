`timescale 1ns / 1ps

// One-clock synchronous read memory for packed weight words.
// Each word contains ARRAY_COLS signed INT8 weights (one K beat, one N tile).
// Read data is registered at the rising edge, like a typical synchronous BRAM.
// This is a small-model baseline; for a production FPGA select an appropriate
// RAM configuration/primitive and verify read-during-write behavior.
module sync_weight_bram #(
    parameter int DEPTH      = 27,
    parameter int WORD_WIDTH = 64,
    parameter int ADDR_WIDTH = 16
)(
    input  logic                  clk,
    input  logic                  wr_en,
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [WORD_WIDTH-1:0] wr_data,
    input  logic                  rd_en,
    input  logic [ADDR_WIDTH-1:0] rd_addr,
    output logic [WORD_WIDTH-1:0] rd_data
);
    (* ram_style = "block" *) logic [WORD_WIDTH-1:0] mem [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        if (rd_en)
            rd_data <= mem[rd_addr];
    end
endmodule
