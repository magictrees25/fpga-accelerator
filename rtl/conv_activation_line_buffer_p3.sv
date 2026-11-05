
`timescale 1ns / 1ps

// P3: 4-row activation line buffer.
//
// Organization:
//   4 rows * 10 pixels/row * 32 channel-pairs/pixel
//   = 1280 entries.
// Each entry stores two INT8 activation values:
//   rd_data[7:0]  = even channel
//   rd_data[15:8] = odd channel.
//
// Two independent 8-bit banks allow:
//   - preload: write one byte into either bank
//   - prefetch: write both banks simultaneously
//   - compute: read both banks simultaneously
//
// Read latency: 1 clock cycle.

module conv_activation_line_buffer_p3 #(
    parameter int ROWS       = 4,
    parameter int W_IN       = 10,
    parameter int C_IN       = 64,
    parameter int ADDR_WIDTH = 11
)(
    input  logic clk,
    input  logic rst_n,

    // Independent write enables for even/odd banks
    input  logic [1:0] wr_en,
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [15:0] wr_data,

    // Synchronous read interface
    input  logic rd_en,
    input  logic [ADDR_WIDTH-1:0] rd_addr,
    output logic [15:0] rd_data,
    output logic rd_valid
);

    localparam int PAIRS_PER_PIXEL = C_IN / 2;
    localparam int DEPTH = ROWS * W_IN * PAIRS_PER_PIXEL;

    // Two 8-bit distributed RAM banks
    (* ram_style = "distributed" *)
    logic [7:0] even_mem [0:DEPTH-1];

    (* ram_style = "distributed" *)
    logic [7:0] odd_mem [0:DEPTH-1];

    logic [7:0] even_q;
    logic [7:0] odd_q;

    // Memory write and synchronous read
    always_ff @(posedge clk) begin

        // Bank 0: even channels
        if (wr_en[0])
            even_mem[wr_addr] <= wr_data[7:0];

        // Bank 1: odd channels
        if (wr_en[1])
            odd_mem[wr_addr] <= wr_data[15:8];

        // Read both banks using the same address
        if (rd_en) begin
            even_q <= even_mem[rd_addr];
            odd_q  <= odd_mem[rd_addr];
        end
    end

    // Output packing
    assign rd_data = {odd_q, even_q};

    // One-cycle read response valid
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_valid <= 1'b0;
        else
            rd_valid <= rd_en;
    end

    // Simulation-only checks
    // synthesis translate_off

    initial begin
        if (ROWS != 4 || W_IN != 10 || C_IN != 64)
            $fatal(1, "P3 Line Buffer: unsupported configuration");

        if (ADDR_WIDTH < $clog2(DEPTH))
            $fatal(1, "P3 Line Buffer: ADDR_WIDTH too small");
    end

    always @(posedge clk) begin
        if (rst_n) begin

            if ((|wr_en) &&
                ($isunknown({wr_addr, wr_data}) ||
                 int'($unsigned(wr_addr)) >= DEPTH))
                $fatal(1, "P3 Line Buffer: illegal write address");

            if (rd_en &&
                ($isunknown(rd_addr) ||
                 int'($unsigned(rd_addr)) >= DEPTH))
                $fatal(1, "P3 Line Buffer: illegal read address");

            // Avoid read/write collision at the same address.
            if (rd_en && (|wr_en) && (rd_addr == wr_addr))
                $fatal(1, "P3 Line Buffer: read/write address collision");
        end
    end

    // synthesis translate_on

endmodule
