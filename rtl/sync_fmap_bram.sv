`timescale 1ns / 1ps

// Simple dual-port, synchronous-read feature-map RAM.
// 1 clock read latency: on a rising edge rd_data <= mem[rd_addr].
// Separate write port is used for bring-up/preloading in the testbench.
// For hardware, consider an appropriate FPGA BRAM primitive/XPM for your device.
module sync_fmap_bram #(
    parameter int DEPTH      = 75,
    parameter int DATA_WIDTH = 8,
    parameter int ADDR_WIDTH = 16
) (
    input  logic clk,
    input  logic wr_en,
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [DATA_WIDTH-1:0] wr_data,
    input  logic rd_en,
    input  logic [ADDR_WIDTH-1:0] rd_addr,
    output logic [DATA_WIDTH-1:0] rd_data
);
    (* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        if (rd_en)
            rd_data <= mem[rd_addr];
    end
endmodule
