`timescale 1ns / 1ps

// P5-B2: Shared Activation Feature Map BRAM
// Maximum feature map: 10x10x64 = 6400 INT8 bytes.
// Bank 0 stores even byte addresses.
// Bank 1 stores odd byte addresses.
// One INT8 write per cycle.
// One INT8 pair read per cycle.
// Synchronous read latency: 1 clock.

module conv_activation_bram_shared_p5 #(
    parameter int ADDR_WIDTH = 16,
    parameter int MAX_BYTES  = 6400
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // Host write interface: byte address
    input  logic                    wr_en,
    input  logic [ADDR_WIDTH-1:0]   wr_addr,
    input  logic [7:0]              wr_data,

    // Compute read interface: pair address
    input  logic                    rd_en,
    input  logic [ADDR_WIDTH-1:0]   rd_addr,

    output logic [15:0]             rd_data,
    output logic                    rd_valid
);

    localparam int BANK_DEPTH = MAX_BYTES / 2;

    logic [ADDR_WIDTH-1:0] bank_wr_addr;
    logic [1:0] bank_wr_en;
    logic [7:0] bank_q [0:1];

    logic wr_accept;
    logic rd_accept;

    // Convert byte address to bank address.
    assign bank_wr_addr = wr_addr >> 1;

    assign wr_accept =
        rst_n &&
        wr_en &&
        (int'($unsigned(wr_addr)) < MAX_BYTES);

    assign rd_accept =
        rst_n &&
        rd_en &&
        !wr_en &&
        (int'($unsigned(rd_addr)) < BANK_DEPTH);

    // Two physical 8-bit BRAM banks.
    for (genvar p = 0; p < 2; p++) begin : gen_act_bank

        assign bank_wr_en[p] =
            wr_accept && (wr_addr[0] == 1'(p));

        sync_fmap_bram #(
            .DEPTH(BANK_DEPTH),
            .DATA_WIDTH(8),
            .ADDR_WIDTH(ADDR_WIDTH)
        ) u_mem (
            .clk(clk),

            .wr_en(bank_wr_en[p]),
            .wr_addr(bank_wr_addr),
            .wr_data(wr_data),

            .rd_en(rd_accept),
            .rd_addr(rd_addr),
            .rd_data(bank_q[p])
        );

    end

    // Pair packing:
    // [7:0]  = even address
    // [15:8] = odd address
    assign rd_data = {bank_q[1], bank_q[0]};

    // One-cycle response valid.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_valid <= 1'b0;
        else
            rd_valid <= rd_accept;
    end

    // Simulation checks.
    // synthesis translate_off

    initial begin
        if (MAX_BYTES <= 0 || (MAX_BYTES % 2) != 0)
            $fatal(1, "Shared Activation BRAM: MAX_BYTES must be positive and even");

        if (ADDR_WIDTH < $clog2(BANK_DEPTH))
            $fatal(1, "Shared Activation BRAM: ADDR_WIDTH too small");
    end

    always @(posedge clk) begin
        if (rst_n) begin

            if (wr_en &&
                ($isunknown(wr_addr) ||
                 int'($unsigned(wr_addr)) >= MAX_BYTES))
                $fatal(1, "Shared Activation BRAM: illegal write address");

            if (rd_en &&
                ($isunknown(rd_addr) ||
                 int'($unsigned(rd_addr)) >= BANK_DEPTH))
                $fatal(1, "Shared Activation BRAM: illegal read address");

            if (wr_en && rd_en)
                $fatal(1, "Shared Activation BRAM: read/write overlap");

        end
    end

    // synthesis translate_on

endmodule