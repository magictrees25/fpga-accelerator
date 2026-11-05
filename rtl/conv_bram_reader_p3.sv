
`timescale 1ns / 1ps

// ============================================================
// P3 Activation Reader
//
// Based on P2.1 activation memory organization.
//
// Input feature map: 10 x 10 x 64
// Kernel:             3 x 3
// M:                  64
// K:                  576
// P:                  2
//
// Initial preload:
//   Rows 0~3 are copied into the 4-row Line Buffer.
//
// During computation:
//   Compute requests are served by Line Buffer.
//   Prefetch engine reads the next feature-map row from BRAM.
//
// BRAM read latency: 1 clock
// Line Buffer read latency: 1 clock
// ============================================================

module conv_bram_reader_p3 #(
    parameter int H_IN       = 10,
    parameter int W_IN       = 10,
    parameter int C_IN       = 64,
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

    localparam int DEPTH = H_IN * W_IN * C_IN;
    localparam int BANK_DEPTH = DEPTH / 2;

    localparam int PAIRS_PER_PIXEL = C_IN / 2;
    localparam int PAIRS_PER_ROW = W_IN * PAIRS_PER_PIXEL;

    localparam int INITIAL_ROWS = 4;
    localparam int INITIAL_BYTES =
        INITIAL_ROWS * W_IN * C_IN;

    // --------------------------------------------------------
    // 1. Activation BRAM: two 8-bit banks
    // --------------------------------------------------------

    logic [ADDR_WIDTH-1:0] bank_wr_addr;
    logic [ADDR_WIDTH-1:0] bank_rd_addr [0:1];

    logic [1:0] bank_wr_en;
    logic [1:0] bank_rd_en;

    logic [7:0] bank_q [0:1];

    assign bank_wr_addr = wr_addr >> 1;

    for (genvar p = 0; p < 2; p++) begin : gen_act_bank

        assign bank_wr_en[p] =
            wr_en &&
            (wr_addr < ADDR_WIDTH'(DEPTH)) &&
            (wr_addr[0] == (p == 1));

        // Both banks read the same pair address.
        assign bank_rd_en[p] = prefetch_issue;
        assign bank_rd_addr[p] = prefetch_bram_addr;

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

    end

    // --------------------------------------------------------
    // 2. M/K -> Sliding Window address mapping
    // --------------------------------------------------------

    logic [3:0] kernel_idx;
    logic [1:0] ky;
    logic [1:0] kx;
    logic kernel_valid;

    logic [3:0] input_row;
    logic [3:0] input_col;

    logic [1:0] row_slot;

    logic [10:0] lb_row_base;
    logic [10:0] lb_col_base;
    logic [10:0] lb_rd_addr;

    always_comb begin

        kernel_idx = fetch_k_base[9:6];

        ky = 0;
        kx = 0;
        kernel_valid = 1'b1;

        case (kernel_idx)

            0: begin ky=0; kx=0; end
            1: begin ky=0; kx=1; end
            2: begin ky=0; kx=2; end

            3: begin ky=1; kx=0; end
            4: begin ky=1; kx=1; end
            5: begin ky=1; kx=2; end

            6: begin ky=2; kx=0; end
            7: begin ky=2; kx=1; end
            8: begin ky=2; kx=2; end

            default: begin
                ky = 0;
                kx = 0;
                kernel_valid = 1'b0;
            end

        endcase

        // M = oy*8 + ox
        // Input row = oy + ky
        // Input col = ox + kx

        input_row =
            {1'b0, m_addr[5:3]} + {2'b00, ky};

        input_col =
            {1'b0, m_addr[2:0]} + {2'b00, kx};

        // Four-row circular buffer.
        row_slot = input_row[1:0];

        // row_base = row_slot * 320
        lb_row_base =
            {1'b0, row_slot, 8'b0} +
            {3'b0, row_slot, 6'b0};

        // col_base = input_col * 32
        lb_col_base = {2'b0, input_col, 5'b0};

        // pair index = K[5:1]
        lb_rd_addr =
            lb_row_base +
            lb_col_base +
            {6'b0, fetch_k_base[5:1]};

    end

    logic request_legal;
    logic pair_accept;

    assign request_legal =
        (m_addr < ADDR_WIDTH'(64)) &&
        (fetch_k_base < ADDR_WIDTH'(576)) &&
        !fetch_k_base[0] &&
        kernel_valid &&
        (input_row < 10) &&
        (input_col < 10);

    assign pair_accept =
        rst_n &&
        pair_req_en &&
        request_legal &&
        !wr_en;

    // --------------------------------------------------------
    // 3. Four-row Line Buffer
    // --------------------------------------------------------

    logic [1:0] lb_wr_en;
    logic [10:0] lb_wr_addr;
    logic [15:0] lb_wr_data;

    logic lb_rd_en;
    logic [15:0] lb_rd_data;
    logic lb_rd_valid;

    assign lb_rd_en = pair_accept;

    // --------------------------------------------------------
    // 4. Initial preload: capture input rows 0~3
    // --------------------------------------------------------

    logic preload_lb_accept;

    assign preload_lb_accept =
        wr_en &&
        (wr_addr < ADDR_WIDTH'(INITIAL_BYTES));

    // --------------------------------------------------------
    // 5. Prefetch controller
    // --------------------------------------------------------

    typedef enum logic [1:0] {
        PF_IDLE,
        PF_READ,
        PF_FLUSH
    } pf_state_t;

    pf_state_t pf_state;

    logic [3:0] prefetch_row;
    logic [8:0] prefetch_index;

    logic [11:0] prefetch_bram_row_base;
    logic [11:0] prefetch_bram_addr;

    logic [10:0] prefetch_lb_row_base;
    logic [10:0] prefetch_lb_addr_now;

    logic prefetch_issue;
    logic prefetch_busy;
    logic prefetch_trigger;

    // BRAM response pipeline
    logic prefetch_pending;
    logic [10:0] prefetch_pending_lb_addr;

    assign prefetch_busy = (pf_state != PF_IDLE);
    assign prefetch_issue = (pf_state == PF_READ);

    // Trigger at the first M of output rows y=1~6.
    //
    // y=1 -> prefetch input row 4
    // y=2 -> prefetch input row 5
    // ...
    // y=6 -> prefetch input row 9

    assign prefetch_trigger =
        pair_accept &&
        (fetch_k_base == 0) &&
        (m_addr[2:0] == 0) &&
        (m_addr[5:3] >= 1) &&
        (m_addr[5:3] <= 6);

    // BRAM address = prefetch_row * 320 + index
    assign prefetch_bram_row_base =
        ({8'b0, prefetch_row} << 8) +
        ({8'b0, prefetch_row} << 6);

    assign prefetch_bram_addr =
        prefetch_bram_row_base +
        {3'b0, prefetch_index};

    // Circular Line Buffer slot = prefetch_row % 4
    assign prefetch_lb_row_base =
        {1'b0, prefetch_row[1:0], 8'b0} +
        {3'b0, prefetch_row[1:0], 6'b0};

    assign prefetch_lb_addr_now =
        prefetch_lb_row_base +
        {2'b0, prefetch_index};

    // Prefetch FSM
    always_ff @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin
            pf_state <= PF_IDLE;
            prefetch_row <= 0;
            prefetch_index <= 0;
        end

        else begin

            case (pf_state)

                PF_IDLE: begin

                    if (prefetch_trigger) begin

                        prefetch_row <=
                            {1'b0, m_addr[5:3]} + 4'd3;

                        prefetch_index <= 0;
                        pf_state <= PF_READ;

                    end

                end

                PF_READ: begin

                    if (prefetch_index == PAIRS_PER_ROW-1) begin
                        pf_state <= PF_FLUSH;
                    end

                    else begin
                        prefetch_index <= prefetch_index + 1'b1;
                    end

                end

                PF_FLUSH: begin
                    pf_state <= PF_IDLE;
                end

                default: begin
                    pf_state <= PF_IDLE;
                end

            endcase

        end

    end

    // Delay the BRAM request by one cycle.
    // The delayed address identifies the returned data.

    always_ff @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin
            prefetch_pending <= 0;
            prefetch_pending_lb_addr <= 0;
        end

        else begin

            prefetch_pending <= prefetch_issue;

            if (prefetch_issue)
                prefetch_pending_lb_addr <= prefetch_lb_addr_now;

        end

    end

    // --------------------------------------------------------
    // 6. Line Buffer write arbitration
    //
    // Priority:
    //   1. BRAM prefetch response
    //   2. Initial preload capture
    // --------------------------------------------------------

    always_comb begin

        lb_wr_en = 0;
        lb_wr_addr = 0;
        lb_wr_data = 0;

        if (prefetch_pending) begin

            lb_wr_en = 2'b11;

            lb_wr_addr = prefetch_pending_lb_addr;

            lb_wr_data = {
                bank_q[1],
                bank_q[0]
            };

        end

        else if (preload_lb_accept) begin

            lb_wr_addr = wr_addr[11:1];

            if (wr_addr[0] == 1'b0) begin

                lb_wr_en = 2'b01;
                lb_wr_data[7:0] = wr_data;

            end

            else begin

                lb_wr_en = 2'b10;
                lb_wr_data[15:8] = wr_data;

            end

        end

    end

    conv_activation_line_buffer_p3 #(
        .ROWS(4),
        .W_IN(10),
        .C_IN(64),
        .ADDR_WIDTH(11)
    ) u_line_buffer (

        .clk(clk),
        .rst_n(rst_n),

        .wr_en(lb_wr_en),
        .wr_addr(lb_wr_addr),
        .wr_data(lb_wr_data),

        .rd_en(lb_rd_en),
        .rd_addr(lb_rd_addr),
        .rd_data(lb_rd_data),
        .rd_valid(lb_rd_valid)

    );

    // --------------------------------------------------------
    // 7. Reader response interface
    // --------------------------------------------------------

    assign act_rsp_valid = {2{lb_rd_valid}};

    assign act_pair_packed =
        lb_rd_valid ? lb_rd_data : 16'b0;

    // Preserve P2-style response tags for debugging.
    logic [ADDR_WIDTH-1:0] rsp_k_base;
    logic [ADDR_WIDTH-1:0] rsp_m_addr;

    always_ff @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin
            rsp_k_base <= 0;
            rsp_m_addr <= 0;
        end

        else begin

            if (pair_accept) begin
                rsp_k_base <= fetch_k_base;
                rsp_m_addr <= m_addr;
            end

            else begin
                rsp_k_base <= 0;
                rsp_m_addr <= 0;
            end

        end

    end

    // --------------------------------------------------------
    // 8. Simulation checks
    // --------------------------------------------------------

    // synthesis translate_off

    initial begin

        if (H_IN != 10 || W_IN != 10 || C_IN != 64)
            $fatal(1, "P3 Reader: only 10x10x64 is supported");

        if (ADDR_WIDTH < 12)
            $fatal(1, "P3 Reader: ADDR_WIDTH must be >= 12");

    end

    always @(posedge clk) begin

        if (rst_n) begin

            if (wr_en &&
                ($isunknown(wr_addr) ||
                 wr_addr >= DEPTH))
                $fatal(1, "P3 Reader: invalid preload address");

            if (pair_req_en && !request_legal)
                $fatal(1, "P3 Reader: illegal M/K request");

            if (prefetch_trigger && prefetch_busy)
                $fatal(1, "P3 Reader: previous prefetch not finished");

            if (wr_en && prefetch_busy)
                $fatal(1, "P3 Reader: preload during prefetch");

            if (preload_lb_accept && prefetch_pending)
                $fatal(1, "P3 Reader: preload/prefetch write conflict");

        end

    end

    // synthesis translate_on

endmodule
