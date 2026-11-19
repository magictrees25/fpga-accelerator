`timescale 1ns / 1ps

// Host-scheduled 8x8x8 tile for ResNet18 layer1.0, using the unchanged P5
// shared P=2 convolution core. Both convolutions use cfg_mode=0 (3x3/S1).
// Host supplies a 10x10x64 halo, 576 packed weight words, 8 BN-folded biases,
// 8 Q24 multipliers, and (for conv2) 64 words of original block-input skips.
// Whole-feature-map storage and inter-tile scheduling remain on the Host/TB.
module fpga_system_top_p7_residual_tile (
    input logic clk, rst_n, start,
    input logic residual_en,
    input logic [255:0] channel_bias, channel_multiplier,
    input logic fmap_wr_en,
    input logic [15:0] fmap_wr_addr,
    input logic [7:0] fmap_wr_data,
    input logic weight_wr_en,
    input logic [15:0] weight_wr_addr,
    input logic [63:0] weight_wr_data,
    input logic skip_wr_en,
    input logic [5:0] skip_wr_addr,
    input logic [63:0] skip_wr_data,
    input logic out_rd_en,
    input logic [15:0] out_rd_addr,
    input logic [2:0] out_rd_lane_sel,
    output logic signed [7:0] out_rd_data,
    output logic out_rd_valid, out_rd_lane_valid,
    output logic busy, done, any_overflow
);
    logic accepted_start, core_busy, core_done, core_done_seen;
    logic raw_valid;
    logic [15:0] raw_addr, raw_m, raw_n;
    logic [7:0] raw_mask;
    logic [255:0] raw_data;
    logic [255:0] bias_q, multiplier_q;
    logic residual_q;
    (* ram_style = "distributed" *) logic [63:0] skip_mem [0:63];
    logic [63:0] skip_word, quant_data;
    logic [7:0] quant_valid;
    logic [15:0] addr_q1, addr_q2;
    logic [7:0] mask_q1, mask_q2;
    logic [6:0] post_write_count;
    logic [63:0] rd_word;
    logic [7:0] rd_mask;
    logic [2:0] lane_q;

    assign accepted_start = start && !busy && !done;
    assign skip_word = skip_mem[raw_addr[5:0]];
    always_ff @(posedge clk)
        if (skip_wr_en && !busy) skip_mem[skip_wr_addr] <= skip_wr_data;

    conv_accelerator_top_p5_multimode u_compute (
        .clk(clk), .rst_n(rst_n), .start(accepted_start), .cfg_mode(2'd0),
        .fmap_wr_en(fmap_wr_en && !busy), .fmap_wr_addr(fmap_wr_addr), .fmap_wr_data(fmap_wr_data),
        .weight_wr_en(weight_wr_en && !busy), .weight_wr_addr(weight_wr_addr), .weight_wr_data(weight_wr_data),
        .write_valid(raw_valid), .write_addr(raw_addr), .write_m_addr(raw_m), .write_n_base(raw_n),
        .write_mask(raw_mask), .write_data(raw_data), .busy(core_busy), .done(core_done),
        .any_overflow(any_overflow)
    );

    for (genvar lane = 0; lane < 8; lane++) begin : gen_post
        requant_residual_p7 u_post (
            .clk(clk), .rst_n(rst_n), .in_valid(raw_valid),
            .in_acc($signed(raw_data[lane*32 +: 32])),
            .in_bias($signed(bias_q[lane*32 +: 32])),
            .in_multiplier($signed(multiplier_q[lane*32 +: 32])),
            .in_skip($signed(skip_word[lane*8 +: 8])), .in_residual_en(residual_q),
            .out_valid(quant_valid[lane]), .out_data(quant_data[lane*8 +: 8])
        );
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0;
            done <= 0;
            core_done_seen <= 0;
            post_write_count <= 0;
            bias_q <= 0;
            multiplier_q <= 0;
            residual_q <= 0;
            addr_q1 <= 0; addr_q2 <= 0;
            mask_q1 <= 0; mask_q2 <= 0;
            lane_q <= 0;
        end else begin
            done <= 0;
            // Same two pipeline stages as requant_residual_p7.
            addr_q1 <= raw_addr; addr_q2 <= addr_q1;
            mask_q1 <= raw_mask; mask_q2 <= mask_q1;
            if (out_rd_en) lane_q <= out_rd_lane_sel;
            if (accepted_start) begin
                busy <= 1;
                core_done_seen <= 0;
                post_write_count <= 0;
                bias_q <= channel_bias;
                multiplier_q <= channel_multiplier;
                residual_q <= residual_en;
            end else if (busy) begin
                if (core_done) core_done_seen <= 1;
                if (quant_valid[0]) post_write_count <= post_write_count + 1'b1;
                // done only after the final quantized word has actually reached RAM.
                if ((core_done_seen || core_done) &&
                    ((post_write_count == 64) || (quant_valid[0] && post_write_count == 63))) begin
                    busy <= 0;
                    done <= 1;
                end
            end
        end
    end

    output_bram #(.ADDR_WIDTH(16), .DATA_WIDTH(8), .LANES(8), .DEPTH(64)) u_output (
        .clk(clk), .rst_n(rst_n), .wr_en(quant_valid[0]), .wr_addr(addr_q2),
        .wr_data(quant_data), .wr_mask(mask_q2), .rd_en(out_rd_en), .rd_addr(out_rd_addr),
        .rd_data(rd_word), .rd_mask(rd_mask), .rd_valid(out_rd_valid)
    );
    assign out_rd_data = $signed(rd_word[lane_q*8 +: 8]);
    assign out_rd_lane_valid = out_rd_valid && rd_mask[lane_q];

    // synthesis translate_off
    always @(posedge clk) if (rst_n) begin
        if (busy && (start || fmap_wr_en || weight_wr_en || skip_wr_en || out_rd_en))
            $fatal(1, "P7: Host must wait for done before loading/reading/restarting");
        if (raw_valid && (raw_addr >= 64 || raw_mask !== 8'hff || raw_m !== raw_addr || raw_n != 0))
            $fatal(1, "P7: unexpected convolution output metadata");
        if (quant_valid !== 8'h00 && quant_valid !== 8'hff)
            $fatal(1, "P7: postprocess lane valid misalignment");
        if (quant_valid[0] && (!busy || post_write_count >= 64))
            $fatal(1, "P7: unexpected postprocess write");
        if (out_rd_en && out_rd_addr >= 64) $fatal(1, "P7: output read out of range");
    end
    // synthesis translate_on
endmodule
