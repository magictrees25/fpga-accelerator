`timescale 1ns / 1ps

// P7-A signed per-channel affine requantization, fixed Q24 multiplier.
// Two register stages. No clamp/ReLU is applied to the convolution branch
// before adding the identity residual. Input and skip metadata travel together.
// Contract: multiplier >= 0, zero_point = 0; output is signed INT8 in [0,127].
module requant_residual_p7 (
    input  logic clk, rst_n,
    input  logic in_valid,
    input  logic signed [31:0] in_acc, in_bias, in_multiplier,
    input  logic signed [7:0] in_skip,
    input  logic in_residual_en,
    output logic out_valid,
    output logic signed [7:0] out_data
);
    logic signed [32:0] biased;
    logic signed [64:0] product_q;
    logic signed [7:0] skip_q;
    logic residual_q, valid_q;
    assign biased = $signed({in_acc[31], in_acc}) + $signed({in_bias[31], in_bias});

    function automatic logic signed [7:0] finish_quant(
        input logic signed [64:0] product,
        input logic signed [7:0] skip,
        input logic residual_en
    );
        logic signed [65:0] extended, magnitude, rounded, combined;
        begin
            extended = {product[64], product};
            magnitude = (extended < 0) ? -extended : extended;
            rounded = (magnitude + (66'sd1 <<< 23)) >>> 24;
            if (extended < 0) rounded = -rounded;
            combined = rounded;
            if (residual_en) combined = rounded + {{58{skip[7]}}, skip};
            if (combined < 0) finish_quant = 8'sd0;
            else if (combined > 127) finish_quant = 8'sd127;
            else finish_quant = combined[7:0];
        end
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_q <= 0;
            out_valid <= 0;
            product_q <= 0;
            skip_q <= 0;
            residual_q <= 0;
            out_data <= 0;
        end else begin
            valid_q <= in_valid;
            out_valid <= valid_q;
            if (in_valid) begin
                // Explicit sign extension fixes multiplication expression width.
                product_q <= $signed({{32{biased[32]}}, biased}) *
                             $signed({{33{in_multiplier[31]}}, in_multiplier});
                skip_q <= in_skip;
                residual_q <= in_residual_en;
            end
            if (valid_q) out_data <= finish_quant(product_q, skip_q, residual_q);
        end
    end

    // synthesis translate_off
    always @(posedge clk)
        if (rst_n && in_valid && ($isunknown(in_multiplier) || in_multiplier < 0))
            $fatal(1, "P7: multiplier must be known and nonnegative");
    // synthesis translate_on
endmodule
