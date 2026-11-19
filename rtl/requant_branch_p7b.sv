`timescale 1ns / 1ps
// Q24 per-channel BN-folded requantization, with a signed INT32 shortcut.
// relu=0: preserve signed INT32 branch (no INT8 clip, no ReLU).
// relu=1: add optional shortcut, then clamp to [0,127] in a 32-bit word.
// Two register stages. Unsupported signed-INT32 branch overflow is reported.
module requant_branch_p7b (
    input logic clk,rst_n,in_valid,
    input logic signed [31:0] in_acc,in_bias,in_multiplier,in_skip,
    input logic in_residual_en,in_relu_en,
    output logic out_valid,out_overflow,
    output logic signed [31:0] out_data
);
    logic signed [32:0] biased;
    logic signed [64:0] product_q;
    logic signed [31:0] skip_q;
    logic valid_q,residual_q,relu_q;
    logic signed [65:0] extended,magnitude,rounded,combined;
    assign biased=$signed({in_acc[31],in_acc})+$signed({in_bias[31],in_bias});
    always_comb begin
        extended={product_q[64],product_q};
        magnitude=(extended<0)?-extended:extended;
        rounded=(magnitude+(66'sd1 <<< 23)) >>> 24;
        if(extended<0) rounded=-rounded;
        combined=rounded;
        if(residual_q) combined=rounded+$signed({{34{skip_q[31]}},skip_q});
    end
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            valid_q<=0;out_valid<=0;out_overflow<=0;out_data<=0;
            product_q<=0;skip_q<=0;residual_q<=0;relu_q<=0;
        end else begin
            valid_q<=in_valid;out_valid<=valid_q;out_overflow<=0;
            if(in_valid) begin
                product_q<=$signed({{32{biased[32]}},biased})*
                           $signed({{33{in_multiplier[31]}},in_multiplier});
                skip_q<=in_skip;residual_q<=in_residual_en;relu_q<=in_relu_en;
            end
            if(valid_q) begin
                if(relu_q) begin
                    if(combined<0) out_data<=0;
                    else if(combined>127) out_data<=127;
                    else out_data<=combined[31:0];
                end else begin
                    out_data<=combined[31:0];
                    out_overflow<=(combined < -66'sd2147483648 || combined > 66'sd2147483647);
                end
            end
        end
    end
    // synthesis translate_off
    always @(posedge clk) if(rst_n && in_valid)
        if($isunknown({in_multiplier,in_residual_en,in_relu_en}) || in_multiplier<0)
            $fatal(1,"P7-B: unknown control or negative multiplier");
    // synthesis translate_on
endmodule
