`timescale 1ns / 1ps

// Self-checking address generator regression for 5x5x3 input,
// 3x3 filter, stride 1, padding 0 -> 3x3 output, M=9, K=27.
module tb_conv_addr_gen;
    localparam int H_IN = 5;
    localparam int W_IN = 5;
    localparam int C_IN = 3;
    localparam int KH = 3;
    localparam int KW = 3;
    localparam int W_OUT = 3;

    logic [15:0] m_addr;
    logic [15:0] k_addr;
    logic [15:0] fmap_addr;
    logic addr_valid;
    int expected_addr;
    int tested = 0;

    conv_addr_gen #(
        .H_IN(H_IN), .W_IN(W_IN), .C_IN(C_IN),
        .KH(KH), .KW(KW), .STRIDE(1), .PADDING(0),
        .ADDR_WIDTH(16)
    ) dut (
        .m_addr(m_addr), .k_addr(k_addr),
        .fmap_addr(fmap_addr), .addr_valid(addr_valid)
    );

    initial begin
        m_addr = '0;
        k_addr = '0;

        // Independent enumeration of output windows and kernel positions.
        // Every (m,k) must map to the correct HWC-flat feature-map index.
        for (int oy = 0; oy < 3; oy++) begin
            for (int ox = 0; ox < 3; ox++) begin
                for (int ky = 0; ky < KH; ky++) begin
                    for (int kx = 0; kx < KW; kx++) begin
                        for (int ci = 0; ci < C_IN; ci++) begin
                            m_addr = 16'(oy*W_OUT + ox);
                            k_addr = 16'((ky*KW + kx)*C_IN + ci);
                            expected_addr = ((oy+ky)*W_IN + (ox+kx))*C_IN + ci;
                            #1;  // wait for always_comb to settle
                            if (addr_valid !== 1'b1 ||
                                fmap_addr !== 16'(expected_addr)) begin
                                $fatal(1,
                                  "FAIL m=%0d k=%0d: addr=%0d valid=%b; expected addr=%0d",
                                  m_addr, k_addr, fmap_addr, addr_valid, expected_addr);
                            end
                            if ((m_addr == 0 && k_addr == 8) ||
                                (m_addr == 1 && k_addr == 0) ||
                                (m_addr == 4 && k_addr == 14) ||
                                (m_addr == 8 && k_addr == 26))
                                $display("CHECK m=%0d k=%0d -> fmap_addr=%0d",
                                         m_addr, k_addr, fmap_addr);
                            tested++;
                        end
                    end
                end
            end
        end

        // Values outside the legal M and K ranges must not request a read.
        m_addr = 16'd9;
        k_addr = 16'd0;
        #1;
        if (addr_valid !== 1'b0)
            $fatal(1, "FAIL: out-of-range M not rejected");

        m_addr = 16'd0;
        k_addr = 16'd27;
        #1;
        if (addr_valid !== 1'b0)
            $fatal(1, "FAIL: out-of-range K not rejected");

        if (tested != 243)
            $fatal(1, "FAIL: checked %0d cases, expected 243", tested);

        $display("PASS: conv_addr_gen 5x5x3 / 3x3 / S1 P0; 243/243 M-K addresses correct");
        $finish;
    end
endmodule
