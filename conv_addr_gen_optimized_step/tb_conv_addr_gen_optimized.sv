`timescale 1ns / 1ps

// Independent reference: direct (y,x,c) coordinate enumeration.
module addr_case #(
    parameter int H_IN=5, W_IN=5, C_IN=3,
    parameter int KH=3, KW=3, STRIDE=1, PADDING=0
)(output logic done);
    localparam int H_OUT=(H_IN+2*PADDING-KH)/STRIDE+1;
    localparam int W_OUT=(W_IN+2*PADDING-KW)/STRIDE+1;
    localparam int M_LEN=H_OUT*W_OUT;
    localparam int K_LEN=KH*KW*C_IN;

    logic [15:0] m_addr, k_addr, fmap_addr;
    logic addr_valid;
    integer oy,ox,ky,kx,ci,iy,ix,expected;
    bit exp_valid;

    conv_addr_gen #(.H_IN(H_IN),.W_IN(W_IN),.C_IN(C_IN),
        .KH(KH),.KW(KW),.STRIDE(STRIDE),.PADDING(PADDING),
        .ADDR_WIDTH(16)) dut (.*);

    initial begin
        done=0;
        m_addr=0; k_addr=0;
        for (int m=0; m<M_LEN; m++) begin
            for (int k=0; k<K_LEN; k++) begin
                m_addr=16'(m); k_addr=16'(k);
                oy=m/W_OUT; ox=m%W_OUT;
                ky=k/(KW*C_IN); kx=(k/C_IN)%KW; ci=k%C_IN;
                iy=oy*STRIDE-PADDING+ky;
                ix=ox*STRIDE-PADDING+kx;
                exp_valid=(iy>=0&&iy<H_IN&&ix>=0&&ix<W_IN);
                expected=exp_valid ? (iy*W_IN+ix)*C_IN+ci : 0;
                #1;
                if (addr_valid !== exp_valid || fmap_addr !== 16'(expected))
                    $fatal(1,"FAIL %0dx%0dx%0d KH%0d KW%0d S%0d P%0d m%0d k%0d got (%b,%0d) expected (%b,%0d)",
                      H_IN,W_IN,C_IN,KH,KW,STRIDE,PADDING,m,k,addr_valid,fmap_addr,exp_valid,expected);
            end
        end
        m_addr=16'(M_LEN); k_addr=0; #1;
        if (addr_valid !== 1'b0) $fatal(1,"FAIL invalid M");
        m_addr=0; k_addr=16'(K_LEN); #1;
        if (addr_valid !== 1'b0) $fatal(1,"FAIL invalid K");
        $display("PASS case H%0d W%0d C%0d KH%0d KW%0d S%0d P%0d (%0d addresses)",
                 H_IN,W_IN,C_IN,KH,KW,STRIDE,PADDING,M_LEN*K_LEN);
        done=1;
    end
endmodule

module tb_conv_addr_gen_optimized;
    logic d0,d1,d2,d3;
    addr_case #(.H_IN(5),.W_IN(5),.C_IN(3),.KH(3),.KW(3),.STRIDE(1),.PADDING(0)) c0(d0);
    addr_case #(.H_IN(7),.W_IN(7),.C_IN(3),.KH(3),.KW(3),.STRIDE(2),.PADDING(1)) c1(d1);
    addr_case #(.H_IN(6),.W_IN(6),.C_IN(8),.KH(1),.KW(1),.STRIDE(1),.PADDING(0)) c2(d2);
    addr_case #(.H_IN(6),.W_IN(6),.C_IN(4),.KH(3),.KW(3),.STRIDE(2),.PADDING(0)) c3(d3);
    initial begin
        wait(d0&&d1&&d2&&d3);
        $display("PASS: optimized conv_addr_gen: 4 parameter configurations");
        $finish;
    end
endmodule
