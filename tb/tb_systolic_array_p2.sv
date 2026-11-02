`timescale 1ns / 1ps

// Standalone array test. No Reader/Controller. Checks every PE input pairing
// before capture, then compares 8 column sums with an independent dot product.
module tb_systolic_array_p2;
  logic clk=0;
  always #5 clk=~clk;
  logic rst_n,enable,clear_acc,valid_reset,data_valid_in,data_valid_out;
  logic [63:0] act_in_packed;
  logic [127:0] wgt_pair_in_packed;
  logic [255:0] acc_out_packed;
  logic any_overflow;
  integer a[0:31],b[0:31][0:7],golden[0:7];
  integer klen,case_id,pairings,outputs_checked;
  logic [255:0] frozen;
  systolic_array_p2 dut (.*);

  task automatic check_inputs(input integer q);
    integer group_at_col,k,expected_a,expected_b;
    begin
      for (int j=0;j<8;j++)
        for (int r=0;r<8;r++) begin
          group_at_col=q-j;
          k=2*group_at_col+r%2;
          expected_a=0; expected_b=0;
          if (group_at_col>=0 && group_at_col<klen/2) begin
            expected_b=b[k][j];
            if (r==k%8) begin
              expected_a=a[k];
              pairings++;
            end
          end
          if (int'($signed(dut.act_wire[r][j])) !== expected_a ||
              int'($signed(dut.wgt_bcast[r][j])) !== expected_b)
            $fatal(1,"FAIL array case=%0d q=%0d PE=(%0d,%0d) A=%0d/%0d W=%0d/%0d",
              case_id,q,r,j,$signed(dut.act_wire[r][j]),expected_a,
              $signed(dut.wgt_bcast[r][j]),expected_b);
        end
    end
  endtask

  initial begin
    rst_n=0; enable=0; clear_acc=0; valid_reset=0; data_valid_in=0;
    act_in_packed=0; wgt_pair_in_packed=0; pairings=0; outputs_checked=0;
    repeat(3) @(negedge clk);
    rst_n=1;
    for (case_id=0;case_id<3;case_id++) begin
      case(case_id) 0:klen=2; 1:klen=16; default:klen=32; endcase
      for (int k=0;k<klen;k++) begin
        a[k]=((k*37+case_id*53)%256)-128;
        for (int n=0;n<8;n++) b[k][n]=((k*29+n*17+case_id*71)%256)-128;
      end
      for(int n=0;n<8;n++) begin
        golden[n]=0;
        for(int k=0;k<klen;k++) golden[n]+=a[k]*b[k][n];
      end
      @(negedge clk);
      enable=1;clear_acc=1;valid_reset=1;act_in_packed=0;wgt_pair_in_packed=0;
      @(posedge clk); #1;
      if(acc_out_packed!==256'b0 || any_overflow!==0)
        $fatal(1,"FAIL array clear did not remove previous partial sums");
      for(int q=0;q<klen/2+13;q++) begin
        @(negedge clk);
        clear_acc=0;valid_reset=0;
        data_valid_in=(q==0);
        act_in_packed=0;wgt_pair_in_packed=0;
        if(q<klen/2) begin
          for(int p=0;p<2;p++) begin
            act_in_packed[((2*q+p)%8)*8 +: 8]=8'(a[2*q+p]);
            for(int n=0;n<8;n++)
              wgt_pair_in_packed[(p*8+n)*8 +: 8]=8'(b[2*q+p][n]);
          end
        end
        @(posedge clk);
        check_inputs(q);
        #1;
        if(any_overflow!==0) $fatal(1,"FAIL array overflow");
      end
      for(int n=0;n<8;n++) begin
        if(int'($signed(acc_out_packed[n*32 +: 32])) !== golden[n])
          $fatal(1,"FAIL array result case=%0d n=%0d got=%0d expected=%0d",
            case_id,n,$signed(acc_out_packed[n*32 +: 32]),golden[n]);
        outputs_checked++;
      end
      @(negedge clk);enable=0;data_valid_in=0;frozen=acc_out_packed;
      repeat(3) begin
        @(posedge clk);#1;
        if(acc_out_packed!==frozen) $fatal(1,"FAIL array disabled state changed");
      end
      $display("CASE PASS: P2 array K=%0d, 8 outputs correct",klen);
    end
    if(pairings!=(2+16+32)*8 || outputs_checked!=24)
      $fatal(1,"FAIL array coverage");
    $display("PASS: systolic_array_p2, K=2/16/32, 400 aligned MAC inputs, 24/24 outputs correct");
    $finish;
  end
  initial begin repeat(2000) @(posedge clk);$fatal(1,"TIMEOUT array P2");end
endmodule

