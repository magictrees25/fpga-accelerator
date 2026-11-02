`timescale 1ns / 1ps

// P2: two independent K weight streams, broadcast to even/odd PE rows.
// Activations move east one enabled cycle per column. Each weight stream
// has its own column skew. All eight row partial sums form each output.
// Reuses the existing mac_unit unchanged.
module systolic_array_p2
  import pkg_accelerator::*;
(
  input logic clk, rst_n, enable, clear_acc, valid_reset,
  input logic [63:0] act_in_packed,
  input logic [127:0] wgt_pair_in_packed,
  input logic data_valid_in,
  output logic data_valid_out,
  output logic [255:0] acc_out_packed,
  output logic any_overflow
);
  logic signed [7:0] act_wire [0:7][0:8];
  logic signed [7:0] stream_wgt [0:1][0:7];
  logic signed [7:0] wgt_bcast [0:7][0:7];
  logic signed [7:0] wgt_unused [0:7][0:7];
  logic signed [31:0] pe_acc [0:7][0:7];
  logic [63:0] overflow_bits;

  for (genvar p=0;p<2;p++) begin : gen_stream
    for (genvar j=0;j<8;j++) begin : gen_column
      if (j==0) begin : gen_direct
        assign stream_wgt[p][j]=$signed(wgt_pair_in_packed[(p*8+j)*8 +: 8]);
      end else begin : gen_delayed
        logic signed [7:0] skew [0:j-1];
        always_ff @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
            for (int s=0;s<j;s++) skew[s]<='0;
          end else if (clear_acc) begin
            for (int s=0;s<j;s++) skew[s]<='0;
          end else if (enable) begin
            skew[0]<=$signed(wgt_pair_in_packed[(p*8+j)*8 +: 8]);
            for (int s=1;s<j;s++) skew[s]<=skew[s-1];
          end
        end
        assign stream_wgt[p][j]=skew[j-1];
      end
    end
  end

  for (genvar r=0;r<8;r++) begin : gen_row
    assign act_wire[r][0]=$signed(act_in_packed[r*8 +: 8]);
    for (genvar c=0;c<8;c++) begin : gen_col
      assign wgt_bcast[r][c]=stream_wgt[r%2][c];
      mac_unit u_pe (
        .clk(clk),.rst_n(rst_n),.clear_acc(clear_acc),.enable(enable),
        .act_in(act_wire[r][c]),.wgt_in(wgt_bcast[r][c]),
        .act_out(act_wire[r][c+1]),.wgt_out(wgt_unused[r][c]),
        .acc_out(pe_acc[r][c]),.overflow(overflow_bits[r*8+c])
      );
    end
  end
  for (genvar c=0;c<8;c++) begin : gen_sum
    logic signed [31:0] col_sum;
    always_comb begin
      col_sum='0;
      for (int r=0;r<8;r++) col_sum=col_sum+pe_acc[r][c];
    end
    assign acc_out_packed[c*32 +: 32]=col_sum;
  end
  assign any_overflow=|overflow_bits;

  // Priming indication only; transaction completion comes from K Controller.
  localparam int PRIME_CYCLES=18;
  logic [4:0] valid_cnt;
  logic counting;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin valid_cnt<=0; counting<=0; end
    else if (clear_acc || valid_reset) begin valid_cnt<=0; counting<=0; end
    else if (enable) begin
      if (data_valid_in) counting<=1;
      if ((counting || data_valid_in) && valid_cnt<PRIME_CYCLES)
        valid_cnt<=valid_cnt+1'b1;
    end
  end
  assign data_valid_out=(valid_cnt>=PRIME_CYCLES) && enable;
  // synthesis translate_off
  initial if (ARRAY_ROWS!=8 || ARRAY_COLS!=8 || ACT_WIDTH!=8 ||
              WGT_WIDTH!=8 || ACC_WIDTH!=32 || MAC_LATENCY!=3)
    $fatal(1,"systolic_array_p2 requires baseline 8x8 INT8/INT32, MAC latency 3");
  // synthesis translate_on
endmodule

