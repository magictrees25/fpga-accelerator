`timescale 1ns / 1ps

// Minimal, synthesizable K controller for taitashaw/fpga-ai-accelerator.
//
// Scope: one output position (M=1), eight output channels (N=8), variable K.
// Assumption: act_data / wgt_row_packed respond combinationally to k_addr.
// A synchronous BRAM requires a separate request/return pipeline (later step).
//
// Mapping used by this project's systolic_array:
//   K index k -> PE row (k % ARRAY_ROWS)
//   output channel n -> PE column n
//   all K beats stream consecutively; accumulated values are NOT cleared
//   at K-tile boundaries, only once before starting the entire dot product.
module k_tile_controller
  import pkg_accelerator::*;
#(
  parameter int K_ADDR_WIDTH = 16,  //k维索引地址16bit
  // K 数据全部输入以后，再等待 13 个周期，让阵列中的数据传播和 MAC 流水线完成。
  parameter int DRAIN_CYCLES = ARRAY_COLS + MAC_LATENCY + 2
)(
  input  logic clk,
  input  logic rst_n,
  input  logic start,                         // 1-clock pulse; accepted when idle
  input  logic [K_ADDR_WIDTH-1:0] k_len,     // number of K elements (>0) 输入k长度

  // Combinational data source. k_addr selects A[k] and B[k, 0..7].
  output logic [K_ADDR_WIDTH-1:0] k_addr,//当前要读取的k位置
  input  logic signed [ACT_WIDTH-1:0] act_data,//当前k位置的一个激活值
  input  logic [WGT_WIDTH*ARRAY_COLS-1:0] wgt_row_packed,//当前K位置对应的8个权重 64bit

  // Direct interface to systolic_array.
  output logic array_enable,
  output logic feed_valid,                    // one beat of K input each clock
  output logic clear_acc,
  output logic valid_reset,
  output logic data_valid_in,
  output logic [ACT_WIDTH*ARRAY_ROWS-1:0] act_in_packed, //送给8个PE行的激活输入
  output logic signed [WGT_WIDTH-1:0] wgt_in [ARRAY_COLS],
  input  logic [ACC_WIDTH*ARRAY_COLS-1:0] acc_out_packed,

  // Status and stable result snapshot.
  output logic busy,
  output logic done,                          // 1-clock pulse
  output logic [ACC_WIDTH*ARRAY_COLS-1:0] result_packed
);

  typedef enum logic [2:0] {
    S_IDLE,
    S_CLEAR,
    S_FEED,
    S_DRAIN,
    S_DONE
  } state_t;

  state_t state;
  logic [K_ADDR_WIDTH-1:0] k_idx;
  logic [K_ADDR_WIDTH-1:0] k_len_saved;
  logic [$clog2(DRAIN_CYCLES+1)-1:0] drain_cnt;

  assign k_addr = k_idx;
  assign busy   = (state == S_CLEAR || state == S_FEED || state == S_DRAIN);
  assign done   = (state == S_DONE);
  assign feed_valid = (state == S_FEED);

  // Combinational outputs: present the correct beat before the rising edge.
  always_comb begin
    array_enable   = (state == S_CLEAR || state == S_FEED || state == S_DRAIN);
    clear_acc      = (state == S_CLEAR);
    valid_reset    = (state == S_CLEAR);
    data_valid_in  = (feed_valid && k_idx == 0);
    act_in_packed  = '0;

    for (int n = 0; n < ARRAY_COLS; n = n + 1)
      wgt_in[n] = '0;

    if (state == S_FEED) begin
      // On beat k, ONLY PE row k%8 receives A[k] at the west boundary.
      act_in_packed[(k_idx % ARRAY_ROWS)*ACT_WIDTH +: ACT_WIDTH] = act_data;

      // B[k,n] streams for all 8 columns; array supplies per-column skew.
      for (int n = 0; n < ARRAY_COLS; n = n + 1)
        wgt_in[n] = $signed(wgt_row_packed[n*WGT_WIDTH +: WGT_WIDTH]);
    end
  end

  // State machine and counters.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state         <= S_IDLE;
      k_idx         <= '0;
      k_len_saved   <= '0;
      drain_cnt     <= '0;
      result_packed <= '0;
    end else begin
      case (state)
        S_IDLE: begin
          if (start && k_len != 0) begin
            k_len_saved <= k_len;
            k_idx       <= '0;
            state       <= S_CLEAR;
          end
        end

        S_CLEAR: begin
          // One edge with clear_acc=1 resets ALL PE accumulators,
          // input/multiply registers, and weight-column skew pipelines.
          k_idx     <= '0;
          drain_cnt <= '0;
          state     <= S_FEED;
        end

        S_FEED: begin
          // Every clock edge consumes one A[k] and eight B[k,n].
          // k=0..7 -> rows 0..7; k=8 -> row 0 again, without clearing.
          if (k_idx == k_len_saved - 1'b1) begin
            drain_cnt <= '0;
            state     <= S_DRAIN;
          end else begin
            k_idx <= k_idx + 1'b1;
          end
        end

        S_DRAIN: begin
          // Inject zero bubbles, allowing skew and 3-stage PE pipes to drain.
          // Don't use data_valid_out for transaction completion: it is a
          // priming indicator in the original systolic_array implementation.
          if (drain_cnt == DRAIN_CYCLES - 1) begin
            result_packed <= acc_out_packed; // snapshot AFTER full pipeline
            state <= S_DONE;
          end else begin
            drain_cnt <= drain_cnt + 1'b1;
          end
        end

        S_DONE: state <= S_IDLE;  // done lasts one clock cycle
        default: state <= S_IDLE;
      endcase
    end
  end
endmodule
