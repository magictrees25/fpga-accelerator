`timescale 1ns / 1ps

// GEMM M-dimension scheduler, stage 1 of a complete GEMM controller.
// Calculates C[m, 0:7] for m=0..m_len-1 using the EXISTING K controller.
// N is fixed to ARRAY_COLS=8 for now.  K is configurable.
//
// Input protocol:
//   act_data = A[m_addr][k_addr]
//   wgt_row_packed[n*8 +: 8] = B[k_addr][n]
//   Both sources are COMBINATIONAL (zero-cycle read latency).
//
// Output write protocol:
//   On a rising edge with write_valid=1, external C memory should write
//   write_data (eight INT32 values) to output row write_addr.
//   This is a synthesizable write interface; TB supplies only the memory.
//
// start: pulse for one cycle while idle; done: one-cycle pulse after the
// last output row was written.  This module owns K controller + PE array.
module gemm_m_controller
  import pkg_accelerator::*;
#(
  parameter int ADDR_WIDTH = 16
)(
  input  logic clk,
  input  logic rst_n,
  input  logic start,
  input  logic [ADDR_WIDTH-1:0] m_len,
  input  logic [ADDR_WIDTH-1:0] k_len,

  output logic [ADDR_WIDTH-1:0] m_addr,
  output logic [ADDR_WIDTH-1:0] k_addr,
  input  logic signed [ACT_WIDTH-1:0] act_data,
  input  logic [WGT_WIDTH*ARRAY_COLS-1:0] wgt_row_packed,

  output logic write_valid,
  output logic [ADDR_WIDTH-1:0] write_addr,
  output logic [ACC_WIDTH*ARRAY_COLS-1:0] write_data,
  output logic busy,
  output logic done,
  output logic any_overflow
);

  typedef enum logic [2:0] {
    M_IDLE,
    M_START_K,
    M_WAIT_K,
    M_WRITE,
    M_DONE
  } m_state_t;
  m_state_t state;

  logic [ADDR_WIDTH-1:0] m_idx;
  logic [ADDR_WIDTH-1:0] m_len_saved;
  logic [ADDR_WIDTH-1:0] k_len_saved;
  logic k_start;
  logic k_done;
  logic k_busy;
  logic array_enable;
  logic feed_valid;
  logic clear_acc;
  logic valid_reset;
  logic data_valid_in;
  logic data_valid_out;
  logic [ACT_WIDTH*ARRAY_ROWS-1:0] act_in_packed;
  logic signed [WGT_WIDTH-1:0] wgt_in [ARRAY_COLS];
  logic [ACC_WIDTH*ARRAY_COLS-1:0] acc_out_packed;
  logic [ACC_WIDTH*ARRAY_COLS-1:0] k_result_packed;

  assign m_addr      = m_idx;
  assign k_start     = (state == M_START_K);
  assign write_valid = (state == M_WRITE);
  assign write_addr  = m_idx;
  assign write_data  = k_result_packed;
  assign busy        = (state == M_START_K || state == M_WAIT_K || state == M_WRITE);
  assign done        = (state == M_DONE);

  k_tile_controller #(.K_ADDR_WIDTH(ADDR_WIDTH)) u_kctrl (
    .clk(clk), .rst_n(rst_n),
    .start(k_start), .k_len(k_len_saved),
    .k_addr(k_addr), .act_data(act_data), .wgt_row_packed(wgt_row_packed),
    .array_enable(array_enable), .feed_valid(feed_valid),
    .clear_acc(clear_acc), .valid_reset(valid_reset),
    .data_valid_in(data_valid_in), .act_in_packed(act_in_packed),
    .wgt_in(wgt_in), .acc_out_packed(acc_out_packed),
    .busy(k_busy), .done(k_done), .result_packed(k_result_packed)
  );

  systolic_array u_array (
    .clk(clk), .rst_n(rst_n),
    .enable(array_enable), .clear_acc(clear_acc), .valid_reset(valid_reset),
    .act_in_packed(act_in_packed), .wgt_in(wgt_in),
    .data_valid_in(data_valid_in), .data_valid_out(data_valid_out),
    .acc_out_packed(acc_out_packed), .any_overflow(any_overflow)
  );

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state       <= M_IDLE;
      m_idx       <= '0;
      m_len_saved <= '0;
      k_len_saved <= '0;
    end else begin
      case (state)
        M_IDLE: begin
          if (start && m_len != '0 && k_len != '0) begin
            m_len_saved <= m_len;
            k_len_saved <= k_len;
            m_idx       <= '0;
            state       <= M_START_K;
          end
        end
        M_START_K: state <= M_WAIT_K;  // one cycle k_start pulse
        M_WAIT_K: begin
          if (k_done) state <= M_WRITE;
        end
        M_WRITE: begin
          // write_valid/write_addr/write_data stable for this rising edge.
          // On the same edge the next m_idx is selected for the next K run.
          if (m_idx == m_len_saved - 1'b1) begin
            state <= M_DONE;
          end else begin
            m_idx <= m_idx + 1'b1;
            state <= M_START_K;
          end
        end
        M_DONE: state <= M_IDLE;
        default: state <= M_IDLE;
      endcase
    end
  end
endmodule
