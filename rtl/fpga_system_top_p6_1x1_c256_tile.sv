`timescale 1ns / 1ps

// P6-C3 fixed local Tile: 8x8x256, 1x1/S2/P0 -> 4x4x8.
// Each M computes four 64-channel chunks using the unchanged P2 K core/array.
// The Host supplies chunk-local data while load_req is high:
//   load_m_addr: output M (0..15), load_c_tile[1:0]: C0/C1/C2/C3.
//   Global input channel = 64*load_c_tile + local channel (0..63).
//   needed activation bytes: ((2*(M/4))*8 + 2*(M%4))*64 + c, c=0..63.
//   weight word address: c=0..63, packed eight INT8 output channels.
// Other activation addresses need not be initialized; they are never read.
// load_done acknowledges completed writes and must not overlap write strobes.
// busy remains high during load waits; permission to write is load_req.
// No automatic DDR/AXI transfer or full-layer/global channel scheduler here.
module fpga_system_top_p6_1x1_c256_tile #(
  parameter int ADDR_WIDTH = 16
)(
  input logic clk, rst_n, start,
  output logic load_req,
  output logic [3:0] load_m_addr,
  output logic [1:0] load_c_tile,
  input logic load_done,
  input logic fmap_wr_en,
  input logic [ADDR_WIDTH-1:0] fmap_wr_addr,
  input logic [7:0] fmap_wr_data,
  input logic weight_wr_en,
  input logic [ADDR_WIDTH-1:0] weight_wr_addr,
  input logic [63:0] weight_wr_data,
  input logic out_rd_en,
  input logic [ADDR_WIDTH-1:0] out_rd_addr,
  input logic [2:0] out_rd_lane_sel,
  output logic signed [31:0] out_rd_data,
  output logic out_rd_lane_valid, out_rd_valid,
  output logic busy, done, any_overflow
);
  typedef enum logic [2:0] {C_IDLE,C_LOAD,C_START_K,C_WAIT_K,C_WRITE,C_DONE} state_t;
  state_t state;
  logic [3:0] m_idx;
  logic [1:0] c_tile_idx;
  logic k_start, k_busy, k_done;
  logic pair_req_en, array_enable, feed_valid, clear_acc, clear_pipe, valid_reset;
  logic data_valid_in, data_valid_out;
  logic [ADDR_WIDTH-1:0] m_addr, k_base, fetch_k_base;
  logic [15:0] act_pair_packed;
  logic [127:0] wgt_pair_packed, wgt_pair_in_packed;
  logic [1:0] act_rsp_valid, wgt_rsp_valid;
  logic [63:0] act_in_packed;
  logic [255:0] acc_out_packed, k_result_packed;
  logic fmap_rd_en, fmap_rd_valid;
  logic [ADDR_WIDTH-1:0] fmap_rd_addr;
  logic [15:0] fmap_rd_data;
  logic act_host_we, weight_host_we;
  logic write_valid;
  logic [ADDR_WIDTH-1:0] write_addr;
  logic [7:0] write_mask;
  logic [255:0] write_data, packed_rd_data;
  logic [7:0] packed_rd_mask;
  logic packed_rd_valid;
  logic [2:0] lane_sel_q;

  assign load_req = (state == C_LOAD);
  assign load_m_addr = m_idx;
  assign load_c_tile = c_tile_idx;
  assign busy = (state != C_IDLE && state != C_DONE);
  assign done = (state == C_DONE);
  assign m_addr = ADDR_WIDTH'(m_idx);
  assign k_start = (state == C_START_K);
  assign act_host_we = rst_n && load_req && fmap_wr_en;
  assign weight_host_we = rst_n && load_req && weight_wr_en;
  assign write_valid = (state == C_WRITE);
  assign write_addr = m_addr;
  assign write_mask = 8'hff;
  assign write_data = k_result_packed;

  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      state<=C_IDLE; m_idx<=0; c_tile_idx<=0;
    end else begin
      case(state)
        C_IDLE: if(start) begin m_idx<=0; c_tile_idx<=0; state<=C_LOAD; end
        C_LOAD: if(load_done && !fmap_wr_en && !weight_wr_en) state<=C_START_K;
        C_START_K: state<=C_WAIT_K;
        C_WAIT_K: if(k_done) begin
          if(c_tile_idx!=2'd3) begin c_tile_idx<=c_tile_idx+2'd1; state<=C_LOAD; end
          else state<=C_WRITE;
        end
        C_WRITE: begin
          if(m_idx==4'd15) state<=C_DONE;
          else begin m_idx<=m_idx+1'b1; c_tile_idx<=0; state<=C_LOAD; end
        end
        C_DONE: state<=C_IDLE;
        default: state<=C_IDLE;
      endcase
    end
  end

  // Chunk data replaces the prior data at LOCAL addresses 0..63. Global K
  // offset mode must stay disabled: global channel selection belongs to Host.
  k_tile_controller_p2 #(.K_ADDR_WIDTH(ADDR_WIDTH), .USE_K_TILE_BASE(1'b0)) u_kctrl (
    .clk(clk), .rst_n(rst_n), .start(k_start),
    .clear_acc_on_start(c_tile_idx==2'd0), .k_len(ADDR_WIDTH'(64)), .k_tile_base({ADDR_WIDTH{1'b0}}),
    .act_pair_packed(act_pair_packed), .act_rsp_valid(act_rsp_valid),
    .wgt_pair_packed(wgt_pair_packed), .wgt_rsp_valid(wgt_rsp_valid),
    .pair_req_en(pair_req_en), .fetch_k_base(fetch_k_base), .k_base(k_base),
    .array_enable(array_enable), .feed_valid(feed_valid),
    .clear_acc(clear_acc), .clear_pipe(clear_pipe), .valid_reset(valid_reset),
    .data_valid_in(data_valid_in), .act_in_packed(act_in_packed),
    .wgt_pair_in_packed(wgt_pair_in_packed), .acc_out_packed(acc_out_packed),
    .busy(k_busy), .done(k_done), .result_packed(k_result_packed)
  );
  systolic_array_p2 u_array (
    .clk(clk), .rst_n(rst_n), .enable(array_enable),
    .clear_acc(clear_acc), .clear_pipe(clear_pipe), .valid_reset(valid_reset),
    .act_in_packed(act_in_packed), .wgt_pair_in_packed(wgt_pair_in_packed),
    .data_valid_in(data_valid_in), .data_valid_out(data_valid_out),
    .acc_out_packed(acc_out_packed), .any_overflow(any_overflow)
  );
  conv_bram_reader_p4_1x1 #(.ADDR_WIDTH(ADDR_WIDTH),.EXTERNAL_ACT_MEM(1'b1)) u_act_reader (
    .clk(clk),.rst_n(rst_n),.m_addr(m_addr),.pair_req_en(pair_req_en),.fetch_k_base(fetch_k_base),
    .wr_en(1'b0),.wr_addr({ADDR_WIDTH{1'b0}}),.wr_data(8'd0),
    .fmap_rd_data(fmap_rd_data),.fmap_rd_en(fmap_rd_en),.fmap_rd_addr(fmap_rd_addr),
    .act_pair_packed(act_pair_packed),.act_rsp_valid(act_rsp_valid)
  );
  conv_activation_bram_shared_p5 #(.ADDR_WIDTH(ADDR_WIDTH),.MAX_BYTES(6400)) u_activation_shared (
    .clk(clk),.rst_n(rst_n),.wr_en(act_host_we),.wr_addr(fmap_wr_addr),.wr_data(fmap_wr_data),
    .rd_en(fmap_rd_en),.rd_addr(fmap_rd_addr),.rd_data(fmap_rd_data),.rd_valid(fmap_rd_valid)
  );
  conv_weight_bram_reader_p5_shared #(.ADDR_WIDTH(ADDR_WIDTH),.K_MAX(576)) u_weight_reader (
    .clk(clk),.rst_n(rst_n),.n_base({ADDR_WIDTH{1'b0}}),.k_len(ADDR_WIDTH'(64)),
    .pair_req_en(pair_req_en),.fetch_k_base(fetch_k_base),
    .wr_en(weight_host_we),.wr_addr(weight_wr_addr),.wr_data(weight_wr_data),
    .wgt_pair_packed(wgt_pair_packed),.wgt_rsp_valid(wgt_rsp_valid)
  );
  output_bram #(.ADDR_WIDTH(ADDR_WIDTH),.DATA_WIDTH(32),.LANES(8),.DEPTH(16)) u_out_mem (
    .clk(clk),.rst_n(rst_n),.wr_en(write_valid),.wr_addr(write_addr),.wr_data(write_data),.wr_mask(write_mask),
    .rd_en(out_rd_en),.rd_addr(out_rd_addr),.rd_data(packed_rd_data),.rd_mask(packed_rd_mask),.rd_valid(packed_rd_valid)
  );
  always_ff @(posedge clk or negedge rst_n)
    if(!rst_n) lane_sel_q<='0;
    else if(out_rd_en) lane_sel_q<=out_rd_lane_sel;
  assign out_rd_data=$signed(packed_rd_data[lane_sel_q*32 +: 32]);
  assign out_rd_valid=packed_rd_valid;
  assign out_rd_lane_valid=packed_rd_valid && packed_rd_mask[lane_sel_q];

  // synthesis translate_off
  initial if(ADDR_WIDTH<13 || ADDR_WIDTH>30)
    $fatal(1,"P6-C3: ADDR_WIDTH must be 13..30");
  always @(posedge clk) if(rst_n) begin
    if((fmap_wr_en || weight_wr_en) && !load_req)
      $fatal(1,"P6-C3: preload permitted only while load_req is high");
    if(load_done && (!load_req || fmap_wr_en || weight_wr_en))
      $fatal(1,"P6-C3: load_done requires pending request and completed writes");
    if(fmap_wr_en && ($isunknown(fmap_wr_addr) || int'($unsigned(fmap_wr_addr))>=4096))
      $fatal(1,"P6-C3: activation byte address outside 8x8x64 local layout");
    if(weight_wr_en && ($isunknown(weight_wr_addr) || int'($unsigned(weight_wr_addr))>=64))
      $fatal(1,"P6-C3: weight address outside local K=64");
    if(load_req && (pair_req_en || array_enable || write_valid))
      $fatal(1,"P6-C3: computation/writeback during Host load wait");
    if(write_valid && c_tile_idx!=2'd3)
      $fatal(1,"P6-C3: attempted writeback before fourth chunk completed");
    if(out_rd_en && (busy || $isunknown({out_rd_addr,out_rd_lane_sel}) ||
                     int'($unsigned(out_rd_addr))>=16))
      $fatal(1,"P6-C3: output read requires idle/done and address 0..15");
  end
  // synthesis translate_on
endmodule
