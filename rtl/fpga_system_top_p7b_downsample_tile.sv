`timescale 1ns / 1ps
// P7-B layer2.0: one unchanged P=2 PE/K core, three Host-selected phases.
// cfg_mode 0: projection 1x1/S2, Cin64, K64, one C chunk, signed output.
// cfg_mode 1: conv1 3x3/S2, Cin64, K576, one C chunk, ReLU INT8 output.
// cfg_mode 2: conv2 3x3/S1, Cin128, 2xK576, signed skip + ReLU output.
// All modes: M16/N8. On each load_req Host writes one M/C window at linear
// byte K addresses and one 8-lane weight word per K. Spatial/halo coordinates
// are a Host responsibility; this is a patch interface, not a full-map Reader.
// Layer/channel/spatial scheduling remains Host/TB driven. All old RTL intact.
module fpga_system_top_p7b_downsample_tile (
    input logic clk,rst_n,start,
    input logic [1:0] cfg_mode,
    input logic [255:0] channel_bias,channel_multiplier,
    output logic load_req,
    output logic [3:0] load_m_addr,
    output logic load_c_tile,
    input logic load_done,
    input logic fmap_wr_en,
    input logic [15:0] fmap_wr_addr,
    input logic [7:0] fmap_wr_data,
    input logic weight_wr_en,
    input logic [15:0] weight_wr_addr,
    input logic [63:0] weight_wr_data,
    input logic skip_wr_en,
    input logic [3:0] skip_wr_addr,
    input logic [255:0] skip_wr_data,
    input logic out_rd_en,
    input logic [15:0] out_rd_addr,
    input logic [2:0] out_rd_lane_sel,
    output logic signed [31:0] out_rd_data,
    output logic out_rd_valid,out_rd_lane_valid,busy,done,any_overflow
);
    typedef enum logic [2:0] {C_IDLE,C_LOAD,C_START,C_WAIT,C_WRITE,C_POST,C_DONE} state_t;
    state_t state;
    logic [1:0] mode_q;
    logic [3:0] m_idx;
    logic c_idx;
    logic [255:0] bias_q,mult_q;
    logic [15:0] k_len,k_base,fetch_k_base;
    logic k_start,k_busy,k_done,pair_req_en,array_enable,feed_valid;
    logic clear_acc,clear_pipe,valid_reset,data_valid_in,data_valid_out;
    logic [15:0] act_pair_packed;
    logic [127:0] wgt_pair_packed,wgt_pair_in_packed;
    logic [1:0] act_rsp_valid,wgt_rsp_valid;
    logic act_valid;
    logic [63:0] act_in_packed;
    logic [255:0] acc_out_packed,k_result_packed;
    logic array_overflow,raw_valid;
    logic [7:0] post_valid,post_overflow;
    logic [255:0] post_data,rd_data;
    logic [7:0] rd_mask;
    logic [2:0] lane_q;
    (* ram_style="distributed" *) logic [255:0] skip_mem[0:15];

    assign load_req=(state==C_LOAD);
    assign load_m_addr=m_idx;
    assign load_c_tile=c_idx;
    assign busy=(state!=C_IDLE && state!=C_DONE);
    assign done=(state==C_DONE);
    assign k_start=(state==C_START);
    assign raw_valid=(state==C_WRITE);
    assign k_len=(mode_q==0)?16'd64:16'd576;
    assign act_rsp_valid={2{act_valid}};
    assign any_overflow=array_overflow || (|(post_overflow & post_valid));

    always_ff @(posedge clk) if(skip_wr_en && state==C_IDLE)
        skip_mem[skip_wr_addr]<=skip_wr_data;
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=C_IDLE;mode_q<=0;m_idx<=0;c_idx<=0;bias_q<=0;mult_q<=0;lane_q<=0;
        end else begin
            if(out_rd_en) lane_q<=out_rd_lane_sel;
            case(state)
                C_IDLE: if(start) begin
                    mode_q<=cfg_mode;bias_q<=channel_bias;mult_q<=channel_multiplier;
                    m_idx<=0;c_idx<=0;state<=C_LOAD;
                end
                C_LOAD: if(load_done && !fmap_wr_en && !weight_wr_en) state<=C_START;
                C_START: state<=C_WAIT;
                C_WAIT: if(k_done) begin
                    if(mode_q==2 && !c_idx) begin c_idx<=1;state<=C_LOAD;end
                    else state<=C_WRITE;
                end
                C_WRITE: state<=C_POST;
                // Hold M and skip address stable until RAM accepts postprocess.
                C_POST: if(post_valid[0]) begin
                    if(m_idx==15) state<=C_DONE;
                    else begin m_idx<=m_idx+1'b1;c_idx<=0;state<=C_LOAD;end
                end
                C_DONE: state<=C_IDLE;
                default: state<=C_IDLE;
            endcase
        end
    end

    k_tile_controller_p2 #(.K_ADDR_WIDTH(16),.USE_K_TILE_BASE(1'b0)) u_kctrl (
        .clk(clk),.rst_n(rst_n),.start(k_start),.clear_acc_on_start(!c_idx),
        .k_len(k_len),.k_tile_base(16'd0),
        .act_pair_packed(act_pair_packed),.act_rsp_valid(act_rsp_valid),
        .wgt_pair_packed(wgt_pair_packed),.wgt_rsp_valid(wgt_rsp_valid),
        .pair_req_en(pair_req_en),.fetch_k_base(fetch_k_base),.k_base(k_base),
        .array_enable(array_enable),.feed_valid(feed_valid),.clear_acc(clear_acc),
        .clear_pipe(clear_pipe),.valid_reset(valid_reset),.data_valid_in(data_valid_in),
        .act_in_packed(act_in_packed),.wgt_pair_in_packed(wgt_pair_in_packed),
        .acc_out_packed(acc_out_packed),.busy(k_busy),.done(k_done),.result_packed(k_result_packed)
    );
    systolic_array_p2 u_array (
        .clk(clk),.rst_n(rst_n),.enable(array_enable),.clear_acc(clear_acc),
        .clear_pipe(clear_pipe),.valid_reset(valid_reset),.act_in_packed(act_in_packed),
        .wgt_pair_in_packed(wgt_pair_in_packed),.data_valid_in(data_valid_in),
        .data_valid_out(data_valid_out),.acc_out_packed(acc_out_packed),.any_overflow(array_overflow)
    );
    // Flat local window: byte k -> bank k%2, bank address k/2.
    // Shared BRAM rd_valid is synchronous and feeds the unchanged K controller.
    conv_activation_bram_shared_p5 #(.ADDR_WIDTH(16),.MAX_BYTES(576)) u_activation (
        .clk(clk),.rst_n(rst_n),.wr_en(fmap_wr_en && load_req),.wr_addr(fmap_wr_addr),.wr_data(fmap_wr_data),
        .rd_en(pair_req_en),.rd_addr(fetch_k_base >> 1),.rd_data(act_pair_packed),.rd_valid(act_valid)
    );
    conv_weight_bram_reader_p5_shared #(.ADDR_WIDTH(16),.K_MAX(576)) u_weight (
        .clk(clk),.rst_n(rst_n),.n_base(16'd0),.k_len(k_len),
        .pair_req_en(pair_req_en),.fetch_k_base(fetch_k_base),
        .wr_en(weight_wr_en && load_req),.wr_addr(weight_wr_addr),.wr_data(weight_wr_data),
        .wgt_pair_packed(wgt_pair_packed),.wgt_rsp_valid(wgt_rsp_valid)
    );
    for(genvar lane=0;lane<8;lane++) begin : gen_post
        requant_branch_p7b u_post (
            .clk(clk),.rst_n(rst_n),.in_valid(raw_valid),
            .in_acc($signed(k_result_packed[lane*32+:32])),
            .in_bias($signed(bias_q[lane*32+:32])),.in_multiplier($signed(mult_q[lane*32+:32])),
            .in_skip($signed(skip_mem[m_idx][lane*32+:32])),
            .in_residual_en(mode_q==2),.in_relu_en(mode_q!=0),
            .out_valid(post_valid[lane]),.out_overflow(post_overflow[lane]),.out_data(post_data[lane*32+:32])
        );
    end
    output_bram #(.ADDR_WIDTH(16),.DATA_WIDTH(32),.LANES(8),.DEPTH(16)) u_output (
        .clk(clk),.rst_n(rst_n),.wr_en(post_valid[0]),.wr_addr({12'd0,m_idx}),
        .wr_data(post_data),.wr_mask(8'hff),.rd_en(out_rd_en),.rd_addr(out_rd_addr),
        .rd_data(rd_data),.rd_mask(rd_mask),.rd_valid(out_rd_valid)
    );
    assign out_rd_data=$signed(rd_data[lane_q*32+:32]);
    assign out_rd_lane_valid=out_rd_valid && rd_mask[lane_q];

    // synthesis translate_off
    always @(posedge clk) if(rst_n) begin
        if(start && (state!=C_IDLE || $isunknown(cfg_mode) || cfg_mode==3 || skip_wr_en))
            $fatal(1,"P7-B illegal start/mode");
        if((fmap_wr_en || weight_wr_en || load_done) && !load_req)
            $fatal(1,"P7-B writes/ack outside load request");
        if((fmap_wr_en && weight_wr_en) || (load_done && (fmap_wr_en || weight_wr_en)))
            $fatal(1,"P7-B overlapping writes/ack");
        if(fmap_wr_en && ($isunknown(fmap_wr_addr) || fmap_wr_addr>=k_len))
            $fatal(1,"P7-B local activation address exceeds chunk");
        if(weight_wr_en && ($isunknown(weight_wr_addr) || weight_wr_addr>=k_len))
            $fatal(1,"P7-B local weight address exceeds chunk");
        if(skip_wr_en && (state!=C_IDLE || $isunknown(skip_wr_addr)))
            $fatal(1,"P7-B skip preload requires idle");
        if(out_rd_en && (busy || $isunknown({out_rd_addr,out_rd_lane_sel}) || out_rd_addr>=16))
            $fatal(1,"P7-B output read requires completed tile");
        if(load_req && (array_enable || pair_req_en || raw_valid || post_valid[0]))
            $fatal(1,"P7-B compute during Host wait");
        if(raw_valid && mode_q==2 && !c_idx) $fatal(1,"P7-B premature partial write");
        if(post_valid!==8'h00 && post_valid!==8'hff) $fatal(1,"P7-B lane valid mismatch");
        if(post_valid[0] && state!=C_POST) $fatal(1,"P7-B unexpected quantized write");
    end
    // synthesis translate_on
endmodule
