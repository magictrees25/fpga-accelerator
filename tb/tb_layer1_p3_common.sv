`timescale 1ns / 1ps

// P3 one-Tile functional/performance testbench.
// Scope: 10x10x64 activation tile (with halo) -> 8x8x8 output, K=576.
// Golden reference is an independent direct-convolution model.
// P3 activation path is checked through Line Buffer reads and BRAM row prefetches.

module tb_layer1_p3_common;

  localparam int ADDR_WIDTH = 16;
  localparam int M_LEN = 64;
  localparam int K_LEN = 576;
  localparam int PAIRS_PER_M = K_LEN / 2;
  localparam int PAIRS_PER_TILE = M_LEN * PAIRS_PER_M;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst_n, start;
  logic fmap_wr_en, weight_wr_en, out_rd_en;
  logic [ADDR_WIDTH-1:0] fmap_wr_addr, weight_wr_addr, out_rd_addr;
  logic [7:0] fmap_wr_data;
  logic [63:0] weight_wr_data;
  logic [2:0] out_rd_lane_sel;

  logic signed [31:0] out_rd_data;
  logic out_rd_lane_valid, out_rd_valid;
  logic busy, done, any_overflow;

  fpga_system_top_layer1_tile_p3 #(
    .ADDR_WIDTH(ADDR_WIDTH)
  ) dut (.*);

  // ---------------- Hierarchical observation points ----------------

  wire clear_acc    = dut.u_compute.clear_acc;
  wire feed_valid   = dut.u_compute.feed_valid;
  wire pair_req_en  = dut.u_compute.pair_req_en;
  wire [15:0] k_base       = dut.u_compute.k_base;
  wire [15:0] fetch_k_base = dut.u_compute.fetch_k_base;
  wire [15:0] m_addr       = dut.u_compute.m_addr;
  wire [15:0] n_base       = dut.u_compute.n_base;

  wire [15:0] act_pair = dut.u_compute.act_pair_packed;
  wire [127:0] wgt_pair = dut.u_compute.wgt_pair_packed;
  wire [1:0] act_valid = dut.u_compute.act_rsp_valid;
  wire [1:0] wgt_valid = dut.u_compute.wgt_rsp_valid;

  wire k_start = dut.u_compute.u_ctrl.k_start;
  wire k_done  = dut.u_compute.u_ctrl.k_done;
  wire array_enable = dut.u_compute.u_ctrl.array_enable;
  wire drain = array_enable && !clear_acc && !feed_valid;

  wire write_valid = dut.write_valid;
  wire [15:0] write_addr = dut.write_addr;
  wire [7:0] write_mask = dut.write_mask;
  wire [255:0] write_data = dut.write_data;

  // ---------------- Independent Golden Model ----------------

  logic signed [7:0] global_fmap [0:55][0:55][0:63];
  logic signed [7:0] global_kernel [0:2][0:2][0:63][0:63];
  logic signed [31:0] golden [0:7][0:7][0:7];

  integer k_hits [0:575];

  function automatic integer activation(input integer m, input integer k);
    integer gy, gx, c;
    begin
      gy = m / 8 + k / 192 - 1;
      gx = m % 8 + (k / 64) % 3 - 1;
      c  = k % 64;
      if (gy < 0 || gy >= 56 || gx < 0 || gx >= 56)
        activation = 0;
      else
        activation = int'($signed(global_fmap[gy][gx][c]));
    end
  endfunction

  function automatic integer weight_value(input integer k, input integer n);
    begin
      weight_value =
        int'($signed(global_kernel[k/192][(k/64)%3][k%64][n]));
    end
  endfunction

  task automatic prepare_golden;
    integer gy, gx;
    integer signed sum;
    begin
      for (int y=0; y<56; y++)
        for (int x=0; x<56; x++)
          for (int c=0; c<64; c++)
            global_fmap[y][x][c] = 8'(((17*y + 11*x + 7*c) % 127) - 63);

      for (int ky=0; ky<3; ky++)
        for (int kx=0; kx<3; kx++)
          for (int c=0; c<64; c++)
            for (int n=0; n<64; n++)
              global_kernel[ky][kx][c][n] =
                8'(((13*ky + 7*kx + 3*c + 5*n) % 19) - 9);

      $display("INFO: Build independent direct-convolution Golden Model");

      for (int oy=0; oy<8; oy++)
        for (int ox=0; ox<8; ox++)
          for (int n=0; n<8; n++) begin
            sum = 0;
            for (int ky=0; ky<3; ky++)
              for (int kx=0; kx<3; kx++)
                for (int c=0; c<64; c++) begin
                  gy = oy + ky - 1;
                  gx = ox + kx - 1;
                  if (gy >= 0 && gy < 56 && gx >= 0 && gx < 56)
                    sum += int'($signed(global_fmap[gy][gx][c])) *
                           int'($signed(global_kernel[ky][kx][c][n]));
                end
            golden[oy][ox][n] = sum;
          end

      if (golden[0][0][0] !== 2453 || golden[7][7][7] !== 1616)
        $fatal(1, "FAIL: Golden Model anchor mismatch");
    end
  endtask

  // ---------------- P3 address and prefetch checks ----------------

  task automatic check_activation_request(
    input integer em,
    input integer ek
  );
    integer input_row, input_col, expected_lb_addr;
    begin
      input_row = em/8 + ek/192;
      input_col = em%8 + (ek/64)%3;

      // Four-row circular buffer:
      // slot = input_row % 4
      // address = slot*320 + input_col*32 + channel_pair
      expected_lb_addr =
        ((input_row % 4) * 10 + input_col) * 32 + ((ek % 64) / 2);

      if (dut.u_compute.u_act_reader.lb_rd_en !== 1'b1)
        $fatal(1, "FAIL: P3 Line Buffer read enable low, M=%0d K=%0d", em, ek);

      if (dut.u_compute.u_act_reader.lb_rd_addr !== 11'(expected_lb_addr))
        $fatal(1,
          "FAIL: P3 Line Buffer address M=%0d K=%0d got=%0d expected=%0d",
          em, ek, dut.u_compute.u_act_reader.lb_rd_addr, expected_lb_addr);

      // Weight Reader remains the P2.1 implementation.
      if (dut.u_compute.u_weight_reader.bank_rd_en !== 2'b11 ||
          dut.u_compute.u_weight_reader.bank_rd_addr[0] !== 16'(ek/2) ||
          dut.u_compute.u_weight_reader.bank_rd_addr[1] !== 16'(ek/2))
        $fatal(1, "FAIL: Weight Reader request/address M=%0d K=%0d", em, ek);
    end
  endtask

  // ---------------- Counters ----------------

  integer fmap_load_count, weight_load_count;
  integer request_count, line_buffer_read_count;
  integer prefetch_trigger_count, prefetch_read_count, prefetch_write_count;

  integer feed_pair_count, clear_count, drain_count;
  integer launch_count, kdone_count, output_write_count;
  integer round_pairs, round_drains;
  integer readback_count;

  integer m_idx, k_idx, got_value, expected_value;
  integer idx, row_idx, col_idx, expected_addr;
  integer waited;

  logic round_open, computing, start_q, done_q, finished;
  logic [255:0] result_snapshot;
  logic [63:0] expected_injection;

  longint unsigned tick_count, start_tick, done_tick, compute_cycles;

  // ---------------- Main monitor ----------------

  always @(posedge clk) begin : monitor
    bit count_compute;
    integer em, ek, p, n;

    if (!rst_n) begin
      tick_count = 0;
      computing = 0;
      start_q = 0;
      done_q = 0;
    end else begin
      tick_count = tick_count + 1;
      count_compute = computing;

      if ($isunknown({clear_acc, feed_valid, pair_req_en,
                      write_valid, busy, done, any_overflow}) ||
          any_overflow !== 1'b0)
        $fatal(1, "FAIL: Unknown control or arithmetic overflow");

      if ((clear_acc || feed_valid || write_valid || k_done || drain) &&
          !computing)
        $fatal(1, "FAIL: Compute transaction outside start/done window");

      if (start) begin
        if (start_q || busy || done || computing ||
            fmap_load_count != 6400 || weight_load_count != 576)
          $fatal(1, "FAIL: Start before preload completion or while busy");
        computing = 1;
        start_tick = tick_count;
      end
      start_q = start;

      if (count_compute)
        compute_cycles = compute_cycles + 1;

      // Host-side preload checking.
      if (weight_wr_en) begin
        if (weight_wr_addr !== 16'(weight_load_count) ||
            weight_load_count >= 576)
          $fatal(1, "FAIL: Weight preload address/order");
        weight_load_count = weight_load_count + 1;
      end

      if (fmap_wr_en) begin
        if (fmap_wr_addr !== 16'(fmap_load_count) ||
            fmap_load_count >= 6400)
          $fatal(1, "FAIL: Activation preload address/order");
        fmap_load_count = fmap_load_count + 1;
      end

      // P3 line-buffer read requests are driven by pair_req_en.
      if (pair_req_en) begin
        check_activation_request(int'(m_addr), int'(fetch_k_base));
        request_count = request_count + 1;
      end

      if (dut.u_compute.u_act_reader.lb_rd_en)
        line_buffer_read_count = line_buffer_read_count + 1;

      // BRAM reads are now row prefetches, not per-K compute requests.
      if (dut.u_compute.u_act_reader.prefetch_trigger)
        prefetch_trigger_count = prefetch_trigger_count + 1;

      if (dut.u_compute.u_act_reader.bank_rd_en === 2'b11) begin
        idx = prefetch_read_count;
        if (idx >= 1920)
          $fatal(1, "FAIL: Too many Activation BRAM prefetch reads");

        row_idx = 4 + idx/320;
        col_idx = idx%320;
        expected_addr = row_idx*320 + col_idx;

        if (dut.u_compute.u_act_reader.prefetch_row !== 4'(row_idx) ||
            dut.u_compute.u_act_reader.prefetch_index !== 9'(col_idx) ||
            dut.u_compute.u_act_reader.bank_rd_addr[0] !== 16'(expected_addr) ||
            dut.u_compute.u_act_reader.bank_rd_addr[1] !== 16'(expected_addr))
          $fatal(1, "FAIL: Activation BRAM prefetch address/order");

        prefetch_read_count = prefetch_read_count + 1;
      end else if (dut.u_compute.u_act_reader.bank_rd_en !== 2'b00) begin
        $fatal(1, "FAIL: Activation BRAM banks not read together");
      end

      // Both Line Buffer banks are written together only for prefetched rows.
      if (dut.u_compute.u_act_reader.lb_wr_en === 2'b11) begin
        idx = prefetch_write_count;
        if (idx >= 1920)
          $fatal(1, "FAIL: Too many Line Buffer prefetch writes");

        row_idx = 4 + idx/320;
        col_idx = idx%320;
        expected_addr = (row_idx%4)*320 + col_idx;

        if (dut.u_compute.u_act_reader.lb_wr_addr !== 11'(expected_addr))
          $fatal(1, "FAIL: Line Buffer prefetch write address/order");

        prefetch_write_count = prefetch_write_count + 1;
      end

      em = int'(m_addr);

      if (clear_acc) begin
        if (round_open || output_write_count >= 64 ||
            clear_count != output_write_count ||
            feed_pair_count != output_write_count*288 ||
            k_base !== 16'd0)
          $fatal(1, "FAIL: K round CLEAR sequence");

        round_open = 1;
        round_pairs = 0;
        round_drains = 0;
        clear_count = clear_count + 1;
      end

      if (round_open && !clear_acc && round_pairs < 288 && !feed_valid)
        $fatal(1, "FAIL: Bubble detected in paired K feed");

      if (feed_valid) begin
        ek = 2*round_pairs;

        if (!round_open || round_pairs >= 288 ||
            k_base !== 16'(ek) || m_addr !== 16'(em) || n_base !== 0 ||
            act_valid !== 2'b11 || wgt_valid !== 2'b11 ||
            dut.u_compute.u_act_reader.rsp_k_base !== 16'(ek) ||
            dut.u_compute.u_act_reader.rsp_m_addr !== 16'(em) ||
            dut.u_compute.u_weight_reader.rsp_k_base !== 16'(ek) ||
            dut.u_compute.u_weight_reader.rsp_n_base !== 0)
          $fatal(1, "FAIL: FEED index/response tag mismatch M=%0d K=%0d", em, ek);

        expected_injection = 64'b0;

        for (p=0; p<2; p=p+1) begin
          expected_value = activation(em, ek+p);
          got_value = int'($signed(act_pair[p*8 +: 8]));

          if (got_value !== expected_value)
            $fatal(1,
              "FAIL: Activation data M=%0d K=%0d got=%0d expected=%0d",
              em, ek+p, got_value, expected_value);

          expected_injection[((ek+p)%8)*8 +: 8] = 8'(expected_value);
          k_hits[ek+p] = k_hits[ek+p] + 1;

          for (n=0; n<8; n=n+1) begin
            expected_value = weight_value(ek+p, n);
            got_value = int'($signed(wgt_pair[(p*8+n)*8 +: 8]));
            if (got_value !== expected_value)
              $fatal(1,
                "FAIL: Weight data M=%0d K=%0d N=%0d got=%0d expected=%0d",
                em, ek+p, n, got_value, expected_value);
          end
        end

        if (dut.u_compute.u_ctrl.act_in_packed !== expected_injection ||
            dut.u_compute.u_ctrl.wgt_pair_in_packed !== wgt_pair)
          $fatal(1, "FAIL: K injection mapping");

        round_pairs = round_pairs + 1;
        feed_pair_count = feed_pair_count + 1;
      end

      if (!pair_req_en &&
          dut.u_compute.u_weight_reader.bank_rd_en !== 2'b00)
        $fatal(1, "FAIL: Weight BRAM read request leaked");

      if (drain) begin
        if (!round_open || round_pairs != 288 ||
            dut.u_compute.u_ctrl.act_in_packed !== 0 ||
            dut.u_compute.u_ctrl.wgt_pair_in_packed !== 0)
          $fatal(1, "FAIL: DRAIN boundary");

        round_drains = round_drains + 1;
        drain_count = drain_count + 1;
      end

      if (k_start)
        launch_count = launch_count + 1;

      if (k_done) begin
        if (!round_open || round_pairs != 288 || round_drains != 13)
          $fatal(1, "FAIL: K_DONE phase count M=%0d", em);

        result_snapshot = dut.u_compute.u_ctrl.k_result_packed;

        for (n=0; n<8; n=n+1)
          if (int'($signed(result_snapshot[n*32 +: 32])) !==
              golden[em/8][em%8][n])
            $fatal(1, "FAIL: Golden result M=%0d N=%0d", em, n);

        kdone_count = kdone_count + 1;
      end

      if (write_valid) begin
        if (!round_open || round_pairs != 288 || round_drains != 13 ||
            output_write_count >= 64 ||
            write_addr !== 16'(em) ||
            dut.write_m_addr !== 16'(em) ||
            dut.write_n_base !== 0 ||
            write_mask !== 8'hff ||
            write_data !== result_snapshot)
          $fatal(1, "FAIL: Output BRAM write metadata/data");

        for (n=0; n<8; n=n+1)
          if (int'($signed(write_data[n*32 +: 32])) !==
              golden[em/8][em%8][n])
            $fatal(1, "FAIL: Output write Golden mismatch M=%0d N=%0d", em, n);

        output_write_count = output_write_count + 1;
        round_open = 0;
        round_pairs = 0;
        round_drains = 0;
      end

      #1;
      if (done && done_q)
        $fatal(1, "FAIL: done wider than one clock");

      if (done) begin
        if (!computing || output_write_count != 64)
          $fatal(1, "FAIL: Unexpected done");
        computing = 0;
        done_tick = tick_count;
      end
      done_q = done;
    end
  end

  // ---------------- Test stimulus ----------------

  task automatic load_one_tile;
    integer gy, gx;
    begin
      // Load 576 K words; each 64-bit word contains 8 output-channel weights.
      for (int k=0; k<576; k++) begin
        @(negedge clk);
        weight_wr_en = 1'b1;
        weight_wr_addr = 16'(k);
        for (int n=0; n<8; n++)
          weight_wr_data[n*8 +: 8] = 8'(weight_value(k,n));
      end
      @(negedge clk);
      weight_wr_en = 1'b0;

      // Load the 10x10x64 input patch including one-pixel zero-padding halo.
      for (int y=0; y<10; y++)
        for (int x=0; x<10; x++)
          for (int c=0; c<64; c++) begin
            @(negedge clk);
            fmap_wr_en = 1'b1;
            fmap_wr_addr = 16'((y*10+x)*64+c);
            gy = y-1;
            gx = x-1;
            if (gy < 0 || gy >= 56 || gx < 0 || gx >= 56)
              fmap_wr_data = 8'd0;
            else
              fmap_wr_data = global_fmap[gy][gx][c];
          end

      @(negedge clk);
      fmap_wr_en = 1'b0;
    end
  endtask

  task automatic read_and_check_outputs;
    integer got, expected;
    begin
      for (int m=0; m<64; m++)
        for (int n=0; n<8; n++) begin
          @(negedge clk);
          out_rd_en = 1'b1;
          out_rd_addr = 16'(m);
          out_rd_lane_sel = 3'(n);

          @(posedge clk);
          #2;

          if (out_rd_valid !== 1'b1 || out_rd_lane_valid !== 1'b1)
            $fatal(1, "FAIL: Output BRAM read valid M=%0d N=%0d", m, n);

          got = int'($signed(out_rd_data));
          expected = golden[m/8][m%8][n];

          if (got !== expected)
            $fatal(1,
              "FAIL: Output BRAM data M=%0d N=%0d got=%0d expected=%0d",
              m, n, got, expected);

          readback_count = readback_count + 1;
        end

      @(negedge clk);
      out_rd_en = 1'b0;
      @(posedge clk);
      #2;
      if (out_rd_valid !== 1'b0)
        $fatal(1, "FAIL: Output BRAM stale read-valid");
    end
  endtask

  // ---------------- Initial sequence ----------------

  initial begin : stimulus
    rst_n = 1'b0;
    start = 1'b0;
    fmap_wr_en = 1'b0;
    fmap_wr_addr = '0;
    fmap_wr_data = '0;
    weight_wr_en = 1'b0;
    weight_wr_addr = '0;
    weight_wr_data = '0;
    out_rd_en = 1'b0;
    out_rd_addr = '0;
    out_rd_lane_sel = '0;

    fmap_load_count = 0;
    weight_load_count = 0;
    request_count = 0;
    line_buffer_read_count = 0;
    prefetch_trigger_count = 0;
    prefetch_read_count = 0;
    prefetch_write_count = 0;
    feed_pair_count = 0;
    clear_count = 0;
    drain_count = 0;
    launch_count = 0;
    kdone_count = 0;
    output_write_count = 0;
    round_pairs = 0;
    round_drains = 0;
    round_open = 0;
    readback_count = 0;
    result_snapshot = '0;
    expected_injection = '0;
    tick_count = 0;
    start_tick = 0;
    done_tick = 0;
    compute_cycles = 0;
    finished = 0;

    for (int k=0; k<576; k++)
      k_hits[k] = 0;

    prepare_golden();

    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    $display("INFO: Start P3 one-Tile preload");
    load_one_tile();

    if (fmap_load_count != 6400 || weight_load_count != 576)
      $fatal(1, "FAIL: Preload count activation=%0d weight=%0d",
             fmap_load_count, weight_load_count);

    repeat (2) @(negedge clk);
    $display("INFO: Start P3 one-Tile computation");

    start = 1'b1;
    @(negedge clk);
    start = 1'b0;

    waited = 0;
    while (done !== 1'b1 && waited < 30000) begin
      @(posedge clk);
      #2;
      waited = waited + 1;
    end

    if (done !== 1'b1)
      $fatal(1, "TIMEOUT: P3 one-Tile compute");

    repeat (3) @(negedge clk);
    read_and_check_outputs();

    // ---------------- Final assertions ----------------

    if (clear_count != 64 ||
        feed_pair_count != PAIRS_PER_TILE ||
        request_count != PAIRS_PER_TILE ||
        line_buffer_read_count != PAIRS_PER_TILE ||
        drain_count != 64*13 ||
        launch_count != 64 ||
        kdone_count != 64 ||
        output_write_count != 64 ||
        readback_count != 512)
      $fatal(1, "FAIL: Compute/output event counts");

    for (int k=0; k<576; k++)
      if (k_hits[k] != 64)
        $fatal(1, "FAIL: K coverage k=%0d hits=%0d", k, k_hits[k]);

    if (prefetch_trigger_count != 6 ||
        prefetch_read_count != 1920 ||
        prefetch_write_count != 1920)
      $fatal(1,
        "FAIL: Prefetch counts trigger=%0d BRAM_reads=%0d LB_writes=%0d",
        prefetch_trigger_count, prefetch_read_count, prefetch_write_count);

    if (round_open || computing || busy !== 1'b0 ||
        done !== 1'b0 || any_overflow !== 1'b0)
      $fatal(1, "FAIL: Final DUT state");

    if (compute_cycles !=
        clear_count + feed_pair_count + drain_count +
        launch_count + kdone_count + output_write_count)
      $fatal(1, "FAIL: Compute-cycle accounting");

    $display("==============================================");
    $display("PASS: P3 one-Tile functional verification");
    $display("PASS: 512/512 output values match Golden Model");
    $display("PASS: Activation Line Buffer reads = %0d", line_buffer_read_count);
    $display("PASS: BRAM prefetch operations = %0d rows", prefetch_trigger_count);
    $display("PASS: BRAM prefetch pair reads = %0d", prefetch_read_count);
    $display("PASS: Line Buffer pair writes = %0d", prefetch_write_count);
    $display("INFO: P3 compute cycles = %0d", compute_cycles);
    $display("INFO: P2.1 reference compute cycles = 19520");
    $display("==============================================");

    finished = 1'b1;
    $finish;
  end

  initial begin
    repeat (40000) @(posedge clk);
    if (!finished)
      $fatal(1, "TIMEOUT: P3 one-Tile testbench");
  end

endmodule
