`timescale 1ns / 1ps

// Stage 1: top-left 8x8x8 output Tile, global origin (0,0), channels 0..7.
// Compile tb_conv1_7x7_p1_common.sv as a Simulation Source dependency.
// Expected: 1 Tile, 64 writes, 9408 K FEED, 512/512 outputs; 1296 padded K FEED.
module tb_fpga_system_top_conv1_7x7_tile_p1;
  tb_conv1_7x7_p1_common #(.FULL_LAYER(1'b0)) test();
endmodule
