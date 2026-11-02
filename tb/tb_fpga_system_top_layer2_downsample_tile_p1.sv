`timescale 1ns / 1ps

// Stage 1: single 4x4x8 output Tile at (0,0), channels 0..7.
// Add tb_layer2_downsample_p1_common.sv to Simulation Sources too.
// Expected: 1 Tile, 16 writes, 1024 K FEED cycles, 128/128 outputs.
module tb_fpga_system_top_layer2_downsample_tile_p1;
  tb_layer2_downsample_p1_common #(.FULL_LAYER(1'b0)) test();
endmodule
