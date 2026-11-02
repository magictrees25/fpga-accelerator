`timescale 1ns / 1ps

// Stage 2: full layer, one DUT/reset, 784 sequential 4x4x8 output Tiles.
// Add tb_layer2_downsample_p1_common.sv to Simulation Sources too.
// Expected: 784 Tile, 12544 writes, 802816 K FEED cycles, 100352/100352 outputs.
module tb_fpga_system_top_layer2_downsample_784tiles_p1;
  tb_layer2_downsample_p1_common #(.FULL_LAYER(1'b1)) test();
endmodule
