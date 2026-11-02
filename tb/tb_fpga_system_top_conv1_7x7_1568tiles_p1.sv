`timescale 1ns / 1ps

// Stage 2: 1568 sequential Tiles, one DUT, one reset, all borders included.
// Compile tb_conv1_7x7_p1_common.sv as a Simulation Source dependency.
// Expected: 100352 writes, 14751744 K FEED, 802816/802816 INT32 outputs.
module tb_fpga_system_top_conv1_7x7_1568tiles_p1;
  tb_conv1_7x7_p1_common #(.FULL_LAYER(1'b1)) test();
endmodule
