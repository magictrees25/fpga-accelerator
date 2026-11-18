`timescale 1ns / 1ps
// Add tb_p6_1x1_c256_common.sv to Simulation Sources as well.
module tb_fpga_system_top_p6_1x1_c256_tile;
  tb_p6_1x1_c256_common #(.FULL_LAYER(1'b0)) test();
endmodule
