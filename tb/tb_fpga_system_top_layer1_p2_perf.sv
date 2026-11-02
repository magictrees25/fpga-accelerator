`timescale 1ns / 1ps
// Simulation Top. Also add tb_layer1_p2_common.sv to Simulation Sources.
module tb_fpga_system_top_layer1_p2_perf;
  tb_layer1_p2_common #(.REGION_H(56),.REGION_W(56),.REGION_N(64),.WRITE_CSV(1)) u_test ();
endmodule

