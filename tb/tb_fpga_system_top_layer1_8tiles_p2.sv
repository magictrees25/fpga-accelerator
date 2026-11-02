`timescale 1ns / 1ps
// Simulation Top. Also add tb_layer1_p2_common.sv to Simulation Sources.
module tb_fpga_system_top_layer1_8tiles_p2;
  tb_layer1_p2_common #(.REGION_H(16),.REGION_W(16),.REGION_N(16),.WRITE_CSV(0)) u_test ();
endmodule

