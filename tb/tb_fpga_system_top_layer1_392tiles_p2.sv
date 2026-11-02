`timescale 1ns / 1ps
// Simulation Top. Also add tb_layer1_p2_common.sv to Simulation Sources.
module tb_fpga_system_top_layer1_392tiles_p2;
  tb_layer1_p2_common #(.REGION_H(56),.REGION_W(56),.REGION_N(64),.WRITE_CSV(0)) u_test ();
endmodule

