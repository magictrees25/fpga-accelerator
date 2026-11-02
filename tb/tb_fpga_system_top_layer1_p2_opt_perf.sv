`timescale 1ns / 1ps
// Simulation Top; also add tb_layer1_p2_opt_common.sv.
module tb_fpga_system_top_layer1_p2_opt_perf;
  tb_layer1_p2_opt_common #(.REGION_H(56),.REGION_W(56),.REGION_N(64),.WRITE_CSV(1)) u_test ();
endmodule

