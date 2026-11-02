1) Back up existing conv_addr_gen.sv.
2) Replace its CONTENTS with conv_addr_gen_optimized_replacement.sv.
   Do not compile two modules both named conv_addr_gen.
3) Add tb_conv_addr_gen_optimized.sv to Simulation Sources, set it as Simulation Top, run all.
4) Re-run tb_fpga_system_top simulation; require original convolution output PASS.
5) Reset/Re-run Synthesis (Design Top: fpga_system_top, keep 10ns clock constraint).
6) Compare LUT/FF/DSP/BRAM and report_timing_summary WNS.
7) If setup timing still fails, the next engineering step is to pipeline the address
   computation / change the prefetch schedule; changing XDC to mask the failure
   would not fix the hardware.

This version is combinational and does not change latency. Generic PADDING>0
uses a slower coordinate-check branch. Address width must fit all relevant
M/K indices and H*W*C activation addresses; default ADDR_WIDTH=16 does not
support every full ResNet18 feature-map size.

CAVEAT: No SystemVerilog simulator is available in the authoring runtime.
Check syntactic/elaboration compatibility and PASS in Vivado before adoption.
