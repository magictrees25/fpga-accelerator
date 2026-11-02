# ResNet18 FPGA accelerator development snapshot

This repository preserves an existing SystemVerilog project and its extensions for a ResNet18 convolution accelerator. The original project is based on [taitashaw/fpga-ai-accelerator](https://github.com/taitashaw/fpga-ai-accelerator). The original README and MIT LICENSE, including the upstream copyright notice, are retained.

The repository was initialized from the local working project on 2026-11-02. It is a new snapshot repository; it does not contain the upstream Git commit history.

## Current development status

- P=1 representative convolution regressions have user-reported Vivado simulation PASS results: layer1.0.conv1 (3x3), layer2.0.downsample.0 (1x1, stride 2), and conv1 (7x7, stride 2, padding 3).
- The conv1 full-layer test checks 1568 logical Tiles, 100352 packed result writes, 14751744 K FEED cycles and 802816 INT32 output values.
- P=2 and its address-optimized layer1.0.conv1 configuration are retained alongside P=1. The address-optimized version has a previously inspected 100 MHz constrained internal timing result on XC7A100T, WNS +0.516 ns. This does not establish timing for every configuration or any board-level deployment.
- The regressions use deterministic signed INT8 stimuli and independent convolution reference calculations. Full ResNet18 inference and Zynq board deployment are not claimed.

## Source layout

- `rtl/`: computational RTL and fixed configuration wrappers.
- `tb/`: independent testbenches and common verification modules.
- `verification/`: software reference and stimulus scripts.
- `doc/`: development documents.
- `fpga_ai_accelerator/*.xpr`: saved Vivado project.
- `fpga_ai_accelerator/*.srcs/`: source/constraint files belonging to the saved project.
- Earlier integration directories are retained for history; they may define modules also present in `rtl/`. Do not recursively compile every SV file into one design.

Vivado build/run directories, simulation databases, waveforms and automatic checkpoints are excluded by `.gitignore`. They remain on the original local computer and are not deleted. The saved XPR may contain generated checkpoint references; regenerate generated artifacts locally as needed. Keep package compilation order and choose the appropriate Simulation Top instead of the RTL design top when running a testbench.

Latest conv1 Simulation Tops:

1. `tb_fpga_system_top_conv1_7x7_tile_p1`
2. `tb_fpga_system_top_conv1_7x7_1568tiles_p1`

Both require `tb_conv1_7x7_p1_common.sv` in Simulation Sources. Their RTL wrapper is `fpga_system_top_conv1_7x7_tile_p1`. This Git publication does not run or modify Vivado.
