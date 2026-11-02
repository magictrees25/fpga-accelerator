卷积加速器 Baseline —— 输出 BRAM 集成（增量包）

本包是对已经在 Vivado 仿真 PASS 的 conv_accelerator_top.sv 的非破坏性扩展，
不覆盖它，也不修改原来的 M/N/K Controller 和双路输入 BRAM。

本包 4 个 SystemVerilog 文件：
1. output_bram.sv
   - 简单双端口同步 RAM：每个字为 8 x 32 = 256 bit。
   - wr_en/wr_addr/wr_data/wr_mask 接收计算结果。
   - 无效 lane 写入 0，并将 wr_mask 另存作为输出有效通道标记。
   - rd_en/rd_addr 同步读回；rd_data/rd_mask/rd_valid 在有效时钟沿后更新。
   - RAM 内容不做复位，禁止读未写地址；同地址读写冲突行为不规定。
2. conv_accelerator_top_outmem.sv
   - 例化原来的 conv_accelerator_top + 新的 output_bram。
   - 通过正式端口写出结果、读回结果，不使用 Testbench 的层级引用做数据连接。
   - Design Top 设为 conv_accelerator_top_outmem。
3. tb_conv_accelerator_top_outmem.sv
   - 由已通过的 tb_conv_accelerator_top.sv 扩展，保留完整输入和权重校验。
   - 完成 M=9,N=4,K=27 单层卷积后，从 output_bram 读回全部 9 个 256-bit 字，
     检查前 4 个有符号 INT32 结果和后 4 个 0 补位、通道掩码 00001111。
   - Simulation Top 设为 tb_conv_accelerator_top_outmem。
4. tb_output_bram.sv
   - 可选：单独测试 RAM 写入、mask、同步读回的最小 Testbench。

运行步骤：
A. 保留原工程之前已通过仿真的 Design Sources：
    pkg_accelerator.sv; mac_unit.sv; systolic_array.sv;
    k_tile_controller.sv; gemm_mn_controller_mem.sv;
    conv_accelerator_top.sv; conv_addr_gen.sv;
    conv_bram_reader.sv; sync_fmap_bram.sv;
    conv_weight_bram_reader.sv; sync_weight_bram.sv。
B. 新增 output_bram.sv 与 conv_accelerator_top_outmem.sv 到 Design Sources。
C. 新增 tb_conv_accelerator_top_outmem.sv 到 Simulation Sources，Set as Top。
D. Run Behavioral Simulation -> Tcl Console: run all。
E. 预期：
    PASS: output BRAM integration; M=9 N=4 K=27; 243/243 aligned A/B beats;
    9 writes; 9 readbacks; 36/36 outputs correct; padded lanes zero

关键波形：write_valid, write_addr, write_mask, write_data, out_rd_en,
          out_rd_addr, out_rd_valid, out_rd_data, out_rd_mask。
写地址 0..8 对应输出空间位置 M0..M8；每个地址有 8 个 INT32 lane。

注意：
- 本包未在作者环境运行 Vivado/其他 SystemVerilog 仿真器；以上 PASS 是预期输出，
  需要使用你本机 Vivado 确认。
- 本阶段存储仍是 INT32 部分和/结果，不是后续卷积层需要的 INT8 输入，
  尚未实现 bias、BN 融合、量化、残差、激活等操作。
- 对于 N 不能被 8 整除的情况，本模块按照 write_mask 将无效 lane 写 0。
- 输出 RAM 的 ram_style = block 属性请求 Vivado 推断块 RAM，但实际 RAMB
  使用量、综合警告和时序必须在目标 FPGA 综合后确认。
- 输入 BRAM 仍在运行前一次性预加载；尚无 AXI、DDR、DMA、双缓冲或背压。
