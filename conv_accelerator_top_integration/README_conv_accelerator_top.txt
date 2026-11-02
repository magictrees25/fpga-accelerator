可综合卷积计算/数据供给顶层 — 第一步集成（固定参数版本）

新增文件：
1. gemm_mn_controller_mem.sv
   - 复制之前已经验证过的调度结构，另起模块名。
   - 将 mem_clear_acc/mem_feed_valid 作为显式模块输出供 RAM 预取使用。
   - 保留原有 gemm_mn_controller.sv，不要删除或替换。
2. conv_accelerator_top.sv
   - 正式连接控制器、激活 BRAM 读取器、权重 BRAM 读取器。
   - 可通过输入写端口加载激活和权重。
   - 返回已有写接口 write_valid/write_addr/write_mask/write_data；外部输出RAM稍后再接。
3. tb_conv_accelerator_top.sv
   - 从之前双 BRAM PASS 测试变更例化，保留原有参考计算和逐周期检查。
   - 检查 243 次有效 K 周期的激活和权重，以及 9 次写入、36 个结果。

需要你原工程已存在的依赖：
- pkg_accelerator.sv
- mac_unit.sv
- systolic_array.sv
- k_tile_controller.sv
- conv_addr_gen.sv
- sync_fmap_bram.sv
- sync_weight_bram.sv
- conv_bram_reader.sv
- conv_weight_bram_reader.sv

在 Vivado：
1. Design Sources 添加 gemm_mn_controller_mem.sv 和 conv_accelerator_top.sv。
2. Simulation Sources 添加 tb_conv_accelerator_top.sv。
3. 右键 tb_conv_accelerator_top -> Set as Top。
4. Run Behavioral Simulation, 在 Tcl Console 中 run all。
5. 若 PASS，查看 dut.u_ctrl.state, dut.k_addr, dut.fetch_k_addr,
   dut.act_data, dut.wgt_row_packed, write_valid, write_addr。

重要限制：
- 当前是一层卷积的固定参数配置，默认 5x5x3 输入、3x3 核、4 输出通道、S=1/P=0。
- 两个 BRAM 使用 1 时钟同步读取模型；读写冲突、具体 FPGA BRAM 映射、时序尚需综合验证。
- 激活和权重先预加载；未实现 AXI/DMA，预加载/计算阶段不能混合修改数据。
- 默认不支持 FEED 中断/backpressure；更复杂系统需要增加握手、数据有效等控制。
- 输出只提供写接口，还没有实例化输出 RAM。
- 没有用该环境直接运行 Vivado，运行结果以你的 Tcl Console 为准。
