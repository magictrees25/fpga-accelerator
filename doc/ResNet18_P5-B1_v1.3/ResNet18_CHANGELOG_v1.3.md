# CHANGELOG — ResNet-18 RTL 卷积加速器

## [v1.3] - 2026-11-15 — P5-B1 Weight BRAM 共享优化 PASS

**设计修改**
- P5-A 已将 3×3/S1、1×1/S2、3×3/S2 三种模式接入一套 8×8 PE 和 K/M/N Controller；P5-B1 在此基础上合并两套 Weight Reader。
- 新增 `rtl/conv_weight_bram_reader_p5_shared.sv`：两个 64-bit Bank、每 Bank 深度 288；Bank 0/1 分别保存偶数/奇数 K；支持 K=64 和 K=576，输出接口保持 128-bit K-pair。
- 修改 `rtl/conv_accelerator_top_p5_multimode.sv` 接入单一 `u_weight_shared`；修改 `tb/tb_p5_multimode_6runs.sv` 检查统一 Reader。Activation Reader、Controller 和 PE 阵列未改。

**Vivado XSim 验证**
- 仿真顶层：`tb_p5_multimode_6runs`；3×3/S1 → 1×1/S2 → 3×3/S2，连续两轮，共 6 次运行，不在模式切换时复位。
- **1,536/1,536 INT32 Golden 输出 PASS**；**47,104/47,104 K-pair**；累计 **50,368 compute cycles**。
- 三种模式分别对应 K=576、K=64、K=576。该测试是多模式单 Tile 集成回归，不代表 P5 顶层已完成三种模式的自动全层调度。

**资源和时序**
- 同一器件/时钟目标下，Post-Route PhysOpt：BRAM Tile **13→11**（减少 2，约 15.38%）；RAMB36 12→10、RAMB18 保持 2；DSP 保持 64；LUT 7,586→7,581；FF 4,612→4,611。
- P5-B1 PhysOpt 后：WNS **+0.038 ns**、TNS 0、WHS **+0.014 ns**、THS 0；Setup/Hold failing endpoints 均为 0；Routing Errors 0。
- PhysOpt 前 WNS −0.654 ns、TNS −6.693 ns、16 个 Setup failing endpoints；执行 `phys_opt_design -directive AggressiveExplore` 后收敛。
- 最差路径仍为 K Controller 的 `k_idx_reg` 到 P3 Line Buffer 的 `odd_q_reg`；Data Path Delay 10.494 ns（Logic 4.328 ns，Route 6.166 ns，15 级逻辑）。
- 报告仍提示 120 个输入端口无 Input Delay、37 个输出端口无 Output Delay；当前只声明已约束内部时序通过。WHS 仅 +14 ps，裕量偏小。

**报告与版本证据**
- `reports/P5_B1/`、`reports/P5_B1/impl/`、`reports/P5_B1/PhysOpt/`。
- Git 分支：`p5-b1-weight-bram-sharing`；HEAD：`d39293e9ad38805b30930860486a591efae51168`。
- 独立阶段记录：`P5_B1_2026-11-15_PASS.md`。
- 文档 v1.3 仅本地保存，未推送 GitHub。

**已知限制与下一步**
- 无真实 FPGA 板级延迟、DDR/AXI 传输或功耗数据；功耗分析仍暂缓。
- P5 多模式外层 M/N Tile 调度仍由 Host/Testbench 驱动；ResNet-18 整网尚未运行。
- 下一阶段：P5-B2 Activation Reader/BRAM 共享评估，先做架构与时序风险分析，再决定是否实现。


> 维护方法：**按时间倒序**追加版本记录，每版说明修改、验证、资源/性能、证据与限制。  
> 本文使用研发版本编号，不代表仓库已创建对应 Git Tag。原始代码和历史记录不应被后续总结直接覆盖。

## [v1.2] - 2026-11-08 — P4-D：3×3/S2/P1 完整层仿真 PASS

**修改与架构**
- 目标：ResNet-18 `layer2.0.conv1`，输入 56×56×64，3×3/S2/P1，输出 28×28×128。
- 继续复用 `gemm_mn_controller_mem_p2`、`k_tile_controller_p2`、`systolic_array_p2`、64 个 PE 和 K576 `conv_weight_bram_reader_p2_opt`。
- 新建 3×3/S2 直接双 Bank Activation Reader、P4-D 计算顶层/Output BRAM 顶层；单 Tile 9×9×64 Patch → 4×4×8 输出，M/N/K=16/8/576。
- 处理外层 Tile 的 global halo 与越界零填充，尚未引入新的 Line Buffer。

**P4-D2：五组单 Tile**
- 2026-11-08 用户 Vivado XSim Transcript：5/5 epoch，**640/640 INT32 Golden PASS**；每 epoch **4608/4608 K-pair**，**4880 compute cycles**。
- Tile 原点：(0,0)、(0,24)、(24,0)、(24,24)、(12,16)；对应 Padding 1088、576、576、0、0 Bytes/Tile。
- 首次 K=64 Weight mismatch 的根因是**早期 TB 仅加载 64 个权重字**；修复为加载 K=576 全部权重字后 PASS。未因此修改 PE/Weight Reader RTL。

**P4-D3：完整层**
- 2026-11-08 用户 Vivado XSim Transcript：**784/784 Tiles，100352/100352 INT32 输出，3612672/3612672 K-pair PASS**。
- **累计计算 3825920 cycles**，10 ns TB clock → **38.2592 ms 累计计算窗口**，**不是板级或完整 E2E**。
- **padding=128000 bytes**（784 个 Tile 预加载中的补零字节总和，包含通道组重复读取），全局 Golden checksum **-28679**。
- 测试模块：`tb_layer2_conv1_784tiles_p4`；硬件设计顶层：`fpga_system_top_layer2_conv1_tile_p4`。
- 对应 Vivado Transcript 截图于 2026-11-08 提供；源码/截图应在本地共同保存，Git 提交 SHA 尚未核对。

**验收限制与后续计划**
- 全层 Tile 遍历与输出拼接由 Testbench/Host 执行；并不说明 RTL 中已有自动完整层调度，更不等于 ResNet-18 整网已运行。
- 暂无 P4-D 的 Synthesis/Implementation 性能、时序或板卡数据；未收到 P4-D1 独立 Reader TB 的单独 PASS 截图（D2/D3 集成路径已 PASS）。
- 下一阶段建议 P5：统一 3×3/S1、1×1/S2、3×3/S2 三种卷积配置的数据供给/调度接口；保持 P3/P4 已有回归。

## [v1.1] - 2026-11-08 — P4-C 完整 1×1/S2 Shortcut PASS

**本版变化**
- 将此前主说明文档 v1.0 更新为 v1.1，新增本 `CHANGELOG.md`、项目入口 `README.md`。
- 增加 `reports/P4/P4_C_2026-11-08_PASS.md` 独立验收记录与图片证据。
- 在交付包附上 `tb/tb_layer2_downsample_784tiles_p4.sv`，便于归档全层测试源码；最终需确认与用户实际仿真源一致。

**Vivado XSim 验证**
- 目标层：ResNet-18 `layer2.0.downsample.0`，1×1、stride=2、padding=0，56×56×64 → 28×28×128。
- `M=16, N=8, K=64`/Tile；49 个空间 Tile × 16 个输出通道 Tile。
- **784/784 Tiles PASS**；**100352/100352 INT32 Golden PASS**；**401408/401408 K-pairs PASS**。
- 累计 **614656 compute cycles**；全局输出 **checksum=-5799936**。
- 10 ns TB 时钟换算为 **6.14656 ms 的累计计算时间**，不含 Host preload/readback，不是板卡实测或 ResNet-18 整网延迟。
- 证据：用户提供的 2026-11-08 Vivado Transcript 截图；本包 `reports/P4/p4_c_pass_2026-11-08.png`。

**已知限制与下一步**
- Git 当前 `p4-conv-support` 分支截至 `c601fbe0a` 尚未包含 P4-C 全层 TB 和完整仿真日志；需提交归档。
- 开发 **P4-D：3×3/S2/P1**，优先独立 Reader → 单 Tile Golden → 784-Tile 全层 Golden。
- P4 尚未完成独立 FPGA Implementation 和真实板级验证。

## [v1.0] - 2026-11-08 — 第一版项目总说明

- 整理课题定位、ResNet-18 层级结构、GEMM M/N/K、8×8 PE 架构和 P1～P4 计划。
- 建立 P2.1/P3 功能、时序、资源、Activation BRAM 访问及 Host 加载对比表。
- 当时 P4-C 的完整层测试处于待运行状态；本版 v1.1 根据随后实际 XSim PASS 更新。

## [P4-B] - 2026-11-07 — 1×1/S2 完整单 Tile 验证

- 新增 K64 Weight Reader、P4 计算顶层、Output BRAM 顶层与单 Tile Testbench。
- 两轮无复位验证：每轮 128/128 INT32 输出、512/512 K-pair，784 compute cycles；两轮共 256/256 输出 PASS。
- 代码提交参考：`c601fbe0a`（`p4 B`）。

## [P4-A] - 2026-11-07 — 1×1/S2 激活 Reader 验证

- 新增 `conv_bram_reader_p4_1x1.sv`，面向 8×8×64 输入、4×4 输出的双 Bank 同步读取。
- 地址 `pair_addr = 512*oy + 64*ox + k/2`，k 为 0～62 的偶数。
- 两组数据，1024/1024 K-pair（2048 INT8）PASS。

## [P3] - 2026-11-07 — 3×3/S1 片上复用与报告

- 引入 4-Row Line Buffer，完成 392 Tiles、200704 个 INT32 输出的全层验证。
- P3 Activation BRAM pair reads：752640；Line Buffer pair reads：7225344。
- 资源：LUT7450、FF4603、LUTRAM492、BRAM36=8、DSP=64。
- PhysOpt 后 Setup WNS +0.007 ns，Hold WHS +0.063 ns，10 ns 已约束内部时序通过。
- 数据 `reports/P3/layer1_p3_summary.csv`、`layer1_p3_tiles.csv`、`physopt/timing_summary_after.rpt`。
- 重要限制：缺失部分外部 I/O 延迟约束，无法据此声称整板时序已闭合。

## [P2.1] - 2026-11 — 双 K 读地址/Weight Reader 优化基线

- `reports/P2_1_WGT_OPT/layer1_p2_opt_summary.csv`：392 Tiles、7,651,840 compute cycles、10,590,267 E2E cycles。
- 资源：LUT6768、FF4559、LUTRAM12、BRAM36=8、DSP=64；100 MHz Setup WNS +0.962 ns。
- 与 P3 对比时，明确 E2E 的 TB 加载条件并不同，禁止直接视为板级加速比。

## [P1 / Baseline] - 2026-10 — 8×8 INT8 卷积核心原型

- 8×8 PE、K/M/N 调度、同步 BRAM、卷积地址映射、输出 RAM、初步综合实现。
- 历史文档：`doc/ResNet18_卷积加速器_Baseline_进度记录_2026-10-28.md`。

---

## 今后每次更新的模板

```markdown
## [vX.Y] - YYYY-MM-DD — 阶段名称

**修改内容**：新建/变更的 RTL、TB、脚本；为什么修改。

**验证证据**：Simulation Top、网络层/尺寸、Golden 输出、K-pairs、cycles、日志/截图路径；PASS 或待验证。

**资源/时序/性能**：FPGA 型号、Vivado 版本、约束、LUT/FF/LUTRAM/BRAM/DSP、WNS、周期；口径是仿真还是硬件。

**已知限制**：未支持的参数、未覆盖的边界、未关闭的时序和接口问题。

**后续任务**：下一可验收里程碑与标准。

**Git SHA**：提交成功后追加短 SHA 和相应文档/报告链接。
```

**维护原则**：有实际 PASS 截图/日志才写 PASS；避免混用 MAC 与 FLOP、仿真时间与板级时间、片上 BRAM 与 DDR 访问量。
