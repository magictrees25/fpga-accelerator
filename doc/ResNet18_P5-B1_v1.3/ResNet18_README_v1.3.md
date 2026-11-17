# ResNet-18 FPGA 卷积加速器（RTL）

> 课题：《面向图像分类的卷积神经网络加速器设计与实现》  
> 当前分支：`p5-b1-weight-bram-sharing` · 项目文档 **v1.3 / 2026-11-15**  
> 当前状态：**P5-B1 Weight BRAM 共享优化已完成：六次连续多模式仿真 PASS，BRAM Tile 13→11，Post-Route PhysOpt 后 100 MHz 已约束内部 Setup/Hold 检查通过。**  
> 注意：尚未完成 ResNet-18 整网自动调度、FPGA 板级推理和功耗测量。

本项目以 ResNet-18 图像分类网络为目标，使用 SystemVerilog 设计 **8×8 INT8 PE 阵列**、双路 K-pair 数据供给、M/N/K 分块控制器、同步 BRAM 和数据复用 Line Buffer，并通过 Vivado/XSim 验证功能、资源和内部时序。研究重点是 RTL/FPGA 卷积加速架构与片上访存优化。

## 文档入口

- [项目说明与开发规划 v1.3](ResNet18_卷积加速器_项目说明与开发规划_v1.3.md)：网络结构、硬件架构、P1～P5 阶段结果、资源/时序对比和后续路线。
- [P5-B1 阶段验收记录](P5_B1_2026-11-15_PASS.md)：Shared Weight Reader 设计、六次连续仿真、BRAM 和 PhysOpt 结果。
- [版本更新日志 v1.3](ResNet18_CHANGELOG_v1.3.md)：按阶段记录功能变更、验证状态、资源/性能、证据与限制。
- [上一版项目说明 v1.2](ResNet18_卷积加速器_项目说明与开发规划_v1.2.md)：保留为历史版本。

## 里程碑

| 阶段 | 代表配置 | 验收状态 |
|---|---|---|
| P1/P2/P2.1 | 初始阵列、双 K 数据供给、读地址/权重 Reader 优化 | 历史基线完成 |
| P3 | `layer1.0.conv1`，3×3/S1/P1 | **392 Tiles；200,704 输出 PASS** |
| P4-C | `layer2.0.downsample.0`，1×1/S2/P0 | **784 Tiles；100,352 输出 PASS** |
| P4-D3 | `layer2.0.conv1`，3×3/S2/P1 | **784 Tiles；100,352 输出 PASS** |
| P5-A | 3×3/S1、1×1/S2、3×3/S2 共用 PE/Controller | **六次连续模式回归 PASS** |
| P5-B1 | 两套 Weight Reader 合并为 Shared Weight Reader | **1,536/1,536 INT32 PASS；BRAM 13→11；PhysOpt 内部时序 PASS** |
| P5-B2 | Activation Reader/BRAM 共享评估 | 下一阶段 |

### P5-B1 核心结果

- Shared Weight Reader：两个 64-bit Bank，每 Bank 深度 288；偶数 K/奇数 K 分 Bank；支持 K=64/576。
- 功能回归：6 次连续运行，1,536/1,536 INT32 Golden 输出 PASS，47,104 K-pair，50,368 compute cycles。
- Post-Route PhysOpt：BRAM Tile 11、DSP 64、WNS +0.038 ns、TNS 0、WHS +0.014 ns、THS 0。
- 约束边界：120 个输入端口未设置 Input Delay，37 个输出端口未设置 Output Delay；目前仅声明已约束内部时序通过。
- 功耗分析暂缓；没有真实 FPGA 板级延迟/吞吐或能耗测量。

## 版本维护规则

每完成一个阶段，保存对应 RTL/TB、PASS Transcript/截图、必要的 CSV/RPT，并在 CHANGELOG 追加记录。**未仿真的设计只能标“待验证”；单 Tile/多模式 TB PASS 不等于整网 PASS。**

文档版本号是研发里程碑编号，不意味着已经创建 Git Tag。当前约定：阶段代码和报告可推送 GitHub；项目说明 Markdown 先在本地维护，确认后再决定是否提交。

> 原工程参考 [taitashaw/fpga-ai-accelerator](https://github.com/taitashaw/fpga-ai-accelerator)，原有版权与 MIT LICENSE 等信息应保留。
