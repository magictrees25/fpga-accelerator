# ResNet-18 卷积加速器 RTL Baseline 阶段进度记录

> 记录日期：2026 年 10 月 28 日  
> 课题名称：**面向图像分类的卷积神经网络加速器设计与实现**  
> 当前阶段：**小规模卷积 RTL Baseline 已完成联合功能仿真、综合和初步 FPGA 布局布线时序验证；下一阶段为 ResNet-18 典型卷积层适配。**

## 1. 课题目标及当前工作定位

以 ResNet-18 图像分类网络为目标，使用 SystemVerilog 自行构建、验证并逐步优化 INT8 卷积计算加速核心。重点放在 RTL 数据通路、PE 阵列、分块调度、片上存储与访存时序，而不是单纯使用现成 DPU 完成网络部署。使用 Vivado 在 FPGA 上验证功能、综合资源和时序，后续通过基线与优化后架构的对比评价设计效果。

**当前已经建立的是“小规模单层卷积、带同步片上存储模型的 RTL 硬件 Baseline 原型”**，不是已完成 ResNet-18 整网推理的最终系统。此阶段以“功能正确、可综合、内部同步路径满足设定时钟”为核心验收目标。

## 2. 已学习并实现的核心模块

| 层次 | 文件/模块 | 已完成内容 |
| --- | --- | --- |
| 计算单元 | `mac_unit.sv` | 学习并验证 INT8 × INT8 乘法、INT32 累加，三级流水线，清零与使能控制。 |
| 并行计算阵列 | `systolic_array.sv` | 8×8、64 个 PE；激活值沿行传递，权重按列广播/错拍；每列累加八行 PE 的结果。 |
| K 维控制 | `k_tile_controller.sv` | K 索引遍历、`S_CLEAR`/`S_FEED`/`S_DRAIN`、跨 K 分块累加、结果有效与完成控制。 |
| M/N 维控制 | `gemm_m_controller.sv`、`gemm_mn_controller.sv` | M 空间位置切换、N 输出通道 Tile 切换、结果写入地址生成。 |
| 存储控制端口 | `gemm_mn_controller_mem.sv` | 将内部需要的存储控制信号显式引出，避免正式硬件顶层依赖 Testbench 层级引用。 |
| 卷积地址生成 | `conv_addr_gen.sv` | 将 GEMM 的 `(m_addr, k_addr)` 映射到 HWC 格式输入特征图线性地址；考虑 stride/padding 和有效性检查。 |
| 激活预取 | `conv_bram_reader.sv` | 同步 BRAM 的 K 维提前预取：`S_CLEAR` 读 K0，`S_FEED` 消费当前 K 同时请求 K+1。 |
| 激活存储 | `sync_fmap_bram.sv` | 采用同步读取的 INT8 特征图存储模型。 |
| 权重预取 | `conv_weight_bram_reader.sv` | 依据 N-tile 与 `fetch_k_addr` 读取每 K 对应的 8 路 INT8 权重，配合激活预取对齐。 |
| 权重存储 | `sync_weight_bram.sv` | 64-bit/字（8×INT8）的同步权重存储。 |
| 输出存储 | `output_bram.sv` | 写入每组 8×INT32（256-bit）卷积结果，保存有效通道掩码，并支持同步读回。 |
| 计算顶层 | `conv_accelerator_top.sv` | 集成 M/N/K Controller、阵列及激活/权重数据供给。 |
| 输出存储顶层 | `conv_accelerator_top_outmem.sv` | 在计算顶层外封装输出 BRAM。 |
| FPGA 窄接口顶层 | `fpga_system_top.sv` | 将 256-bit 内部结果总线留在芯片内部，外部通过 32-bit 通道选择读出，降低物理 IO 占用。 |

**实现细节：** 在当前 K Controller 调度方式下，每个有效 FEED 周期选定一个 K 元素送入对应 PE 行入口；虽然阵列物理上有 8 路激活输入，但不能把 8×8 PE 全部视为每时钟均在执行有效乘法。后续需通过计数器/波形测量真实 PE 利用率。

## 3. GEMM 与卷积映射学习成果

采用 im2col/GEMM 数学映射：

\[
A_{M\times K}\times B_{K\times N}=C_{M\times N}
\]

其中：

- `M = H_out × W_out`：输出特征图的空间位置数，等于滑动窗口数量；
- `N = C_out`：输出通道数；
- `K = KH × KW × C_in`：每个输出元素对应的乘加项数。

在 HWC 数据排列下，使用 M 定位输出窗口、K 定位窗口内行/列/输入通道，再将三维输入坐标转换为线性 BRAM 地址。例如针对 5×5×3 输入、3×3、stride=1、padding=0：

- 输出为 3×3×4，故 `M=9, N=4, K=27`；
- `m_addr=4, k_addr=14` 时，读取 `fmap[2][2][2]`；
- HWC 线性地址为 `(2×5+2)×3+2=38`。

已将最初 Testbench 内的组合式 im2col 数据选择逐步替换为**可综合地址生成 RTL + 同步 BRAM + 预取控制**。

## 4. 已完成的功能仿真与结果

### 4.1 基础 GEMM 测试

- 验证 K=9、K=27 的乘加与跨 K 分块累加。
- 验证 M/N 控制器自动切换和结果写入。
- 完成 `M=16, N=16, K=27` 的 GEMM 功能仿真，并观察 `k_start`、`k_done`、`write_valid`、`write_addr`、`m_idx`、`n_tile_idx` 等关键波形。

### 4.2 首次真实卷积功能测试

测试参数：输入 `5×5×3`；卷积 `3×3`；输出通道 `4`；stride `1`；padding `0`；输出 `3×3×4`。

- `243/243` 个 M/K 地址检查通过；
- `9` 组输出写入；
- `36/36` 个有效输出元素正确；
- 观察并确认 `m_addr` 对应窗口移动、`k_addr` 对应窗口内部元素遍历。

### 4.3 同步 BRAM 与预取联合验证

- 激活 BRAM：`S_CLEAR` 预读 K0；`S_FEED K0` 使用 K0 数据，同时请求 K1；
- 权重 BRAM：与激活 BRAM 采用相同的预取 K 索引；
- 通过波形核对 `k_addr`、`fetch_k_addr`、`fmap_addr`、`act_data`、权重数据和 data_valid 的关系；
- 双路同步 BRAM 联合卷积仿真 PASS。

### 4.4 输出 RAM 与系统顶层回归

完整顶层 `tb_fpga_system_top.sv` 的 Tcl 结果：

```text
PASS: fpga_system_top;
243/243 aligned A/B beats;
9 writes;
72 lane readbacks;
36/36 valid outputs correct;
padded lanes zero
```

含义：9 个输出地址 × 每地址 8 个通道 = 72 次通道读回，其中 36 个是有效卷积输出，其余补零通道正确。

### 4.5 地址生成优化后的回归

优化版 `tb_conv_addr_gen_optimized.sv` 通过 **4 组参数配置**测试（包括不同 H/W/C、卷积核、步长、padding 的组合）；修改地址生成 RTL 后，完整 `tb_fpga_system_top.sv` 亦重新 PASS。

> 以上为当前测试规模和测试用例的功能验证。尚不能据此宣称所有 ResNet-18 层配置都已经正确。

## 5. Vivado FPGA 综合与实现结果

### 5.1 工程与约束

- Vivado：2023.2（来自时序报告）；
- FPGA 器件：`xc7a100tcsg324-1`；
- Design Top：`fpga_system_top`；
- 常用 Simulation Top：`tb_fpga_system_top`；
- 初步时钟约束：`create_clock -name sys_clk -period 10.000 [get_ports clk]`；
- 目标时钟：**100 MHz**。

### 5.2 顶层 IO 问题与解决

初期以 `conv_accelerator_top_outmem` 直接作顶层，暴露两路 256-bit 结果总线等大量调试信号，综合报告显示 **675/210 个 IO**，明显超限。

后加入 `fpga_system_top.sv`，将计算/存储内部宽总线保持在 FPGA 内部，通过较窄的 32-bit 通道读回接口提供输出，IO 降为 **134/210**。该修改改善了接口映射条件，但仍不是最终的板级 AXI/DDR 系统接口。

### 5.3 综合和实现资源记录

已观测到以下不同阶段的资源数据；**不能把不同阶段的 LUT 数量视为完全可比的最终资源数据**。

| 阶段/版本 | LUT | FF | DSP | BRAM Tile | IO | 备注 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 初始宽 IO 顶层，综合后 | 7408 | 4341 | 64 | 4.5 | 675 | IO 超限。 |
| 窄接口顶层，综合后（先前记录） | 6997 | 4236 | 64 | 4.5 | 134 | 解决 IO 超限。 |
| 先前 Placed 报告 | 6206 | 4236 | 65 | 4.5 | 134 | 权重读取模块额外用了 1 个 DSP。 |
| N_TILES=1 专门化修改后的层级资源截图 | 6433 | 4236 | **64** | 4.5 | 134 | DSP 降到 64；此处 LUT 数值应以该截图对应设计阶段为准。 |

分析 Hierarchy 发现，使用 65 DSP 的版本里：

- `u_ctrl`（包含 8×8 阵列）使用 64 DSP；
- `u_weight_reader` 使用额外 1 DSP。

排查 `conv_weight_bram_reader.sv` 后，怀疑额外 DSP 对应权重地址运算：

```systemverilog
weight_word_addr = n_tile_idx * K_LEN + fetch_k_addr;
```

对当前 `N_TILES=1` 的配置，采用编译期分支直接使用 `fetch_k_addr`；重新综合后 DSP 总数降为 **64**。

**限制：** 多 N-tile（`N_TILES>1`）分支仍需要单独验证，其地址乘法的实现成本不能由单 Tile 的资源结果推断。

### 5.4 时序定位与改进

原始 `conv_addr_gen.sv` 使用多个 `int` 中间变量及除法、取模、地址乘加。Vivado 最差时序路径从 K Controller 的 `k_idx` 寄存器出发，经预取和卷积地址计算，到激活 BRAM 地址端口。原始综合报告显示 44 级组合逻辑、24 个 CARRY4，导致 100 MHz 下严重违例。

优化思路：依据 HWC K 展开关系简化线性地址生成，减少部分中间坐标展开；保持外部接口和读延迟不变，并完成地址生成器与整机功能回归。

| 时序阶段 | WNS | TNS | 备注 |
| --- | ---: | ---: | --- |
| 原始地址生成 RTL，Post-Synthesis | -20.701 ns | -146.774 ns | 时序不满足。 |
| 地址生成简化后，Post-Synthesis | -0.592 ns | -0.733 ns | 明显改善，仍有 2 个违例端点。 |
| 优化后先前 Routed 实现 | +0.281 ns | 0.000 ns | 已满足 100 MHz 的已约束时序。 |
| **当前 64 DSP 版本 Implementation** | **+0.033 ns** | **0.000 ns** | **无建立时间违例端点。** |

当前最新 Implementation 的其他数值：

- **WHS = +0.048 ns**；
- **THS = 0.000 ns**；
- 时钟周期 **10.000 ns（100 MHz）**；
- Timing Summary 显示 *All user specified timing constraints are met*。

**重要限定：** 当前建立/保持时间裕量比较小（WNS 仅 +0.033 ns）。外部输入/输出的时序约束及板级引脚配置尚不完整，当前结果主要证明**已约束的内部同步路径通过**；不能等同于完成上板或整网 100 MHz 验证。

### 5.5 已保存的报告

已通过 Vivado Tcl 的 `report_timing_summary` 和 `report_utilization` 导出 Implementation 报告，并保存当前阶段的 Baseline 数据。建议将当前 RTL、XDC、Testbench、报告和 Vivado 工程版本一并归档；下阶段另建实验目录，不覆盖本版。

## 6. 当前 Baseline 的完成程度

### 已能确认

1. 完成以 8×8 INT8 PE 阵列为核心的 RTL 卷积计算数据通路。
2. 完成 K/M/N 控制、卷积地址生成、激活/权重同步 BRAM 预取、输出存储与窄接口顶层集成。
3. 在 `M=9, N=4, K=27` 的完整卷积测试中，地址、输入拍、存储读写及结果校验 PASS。
4. 优化后的地址生成器通过 4 组参数测试。
5. Vivado 综合成功；当前 64 DSP 版本已完成 Implementation，所设 100 MHz 内部时钟约束检查通过。

### 尚未完成，不能宣称完成

1. ResNet-18 实际大尺寸卷积层的完整映射与仿真；
2. 多 N-tile 情况下的**完整卷积、双路 BRAM 与输出存储联合验证**（之前虽然验证过 GEMM 的多 N-tile，但不是这套全部数据通路）；
3. ResNet-18 所需的 1×1、7×7、stride=2、padding 等各种层配置的系统级回归；
4. 大尺寸特征图/权重的分块加载、缓存容量、地址位宽、DDR/AXI 或主机数据供给设计；
5. BN、ReLU、池化、残差加法、全连接及整网推理集成；
6. 真实 FPGA 上板、外部 IO/板级时序约束完整闭合；
7. 真正的计算周期、吞吐、端到端推理延迟、片外数据量与功耗测量。

**综上：小规模单层卷积 RTL Baseline 原型基本完成；ResNet-18 可运行的完整 Baseline 尚未完成。**

## 7. 下一阶段具体计划

### 下一步（推荐优先级最高）：测试更大的 M/N/K 和多 N-tile

先选择容易验证、但能覆盖多 Tile 的 3×3 卷积：

| 参数 | 本阶段已 PASS | 下一轮建议 |
| --- | ---: | ---: |
| 输入特征图 | 5×5×3 | **5×5×8** |
| 卷积核 | 3×3 | **3×3** |
| 输出通道 | 4 | **16** |
| Stride / Padding | 1 / 0 | **1 / 0** |
| 输出特征图 | 3×3×4 | **3×3×16** |
| GEMM (M,N,K) | (9,4,27) | **(9,16,72)** |
| N-tile 数 | 1 | **2** |

依次完成：

1. 检查 `conv_accelerator_top.sv`、`conv_accelerator_top_outmem.sv`、`fpga_system_top.sv` 的参数传递和任何固定常数；
2. 扩展存储深度：激活数据 5×5×8=200 个 INT8，权重 72×2=144 个 64-bit 字，输出至少 9×2=18 个 256-bit 字；
3. 检查权重读取模块 `N_TILES>1` 分支，以及 M/N 切换时的预取、写入地址；
4. 建立软件卷积参考模型，对每个输出元素进行自动检查；
5. 功能 PASS 后再重新综合与实现，记录真实资源、时序、计算周期。

### 随后：ResNet-18 典型层适配

- **3×3 卷积代表层**：例如 Cin=64、Cout=64、输出空间 56×56，对应 `M=3136, N=64, K=576`；
- **1×1 下采样卷积代表层**：例如 Cin=64、Cout=128、输出空间 28×28，对应 `M=784, N=128, K=64`；
- 最后扩展首层 7×7 卷积及其他 stride/padding 配置。

重点检查 16-bit 地址宽度是否足够（例如完整 56×56×64 的 HWC 输入需要 200704 个地址，至少需要 18 bit）、片上存储容量是否允许、是否必须使用 Tile 装载和更实际的数据供给协议。

### 再随后：建立可比较的性能基准

统计/测量：各卷积层 MAC 次数、执行周期、频率、单层延迟、吞吐率、PE 有效利用率、片上/片外传输量，以及 LUT/FF/DSP/BRAM 资源占用。在功能与性能基线稳定后，再通过实验选择数据复用、访存调度或 PE 利用率等重点优化方向。

## 8. 可以用于组会的简短汇报

> 目前已经基于 8×8 INT8 PE 阵列搭建了卷积加速器的 RTL Baseline，包括 M/N/K 分块控制、卷积地址生成、激活和权重同步 BRAM 预取，以及输出存储和 FPGA 顶层接口。完成了小规模 3×3 卷积的联合功能仿真，地址和卷积输出均通过参考值校验。随后使用 Vivado 进行了综合和布局布线，发现主要时序瓶颈位于 K 索引到激活 BRAM 地址的组合逻辑，通过简化卷积地址计算改善了时序。当前版本使用 64 个 DSP，在设置的 100 MHz 时钟约束下，Implementation 的 WNS 为 +0.033 ns，内部已约束时序通过。下一步准备扩展到更大的 M/N/K，首先验证多 N-tile 的完整卷积数据通路，再逐步适配 ResNet-18 典型卷积层。

---

**记录原则：** 本文件仅将已完成的工作标记为“已验证”，对 ResNet-18 整网、板级运行、外部接口和未来优化效果不作提前承诺。后续每完成一轮测试，追加 Testbench、参数、PASS 记录、综合/Implementation 版本和指标，确保基线与优化后结果具有可比性。
