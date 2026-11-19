# P7-A：真实 ResNet18 layer1.0 残差块验证准备

日期：2026-11-19。基于 `p6-ktiling-accum` 分支，提交 `b2b20b50591b14e9bebc5ccad48ed49a8fff81f1`。

**当前状态：新增 RTL、TB、真实权重测试数据与独立软件参考已准备；仅运行 Python 软件检查和源文本/端口静态检查，未启动 Vivado，未编译或仿真 HDL，未综合/Implementation。下述硬件 PASS 均为待用户运行的验收标准。**

## 1. 这次验证什么

选择 torchvision ResNet18 的第一个 BasicBlock：`layer1.0`。输入和输出均为 `56×56×64`，identity 分支没有 downsample。

数据流：

```text
真实图像 → 软件 FP32 stem → 量化后的 block input X（56×56×64）
                           │
                           ├────────────────────────────────────┐
                           ↓                                    │
                DUT conv1（3×3/S1/P1）                           │
                           ↓                                    │
                RTL 融合 BN 偏置 + 重定标 + ReLU                  │
                           ↓                                    │
                同步输出 RAM 读回 → mid_actual                   │
                           ↓                                    │
                DUT conv2（3×3/S1/P1）                           │
                           ↓                                    │
                RTL 融合 BN 偏置 + 重定标                        │
                           ↓                                    ↓
                RTL residual add ←──────────────────────────────X
                           ↓
                RTL ReLU + INT8 饱和 → 同步 RAM → 完整输出拼接
```

两层各使用 392 个 `8×8×8` 输出 Tile，共 784 次启动。复用现有 `conv_accelerator_top_p5_multimode` 的 mode 0：单个共享 P=2 卷积核心、原有 8×8 PE 阵列、共享 Activation/Weight BRAM。没有改动原 PE、K/M/N Controller 或 Reader。

由 TB 模拟 Host 装载、启动、等待、读回和全局特征图缓存。两层实际复用同一个 DUT，一次复位连续执行。**第二层的输入只来自 DUT 第一层输出 RAM 的读回数组 `mid_actual`，不能来自 `golden1`。** identity 分支使用原始量化块输入。

这属于**真实预训练权重、真实图像输入、TB 调度的完整残差块数值验证**。stem 在软件中计算；还不是整网 RTL、自动硬件整块调度、AXI/DDR 集成或上板部署。

## 2. 新增文件与顶层

| 文件 | 用途 / 添加位置 |
|---|---|
| `rtl/requant_residual_p7.sv` | 新增按通道偏置、Q24 重定标、残差相加及 ReLU；Design Sources |
| `rtl/fpga_system_top_p7_residual_tile.sv` | 复用 P5 核心的 Tile 顶层，集成 skip 缓存和 INT8 输出 RAM；Design Sources |
| `tb/tb_p7_requant_residual.sv` | 256 个定向/随机后处理向量；Simulation Sources |
| `tb/tb_fpga_system_top_p7_layer1_block.sv` | 完整真实 `layer1.0` 两层连续计算；Simulation Sources |
| `reference/generate_p7a_residual.py` | 官方模型/图像导入、BN 融合、量化、独立完整卷积 Golden 和 mem 导出 |
| `reference/check_p7a_vectors.py` | mem SHA256、打包/Tile 映射、拼接和整数算术的软件检查 |
| `reference/p7a_layer1_0/*.mem` | 已生成的输入、两层权重/偏置/乘数、Golden 和后处理向量 |
| `reference/p7a_layer1_0/manifest.json` | 资产来源、SHA256、版本、量化参数及软件误差记录 |

**Design Sources 综合顶层：`fpga_system_top_p7_residual_tile`。**

按顺序设置 Simulation Top，分别运行 `Run All`：

1. `tb_p7_requant_residual`
2. `tb_fpga_system_top_p7_layer1_block`

已有模块继续使用当前分支版本。依赖为 `pkg_accelerator`、`conv_accelerator_top_p5_multimode`、`gemm_mn_controller_mem_p2`、`k_tile_controller_p2`、`systolic_array_p2`、`mac_unit`、`conv_bram_reader_p3`、`conv_activation_line_buffer_p3`、`conv_bram_reader_p4_1x1`、`conv_bram_reader_p4_3x3s2`、`conv_activation_bram_shared_p5`、`conv_weight_bram_reader_p5_shared`、`sync_fmap_bram`、`sync_weight_bram`、`output_bram`。包应先于引用它的模块编译；已在当前工程中的文件无需重复添加。不要将其他目录的同名旧版本再次加入。

TB 不加入 Design Sources。本次无需先做综合，先验证两个 Simulation Top。

## 3. 数据路径与运行

数据已经生成，不必先安装 Python 或重新下载模型。

两个 TB 都优先接收 `P7_DATA_DIR` plusarg；未指定时尝试：

1. `reference/p7a_layer1_0`（从仓库根目录运行）。
2. `../../../../../reference/p7a_layer1_0`（当前工程默认的 `fpga_ai_accelerator/fpga_ai_accelerator.sim/sim_1/behav/xsim` 工作目录）。

若更换工程位置或仿真目录，在 Simulation Settings 的 xsim additional options 中追加：

```text
-testplusarg P7_DATA_DIR=D:/2026PRJ/fpga-ai-accelerator-main/fpga-ai-accelerator-main/reference/p7a_layer1_0
```

这只是提供给用户的设置说明，没有执行这些操作。不要覆盖已有的其他仿真选项。`-testplusarg` 的含义见 [AMD UG900](https://docs.amd.com/r/en-US/ug900-vivado-logic-simulation/xsim-Executable-Options)。缺文件、不完整/X 数据、超时均会 `$fatal`，不会误打印 PASS。

首次完整仿真会有 784 行 `TILE PASS` 和两行 `LAYER PASS`，其后才有最终 PASS。耗时取决于本地仿真器和波形记录设置。

## 4. 真实资产来源和独立 Golden

- 模型：[torchvision ResNet18 IMAGENET1K_V1](https://docs.pytorch.org/vision/2.0/models/generated/torchvision.models.resnet18.html)。
- 权重：[resnet18-f37072fd.pth](https://download.pytorch.org/models/resnet18-f37072fd.pth)。SHA256：`f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec`。
- 图像：[PyTorch Hub 示例 dog.jpg](https://raw.githubusercontent.com/pytorch/hub/master/images/dog.jpg)，使用官方 weights transforms 预处理。SHA256：`f3f87bb8ab3c26c7ecfd3ac60421d7f32b0503d1d6c5baf8bac42ed93d86351a`。
- 官方 [BasicBlock 定义](https://github.com/pytorch/vision/blob/main/torchvision/models/resnet.py)：conv1/BN/ReLU → conv2/BN → identity add → ReLU。

完整约 45 MB checkpoint 和原始图像留在工作缓存，未复制进 Git 工程。工程仅包含本块所需的量化权重、输入和 Golden，来源与哈希保留在 manifest 中。上游模型/样本的许可和使用条件仍由各自来源决定。

Golden 在 Python 中直接计算完整 `56×56×64` 张量卷积，使用 NumPy INT64 累加，并与另一套 PyTorch double 的整数值卷积逐点核对；相关中间值远低于 double 精确整数上限。Golden 不使用 DUT 输出、局部 Tile 地址生成或 PE 内部状态。

`check_p7a_vectors.py` 另行检查导出的 mem、每字 8 路权重的符号和打包、784 个局部 Tile 与全局 Golden 的映射、全局拼接，以及 Python 标量定点算术。

## 5. 固定数值契约

这是一套明确的硬件整数规范，不声称与 torchvision 的 FBGEMM 量化模型逐位兼容。FP32 模型为来源参考，硬件应逐位一致的对象是以下整数 Golden。

1. eval 模式下融合 BN：`Wf = W * gamma/sqrt(var+eps)`，`bf = beta - mean*gamma/sqrt(var+eps)`。本块原卷积无 bias。
2. INT8 权重按输出通道对称量化：`Sw[n] = max(abs(Wf[:,:,:,n]))/127`，`Wq = clamp(round_away(Wf/Sw), -127,127)`。
3. 非负激活按张量量化，zero point 为 0，范围 `[0,127]`。块输入与最终输出共用 `S0`，第一层输出使用 `S1`。本次单图校准：`S0=0.026280711016317054`，`S1=0.010387958504083588`。
4. `bias_q[n] = round_away(bf[n]/(Sin*Sw[n]))`，保存为 INT32。
5. `mult[n] = round_away((Sin*Sw[n]/Sout)*2^24)`，保存为非负有符号 INT32。
6. `branch = round_away((acc + bias_q)*mult/2^24)`。
7. 第一层：`q1=clamp(branch,0,127)`。
8. 第二层：`q2=clamp(branch + input_q,0,127)`。**在残差相加之前不能对 branch 做 ReLU 或 INT8 饱和。**

`round_away` 表示四舍五入到最近整数，恰好半整数时远离零。RTL 使用 33-bit 偏置加法、65-bit 乘积和 66-bit 舍入/残差加法，最终才截到 8 bit。没有把有符号权重当 unsigned，也没有将残差加法提前截成 INT8。

新增后处理使用两级寄存器。地址和 mask 同步延迟；顶层等卷积完成且 64 个后处理结果已写入输出 RAM 后才发 `done`。bias/multiplier/residual_en 在 start 被接受时锁存，TB 在启动后改变外部配置以检查锁存有效。

现有 `quantize_unit.sv` 只有移位量化，缺少按通道乘数、BN 偏置和 residual add，本次保留它不变，使用独立新模块。

## 6. 验收计数与覆盖

| 指标 | 完整残差块预期 |
|---|---:|
| 卷积层数 | 2 |
| Tile / start / done | 784 |
| 每 Tile M/N/K | 64 / 8 / 576 |
| 有效 paired FEED | 14,450,688 |
| INT32 原始结果写入 | 50,176 个 8-lane 字 |
| INT32 Golden 比较 | 401,408 个数 |
| INT8 后处理结果写入 | 50,176 个 8-lane 字 |
| 同步 INT8 lane 读回 | 401,408 个数 |
| 每层独立全局拼接 | 200,704 个位置，各写一次 |
| 最终残差块 INT8 输出 | 200,704 |
| Activation 装载 | 5,017,600 字节 |
| Weight 装载 | 451,584 个 64-bit 字 |
| Skip 装载 | 50,176 个 64-bit 字（两层都装载，第一层关闭残差） |
| 权重组/层切换 | 783 |
| 边界 padding 装载 | 282,624 字节 |
| FEED 中 padding K 元素 | 684,032 个标量，不是 pair 数 |

每个有效 FEED 同时检查两路激活与两组权重；请求检查 K 次序和两 bank 地址；输出检查地址、mask、INT32 值、后处理值、同步读延迟、缓存覆盖和全局写入唯一性。每 Tile 有 40,000 周期等待上限，另有全局 watchdog。

独立后处理 TB 覆盖 256 个向量：正负半整数舍入、INT32 极值及偏置和超过 INT32 的情况、零乘数、饱和、残差开关、负数分支被 skip 恢复为正数、连续输入和 valid 气泡。

软件检查目前记录：

- 784 个 Tile 的导出布局与 Golden 一致；401,408 个 INT32、200,704 个最终 INT8 校验一致。
- 预计 357,061 个输出缓存 lane 的新值与前一个 Tile 不同。
- 残差相加改变 141,484 个最终量化输出；其中 57,962 个位置的负分支经 skip 相加后变为正。
- 与原 FP32 残差块输出相比，反量化 INT8 输出 MAE 约 0.0110152，相对 L2 误差约 2.83035%。这只是该图像该块的量化误差，**不是分类准确率**。

预期控制台末尾（只有用户实际仿真满足检查后才会出现）：

```text
PASS: P7-A requant/residual 256/256 vectors, signed rounding, bias, saturation and pipeline bubbles
```

完整块 TB：

```text
PASS: P7-A REAL RESNET18 layer1.0 RESIDUAL BLOCK, 784 Tiles, 401408/401408 INT32 accumulators, 200704/200704 final INT8 outputs
```

## 7. 软件复现（可选）

已有 mem 可直接用于仿真。若要核对其哈希、软件布局和数值规则，从仓库根目录运行：

```powershell
python reference/check_p7a_vectors.py
```

需要重新生成时，安装对应 Python/NumPy/PyTorch/torchvision/Pillow；版本见 manifest。缓存目录放在工程外，首次下载加 `--download`：

```powershell
python reference/generate_p7a_residual.py --cache D:/resnetcnn/experiments/p7a_residual/assets --download
```

不同框架版本、预处理或图像会影响量化参数和数据。软件复现检查不会启动任何 HDL 工具。

## 8. 后续边界

两个新 TB 均由用户仿真通过后，才能记录“真实残差块在本整数契约下端到端 PASS”。新增后处理的资源和时序仍需单独综合/Implementation；不能继承旧 P5/P2 的 WNS 结论。

之后再处理具有 downsample 的残差块、池化/全连接、跨块量化尺度、真实整网层间缓存和调度。当前一步没有修改已有 P1/P2/P5/P6 已通过文件，也没有提交或推送 Git。
