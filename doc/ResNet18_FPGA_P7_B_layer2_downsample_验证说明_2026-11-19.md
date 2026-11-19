# P7-B：真实 layer2.0 带 downsample 的残差块

日期：2026-11-19。基于用户当前分支 `p6-ktiling-accum`，本地与远端均为 `af9606c45c69cdda3d6c8bc4d878e939d66ef0cf`（P7-A）。

**状态：RTL/TB/真实数据准备完成，Python 软件参考和静态文本/端口检查完成。未启动 Vivado，未编译、仿真、综合 HDL，硬件 PASS 等待用户验证。**

上一阶段 P7-A 的后处理单元和完整 `layer1.0` 残差块，用户已提供 Vivado PASS 截图。本阶段保留所有旧文件，独立新增 P7-B 文件，不提交或推送 Git。

## 1. 目标和实际边界

选择 ResNet18 `layer2.0`，输入 `56×56×64`，输出 `28×28×128`。

| mode | 实际层 | 卷积 | Cin / Cout | 每输出完整 K | 每 M 的 C chunk |
|---|---|---|---:|---:|---:|
| 0 | `layer2.0.downsample.0` + BN | 1×1 / stride 2 / padding 0 | 64 / 128 | 64 | 1 × 64 |
| 1 | `layer2.0.conv1` + BN + ReLU | 3×3 / stride 2 / padding 1 | 64 / 128 | 576 | 1 × 576 |
| 2 | `layer2.0.conv2` + BN + add + ReLU | 3×3 / stride 1 / padding 1 | 128 / 128 | 1152 | 2 × 576 |

每个输出 Tile 为 `4×4×8`。每条卷积路径 `7×7×16=784` 个 Tile，整个块共 **2352 次 Tile 启动**。复用同一个 DUT、同一个 8×8 P=2 PE 阵列；全程只复位一次。

顺序：

```text
真实图像 → 软件 FP32 stem + layer1 → 量化后的本块输入 X
                                        │
          ┌─────────────────────────────┴─────────────────────────┐
          ↓                                                       ↓
     mode 0：1×1/S2 + BN                                  mode 1：3×3/S2 + BN
          ↓                                                       ↓
     RTL 重定标，保留符号                                 RTL 重定标 + ReLU
          ↓                                                       ↓
     DUT 同步读回 → shortcut_actual                      DUT 同步读回 → mid_actual
          │                                                       ↓
          │                                              mode 2：3×3/S1 + BN
          │                                              两个 Cin64 chunk 累加
          │                                                       ↓
          └──────────── 同尺度、有符号 INT32 ─────────→ RTL 重定标后相加
                                                                  ↓
                                                           RTL ReLU / INT8 饱和
                                                                  ↓
                                                           同步读回与全局拼接
```

mode 2 的激活来自 **DUT mode 1 实际读回**；skip 来自 **DUT mode 0 实际读回**。Golden 仅参与比较，不替代任何块内层间数据。

这是独立验证 `layer2.0` 的测试。其输入由软件执行 FP32 前序网络并在块边界量化，**没有把 P7-A 的量化输出直接接成完整前序网络**。完成本测试不等于整网分类或上板部署。

## 2. 为什么新增这些适配

原 P5 3×3/S1 Reader 固定 Cin64；此块 Conv2 的 Cin128 需要两次 C chunk 累加。沿用 P6 已验证的“每 M/C chunk 发 load_req、Host 装载、clear_pipe 保留部分和、仅最后 chunk 写结果”的方式，保留原 `k_tile_controller_p2`、`systolic_array_p2` 和 `mac_unit`。

本次提供**线性卷积窗口接口**：Host/TB 将当前 M 的窗口按 `ky,kx,c_local` 排列后，写入局部字节地址 `0..Kchunk-1`。Weight 每个字存放 8 个输出通道。激活地址 `k` 对应 bank `k%2`、bank 地址 `k/2`。直接使用现有同步共享 Activation BRAM 和 Weight Reader，返回两个 K 元素。没有声称这是硬件完整特征图地址生成器；窗口提取、stride、padding 和全局调度均由 Host/TB 负责。

坐标规则独立写明：

- mode 0：`iy=2*(oy+m/4)`，`ix=2*(ox+m%4)`，`c=k`。
- mode 1：`iy=2*(oy+m/4)+ky-1`，`ix=2*(ox+m%4)+kx-1`，`c=k%64`。
- mode 2：`iy=oy+m/4+ky-1`，`ix=ox+m%4+kx-1`，`c=64*c_tile+k%64`。
- 3×3 的 `ky=k/192`，`kx=(k/64)%3`。坐标超出真实输入尺寸才补零。
- mode 1 输入 56、stride 2、padding 1，最后一个输出窗口落到输入索引 55，因此不应人为给底部/右边多加一圈有效计算用零；mode 2 的四条边都涉及 padding。

P7-A 后处理模块固定输出 ReLU INT8，不适合 downsample BN 后的负值。新增 `requant_branch_p7b`，保留旧模块原样。

## 3. 文件和 Vivado 顶层

新增 RTL，加入 **Design Sources**：

- `rtl/requant_branch_p7b.sv`
- `rtl/fpga_system_top_p7b_downsample_tile.sv`

**Design Sources / Synthesis Top：`fpga_system_top_p7b_downsample_tile`。**

新增 TB，加入 **Simulation Sources**，按顺序分别设置 Simulation Top 并 Run All：

| 顺序 | Simulation Top | 用途 |
|---|---|---|
| 1 | `tb_p7b_requant_branch` | 256 个有符号 shortcut、舍入、偏置、饱和及 valid 气泡向量 |
| 2 | `tb_fpga_system_top_p7b_layer2_block` | 完整 2352 Tile 的真实 downsample 残差块 |

需要保留当前版本依赖：`pkg_accelerator.sv`、`k_tile_controller_p2.sv`、`systolic_array_p2.sv`、`mac_unit.sv`、`conv_activation_bram_shared_p5.sv`、`conv_weight_bram_reader_p5_shared.sv`、`sync_fmap_bram.sv`、`sync_weight_bram.sv`、`output_bram.sv`。包放在引用模块之前。已有文件不必重复加入；不要从旧目录导入同名模块。

其他新增内容：

- `reference/generate_p7b_downsample.py`：真实资产、BN 融合、量化、全张量 Golden 和文件导出。
- `reference/check_p7b_vectors.py`：mem 校验、权重打包、2352 Tile 软件映射、C chunk 前缀累加和全局拼接检查。
- `reference/p7b_layer2_0/`：已经生成的 18 个 `.mem`、`manifest.json`、`software_audit.json`。无需重新下载或生成即可仿真。

## 4. 仿真数据路径

TB 先接受 `P7B_DATA_DIR` plusarg；未指定时，先尝试当前 Vivado xsim 工作目录对应的：

```text
../../../../../reference/p7b_layer2_0
```

若失败，再尝试仓库根目录下的 `reference/p7b_layer2_0`。路径探测失败可能由 `$fopen` 打印 WARNING；两条路径都失败或数据不完整，TB 会 `$fatal`，不会误报 PASS。

若使用其他仿真工作目录，可在 xsim additional options 中追加，保留已有选项：

```text
-testplusarg P7B_DATA_DIR=D:/2026PRJ/fpga-ai-accelerator-main/fpga-ai-accelerator-main/reference/p7b_layer2_0
```

未替用户执行任何 Vivado 设置或 Tcl。两个 TB 均需 Run All；完整块会打印 2352 行 Tile 信息和 3 行 Phase 信息，再给出最终结论。

## 5. 整数数值契约

模型来自 [torchvision ResNet18 IMAGENET1K_V1](https://docs.pytorch.org/vision/2.0/models/generated/torchvision.models.resnet18.html)，使用与 P7-A 相同的官方 checkpoint 和 PyTorch Hub `dog.jpg`。完整来源 URL、SHA256、框架版本、各通道 scale 见 manifest。约 45 MB 原始 checkpoint 仍放工程外缓存，没有复制到仓库。

按通道 BN 融合：

```text
factor[n] = gamma[n] / sqrt(running_var[n] + eps)
Wf[...,n] = W[...,n] * factor[n]
bf[n] = beta[n] - running_mean[n] * factor[n]
Sw[n] = max(abs(Wf[...,n])) / 127
Wq = clamp(round_away(Wf/Sw), -127, 127)
bias[n] = round_away(bf[n] / (Sin*Sw[n]))
mult[n] = round_away((Sin*Sw[n]/Sout) * 2^24)
branch = round_away((acc + bias[n]) * mult[n] / 2^24)
```

`round_away` 是最近整数、半整数远离零；zero point 为 0。输入/Conv1 中间激活为非负 signed INT8 `[0,127]`。按这张图像分别标定输入 scale、Conv1 输出 scale 和最终输出 scale；**捷径输出与 Conv2 输出统一重定标到最终输出 scale**。

- mode 0：保存 signed INT32 `branch`，无 ReLU，无 INT8 饱和。
- mode 1：输出 `clamp(branch,0,127)`，接口上零扩展成 32 bit。
- mode 2：输出 `clamp(branch + shortcut_actual,0,127)`，接口上零扩展成 32 bit。

两条分支先各自按 Q24 舍入到同一尺度，再相加；不声称逐位等价于 PyTorch FBGEMM 的量化实现。不能在相加前提前剪裁某条分支。

后处理使用 33-bit 偏置和、65-bit 乘积、66-bit 舍入和相加。mode 0 若超出 signed INT32 会置 overflow，测试会失败；本次所有数据均在可表示范围。mode 1/2 的 `[0,127]` 饱和属于约定量化行为，不当作溢出。

配置在 start 被接受时锁存；TB 随后把外部 mode 改为保留值并把偏置/乘数置 X，确认运行不依赖变化中的配置。每 M 后处理完成并实际写入输出 RAM 后才进入下一 M；最后一字落 RAM 后才 done。

## 6. 独立 Golden 与验收

Python Golden 直接计算完整输入张量，NumPy INT64 卷积与 PyTorch double 的精确整数值卷积逐点一致；BN 融合另与原 FP32 运算核对。Golden 不使用 DUT、Host Tile 地址计算或 PE 内部信号。

Conv2 额外导出输入通道 0..63 的完整卷积前缀 `prefix2.mem`，检查 C0 的 snapshot；C1 snapshot 检查完整 Cin128 结果。TB 同时维护每个 PE 的参考和，并检查 Host 等待前后及 clear_pipe 时保留的 64 个累加值。只有最终 C chunk 可以产生输出。

| 指标 | mode 0 | mode 1 | mode 2 | 总计 |
|---|---:|---:|---:|---:|
| Tile | 784 | 784 | 784 | 2352 |
| load ack / K segment | 12544 | 12544 | 25088 | 50176 |
| paired FEED | 401408 | 3612672 | 7225344 | 11239424 |
| 原始 INT32 写字 | 12544 | 12544 | 12544 | 37632 |
| 后处理写字 | 12544 | 12544 | 12544 | 37632 |
| INT32 Golden 比较数 | 100352 | 100352 | 100352 | 301056 |
| 同步 lane 读回数 | 100352 | 100352 | 100352 | 301056 |

另外检查：

- 全程单 DUT、单次复位，2352 次 start/done 连续执行。
- 50176 次 pipeline clear、37632 次 accumulator clear。
- 22478848 个 Activation 字节写入、同样数量的 64-bit Weight 字写入；这是当前逐 M/C 重装载方案的功能开销，不是优化后的带宽指标。
- 850944 个 padding 标量装载，并在 FEED 中逐个核对；这里不是 pair 数。
- 所有 bank 请求地址、局部 K 顺序、两路激活和两组权重对齐。
- 同步输出读延迟、lane mask、写地址、缓存覆盖；各阶段 100352 个全局 HWC 位置恰好写入一次。
- 每次等待 load_req/done 最多 2000 周期，另有 2 秒仿真时间的全局 watchdog。

软件侧已经核对（不是 HDL PASS）：

- 2352 Tile、301056 个 INT32、100352 个最终输出全部与独立 Golden 一致。
- 捷径输出中 **44674 个负数**得以保留。
- 捷径相加影响 **54079** 个最终输出；错误地提前对捷径做 ReLU 会改变 **16537** 个最终输出。
- 预计 **234225** 个缓存 lane 与上一个 Tile 值不同。
- 本图像本块反量化输出相对 FP32 的 MAE 约 **0.00724548**，相对 L2 误差约 **4.08932%**。单图校准仅用于功能实验，不表示 ImageNet 分类准确率。

预期单元仿真末尾：

```text
PASS: P7-B signed shortcut/requant 256/256 vectors, no premature ReLU, signed rounding and valid bubbles
```

预期完整块仿真末尾：

```text
PASS: P7-B REAL RESNET18 layer2.0 DOWNSAMPLE RESIDUAL BLOCK, 2352 Tiles, 301056/301056 INT32 accumulators, 100352/100352 final INT8 outputs
```

只在用户实际得到上述检查通过后，才能记录本阶段硬件仿真完成。

## 7. 可选软件复现

从工程根目录，软件核对：

```powershell
python reference/check_p7b_vectors.py
```

重新生成（Python、NumPy、PyTorch、torchvision、Pillow 版本见 manifest）：

```powershell
python reference/generate_p7b_downsample.py --cache D:/resnetcnn/experiments/p7a_residual/assets
```

缓存没有官方文件时可加 `--download`。脚本不调用 HDL 工具。单独核对脚本使用 NumPy，并导入生成脚本的标量规则，因此也需要生成脚本所依赖的 Python 包。

新增顶层和后处理尚未做综合/Implementation；不能沿用旧 P5/P2 的 WNS、资源或 100 MHz 达标结论。通过该块后再推进跨块尺度衔接、其他残差块和整网调度。
