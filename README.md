

<h1 align="center">FPGA AI Inference Accelerator</h1>

<p align="center">
  <strong>An open-source 8×8 INT8 systolic array inference engine — from RTL to bitstream on a $130 FPGA board.</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Language-SystemVerilog-blue?style=flat-square" alt="SystemVerilog"/>
  <img src="https://img.shields.io/badge/Target-Xilinx_Artix--7-E31937?style=flat-square" alt="Artix-7"/>
  <img src="https://img.shields.io/badge/Precision-INT8-green?style=flat-square" alt="INT8"/>
  <img src="https://img.shields.io/badge/Simulators-iverilog_%7C_XSim_%7C_Verilator-orange?style=flat-square" alt="Simulators"/>
  <img src="https://img.shields.io/badge/Tests-10%2F10_PASS-brightgreen?style=flat-square" alt="Tests"/>
  <img src="https://img.shields.io/badge/Timing-100_MHz_MET-brightgreen?style=flat-square" alt="Timing Met"/>
  <img src="https://img.shields.io/badge/License-MIT-yellow?style=flat-square" alt="MIT License"/>
</p>

---

## What This Is

A **production-grade neural network inference accelerator** built entirely from scratch in SystemVerilog. No vendor IP. No HLS. Just 2,000 lines of hand-written RTL that implements:

- **8×8 systolic array** of INT8 multiply-accumulate (MAC) units
- **Automatic tiling engine** that handles any matrix dimension — not just powers of 2
- **Dual-bank SRAM** with zero-conflict interleaved read/write
- **Full requantization pipeline**: ReLU → arithmetic right-shift → zero-point → INT8 saturation
- **AXI4-Stream interfaces** for plug-and-play system integration
- **IP Integrator block design** with VIO debug, ready for Arty A7 deployment

Verified across 3 simulators with 784 output values matched. Synthesized, placed, routed, and timed at **100 MHz with positive slack** on Artix-7.

<p align="center">
  <img width="3238" height="1821" alt="Screenshot from 2026-02-16 08-39-13_testcase1" src="https://github.com/user-attachments/assets/3419c4f7-ebb5-4973-881c-dc559f42e680" />
  <br/>
  <em>XSim waveform: 24×40 asymmetric matrix — 15 tiles, 240 outputs verified</em>
</p>
<p align="center">
  <img width="3238" height="1821" alt="Screenshot from 2026-02-16 11-00-19_testcase3" src="https://github.com/user-attachments/assets/2dd8f2f6-2889-4ecc-9b76-619928313c91" />
  <br/>
  <em>XSim waveform: 32×32 large tile — 16 tiles, 8 spatial batches, 256 outputs verified</em>
</p>

---

## Why This Matters

Edge AI inference doesn't need a GPU. For latency-critical applications — robotics, industrial inspection, medical devices, autonomous systems — a purpose-built FPGA accelerator running at **deterministic, microsecond-level latency** beats a GPU every time.

This project proves you can build one from first principles, verify it rigorously, and target a **$130 development board**.

---

## 🏗️ Architecture

```
                         ┌─────────────────────────────────────┐
                         │       Layer Control FSM             │
                         │  (orchestrates all tile sequencing) │
                         └───┬─────────┬─────────┬─────────┬───┘
                             │         │         │         │
                    ┌────────▼──┐  ┌───▼───┐  ┌──▼──┐  ┌──-▼─────────┐
  Activations ─────►│ AXI-Stream│  │ Dual  │  │ 8×8 │  │ Quantize    │────► Results
  (AXI4-S)          │   Input   │  │ Bank  │  │Sys. │  │   Unit      │     (AXI4-S)
                    │ + Skew    │  │ SRAM  │  │Array│  │ ReLU+Shift  │
                    └───────────┘  └───────┘  └─────┘  │ +Saturate   │
                                       │               └─────────────┘
  Weights ─────────────────────────────┘
  (DMA)
```

The accelerator processes one layer at a time. The FSM breaks large matrices into 8×8 tiles and sequences through them:

1. **Load weights** into inactive SRAM bank via DMA
2. **Stream activations** through AXI4-Stream, apply diagonal skew for systolic timing
3. **Compute** — 64 MACs fire in parallel, accumulating partial sums across IC tiles
4. **Drain** pipeline, then **requantize** (ReLU → shift → zero-point → saturate to INT8)
5. **Output** results via AXI4-Stream master
6. **Repeat** for next tile until all OC × IC tiles complete

---

## 📊 Resource Utilization

Synthesized and routed on **Xilinx Artix-7 (xc7a100tcsg324-1)** at 100 MHz:

| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| **DSP48E1** | 65 | 240 | 27% |
| **RAMB36** | 6 | 135 | 4% |
| **LUT** | 8,871 | 63,400 | 14% |
| **FF** | 6,289 | 126,800 | 5% |

**Timing:** WNS = **+0.100 ns** (100 MHz met with margin)

65 DSPs = 64 MAC units + 1 address calculator. Plenty of headroom to scale to 16×16 on the same chip.

---

## ✅ Verification

**10/10 tests PASS — 784/784 outputs matched** across three independent simulators:

| Test | Dimensions | Tiles | Outputs | What It Catches |
|------|-----------|-------|---------|----------------|
| `identity_8x8` | 8×8, 1 spatial | 1 | 8 | Basic datapath wiring |
| `boundary_max` | 8×8, +127 all | 1 | 8 | Positive overflow, saturation |
| `boundary_min` | 8×8, -128×+127 | 1 | 8 | Signed arithmetic, sign extension |
| `relu_activation` | 8×8, 2 spatial | 1 | 16 | ReLU clamping, negative outputs |
| `random_small` | 8×8, 4 spatial | 1 | 32 | Data-dependent bugs |
| `multi_tile` | 16×16, 1 spatial | 4 | 64 | Tile sequencing, SRAM addressing |
| `deep_ic` | 32×8, 1 spatial | 4 | 32 | Partial sum accumulation across IC |
| `deep_oc` | 8×32, 1 spatial | 4 | 128 | Output ordering, weight reloading |
| `large_tile` | 32×32, 8 spatial | 16 | 256 | Full-scale stress test |
| `asymmetric` | 24×40, 1 spatial | 15 | 240 | Non-power-of-2 edge cases |

<p align="center">
  <img width="496" height="500" alt="Screenshot from 2026-02-16 20-19-05" src="https://github.com/user-attachments/assets/e7ba80ff-00e8-4715-a9cb-df32361bada0" />
  <br/>
  <em>All 10 tests PASS with bit-exact match against Python/NumPy golden model</em>
</p>

A **bit-accurate Python golden model** replicates the exact tiling order, accumulation pattern, and requantization pipeline of the hardware — not just a `numpy.matmul()`.

---

## 🔌 Block Design (IP Integrator)

The accelerator is packaged as reusable Vivado IP and integrated into a complete system:

<p align="center">
  <img width="3065" height="1601" alt="Screenshot from 2026-02-16 14-33-06" src="https://github.com/user-attachments/assets/950f2376-c221-4927-ac55-a5eb1304fbf4" />
  <br/>
  <em>IP Integrator block design: Clocking Wizard + Reset + Accelerator IP + VIO Debug + AXI-Stream FIFO</em>
</p>

| Block | Purpose |
|-------|---------|
| **Clocking Wizard** | PLL-based 100 MHz from board oscillator |
| **Proc Sys Reset** | Synchronized reset, held until PLL locks |
| **Accelerator IP** | Your 8×8 INT8 systolic array engine |
| **VIO** | JTAG debug — control start/config without a processor |
| **AXI-Stream FIFO** | Loopback for hardware self-test |

---

## 🚀 Quick Start

### Prerequisites

- **Icarus Verilog** 12+ (`sudo apt install iverilog`) — for simulation
- **Python 3.8+** with NumPy — for golden model / test generation
- **Vivado 2024.x+** — for synthesis, block design, and bitstream (optional)

Expected output:
```
=== test_identity_8x8 ===      PASS (8/8 matched)
=== test_boundary_max ===      PASS (8/8 matched)
=== test_boundary_min ===      PASS (8/8 matched)
=== test_relu_activation ===   PASS (16/16 matched)
=== test_random_small ===      PASS (32/32 matched)
=== test_multi_tile ===        PASS (64/64 matched)
=== test_deep_ic ===           PASS (32/32 matched)
=== test_deep_oc ===           PASS (128/128 matched)
=== test_large_tile ===        PASS (256/256 matched)
=== test_asymmetric ===        PASS (240/240 matched)

RESULT: 10/10 tests PASSED (784/784 outputs matched)
```

---

## 📁 Repository Structure

```
fpga-ai-accelerator/
├── rtl/                              # Synthesizable RTL (9 modules)
│   ├── pkg_accelerator.sv            #   Shared types, parameters, functions
│   ├── mac_unit.sv                   #   Single multiply-accumulate PE
│   ├── systolic_array.sv             #   8×8 systolic array (64 MACs)
│   ├── sram_controller.sv            #   Dual-bank weight SRAM
│   ├── axi_stream_input.sv           #   AXI4-Stream slave + diagonal skew
│   ├── axi_stream_output.sv          #   AXI4-Stream master
│   ├── quantize_unit.sv              #   ReLU + shift + saturate pipeline
│   ├── layer_ctrl_fsm.sv             #   11-state tiling FSM
│   └── inference_accelerator_top.sv  #   Top-level integration
│
├── tb/                               # Testbench
│   └── tb_inference_accelerator.sv   #   Universal test driver
│
├── verification/                     # Verification framework
│   ├── scripts/
│       ├── golden_model.py           #   Bit-accurate Python reference
│       └── test_generator.py         #   Automated test vector generation
└── README.md         
```

---

## 🧠 How the Systolic Array Works

Each PE (processing element) performs `acc += act × wgt` every cycle. Activations flow left-to-right with diagonal skew; weights are broadcast column-wise from SRAM. After all IC tiles accumulate, the accumulators drain through the quantization pipeline:

```
INT8 act × INT8 wgt → 16-bit product → 32-bit accumulator
                                              │
                                    ┌─────────▼─────────┐
                                    │  Optional ReLU    │
                                    │  max(0, acc)      │
                                    ├───────────────────┤
                                    │  Arithmetic >>    │
                                    │  (divide by 2^N)  │
                                    ├───────────────────┤
                                    │  + Zero Point     │
                                    ├───────────────────┤
                                    │  Saturate to INT8 │
                                    │  [-128, +127]     │
                                    └─────────┬─────────┘
                                              │
                                         INT8 output
```

This is the same quantization scheme used by TensorFlow Lite, ONNX Runtime, and other production inference frameworks.

---

## 🔧 Extending the Design

**Scale to 16×16 array:** Change `ARRAY_ROWS` and `ARRAY_COLS` in `pkg_accelerator.sv`. Artix-7 100T has 240 DSPs — enough for 256 MACs.

**Add a processor:** Replace VIO with MicroBlaze or RISC-V soft core for software-driven control.

**Add DMA:** Connect weight port to AXI DMA for streaming weight loading from DDR.

**Multi-layer inference:** Chain layers by connecting output AXI-Stream back to input with double-buffering.

**Target a larger FPGA:** Zynq UltraScale+ gives you ARM cores + 2,500+ DSPs for massive arrays.

---

## ⚡ Performance

| Metric | Value |
|--------|-------|
| Clock frequency | 100 MHz |
| MACs per cycle | 64 |
| Peak throughput | 6.4 GOPS (INT8) |
| Latency (8×8 tile) | ~15 cycles (150 ns) |
| Target board | Arty A7-100T  |
| Power (estimated) | < 1W accelerator core |

---

## 🤝 Contributing

Contributions welcome!

Areas where help is especially valuable:
- **Benchmarks** against TFLite / ONNX Runtime on real models (MNIST, MobileNet)
- **Zynq port** with PS-PL integration and Linux driver
- **Multi-layer support** with layer descriptor FIFO
- **Performance profiling** on actual hardware

---

## 📄 License

This project is licensed under the [MIT License](LICENSE). Use it, learn from it, build on it.

---

## 🙏 Acknowledgments

Built with SystemVerilog, verified with open-source tools (Icarus Verilog, Verilator), synthesized with AMD/Xilinx Vivado. Architecture inspired by Google's TPU v1 systolic array design and Stanford's Gemmini accelerator.

---

<p align="center">
  <strong>If this project helped you learn or build something, please ⭐ star the repo!</strong>
</p>

---

## Author

**John Bagshaw** -- Founder, [ShawSilicon](https://shawsilicon.ai) | Senior FPGA Design Engineer, 8+ years in production FPGA/RTL design

CXL protocol integration, DDR4 memory controllers, multi-clock domain architectures, AXI4/Avalon-MM interfaces, and production verification on Intel Agilex 7 and AMD Zynq UltraScale+ platforms.

ShawSilicon runs fixed-scope RTL audits and verifies chip engineers on a four-stage technical screen. [shawsilicon.ai](https://shawsilicon.ai)

[![GitHub](https://img.shields.io/badge/GitHub-taitashaw-181717?style=flat-square&logo=github)](https://github.com/taitashaw)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-jotshaw-0A66C2?style=flat-square&logo=linkedin)](https://www.linkedin.com/in/jotshaw/)
[![Website](https://img.shields.io/badge/Website-shawsilicon.ai-FF6B35?style=flat-square&logo=safari)](https://shawsilicon.ai)
