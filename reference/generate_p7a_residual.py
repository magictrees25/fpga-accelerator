"""Export a real ResNet18 layer1.0 INT8 residual-block test, without HDL tools.

Golden convolutions operate on whole HWC tensors, independently of RTL tiling.
Python 3, numpy, torch, torchvision, Pillow are required. See the P7-A guide.
"""
import argparse
import hashlib
import json
import platform
from pathlib import Path

import numpy as np
import torch
import torchvision
from PIL import Image
from torchvision.models import ResNet18_Weights, resnet18

WEIGHT_URL = "https://download.pytorch.org/models/resnet18-f37072fd.pth"
IMAGE_URL = "https://raw.githubusercontent.com/pytorch/hub/master/images/dog.jpg"
SHIFT = 24


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def round_away(x):
    return (np.sign(x) * np.floor(np.abs(x) + 0.5)).astype(np.int64)


def rounded_fixed(x):
    x = np.asarray(x, dtype=np.int64)
    return np.sign(x) * ((np.abs(x) + (1 << (SHIFT - 1))) >> SHIFT)


def conv_full(x, w):
    """HWC x HWIO -> HWO, stride=1, padding=1, signed INT64 reference."""
    h, width, _ = x.shape
    padded = np.pad(x.astype(np.int64), ((1, 1), (1, 1), (0, 0)))
    result = np.zeros((h, width, w.shape[3]), dtype=np.int64)
    for ky in range(3):
        for kx in range(3):
            result += padded[ky:ky+h, kx:kx+width] @ w[ky, kx].astype(np.int64)
    # Independent full-tensor PyTorch convolution, with exact integer-valued
    # double arithmetic (all intermediates are far below 2**53).
    xt = torch.from_numpy(x.transpose(2, 0, 1).copy()).double()[None]
    wt = torch.from_numpy(w.transpose(3, 2, 0, 1).copy()).double()
    other = torch.nn.functional.conv2d(xt, wt, padding=1)[0].permute(1, 2, 0).numpy()
    assert np.array_equal(result, other), "independent full convolution mismatch"
    assert result.min() >= -(1 << 31) and result.max() < (1 << 31)
    return result


def emit(directory, name, values, bits):
    values = np.asarray(values).reshape(-1)
    mask = (1 << bits) - 1
    path = directory / (name + ".mem")
    path.write_text("".join(f"{int(v) & mask:0{bits // 4}x}\n" for v in values), encoding="ascii")
    back = np.array([int(s, 16) for s in path.read_text().splitlines()], dtype=np.uint64)
    assert np.array_equal(back, np.array([int(v) & mask for v in values], dtype=np.uint64))
    return {"elements": len(values), "bits": bits, "sha256": sha(path)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path(__file__).parent / "p7a_layer1_0")
    parser.add_argument("--cache", type=Path, required=True, help="checkpoint/image cache; outside Git")
    parser.add_argument("--download", action="store_true", help="allow downloading missing official assets")
    args = parser.parse_args()
    args.cache.mkdir(parents=True, exist_ok=True)
    args.out.mkdir(parents=True, exist_ok=True)
    checkpoint, photo = args.cache / "resnet18-f37072fd.pth", args.cache / "dog.jpg"
    for path, url in ((checkpoint, WEIGHT_URL), (photo, IMAGE_URL)):
        if not path.exists():
            if not args.download:
                raise FileNotFoundError(f"Missing {path}; use --download or supply the cached file")
            torch.hub.download_url_to_file(url, str(path), progress=False)
    assert sha(checkpoint).startswith("f37072fd"), "official checkpoint hash mismatch"
    torch.set_num_threads(4)
    torch.manual_seed(0)
    model = resnet18(weights=None).eval()
    model.load_state_dict(torch.load(checkpoint, map_location="cpu", weights_only=True))
    image = ResNet18_Weights.IMAGENET1K_V1.transforms()(Image.open(photo).convert("RGB"))[None]
    block = model.layer1[0]
    with torch.no_grad():
        inp = model.maxpool(model.relu(model.bn1(model.conv1(image))))
        mid = block.relu(block.bn1(block.conv1(inp)))
        fpout = block(inp)
    to_hwc = lambda t: t[0].permute(1, 2, 0).numpy().astype(np.float64)
    xfp, midfp, yfp = map(to_hwc, (inp, mid, fpout))
    assert xfp.shape == (56, 56, 64) and block.downsample is None
    # Shared input/output scale makes the identity residual an exact integer add.
    s0 = float(max(xfp.max(), yfp.max()) / 127)
    s1 = float(midfp.max() / 127)
    qx = np.clip(round_away(xfp / s0), 0, 127)
    files = {}
    files["input.mem"] = emit(args.out, "input", qx, 8)
    stage = qx
    layers = []
    for i, (conv, bn, si, so) in enumerate(((block.conv1, block.bn1, s0, s1),
                                           (block.conv2, block.bn2, s1, s0)), 1):
        factor = bn.weight.detach().double() / torch.sqrt(bn.running_var.double() + bn.eps)
        folded_w = conv.weight.detach().double() * factor[:, None, None, None]
        folded_b = bn.bias.detach().double() - factor * bn.running_mean.double()
        # Verify BN fusion against the original evaluation-mode block operators.
        with torch.no_grad():
            test_in = inp if i == 1 else mid
            original = bn(conv(test_in))
            fused = torch.nn.functional.conv2d(test_in.double(), folded_w, folded_b, padding=1)
            fusion_error = float((original.double() - fused).abs().max())
            assert fusion_error < 1e-4
        w = folded_w.numpy().transpose(2, 3, 1, 0)
        sw = np.maximum(np.max(np.abs(w), axis=(0, 1, 2)) / 127, np.finfo(np.float64).tiny)
        qw = np.clip(round_away(w / sw), -127, 127)
        bias = round_away(folded_b.numpy() / (si * sw))
        mult = round_away((si * sw / so) * (1 << SHIFT))
        assert np.all((mult > 0) & (mult < (1 << 31)))
        assert np.all((bias >= -(1 << 31)) & (bias < (1 << 31)))
        acc = conv_full(stage, qw)
        product = (acc + bias) * mult
        assert np.max(np.abs(product)) < (1 << 62)
        branch = rounded_fixed(product)
        combined = branch if i == 1 else branch + qx
        quant = np.clip(combined, 0, 127)
        for name, values, bits in ((f"weight{i}", qw, 8), (f"bias{i}", bias, 32),
                                    (f"mult{i}", mult, 32), (f"acc{i}", acc, 32),
                                    (f"output{i}", quant, 8)):
            files[name + ".mem"] = emit(args.out, name, values, bits)
        layers.append({"conv": i, "bn_fusion_max_abs_error": fusion_error,
                       "weight_scales": sw.tolist(), "acc_min": int(acc.min()), "acc_max": int(acc.max()),
                       "relu_negative_count": int(np.count_nonzero(combined < 0)),
                       "saturated_count": int(np.count_nonzero(combined > 127)),
                       "negative_branch_rescued_by_skip": int(np.count_nonzero((branch < 0) & (combined > 0))),
                       "quant_output_sum": int(quant.sum())})
        if i == 2:
            without_skip = np.clip(branch, 0, 127)
            assert np.any(without_skip != quant), "residual path must affect this test"
            skip_changed = int(np.count_nonzero(without_skip != quant))
        stage = quant

    # Separate postprocess unit vectors: integer Python arithmetic including
    # signed extremes, half-way rounding, residual enable and saturation.
    rng = np.random.default_rng(704)
    cases = [(1, 0, 1 << 23, 0, 0), (-1, 0, 1 << 23, 2, 1),
             (-1, 0, 1 << 23, 0, 0), (0, -1, 1 << 23, 1, 1),
             (127, 0, 1 << 24, 1, 1), (0, 0, 1, 127, 0),
             ((1 << 31)-1, (1 << 31)-1, (1 << 31)-1, 127, 1),
             (-(1 << 31), -(1 << 31), (1 << 31)-1, 127, 1),
             (0, 0, 1 << 24, -128, 1), (128, 0, 1 << 24, -1, 1)]
    while len(cases) < 256:
        if len(cases) < 160:
            cases.append((int(rng.integers(-256, 257)), int(rng.integers(-128, 129)),
                          int(rng.integers(0, 1 << 24)), int(rng.integers(-128, 128)), len(cases) % 2))
        else:
            cases.append((int(rng.integers(-(1 << 31), 1 << 31)),
                          int(rng.integers(-(1 << 31), 1 << 31)),
                          int(rng.integers(0, 1 << 31)), int(rng.integers(-128, 128)), len(cases) % 2))
    packed = []
    for acc, bias, mult, skip, enabled in cases:
        product = (acc + bias) * mult
        rounded = (abs(product) + (1 << 23)) >> 24
        rounded = -rounded if product < 0 else rounded
        expected = max(0, min(127, rounded + (skip if enabled else 0)))
        fields = [(enabled, 8), (skip, 8), (mult, 32), (bias, 32), (acc, 32), (expected, 8)]
        word = 0
        for value, bits in fields:
            word = (word << bits) | (value & ((1 << bits)-1))
        packed.append(f"{word:030x}\n")
    vector_path = args.out / "post_vectors.mem"
    vector_path.write_text("".join(packed), encoding="ascii")
    files["post_vectors.mem"] = {"elements": 256, "bits": 120, "sha256": sha(vector_path)}
    metrics = {"mae": float(np.mean(np.abs(stage * s0 - yfp))),
               "max_abs_error": float(np.max(np.abs(stage * s0 - yfp))),
               "relative_l2": float(np.linalg.norm(stage * s0 - yfp) / np.linalg.norm(yfp))}
    metadata = {"model": "torchvision ResNet18 IMAGENET1K_V1", "block": "layer1.0",
                "shape_hwc": [56, 56, 64], "weights_url": WEIGHT_URL,
                "weights_sha256": sha(checkpoint), "image_url": IMAGE_URL, "image_sha256": sha(photo),
                "versions": {"python": platform.python_version(), "torch": torch.__version__,
                             "torchvision": torchvision.__version__, "numpy": np.__version__},
                "preprocessing": str(ResNet18_Weights.IMAGENET1K_V1.transforms()),
                "scale_input_output": s0, "scale_intermediate": s1, "multiplier_shift": SHIFT,
                "rounding": "nearest, ties away from zero; zero_point=0; final clamp [0,127]",
                "calibration": "one real image; functional experiment only, no dataset accuracy claim",
                "activation_layout": "HWC", "weight_layout": "ky,kx,ci,co; co fastest",
                "software_validation": "full NumPy INT64 convolutions == PyTorch exact-double convolutions; BN fusion checked; mem round-trip checked",
                "hardware_validation": "NOT RUN; user must run Vivado simulations",
                "generator_sha256": sha(__file__),
                "layers": layers, "residual_changed_output_count": skip_changed,
                "quantized_vs_float_block": metrics, "files": files}
    (args.out / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"out": str(args.out), "files": len(files), "software_checks": "OK",
                      "residual_changed_outputs": skip_changed, "quantized_vs_float": metrics}, indent=2))


if __name__ == "__main__":
    main()
