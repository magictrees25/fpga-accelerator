"""Offline memory/hash/layout audit only; never invokes an HDL tool.

Checks all exported words, 784 software tile mappings and global scatter,
and the scalar fixed-point postprocess contract. No PyTorch dependency.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def read_signed(path, bits):
    values = np.array([int(x, 16) for x in path.read_text().split()], dtype=np.int64)
    return np.where(values & (1 << (bits-1)), values - (1 << bits), values)


def post_scalar(acc, bias, mult, skip, enabled):
    product = (int(acc) + int(bias)) * int(mult)
    magnitude = (abs(product) + (1 << 23)) // (1 << 24)
    rounded = -magnitude if product < 0 else magnitude
    return min(127, max(0, rounded + (int(skip) if enabled else 0)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path(__file__).parent / "p7a_layer1_0")
    args = parser.parse_args()
    manifest = json.loads((args.data / "manifest.json").read_text(encoding="utf-8"))
    for name, info in manifest["files"].items():
        path = args.data / name
        assert hashlib.sha256(path.read_bytes()).hexdigest() == info["sha256"], name
        words = path.read_text().split()
        assert len(words) == info["elements"], name
        assert all(len(w) == info["bits"]//4 and 0 <= int(w, 16) < 2**info["bits"] for w in words), name
    load = lambda name, bits: read_signed(args.data / (name+".mem"), bits)
    original = load("input", 8).reshape(56, 56, 64)
    actual = original.copy()
    cached = None
    changed = 0
    pad_loads = 0
    pad_k = 0
    checked_acc = 0
    for layer in (1, 2):
        weights = load(f"weight{layer}", 8).reshape(576, 64)
        bias, mult = load(f"bias{layer}", 32), load(f"mult{layer}", 32)
        gold_acc = load(f"acc{layer}", 32).reshape(56, 56, 64)
        gold_out = load(f"output{layer}", 8).reshape(56, 56, 64)
        assembled = np.zeros_like(actual)
        seen = np.zeros_like(actual, dtype=bool)
        padded = np.pad(actual, ((1, 1), (1, 1), (0, 0)))
        valid_pad = np.pad(np.ones((56, 56), dtype=bool), ((1, 1), (1, 1)))
        for oy in range(0, 56, 8):
            for ox in range(0, 56, 8):
                # Byte-addressed 10x10 HWC preload, then local 3x3 windows.
                window = padded[oy:oy+10, ox:ox+10].copy()
                flat_a = np.stack([window[y:y+3, x:x+3].reshape(-1)
                                   for y in range(8) for x in range(8)])
                pad_loads += 8 * 64 * np.count_nonzero(~valid_pad[oy:oy+10, ox:ox+10])
                for y in range(8):
                    for x in range(8):
                        pad_k += 8 * 64 * np.count_nonzero(~valid_pad[oy+y:oy+y+3, ox+x:ox+x+3])
                for n in range(0, 64, 8):
                    # Encode/decode exactly 8 signed bytes per 64-bit host word.
                    packed_words = [sum((int(weights[k, n+lane]) & 255) << (8*lane)
                                        for lane in range(8)) for k in range(576)]
                    decoded = np.array([[(v >> (8*lane)) & 255 for lane in range(8)]
                                        for v in packed_words], dtype=np.int64)
                    decoded = np.where(decoded >= 128, decoded-256, decoded)
                    acc = flat_a @ decoded
                    expected = gold_acc[oy:oy+8, ox:ox+8, n:n+8].reshape(64, 8)
                    assert np.array_equal(acc, expected), (layer, oy, ox, n, "INT32")
                    checked_acc += acc.size
                    result = np.empty((64, 8), dtype=np.int64)
                    for m in range(64):
                        for lane in range(8):
                            y, x, c = oy+m//8, ox+m%8, n+lane
                            result[m, lane] = post_scalar(acc[m, lane], bias[c], mult[c], original[y, x, c], layer == 2)
                    assert np.array_equal(result.reshape(8, 8, 8), gold_out[oy:oy+8, ox:ox+8, n:n+8])
                    assert not seen[oy:oy+8, ox:ox+8, n:n+8].any()
                    seen[oy:oy+8, ox:ox+8, n:n+8] = True
                    assembled[oy:oy+8, ox:ox+8, n:n+8] = result.reshape(8, 8, 8)
                    if cached is not None:
                        changed += np.count_nonzero(cached != result)
                    cached = result.copy()
        assert seen.all() and np.array_equal(assembled, gold_out)
        actual = assembled
    for line in (args.data / "post_vectors.mem").read_text().split():
        word = int(line, 16)
        sign = lambda v, bits: v-(1 << bits) if v & (1 << (bits-1)) else v
        acc = sign((word >> 8) & 0xffffffff, 32)
        bias = sign((word >> 40) & 0xffffffff, 32)
        mult = sign((word >> 72) & 0xffffffff, 32)
        skip = sign((word >> 104) & 255, 8)
        assert post_scalar(acc, bias, mult, skip, (word >> 112) & 1) == (word & 255)
    print(json.dumps({"software_only": True, "hardware_simulation": "NOT RUN",
                      "tiles": 784, "int32_checked": checked_acc, "final_int8_checked": actual.size,
                      "changed_cache_lanes": int(changed), "padding_load_bytes": int(pad_loads),
                      "padding_k_elements": int(pad_k), "mem_hashes_checked": len(manifest["files"]),
                      "post_vectors_checked": 256}, indent=2))


if __name__ == "__main__":
    main()
