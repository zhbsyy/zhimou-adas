"""Export a separate 256x448 R15 trial; never edits the trained checkpoint.

Run on the Windows training machine.  The source model and the converter must
still be validated on RK3568 before this can replace the 384x640 model.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project", type=Path, required=True,
                        help="twinlitenet_plus_small source directory")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    project = args.project.resolve()
    checkpoint = args.checkpoint.resolve()
    output_dir = args.output_dir.resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    output_dir.mkdir(parents=True, exist_ok=True)
    original = output_dir / "R15_256x448_original.onnx"
    patched = output_dir / "R15_256x448_gcn_matmul.onnx"
    report_path = output_dir / "export_report.json"
    for path in (original, patched, report_path):
        if path.exists():
            raise FileExistsError(path)

    sys.path.insert(0, str(project / "runs" / "R13_competition_video"))
    from r13_video import load_model  # noqa: E402
    from patch_gcn_conv_to_matmul import patch  # noqa: E402

    torch.set_num_threads(4)
    model = load_model(checkpoint, torch.device("cpu"))
    sample = torch.rand(1, 3, 256, 448,
                        generator=torch.Generator().manual_seed(20261023))
    with torch.inference_mode():
        native = [item.numpy() for item in model(sample)]
        if [item.shape for item in native] != [(1, 2, 256, 448),
                                                (1, 3, 256, 448)]:
            raise RuntimeError("Unexpected R15 output dimensions")
        torch.onnx.export(model, sample, str(original), opset_version=17,
                          input_names=["rgb_float_0_1"],
                          output_names=["direct_logits", "lane_logits"],
                          dynamic_axes=None, do_constant_folding=True,
                          dynamo=False)
    onnx.checker.check_model(onnx.load(str(original)))
    patch(original, patched)

    session = ort.InferenceSession(str(patched),
                                   providers=["CPUExecutionProvider"])
    converted = session.run(None, {"rgb_float_0_1": sample.numpy()})
    parity = []
    for task, a, b in zip(("direct", "lane"), native, converted):
        max_diff = float(np.max(np.abs(a - b)))
        agreement = float(np.mean(a.argmax(1) == b.argmax(1)))
        parity.append({"task": task, "max_abs_logit_diff": max_diff,
                       "class_pixel_agreement": agreement})
        if max_diff > 1e-3 or agreement < 0.99999:
            raise RuntimeError(f"ONNX parity failed: {task}: {parity[-1]}")
    report = {
        "checkpoint": str(checkpoint), "checkpoint_sha256": sha256(checkpoint),
        "original_onnx_sha256": sha256(original),
        "patched_onnx_sha256": sha256(patched),
        "input": "RGB float NCHW [1,3,256,448], range 0..1",
        "outputs": [list(item.shape) for item in converted],
        "parity": parity,
        "status": "PyTorch/ONNX agreement only; RKNN speed and accuracy untested",
    }
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
