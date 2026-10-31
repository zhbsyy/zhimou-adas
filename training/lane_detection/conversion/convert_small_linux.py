"""Separate RK3568 256x448 R15 trial; never overwrites the 384x640 model.

Run on x86_64 Linux with RKNN-Toolkit2; this file does not run on RK3568.
Both FP16 and INT8 builds use RGB uint8/NHWC input with RKNN std=255.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import onnx


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--onnx", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--precision", choices=("fp16", "int8"), required=True)
    p.add_argument("--calibration-dir", type=Path,
                   help="directory of pre-letterboxed PNGs; required for INT8")
    args = p.parse_args()
    source = args.onnx.resolve()
    output = args.out.resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    if output.exists():
        raise FileExistsError(output)
    model = onnx.load(str(source))
    onnx.checker.check_model(model)
    dims = model.graph.input[0].type.tensor_type.shape.dim
    input_shape = [dimension.dim_value for dimension in dims]
    if len(model.graph.input) != 1 or len(model.graph.output) != 2 or \
            len(input_shape) != 4 or input_shape[0] != 1 or \
            input_shape[1] != 3 or input_shape[2:] != [256, 448]:
        raise ValueError("Expected R15 RGB NCHW [1,3,256,448] and two outputs")
    output_shapes = [[dimension.dim_value for dimension in
                      item.type.tensor_type.shape.dim]
                     for item in model.graph.output]
    if output_shapes != [[1, 2, 256, 448], [1, 3, 256, 448]]:
        raise ValueError("Unexpected direct/lane output shapes: " + str(output_shapes))
    output.parent.mkdir(parents=True, exist_ok=True)
    dataset_file = None
    images = []
    if args.precision == "int8":
        if args.calibration_dir is None:
            p.error("INT8 requires --calibration-dir")
        images = sorted(args.calibration_dir.resolve().glob("*.png"))
        if len(images) < 100:
            raise ValueError("INT8 calibration needs at least 100 PNG images")
        for image in images:
            decoded = cv2.imread(str(image))
            if decoded is None or decoded.shape[:2] != (256, 448):
                raise ValueError("Cannot decode: " + str(image))
        dataset_file = output.with_name(output.stem + "_calibration.txt")
        if dataset_file.exists():
            raise FileExistsError(dataset_file)
        dataset_file.write_text("\n".join(str(image) for image in images) + "\n",
                                encoding="utf-8")
    from rknn.api import RKNN

    rk = RKNN(verbose=True)
    try:
        result = rk.config(target_platform="rk3568", mean_values=[[0, 0, 0]],
                           std_values=[[255, 255, 255]])
        if result != 0:
            raise RuntimeError(f"config failed: {result}")
        result = rk.load_onnx(model=str(source))
        if result != 0:
            raise RuntimeError(f"load_onnx failed: {result}")
        kwargs = {"do_quantization": args.precision == "int8"}
        if dataset_file:
            kwargs["dataset"] = str(dataset_file)
        result = rk.build(**kwargs)
        if result != 0:
            raise RuntimeError(f"build failed: {result}")
        result = rk.export_rknn(str(output))
        if result != 0:
            raise RuntimeError(f"export failed: {result}")
    finally:
        rk.release()
    record = {
        "onnx": str(source), "onnx_sha256": sha256(source),
        "rknn": str(output), "rknn_sha256": sha256(output),
        "precision": args.precision, "calibration_count": len(images),
        "dataset_file": str(dataset_file) if dataset_file else None,
        "input_contract": "RGB uint8 NHWC; model internally scales by 1/255",
        "status": "converted only; board inference and quality not yet tested",
    }
    meta = output.with_suffix(output.suffix + ".json")
    meta.write_text(json.dumps(record, indent=2), encoding="utf-8")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
