"""Convert the verified 11-class nine-output ONNX to an RK3568 INT8 trial.

Run on x86_64 Linux with RKNN-Toolkit2. No historical model is overwritten.
Board input contract: RGB uint8 NHWC; RKNN config applies division by 255.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import onnx


EXPECTED_CLASSES = [
    "person", "rider", "car", "truck", "bus", "train", "motor", "bike",
    "traffic light", "traffic sign", "crosswalk",
]
EXPECTED_OUTPUTS = [
    (1, channels, 640 // stride, 640 // stride)
    for stride in (8, 16, 32) for channels in (64, 11, 1)
]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def dimensions(value):
    return tuple(dim.dim_value for dim in value.type.tensor_type.shape.dim)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifacts", type=Path,
                        default=Path(__file__).resolve().parent / "artifacts")
    parser.add_argument("--calibration-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    root = args.artifacts.resolve()
    manifest = json.loads((root / "deployment_manifest.json").read_text(encoding="utf-8"))
    report = json.loads((root / "verification.json").read_text(encoding="utf-8"))
    if not report.get("passed"):
        raise RuntimeError("Nine-output ONNX numerical verification did not pass")
    if manifest["classes"] != EXPECTED_CLASSES:
        raise ValueError("Class order differs from classes_11.txt")
    source = root / manifest["onnx_file"]
    if source.name != "A_original_rkhead9_640.onnx":
        raise ValueError("Unexpected ONNX filename")
    if sha256(source) != manifest["onnx_sha256"]:
        raise ValueError("ONNX SHA256 differs from verified manifest")
    model = onnx.load(str(source))
    onnx.checker.check_model(model)
    if len(model.graph.input) != 1 or dimensions(model.graph.input[0]) != (1, 3, 640, 640):
        raise ValueError("Expected static RGB NCHW 640 ONNX input")
    if [dimensions(x) for x in model.graph.output] != EXPECTED_OUTPUTS:
        raise ValueError("Expected 9 outputs with 11-class branch per scale")

    images = sorted(args.calibration_dir.resolve().glob("*.png"))
    if len(images) < 100:
        raise ValueError("At least 100 calibrated-size PNGs are required")
    for image in images:
        decoded = cv2.imread(str(image))
        if decoded is None or decoded.shape[:2] != (640, 640):
            raise ValueError("Calibration image missing, undecodable or wrong size: " + str(image))
    output = args.out.resolve()
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    dataset_file = output.with_name(output.stem + "_calibration.txt")
    if dataset_file.exists():
        raise FileExistsError(dataset_file)
    dataset_file.write_text("\n".join(str(image) for image in images) + "\n",
                            encoding="utf-8")

    from rknn.api import RKNN
    rk = RKNN(verbose=True)
    try:
        def check(operation, result):
            if result != 0:
                raise RuntimeError(f"{operation} failed: {result}")

        check("config", rk.config(target_platform="rk3568", mean_values=[[0, 0, 0]],
                                  std_values=[[255, 255, 255]]))
        check("load_onnx", rk.load_onnx(model=str(source)))
        check("build", rk.build(do_quantization=True, dataset=str(dataset_file)))
        check("export_rknn", rk.export_rknn(str(output)))
    finally:
        rk.release()
    metadata = {
        "source": str(source), "source_sha256": sha256(source),
        "output": str(output), "output_sha256": sha256(output),
        "precision": "int8", "calibration_count": len(images),
        "calibration_file": str(dataset_file),
        "input_contract": "RGB uint8 NHWC; RKNN applies /255",
        "status": "converted only; board FPS and class-wise accuracy not verified",
    }
    output.with_suffix(output.suffix + ".json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
