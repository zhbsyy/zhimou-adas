"""Experimental ONNX graph rewrite for the RK3568 GCN Conv submit failure.

This creates a new ONNX file and checks its numerical agreement with the
original. It never edits the training checkpoint or the existing ONNX files.
Reconversion and actual RK3568 inference are still required.
"""
import argparse
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
import onnxruntime as ort


TARGET = "/caam/gcn/conv1/Conv"


def patch(source: Path, destination: Path):
    model = onnx.load(str(source))
    nodes = list(model.graph.node)
    matches = [(index, node) for index, node in enumerate(nodes) if node.name == TARGET]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one {TARGET}, found {len(matches)}")
    index, node = matches[0]
    attrs = {a.name: helper.get_attribute_value(a) for a in node.attribute}
    if node.op_type != "Conv" or attrs.get("group") != 1 or attrs.get("kernel_shape") != [1, 1]:
        raise ValueError("Only group=1 1x1 Conv is supported")
    weights = {w.name: numpy_helper.to_array(w) for w in model.graph.initializer}
    if node.input[1] not in weights or len(node.input) != 2:
        raise ValueError("Unexpected GCN Conv weights or bias")
    w = weights[node.input[1]]
    if w.ndim != 4 or w.shape[2:] != (1, 1):
        raise ValueError(f"Unexpected GCN Conv weight shape: {w.shape}")
    # Conv weight [out,in,1,1] -> MatMul weight [in,out].
    new_weight_name = "caam.gcn.conv1.matmul_weight"
    model.graph.initializer.append(numpy_helper.from_array(
        np.ascontiguousarray(w[:, :, 0, 0].T), new_weight_name))
    inp = node.input[0]
    out = node.output[0]
    replacement = [
        helper.make_node("Transpose", [inp], [out + "_nhwc"],
                         name="/caam/gcn/conv1/pre_transpose", perm=[0, 2, 3, 1]),
        helper.make_node("MatMul", [out + "_nhwc", new_weight_name],
                         [out + "_matmul"], name="/caam/gcn/conv1/MatMul"),
        helper.make_node("Transpose", [out + "_matmul"], [out],
                         name="/caam/gcn/conv1/post_transpose", perm=[0, 3, 1, 2]),
    ]
    del model.graph.node[:]
    model.graph.node.extend(nodes[:index] + replacement + nodes[index + 1:])
    onnx.checker.check_model(model)
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(destination))


def verify(source: Path, destination: Path, height: int, width: int):
    original = ort.InferenceSession(str(source), providers=["CPUExecutionProvider"])
    patched = ort.InferenceSession(str(destination), providers=["CPUExecutionProvider"])
    inputs = [np.random.default_rng(15).random((1, 3, height, width),
                                               dtype=np.float32),
              np.full((1, 3, height, width), 114 / 255, dtype=np.float32)]
    for i, x in enumerate(inputs):
        a = original.run(None, {original.get_inputs()[0].name: x})
        b = patched.run(None, {patched.get_inputs()[0].name: x})
        for task, (y1, y2) in enumerate(zip(a, b)):
            diff = float(np.max(np.abs(y1 - y2)))
            agree = float(np.mean(y1.argmax(1) == y2.argmax(1)))
            print(f"sample={i} task={task} max_abs_logit_diff={diff:.8f} "
                  f"pixel_class_agreement={agree:.8f}")
            if diff > 1e-3 or agree < 0.99999:
                raise RuntimeError("Patched ONNX is not numerically equivalent")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--destination", type=Path, required=True)
    p.add_argument("--height", type=int, required=True)
    p.add_argument("--width", type=int, default=640)
    args = p.parse_args()
    patch(args.source, args.destination)
    verify(args.source, args.destination, args.height, args.width)
    print(f"VERIFIED {args.destination}")


if __name__ == "__main__":
    main()
