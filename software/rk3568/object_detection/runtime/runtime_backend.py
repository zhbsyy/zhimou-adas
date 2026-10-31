"""Float32 C API: independent copies or explicitly reused NumPy outputs.

This is not zero-copy. Buffered mode uses outputs_get(is_prealloc=1) and
arrays remain valid only until that context's next inference. The existing
pipeline has one worker per context and finishes postprocess before reuse.
No runtime output arrays enter STATE; it stores decoded boxes/uint8 masks.
"""
import ctypes as C
import hashlib
from pathlib import Path
import platform
import threading
import time

import numpy as np

from native_api import Input, Native, Output


EXPECTED = {
    "A_rk3568_int8.rknn": "65111dd43d15d289d6001657832b77bb4e68a1afa277684564517eef8c4d9543",
    "R15_256x448_patched_native_int8.rknn": "65a233fef461d89f1413bddc9d0316e24aaa310512caac686d88bab388c8c289",
}


def check_model(path):
    path = Path(path)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if EXPECTED.get(path.name) != digest:
        raise RuntimeError("Unverified model for this trial: " + str(path))
    return digest


class NativeFloatRuntime:
    def __init__(self, reuse_outputs=False):
        self.path = None
        self.native = None
        self.lock = threading.Lock()
        self.last = {}
        self.reuse_outputs = reuse_outputs
        self.output_buffers = []

    def load_rknn(self, path):
        check_model(path)
        self.path = Path(path)
        return 0

    def init_runtime(self):
        if platform.system() != "Linux" or platform.machine() not in ("aarch64", "arm64"):
            raise RuntimeError("Native runtime requires aarch64 Linux")
        if self.path is None or self.native is not None:
            raise RuntimeError("Load one model before initializing one context")
        ctx = Native(self.path, collect=False)
        try:
            inp = ctx.inputs[0]
            if inp.fmt != 1 or inp.n_dims != 4 or inp.dims[0] != 1 or inp.dims[3] != 3:
                raise RuntimeError("Expected logical NHWC RGB input")
            for out in ctx.outputs:
                shape = list(out.dims[:out.n_dims])
                if out.fmt != 0 or len(shape) != 4 or min(shape) <= 0 or np.prod(shape) != out.n_elems:
                    raise RuntimeError("Expected dense logical NCHW outputs")
            self.native = ctx
            if self.reuse_outputs:
                self.output_buffers = [np.empty(tuple(a.dims[:a.n_dims]), np.float32) for a in ctx.outputs]
        except Exception:
            ctx.close()
            self.native = None
            self.output_buffers = []
            raise
        return 0

    def inference(self, inputs, data_format="nhwc"):
        if data_format != "nhwc" or len(inputs) != 1:
            raise ValueError("This trial accepts exactly one uint8 NHWC tensor")
        tensor = inputs[0]
        ctx = self.native
        if ctx is None:
            raise RuntimeError("Runtime is not initialized")
        shape = tuple(ctx.inputs[0].dims[:4])
        if tensor.dtype != np.uint8 or tensor.shape != shape or not tensor.flags.c_contiguous:
            raise ValueError("Input dtype/shape/contiguity differs from probed contract")
        with self.lock:
            inp = Input(index=0, buf=tensor.ctypes.data, size=tensor.nbytes,
                        pass_through=0, type=3, fmt=1)
            outputs = (Output * ctx.io.n_output)()
            for i, out in enumerate(outputs):
                out.index, out.want_float, out.is_prealloc = i, 1, int(self.reuse_outputs)
                if self.reuse_outputs:
                    out.buf = self.output_buffers[i].ctypes.data
                    out.size = self.output_buffers[i].nbytes
            started = time.perf_counter()
            ctx.check(ctx.lib.rknn_inputs_set(ctx.ctx, 1, C.byref(inp)), "inputs_set")
            after_set = time.perf_counter()
            ctx.check(ctx.lib.rknn_run(ctx.ctx, None), "run")
            after_run = time.perf_counter()
            ctx.check(ctx.lib.rknn_outputs_get(ctx.ctx, ctx.io.n_output, outputs, None), "outputs_get")
            after_get = time.perf_counter()
            arrays = []
            try:
                for i, (out, attr) in enumerate(zip(outputs, ctx.outputs)):
                    expected_bytes = attr.n_elems * C.sizeof(C.c_float)
                    if not out.buf or out.size != expected_bytes:
                        raise RuntimeError("Unexpected float output size; no padded/native reinterpretation allowed")
                    if self.reuse_outputs:
                        if out.buf != self.output_buffers[i].ctypes.data:
                            raise RuntimeError("Runtime replaced a caller-owned output pointer")
                        arrays.append(self.output_buffers[i])
                    else:
                        borrowed = np.ctypeslib.as_array((C.c_float * attr.n_elems).from_address(out.buf))
                        arrays.append(borrowed.copy().reshape(tuple(attr.dims[:attr.n_dims])))
            finally:
                after_copy = time.perf_counter()
                ctx.check(ctx.lib.rknn_outputs_release(ctx.ctx, ctx.io.n_output, outputs), "outputs_release")
            ended = time.perf_counter()
            self.last = {
                "set_ms": (after_set-started)*1000,
                "run_ms": (after_run-after_set)*1000,
                "get_ms": (after_get-after_run)*1000,
                "copy_ms": (after_copy-after_get)*1000,
                "release_ms": (ended-after_copy)*1000,
                "owned_call_ms": (ended-started)*1000,
                "buffer_mode": "reused_until_next_call" if self.reuse_outputs else "independent_copy",
            }
            return arrays

    def release(self):
        with self.lock:
            if self.native is not None:
                self.native.close()
                self.native = None
                self.output_buffers = []


def runtime_factory(backend):
    if backend == "native_float":
        return NativeFloatRuntime
    if backend == "native_buffered":
        return lambda: NativeFloatRuntime(reuse_outputs=True)
    from rknnlite.api import RKNNLite
    return RKNNLite


def verify_outputs(models, post, lane_module, video, backend="native_float"):
    """Serial, finite preflight before capture/HTTP/OIB; both models must pass."""
    import cv2
    from rknnlite.api import RKNNLite
    cap = cv2.VideoCapture(video)
    try:
        ok, frame = cap.read() if cap.isOpened() else (False, None)
        if not ok:
            raise RuntimeError("Preflight cannot read the existing video: " + video)
    finally:
        cap.release()
    full = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
    front = cv2.resize(full, (640, 360), interpolation=cv2.INTER_AREA)
    lane = lane_module.R15Front.__new__(lane_module.R15Front)
    lane.height, lane.width, lane.input_mode = 256, 448, "uint8_nhwc"
    actual = {"det": post.preprocess_fast(full, 640)[0], "lane": lane.preprocess(front)[0]}
    rows = []
    for label, path in models.items():
        check_model(path)
        native, lite = runtime_factory(backend)(), RKNNLite()
        try:
            if native.load_rknn(path) or native.init_runtime() or lite.load_rknn(path) or lite.init_runtime():
                raise RuntimeError("Preflight runtime initialization failed")
            sample = actual[label]
            h, w = sample.shape[1:3]
            ramp = np.arange(h*w*3, dtype=np.uint32).reshape(1, h, w, 3).astype(np.uint8)
            for name, tensor in (("flat114", np.full_like(sample, 114)), ("ramp", ramp), ("video_frame", sample)):
                before = hashlib.sha256(tensor.tobytes()).hexdigest()
                a = native.inference([tensor])
                b = lite.inference(inputs=[tensor], data_format="nhwc")
                if b is None or len(a) != len(b):
                    raise RuntimeError("Preflight output count mismatch: " + label)
                equal = all(x.dtype == y.dtype and x.shape == y.shape and np.array_equal(x, y)
                            for x, y in zip(a, b))
                if hashlib.sha256(tensor.tobytes()).hexdigest() != before:
                    raise RuntimeError("Preflight runtime modified input")
                rows.append({"model": label, "input": name, "outputs": len(a), "exact_equal": equal})
                if not equal:
                    raise RuntimeError("Native/Lite float outputs differ; trial stopped: " + label + "/" + name)
        finally:
            native.release()
            lite.release()
    return rows
