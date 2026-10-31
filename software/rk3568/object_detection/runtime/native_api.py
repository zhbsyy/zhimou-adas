"""Bounded RKNN C-API measurements, ABI pinned to RKNN-Toolkit2 v2.3.0.

No async flags, cache bypass flags, external memory or core-mask changes.
Only supports the 64-bit ABI already used by this project's board query.
"""
import ctypes as C
import time

import numpy as np


class IO(C.Structure):
    _fields_ = [("n_input", C.c_uint32), ("n_output", C.c_uint32)]


class Attr(C.Structure):
    _fields_ = [("index", C.c_uint32), ("n_dims", C.c_uint32),
                ("dims", C.c_uint32 * 16), ("name", C.c_char * 256),
                ("n_elems", C.c_uint32), ("size", C.c_uint32),
                ("fmt", C.c_int), ("type", C.c_int), ("qnt_type", C.c_int),
                ("fl", C.c_int8), ("zp", C.c_int32), ("scale", C.c_float),
                ("w_stride", C.c_uint32), ("size_with_stride", C.c_uint32),
                ("pass_through", C.c_uint8), ("h_stride", C.c_uint32)]


class Input(C.Structure):
    _fields_ = [("index", C.c_uint32), ("buf", C.c_void_p),
                ("size", C.c_uint32), ("pass_through", C.c_uint8),
                ("type", C.c_int), ("fmt", C.c_int)]


class Output(C.Structure):
    _fields_ = [("want_float", C.c_uint8), ("is_prealloc", C.c_uint8),
                ("index", C.c_uint32), ("buf", C.c_void_p), ("size", C.c_uint32)]


class Perf(C.Structure):
    _fields_ = [("run_duration", C.c_int64)]


class Detail(C.Structure):
    _fields_ = [("perf_data", C.c_void_p), ("data_len", C.c_uint64)]


class SDK(C.Structure):
    _fields_ = [("api_version", C.c_char * 256), ("drv_version", C.c_char * 256)]


class Memory(C.Structure):
    _fields_ = [("weight", C.c_uint32), ("internal", C.c_uint32),
                ("dma_allocated", C.c_uint64), ("sram", C.c_uint32),
                ("free_sram", C.c_uint32), ("reserved", C.c_uint32 * 10)]


TYPES = {0: "float32", 1: "float16", 2: "int8", 3: "uint8", 4: "int16",
         5: "uint16", 6: "int32", 7: "uint32", 8: "int64", 9: "bool",
         10: "int4", 11: "bfloat16"}
FORMATS = {0: "NCHW", 1: "NHWC", 2: "NC1HWC2", 3: "UNDEFINED"}


def attr_dict(a):
    return {"index": a.index, "name": a.name.decode(errors="replace"),
            "dims": list(a.dims[:min(a.n_dims, 16)]), "n_elems": a.n_elems,
            "size": a.size, "layout": FORMATS.get(a.fmt, str(a.fmt)),
            "type": TYPES.get(a.type, str(a.type)), "type_enum": a.type,
            "qnt_type": a.qnt_type, "zp": a.zp, "scale": float(a.scale),
            "fl": a.fl, "w_stride": a.w_stride,
            "size_with_stride": a.size_with_stride}


class Native:
    def __init__(self, model, lib_path="/usr/lib/librknnrt.so", collect=False):
        if C.sizeof(C.c_void_p) != 8:
            raise RuntimeError("This diagnostic requires 64-bit Python")
        assert (C.sizeof(Attr), C.sizeof(Input), C.sizeof(Output), C.sizeof(Memory)) == (376, 32, 24, 64)
        self.lib = C.CDLL(lib_path)
        specs = {
            "rknn_init": [C.POINTER(C.c_uint64), C.c_void_p, C.c_uint32, C.c_uint32, C.c_void_p],
            "rknn_query": [C.c_uint64, C.c_int, C.c_void_p, C.c_uint32],
            "rknn_inputs_set": [C.c_uint64, C.c_uint32, C.POINTER(Input)],
            "rknn_run": [C.c_uint64, C.c_void_p],
            "rknn_outputs_get": [C.c_uint64, C.c_uint32, C.POINTER(Output), C.c_void_p],
            "rknn_outputs_release": [C.c_uint64, C.c_uint32, C.POINTER(Output)],
            "rknn_destroy": [C.c_uint64],
        }
        for name, args in specs.items():
            fn = getattr(self.lib, name)
            fn.argtypes, fn.restype = args, C.c_int
        self.ctx = C.c_uint64()
        self.closed = True
        self.collect = collect
        blob = model.read_bytes()
        self.model_buffer = C.create_string_buffer(blob)
        self.check(self.lib.rknn_init(C.byref(self.ctx), self.model_buffer, len(blob), 8 if collect else 0, None), "init")
        self.closed = False
        try:
            self.io = IO()
            self.check(self.query(0, self.io), "io query")
            if self.io.n_input != 1 or not 0 < self.io.n_output <= 32:
                raise RuntimeError("Unsupported input/output count")
            self.inputs, self.outputs = [], []
            for cmd, count, dest in ((1, self.io.n_input, self.inputs), (2, self.io.n_output, self.outputs)):
                for i in range(count):
                    a = Attr(); a.index = i
                    self.check(self.query(cmd, a), "tensor query")
                    dest.append(a)
        except Exception:
            self.close()
            raise

    @staticmethod
    def check(rc, stage):
        if rc != 0:
            raise RuntimeError("RKNN %s rc=%d" % (stage, rc))

    def query(self, cmd, value):
        return self.lib.rknn_query(self.ctx, cmd, C.byref(value), C.sizeof(value))

    def metadata(self):
        sdk = SDK(); sdk_rc = self.query(5, sdk)
        mem = Memory(); mem_rc = self.query(6, mem)
        extra = {}
        for cmd, label, count in ((8, "native_inputs", self.io.n_input),
                                   (9, "native_outputs", self.io.n_output),
                                   (10, "native_nhwc_inputs", self.io.n_input),
                                   (11, "native_nhwc_outputs", self.io.n_output)):
            extra[label] = []
            for i in range(count):
                a = Attr(); a.index = i; rc = self.query(cmd, a)
                extra[label].append({"rc": rc, **(attr_dict(a) if rc == 0 else {"index": i})})
        return {"inputs": [attr_dict(a) for a in self.inputs],
                "outputs": [attr_dict(a) for a in self.outputs], **extra,
                "sdk_rc": sdk_rc, "sdk": sdk.api_version.decode(errors="replace"),
                "driver": sdk.drv_version.decode(errors="replace"),
                "memory_rc": mem_rc,
                "memory": {k: getattr(mem, k) for k in ("weight", "internal", "dma_allocated", "sram", "free_sram")} if mem_rc == 0 else None,
                "collect_perf": self.collect}

    def call(self, tensor, want_float=True, copy_outputs=False):
        if tensor.dtype != np.uint8 or not tensor.flags.c_contiguous or tensor.ndim != 4 or tensor.shape[0] != 1 or tensor.shape[-1] != 3:
            raise ValueError("Expected contiguous uint8 NHWC RGB")
        if tensor.size != self.inputs[0].n_elems:
            raise ValueError("Input element count differs from model")
        inp = Input(index=0, buf=tensor.ctypes.data, size=tensor.nbytes,
                    pass_through=0, type=3, fmt=1)
        out = (Output * self.io.n_output)()
        for i in range(self.io.n_output):
            out[i].index = i; out[i].want_float = int(want_float)
        started = time.perf_counter()
        self.check(self.lib.rknn_inputs_set(self.ctx, 1, C.byref(inp)), "inputs_set")
        after_set = time.perf_counter()
        self.check(self.lib.rknn_run(self.ctx, None), "run")
        after_run = time.perf_counter()
        self.check(self.lib.rknn_outputs_get(self.ctx, self.io.n_output, out, None), "outputs_get")
        after_get = time.perf_counter()
        try:
            perf = Perf(); perf_rc = self.query(4, perf)
            arrays, sizes = [], [o.size for o in out]
            if copy_outputs:
                for o, a in zip(out, self.outputs):
                    dtype = np.dtype("float32" if want_float else TYPES.get(a.type, "uint8"))
                    if o.size != a.n_elems * dtype.itemsize:
                        raise ValueError("Output has padding/unsupported representation; comparison unavailable")
                    data = np.frombuffer(C.string_at(o.buf, o.size), dtype=dtype).copy()
                    arrays.append(data.reshape(list(a.dims[:a.n_dims])))
        finally:
            before_release = time.perf_counter()
            self.check(self.lib.rknn_outputs_release(self.ctx, self.io.n_output, out), "outputs_release")
            after_release = time.perf_counter()
        return {"inputs_set_ms": (after_set-started)*1000,
                "run_call_ms": (after_run-after_set)*1000,
                "outputs_get_ms": (after_get-after_run)*1000,
                "outputs_release_ms": (after_release-before_release)*1000,
                "api_sum_ms": (after_get-started+after_release-before_release)*1000,
                "runtime_reported_ms": perf.run_duration/1000 if perf_rc == 0 else None,
                "perf_query_rc": perf_rc, "output_bytes": sizes}, arrays

    def detail(self):
        detail = Detail(); rc = self.query(3, detail)
        if rc != 0 or not detail.perf_data or not detail.data_len:
            return rc, ""
        if detail.data_len > 8*1024*1024:
            raise RuntimeError("Unexpectedly large performance report")
        return rc, C.string_at(detail.perf_data, detail.data_len).decode(errors="replace").rstrip("\0")

    def close(self):
        if not self.closed:
            self.lib.rknn_destroy(self.ctx)
            self.closed = True

