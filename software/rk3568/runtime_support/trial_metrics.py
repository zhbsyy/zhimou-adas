"""Finite trial counters and sampled /proc process-tree CPU, no frequency writes."""
import json
import os
from pathlib import Path
import threading
import time


COUNTERS = ("cap_count", "det_count", "lane_count", "processed_count", "stream_count")


def process_tree(root):
    found = {}
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            raw = path.read_text()
            tail = raw[raw.rfind(")")+2:].split()
            pid = int(path.parent.name)
            found[pid] = (int(tail[1]), int(tail[19]), int(tail[11])+int(tail[12]))
        except (OSError, ValueError, IndexError):
            continue
    members = {root}
    while True:
        expanded = members | {pid for pid, v in found.items() if v[0] in members}
        if expanded == members:
            break
        members = expanded
    return {(pid, found[pid][1]): found[pid][2] for pid in members if pid in found}


class TrialMetrics:
    def __init__(self, stats, warmup=10.0):
        self.stats, self.warmup = stats, warmup
        self.started = time.monotonic()
        self.begin = None
        self.begin_counts = {}
        self.cpu_ticks = 0
        self.previous = {}
        self.samples = 0
        self.observation = 0.0
        self.last_sample_at = None
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.monitor, daemon=True)
        self.thread.start()

    def sample(self):
        now = time.monotonic()
        if now-self.started < self.warmup:
            return
        current = process_tree(os.getpid())
        if self.begin is None:
            self.begin = now
            self.begin_counts = {k: self.stats[k] for k in COUNTERS}
        else:
            self.cpu_ticks += sum(max(0, ticks-self.previous[key])
                                  for key, ticks in current.items() if key in self.previous)
            self.observation += now-self.last_sample_at
        self.previous = current
        self.last_sample_at = now
        self.samples += 1

    def monitor(self):
        while not self.stop.wait(0.5):
            with self.lock:
                self.sample()

    def finish(self, backend, roi, errors, display_mode="original", publication_mode="submit_collect_latest"):
        self.stop.set()
        self.thread.join(timeout=2)
        with self.lock:
            self.sample()
        now = time.monotonic()
        wall = now-self.begin if self.begin is not None else 0.0
        deltas = {k: self.stats[k]-self.begin_counts[k] for k in COUNTERS} if wall else {}
        rates = {k: value/wall for k, value in deltas.items()}
        cpu_s = self.cpu_ticks/os.sysconf("SC_CLK_TCK")
        summary = {
            "runtime_backend": backend, "roi_overlay": roi,
            "display_mode": display_mode,
            "publication_mode": publication_mode,
            "lane_argmax": self.stats.get("lane_argmax", "numpy"),
            "frame_age_valid": False,
            "warmup_s": self.warmup, "observation_s": wall,
            "counts": deltas, "fps": rates,
            "cpu_seconds_process_tree_sampled": cpu_s,
            "cpu_core_percent_sampled": 100*cpu_s/self.observation if self.observation else None,
            "cpu_sample_seconds": self.observation, "cpu_samples": self.samples,
            "cpu_note": "100%=one core. Includes seen descendants; first/last partial lifetimes can be missed.",
            "stage_ema_ms": {k: self.stats.get(k) for k in
                             ("det_call_ms", "lane_call_ms", "r15_pre_ms", "r15_post_ms",
                              "lane_draw_ms", "vis_draw_ms", "enc_ms", "age_ms", "inf_age_ms")},
            "errors": errors,
            "rate_gate": bool(wall >= 10 and rates.get("det_count", 0) >= 8 and
                              rates.get("lane_count", 0) >= 8 and rates.get("stream_count", 0) >= 20 and not errors),
            "note": "Rate gate only. Unique CARLA source frames, visual quality, frame age and long-run behavior require checks. Callback mode publishes each parsed hardware JPEG once; legacy mode may drain several outputs into one publication. Callback age_ms=-1 means not measured. No exact matched-frame or PC-to-board end-to-end latency here.",
        }
        print("TRIAL_SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
        return summary
