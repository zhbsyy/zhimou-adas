"""Prepare representative train-only RGB input images for RKNN calibration.

No validation/test frame is used. Generated PNGs are pre-letterboxed to the
model input dimensions, with BGR storage so RKNN's image reader yields RGB.
"""
import argparse
import csv
import hashlib
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np


MANIFEST = None
OUT = None
PER_SOURCE = 40


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def make_image(image, height, width):
    h, w = image.shape[:2]
    ratio = min(width / w, height / h)
    nw, nh = round(w * ratio), round(h * ratio)
    left = round((width - nw) / 2 - 0.1)
    top = round((height - nh) / 2 - 0.1)
    result = np.full((height, width, 3), 114, np.uint8)
    result[top:top + nh, left:left + nw] = cv2.resize(
        image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return result


def select_rows():
    groups = defaultdict(list)
    with MANIFEST.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            if row["split"] != "train":
                continue
            image = Path(row["image_path"])
            if image.is_file():
                groups[row["source"]].append(row)
    selected = []
    for source, rows in sorted(groups.items()):
        # Positive lane labels receive some priority, then stable hash order.
        rows.sort(key=lambda r: (-(int(r.get("solid_pixels") or 0) > 0) -
                                 (int(r.get("dashed_pixels") or 0) > 0),
                                 hashlib.sha256(r["sample_id"].encode()).hexdigest()))
        # Spread over the ordered pool rather than taking only nearby frames.
        idx = np.unique(np.linspace(0, len(rows) - 1,
                                    min(PER_SOURCE, len(rows)), dtype=int))
        selected.extend(rows[int(i)] for i in idx)
    return selected


def main():
    global MANIFEST, OUT
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    MANIFEST = args.manifest.resolve()
    OUT = args.output_dir.resolve()
    if not MANIFEST.is_file():
        raise FileNotFoundError(MANIFEST)
    if OUT.exists():
        raise FileExistsError(f"Calibration version already exists: {OUT}")
    rows = select_rows()
    if len(rows) < 100:
        raise RuntimeError(f"Too few train images for calibration: {len(rows)}")
    (OUT / "input_256x448").mkdir(parents=True)
    records = []
    counts = defaultdict(int)
    for row in rows:
        image_path = Path(row["image_path"])
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Cannot decode {image_path}")
        actual_hash = sha256(image_path)
        if row.get("image_sha256") and actual_hash.lower() != row["image_sha256"].lower():
            raise ValueError(f"Source hash mismatch: {image_path}")
        source = row["source"]
        number = counts[source]
        counts[source] += 1
        stem = hashlib.sha256((source + ":" + row["sample_id"]).encode()).hexdigest()[:16]
        outputs = {}
        for height, width in ((256, 448),):
            target = OUT / f"input_{height}x{width}" / f"{stem}.png"
            if target.exists():
                raise ValueError(f"Duplicate output ID: {target}")
            if not cv2.imwrite(str(target), make_image(image, height, width)):
                raise OSError(f"Cannot write {target}")
            outputs[f"input_{height}x{width}"] = str(target.relative_to(OUT))
            outputs[f"sha256_{height}x{width}"] = sha256(target)
        records.append({"sample_id": row["sample_id"], "source": source,
                        "split": row["split"], "source_image": str(image_path),
                        "source_sha256": actual_hash,
                        "solid_pixels": row.get("solid_pixels", ""),
                        "dashed_pixels": row.get("dashed_pixels", ""), **outputs})
    columns = list(records[0])
    with (OUT / "manifest.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(records)
    print("calibration_train_images", len(records), dict(counts))
    print("output", OUT)


if __name__ == "__main__":
    main()
