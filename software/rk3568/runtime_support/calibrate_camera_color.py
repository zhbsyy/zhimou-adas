"""One user-declared neutral-reference sample from the existing capture process."""
import argparse
import base64
from datetime import datetime, timezone
import json
from pathlib import Path
import urllib.request

import cv2
import numpy as np

from camera_input_color import estimate_neutral_gains


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--url", default="http://127.0.0.1:18088")
    p.add_argument("--roi", nargs=4, type=float, default=[.30, .30, .70, .70])
    p.add_argument("--confirm-neutral", action="store_true",
                   help="declare the ROI is filled with a plain neutral sheet, away from clipped light")
    p.add_argument("--output", default="camera_color_profile.local.json")
    args = p.parse_args()
    if not args.confirm_neutral:
        p.error("Fill the central 40% rectangle with a plain neutral sheet, then pass --confirm-neutral")
    try:
        with urllib.request.urlopen(args.url.rstrip("/")+"/camera_debug?image=1", timeout=5) as r:
            payload = r.read(4*1024*1024+1)
        if len(payload) > 4*1024*1024:
            raise ValueError("Camera reply exceeds bound")
        data = json.loads(payload)
        if (data.get("schema") != "rk_camera_debug.v1" or data.get("source") != "pcie" or
                data.get("evidence_mode") != "live" or
                data.get("stage") != "before_input_gains_and_all_overlays"):
            raise ValueError("Expected live PCIe pixels before any gain correction")
        image = cv2.imdecode(np.frombuffer(base64.b64decode(data["raw_png_base64"], validate=True), np.uint8),
                             cv2.IMREAD_COLOR)
        if image is None or list(image.shape) != data["shape"]:
            raise ValueError("Invalid camera PNG/shape")
        profile = estimate_neutral_gains(image, args.roi)
        profile["reference"].update(source_epoch=data["source_epoch"], frame_seq=data["frame_seq"],
                                     recorded_utc=datetime.now(timezone.utc).isoformat())
        out = Path(args.output).resolve()
        if out.suffix != ".json" or not out.parent.is_dir():
            raise ValueError("Use a JSON output in an existing directory")
        # Preserve every previous profile; no history or running source is deleted.
        if out.exists():
            backup = out.with_name(out.stem+".before_"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")+".json")
            backup.write_bytes(out.read_bytes())
            print("PROFILE_BACKUP", backup)
        temporary = out.with_suffix(".json.pending")
        temporary.write_text(json.dumps(profile, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        temporary.replace(out)
        print("CAMERA_COLOR_PROFILE "+json.dumps({"path": str(out), "gains_rgb": profile["gains_rgb"],
                                                 "reference": profile["reference"]}, ensure_ascii=False))
        print("Profile saved. Restart this candidate to apply; verify white, actual red and actual green.")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        p.exit(2, "Calibration rejected: "+str(exc)+"\n")


if __name__ == "__main__":
    main()
