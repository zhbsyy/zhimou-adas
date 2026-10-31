"""USB port/control mapping and fixed tile geometry. Color lives in color_profiles.py."""
from __future__ import annotations

import argparse
from pathlib import Path


TILE_WIDTH, TILE_HEIGHT = 640, 360
FRONT_CROP = (0, 0, 640, 360)
REAR_CROP = (0, 360, 640, 360)
# These are controller/physical-port identifiers, not /dev/video numbers.
# Their association with LEFT/RIGHT was observed in v4l2-ctl --list-devices;
# confirm by covering each lens after any wiring change.
USB_PORT_MARKERS = {"right": "xhci-hcd.1.auto", "left": "xhci-hcd.0.auto"}
USB_CONTROLS = {
    "right": {"exposure_dynamic_framerate": 0, "power_line_frequency": 1},
    "left": {
        "exposure_dynamic_framerate": 0,
        "power_line_frequency": 1,
        "auto_exposure": 3,
        "gain": 0,
        "white_balance_automatic": 1,
    },
}
LEFT_REQUIRED_CONTROLS = ("auto_exposure", "gain")


def resolve_usb_devices(*, names=("right", "left"), overrides=None,
                        by_path_dir="/dev/v4l/by-path") -> dict[str, str]:
    """Return stable port symlinks; fail if a direction cannot be identified.

    A /dev/videoN override is allowed for diagnosis but is not persistent
    across node renumbering.  Never silently fall back to video0/video2.
    """
    overrides = overrides or {}
    root = Path(by_path_dir)
    links = sorted(p for p in root.glob("*video-index0") if p.is_symlink())
    found = {}
    for name in names:
        if name not in USB_PORT_MARKERS:
            raise ValueError(f"unknown camera: {name}")
        override = overrides.get(name)
        if override:
            path = Path(override)
            if not path.exists():
                raise RuntimeError(f"{name}: explicit device does not exist: {path}")
            found[name] = str(path)
            continue
        matches = [p for p in links if USB_PORT_MARKERS[name] in p.name and p.exists()]
        if len(matches) != 1:
            detail = ", ".join(p.name for p in links) or "none"
            raise RuntimeError(
                f"{name}: expected one capture device on {USB_PORT_MARKERS[name]}, "
                f"found {len(matches)}; by-path index0 links: {detail}. "
                "Check cabling, then specify USB_RIGHT/USB_LEFT explicitly if needed."
            )
        found[name] = str(matches[0])
    if len({str(Path(v).resolve()) for v in found.values()}) != len(found):
        raise RuntimeError("LEFT and RIGHT resolved to the same video device")
    return found


def open_usb_camera(name: str, *, device: str | None = None,
                    width: int = 320, height: int = 240, fps: int = 30):
    """Create a configured source. Caller must start(), read_newer(), close()."""
    from usb_source import UsbCameraSource

    path = resolve_usb_devices(names=(name,), overrides={name: device} if device else {})[name]
    return UsbCameraSource(
        path, width, height, fps, label=name,
        ctrls=USB_CONTROLS[name],
        required_ctrls=LEFT_REQUIRED_CONTROLS if name == "left" else (),
    )


def _main() -> None:
    parser = argparse.ArgumentParser(description="Resolve physical USB cameras by port")
    parser.add_argument("--right", default="")
    parser.add_argument("--left", default="")
    args = parser.parse_args()
    found = resolve_usb_devices(overrides={"right": args.right, "left": args.left})
    print(found["right"])
    print(found["left"])


if __name__ == "__main__":
    _main()
