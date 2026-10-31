"""Read-only PCIe resource checks before the legacy /dev/mem capture reader."""
import json
from pathlib import Path


DEVICE = "/sys/bus/pci/devices/0002:21:00.0"
RECOVERY = ("Stop the capture/inference process normally, then run "
            "sudo sh /home/linaro/03_RK3568_Perception/pcie_up.sh; "
            "confirm Mem+ BusMaster+ and nonzero Region 1 before restarting.")


def require_pcie_ready(device=DEVICE):
    device = Path(device)
    try:
        rows = (device/"resource").read_text(encoding="ascii").splitlines()
        values = rows[1].split()
        if len(values) != 3:
            raise ValueError("BAR1 resource line must contain start, end and flags")
        start, end, flags = (int(v, 16) for v in values)
        if start == 0 or end < start or end-start+1 < 4096 or start % 4096:
            raise ValueError("BAR1 has no valid aligned 4 KiB memory resource (start=0x%x)" % start)
        if not flags & 0x200:  # Linux IORESOURCE_MEM
            raise ValueError("BAR1 is not a memory resource")
        with (device/"config").open("rb") as stream:
            prefix = stream.read(6)
        if len(prefix) != 6:
            raise ValueError("PCI config header is unavailable")
        command = int.from_bytes(prefix[4:6], "little")
        if command & 0x6 != 0x6:
            raise ValueError("PCI command requires both Memory and BusMaster (command=0x%x)" % command)
        return {"device": str(device), "bar1": hex(start), "memory": True,
                "bus_master": True, "check": "resource/config only; not optical-frame validation"}
    except (OSError, ValueError, IndexError) as exc:
        raise RuntimeError("PCIE_NOT_READY: "+str(exc)+". "+RECOVERY) from exc


if __name__ == "__main__":
    try:
        print("PCIE_PREFLIGHT_OK "+json.dumps(require_pcie_ready()))
    except RuntimeError as exc:
        raise SystemExit(str(exc))
