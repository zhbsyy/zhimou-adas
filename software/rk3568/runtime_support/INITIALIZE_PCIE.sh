#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
[[ $EUID == 0 ]] || { printf '执行 sudo bash ./INITIALIZE_PCIE.sh\n' >&2; exit 2; }
python3 -B ./launch_decision.py --check-idle
[[ "$(uname -r)" == 6.1.99 ]] || { printf '随包驱动适配 6.1.99；当前内核不同，请按环境文档核对，未操作 PCIe。\n' >&2; exit 2; }
DEV=/sys/bus/pci/devices/0002:21:00.0
KO="$PWD/System_Driver/Prebuilt_Kernel_6.1.99/pango_pci_driver.ko"
[[ -s "$KO" ]] || { printf '随包驱动缺失，停止。\n' >&2; exit 2; }
if [[ -d "$DEV" ]]; then printf '1\n' > "$DEV/remove"; fi
sleep 1
printf '1\n' > /sys/bus/pci/rescan
sleep 2
if [[ -d /sys/module/pango_pci_driver ]]; then rmmod pango_pci_driver; fi
insmod "$KO"
setpci -s 0002:21:00.0 command=0x0006
lspci -vv -s 0002:21:00.0 2>/dev/null | head -n 9 || true
python3 -B ./launch_decision.py --check-pcie
