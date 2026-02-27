from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision.cameras.detector import detect_cameras
from vision.cameras.network import get_network_cameras
from vision.cameras.unified_registry import register_camera


def main() -> int:
    print("=== Migracion a unified registry ===")
    usb = detect_cameras()
    net = get_network_cameras()
    total = 0
    for cam in usb:
        idx = cam.get("index", 0)
        register_camera(
            type="usb",
            url=f"cv2://{idx}",
            label=cam.get("model", f"USB {idx}"),
            priority=1,
            connection_method="direct",
            metadata=cam,
        )
        total += 1
    for cam in net:
        register_camera(
            type=cam.get("type", "network"),
            url=cam.get("url", ""),
            label=cam.get("model", cam.get("ip", "Network Camera")),
            priority=2,
            connection_method=cam.get("connection_method", cam.get("source", "network")),
            metadata=cam,
        )
        total += 1
    print(f"Migradas: {total}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
