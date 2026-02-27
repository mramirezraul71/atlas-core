from __future__ import annotations

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision.cameras.auto_discovery import discover_all_cameras


async def _main() -> int:
    print("=== Auto-discovery de camaras ===")
    result = await discover_all_cameras(auto_register=True)
    print(f"USB: {result.get('usb_count', 0)}")
    print(f"Network: {result.get('network_count', 0)}")
    print(f"ONVIF: {result.get('onvif_count', 0)}")
    print(f"mDNS: {result.get('mdns_count', 0)}")
    print(f"Registradas: {result.get('registered_count', 0)}")
    errors = result.get("errors", [])
    if errors:
        print("Errores:")
        for e in errors:
            print(f"- {e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(_main()))
