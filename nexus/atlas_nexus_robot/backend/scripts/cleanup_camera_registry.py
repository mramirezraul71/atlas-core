from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision.cameras.unified_registry import deduplicate_cameras, get_active_camera_by_priority


def main() -> int:
    out = deduplicate_cameras()
    active = get_active_camera_by_priority(prefer_online=False)
    print(f"cleanup_ok={out.get('ok')} removed={out.get('removed')} total={out.get('total')}")
    if active:
        print(f"active={active.get('label')} url={active.get('url')} priority={active.get('priority')}")
    else:
        print("active=None")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
