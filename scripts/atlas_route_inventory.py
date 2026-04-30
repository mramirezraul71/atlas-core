#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from atlas_adapter.atlas_http_api import app

OUT_DIR = ROOT / "reports"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def main() -> int:
    by_module: dict[str, int] = defaultdict(int)
    duplicates = []
    seen: set[tuple[str, tuple[str, ...]]] = set()

    for route in app.routes:
        methods = getattr(route, "methods", None)
        if not methods:
            continue
        http_methods = tuple(sorted(m for m in methods if m not in {"HEAD", "OPTIONS"}))
        if not http_methods:
            continue
        endpoint = getattr(route, "endpoint", None)
        mod = getattr(endpoint, "__module__", "unknown")
        by_module[mod] += 1
        key = (getattr(route, "path", ""), http_methods)
        if key in seen:
            duplicates.append({"path": key[0], "methods": list(http_methods), "module": mod})
        seen.add(key)

    payload = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "total_routes": sum(by_module.values()),
        "route_count_by_module": dict(sorted(by_module.items(), key=lambda kv: kv[1], reverse=True)),
        "duplicate_candidates": duplicates,
    }

    out = OUT_DIR / "route_inventory.json"
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
