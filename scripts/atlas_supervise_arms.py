from __future__ import annotations

import json
import os
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Tuple

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.atlas_arm_state import set_arm_state


BASE_DIR = PROJECT_ROOT
LOG_DIR = BASE_DIR / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
SUP_LOG = LOG_DIR / "atlas_supervisor.log"


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _log(payload: Dict[str, Any]) -> None:
    with SUP_LOG.open("a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=False) + "\n")


def _check(url: str, timeout: int = 5) -> Tuple[bool, str, int]:
    req = urllib.request.Request(url, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            code = int(resp.status)
            return (200 <= code < 300), "ok", code
    except urllib.error.HTTPError as e:
        return False, f"http_{e.code}", int(e.code)
    except Exception as e:
        return False, f"{type(e).__name__}:{e}", 0


def main() -> int:
    pan_base = (os.getenv("PANADERIA_API_BASE") or "http://127.0.0.1:3001").rstrip("/")
    vision_base = (os.getenv("RAULI_VISION_PROXY_BASE") or "http://127.0.0.1:3000").rstrip("/")
    atlas_base = (os.getenv("ATLAS_PUSH_BASE") or "http://127.0.0.1:8791").rstrip("/")

    checks = {
        "atlas": atlas_base + "/health",
        "panaderia": pan_base + "/api/health",
        "vision": vision_base + "/api/health",
    }
    out: Dict[str, Any] = {"ts": _now_iso(), "checks": {}}

    for name, url in checks.items():
        ok, reason, code = _check(url)
        out["checks"][name] = {"ok": ok, "url": url, "code": code, "reason": reason}
        if name in {"panaderia", "vision"}:
            set_arm_state(name, healthy=ok, reason=reason if not ok else "")

    out["status"] = (
        "ok"
        if all(x["ok"] for x in out["checks"].values())
        else ("warning" if out["checks"]["atlas"]["ok"] else "critical")
    )
    _log(out)
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
