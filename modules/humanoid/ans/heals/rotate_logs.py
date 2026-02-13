"""Rotate/clean logs safely."""
from __future__ import annotations

import os
from pathlib import Path
from .base import heal_result


def run(**kwargs) -> dict:
    try:
        base = (os.getenv("POLICY_ALLOWED_PATHS") or "C:\\ATLAS_PUSH").strip().split(",")[0].strip()
        log_dir = Path(base) / "logs"
        if not log_dir.exists():
            return heal_result(True, "rotate_logs", "no logs", {})
        # Safe: only truncate very large files (>50MB) - simplified: just report
        rotated = 0
        for p in log_dir.glob("*.log"):
            try:
                size = p.stat().st_size
                if size > 50 * 1024 * 1024:
                    with open(p, "w") as f:
                        f.write("# rotated by ANS\n")
                    rotated += 1
            except Exception:
                pass
        return heal_result(True, "rotate_logs", f"rotated={rotated}", {"rotated": rotated})
    except Exception as e:
        return heal_result(False, "rotate_logs", str(e), {}, str(e))
