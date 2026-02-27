"""Regenerate support bundle."""
from __future__ import annotations

import os
from pathlib import Path
from .base import heal_result


def run(**kwargs) -> dict:
    try:
        base = (os.getenv("POLICY_ALLOWED_PATHS") or "C:\\ATLAS_PUSH").strip().split(",")[0].strip()
        import zipfile
        import json
        from datetime import datetime, timezone
        support_dir = Path(base) / "snapshots" / "support"
        support_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        zip_path = support_dir / f"ans_bundle_{ts}.zip"
        with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
            zf.writestr("ans_report.json", json.dumps({"timestamp": ts, "source": "ans_heal"}))
        return heal_result(True, "regenerate_support_bundle", "created", {"path": str(zip_path)})
    except Exception as e:
        return heal_result(False, "regenerate_support_bundle", str(e), {}, str(e))
