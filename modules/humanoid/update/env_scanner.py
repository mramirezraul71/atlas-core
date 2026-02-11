"""Environment scanner: pip list, python version."""
from __future__ import annotations

import json
import subprocess
import sys
from typing import Any, Dict


class EnvScanner:
    """Scan current env: python version, pip list. No installs."""

    def python_version(self) -> Dict[str, Any]:
        try:
            v = sys.version_info
            return {"ok": True, "version": f"{v.major}.{v.minor}.{v.micro}", "error": None}
        except Exception as e:
            return {"ok": False, "version": "", "error": str(e)}

    def pip_list(self) -> Dict[str, Any]:
        try:
            r = subprocess.run(
                [sys.executable, "-m", "pip", "list", "--format=json"],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if r.returncode != 0:
                return {"ok": False, "packages": [], "error": r.stderr or "pip list failed"}
            raw = json.loads(r.stdout)
            packages = [{"name": x.get("name"), "version": x.get("version")} for x in raw]
            return {"ok": True, "packages": packages, "error": None}
        except Exception as e:
            return {"ok": False, "packages": [], "error": str(e)}
