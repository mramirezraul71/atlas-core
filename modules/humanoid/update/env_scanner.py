"""Environment scanner: python version, pip version/list, where python, venv detection."""
from __future__ import annotations

import json
import subprocess
import sys
from typing import Any, Dict


class EnvScanner:
    """Scan current env: python version, pip version/list, where python, venv. No installs."""

    def python_version(self) -> Dict[str, Any]:
        try:
            v = sys.version_info
            return {"ok": True, "version": f"{v.major}.{v.minor}.{v.micro}", "error": None}
        except Exception as e:
            return {"ok": False, "version": "", "error": str(e)}

    def pip_version(self) -> Dict[str, Any]:
        try:
            r = subprocess.run(
                [sys.executable, "-m", "pip", "--version"],
                capture_output=True,
                text=True,
                timeout=10,
            )
            out = (r.stdout or r.stderr or "").strip()
            return {"ok": r.returncode == 0, "version_output": out, "error": None if r.returncode == 0 else out}
        except Exception as e:
            return {"ok": False, "version_output": "", "error": str(e)}

    def where_python(self) -> Dict[str, Any]:
        try:
            r = subprocess.run(
                ["where", "python"] if sys.platform == "win32" else ["which", "python"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            out = (r.stdout or "").strip()
            paths = [p.strip() for p in out.splitlines() if p.strip()]
            return {"ok": True, "paths": paths, "error": None}
        except Exception as e:
            return {"ok": False, "paths": [], "error": str(e)}

    def is_venv(self) -> Dict[str, Any]:
        try:
            active = getattr(sys, "prefix", "") != getattr(sys, "base_prefix", sys.prefix)
            return {"ok": True, "in_venv": active, "prefix": getattr(sys, "prefix", ""), "error": None}
        except Exception as e:
            return {"ok": False, "in_venv": False, "error": str(e)}

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
