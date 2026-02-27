"""Dependency resolver: compare required vs installed."""
from __future__ import annotations

from typing import Any, Dict, List, Optional


class DepResolver:
    """Check if required packages are installed (by name). No install."""

    def __init__(self, pip_list_result: Optional[Dict[str, Any]] = None) -> None:
        self._packages = {
            (p.get("name") or "").lower(): p.get("version")
            for p in (pip_list_result or {}).get("packages", [])
        }

    def set_packages(self, packages: List[Dict[str, Any]]) -> None:
        self._packages = {(p.get("name") or "").lower(): p.get("version") for p in packages}

    def check(self, required: List[str]) -> Dict[str, Any]:
        """Returns {ok, missing, installed}."""
        missing = []
        installed = []
        for name in required:
            key = name.lower().split("[")[0].strip()
            if key in self._packages:
                installed.append({"name": key, "version": self._packages[key]})
            else:
                missing.append(key)
        return {"ok": len(missing) == 0, "missing": missing, "installed": installed}
