"""Safe updater: plan only, no automatic execution."""
from __future__ import annotations

from typing import Any, Dict, List

from .env_scanner import EnvScanner
from .dep_resolver import DepResolver


class Updater:
    """Produce an update plan (pip install commands). Does NOT run them."""

    def __init__(self) -> None:
        self.scanner = EnvScanner()
        self.resolver = DepResolver()

    def plan(self, required_packages: List[str]) -> Dict[str, Any]:
        """Returns {ok, python_version, pip_list_ok, missing, install_plan, error}."""
        out: Dict[str, Any] = {
            "ok": True,
            "python_version": None,
            "pip_list_ok": False,
            "missing": [],
            "install_plan": [],
            "error": None,
        }
        pv = self.scanner.python_version()
        out["python_version"] = pv.get("version")
        if not pv.get("ok"):
            out["ok"] = False
            out["error"] = pv.get("error")
            return out
        pl = self.scanner.pip_list()
        out["pip_list_ok"] = pl.get("ok")
        if pl.get("ok"):
            self.resolver.set_packages(pl.get("packages", []))
            ch = self.resolver.check(required_packages)
            out["missing"] = ch.get("missing", [])
            for p in out["missing"]:
                out["install_plan"].append(f"pip install {p}")
        else:
            out["error"] = pl.get("error")
            out["ok"] = False
        return out
