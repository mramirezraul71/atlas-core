"""Safe updater: plan only. Apply controlled by env (UPDATE_APPLY=false = never run)."""
from __future__ import annotations

import os
from typing import Any, Dict, List

from .env_scanner import EnvScanner
from .dep_resolver import DepResolver


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name, "true" if default else "false").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _env_str(name: str, default: str) -> str:
    return (os.getenv(name) or default).strip()


class Updater:
    """Produce an update plan. Does NOT run installs unless UPDATE_APPLY=true (not recommended)."""

    def __init__(self) -> None:
        self.scanner = EnvScanner()
        self.resolver = DepResolver()
        self.update_mode = _env_str("UPDATE_MODE", "plan_only")
        self.update_apply = _env_bool("UPDATE_APPLY", False)
        self.snapshot_before = _env_bool("UPDATE_SNAPSHOT_BEFORE", True)
        self.allow_pip_upgrade = _env_bool("UPDATE_ALLOW_PIP_UPGRADE", False)

    def plan(self, required_packages: List[str]) -> Dict[str, Any]:
        """Returns {ok, python_version, pip_list_ok, missing, install_plan, update_config, error}."""
        out: Dict[str, Any] = {
            "ok": True,
            "python_version": None,
            "pip_list_ok": False,
            "missing": [],
            "install_plan": [],
            "update_config": {
                "UPDATE_MODE": self.update_mode,
                "UPDATE_APPLY": self.update_apply,
                "UPDATE_SNAPSHOT_BEFORE": self.snapshot_before,
                "UPDATE_ALLOW_PIP_UPGRADE": self.allow_pip_upgrade,
            },
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
