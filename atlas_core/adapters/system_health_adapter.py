"""System health with graceful fallback and actionable risk signals."""
from __future__ import annotations

import importlib
import logging
import shutil
import time
from pathlib import Path
from typing import Any

from ..brain.models import Command, ModuleState, RiskState

logger = logging.getLogger("atlas.brain.adapter.system_health")


class SystemHealthAdapter:
    name = "system_health"

    def __init__(self, psutil_module: Any | None = None, root_path: Path | None = None) -> None:
        self._mode = "monitor"
        self._started_monotonic = time.monotonic()
        self._psutil_module = psutil_module
        self._root_path = root_path or Path("C:\\")

    def _get_psutil(self) -> Any | None:
        if self._psutil_module is False:  # explicit disable for tests/fallback
            return None
        if self._psutil_module is not None:
            return self._psutil_module
        try:
            return importlib.import_module("psutil")
        except Exception:
            return None

    def _metrics(self) -> dict[str, Any]:
        psutil = self._get_psutil()
        cpu = mem = None
        process_uptime_sec = round(time.monotonic() - self._started_monotonic, 2)
        if psutil is not None:
            try:
                cpu = float(psutil.cpu_percent(interval=None))
            except Exception:
                cpu = None
            try:
                mem = float(psutil.virtual_memory().percent)
            except Exception:
                mem = None
        disk_total = disk_free = None
        disk_used_pct = None
        try:
            usage = shutil.disk_usage(str(self._root_path))
            disk_total = float(usage.total)
            disk_free = float(usage.free)
            if usage.total > 0:
                disk_used_pct = round((usage.used / usage.total) * 100.0, 2)
        except Exception:
            pass

        dep_checks = {
            "psutil_available": psutil is not None,
            "disk_metrics_available": disk_total is not None,
        }
        return {
            "cpu_percent": cpu,
            "memory_percent": mem,
            "process_uptime_sec": process_uptime_sec,
            "disk_total_bytes": disk_total,
            "disk_free_bytes": disk_free,
            "disk_used_pct": disk_used_pct,
            "dependency_checks": dep_checks,
            "fallback_mode": psutil is None,
        }

    @staticmethod
    def _derive_health(metrics: dict[str, Any]) -> tuple[str, str]:
        cpu = metrics.get("cpu_percent")
        mem = metrics.get("memory_percent")
        disk_pct = metrics.get("disk_used_pct")
        disk_free = metrics.get("disk_free_bytes")

        if (cpu is not None and cpu >= 98) or (mem is not None and mem >= 98):
            return "critical", "cpu_or_memory_critical"
        if disk_pct is not None and disk_pct >= 98:
            return "critical", "disk_usage_critical"
        if disk_free is not None and float(disk_free) < 1_000_000_000:
            return "critical", "disk_free_critical"

        if (cpu is not None and cpu >= 90) or (mem is not None and mem >= 90):
            return "degraded", "cpu_or_memory_degraded"
        if disk_pct is not None and disk_pct >= 92:
            return "degraded", "disk_usage_degraded"
        if disk_free is not None and float(disk_free) < 3_000_000_000:
            return "degraded", "disk_free_degraded"
        return "ok", "within_thresholds"

    @staticmethod
    def _derive_risk(health: str, reason: str) -> str:
        if health == "critical":
            return "critical"
        if reason.startswith("cpu_or_memory_degraded"):
            return "high"
        if health == "degraded":
            return "medium"
        return "low"

    def get_state(self) -> ModuleState:
        metrics = self._metrics()
        health, reason = self._derive_health(metrics)
        details: dict[str, Any] = {
            **metrics,
            "health_reason": reason,
        }
        return ModuleState(name=self.name, health=health, mode=self._mode, details=details)

    def get_risks(self) -> RiskState:
        metrics = self._metrics()
        health, reason = self._derive_health(metrics)
        lvl = self._derive_risk(health, reason)
        return RiskState(
            name=self.name,
            level=lvl,  # type: ignore[arg-type]
            details={
                "health_reason": reason,
                "cpu": metrics.get("cpu_percent"),
                "mem": metrics.get("memory_percent"),
                "disk_used_pct": metrics.get("disk_used_pct"),
            },
        )

    def apply_command(self, command: Command) -> dict[str, Any]:
        a = command.action.lower()
        if a == "status":
            st = self.get_state()
            return {"ok": True, "health": st.health, "details": st.details}
        if a == "set_mode":
            self._mode = str(command.params.get("mode", "monitor"))
            return {"ok": True, "mode": self._mode}
        return {"ok": False, "error": f"unsupported:{a}"}
