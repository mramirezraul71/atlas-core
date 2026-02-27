"""
RecoveryStrategies - Estrategias de recuperación: retry, restart, rollback, fallback, degrade, isolate, alert.
"""
from __future__ import annotations

import logging
import os
import subprocess
import time
from pathlib import Path
from typing import Any, Callable, TypeVar

from .error_classifier import RecoveryStrategy

logger = logging.getLogger(__name__)
T = TypeVar("T")


def _load_config() -> dict:
    cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "autonomous.yaml"
    if not cfg_path.exists():
        return {}
    try:
        import yaml
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


class RecoveryStrategies:
    """
    Ejecuta estrategias de recuperación.
    restart_service usa scripts del repo (nexus_heartbeat / scripts).
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("self_healing", {})
        self._retry_cfg = self._config.get("retry_policy", {})
        self._base = Path(__file__).resolve().parent.parent.parent

    def retry_with_backoff(
        self,
        func: Callable[[], T],
        max_attempts: int | None = None,
        initial_delay: float = 5,
        max_delay: float = 300,
        exponential_base: float = 3,
    ) -> T:
        """Reintenta func con exponential backoff + jitter. Lanza la última excepción si todos fallan."""
        max_attempts = max_attempts or int(self._retry_cfg.get("max_attempts", 5))
        initial_delay = float(self._retry_cfg.get("initial_delay", initial_delay))
        max_delay = float(self._retry_cfg.get("max_delay", max_delay))
        exponential_base = float(self._retry_cfg.get("exponential_base", exponential_base))
        import random
        last_exc = None
        for attempt in range(max_attempts):
            try:
                return func()
            except Exception as e:
                last_exc = e
                if attempt == max_attempts - 1:
                    raise
                delay = min(initial_delay * (exponential_base ** attempt), max_delay)
                jitter = delay * 0.1 * random.random()
                time.sleep(delay + jitter)
        raise last_exc

    def restart_service(self, service_name: str) -> bool:
        """Reinicia un servicio (nexus, robot, push). Usa scripts del repo."""
        service_name = (service_name or "").lower()
        if service_name == "nexus":
            script = self._base / "scripts" / "restart_service_clean.ps1"
            if script.exists():
                try:
                    subprocess.Popen(
                        ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(script), "-Service", "nexus"],
                        cwd=str(self._base),
                        creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
                    )
                    return True
                except Exception as e:
                    logger.exception("restart_service nexus: %s", e)
                    return False
            # Fallback: free port + python
            free_port = self._base / "scripts" / "free_port.ps1"
            if free_port.exists():
                try:
                    subprocess.run(
                        ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(free_port), "-Port", "8000", "-Kill"],
                        cwd=str(self._base),
                        timeout=15,
                        capture_output=True,
                    )
                except Exception:
                    pass
            nexus_dir = os.getenv("NEXUS_ATLAS_PATH") or str(self._base / "nexus" / "atlas_nexus")
            try:
                subprocess.Popen(
                    [os.environ.get("PYTHON", "python"), "nexus.py", "--mode", "api"],
                    cwd=nexus_dir,
                    creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
                )
                return True
            except Exception as e:
                logger.exception("nexus.py start: %s", e)
                return False
        if service_name == "robot":
            script = self._base / "scripts" / "restart_service_clean.ps1"
            if script.exists():
                try:
                    subprocess.Popen(
                        ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(script), "-Service", "robot"],
                        cwd=str(self._base),
                        creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
                    )
                    return True
                except Exception as e:
                    logger.exception("restart_service robot: %s", e)
            return False
        if service_name == "push":
            logger.warning("Restart PUSH not auto-executed (would stop this process)")
            return False
        return False

    def restart_all(self) -> bool:
        """Reinicia nexus + robot (push no se reinicia desde aquí)."""
        ok_nexus = self.restart_service("nexus")
        time.sleep(2)
        ok_robot = self.restart_service("robot")
        return ok_nexus or ok_robot

    def rollback_to_version(self, version: str) -> bool:
        """Restaura a una versión/snapshot. Depende de Evolution BackupManager (opcional)."""
        try:
            from autonomous.evolution.backup_manager import BackupManager
            bm = BackupManager()
            return bm.restore_snapshot(version)
        except ImportError:
            logger.warning("rollback_to_version: BackupManager no disponible")
            return False
        except Exception as e:
            logger.exception("rollback_to_version: %s", e)
            return False

    def activate_fallback(self, component: str) -> bool:
        """Activa componente de respaldo (p. ej. LLM alternativo). Por ahora solo log."""
        logger.info("activate_fallback component=%s (no-op in this version)", component)
        return True

    def enter_degraded_mode(self, available_resources: dict[str, Any] | None = None) -> bool:
        """Entra en modo degradado. Delega a Resilience SurvivalMode (opcional)."""
        try:
            from autonomous.resilience.survival_mode import SurvivalMode
            sm = SurvivalMode()
            sm.enter_survival_mode("self_healing requested")
            return True
        except ImportError:
            logger.warning("enter_degraded_mode: SurvivalMode no disponible")
            return False
        except Exception as e:
            logger.exception("enter_degraded_mode: %s", e)
            return False

    def isolate_component(self, component: str) -> bool:
        """Desactiva componente sin afectar el resto. Por ahora log."""
        logger.warning("isolate_component %s (no-op in this version)", component)
        return True
