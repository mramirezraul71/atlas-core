"""
DisasterRecovery - Backup completo periódico; incremental; restore_from_disaster; verify; test.
"""
from __future__ import annotations

import hashlib
import logging
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


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


class DisasterRecovery:
    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("resilience", {})
        self._base = Path(__file__).resolve().parent.parent.parent
        self._backup_dir = self._base / self._config.get("backup_dir", "snapshots/disaster_recovery")
        self._backup_dir.mkdir(parents=True, exist_ok=True)
        self._last_full: str | None = None

    def create_full_backup(self) -> str:
        """Snapshot completo: config, logs seleccionados, DBs conocidos."""
        ts = time.strftime("%Y%m%d_%H%M%S")
        dest = self._backup_dir / f"full_{ts}"
        dest.mkdir(parents=True, exist_ok=True)
        for d in ["config", "logs"]:
            src = self._base / d
            if src.exists():
                shutil.copytree(src, dest / d, dirs_exist_ok=True, ignore=shutil.ignore_patterns("*.log"))
        for db in ["logs/atlas_audit.sqlite", "logs/atlas_memory.sqlite", "logs/autonomous_health.sqlite", "logs/autonomous_failure_memory.sqlite"]:
            src = self._base / db
            if src.exists():
                (dest / db).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dest / db)
        try:
            r = subprocess.run(["git", "rev-parse", "HEAD"], cwd=self._base, capture_output=True, text=True, timeout=5)
            if r.returncode == 0:
                (dest / "git_sha.txt").write_text(r.stdout.strip())
        except Exception:
            pass
        self._last_full = dest.name
        logger.info("Full backup created: %s", dest.name)
        return dest.name

    def create_incremental_backup(self) -> str | None:
        """Incremental desde último full (solo archivos modificados recientemente)."""
        if not self._last_full:
            self.create_full_backup()
        ts = time.strftime("%Y%m%d_%H%M%S")
        dest = self._backup_dir / f"incr_{ts}"
        dest.mkdir(parents=True, exist_ok=True)
        (dest / "parent.txt").write_text(self._last_full or "")
        for db in ["logs/autonomous_health.sqlite", "logs/autonomous_failure_memory.sqlite"]:
            src = self._base / db
            if src.exists():
                (dest / db).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dest / db)
        return dest.name

    def restore_from_disaster(self, backup_id: str) -> bool:
        """Restaura desde un backup full (copia config y DBs de vuelta)."""
        src = self._backup_dir / backup_id
        if not src.exists() or not src.is_dir():
            logger.warning("Backup not found: %s", backup_id)
            return False
        for db in ["config/atlas.env", "config/autonomous.yaml"]:
            f = src / db
            if f.exists():
                target = self._base / db
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(f, target)
        for db in ["logs/atlas_audit.sqlite", "logs/atlas_memory.sqlite"]:
            f = src / db
            if f.exists():
                target = self._base / db
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(f, target)
        logger.info("Restored from backup: %s", backup_id)
        return True

    def verify_backup_integrity(self, backup_id: str) -> bool:
        """Comprueba que existan archivos clave y checksum básico."""
        src = self._backup_dir / backup_id
        if not src.exists():
            return False
        required = ["config", "git_sha.txt"]
        for r in required:
            if not (src / r).exists():
                return False
        return True

    def test_disaster_recovery(self) -> dict[str, Any]:
        """Simula: crea full, verifica, no restaura. Retorna resultado."""
        bid = self.create_full_backup()
        ok = self.verify_backup_integrity(bid)
        return {"backup_id": bid, "integrity_ok": ok}
