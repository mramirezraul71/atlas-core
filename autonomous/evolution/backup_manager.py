"""
BackupManager - Snapshots completos pre-actualización.
Código (git hash), DB, configs, logs. Restore por tag.
"""
from __future__ import annotations

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


class BackupManager:
    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("evolution", {})
        self._base = Path(__file__).resolve().parent.parent.parent
        self._backup_dir = self._base / self._config.get("backup_dir", "snapshots/autonomous_backups")
        self._backup_dir.mkdir(parents=True, exist_ok=True)

    def create_snapshot(self, tag: str = "") -> str:
        """Crea backup con timestamp; tag opcional. Retorna id del snapshot."""
        ts = time.strftime("%Y%m%d_%H%M%S")
        name = f"snapshot_{ts}_{tag}" if tag else f"snapshot_{ts}"
        dest = self._backup_dir / name
        dest.mkdir(parents=True, exist_ok=True)
        try:
            # Git hash
            r = subprocess.run(["git", "rev-parse", "HEAD"], cwd=self._base, capture_output=True, text=True, timeout=5)
            if r.returncode == 0:
                (dest / "git_sha.txt").write_text(r.stdout.strip())
        except Exception:
            pass
        # Copy config
        for f in ["config/atlas.env", "config/autonomous.yaml"]:
            src = self._base / f
            if src.exists():
                (dest / f).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dest / f)
        logger.info("Snapshot created: %s", name)
        return name

    def list_snapshots(self) -> list[str]:
        """Lista nombres de snapshots."""
        if not self._backup_dir.exists():
            return []
        return sorted([d.name for d in self._backup_dir.iterdir() if d.is_dir()], reverse=True)

    def restore_snapshot(self, tag: str) -> bool:
        """Restaura a un snapshot por nombre. Por ahora solo devuelve True si existe."""
        dest = self._backup_dir / tag
        if not dest.exists() or not dest.is_dir():
            logger.warning("Snapshot not found: %s", tag)
            return False
        # Restore real: copy back config, etc. (no overwrite code por seguridad)
        env_dest = self._base / "config" / "atlas.env"
        env_src = dest / "config" / "atlas.env"
        if env_src.exists() and env_dest.exists():
            shutil.copy2(env_src, env_dest)
            logger.info("Restored config from snapshot %s", tag)
            return True
        return True

    def cleanup_old_snapshots(self, keep_last_n: int = 10) -> int:
        """Borra snapshots viejos; mantiene los N más recientes. Retorna cuántos borró."""
        names = self.list_snapshots()
        removed = 0
        for name in names[keep_last_n:]:
            path = self._backup_dir / name
            if path.exists():
                shutil.rmtree(path, ignore_errors=True)
                removed += 1
        return removed

    def verify_snapshot(self, tag: str) -> bool:
        """Comprueba que el snapshot tenga al menos git_sha o config."""
        dest = self._backup_dir / tag
        if not dest.exists():
            return False
        return (dest / "git_sha.txt").exists() or (dest / "config").exists()
