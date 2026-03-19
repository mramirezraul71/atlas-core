"""Backups y rollback local por archivo para autoprogramación."""
from __future__ import annotations

import hashlib
import os
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional


def _repo_root() -> Path:
    root = (
        os.getenv("ATLAS_REPO_PATH")
        or os.getenv("ATLAS_PUSH_ROOT")
        or os.getenv("ATLAS_ROOT")
        or ""
    ).strip()
    if root:
        return Path(root).resolve()
    return Path(__file__).resolve().parents[3]


def _safe_slug(path: Path) -> str:
    return str(path).replace(":", "").replace("\\", "_").replace("/", "_")


@dataclass
class BackupRecord:
    ok: bool
    target_file: str
    existed: bool
    backup_file: Optional[str]
    error: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class RollbackManager:
    """Gestiona snapshots locales y restauración."""

    def __init__(self, repo_root: Optional[Path] = None, snapshot_dir: Optional[Path] = None) -> None:
        self.repo_root = (repo_root or _repo_root()).resolve()
        self.snapshot_dir = (
            snapshot_dir
            or (self.repo_root / "modules" / "humanoid" / "supervisor" / "_snapshots")
        ).resolve()
        self.snapshot_dir.mkdir(parents=True, exist_ok=True)

    def _resolve(self, target_file: str) -> Path:
        p = Path(str(target_file or "").strip())
        if p.is_absolute():
            return p.resolve()
        return (self.repo_root / p).resolve()

    def create_backup(self, target_file: str) -> BackupRecord:
        target = self._resolve(target_file)
        if not target.exists():
            return BackupRecord(
                ok=True,
                target_file=str(target),
                existed=False,
                backup_file=None,
            )

        try:
            content = target.read_bytes()
            digest = hashlib.sha1(content).hexdigest()[:12]
            ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            backup_name = f"{_safe_slug(target)}.{ts}.{digest}.bak"
            backup_path = self.snapshot_dir / backup_name
            shutil.copy2(target, backup_path)
            return BackupRecord(
                ok=True,
                target_file=str(target),
                existed=True,
                backup_file=str(backup_path),
            )
        except Exception as exc:
            return BackupRecord(
                ok=False,
                target_file=str(target),
                existed=True,
                backup_file=None,
                error=str(exc),
            )

    def restore(self, record: BackupRecord) -> Dict[str, Any]:
        target = Path(record.target_file).resolve()
        if not record.ok:
            return {
                "ok": False,
                "target_file": str(target),
                "error": "invalid backup record",
            }

        try:
            if record.existed:
                if not record.backup_file:
                    return {
                        "ok": False,
                        "target_file": str(target),
                        "error": "missing backup file",
                    }
                backup_path = Path(record.backup_file).resolve()
                if not backup_path.exists():
                    return {
                        "ok": False,
                        "target_file": str(target),
                        "error": "backup file not found",
                    }
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(backup_path, target)
                return {
                    "ok": True,
                    "target_file": str(target),
                    "action": "restore_existing_file",
                    "backup_file": str(backup_path),
                }

            if target.exists():
                target.unlink()
            return {
                "ok": True,
                "target_file": str(target),
                "action": "delete_new_file",
            }
        except Exception as exc:
            return {
                "ok": False,
                "target_file": str(target),
                "error": str(exc),
            }

