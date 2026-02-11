"""File system controller: safe read/list operations. Respects POLICY_ALLOWED_PATHS."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List

def _allowed_paths() -> List[Path]:
    v = os.getenv("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH")
    return [Path(x.strip()) for x in v.split(",") if x.strip()]


def _path_allowed(p: Path) -> bool:
    try:
        resolved = p.resolve()
        for allowed in _allowed_paths():
            try:
                allowed_resolved = allowed.resolve()
                if resolved == allowed_resolved or resolved.is_relative_to(allowed_resolved):
                    return True
            except (ValueError, OSError):
                continue
        return False
    except (ValueError, OSError):
        return False


class FileSystemController:
    """Safe filesystem operations: list dirs, read files, resolve paths. Only under POLICY_ALLOWED_PATHS."""

    def list_dir(self, path: str, pattern: str = "*") -> Dict[str, Any]:
        try:
            p = Path(path).resolve()
            if not _path_allowed(p):
                return {"ok": False, "entries": [], "error": "path not in POLICY_ALLOWED_PATHS"}
            if not p.exists():
                return {"ok": False, "entries": [], "error": "path does not exist"}
            if not p.is_dir():
                return {"ok": False, "entries": [], "error": "not a directory"}
            entries = [x.name for x in p.iterdir() if not pattern or x.match(pattern)]
            return {"ok": True, "entries": entries, "error": None}
        except Exception as e:
            return {"ok": False, "entries": [], "error": str(e)}

    def read_text(self, path: str, encoding: str = "utf-8", max_size: int = 1024 * 1024) -> Dict[str, Any]:
        try:
            p = Path(path).resolve()
            if not _path_allowed(p):
                return {"ok": False, "content": "", "error": "path not in POLICY_ALLOWED_PATHS"}
            if not p.exists() or not p.is_file():
                return {"ok": False, "content": "", "error": "file not found"}
            if p.stat().st_size > max_size:
                return {"ok": False, "content": "", "error": f"file too large (>{max_size})"}
            return {"ok": True, "content": p.read_text(encoding=encoding, errors="replace"), "error": None}
        except Exception as e:
            return {"ok": False, "content": "", "error": str(e)}

    def resolve(self, path: str) -> Dict[str, Any]:
        try:
            p = Path(path).resolve()
            if not _path_allowed(p):
                return {"ok": False, "path": "", "error": "path not in POLICY_ALLOWED_PATHS"}
            return {"ok": True, "path": str(p), "error": None}
        except Exception as e:
            return {"ok": False, "path": "", "error": str(e)}
