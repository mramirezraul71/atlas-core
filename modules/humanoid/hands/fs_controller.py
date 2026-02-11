"""File system controller: safe read/list operations."""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

class FileSystemController:
    """Safe filesystem operations: list dirs, read files, resolve paths. No write/delete."""

    def list_dir(self, path: str, pattern: str = "*") -> Dict[str, Any]:
        try:
            p = Path(path).resolve()
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
            if not p.exists() or not p.is_file():
                return {"ok": False, "content": "", "error": "file not found"}
            if p.stat().st_size > max_size:
                return {"ok": False, "content": "", "error": f"file too large (>{max_size})"}
            return {"ok": True, "content": p.read_text(encoding=encoding, errors="replace"), "error": None}
        except Exception as e:
            return {"ok": False, "content": "", "error": str(e)}

    def resolve(self, path: str) -> Dict[str, Any]:
        try:
            return {"ok": True, "path": str(Path(path).resolve()), "error": None}
        except Exception as e:
            return {"ok": False, "path": "", "error": str(e)}
