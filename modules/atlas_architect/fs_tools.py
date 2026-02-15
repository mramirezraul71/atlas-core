from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from .ans_logger import ANSBitacoraLogger, ChangeRecord
from .atomic_patcher import AtomicPatcher, PatchOp


@dataclass
class FSResult:
    ok: bool
    path: str
    error: Optional[str] = None


class FilesystemTools:
    """FS tools para el agente: read_file, write_file, create_directory, list_files."""

    def __init__(self, repo_root: Path, ans_logger: Optional[ANSBitacoraLogger] = None) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.ans = ans_logger
        self.patcher = AtomicPatcher()

    def _abs(self, p: str | Path) -> Path:
        p = Path(p)
        if p.is_absolute():
            return p
        return (self.repo_root / p).resolve()

    def read_file(self, path: str, max_chars: int = 200_000) -> str:
        p = self._abs(path)
        txt = p.read_text(encoding="utf-8", errors="ignore")
        return txt if len(txt) <= max_chars else txt[:max_chars] + "\n\n…(truncado)"

    def write_file(self, path: str, content: str, justification: str = "") -> ChangeRecord:
        p = self._abs(path)
        before = ""
        try:
            if p.exists():
                before = p.read_text(encoding="utf-8", errors="ignore")
        except Exception:
            before = ""
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(content, encoding="utf-8", errors="ignore")
        after = content
        if self.ans:
            return self.ans.log_change(p, before, after, justification=justification or "update file", ok=True)
        # Sin logger: devolver record mínimo
        return ChangeRecord(file_path=str(p), justification=justification, diff_path="", ok=True)

    def atomic_patch(self, path: str, ops: List[PatchOp], justification: str = "") -> ChangeRecord:
        """Edita un archivo existente aplicando operaciones atómicas (anclas/regex)."""
        p = self._abs(path)
        before = ""
        if p.exists():
            before = p.read_text(encoding="utf-8", errors="ignore")
        else:
            before = ""
        after, _diff = self.patcher.apply(before, ops, file_path=str(p).replace("\\", "/"))
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(after, encoding="utf-8", errors="ignore")
        if self.ans:
            return self.ans.log_change(p, before, after, justification=justification or "atomic patch", ok=True)
        return ChangeRecord(file_path=str(p), justification=justification, diff_path="", ok=True)

    def atomic_patch_preview(self, path: str, ops: List[PatchOp]) -> dict:
        """Preview sin escribir: retorna {ok, before_len, after_len, diff}."""
        p = self._abs(path)
        before = p.read_text(encoding="utf-8", errors="ignore") if p.exists() else ""
        after, diff = self.patcher.apply(before, ops, file_path=str(p).replace("\\", "/"))
        return {"ok": True, "path": str(p), "before_len": len(before), "after_len": len(after), "diff": diff[:20000]}

    def create_directory(self, path: str) -> FSResult:
        p = self._abs(path)
        try:
            p.mkdir(parents=True, exist_ok=True)
            if self.ans:
                self.ans.log_suggestion(f"Directorio creado/asegurado: {p}")
            return FSResult(ok=True, path=str(p))
        except Exception as e:
            if self.ans:
                self.ans.log_suggestion(f"Fallo creando directorio {p}: {e}")
            return FSResult(ok=False, path=str(p), error=str(e))

    def list_files(self, path: str = ".", pattern: str = "*", limit: int = 200) -> List[str]:
        p = self._abs(path)
        if not p.exists():
            return []
        out: List[str] = []
        try:
            for fp in p.rglob(pattern):
                if fp.is_file():
                    out.append(str(fp))
                if len(out) >= limit:
                    break
        except Exception:
            return out
        return out

