from __future__ import annotations

import difflib
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional


def _now_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def _diff_text(before: str, after: str, file_path: str) -> str:
    return "\n".join(
        difflib.unified_diff(
            before.splitlines(),
            after.splitlines(),
            fromfile=file_path + " (before)",
            tofile=file_path + " (after)",
            lineterm="",
        )
    )


@dataclass
class ChangeRecord:
    file_path: str
    justification: str
    diff_path: str
    ok: bool


class ANSBitacoraLogger:
    """Logger industrial a Bitácora ANS (evolution-log) con diff+justificación."""

    def __init__(self, repo_root: Path) -> None:
        self.repo_root = Path(repo_root)
        self.diff_dir = self.repo_root / "logs" / "ans_architect_diffs"
        self.diff_dir.mkdir(parents=True, exist_ok=True)

    def log_suggestion(self, message: str) -> None:
        self._append(f"[ARCHITECT] {message}", ok=True)

    def log_debug(self, message: str, ok: bool = True) -> None:
        self._append(f"[DEBUG] {message}", ok=ok)

    def log_change(self, file_path: Path, before: str, after: str, justification: str, ok: bool = True) -> ChangeRecord:
        rel = str(file_path).replace("\\", "/")
        diff = _diff_text(before, after, rel)
        name = f"{_now_stamp()}_{Path(rel).name}.diff"
        diff_path = self.diff_dir / name
        try:
            diff_path.write_text(diff, encoding="utf-8", errors="ignore")
        except Exception:
            # fallback: best-effort
            diff_path = self.diff_dir / f"{_now_stamp()}_diff.txt"
            diff_path.write_text(diff[:20000], encoding="utf-8", errors="ignore")

        msg = (
            f"[CODE] Aplicando parche en `{rel}`.\n"
            f"Resultado: {'OK' if ok else 'FAIL'}\n"
            f"Justificación: {justification[:300]}\n"
            f"Diff: {diff_path}"
        )
        self._append(msg, ok=ok)
        return ChangeRecord(file_path=rel, justification=justification, diff_path=str(diff_path), ok=ok)

    def _append(self, message: str, ok: bool) -> None:
        try:
            from modules.humanoid.ans.evolution_bitacora import append_evolution_log

            append_evolution_log(message=message, ok=ok, source="atlas_architect")
        except Exception:
            # Best-effort fallback: ignore logging failure
            pass

