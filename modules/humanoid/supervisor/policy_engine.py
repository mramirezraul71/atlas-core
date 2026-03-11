"""Policy engine local para autoprogramación gobernada."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

from .models import (
    ALLOWED_WRITE_ROOTS,
    AUTONOMY_BUILD,
    AUTONOMY_OFF,
    AUTONOMY_SAFE,
    PROTECTED_PATH_SEGMENTS,
    PolicyDecision,
)


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


def _is_under(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


class PolicyEngine:
    """Valida si un patch puede tocar un archivo según políticas locales."""

    def __init__(self, repo_root: Optional[Path] = None) -> None:
        self.repo_root = (repo_root or _repo_root()).resolve()
        self.allowed_roots = tuple((self.repo_root / p).resolve() for p in ALLOWED_WRITE_ROOTS)

    def _resolve_target(self, target_file: str) -> Path:
        raw = Path(str(target_file or "").strip())
        if raw.is_absolute():
            return raw.resolve()
        return (self.repo_root / raw).resolve()

    def _contains_protected_segment(self, target: Path) -> bool:
        parts = {p.lower() for p in target.parts}
        for segment in PROTECTED_PATH_SEGMENTS:
            if segment.lower() in parts:
                return True
        return False

    def _risk_for_target(self, target: Path) -> str:
        supervisor_root = (self.repo_root / "modules" / "humanoid" / "supervisor").resolve()
        tests_root = (self.repo_root / "tests").resolve()
        if _is_under(target, supervisor_root) or _is_under(target, tests_root):
            return "low"
        if _is_under(target, self.repo_root / "modules"):
            return "medium"
        return "high"

    def evaluate(
        self,
        target_file: str,
        *,
        mode: str,
        allow_new_file: bool = False,
        operation: str = "patch",
    ) -> PolicyDecision:
        target = self._resolve_target(target_file)
        normalized = str(target)
        mode_norm = (mode or AUTONOMY_SAFE).strip().lower()
        op_norm = (operation or "patch").strip().lower()

        if mode_norm == AUTONOMY_OFF:
            return PolicyDecision(
                allowed=False,
                reason="autonomy mode OFF (recommendation only)",
                risk="none",
                normalized_target=normalized,
                details={"mode": mode_norm, "operation": op_norm},
            )

        if op_norm == "delete":
            return PolicyDecision(
                allowed=False,
                reason="delete operations are blocked",
                risk="high",
                normalized_target=normalized,
                details={"mode": mode_norm, "operation": op_norm},
            )

        if self._contains_protected_segment(target):
            return PolicyDecision(
                allowed=False,
                reason="protected path segment blocked",
                risk="critical",
                normalized_target=normalized,
                details={"protected_segments": list(PROTECTED_PATH_SEGMENTS)},
            )

        if not _is_under(target, self.repo_root):
            return PolicyDecision(
                allowed=False,
                reason="target is outside repository root",
                risk="critical",
                normalized_target=normalized,
                details={"repo_root": str(self.repo_root)},
            )

        if not any(_is_under(target, root) for root in self.allowed_roots):
            return PolicyDecision(
                allowed=False,
                reason="target path is not in allowed roots",
                risk="high",
                normalized_target=normalized,
                details={"allowed_roots": [str(p) for p in self.allowed_roots]},
            )

        exists = target.exists()
        if not exists and not allow_new_file:
            return PolicyDecision(
                allowed=False,
                reason="new file creation not authorized for this proposal",
                risk="medium",
                normalized_target=normalized,
                details={"mode": mode_norm},
            )

        if not exists and mode_norm == AUTONOMY_SAFE:
            supervisor_root = (self.repo_root / "modules" / "humanoid" / "supervisor").resolve()
            tests_root = (self.repo_root / "tests").resolve()
            if not (_is_under(target, supervisor_root) or _is_under(target, tests_root)):
                return PolicyDecision(
                    allowed=False,
                    reason="SAFE mode allows new files only in supervisor or tests",
                    risk="high",
                    normalized_target=normalized,
                    details={"mode": mode_norm},
                )

        if mode_norm not in (AUTONOMY_SAFE, AUTONOMY_BUILD):
            return PolicyDecision(
                allowed=False,
                reason="unsupported autonomy mode",
                risk="high",
                normalized_target=normalized,
                details={"mode": mode_norm},
            )

        return PolicyDecision(
            allowed=True,
            reason="policy check passed",
            risk=self._risk_for_target(target),
            normalized_target=normalized,
            details={
                "mode": mode_norm,
                "operation": op_norm,
                "exists": exists,
                "allow_new_file": bool(allow_new_file),
            },
        )

