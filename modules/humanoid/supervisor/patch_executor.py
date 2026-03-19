"""Patch executor conservador basado en reemplazo de bloques de texto."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

from .models import (
    ALLOW_DELETE_FILES,
    ALLOW_NEW_FILES,
    ChangeProposal,
    PatchExecutionResult,
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


class PatchExecutor:
    """Aplica patches textuales de forma explícita y reversible."""

    def __init__(self, repo_root: Optional[Path] = None) -> None:
        self.repo_root = (repo_root or _repo_root()).resolve()

    def _resolve_target(self, target_file: str) -> Path:
        p = Path(str(target_file or "").strip())
        if p.is_absolute():
            return p.resolve()
        return (self.repo_root / p).resolve()

    def apply_text_patch(
        self,
        proposal: ChangeProposal,
        *,
        allow_new_files: bool = ALLOW_NEW_FILES,
        allow_delete_files: bool = ALLOW_DELETE_FILES,
    ) -> PatchExecutionResult:
        if not proposal.target_file:
            return PatchExecutionResult(
                ok=False,
                changed=False,
                target_file="",
                error="target_file is required",
            )

        if not allow_delete_files and proposal.metadata.get("delete_file"):
            return PatchExecutionResult(
                ok=False,
                changed=False,
                target_file=proposal.target_file,
                error="delete operations are disabled",
            )

        target = self._resolve_target(proposal.target_file)
        target.parent.mkdir(parents=True, exist_ok=True)

        if not target.exists():
            if not allow_new_files or not proposal.allow_new_file:
                return PatchExecutionResult(
                    ok=False,
                    changed=False,
                    target_file=str(target),
                    error="new file creation denied",
                )
            if not proposal.patches:
                return PatchExecutionResult(
                    ok=False,
                    changed=False,
                    target_file=str(target),
                    error="proposal has no patches for new file",
                )
            content = "".join(block.new_text for block in proposal.patches)
            target.write_text(content, encoding="utf-8")
            return PatchExecutionResult(
                ok=True,
                changed=True,
                target_file=str(target),
                changed_files=[str(target)],
                details={"created": True, "patches_applied": len(proposal.patches)},
            )

        original = target.read_text(encoding="utf-8")
        updated = original
        details = {"created": False, "patches_applied": 0}

        for index, block in enumerate(proposal.patches):
            old_text = block.old_text
            new_text = block.new_text

            if old_text:
                if old_text not in updated:
                    return PatchExecutionResult(
                        ok=False,
                        changed=False,
                        target_file=str(target),
                        error=f"patch block {index} not found in target",
                        details={"block_index": index},
                    )
                if block.replace_all:
                    updated = updated.replace(old_text, new_text)
                else:
                    updated = updated.replace(old_text, new_text, 1)
            elif new_text and new_text not in updated:
                updated = updated + new_text

            details["patches_applied"] += 1

        if updated == original:
            return PatchExecutionResult(
                ok=True,
                changed=False,
                target_file=str(target),
                changed_files=[],
                details={**details, "no_op": True},
            )

        target.write_text(updated, encoding="utf-8")
        return PatchExecutionResult(
            ok=True,
            changed=True,
            target_file=str(target),
            changed_files=[str(target)],
            details=details,
        )

