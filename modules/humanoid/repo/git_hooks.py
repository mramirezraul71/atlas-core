from __future__ import annotations

import os
from pathlib import Path


def _repo_root() -> Path:
    root = (os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or "").strip()
    if root:
        return Path(root).resolve()
    return Path(__file__).resolve().parents[4]


def ensure_post_commit_hook() -> dict:
    """
    Instala (best-effort) un hook post-commit para notificar por OPS (Audio/Telegram).
    Nota: .git/hooks no se versiona, por eso se auto-instala al arranque.
    """
    repo = _repo_root()
    hooks = repo / ".git" / "hooks"
    if not hooks.exists():
        return {"ok": False, "error": "no_git_hooks_dir", "path": str(hooks)}
    target = hooks / "post-commit"
    marker = hooks / ".atlas_post_commit_v1"

    script = """#!/bin/sh
# ATLAS: post-commit notify (Audio/Telegram). Best-effort, never blocks commit.
PYTHON_BIN="${ATLAS_HOOK_PYTHON:-python}"
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT" >/dev/null 2>&1 || true
$PYTHON_BIN "scripts/git_post_commit_hook.py" >/dev/null 2>&1 || true
exit 0
"""
    try:
        hooks.mkdir(parents=True, exist_ok=True)
        # Idempotente: si marker existe, no reescribir salvo que falte el hook
        if target.exists() and marker.exists():
            return {"ok": True, "installed": False, "path": str(target)}
        target.write_text(script, encoding="utf-8", errors="ignore")
        marker.write_text("v1", encoding="utf-8")
        try:
            # En Unix, hacerlo ejecutable; en Windows no molesta
            target.chmod(0o755)
        except Exception:
            pass
        return {"ok": True, "installed": True, "path": str(target)}
    except Exception as e:
        return {"ok": False, "error": str(e), "path": str(target)}

