from __future__ import annotations

import os
import stat
import subprocess
import sys
from pathlib import Path

HOOK_SCRIPT = """#!/bin/sh
# ATLAS: post-commit notify (Audio/Telegram). Best-effort, never blocks commit.
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$REPO_ROOT/logs"
LOG_FILE="$LOG_DIR/git_post_commit.log"
if [ -n "${ATLAS_HOOK_PYTHON:-}" ]; then
  PYTHON_BIN="${ATLAS_HOOK_PYTHON}"
elif [ -x "$REPO_ROOT/.venv/Scripts/python.exe" ]; then
  PYTHON_BIN="$REPO_ROOT/.venv/Scripts/python.exe"
else
  PYTHON_BIN="python"
fi

mkdir -p "$LOG_DIR" >/dev/null 2>&1 || true

if [ "${ATLAS_GIT_HOOKS_ENABLED:-1}" != "1" ]; then
  printf "[%s] [INFO] post-commit deshabilitado (ATLAS_GIT_HOOKS_ENABLED=%s)\\n" "$(date +"%Y-%m-%dT%H:%M:%S%z")" "${ATLAS_GIT_HOOKS_ENABLED:-unset}" >> "$LOG_FILE" 2>/dev/null || true
  exit 0
fi

cd "$REPO_ROOT" >/dev/null 2>&1 || true
{
  printf "[%s] [INFO] post-commit start\\n" "$(date +"%Y-%m-%dT%H:%M:%S%z")"
  ATLAS_GIT_HOOKS_ENABLED=1 "$PYTHON_BIN" "scripts/git_post_commit_hook.py"
  RC=$?
  printf "[%s] [INFO] post-commit end rc=%s\\n" "$(date +"%Y-%m-%dT%H:%M:%S%z")" "$RC"
} >> "$LOG_FILE" 2>&1 || true

exit 0
"""


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _git(repo: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *args],
        cwd=str(repo),
        capture_output=True,
        text=True,
        timeout=10,
        env={**os.environ, "LANG": "C"},
    )


def install_git_hooks() -> dict:
    repo = _repo_root()
    if not (repo / ".git").exists():
        return {"ok": False, "error": "no_git_repo", "repo": str(repo)}

    hooks_dir = repo / ".githooks"
    hooks_dir.mkdir(parents=True, exist_ok=True)
    hook_path = hooks_dir / "post-commit"
    hook_path.write_text(HOOK_SCRIPT, encoding="utf-8")

    try:
        mode = hook_path.stat().st_mode
        hook_path.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    except Exception:
        pass

    cfg = _git(repo, "config", "core.hooksPath", ".githooks")
    if cfg.returncode != 0:
        return {
            "ok": False,
            "error": "git_config_failed",
            "stderr": (cfg.stderr or "").strip(),
        }

    check = _git(repo, "config", "--get", "core.hooksPath")
    hooks_path = (check.stdout or "").strip() if check.returncode == 0 else ""
    return {
        "ok": True,
        "hooks_path": hooks_path,
        "hook_file": str(hook_path),
        "python": sys.executable,
    }


def main() -> int:
    result = install_git_hooks()
    if result.get("ok"):
        print(
            "OK hooks installed: core.hooksPath=%s hook=%s"
            % (result.get("hooks_path"), result.get("hook_file"))
        )
        return 0
    print("ERROR install_git_hooks: %s" % result)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

