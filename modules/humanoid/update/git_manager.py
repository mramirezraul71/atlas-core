"""Git operations via SafeShell + Policy. All commands audited."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List, Optional

_UPDATE_ACTOR = "update_engine"


def _repo_path() -> Path:
    p = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or "C:\\ATLAS_PUSH"
    return Path(p).resolve()


def _shell(cmd: str, timeout_sec: int = 60) -> Dict[str, Any]:
    """Run git command via SafeShell + Policy. Returns {ok, stdout, stderr, returncode, error}."""
    try:
        from modules.humanoid import get_humanoid_kernel
        from modules.humanoid.policy import ActorContext, get_policy_engine
        actor = ActorContext(actor=_UPDATE_ACTOR, role=os.getenv("POLICY_DEFAULT_ROLE", "owner"))
        decision = get_policy_engine().can(actor, "hands", "exec_command", target=cmd)
        if not decision.allow:
            return {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": decision.reason or "policy denied"}
        k = get_humanoid_kernel()
        hands = k.registry.get("hands")
        if not hands or not hasattr(hands, "shell"):
            return {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": "hands module not available"}
        return hands.shell.run(cmd, cwd=str(_repo_path()), timeout_sec=timeout_sec, actor=actor)
    except Exception as e:
        return {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": str(e)}


def fetch(remote: str = "origin") -> Dict[str, Any]:
    r = _shell(f"git fetch {remote}", timeout_sec=30)
    return r


def get_head_commit() -> Dict[str, Any]:
    r = _shell("git rev-parse HEAD", timeout_sec=5)
    if r.get("ok"):
        r["commit"] = (r.get("stdout") or "").strip()
    return r


def get_remote_commit(remote: str = "origin", branch: str = "main") -> Dict[str, Any]:
    r = _shell(f"git rev-parse {remote}/{branch}", timeout_sec=5)
    if r.get("ok"):
        r["commit"] = (r.get("stdout") or "").strip()
    return r


def get_status() -> Dict[str, Any]:
    r = _shell("git status --short", timeout_sec=10)
    if r.get("ok"):
        r["output"] = (r.get("stdout") or "").strip()
    return r


def get_diff() -> Dict[str, Any]:
    r = _shell("git diff --no-color", timeout_sec=10)
    if r.get("ok"):
        r["output"] = (r.get("stdout") or "")[:8192]
    return r


def create_staging_branch(remote: str = "origin", branch: str = "main", staging_name: str = "staging") -> Dict[str, Any]:
    """git checkout -B staging origin/main"""
    r = _shell(f"git checkout -B {staging_name} {remote}/{branch}", timeout_sec=15)
    return r


def checkout_branch(branch: str) -> Dict[str, Any]:
    r = _shell(f"git checkout {branch}", timeout_sec=15)
    return r


def merge_staging(staging_name: str = "staging") -> Dict[str, Any]:
    r = _shell(f"git merge {staging_name} --no-edit", timeout_sec=30)
    return r


def delete_branch(branch: str, force: bool = True) -> Dict[str, Any]:
    opt = " -D" if force else " -d"
    r = _shell(f"git branch{opt} {branch}", timeout_sec=5)
    return r


def reset_hard(commit: str) -> Dict[str, Any]:
    r = _shell(f"git reset --hard {commit}", timeout_sec=15)
    return r


def get_current_branch() -> Dict[str, Any]:
    r = _shell("git rev-parse --abbrev-ref HEAD", timeout_sec=5)
    if r.get("ok"):
        r["branch"] = (r.get("stdout") or "").strip()
    return r
