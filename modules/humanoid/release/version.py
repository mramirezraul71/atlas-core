"""Read VERSION file and git sha; expose channel (stable/canary)."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict


def _repo_root() -> Path:
    p = os.getenv("ATLAS_REPO_PATH") or os.getenv("POLICY_ALLOWED_PATHS") or "C:\\ATLAS_PUSH"
    return Path(p).resolve()


def _read_version() -> str:
    vf = _repo_root() / "VERSION"
    if vf.exists():
        return vf.read_text(encoding="utf-8").strip() or "0.0.0"
    return "0.0.0"


def _git_sha() -> str:
    try:
        import subprocess
        r = subprocess.run(
            ["git", "rev-parse", "--short=8", "HEAD"],
            cwd=str(_repo_root()),
            capture_output=True,
            text=True,
            timeout=5,
        )
        if r.returncode == 0 and r.stdout:
            return r.stdout.strip()
    except Exception:
        pass
    return ""


def _channel() -> str:
    v = os.getenv("RELEASE_CHANNEL", "canary").strip().lower()
    return v if v in ("stable", "canary") else "canary"


def _version_from_tag() -> str:
    """If PRODUCT_VERSION_FROM_TAG=true, try git describe --tags."""
    if os.getenv("PRODUCT_VERSION_FROM_TAG", "true").strip().lower() not in ("1", "true", "yes"):
        return ""
    try:
        import subprocess
        r = subprocess.run(
            ["git", "describe", "--tags", "--always", "--dirty"],
            cwd=str(_repo_root()),
            capture_output=True,
            text=True,
            timeout=5,
        )
        if r.returncode == 0 and r.stdout:
            return r.stdout.strip().strip("v")
    except Exception:
        pass
    return ""


def _product_edition() -> str:
    return os.getenv("PRODUCT_EDITION", "community").strip().lower() or "community"


def get_version_info() -> Dict[str, Any]:
    version = _read_version()
    from_tag = _version_from_tag()
    if from_tag:
        version = from_tag
    return {
        "version": version,
        "git_sha": _git_sha(),
        "channel": _channel(),
        "edition": _product_edition(),
        "product_mode": os.getenv("PRODUCT_MODE", "false").strip().lower() in ("1", "true", "yes"),
    }
