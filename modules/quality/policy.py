from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Tuple


@dataclass(frozen=True)
class GitPolicy:
    """
    Política dura: SIN REBASE.

    Default = seguro (solo chequeo). Para permitir acciones se requiere env vars explícitas.
    """

    allow_commit: bool
    allow_push: bool
    allow_pull: bool
    require_clean_tree_for_pull: bool
    require_clean_tree_for_push: bool
    forbid_rebase: bool
    include_paths_for_commit: Tuple[str, ...]
    exclude_paths: Tuple[str, ...]


def repo_root() -> Path:
    # Este repo (ATLAS_PUSH) es la fuente de verdad para POTs.
    env_root = (os.getenv("ATLAS_PUSH_ROOT") or os.getenv("ATLAS_REPO_PATH") or "").strip()
    if env_root:
        return Path(env_root).resolve()
    return Path(__file__).resolve().parents[2]


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name, "true" if default else "false").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def get_git_policy() -> GitPolicy:
    # Default: solo check (no commit/push/pull) y jamás rebase.
    allow_commit = _env_bool("QUALITY_GIT_ALLOW_COMMIT", False)
    allow_push = _env_bool("QUALITY_GIT_ALLOW_PUSH", False)
    allow_pull = _env_bool("QUALITY_GIT_ALLOW_PULL", False)

    include_paths = tuple(
        p.strip()
        for p in (os.getenv("QUALITY_GIT_INCLUDE_PATHS") or "atlas_adapter,modules,tools,config").split(",")
        if p.strip()
    )
    exclude_paths = tuple(
        p.strip()
        for p in (
            os.getenv("QUALITY_GIT_EXCLUDE_PATHS")
            or ".venv,venv,.temp_venv,logs,snapshots,temp_growth,temp_scripts,temp_workspace,__pycache__,ATLAS_VAULT"
        ).split(",")
        if p.strip()
    )

    return GitPolicy(
        allow_commit=allow_commit,
        allow_push=allow_push,
        allow_pull=allow_pull,
        require_clean_tree_for_pull=True,
        require_clean_tree_for_push=True,
        forbid_rebase=True,
        include_paths_for_commit=include_paths,
        exclude_paths=exclude_paths,
    )


def is_path_excluded(path: str, exclude_paths: Iterable[str]) -> bool:
    p = (path or "").replace("\\", "/").lstrip("./")
    for ex in exclude_paths:
        exn = (ex or "").replace("\\", "/").strip().strip("/")
        if not exn:
            continue
        if p == exn or p.startswith(exn + "/"):
            return True
    return False


def is_path_included(path: str, include_paths: Iterable[str]) -> bool:
    p = (path or "").replace("\\", "/").lstrip("./")
    for inc in include_paths:
        incn = (inc or "").replace("\\", "/").strip().strip("/")
        if not incn:
            continue
        if p == incn or p.startswith(incn + "/"):
            return True
    return False


def filter_paths_for_commit(paths: List[str], policy: GitPolicy) -> List[str]:
    out: List[str] = []
    for p in paths:
        if is_path_excluded(p, policy.exclude_paths):
            continue
        if not is_path_included(p, policy.include_paths_for_commit):
            continue
        out.append(p)
    return out

