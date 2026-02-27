"""Push de repos: otras apps cuando se solicita por Cursor o por chat. Usa config repo_monitor (known_apps)."""
from __future__ import annotations

import fnmatch
import os
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional

# Raíz por defecto (ATLAS_PUSH)
def _default_root() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT")
    if root:
        return Path(root).resolve()
    return Path(__file__).resolve().parent.parent


def get_config() -> Dict[str, Any]:
    """Carga config/repo_monitor.yaml (known_apps, git.exclude_paths)."""
    config_path = os.getenv("REPO_MONITOR_CONFIG") or str(_default_root() / "config" / "repo_monitor.yaml")
    path = Path(config_path)
    if not path.is_file():
        return {"known_apps": {}, "git": {"exclude_paths": []}}
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return data
    except Exception:
        return {"known_apps": {}, "git": {"exclude_paths": []}}


def resolve_path(app_id: Optional[str] = None, repo_path: Optional[str] = None) -> Path:
    """
    Resuelve directorio del repo: app_id -> known_apps, o repo_path, o raíz actual.
    app_id puede ser "atlas_push", "atlas_nexus", "robot", etc.
    """
    cfg = get_config()
    known = (cfg.get("known_apps") or {}) or {}
    if app_id:
        app_id = (app_id or "").strip().lower().replace(" ", "_")
        if app_id in known:
            p = known[app_id]
            if p is None or (isinstance(p, str) and p.strip() == ""):
                return _default_root()
            path = Path(p)
            if not path.is_absolute():
                path = _default_root() / path
            return path.resolve()
    if repo_path and str(repo_path).strip():
        return Path(repo_path).resolve()
    return _default_root()


def _git(repo: Path, *args: str, timeout_sec: int = 60) -> Dict[str, Any]:
    try:
        r = subprocess.run(
            ["git"] + list(args),
            cwd=str(repo),
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            env={**os.environ, "LANG": "C"},
        )
        return {
            "ok": r.returncode == 0,
            "stdout": (r.stdout or "").strip(),
            "stderr": (r.stderr or "").strip(),
            "returncode": r.returncode,
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "stdout": "", "stderr": "timeout", "returncode": -1}
    except Exception as e:
        return {"ok": False, "stdout": "", "stderr": str(e), "returncode": -1}


def _should_exclude(path: str, exclude_paths: List[str]) -> bool:
    path_n = path.replace("\\", "/")
    for pat in exclude_paths:
        if not pat:
            continue
        pat_n = (pat or "").replace("\\", "/").rstrip("/")
        if "*" in pat_n:
            if fnmatch.fnmatch(path_n, pat_n):
                return True
        if path_n == pat_n or path_n.startswith(pat_n + "/"):
            return True
    return False


def _filter_status_lines(lines: List[str], exclude_paths: List[str]) -> List[str]:
    return [ln for ln in lines if not _should_exclude(ln.split(maxsplit=1)[-1].strip() if len(ln.split(maxsplit=1)) >= 2 else ln, exclude_paths)]


def push_repo(
    repo_path: Optional[Path] = None,
    app_id: Optional[str] = None,
    repo_path_str: Optional[str] = None,
    message: str = "chore: sync (solicitado por Cursor/chat)",
    config: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Ejecuta add + commit + push en el repo indicado (otra app o el actual).
    repo_path | app_id | repo_path_str: cuál usar (prioridad repo_path > app_id > repo_path_str > raíz).
    Retorna {ok, message, stdout, stderr, error, branch}.
    """
    repo = repo_path if repo_path is not None else resolve_path(app_id=app_id, repo_path=repo_path_str)
    cfg = config or get_config()
    exclude = (cfg.get("git") or {}).get("exclude_paths") or []
    remote = (cfg.get("repo") or {}).get("remote", "origin")

    if not (repo / ".git").exists():
        return {"ok": False, "message": "No es un repositorio git: %s" % repo, "error": "not_a_repo", "branch": None}

    # Branch actual
    r_branch = _git(repo, "rev-parse", "--abbrev-ref", "HEAD", timeout_sec=5)
    branch = (r_branch.get("stdout") or "main").strip() if r_branch.get("ok") else "main"

    # Status filtrado
    status_r = _git(repo, "status", "--short", timeout_sec=10)
    if not status_r.get("ok"):
        return {"ok": False, "message": "git status falló", "stderr": status_r.get("stderr"), "error": "status_fail", "branch": branch}
    lines = [ln for ln in (status_r.get("stdout") or "").splitlines() if ln.strip()]
    filtered = _filter_status_lines(lines, exclude)
    if not filtered:
        return {"ok": True, "message": "Sin cambios para commit en %s" % repo.name, "branch": branch, "pushed": False}

    paths = [ln.split(maxsplit=1)[1].strip() for ln in filtered if len(ln.split(maxsplit=1)) >= 2]
    for p in paths:
        _git(repo, "add", p, timeout_sec=5)
    commit_r = _git(repo, "commit", "-m", message, timeout_sec=10)
    if not commit_r.get("ok"):
        if "nothing to commit" in (commit_r.get("stdout") or "") + (commit_r.get("stderr") or ""):
            return {"ok": True, "message": "Nada que commitear (working tree clean)", "branch": branch, "pushed": False}
        return {"ok": False, "message": "Commit falló", "stdout": commit_r.get("stdout"), "stderr": commit_r.get("stderr"), "error": "commit_fail", "branch": branch}

    push_r = _git(repo, "push", remote, branch, timeout_sec=60)
    if not push_r.get("ok"):
        return {"ok": False, "message": "Push falló: %s" % (push_r.get("stderr") or "")[:200], "stderr": push_r.get("stderr"), "error": "push_fail", "branch": branch}
    return {"ok": True, "message": "Push OK: %s/%s (%s)" % (remote, branch, repo.name), "branch": branch, "pushed": True}


def list_known_apps() -> Dict[str, str]:
    """Devuelve { app_id: ruta } para uso en chat/Cursor."""
    cfg = get_config()
    known = (cfg.get("known_apps") or {}) or {}
    root = _default_root()
    out = {}
    for k, v in known.items():
        if v is None or (isinstance(v, str) and not v.strip()):
            out[k] = str(root)
        else:
            out[k] = str(Path(v).resolve())
    return out
