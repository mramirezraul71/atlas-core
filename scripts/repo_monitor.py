#!/usr/bin/env python3
"""
ATLAS PUSH — Monitoreo y actualización automática del repositorio.

Tarea asignada: este script es el responsable de monitorear y actualizar el repo.
Modos:
  --cycle         Un ciclo: fetch, status, opcional pull (según config).
  --after-fix     Commit + push de cambios filtrados (tras arreglos).
  --status-only   Solo estado: branch, head, remote, corto.

Config: config/repo_monitor.yaml (o REPO_MONITOR_CONFIG).
Log: logs/repo_monitor.log
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import logging
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional

# -----------------------------------------------------------------------------
# Configuración
# -----------------------------------------------------------------------------

def _repo_root() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT")
    if root:
        return Path(root).resolve()
    # Desde scripts/ subir a raíz
    p = Path(__file__).resolve().parent.parent
    return p


def _load_config() -> Dict[str, Any]:
    config_path = os.getenv("REPO_MONITOR_CONFIG")
    if not config_path:
        config_path = _repo_root() / "config" / "repo_monitor.yaml"
    path = Path(config_path)
    if not path.is_file():
        return _default_config()
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return {**_default_config(), **data}
    except Exception as e:
        logging.warning("No se pudo cargar %s: %s. Usando valores por defecto.", path, e)
        return _default_config()


def _default_config() -> Dict[str, Any]:
    return {
        "repo": {"path": None, "remote": "origin", "branch": "main", "push_branch": None},
        "git": {"exclude_paths": [], "include_paths_for_commit": []},
        "cycle": {
            "enabled": True,
            "interval_seconds": 300,
            "fetch_on_cycle": True,
            "pull_on_cycle": False,
            "report_only": True,
        },
        "after_fix": {
            "enabled": True,
            "default_message": "chore: auto-sync repo (monitor)",
            "allow_empty_commit": False,
        },
        "on_error": {
            "fetch_fail": {"retries": 3, "delay_seconds": 5, "then": "log_and_continue"},
            "pull_fail": {"retries": 2, "then": "log_and_abort"},
            "push_fail": {"retries": 3, "delay_seconds": 10, "then": "log_and_abort"},
            "reset_branch": "main",
        },
        "bitacora": {
            "enabled": True,
            "dashboard_url": "http://127.0.0.1:8791",
            "timeout_seconds": 3,
        },
        "logging": {
            "file": "logs/repo_monitor.log",
            "level": "INFO",
            "max_bytes": 10485760,
            "backup_count": 3,
        },
    }


# -----------------------------------------------------------------------------
# Logging
# -----------------------------------------------------------------------------

def _setup_logging(cfg: Dict[str, Any]) -> None:
    log_cfg = cfg.get("logging", {}) or {}
    log_file = log_cfg.get("file", "logs/repo_monitor.log")
    level_name = (log_cfg.get("level") or "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)
    root = _repo_root()
    log_path = root / log_file if not os.path.isabs(log_file) else Path(log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        from logging.handlers import RotatingFileHandler
        handler = RotatingFileHandler(
            log_path,
            maxBytes=int(log_cfg.get("max_bytes", 10485760)),
            backupCount=int(log_cfg.get("backup_count", 3)),
            encoding="utf-8",
        )
    except Exception:
        handler = logging.FileHandler(log_path, encoding="utf-8")
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
    handler.setFormatter(fmt)
    log = logging.getLogger("repo_monitor")
    log.setLevel(level)
    log.handlers.clear()
    log.addHandler(handler)
    # También a consola si se pide
    if os.getenv("REPO_MONITOR_VERBOSE", "").strip().lower() in ("1", "true", "yes"):
        ch = logging.StreamHandler(sys.stdout)
        ch.setFormatter(fmt)
        log.addHandler(ch)


# -----------------------------------------------------------------------------
# Bitácora ANS: enviar tareas a la Bitácora (dashboard) para que se vean reflejadas
# -----------------------------------------------------------------------------

def _bitacora(cfg: Dict[str, Any], message: str, ok: bool = True) -> None:
    """Envía una entrada a la Bitácora ANS (POST /ans/evolution-log, source=repo_monitor)."""
    bc = cfg.get("bitacora") or {}
    if not bc.get("enabled", True):
        return
    url = (bc.get("dashboard_url") or "").rstrip("/")
    if not url:
        return
    timeout = int(bc.get("timeout_seconds", 3))
    try:
        data = json.dumps({"message": (message or "")[:500], "ok": ok, "source": "repo_monitor"}).encode("utf-8")
        req = urllib.request.Request(
            f"{url}/ans/evolution-log",
            data=data,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=timeout) as r:
            if r.status != 200:
                logging.getLogger("repo_monitor").debug("bitacora HTTP %s", r.status)
    except Exception as e:
        logging.getLogger("repo_monitor").debug("bitacora POST: %s", e)


# -----------------------------------------------------------------------------
# Git (subprocess, sin dependencias humanoid)
# -----------------------------------------------------------------------------

def _git(repo_path: Path, *args: str, timeout_sec: int = 60) -> Dict[str, Any]:
    cmd = ["git"] + list(args)
    try:
        r = subprocess.run(
            cmd,
            cwd=str(repo_path),
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


def _git_fetch(repo: Path, remote: str, cfg: Dict) -> Dict[str, Any]:
    on_err = (cfg.get("on_error") or {}).get("fetch_fail") or {}
    retries = int(on_err.get("retries", 3))
    delay = int(on_err.get("delay_seconds", 5))
    last = {}
    for attempt in range(1, retries + 1):
        last = _git(repo, "fetch", remote, timeout_sec=30)
        if last.get("ok"):
            return last
        logging.getLogger("repo_monitor").warning("fetch intento %s/%s: %s", attempt, retries, last.get("stderr"))
        if attempt < retries:
            time.sleep(delay)
    then = on_err.get("then", "log_and_continue")
    if then == "reset_to_remote":
        branch = (cfg.get("on_error") or {}).get("reset_branch", "main")
        r2 = _git(repo, "reset", "--hard", f"{remote}/{branch}", timeout_sec=15)
        if r2.get("ok"):
            last["recovered"] = True
    return last


def _git_status_short(repo: Path) -> Dict[str, Any]:
    r = _git(repo, "status", "--short", timeout_sec=10)
    if not r.get("ok"):
        return r
    lines = [ln for ln in (r.get("stdout") or "").splitlines() if ln.strip()]
    r["lines"] = lines
    r["changed"] = len(lines)
    return r


def _should_exclude(path: str, exclude_paths: List[str]) -> bool:
    path_n = path.replace("\\", "/")
    for pat in exclude_paths:
        if not pat:
            continue
        pat_n = pat.replace("\\", "/").rstrip("/")
        if "*" in pat_n:
            if fnmatch.fnmatch(path_n, pat_n):
                return True
        if path_n == pat_n or path_n.startswith(pat_n + "/"):
            return True
    return False


def _filter_status_lines(lines: List[str], exclude_paths: List[str]) -> List[str]:
    out = []
    for ln in lines:
        # " M path" o "MM path"
        parts = ln.split(maxsplit=1)
        if len(parts) < 2:
            out.append(ln)
            continue
        p = parts[1].strip()
        if _should_exclude(p, exclude_paths):
            continue
        out.append(ln)
    return out


def _current_branch(repo: Path) -> Dict[str, Any]:
    r = _git(repo, "rev-parse", "--abbrev-ref", "HEAD", timeout_sec=5)
    if r.get("ok"):
        r["branch"] = (r.get("stdout") or "").strip()
    return r


def _head_commit(repo: Path) -> Dict[str, Any]:
    r = _git(repo, "rev-parse", "HEAD", timeout_sec=5)
    if r.get("ok"):
        r["commit"] = (r.get("stdout") or "").strip()
    return r


def _remote_commit(repo: Path, remote: str, branch: str) -> Dict[str, Any]:
    r = _git(repo, "rev-parse", f"{remote}/{branch}", timeout_sec=5)
    if r.get("ok"):
        r["commit"] = (r.get("stdout") or "").strip()
    return r


# -----------------------------------------------------------------------------
# Ciclo: fetch + status + opcional pull
# -----------------------------------------------------------------------------

def run_cycle(cfg: Dict[str, Any]) -> int:
    repo_cfg = cfg.get("repo") or {}
    path = repo_cfg.get("path")
    repo = Path(path).resolve() if path else _repo_root()
    remote = repo_cfg.get("remote", "origin")
    branch = repo_cfg.get("branch", "main")
    cycle_cfg = cfg.get("cycle") or {}
    fetch_on = cycle_cfg.get("fetch_on_cycle", True)
    pull_on = cycle_cfg.get("pull_on_cycle", False)
    report_only = cycle_cfg.get("report_only", True)
    log = logging.getLogger("repo_monitor")

    if not (repo / ".git").exists():
        log.error("No es un repositorio git: %s", repo)
        return 2

    _bitacora(cfg, "[REPO] Ciclo de monitoreo iniciado", ok=True)

    # Fetch
    if fetch_on:
        r = _git_fetch(repo, remote, cfg)
        if not r.get("ok"):
            log.error("Fetch falló: %s", r.get("stderr"))
            _bitacora(cfg, "[REPO] Ciclo: fetch fallido. %s" % (r.get("stderr") or "")[:120], ok=False)
            return 1

    # Estado
    head = _head_commit(repo)
    rem = _remote_commit(repo, remote, branch)
    cur = _current_branch(repo)
    status_r = _git_status_short(repo)
    exclude = (cfg.get("git") or {}).get("exclude_paths") or []
    filtered = _filter_status_lines(status_r.get("lines") or [], exclude) if status_r.get("ok") else []

    head_commit = head.get("commit", "?")[:8]
    remote_commit = rem.get("commit", "?")[:8] if rem.get("ok") else "?"
    has_remote_newer = rem.get("ok") and head.get("commit") != rem.get("commit")
    log.info(
        "Ciclo: branch=%s head=%s remote=%s has_update=%s changed=%s (filtered)",
        cur.get("branch", "?"),
        head_commit,
        remote_commit,
        has_remote_newer,
        len(filtered),
    )
    _bitacora(
        cfg,
        "[REPO] Ciclo finalizado: branch=%s head=%s remote=%s has_update=%s changed=%s"
        % (cur.get("branch", "?"), head_commit, remote_commit, has_remote_newer, len(filtered)),
        ok=True,
    )

    if pull_on and has_remote_newer and not report_only:
        on_err = (cfg.get("on_error") or {}).get("pull_fail") or {}
        retries = int(on_err.get("retries", 2))
        last = {}
        for _ in range(retries):
            last = _git(repo, "pull", "--no-edit", remote, branch, timeout_sec=60)
            if last.get("ok"):
                log.info("Pull OK")
                _bitacora(cfg, "[REPO] Ciclo: pull OK", ok=True)
                return 0
            time.sleep(2)
        log.error("Pull falló: %s", last.get("stderr"))
        _bitacora(cfg, "[REPO] Ciclo: pull fallido. %s" % (last.get("stderr") or "")[:120], ok=False)
        return 1
    return 0


# -----------------------------------------------------------------------------
# After-fix: add + commit + push (solo archivos no excluidos)
# -----------------------------------------------------------------------------

def run_after_fix(cfg: Dict[str, Any], message: Optional[str] = None) -> int:
    repo_cfg = cfg.get("repo") or {}
    path = repo_cfg.get("path")
    repo = Path(path).resolve() if path else _repo_root()
    remote = repo_cfg.get("remote", "origin")
    push_branch = repo_cfg.get("push_branch")
    af_cfg = cfg.get("after_fix") or {}
    allow_empty = af_cfg.get("allow_empty_commit", False)
    msg = message or af_cfg.get("default_message", "chore: auto-sync repo (monitor)")
    log = logging.getLogger("repo_monitor")

    if not (repo / ".git").exists():
        log.error("No es un repositorio git: %s", repo)
        return 2

    cur = _current_branch(repo)
    branch = push_branch or cur.get("branch", "main")
    _bitacora(cfg, "[REPO] After-fix: commit + push (rama %s)" % branch, ok=True)

    # Status y filtro
    status_r = _git_status_short(repo)
    if not status_r.get("ok"):
        log.error("Status falló: %s", status_r.get("stderr"))
        _bitacora(cfg, "[REPO] After-fix: status fallido", ok=False)
        return 1
    exclude = (cfg.get("git") or {}).get("exclude_paths") or []
    include = (cfg.get("git") or {}).get("include_paths_for_commit") or []
    lines = status_r.get("lines") or []
    filtered = _filter_status_lines(lines, exclude)
    if not filtered and not allow_empty:
        log.info("Sin cambios para commit (after-fix).")
        _bitacora(cfg, "[REPO] After-fix: sin cambios para commit", ok=True)
        return 0

    # Archivos a añadir: solo los que están en filtered (path en segunda columna)
    paths_to_add = []
    for ln in filtered:
        parts = ln.split(maxsplit=1)
        if len(parts) >= 2:
            paths_to_add.append(parts[1].strip())
    if include:
        paths_to_add = [p for p in paths_to_add if any(_path_matches(p, inc) for inc in include)]
    if not paths_to_add and not allow_empty:
        log.info("Sin archivos a añadir tras filtros.")
        _bitacora(cfg, "[REPO] After-fix: sin archivos tras filtros", ok=True)
        return 0

    # POT: evitar megacommits. Si son demasiados archivos, dividir por grupos.
    max_files = int(os.getenv("REPO_AFTERFIX_MAX_FILES", "25") or 25)
    groups = {}
    if len(paths_to_add) > max_files:
        for p in paths_to_add:
            p2 = p.replace("\\", "/")
            top = p2.split("/", 1)[0] if "/" in p2 else p2
            # agrupar módulos de forma estable
            if p2.startswith("modules/"):
                top = "modules"
            if p2.startswith("atlas_adapter/"):
                top = "atlas_adapter"
            if p2.startswith("brain/") or p2.startswith("training/") or p2.startswith("tests/"):
                top = "learning"
            groups.setdefault(top, []).append(p)
    else:
        groups = {"all": list(paths_to_add)}

    # commit(s)
    did_commit = False
    for gname, gpaths in groups.items():
        # add por grupo
        for p in gpaths:
            _git(repo, "add", p, timeout_sec=5)
        gmsg = msg if gname == "all" else f"{msg} ({gname})"
        r = _git(repo, "commit", "-m", gmsg, timeout_sec=15)
        if not r.get("ok"):
            if "nothing to commit" in (r.get("stdout") or "") + (r.get("stderr") or ""):
                continue
            log.error("Commit falló: %s %s", r.get("stdout"), r.get("stderr"))
            _bitacora(cfg, "[REPO] After-fix: commit fallido", ok=False)
            return 1
        did_commit = True
        # Notificación multicanal (dashboard si está disponible; fallback OPS Bus)
        try:
            bc = cfg.get("bitacora") or {}
            url = (bc.get("dashboard_url") or "").rstrip("/")
            if url:
                data = json.dumps({"message": "Repositorio actualizado: cambios guardados en Git.", "subsystem": "repo", "level": "info"}).encode("utf-8")
                req = urllib.request.Request(f"{url}/api/comms/test", data=data, method="POST", headers={"Content-Type": "application/json"})
                urllib.request.urlopen(req, timeout=3).read()
        except Exception:
            try:
                from modules.humanoid.comms.ops_bus import emit as ops_emit  # type: ignore

                ops_emit("repo", "Repositorio actualizado: cambios guardados en Git.", level="info")
            except Exception:
                pass

    if not did_commit and not allow_empty:
        log.info("Nada que commitear.")
        _bitacora(cfg, "[REPO] After-fix: nada que commitear", ok=True)
        return 0

    # push
    on_err = (cfg.get("on_error") or {}).get("push_fail") or {}
    retries = int(on_err.get("retries", 3))
    delay = int(on_err.get("delay_seconds", 10))
    for attempt in range(1, retries + 1):
        push_r = _git(repo, "push", remote, branch, timeout_sec=60)
        if push_r.get("ok"):
            log.info("Push OK a %s/%s", remote, branch)
            _bitacora(cfg, "[REPO] After-fix: push OK a %s/%s" % (remote, branch), ok=True)
            try:
                bc = cfg.get("bitacora") or {}
                url = (bc.get("dashboard_url") or "").rstrip("/")
                if url:
                    data = json.dumps({"message": "Cambios subidos a GitHub.", "subsystem": "repo", "level": "info"}).encode("utf-8")
                    req = urllib.request.Request(f"{url}/api/comms/test", data=data, method="POST", headers={"Content-Type": "application/json"})
                    urllib.request.urlopen(req, timeout=3).read()
            except Exception:
                try:
                    from modules.humanoid.comms.ops_bus import emit as ops_emit  # type: ignore

                    ops_emit("repo", "Cambios subidos a GitHub.", level="info")
                except Exception:
                    pass
            return 0
        log.warning("Push intento %s/%s: %s", attempt, retries, push_r.get("stderr"))
        if attempt < retries:
            time.sleep(delay)
    log.error("Push falló tras %s intentos.", retries)
    _bitacora(cfg, "[REPO] After-fix: push fallido tras reintentos", ok=False)
    return 1


def _path_matches(path: str, pattern: str) -> bool:
    path = path.replace("\\", "/")
    pattern = pattern.replace("\\", "/").rstrip("/")
    if "*" in pattern:
        return fnmatch.fnmatch(path, pattern)
    return path == pattern or path.startswith(pattern + "/")


# -----------------------------------------------------------------------------
# Status solo
# -----------------------------------------------------------------------------

def run_status_only(cfg: Dict[str, Any]) -> int:
    repo_cfg = cfg.get("repo") or {}
    path = repo_cfg.get("path")
    repo = Path(path).resolve() if path else _repo_root()
    remote = repo_cfg.get("remote", "origin")
    branch = repo_cfg.get("branch", "main")
    log = logging.getLogger("repo_monitor")

    if not (repo / ".git").exists():
        log.error("No es un repositorio git: %s", repo)
        return 2

    cur = _current_branch(repo)
    head = _head_commit(repo)
    rem = _remote_commit(repo, remote, branch)
    status_r = _git_status_short(repo)
    exclude = (cfg.get("git") or {}).get("exclude_paths") or []
    filtered = _filter_status_lines(status_r.get("lines") or [], exclude) if status_r.get("ok") else []

    summary = "branch=%s head=%s remote=%s changed=%s" % (
        cur.get("branch"),
        (head.get("commit") or "?")[:8],
        (rem.get("commit") or "?")[:8] if rem.get("ok") else "?",
        len(filtered),
    )
    log.info(summary)
    _bitacora(cfg, "[REPO] Status: " + summary, ok=True)
    return 0


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS PUSH — Monitoreo y actualización del repo.")
    parser.add_argument("--cycle", action="store_true", help="Un ciclo: fetch, status, opcional pull")
    parser.add_argument("--after-fix", action="store_true", help="Commit + push de cambios filtrados")
    parser.add_argument("--status-only", action="store_true", help="Solo mostrar estado")
    parser.add_argument("-m", "--message", type=str, default=None, help="Mensaje de commit (con --after-fix)")
    parser.add_argument("--config", type=str, default=None, help="Ruta a repo_monitor.yaml")
    args = parser.parse_args()

    if args.config:
        os.environ["REPO_MONITOR_CONFIG"] = args.config
    cfg = _load_config()
    _setup_logging(cfg)

    if args.after_fix:
        return run_after_fix(cfg, message=args.message)
    if args.status_only:
        return run_status_only(cfg)
    if args.cycle:
        return run_cycle(cfg)
    # Por defecto: un ciclo
    return run_cycle(cfg)


if __name__ == "__main__":
    sys.exit(main())
