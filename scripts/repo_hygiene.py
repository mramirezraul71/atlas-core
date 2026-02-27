"""
Repo Hygiene (ATLAS): evita que se acumulen artefactos en Git.

Objetivo:
- Nunca versionar evidencia/local: logs/, snapshots/, capturas de cámara.
- Detectar y des-trackear (git rm --cached) si por accidente quedaron trackeados.
- (Opcional) Commit + push automático, controlado por flags o variables de entorno.

Uso:
  python scripts/repo_hygiene.py --scan
  python scripts/repo_hygiene.py --fix
  python scripts/repo_hygiene.py --fix --commit
  python scripts/repo_hygiene.py --auto   # fix + commit + push (si habilitado)
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Tuple


FORBIDDEN_DIRS = ["logs", "snapshots"]
FORBIDDEN_GLOBS = [
    "camera_*.jpg",
    "camera_*.png",
    "*_camera_*.jpg",
    "*_camera_*.png",
    "direct_camera_*.jpg",
    "direct_camera_*.png",
    "proxy_camera_*.jpg",
    "proxy_camera_*.png",
    "vision_camera_*.jpg",
    "vision_camera_*.png",
]


def _repo_root() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT")
    if root:
        return Path(root).resolve()
    return Path(__file__).resolve().parent.parent


def _run_git(args: List[str], cwd: Path, timeout: int = 120) -> Tuple[int, str, str]:
    r = subprocess.run(
        ["git", *args],
        cwd=str(cwd),
        capture_output=True,
        text=True,
        timeout=timeout,
        env={**os.environ},
    )
    return r.returncode, r.stdout or "", r.stderr or ""


def _ops_emit(subsystem: str, message: str, level: str = "info", data: Dict | None = None) -> None:
    try:
        repo = _repo_root()
        if str(repo) not in sys.path:
            sys.path.insert(0, str(repo))
        from modules.humanoid.comms.ops_bus import emit as ops_emit

        ops_emit(subsystem, message, level=level, data=data or {}, evidence_path="")
    except Exception:
        # Silencioso: este script debe ser usable sin el bus.
        pass


def _ensure_gitignore_rules(repo: Path) -> Dict[str, int]:
    """
    Asegura reglas mínimas en .gitignore.
    No reordena ni borra contenido; solo agrega si falta.
    """
    path = repo / ".gitignore"
    added = 0
    if not path.exists():
        existing = ""
    else:
        existing = path.read_text(encoding="utf-8", errors="ignore")

    required = [
        "snapshots/**",
        "logs/**",
        "camera_*.jpg",
        "camera_*.png",
    ]
    missing = [r for r in required if r not in existing]
    if missing:
        with open(path, "a", encoding="utf-8") as f:
            if not existing.endswith("\n"):
                f.write("\n")
            f.write("\n# Repo Hygiene (ATLAS) - agregado automáticamente\n")
            for r in missing:
                f.write(r + "\n")
                added += 1
    return {"added_rules": added}


def _list_tracked(repo: Path) -> Dict[str, List[str]]:
    out: Dict[str, List[str]] = {"logs": [], "snapshots": [], "forbidden_globs": []}
    for d in FORBIDDEN_DIRS:
        code, stdout, _ = _run_git(["ls-files", "--", d], repo, timeout=120)
        if code == 0 and stdout.strip():
            out[d] = [ln.strip() for ln in stdout.splitlines() if ln.strip()]
    # globs sueltos
    code, stdout, _ = _run_git(["ls-files", "--", *FORBIDDEN_GLOBS], repo, timeout=120)
    if code == 0 and stdout.strip():
        out["forbidden_globs"] = [ln.strip() for ln in stdout.splitlines() if ln.strip()]
    return out


def _untrack(repo: Path, tracked: Dict[str, List[str]]) -> Dict[str, int]:
    removed = 0
    # Directorios completos
    for d in ("logs", "snapshots"):
        if tracked.get(d):
            code, _, err = _run_git(["rm", "-r", "--cached", "--", d], repo, timeout=300)
            if code != 0 and err:
                _ops_emit("repo_hygiene", f"git rm --cached {d} falló: {err[:200]}", level="warn")
            else:
                removed += len(tracked.get(d) or [])
    # Archivos por glob (en raíz u otros)
    for p in tracked.get("forbidden_globs") or []:
        code, _, err = _run_git(["rm", "--cached", "--", p], repo, timeout=120)
        if code == 0:
            removed += 1
        elif err:
            _ops_emit("repo_hygiene", f"git rm --cached {p} falló: {err[:200]}", level="warn")
    return {"removed_tracked": removed}


def _has_changes(repo: Path) -> bool:
    code, stdout, _ = _run_git(["status", "--porcelain=v1"], repo, timeout=60)
    return code == 0 and bool(stdout.strip())


def _commit(repo: Path, message: str) -> Dict[str, str]:
    # Asegura .gitignore en staging si cambió.
    _run_git(["add", "--", ".gitignore"], repo, timeout=60)
    code, stdout, err = _run_git(["commit", "-m", message], repo, timeout=120)
    return {"ok": "true" if code == 0 else "false", "stdout": stdout[-400:], "stderr": err[-400:]}


def _push(repo: Path) -> Dict[str, str]:
    # No interactivo: evita colgarse pidiendo credenciales.
    env = {**os.environ, "GIT_TERMINAL_PROMPT": "0"}
    r = subprocess.run(
        ["git", "push"],
        cwd=str(repo),
        capture_output=True,
        text=True,
        timeout=600,
        env=env,
    )
    return {"ok": "true" if r.returncode == 0 else "false", "stdout": (r.stdout or "")[-400:], "stderr": (r.stderr or "")[-400:]}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan", action="store_true", help="Solo reporta (no toca Git).")
    ap.add_argument("--fix", action="store_true", help="Aplica higiene: untrack + refuerza .gitignore.")
    ap.add_argument("--commit", action="store_true", help="Hace commit si hubo cambios.")
    ap.add_argument("--push", action="store_true", help="Hace push si hubo commit.")
    ap.add_argument("--auto", action="store_true", help="Equivalente a --fix --commit --push (si está permitido).")
    args = ap.parse_args()

    repo = _repo_root()
    t0 = time.perf_counter()

    allow_auto = os.getenv("REPO_HYGIENE_AUTO", "false").strip().lower() in ("1", "true", "yes")
    do_fix = args.fix or args.auto
    do_commit = args.commit or args.auto
    do_push = args.push or args.auto

    if args.auto and not allow_auto:
        _ops_emit("repo_hygiene", "AUTO deshabilitado. Set REPO_HYGIENE_AUTO=true para permitir --auto.", level="warn")
        # Degradar a scan (seguro)
        do_fix = False
        do_commit = False
        do_push = False

    info_ignore = _ensure_gitignore_rules(repo) if do_fix else {"added_rules": 0}
    tracked = _list_tracked(repo)

    counts = {
        "tracked_logs": len(tracked.get("logs") or []),
        "tracked_snapshots": len(tracked.get("snapshots") or []),
        "tracked_forbidden_globs": len(tracked.get("forbidden_globs") or []),
        "added_gitignore_rules": info_ignore.get("added_rules", 0),
    }

    if args.scan and not do_fix:
        ms = int((time.perf_counter() - t0) * 1000)
        _ops_emit("repo_hygiene", "Scan completado.", level="info", data={**counts, "ms": ms})
        print(counts)
        return 0

    removed = {"removed_tracked": 0}
    if do_fix:
        removed = _untrack(repo, tracked)

    changed = _has_changes(repo)
    ms = int((time.perf_counter() - t0) * 1000)
    _ops_emit(
        "repo_hygiene",
        "Higiene aplicada." if do_fix else "Scan aplicado (sin fix).",
        level="info",
        data={**counts, **removed, "changed": changed, "ms": ms},
    )

    if do_commit and changed:
        now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%MZ")
        msg = f"chore: repo hygiene (logs/snapshots) [{now}]"
        c = _commit(repo, msg)
        _ops_emit("repo_hygiene", "Commit higiene OK." if c.get("ok") == "true" else "Commit higiene FAIL.", level="info" if c.get("ok") == "true" else "error", data=c)
        if c.get("ok") != "true":
            return 2
        if do_push:
            p = _push(repo)
            _ops_emit("repo_hygiene", "Push higiene OK." if p.get("ok") == "true" else "Push higiene FAIL.", level="info" if p.get("ok") == "true" else "error", data=p)
            return 0 if p.get("ok") == "true" else 3

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

