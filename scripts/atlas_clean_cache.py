"""
ATLAS — Limpieza profunda de caché del sistema.

Limpia:
- __pycache__/ y archivos .pyc en todo el repo
- SQLite WAL y SHM files (.wal, .shm) que puedan estar corruptos o huerfanos
- Logs rotados antiguos (*.log.1, *.log.2, etc.) más antiguos de N días
- temp_models_cache/
- data/autonomy_tasks.db si es muy grande (solo trunca, no borra)
- Archivos de lock de git huérfanos (.git/index.lock)

Uso:
  python scripts/atlas_clean_cache.py             # modo seco (muestra qué haría)
  python scripts/atlas_clean_cache.py --execute   # ejecuta la limpieza
  python scripts/atlas_clean_cache.py --execute --logs-days 7   # logs > 7 días
  python scripts/atlas_clean_cache.py --execute --no-pycache    # sin __pycache__
  python scripts/atlas_clean_cache.py --execute --all           # todo incluyendo WAL activos

Seguridad:
- Por defecto es DRY RUN (solo muestra, no borra)
- Nunca borra datos de producción (SQLite bases de datos)
- Nunca borra logs actuales (solo rotados)
- Nunca borra .py o código fuente
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import List, NamedTuple


# ─── Config ───────────────────────────────────────────────────────────────────

REPO_ROOT = Path(__file__).resolve().parent.parent
LOG_DIR   = REPO_ROOT / "logs"
DATA_DIR  = REPO_ROOT / "data"

# Directorios donde limpiar __pycache__ y .pyc
SCAN_DIRS = [
    REPO_ROOT / "atlas_adapter",
    REPO_ROOT / "modules",
    REPO_ROOT / "core",
    REPO_ROOT / "brain",
    REPO_ROOT / "nexus",
    REPO_ROOT / "scripts",
    REPO_ROOT / "autonomous",
]

# Archivos de caché temporales seguros de borrar
TEMP_DIRS = [
    REPO_ROOT / "temp_models_cache",
    REPO_ROOT / ".mypy_cache",
    REPO_ROOT / ".pytest_cache",
    REPO_ROOT / ".ruff_cache",
]

# SQLite WAL/SHM — solo huérfanos (sin el .db correspondiente activo)
WAL_EXTENSIONS = {".wal", ".shm"}

# Patrones de logs rotados (seguros de borrar)
LOG_ROTATE_SUFFIXES = [f".{i}" for i in range(1, 20)] + [".bak", ".old"]

# ─── Tipos ────────────────────────────────────────────────────────────────────

class CleanAction(NamedTuple):
    category: str
    path: Path
    size_bytes: int
    reason: str


# ─── Helpers ──────────────────────────────────────────────────────────────────

def _size(p: Path) -> int:
    try:
        if p.is_file():
            return p.stat().st_size
        return sum(f.stat().st_size for f in p.rglob("*") if f.is_file())
    except Exception:
        return 0


def _fmt(b: int) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if b < 1024:
            return f"{b:.1f} {unit}"
        b //= 1024
    return f"{b:.1f} TB"


def _age_days(p: Path) -> float:
    try:
        return (time.time() - p.stat().st_mtime) / 86400
    except Exception:
        return 0.0


# ─── Recolectores ─────────────────────────────────────────────────────────────

def collect_pycache(actions: List[CleanAction]) -> None:
    for base in SCAN_DIRS:
        if not base.exists():
            continue
        for d in base.rglob("__pycache__"):
            if d.is_dir():
                actions.append(CleanAction("pycache", d, _size(d), "__pycache__ directory"))
        for f in base.rglob("*.pyc"):
            actions.append(CleanAction("pycache", f, _size(f), ".pyc compiled file"))


def collect_temp_dirs(actions: List[CleanAction]) -> None:
    for d in TEMP_DIRS:
        if d.exists() and d.is_dir():
            sz = _size(d)
            if sz > 0:
                actions.append(CleanAction("temp", d, sz, f"temporary cache dir ({_fmt(sz)})"))


def collect_sqlite_wal(actions: List[CleanAction], all_wal: bool = False) -> None:
    """Recolecta archivos WAL/SHM huérfanos (sin .db activo o si --all)."""
    for d in [LOG_DIR, DATA_DIR, REPO_ROOT]:
        if not d.exists():
            continue
        for f in d.glob("*.wal"):
            db = f.with_suffix("")
            if all_wal or not db.exists():
                actions.append(CleanAction("sqlite_wal", f, _size(f), "huérfano (sin .db)" if not db.exists() else "WAL file"))
        for f in d.glob("*.shm"):
            db = Path(str(f).replace(".shm", ""))
            if all_wal or not db.exists():
                actions.append(CleanAction("sqlite_wal", f, _size(f), "huérfano (sin .db)" if not db.exists() else "SHM file"))


def collect_old_logs(actions: List[CleanAction], max_age_days: float) -> None:
    """Logs rotados antiguos (*.log.1, *.log.bak, etc.)"""
    if not LOG_DIR.exists():
        return
    for f in LOG_DIR.iterdir():
        if not f.is_file():
            continue
        name = f.name
        is_rotated = any(name.endswith(suf) or f".log{suf}" in name for suf in LOG_ROTATE_SUFFIXES)
        if is_rotated and _age_days(f) > max_age_days:
            actions.append(CleanAction("old_log", f, _size(f), f"log rotado {_age_days(f):.0f} días"))


def collect_git_locks(actions: List[CleanAction]) -> None:
    """Archivos de lock huérfanos de git."""
    git_dir = REPO_ROOT / ".git"
    if not git_dir.exists():
        return
    for lock_name in ("index.lock", "HEAD.lock", "packed-refs.lock"):
        lock = git_dir / lock_name
        if lock.exists():
            age = _age_days(lock)
            if age > 0.01:  # > ~15 minutos
                actions.append(CleanAction("git_lock", lock, _size(lock), f"git lock huérfano {age:.1f} días"))


# ─── Ejecución ────────────────────────────────────────────────────────────────

def execute_clean(actions: List[CleanAction], dry_run: bool) -> dict:
    removed = 0
    freed   = 0
    errors  = 0

    for action in actions:
        try:
            if not dry_run:
                if action.path.is_dir():
                    shutil.rmtree(action.path, ignore_errors=True)
                elif action.path.is_file():
                    action.path.unlink(missing_ok=True)
            removed += 1
            freed   += action.size_bytes
        except Exception as e:
            errors += 1
            print(f"  ERROR: {action.path} — {e}", file=sys.stderr)

    return {"removed": removed, "freed_bytes": freed, "errors": errors}


# ─── Main ─────────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(
        description="ATLAS cache cleaner — por defecto modo DRY RUN (sin borrar nada)."
    )
    ap.add_argument("--execute",    action="store_true", help="Ejecutar limpieza real (sin esto, solo muestra).")
    ap.add_argument("--no-pycache", action="store_true", help="Omitir limpieza de __pycache__ y .pyc.")
    ap.add_argument("--no-temp",    action="store_true", help="Omitir temp_models_cache y similares.")
    ap.add_argument("--no-logs",    action="store_true", help="Omitir logs rotados.")
    ap.add_argument("--no-locks",   action="store_true", help="Omitir git locks.")
    ap.add_argument("--logs-days",  type=float, default=3.0, help="Logs rotados más viejos que N días (default: 3).")
    ap.add_argument("--all",        action="store_true", help="Incluir WAL activos (más agresivo).")
    args = ap.parse_args()

    dry_run = not args.execute
    now     = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")

    print(f"\n{'='*65}")
    print(f"  ATLAS Cache Cleaner — {now}")
    print(f"  Repo: {REPO_ROOT}")
    print(f"  Modo: {'DRY RUN (usa --execute para borrar)' if dry_run else '*** EJECUCIÓN REAL ***'}")
    print(f"{'='*65}\n")

    actions: List[CleanAction] = []

    if not args.no_pycache:
        collect_pycache(actions)
    if not args.no_temp:
        collect_temp_dirs(actions)
    collect_sqlite_wal(actions, all_wal=args.all)
    if not args.no_logs:
        collect_old_logs(actions, max_age_days=args.logs_days)
    if not args.no_locks:
        collect_git_locks(actions)

    if not actions:
        print("  ✅ Nada que limpiar. El repo está limpio.")
        return 0

    # Agrupar por categoría para el reporte
    by_cat: dict = {}
    for a in actions:
        by_cat.setdefault(a.category, []).append(a)

    labels = {
        "pycache":    "Python caché (__pycache__, .pyc)",
        "temp":       "Directorios temporales",
        "sqlite_wal": "SQLite WAL/SHM huérfanos",
        "old_log":    "Logs rotados antiguos",
        "git_lock":   "Git locks huérfanos",
    }

    total_size = sum(a.size_bytes for a in actions)
    print(f"  {'Categoría':<35} {'Archivos':>8}  {'Tamaño':>10}")
    print(f"  {'-'*55}")
    for cat, items in by_cat.items():
        label = labels.get(cat, cat)
        cat_size = sum(a.size_bytes for a in items)
        print(f"  {label:<35} {len(items):>8}  {_fmt(cat_size):>10}")
        if len(items) <= 5:
            for a in items:
                rel = a.path.relative_to(REPO_ROOT) if a.path.is_relative_to(REPO_ROOT) else a.path
                print(f"    >> {rel}  [{_fmt(a.size_bytes)}]  {a.reason}")

    print(f"  {'-'*55}")
    print(f"  {'TOTAL':<35} {len(actions):>8}  {_fmt(total_size):>10}")
    print()

    result = execute_clean(actions, dry_run=dry_run)

    if dry_run:
        print(f"  [DRY RUN] Se liberarían {_fmt(result['freed_bytes'])} en {result['removed']} elementos.")
        print(f"  → Ejecuta con --execute para aplicar.\n")
    else:
        icon = "✅" if result["errors"] == 0 else "⚠️ "
        print(f"  {icon} Limpieza completada: {result['removed']} eliminados, {_fmt(result['freed_bytes'])} liberados.")
        if result["errors"]:
            print(f"  ⚠️  {result['errors']} errores (ver arriba).")
        print()

    return 0 if result["errors"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
