"""Procedencia efectiva de ``atlas_core`` en runtime (evidencia estructurada, sin mover paquetes).

Política: una sola función de resolución; comportamiento conservador (no declarar ``ok`` sin
coincidencia verificable con el árbol ``<repo>/atlas_core`` derivado de ``atlas_code_quant``).
"""

from __future__ import annotations

import importlib
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal

CanonicalStatus = Literal["ok", "missing", "ambiguous", "error", "partial", "unknown"]
SourceKind = Literal["repo_local", "sibling", "external", "unknown"]

_LAST: dict[str, Any] | None = None


def repo_root_from_atlas_code_quant() -> Path:
    """Raíz del repositorio: padre del paquete ``atlas_code_quant``."""
    import atlas_code_quant

    return Path(atlas_code_quant.__file__).resolve().parent.parent


def _canonical_atlas_core_dir(repo_root: Path) -> Path:
    """Única ruta canónica del núcleo en este repo: ``<repo>/atlas_core`` (no el shim ``atlas_code_quant/atlas_core``)."""
    return (repo_root / "atlas_core").resolve()


def _extra_atlas_core_package_dirs(repo_root: Path) -> list[Path]:
    """Otras carpetas ``<repo>/*/atlas_core`` con ``__init__.py`` (un nivel bajo la raíz; sin rglob profundo).

    Excluye la canónica ``<repo>/atlas_core`` y el shim ``atlas_code_quant/atlas_core``.
    """
    canonical = _canonical_atlas_core_dir(repo_root)
    shim_pkg = (repo_root / "atlas_code_quant" / "atlas_core").resolve()
    extra: list[Path] = []
    try:
        globs = list(repo_root.glob("*/atlas_core/__init__.py"))
    except OSError:
        return extra
    for init in globs:
        try:
            parent = init.resolve().parent
        except OSError:
            continue
        if parent in (canonical, shim_pkg) or parent in extra:
            continue
        extra.append(parent)
    return extra


def _resolved_atlas_core_package_dir() -> Path | None:
    mod = sys.modules.get("atlas_core")
    if mod is None:
        return None
    f = getattr(mod, "__file__", None)
    if f:
        return Path(f).resolve().parent
    paths = getattr(mod, "__path__", None)
    if paths:
        try:
            first = next(iter(paths))
        except StopIteration:
            return None
        return Path(first).resolve()
    return None


def resolve_atlas_core_provenance(*, ensure_shim_loaded: bool = True) -> dict[str, Any]:
    """Resuelve evidencia de qué copia de ``atlas_core`` usa el intérprete actual.

    ``ensure_shim_loaded`` (default True): importa ``atlas_code_quant.atlas_core`` para alinear
    con el runtime típico de Quant (shim canónico). Desactivar solo en tests aislados.
    """
    global _LAST
    ts = datetime.now(timezone.utc).isoformat()
    notes: list[str] = []

    try:
        repo_root = repo_root_from_atlas_code_quant()
    except Exception as exc:  # pragma: no cover - defensivo
        out = {
            "resolved_path": None,
            "exists": False,
            "importable": False,
            "source_kind": "unknown",
            "canonical_status": "error",
            "timestamp": ts,
            "notes": [f"repo_root_failed:{type(exc).__name__}:{exc}"],
        }
        _LAST = out
        return out

    canonical_dir = _canonical_atlas_core_dir(repo_root)
    canonical_init = canonical_dir / "__init__.py"
    exists = canonical_init.is_file()

    extra_trees = _extra_atlas_core_package_dirs(repo_root)
    if extra_trees:
        notes.append("extra_atlas_core_dirs_on_disk:" + ",".join(str(p) for p in extra_trees[:6]))

    if ensure_shim_loaded:
        try:
            importlib.import_module("atlas_code_quant.atlas_core")
        except Exception as exc:
            notes.append(f"shim_import_failed:{type(exc).__name__}:{exc}")

    importable = False
    import_error: str | None = None
    try:
        importlib.import_module("atlas_core")
        importable = True
    except Exception as exc:
        import_error = f"{type(exc).__name__}:{exc}"
        notes.append(f"atlas_core_import_failed:{import_error}")

    resolved = _resolved_atlas_core_package_dir()
    resolved_str = str(resolved) if resolved is not None else None

    if not exists:
        st: CanonicalStatus = "missing"
        if importable and resolved is not None:
            notes.append("canonical_tree_missing_under_repo_root_but_atlas_core_loaded")
            st = "ambiguous"
        out = {
            "resolved_path": resolved_str or str(canonical_dir),
            "exists": False,
            "importable": importable,
            "source_kind": "unknown",
            "canonical_status": st,
            "timestamp": ts,
            "notes": notes[:8],
        }
        _LAST = out
        return out

    if not importable or resolved is None:
        out = {
            "resolved_path": resolved_str or str(canonical_dir),
            "exists": True,
            "importable": importable,
            "source_kind": "unknown",
            "canonical_status": "partial",
            "timestamp": ts,
            "notes": notes[:8],
        }
        _LAST = out
        return out

    same = resolved == canonical_dir

    def _is_under(root: Path, p: Path) -> bool:
        try:
            p.relative_to(root)
            return True
        except ValueError:
            return False

    under_repo = _is_under(repo_root, resolved)

    if extra_trees and same:
        # Coincide el runtime con el canónico declarado, pero hay otras copias bajo el repo
        out = {
            "resolved_path": resolved_str,
            "exists": True,
            "importable": True,
            "source_kind": "repo_local",
            "canonical_status": "ambiguous",
            "timestamp": ts,
            "notes": notes[:8],
        }
        _LAST = out
        return out

    if same:
        out = {
            "resolved_path": resolved_str,
            "exists": True,
            "importable": True,
            "source_kind": "repo_local",
            "canonical_status": "ok",
            "timestamp": ts,
            "notes": notes[:8],
        }
        _LAST = out
        return out

    # Cargado desde otra ruta que la canónica esperada
    if under_repo and resolved != canonical_dir:
        sk: SourceKind = "sibling"
    elif not under_repo:
        sk = "external"
    else:
        sk = "unknown"
    notes.append(f"package_dir_mismatch:expected={canonical_dir}:got={resolved}")
    out = {
        "resolved_path": resolved_str,
        "exists": True,
        "importable": True,
        "source_kind": sk,
        "canonical_status": "ambiguous",
        "timestamp": ts,
        "notes": notes[:8],
    }
    _LAST = out
    return out


def get_last_atlas_core_provenance() -> dict[str, Any] | None:
    """Último dict devuelto por ``resolve_atlas_core_provenance`` (útil en tests y auditoría)."""
    return dict(_LAST) if isinstance(_LAST, dict) else None


def attach_atlas_core_provenance_to_plan(plan: dict[str, Any]) -> None:
    """Adjunta evidencia al dict de session plan (mutación in-place)."""
    plan["atlas_core_provenance"] = resolve_atlas_core_provenance()
