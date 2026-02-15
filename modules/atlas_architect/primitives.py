from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict


def _repo_root() -> Path:
    root = (os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or "").strip()
    if root:
        return Path(root).resolve()
    return Path(__file__).resolve().parents[2]


def create_environment(project_name: str) -> Dict[str, Any]:
    """
    Genera un entorno virtual aislado para nuevos proyectos.
    Por defecto crea: apps/<project_name>/.venv
    """
    name = (project_name or "").strip().replace(" ", "_")
    if not name:
        return {"ok": False, "error": "missing project_name"}
    root = _repo_root()
    proj = (root / "apps" / name).resolve()
    proj.mkdir(parents=True, exist_ok=True)
    try:
        from modules.atlas_architect.agent import AtlasArchitect

        arch = AtlasArchitect(repo_root=root)
        v = arch.venv.ensure_venv(proj, venv_name=".venv")
        return {"ok": bool(v.ok), "project_dir": str(proj), "venv_dir": v.venv_dir, "python_exe": v.python_exe, "error": v.error}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200], "project_dir": str(proj)}


def patch_code(file: str, pattern: str, replacement: str, *, pattern_is_regex: bool = False) -> Dict[str, Any]:
    """Edición atómica (regex o literal) con diff unificado."""
    p = Path(file).resolve()
    try:
        from modules.atlas_architect.atomic_patcher import AtomicPatcher, RegexReplace
        import re

        rx = pattern if pattern_is_regex else re.escape(pattern)
        ops = [RegexReplace(pattern=rx, repl=replacement, count=1)]
        patcher = AtomicPatcher()
        before, after, diff = patcher.patch_file_preview(p, ops)
        # Aplicar atómicamente
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(after, encoding="utf-8")
        return {"ok": True, "file": str(p), "diff": diff[:12000]}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200], "file": str(p)}


def run_debug(script_path: str, *, max_attempts: int = 3, cwd: str = "") -> Dict[str, Any]:
    """
    Bucle de ejecución con captura de stderr para auto-reparación (SelfHealingLoop).
    Ejecuta: python <script_path>
    """
    p = Path(script_path).resolve()
    if not p.exists():
        return {"ok": False, "error": "script_not_found", "script_path": str(p)}
    try:
        from modules.atlas_architect.agent import AtlasArchitect
        root = _repo_root()
        arch = AtlasArchitect(repo_root=root)
        cmd = f'python "{str(p)}"'
        out = arch.healer.heal_command(cmd, max_attempts=int(max_attempts or 3), governed=False, cwd=Path(cwd).resolve() if cwd else None)
        return {"ok": bool(out.get("ok")), "result": out}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200]}


def generate_docs(app_context: Dict[str, Any]) -> Dict[str, Any]:
    """
    Genera documentación automática (determinista) de contexto de app: README/REPORTE.
    """
    root = _repo_root()
    out_dir = root / "docs" / "auto"
    out_dir.mkdir(parents=True, exist_ok=True)
    name = (app_context or {}).get("name") or "app"
    p = out_dir / f"AUTO_DOC_{str(name).replace(' ', '_')}.md"
    try:
        lines = []
        lines.append(f"## Documento automático — {name}")
        lines.append("")
        lines.append("### Contexto")
        for k, v in (app_context or {}).items():
            if isinstance(v, (dict, list)):
                continue
            lines.append(f"- **{k}**: {str(v)[:300]}")
        lines.append("")
        lines.append("### Arquitectura (si existe)")
        try:
            from modules.atlas_architect.agent import AtlasArchitect
            arch = AtlasArchitect(repo_root=root)
            idx = arch.index_architecture()
            lines.append(f"- Index: `{idx.get('path')}`")
        except Exception:
            lines.append("- Index: no disponible")
        lines.append("")
        p.write_text("\n".join(lines), encoding="utf-8")
        return {"ok": True, "path": str(p)}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200]}

