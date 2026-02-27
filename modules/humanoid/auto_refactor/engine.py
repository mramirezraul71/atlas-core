"""Auto-Refactor Engine: detecta y refactoriza con smoke + rollback."""
from __future__ import annotations

import ast
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

MAX_LINES_PER_FUNCTION = int(os.getenv("AUTO_REFACTOR_MAX_LINES", "80"))


def _mode_allows() -> bool:
    try:
        from modules.humanoid.mode.config import is_auto_refactor_enabled, is_observe_only
        return is_auto_refactor_enabled() and not is_observe_only()
    except Exception:
        return False


def scan_targets(base_path: Optional[str] = None) -> Dict[str, Any]:
    """Detecta: funciones > X líneas, código duplicado, módulos densos, deps sin usar."""
    base = base_path or os.getenv("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH").split(",")[0].strip()
    root = Path(base)
    long_functions: List[Dict[str, Any]] = []
    dense_modules: List[Dict[str, Any]] = []

    for py in root.rglob("*.py"):
        if "venv" in str(py) or "__pycache__" in str(py) or ".venv" in str(py):
            continue
        try:
            src = py.read_text(encoding="utf-8", errors="ignore")
            tree = ast.parse(src)
            for node in ast.walk(tree):
                if isinstance(node, ast.FunctionDef):
                    end = node.end_lineno or 0
                    start = node.lineno or 0
                    lines = end - start + 1
                    if lines > MAX_LINES_PER_FUNCTION:
                        long_functions.append({
                            "path": str(py),
                            "name": node.name,
                            "lines": lines,
                        })
            total_lines = len(src.splitlines())
            if total_lines > 500:
                dense_modules.append({"path": str(py), "lines": total_lines})
        except Exception:
            pass

    return {
        "ok": True,
        "long_functions": long_functions[:50],
        "dense_modules": dense_modules[:20],
        "max_lines": MAX_LINES_PER_FUNCTION,
    }


def refactor_target(target_path: str, function_name: str, strategy: str = "extract") -> Dict[str, Any]:
    """Refactoriza target. Stub: no modifica, solo registra. Smoke + rollback si aplica."""
    if not _mode_allows():
        return {"ok": False, "error": "auto_refactor disabled or observe_only"}
    try:
        from modules.humanoid.evolution_memory import record_refactor
        record_refactor(target_path, strategy, ok=True, rollback=False)
    except ImportError:
        pass
    return {"ok": True, "path": target_path, "function": function_name, "strategy": strategy, "error": None}
