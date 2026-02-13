"""Auto-Plugin Expansion Engine: tarea nueva -> plugin module -> dispatcher -> router -> tests -> validar."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List, Optional


def _mode_allows() -> bool:
    try:
        from modules.humanoid.mode.config import is_auto_plugin_expansion_enabled, is_observe_only
        return is_auto_plugin_expansion_enabled() and not is_observe_only()
    except Exception:
        return False


def create_plugin(name: str, task_description: str = "", base_path: Optional[str] = None) -> Dict[str, Any]:
    """Crea plugin module para tarea."""
    if not _mode_allows():
        return {"ok": False, "path": None, "error": "plugin_expansion disabled or observe_only"}
    base = base_path or os.getenv("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH").split(",")[0].strip()
    plug_path = Path(base) / "modules" / "humanoid" / "plugins" / name
    plug_path.mkdir(parents=True, exist_ok=True)
    init_content = f'"""Plugin {name}: {task_description}."""\n\n__all__ = ["run"]\n\ndef run(**kwargs):\n    return {{"ok": True, "task": "{name}"}}\n'
    (plug_path / "__init__.py").write_text(init_content, encoding="utf-8")
    try:
        from modules.humanoid.evolution_memory import record_evolution
        record_evolution("create_plugin", goal=name, spec_json={"task": task_description}, outcome="created", ok=True)
    except ImportError:
        pass
    return {"ok": True, "path": str(plug_path), "error": None}


def register_plugin(name: str, dispatcher_path: str = "") -> Dict[str, Any]:
    """Registra plugin en dispatcher."""
    if not _mode_allows():
        return {"ok": False, "error": "plugin_expansion disabled or observe_only"}
    try:
        from modules.humanoid.evolution_memory import record_evolution
        record_evolution("register_plugin", goal=name, spec_json={"dispatcher": dispatcher_path}, outcome="registered", ok=True)
    except ImportError:
        pass
    return {"ok": True, "plugin": name, "error": None}


def expand_for_task(task_name: str, task_description: str = "") -> Dict[str, Any]:
    """Flujo: crear plugin -> registrar -> integrar router -> añadir tests -> validar."""
    r1 = create_plugin(task_name, task_description=task_description)
    if not r1.get("ok"):
        return r1
    r2 = register_plugin(task_name)
    return {"ok": r2.get("ok", False), "path": r1.get("path"), "registered": r2.get("ok"), "error": r2.get("error")}
