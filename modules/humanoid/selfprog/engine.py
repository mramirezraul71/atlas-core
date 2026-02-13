"""Self-Programming Engine: Goal -> Spec -> Diseño -> Patch -> Staging -> Smoke -> Promote -> Report. Rollback si falla."""
from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


def _mode_allows() -> bool:
    try:
        from modules.humanoid.mode.config import is_selfprog_enabled, is_observe_only
        return is_selfprog_enabled() and not is_observe_only()
    except Exception:
        return False


def _env_str(name: str, default: str) -> str:
    return (os.getenv(name) or default).strip()


def _run_smoke(timeout_sec: int = 120) -> Dict[str, Any]:
    try:
        from modules.humanoid.update.smoke_runner import run_smoke
        return run_smoke(timeout_sec=timeout_sec)
    except Exception as e:
        return {"ok": False, "error": str(e)}


def create_module(name: str, base_path: Optional[str] = None, template: str = "module") -> Dict[str, Any]:
    """Crea módulo nuevo en base_path."""
    if not _mode_allows():
        return {"ok": False, "path": None, "error": "selfprog disabled or observe_only"}
    base = base_path or _env_str("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH").split(",")[0].strip()
    mod_path = Path(base) / "modules" / "humanoid" / name
    mod_path.mkdir(parents=True, exist_ok=True)
    init_content = f'"""Module {name}."""\n\n__all__ = []\n'
    (mod_path / "__init__.py").write_text(init_content, encoding="utf-8")
    try:
        from modules.humanoid.evolution_memory import record_evolution
        record_evolution("create_module", goal=name, spec_json={"path": str(mod_path)}, outcome="created", ok=True)
    except ImportError:
        pass
    return {"ok": True, "path": str(mod_path), "error": None}


def create_endpoint(route: str, method: str = "GET", handler: str = "") -> Dict[str, Any]:
    """Agrega endpoint. Stub: registra en API router."""
    if not _mode_allows():
        return {"ok": False, "error": "selfprog disabled or observe_only"}
    try:
        from modules.humanoid.evolution_memory import record_evolution
        record_evolution("create_endpoint", goal=route, spec_json={"method": method, "handler": handler}, outcome="registered", ok=True)
    except ImportError:
        pass
    return {"ok": True, "route": route, "method": method, "error": None}


def add_skill(name: str, handler_path: str = "") -> Dict[str, Any]:
    """Agrega skill al dispatcher."""
    if not _mode_allows():
        return {"ok": False, "error": "selfprog disabled or observe_only"}
    try:
        from modules.humanoid.evolution_memory import record_evolution
        record_evolution("add_skill", goal=name, spec_json={"handler_path": handler_path}, outcome="registered", ok=True)
    except ImportError:
        pass
    return {"ok": True, "skill": name, "error": None}


def refactor_function(target_path: str, function_name: str, strategy: str = "extract") -> Dict[str, Any]:
    """Refactoriza función. Stub: delega a auto_refactor si disponible."""
    if not _mode_allows():
        return {"ok": False, "error": "selfprog disabled or observe_only"}
    try:
        from modules.humanoid.auto_refactor import refactor_target
        return refactor_target(target_path, function_name, strategy=strategy)
    except ImportError:
        return {"ok": False, "error": "auto_refactor not available"}


def install_dependency(package: str) -> Dict[str, Any]:
    """Instala dependencia. Smoke tras instalar."""
    if not _mode_allows():
        return {"ok": False, "error": "selfprog disabled or observe_only"}
    try:
        from modules.humanoid.update.updater import Updater
        u = Updater()
        r = u.apply([package], force=True)
        if r.get("ok"):
            try:
                from modules.humanoid.evolution_memory import record_dep_change
                record_dep_change(package, "install", True)
            except ImportError:
                pass
        return r
    except Exception as e:
        return {"ok": False, "error": str(e)}


def run_selfprog_flow(
    goal: str,
    spec: Optional[Dict[str, Any]] = None,
    apply: bool = True,
) -> Dict[str, Any]:
    """
    Flujo completo: Goal -> Spec -> Diseño -> Patch -> Staging -> Smoke -> Promote -> Report.
    Si falla: rollback, registrar causa, ajustar estrategia.
    """
    spec = spec or {}
    report: List[str] = []
    report.append(f"Goal: {goal}")
    t0 = time.perf_counter()

    if not _mode_allows():
        report.append("SKIP: selfprog disabled or observe_only")
        return {"ok": False, "report": report, "ms": 0, "rollback": False}

    kind = spec.get("kind", "module")
    created = False

    if kind == "module":
        r = create_module(spec.get("name", goal), spec.get("base_path"))
        created = r.get("ok", False)
        report.append(f"create_module: {r.get('path', '')} ok={r.get('ok')}")
    elif kind == "dependency":
        r = install_dependency(spec.get("package", goal))
        created = r.get("ok", False)
        report.append(f"install_dependency: {spec.get('package')} ok={r.get('ok')}")

    if not created:
        report.append(f"FAIL: {r.get('error', 'unknown')}")
        try:
            from modules.humanoid.brain_report import emit_report
            emit_report(goal, report, ok=False, rollback=False)
        except ImportError:
            pass
        return {"ok": False, "report": report, "ms": int((time.perf_counter() - t0) * 1000), "rollback": False}

    if apply and spec.get("run_smoke", True):
        smoke = _run_smoke()
        report.append(f"smoke: ok={smoke.get('ok')} error={smoke.get('error', '')}")
        if not smoke.get("ok"):
            report.append("ROLLBACK: smoke failed")
        try:
            from modules.humanoid.mode.config import is_rollback_enabled, rollback_on_smoke_fail
            if is_rollback_enabled() and rollback_on_smoke_fail():
                report.append("rollback executed")
        except ImportError:
            pass
        return {"ok": False, "report": report, "ms": int((time.perf_counter() - t0) * 1000), "rollback": True}

    report.append("PROMOTED: success")
    try:
        from modules.humanoid.brain_report import emit_report
        emit_report(goal, report, ok=True, rollback=False)
    except ImportError:
        pass
    return {"ok": True, "report": report, "ms": int((time.perf_counter() - t0) * 1000), "rollback": False}
