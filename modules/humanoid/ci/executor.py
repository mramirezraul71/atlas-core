"""Execute only allowed CI actions (safe mode)."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List

from .policy_gate import can_autofix, _ci_autofix_limit

REPO_ROOT = Path(os.getenv("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH")).resolve()


def apply_autofix(item: Dict[str, Any]) -> Dict[str, Any]:
    """Apply a single safe autofix. Returns {ok, path, error}."""
    action = item.get("action") or ""
    path = item.get("path") or ""
    if not can_autofix(action):
        return {"ok": False, "path": path, "error": "policy denied or not safe"}
    full = REPO_ROOT / path
    if not full.exists():
        return {"ok": False, "path": path, "error": "file not found"}
    if action == "add_param_defaults" and path.endswith(".ps1"):
        try:
            text = full.read_text(encoding="utf-8", errors="replace")
            if "param(" in text or "Param(" in text:
                return {"ok": True, "path": path, "error": None}
            insert = "param(\n    [Parameter(Mandatory=$false)][string]$ParamName = \"default\"\n)\n"
            if text.lstrip().startswith("#"):
                idx = text.find("\n") + 1
                new_text = text[:idx] + insert + text[idx:]
            else:
                new_text = insert + text
            full.write_text(new_text, encoding="utf-8")
            return {"ok": True, "path": path, "error": None}
        except Exception as e:
            return {"ok": False, "path": path, "error": str(e)}
    return {"ok": False, "path": path, "error": f"unsupported action: {action}"}


def execute_plan(plan: Dict[str, Any], max_auto: int = None) -> Dict[str, Any]:
    """Run auto_executed items only. Returns {ok, executed: [...], errors: [...]}."""
    max_auto = max_auto or _ci_autofix_limit()
    executed: List[Dict[str, Any]] = []
    errors: List[str] = []
    for item in (plan.get("auto_executed") or [])[:max_auto]:
        r = apply_autofix(item)
        if r.get("ok"):
            executed.append({"item_id": item.get("item_id"), "path": r.get("path")})
        else:
            errors.append(r.get("error") or "unknown")
    return {"ok": len(errors) == 0, "executed": executed, "errors": errors}
