"""Scaffold generator: build directory tree and files under allowed path."""
from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Any, Dict, List, Optional

from .runbook import generate_runbook
from .templates import get_template


def _allowed_base() -> str:
    base = os.getenv("ATLAS_SCAFFOLD_BASE") or os.getenv("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH").split(",")[0].strip()
    return base.rstrip("\\/")


def _sanitize_name(name: str) -> str:
    return re.sub(r"[^\w\-]", "_", name).strip("_") or "app"


def generate(
    app_type: str,
    name: str,
    options: Optional[Dict[str, Any]] = None,
    fs_controller: Any = None,
) -> Dict[str, Any]:
    """
    Generate project structure. Uses FileSystemController if provided (allowed path).
    Returns {ok, tree, files_created, runbook_path, base_path, error}.
    """
    options = options or {}
    app_type = (app_type or "fastapi").lower().strip()
    name = _sanitize_name(name or "app")
    base = _allowed_base()
    subdir = options.get("subdir") or "_generated"
    base_path = str(Path(base) / subdir / name)

    if fs_controller is None:
        try:
            from modules.humanoid import get_humanoid_kernel
            k = get_humanoid_kernel()
            hands = k.registry.get("hands")
            fs_controller = getattr(hands, "fs", None) or getattr(hands, "fs_controller", None)
        except Exception:
            pass
    if fs_controller is None:
        try:
            from modules.humanoid.hands.fs_controller import FileSystemController
            fs_controller = FileSystemController()
        except Exception:
            return {"ok": False, "tree": [], "files_created": [], "runbook_path": "", "base_path": base_path, "error": "FileSystemController not available"}

    template = get_template(app_type)
    content_map = {k: v.replace("{{name}}", name) for k, v in template.items()}

    tree = []
    files_created = []

    # Ensure base dir
    r = fs_controller.ensure_dir(base_path)
    if not r.get("ok"):
        return {"ok": False, "tree": [], "files_created": [], "runbook_path": "", "base_path": base_path, "error": r.get("error")}

    for rel_path, content in content_map.items():
        full_path = str(Path(base_path) / rel_path)
        parent = str(Path(full_path).parent)
        dir_r = fs_controller.ensure_dir(parent)
        if not dir_r.get("ok"):
            continue
        w = fs_controller.write_text(full_path, content)
        if w.get("ok"):
            tree.append(rel_path)
            files_created.append(full_path)

    runbook_content = generate_runbook(app_type, name, base_path)
    runbook_path = str(Path(base_path) / "RUNBOOK.md")
    w_runbook = fs_controller.write_text(runbook_path, runbook_content)
    if w_runbook.get("ok"):
        tree.append("RUNBOOK.md")
        files_created.append(runbook_path)

    try:
        from modules.humanoid.memory_engine import store_artifact, ensure_thread
        thread_id = ensure_thread(None, f"scaffold:{name}")
        for fp in files_created:
            store_artifact(None, None, "scaffold_file", fp, "")
    except Exception:
        pass

    return {
        "ok": True,
        "tree": tree,
        "files_created": files_created,
        "runbook_path": runbook_path,
        "base_path": base_path,
        "error": None,
    }
