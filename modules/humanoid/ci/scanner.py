"""Repo scanner: deterministic checks (no LLM)."""
from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Any, Dict, List

MAX_FILE_KB = 400
REPO_ROOT = Path(os.getenv("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH")).resolve()
SKIP_DIRS = {".git", "node_modules", "__pycache__", ".venv", "venv", "env"}


def _iter_files(root: Path, ext: str = "") -> List[Path]:
    out: List[Path] = []
    try:
        for p in root.rglob("*"):
            if not p.is_file():
                continue
            if any(s in p.parts for s in SKIP_DIRS):
                continue
            if ext and p.suffix != ext:
                continue
            try:
                if not p.resolve().is_relative_to(root):
                    continue
            except (ValueError, OSError):
                continue
            out.append(p)
    except Exception:
        pass
    return out


def scan_repo(scope: str = "repo", max_items: int = 50) -> Dict[str, Any]:
    """Deterministic repo scan. Returns {ok, findings: [{kind, path, detail, ...}]}."""
    findings: List[Dict[str, Any]] = []
    root = REPO_ROOT

    # Large files
    for p in _iter_files(root)[:500]:
        try:
            size_kb = p.stat().st_size / 1024
            if size_kb > MAX_FILE_KB:
                findings.append({"kind": "large_file", "path": str(p.relative_to(root)), "detail": f"{size_kb:.0f} KB", "score": min(1.0, size_kb / 1000), "autofix_allowed": False})
        except (OSError, ValueError):
            pass
        if len(findings) >= max_items:
            break

    # TODO/FIXME (aggregate)
    todo_count = 0
    for p in _iter_files(root, ".py")[:200]:
        try:
            text = p.read_text(encoding="utf-8", errors="ignore")
            todo_count += len(re.findall(r"#\s*(TODO|FIXME|XXX)[^\n]*", text, re.IGNORECASE))
        except Exception:
            pass
    if todo_count > 0:
        findings.append({"kind": "todo_count", "path": "repo", "detail": f"{todo_count} TODO/FIXME in .py", "score": 0.3, "autofix_allowed": False})

    # PowerShell without param block (heuristic)
    for p in _iter_files(root, ".ps1")[:50]:
        if len(findings) >= max_items:
            break
        try:
            text = p.read_text(encoding="utf-8", errors="ignore")
            if "param(" not in text and "Param(" not in text and text.strip():
                findings.append({"kind": "ps1_no_param", "path": str(p.relative_to(root)), "detail": "No param() block", "score": 0.2, "autofix_allowed": True})
        except Exception:
            pass

    # .env.example vs atlas.env keys (config env missing)
    env_example = root / "config" / ".env.example"
    if env_example.exists() and scope in ("repo", "all"):
        try:
            keys = set()
            for line in env_example.read_text(encoding="utf-8", errors="ignore").splitlines():
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    keys.add(line.split("=")[0].strip())
            if keys and len(findings) < max_items:
                findings.append({"kind": "config_env_keys", "path": "config/.env.example", "detail": f"{len(keys)} keys documented", "score": 0.1, "autofix_allowed": False})
        except Exception:
            pass

    # Dedupe by kind+path
    seen = set()
    unique = []
    for f in findings[:max_items]:
        k = (f.get("kind"), f.get("path"))
        if k not in seen:
            seen.add(k)
            unique.append(f)
    return {"ok": True, "findings": unique}
