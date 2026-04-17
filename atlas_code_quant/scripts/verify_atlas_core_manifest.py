#!/usr/bin/env python3
"""Verifica que los módulos listados en atlas_core/MANIFEST.json importan sin error.

Uso (desde la raíz del repo):
  python atlas_code_quant/scripts/verify_atlas_core_manifest.py

Código de salida: 0 si todo OK, 1 si algún import falla, 2 si falta el manifest.
"""
from __future__ import annotations

import importlib
import json
import sys
from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def main() -> int:
    root = _repo_root()
    manifest_path = root / "atlas_core" / "MANIFEST.json"
    if not manifest_path.is_file():
        print(f"MANIFEST missing: {manifest_path}", file=sys.stderr)
        return 2
    try:
        data = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"MANIFEST unreadable: {exc}", file=sys.stderr)
        return 2
    modules = data.get("modules")
    if not isinstance(modules, list) or not modules:
        print("MANIFEST.modules must be a non-empty list", file=sys.stderr)
        return 2

    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

    failures: list[tuple[str, str]] = []
    for name in modules:
        if not isinstance(name, str) or not name.strip():
            failures.append((repr(name), "invalid module name"))
            continue
        mod = name.strip()
        try:
            importlib.import_module(mod)
        except Exception as exc:
            failures.append((mod, str(exc)))

    if failures:
        for mod, err in failures:
            print(f"IMPORT_FAIL {mod}: {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
