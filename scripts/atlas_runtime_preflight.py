#!/usr/bin/env python3
"""ATLAS runtime preflight for core PUSH/NEXUS/Robot operation."""
from __future__ import annotations

import argparse
import importlib
import json
import platform
import sys
from pathlib import Path


REQUIRED_IMPORTS = [
    "fastapi",
    "uvicorn",
    "requests",
    "psutil",
    "prometheus_client",
    "yaml",
    "pydantic",
    "modules.humanoid.metrics",
]

OPTIONAL_IMPORTS = [
    "autonomous.telemetry.metrics_aggregator",
]


def _check_import(module_name: str) -> dict:
    try:
        module = importlib.import_module(module_name)
        version = getattr(module, "__version__", None)
        return {"module": module_name, "ok": True, "version": version}
    except Exception as exc:
        return {"module": module_name, "ok": False, "error": str(exc)}


def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS runtime preflight (core)")
    parser.add_argument("--json", action="store_true", help="Output JSON only")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    required = [_check_import(name) for name in REQUIRED_IMPORTS]
    optional = [_check_import(name) for name in OPTIONAL_IMPORTS]

    ok = all(item.get("ok") for item in required)
    output = {
        "ok": ok,
        "python": {
            "executable": sys.executable,
            "version": sys.version.splitlines()[0],
            "platform": platform.platform(),
        },
        "repo_root": str(repo_root),
        "required": required,
        "optional": optional,
    }

    if args.json:
        print(json.dumps(output, ensure_ascii=False))
    else:
        print(json.dumps(output, indent=2, ensure_ascii=False))

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
