#!/usr/bin/env python3
"""ATLAS Autonomy Manager CLI."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from modules.humanoid.autonomy_manager.manager import get_latest_status, run_cycle


def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS Autonomy Manager")
    parser.add_argument("--mode", choices=("detect-only", "auto"), default="auto")
    parser.add_argument("--no-emit", action="store_true")
    parser.add_argument("--no-ai", action="store_true")
    parser.add_argument("--status", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    if args.status:
        payload = get_latest_status()
    else:
        payload = run_cycle(
            trigger_mode=args.mode,
            emit=not args.no_emit,
            use_ai=not args.no_ai,
        )

    if args.json:
        print(json.dumps(payload, indent=2, ensure_ascii=False))
    else:
        print(
            json.dumps(
                {
                    "generated_at": payload.get("generated_at"),
                    "cycle_id": payload.get("cycle_id"),
                    "manager_status": payload.get("manager_status"),
                    "policy_mode": ((payload.get("policy") or {}).get("mode")),
                    "summary": payload.get("summary"),
                },
                indent=2,
                ensure_ascii=False,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
