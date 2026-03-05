"""CLI entrypoint for ATLAS autonomous agent."""
from __future__ import annotations

import argparse
import json
import sys
from typing import Dict
from pathlib import Path

# Ensure local module imports work when running as a script.
THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

try:
    from .agent import AtlasAutonomousAgent
    from .config import AgentConfig
except Exception:  # pragma: no cover - script-mode fallback
    from agent import AtlasAutonomousAgent
    from config import AgentConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ATLAS autonomous agent (Clawdbot-style) powered by OpenAI."
    )
    parser.add_argument("--goal", required=True, help="Objective for the autonomous agent.")
    parser.add_argument("--model", default=None, help="Override OpenAI model.")
    parser.add_argument("--max-steps", type=int, default=None, help="Max autonomous steps.")
    parser.add_argument(
        "--mode",
        choices=["safe", "aggressive"],
        default=None,
        help="Tool approval policy mode.",
    )
    parser.add_argument(
        "--workspace",
        default=None,
        help="Workspace root for file and shell tools (default: repository root).",
    )
    parser.add_argument(
        "--disable-shell",
        action="store_true",
        help="Disable run_shell tool.",
    )
    parser.add_argument(
        "--dry-run-tools",
        action="store_true",
        help="Do not execute tools, only log requested actions.",
    )
    parser.add_argument(
        "--approved-tool",
        action="append",
        default=[],
        help="Tool approved for this run (repeatable).",
    )
    parser.add_argument(
        "--approval-override",
        action="append",
        default=[],
        help="Override policy per tool, format: tool=true|false (repeatable).",
    )
    return parser.parse_args()


def _parse_approval_overrides(items: list[str]) -> Dict[str, bool]:
    parsed: Dict[str, bool] = {}
    for raw in items:
        item = (raw or "").strip()
        if not item:
            continue
        if "=" not in item:
            raise ValueError(f"invalid --approval-override value: {raw}")
        key, value = item.split("=", 1)
        tool = key.strip().lower()
        val = value.strip().lower()
        if val in {"1", "true", "yes", "y", "on"}:
            parsed[tool] = True
        elif val in {"0", "false", "no", "n", "off"}:
            parsed[tool] = False
        else:
            raise ValueError(f"invalid boolean in --approval-override: {raw}")
    return parsed


def main() -> int:
    args = parse_args()
    config = AgentConfig.from_env()
    try:
        workspace = Path(args.workspace).resolve() if args.workspace else None
        approval_overrides = _parse_approval_overrides(args.approval_override)
        config = config.with_overrides(
            model=args.model,
            max_steps=args.max_steps,
            workspace=workspace,
            allow_shell=(False if args.disable_shell else None),
            dry_run_tools=(True if args.dry_run_tools else None),
            mode=args.mode,
        )
        agent = AtlasAutonomousAgent(
            config,
            approval_overrides=approval_overrides,
            approved_tools=args.approved_tool,
        )
        result = agent.run(args.goal)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0 if result.get("ok") else 1
    except Exception as exc:
        print(
            json.dumps(
                {
                    "ok": False,
                    "status": "startup_error",
                    "error": str(exc),
                    "hint": "Ensure OPENAI_API_KEY is set and openai package is installed.",
                },
                indent=2,
                ensure_ascii=False,
            )
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
