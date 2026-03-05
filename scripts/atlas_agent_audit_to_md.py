"""Render an executive Markdown summary from atlas_agent E2E audit JSON."""
from __future__ import annotations

import argparse
import glob
import json
import os
from pathlib import Path
from typing import Any, Dict, List


def latest_report(audit_dir: Path) -> Path | None:
    files = sorted(
        glob.glob(str(audit_dir / "atlas_agent_e2e_*.json")),
        key=os.path.getmtime,
        reverse=True,
    )
    return Path(files[0]) if files else None


def _endpoint_row(name: str, endpoint: Dict[str, Any]) -> str:
    status = endpoint.get("status", "n/a")
    latency = endpoint.get("latency_ms", "n/a")
    ok = endpoint.get("ok")
    if isinstance(ok, bool):
        ok_s = "yes" if ok else "no"
    else:
        ok_s = "n/a"
    return f"| {name} | {status} | {latency} | {ok_s} |"


def render_md(data: Dict[str, Any], source_path: Path) -> str:
    runtime = data.get("runtime", {})
    endpoints = runtime.get("endpoints", {})
    agent = data.get("agent_e2e", {})
    run_resp = agent.get("run_response", {})
    realtime = data.get("realtime", {})
    sse = realtime.get("monitor_stream", {})
    gates = data.get("quality_gates", {})
    compileall = gates.get("compileall", {})
    tests = gates.get("tests", {})

    rows: List[str] = []
    for key in [
        "push_health",
        "push_deep",
        "nexus_health",
        "robot_health",
        "metrics",
        "telemetry_metrics",
        "governance_log",
        "agent_recent",
    ]:
        val = endpoints.get(key, {})
        if isinstance(val, dict):
            rows.append(_endpoint_row(key, val))

    lines = [
        "# ATLAS Agent E2E Audit - Executive Summary",
        "",
        f"- Source report: `{source_path}`",
        f"- Generated at: `{data.get('generated_at', 'n/a')}`",
        "",
        "## Runtime Health",
        "",
        "| Endpoint | HTTP | Latency ms | Probe OK |",
        "|---|---:|---:|---:|",
    ]
    lines.extend(rows)
    lines.extend(
        [
            "",
            "## Autonomous Agent Smoke",
            "",
            f"- status: `{run_resp.get('status', 'n/a')}`",
            f"- ok: `{run_resp.get('ok', 'n/a')}`",
            f"- steps_used: `{run_resp.get('steps_used', 'n/a')}`",
            f"- session_id: `{run_resp.get('session_id', 'n/a')}`",
            f"- has_final_answer: `{run_resp.get('has_final_answer', 'n/a')}`",
            "",
            "## Realtime",
            "",
            f"- monitor_stream first event ok: `{sse.get('ok', 'n/a')}`",
            f"- monitor_stream first event elapsed_ms: `{sse.get('elapsed_ms', 'n/a')}`",
            "",
            "## Quality Gates",
            "",
            f"- compileall returncode: `{compileall.get('returncode', 'n/a')}`",
            f"- tests returncode: `{tests.get('returncode', 'n/a')}`",
            "",
        ]
    )
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render markdown summary for atlas_agent audit")
    parser.add_argument("--report", default=None, help="Path to audit JSON report")
    parser.add_argument("--audit-dir", default="snapshots/audit", help="Audit directory")
    parser.add_argument("--output", default=None, help="Output markdown path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    audit_dir = (repo_root / args.audit_dir).resolve()
    src = Path(args.report).resolve() if args.report else latest_report(audit_dir)
    if not src or not src.exists():
        raise FileNotFoundError("No audit report found.")
    data = json.loads(src.read_text(encoding="utf-8"))
    output = (
        Path(args.output).resolve()
        if args.output
        else src.with_suffix(".md")
    )
    md = render_md(data, src)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(md, encoding="utf-8")
    print(str(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
