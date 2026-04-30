"""One-command E2E audit for ATLAS runtime + atlas_agent endpoints.

Usage:
  python scripts/atlas_agent_e2e_audit.py
  python scripts/atlas_agent_e2e_audit.py --restart-push
"""
from __future__ import annotations

import argparse
import json
import platform
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

import requests


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def truncate(text: str, limit: int = 600) -> str:
    if len(text) <= limit:
        return text
    return text[:limit] + f"...[truncated {len(text) - limit} chars]"


def run_cmd(cmd: List[str], cwd: Path, timeout_sec: int = 120) -> Dict[str, Any]:
    t0 = time.perf_counter()
    proc = subprocess.run(
        cmd,
        cwd=str(cwd),
        capture_output=True,
        text=True,
        timeout=timeout_sec,
        shell=False,
    )
    elapsed_ms = int((time.perf_counter() - t0) * 1000)
    return {
        "cmd": " ".join(cmd),
        "returncode": int(proc.returncode),
        "elapsed_ms": elapsed_ms,
        "stdout": truncate(proc.stdout or ""),
        "stderr": truncate(proc.stderr or ""),
    }


def probe_http(
    method: str,
    url: str,
    *,
    json_body: Dict[str, Any] | None = None,
    timeout_sec: int = 40,
) -> Dict[str, Any]:
    t0 = time.perf_counter()
    try:
        resp = requests.request(method, url, json=json_body, timeout=timeout_sec)
        elapsed_ms = int((time.perf_counter() - t0) * 1000)
        text = resp.text or ""
        parsed = None
        try:
            parsed = resp.json()
        except Exception:
            parsed = None
        return {
            "ok": True,
            "status": int(resp.status_code),
            "latency_ms": elapsed_ms,
            "sample": truncate(text, 500),
            "json": parsed,
        }
    except Exception as exc:
        elapsed_ms = int((time.perf_counter() - t0) * 1000)
        return {
            "ok": False,
            "status": 0,
            "latency_ms": elapsed_ms,
            "error": str(exc),
        }


def first_sse_event(url: str, timeout_connect: int = 3, timeout_read: int = 8) -> Dict[str, Any]:
    t0 = time.perf_counter()
    try:
        with requests.get(url, stream=True, timeout=(timeout_connect, timeout_read)) as resp:
            for line in resp.iter_lines(decode_unicode=True):
                if not line:
                    continue
                if line.startswith("data:"):
                    return {
                        "ok": True,
                        "status": int(resp.status_code),
                        "elapsed_ms": int((time.perf_counter() - t0) * 1000),
                        "sample": truncate(line[5:].strip(), 320),
                    }
        return {"ok": False, "status": 0, "error": "no_data_event"}
    except Exception as exc:
        return {"ok": False, "status": 0, "error": str(exc)}


def ports_snapshot(repo_root: Path) -> List[Dict[str, Any]]:
    if platform.system().lower() != "windows":
        return []
    cmd = [
        "powershell",
        "-NoProfile",
        "-Command",
        (
            "Get-NetTCPConnection -State Listen -LocalPort 8791,8000,8002 "
            "-ErrorAction SilentlyContinue | "
            "Select-Object LocalAddress,LocalPort,OwningProcess,State | "
            "ConvertTo-Json -Depth 4"
        ),
    ]
    data = run_cmd(cmd, repo_root, timeout_sec=30)
    try:
        parsed = json.loads(data["stdout"])
        if isinstance(parsed, dict):
            parsed = [parsed]
        return parsed if isinstance(parsed, list) else []
    except Exception:
        return []


def bool_arg(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ATLAS Agent E2E audit runner")
    parser.add_argument("--base-url", default="http://127.0.0.1:8791")
    parser.add_argument("--nexus-url", default="http://127.0.0.1:8000")
    parser.add_argument("--robot-url", default="http://127.0.0.1:8002")
    parser.add_argument("--time-range", type=int, default=300)
    parser.add_argument("--restart-push", action="store_true")
    parser.add_argument("--run-quality-gates", type=bool_arg, default=True)
    parser.add_argument("--agent-goal", default="Return final answer immediately in one short line.")
    parser.add_argument("--agent-max-steps", type=int, default=3)
    parser.add_argument("--agent-mode", choices=["safe", "aggressive"], default="safe")
    parser.add_argument("--agent-dry-run-tools", type=bool_arg, default=True)
    parser.add_argument("--report-dir", default="snapshots/audit")
    parser.add_argument("--report-prefix", default="atlas_agent_e2e")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    report_dir = (repo_root / args.report_dir).resolve()
    report_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = report_dir / f"{args.report_prefix}_{ts}.json"

    base_url = args.base_url.rstrip("/")
    nexus_url = args.nexus_url.rstrip("/")
    robot_url = args.robot_url.rstrip("/")

    report: Dict[str, Any] = {
        "generated_at": now_iso(),
        "repo_root": str(repo_root),
        "args": vars(args),
        "restart": {},
        "runtime": {},
        "agent_e2e": {},
        "realtime": {},
        "quality_gates": {},
    }

    if args.restart_push:
        restart_cmd = [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            "scripts/restart_push_from_api.ps1",
            "-DelaySeconds",
            "1",
            "-HealthTimeoutSec",
            "90",
        ]
        report["restart"] = run_cmd(restart_cmd, repo_root, timeout_sec=240)
    else:
        report["restart"] = {"skipped": True}

    report["runtime"]["ports"] = ports_snapshot(repo_root)
    report["runtime"]["endpoints"] = {
        "push_health": probe_http("GET", f"{base_url}/health"),
        "push_deep": probe_http("GET", f"{base_url}/health/deep"),
        "nexus_health": probe_http("GET", f"{nexus_url}/health"),
        "robot_health": probe_http("GET", f"{robot_url}/api/health"),
        "metrics": probe_http("GET", f"{base_url}/metrics"),
        "telemetry_metrics": probe_http(
            "GET",
            f"{base_url}/api/telemetry/metrics?time_range={args.time_range}",
        ),
        "governance_log": probe_http("GET", f"{base_url}/governance/log"),
        "agent_recent": probe_http("GET", f"{base_url}/api/agent/autonomous/runs/recent?limit=3"),
    }

    run_payload = {
        "goal": args.agent_goal,
        "max_steps": int(args.agent_max_steps),
        "mode": args.agent_mode,
        "dry_run_tools": bool(args.agent_dry_run_tools),
    }
    run_resp = probe_http(
        "POST",
        f"{base_url}/api/agent/autonomous/run",
        json_body=run_payload,
        timeout_sec=240,
    )
    run_json = run_resp.get("json") if isinstance(run_resp.get("json"), dict) else {}
    session_id = (run_json or {}).get("session_id")
    report["agent_e2e"]["run_request"] = run_payload
    report["agent_e2e"]["run_response"] = {
        "ok": (run_json or {}).get("ok"),
        "status": (run_json or {}).get("status"),
        "latency_ms": run_resp.get("latency_ms"),
        "session_id": session_id,
        "steps_used": (run_json or {}).get("steps_used"),
        "error": (run_json or {}).get("error"),
        "has_final_answer": bool((run_json or {}).get("final_answer")),
    }
    if session_id:
        detail = probe_http("GET", f"{base_url}/api/agent/autonomous/runs/{session_id}")
        detail_json = detail.get("json") if isinstance(detail.get("json"), dict) else {}
        report["agent_e2e"]["run_detail"] = {
            "ok": detail_json.get("ok"),
            "status": detail.get("status"),
            "latency_ms": detail.get("latency_ms"),
            "events_path": detail_json.get("events_path"),
        }

    trading = probe_http(
        "POST",
        f"{base_url}/api/trading/proxy",
        json_body={"prompt": "Return short heartbeat confirmation", "max_tokens": 32},
        timeout_sec=120,
    )
    trading_json = trading.get("json") if isinstance(trading.get("json"), dict) else {}
    report["agent_e2e"]["trading_proxy"] = {
        "status": trading.get("status"),
        "ok_field": trading_json.get("ok"),
        "has_data": isinstance(trading_json.get("data"), dict),
        "has_error": isinstance(trading_json.get("error"), str),
        "latency_ms": trading.get("latency_ms"),
    }

    report["realtime"]["monitor_stream"] = first_sse_event(f"{base_url}/api/monitor/stream")

    telemetry_json = report["runtime"]["endpoints"]["telemetry_metrics"].get("json")
    if isinstance(telemetry_json, dict):
        meta = telemetry_json.get("meta") if isinstance(telemetry_json.get("meta"), dict) else {}
        report["realtime"]["telemetry_meta"] = {
            "count": meta.get("count"),
            "latest_ts": meta.get("latest_ts"),
            "stale": meta.get("stale"),
            "source": meta.get("source"),
            "time_range_sec": meta.get("time_range_sec"),
        }

    if args.run_quality_gates:
        report["quality_gates"]["compileall"] = run_cmd(
            ["python", "-m", "compileall", "-q", "tools/atlas_agent", "atlas_adapter"],
            repo_root,
            timeout_sec=180,
        )
        report["quality_gates"]["tests"] = run_cmd(
            ["pytest", "tests/tools", "-q"],
            repo_root,
            timeout_sec=180,
        )
    else:
        report["quality_gates"] = {"skipped": True}

    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(str(report_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
