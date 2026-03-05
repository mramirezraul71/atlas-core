from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
LATEST_FILE = ROOT / "logs" / "software_maintenance" / "latest_cycle.json"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _run_subprocess(cmd: List[str], timeout: int = 300, cwd: Path | None = None) -> Dict[str, Any]:
    t0 = time.perf_counter()
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(cwd or ROOT),
            capture_output=True,
            text=True,
            timeout=timeout,
            shell=False,
        )
        stdout = (proc.stdout or "").strip()
        stderr = (proc.stderr or "").strip()
        return {
            "ok": proc.returncode == 0,
            "returncode": proc.returncode,
            "stdout_tail": "\n".join(stdout.splitlines()[-8:]),
            "stderr_tail": "\n".join(stderr.splitlines()[-8:]),
            "ms": int((time.perf_counter() - t0) * 1000),
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": f"timeout_{timeout}s", "ms": int((time.perf_counter() - t0) * 1000)}
    except Exception as ex:
        return {"ok": False, "error": str(ex), "ms": int((time.perf_counter() - t0) * 1000)}


def _write_markdown_report(report: Dict[str, Any]) -> str:
    reports_dir = ROOT / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = reports_dir / f"software_maintenance_{ts}.md"
    lines: List[str] = []
    lines.append("# ATLAS Software Maintenance Report")
    lines.append("")
    lines.append(f"- Start: `{report.get('started_at')}`")
    lines.append(f"- End: `{report.get('finished_at')}`")
    lines.append(f"- Status: `{report.get('status')}`")
    lines.append(f"- Steps OK: `{report.get('summary', {}).get('ok_steps', 0)}`")
    lines.append(f"- Steps Failed: `{report.get('summary', {}).get('failed_steps', 0)}`")
    lines.append("")
    lines.append("## Steps")
    for step in report.get("steps", []):
        if not isinstance(step, dict):
            continue
        flag = "OK" if step.get("ok") else "FAIL"
        lines.append(f"- `{step.get('step')}`: **{flag}**")
        err = step.get("error")
        if err:
            lines.append(f"  - Error: `{err}`")
        details = step.get("details")
        if isinstance(details, dict):
            tail = details.get("stdout_tail") or details.get("message") or details.get("branch")
            if tail:
                lines.append(f"  - Detail: `{str(tail)[:200]}`")
    lines.append("")
    lines.append("## Raw JSON")
    lines.append("```json")
    lines.append(json.dumps(report, ensure_ascii=False, indent=2))
    lines.append("```")
    path.write_text("\n".join(lines), encoding="utf-8")
    return str(path)


def run_cycle(apply_repo: bool = False, job_file: Path | None = None) -> Dict[str, Any]:
    started_at = _utc_now()
    steps: List[Dict[str, Any]] = []
    total_steps = 8

    def _update_job(status: str, done: int, failed: int, current_step: str = "", extra: Dict[str, Any] | None = None) -> None:
        if not job_file:
            return
        payload = {
            "ok": failed == 0,
            "status": status,
            "started_at": started_at,
            "updated_at": _utc_now(),
            "total": total_steps,
            "done": done,
            "failed": failed,
            "current_step": current_step,
            "steps": steps,
        }
        if extra:
            payload.update(extra)
        _write_json(job_file, payload)

    def _add_step(name: str, fn):
        done = len(steps)
        failed = sum(1 for s in steps if not s.get("ok"))
        _update_job("running", done, failed, current_step=name)
        t0 = time.perf_counter()
        try:
            details = fn()
            ok = bool(details.get("ok", False)) if isinstance(details, dict) else False
            entry = {
                "step": name,
                "ok": ok,
                "ms": int((time.perf_counter() - t0) * 1000),
                "details": details if isinstance(details, dict) else {"value": details},
            }
            if not ok:
                entry["error"] = str((details or {}).get("error") or f"{name}_failed")
        except Exception as ex:
            entry = {
                "step": name,
                "ok": False,
                "ms": int((time.perf_counter() - t0) * 1000),
                "error": str(ex),
                "details": {},
            }
        steps.append(entry)
        done = len(steps)
        failed = sum(1 for s in steps if not s.get("ok"))
        _update_job("running", done, failed, current_step=name)

    def _ensure_scheduler_jobs() -> Dict[str, Any]:
        out = {"ok": True, "ensured": []}
        from modules.humanoid.ans.scheduler_jobs import ensure_ans_jobs
        from modules.humanoid.comms.makeplay_scheduler import ensure_makeplay_jobs
        from modules.humanoid.scheduler.repo_monitor_jobs import ensure_repo_monitor_jobs

        ensure_ans_jobs()
        out["ensured"].append("ans_cycle")
        ensure_makeplay_jobs()
        out["ensured"].append("makeplay_scanner")
        ensure_repo_monitor_jobs()
        out["ensured"].append("repo_monitor_cycle")
        return out

    def _run_makeplay_scan() -> Dict[str, Any]:
        from modules.humanoid.comms.makeplay_scanner import run_scan

        res = run_scan() or {}
        return {"ok": bool(res.get("ok")), "snapshot": res.get("snapshot")}

    def _run_ans_cycle() -> Dict[str, Any]:
        from modules.humanoid.ans.engine import run_ans_cycle

        res = run_ans_cycle(mode="auto", timeout_sec=60) or {}
        return {
            "ok": bool(res.get("ok")),
            "issues_count": int(res.get("issues_count") or 0),
            "actions_taken": len(res.get("actions_taken") or []),
            "report_path": res.get("report_path"),
        }

    def _run_triada_once() -> Dict[str, Any]:
        return _run_subprocess([sys.executable, str(ROOT / "evolution_daemon.py"), "--run-once"], timeout=900, cwd=ROOT)

    def _run_repo_monitor_cycle() -> Dict[str, Any]:
        return _run_subprocess([sys.executable, str(ROOT / "scripts" / "repo_monitor.py"), "--cycle"], timeout=180, cwd=ROOT)

    def _run_repo_update_cycle() -> Dict[str, Any]:
        from modules.humanoid.update.update_engine import apply as update_apply
        from modules.humanoid.update.update_engine import check as update_check

        checked = update_check() or {}
        if not checked.get("ok"):
            return {"ok": False, "phase": "check", "error": checked.get("error"), "data": checked.get("data")}
        out: Dict[str, Any] = {"ok": True, "phase": "check", "data": checked.get("data")}
        if apply_repo:
            applied = update_apply() or {}
            out["apply"] = applied
            out["ok"] = bool(applied.get("ok", False))
            out["phase"] = "apply"
            if not out["ok"]:
                out["error"] = applied.get("error") or "update_apply_failed"
        return out

    def _refresh_tools_watchdog() -> Dict[str, Any]:
        return _run_subprocess([sys.executable, str(ROOT / "scripts" / "atlas_tools_watchdog.py"), "--force"], timeout=240, cwd=ROOT)

    def _refresh_software_watchdog() -> Dict[str, Any]:
        return _run_subprocess([sys.executable, str(ROOT / "scripts" / "atlas_software_watchdog.py"), "--force"], timeout=240, cwd=ROOT)

    _update_job("running", 0, 0, current_step="boot")
    _add_step("ensure_scheduler_jobs", _ensure_scheduler_jobs)
    _add_step("makeplay_scan", _run_makeplay_scan)
    _add_step("ans_cycle", _run_ans_cycle)
    _add_step("triada_run_once", _run_triada_once)
    _add_step("repo_monitor_cycle", _run_repo_monitor_cycle)
    _add_step("repo_update_cycle", _run_repo_update_cycle)
    _add_step("refresh_tools_watchdog", _refresh_tools_watchdog)
    _add_step("refresh_software_watchdog", _refresh_software_watchdog)

    failed = sum(1 for s in steps if not s.get("ok"))
    finished_at = _utc_now()
    result = {
        "ok": failed == 0,
        "started_at": started_at,
        "finished_at": finished_at,
        "status": "done" if failed == 0 else "done_with_errors",
        "summary": {
            "total_steps": len(steps),
            "ok_steps": len(steps) - failed,
            "failed_steps": failed,
            "apply_repo": bool(apply_repo),
        },
        "steps": steps,
    }
    report_path = _write_markdown_report(result)
    result["report_path"] = report_path

    latest_payload = dict(result)
    latest_payload["latest_at"] = finished_at
    _write_json(LATEST_FILE, latest_payload)

    _update_job(
        "done" if failed == 0 else "done_with_errors",
        len(steps),
        failed,
        current_step="completed",
        extra={"report_path": report_path, "finished_at": finished_at},
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS software maintenance cycle")
    parser.add_argument("--apply-repo", action="store_true", help="Attempt repo apply cycle after check")
    parser.add_argument("--job-file", type=str, default="", help="Optional status JSON path for background tracking")
    parser.add_argument("--pretty", action="store_true", help="Pretty print final JSON")
    args = parser.parse_args()

    job_file = Path(args.job_file).resolve() if args.job_file else None
    result = run_cycle(apply_repo=bool(args.apply_repo), job_file=job_file)
    if args.pretty:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
