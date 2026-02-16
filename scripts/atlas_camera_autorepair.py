#!/usr/bin/env python3
"""
ATLAS camera auto-repair script.

What it does:
1) Verifies Robot (8002) and PUSH (8791) health.
2) Restarts Robot/PUSH when needed.
3) Tests camera indices 0/1/2 directly against Robot API.
4) Selects best index and applies configuration.
5) Validates camera control through PUSH proxy (/cuerpo/api/...).
6) Writes a JSON report in snapshots/support/.
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from urllib import error, parse, request


REPO_ROOT = Path(__file__).resolve().parent.parent
ROBOT_BASE = "http://127.0.0.1:8002"
PUSH_BASE = "http://127.0.0.1:8791"
INDICES = [0, 1, 2]


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _req_json(
    method: str,
    url: str,
    payload: Optional[Dict[str, Any]] = None,
    timeout: int = 15,
) -> Tuple[bool, int, Dict[str, Any], str]:
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = request.Request(url, data=data, method=method.upper(), headers=headers)
    try:
        with request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode("utf-8", "replace")
            try:
                body = json.loads(raw) if raw else {}
                return True, int(r.status), body, ""
            except Exception:
                return False, int(r.status), {}, f"invalid_json_response: {raw[:180]}"
    except error.HTTPError as e:
        raw = e.read().decode("utf-8", "replace")
        try:
            body = json.loads(raw) if raw else {}
        except Exception:
            body = {"raw": raw[:180]}
        return False, int(e.code), body, f"http_{e.code}"
    except Exception as e:
        return False, 0, {}, f"{type(e).__name__}: {e}"


def _req_status(url: str, timeout: int = 10) -> int:
    req = request.Request(url, method="GET")
    try:
        with request.urlopen(req, timeout=timeout) as r:
            return int(r.status)
    except Exception:
        return 0


def _run_ps(args: list[str]) -> Tuple[bool, str]:
    cmd = ["powershell", "-ExecutionPolicy", "Bypass", "-File"] + args
    try:
        p = subprocess.run(
            cmd,
            cwd=str(REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=120,
        )
        out = ((p.stdout or "") + "\n" + (p.stderr or "")).strip()
        return p.returncode == 0, out[:1200]
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"


def _wait_health(url: str, seconds: int = 35) -> bool:
    end = time.time() + seconds
    while time.time() < end:
        if _req_status(url, timeout=5) == 200:
            return True
        time.sleep(1.5)
    return False


def _test_index(index: int) -> Dict[str, Any]:
    out: Dict[str, Any] = {"index": index, "ok": False}
    ok_c, sc_c, body_c, err_c = _req_json(
        "POST",
        f"{ROBOT_BASE}/api/camera/connect-index",
        {"index": int(index), "resolution": [640, 480]},
        timeout=28,
    )
    out["connect_status"] = sc_c
    out["connect_ok"] = bool(ok_c and body_c.get("ok") and body_c.get("connected"))
    out["frame_ok"] = bool(body_c.get("frame_ok")) if body_c else False
    out["connect_error"] = err_c or body_c.get("error") or body_c.get("probe_error") or ""

    snap_qs = parse.urlencode({"source": "camera", "index": index, "jpeg_quality": 80, "t": int(time.time())})
    snap_status = _req_status(f"{ROBOT_BASE}/api/vision/snapshot?{snap_qs}", timeout=14)
    out["snapshot_status"] = snap_status
    out["snapshot_ok"] = snap_status == 200

    out["ok"] = bool(out["connect_ok"] and (out["frame_ok"] or out["snapshot_ok"]))

    _req_json(
        "POST",
        f"{ROBOT_BASE}/api/camera/disconnect",
        {"index": int(index)},
        timeout=10,
    )
    return out


def _apply_best(index: int) -> Dict[str, Any]:
    result: Dict[str, Any] = {"index": index}
    ok_cfg, sc_cfg, body_cfg, err_cfg = _req_json(
        "POST",
        f"{ROBOT_BASE}/api/camera/configure",
        {"index": int(index), "resolution": [640, 480]},
        timeout=15,
    )
    result["configure"] = {"ok": ok_cfg, "status": sc_cfg, "error": err_cfg, "body": body_cfg}

    ok_p, sc_p, body_p, err_p = _req_json(
        "POST",
        f"{PUSH_BASE}/cuerpo/api/camera/connect-index",
        {"index": int(index), "resolution": [640, 480]},
        timeout=35,
    )
    result["proxy_connect"] = {"ok": ok_p, "status": sc_p, "error": err_p, "body": body_p}
    _req_json(
        "POST",
        f"{PUSH_BASE}/cuerpo/api/camera/disconnect",
        {"index": int(index)},
        timeout=12,
    )
    return result


def main() -> int:
    report: Dict[str, Any] = {
        "started_at": _now_iso(),
        "actions": [],
        "tests": [],
        "selected_index": None,
        "result": "unknown",
    }

    robot_ok = _req_status(f"{ROBOT_BASE}/api/health", timeout=6) == 200
    if not robot_ok:
        ok, out = _run_ps([str(REPO_ROOT / "scripts" / "restart_service_clean.ps1"), "-Service", "robot"])
        report["actions"].append({"step": "restart_robot", "ok": ok, "output": out})
        robot_ok = _wait_health(f"{ROBOT_BASE}/api/health", seconds=45)
    report["robot_health"] = robot_ok

    push_ok = _req_status(f"{PUSH_BASE}/ui", timeout=6) == 200
    if not push_ok:
        ok, out = _run_ps([str(REPO_ROOT / "scripts" / "restart_service_clean.ps1"), "-Service", "push"])
        report["actions"].append({"step": "restart_push", "ok": ok, "output": out})
        push_ok = _wait_health(f"{PUSH_BASE}/ui", seconds=40)
    report["push_health"] = push_ok

    if not robot_ok:
        report["result"] = "failed_robot_unhealthy"
        report["ended_at"] = _now_iso()
        _save_report(report)
        _print_summary(report)
        return 2

    tests = [_test_index(i) for i in INDICES]
    report["tests"] = tests

    best = next((t for t in tests if t.get("ok")), None)
    if best is None:
        # One extra recovery attempt for Robot and a final short retest.
        ok, out = _run_ps([str(REPO_ROOT / "scripts" / "restart_service_clean.ps1"), "-Service", "robot"])
        report["actions"].append({"step": "restart_robot_retry", "ok": ok, "output": out})
        _wait_health(f"{ROBOT_BASE}/api/health", seconds=40)
        tests_retry = [_test_index(i) for i in INDICES]
        report["tests_retry"] = tests_retry
        best = next((t for t in tests_retry if t.get("ok")), None)

    if best is None:
        report["result"] = "failed_no_stable_camera"
        report["ended_at"] = _now_iso()
        _save_report(report)
        _print_summary(report)
        return 3

    report["selected_index"] = int(best["index"])
    apply = _apply_best(int(best["index"]))
    report["apply"] = apply
    proxy_ok = bool(apply.get("proxy_connect", {}).get("ok"))
    report["result"] = "ok" if proxy_ok else "partial_proxy_failed"
    report["ended_at"] = _now_iso()
    _save_report(report)
    _print_summary(report)
    return 0 if report["result"] == "ok" else 4


def _save_report(report: Dict[str, Any]) -> None:
    out_dir = REPO_ROOT / "snapshots" / "support"
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    path = out_dir / f"camera_autorepair_{ts}.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    report["_report_path"] = str(path)


def _print_summary(report: Dict[str, Any]) -> None:
    print("AUTO_REPAIR_RESULT:", report.get("result"))
    print("SELECTED_INDEX:", report.get("selected_index"))
    print("ROBOT_HEALTH:", report.get("robot_health"))
    print("PUSH_HEALTH:", report.get("push_health"))
    print("REPORT_PATH:", report.get("_report_path"))
    tests = report.get("tests") or []
    if tests:
        print("TESTS:", json.dumps(tests, ensure_ascii=False))


if __name__ == "__main__":
    sys.exit(main())

