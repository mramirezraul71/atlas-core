from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
HUB_DIR = ROOT / "software_hub"
SOFTWARE_CATALOG_FILE = HUB_DIR / "software_catalog.json"
DRIVER_REQUIREMENTS_FILE = HUB_DIR / "driver_requirements.json"
DEV_STACK_FILE = HUB_DIR / "development_stack.json"
REGISTRY_FILE = HUB_DIR / "software_registry.json"
CACHE_FILE = ROOT / "logs" / "atlas_software_watchdog_cache.json"
LOG_FILE = ROOT / "logs" / "atlas_software_watchdog.log"
LATEST_TTL_SEC = 20 * 60


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _append_log(line: str) -> None:
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    with LOG_FILE.open("a", encoding="utf-8") as fh:
        fh.write(f"{_utc_now()} {line}\n")


def _read_json(path: Path, default: Any) -> Any:
    if not path.exists():
        return default
    try:
        return json.loads(path.read_text(encoding="utf-8-sig", errors="replace"))
    except Exception:
        return default


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _run_cmd(cmd: List[str], timeout: int = 12, cwd: Optional[Path] = None) -> Tuple[bool, str]:
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(cwd or ROOT),
            capture_output=True,
            text=True,
            timeout=timeout,
            shell=False,
        )
        out = "\n".join(
            [x.strip() for x in ((proc.stdout or ""), (proc.stderr or "")) if x and x.strip()]
        ).strip()
        return proc.returncode == 0, out
    except Exception as ex:
        return False, str(ex)


def _http_json(url: str, timeout: int = 8) -> Tuple[bool, Dict[str, Any], str]:
    req = urllib.request.Request(
        url=url,
        headers={"Accept": "application/json", "User-Agent": "ATLAS-Software-Watchdog/1.0"},
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
        data = json.loads(raw)
        if isinstance(data, dict):
            return True, data, ""
        return True, {"data": data}, ""
    except urllib.error.HTTPError as ex:
        return False, {}, f"http_{ex.code}"
    except Exception as ex:
        return False, {}, str(ex)


def _probe(url: str, timeout_sec: float = 6.0) -> Tuple[bool, Optional[int], str]:
    t0 = time.perf_counter()
    try:
        req = urllib.request.Request(url, method="GET", headers={"User-Agent": "ATLAS-Software-Watchdog/1.0"})
        with urllib.request.urlopen(req, timeout=timeout_sec) as resp:
            _ = resp.read(512)
        return True, int((time.perf_counter() - t0) * 1000), ""
    except Exception as ex:
        return False, None, str(ex)[:180]


def _extract_version(text: str) -> str:
    if not text:
        return ""
    m = re.search(r"(\d+\.\d+\.\d+(?:[\w.+-]*)?)", text)
    if m:
        return m.group(1).lstrip("vV")
    m2 = re.search(r"(\d+\.\d+)", text)
    return m2.group(1).lstrip("vV") if m2 else ""


def _norm_version_tuple(raw: str) -> Tuple[int, ...]:
    if not raw:
        return tuple()
    return tuple(int(x) for x in re.findall(r"\d+", raw)[:4])


def _is_newer(latest: str, local: str) -> bool:
    a = _norm_version_tuple(latest)
    b = _norm_version_tuple(local)
    if not a or not b:
        return False
    return a > b


def _load_cache() -> Dict[str, Any]:
    data = _read_json(CACHE_FILE, {})
    if not isinstance(data, dict):
        data = {}
    data.setdefault("latest", {})
    return data


def _latest_from_source(spec: Dict[str, Any]) -> str:
    stype = str(spec.get("type") or "none").strip().lower()
    if stype in ("", "none"):
        return ""
    if stype == "pypi":
        pkg = str(spec.get("package") or "").strip()
        if not pkg:
            return ""
        ok, payload, _ = _http_json(f"https://pypi.org/pypi/{pkg}/json")
        if not ok:
            return ""
        return _extract_version(str((payload.get("info") or {}).get("version") or ""))
    if stype == "github":
        owner = str(spec.get("owner") or "").strip()
        repo = str(spec.get("repo") or "").strip()
        if not owner or not repo:
            return ""
        ok, payload, _ = _http_json(f"https://api.github.com/repos/{owner}/{repo}/releases/latest")
        if not ok:
            return ""
        return _extract_version(str(payload.get("tag_name") or payload.get("name") or ""))
    if stype == "nodejs":
        req = urllib.request.Request(
            url="https://nodejs.org/dist/index.json",
            headers={"User-Agent": "ATLAS-Software-Watchdog/1.0"},
        )
        try:
            with urllib.request.urlopen(req, timeout=8) as resp:
                data = json.loads(resp.read().decode("utf-8", errors="replace"))
            if isinstance(data, list) and data:
                return _extract_version(str(data[0].get("version") or ""))
            return ""
        except Exception:
            return ""
    return ""


def _winget_installed(package_id: str) -> Tuple[bool, str]:
    if not package_id:
        return False, ""
    ok, out = _run_cmd(["winget", "list", "--id", package_id, "--source", "winget"], timeout=30)
    low = out.lower()
    if ok and package_id.lower() in low:
        return True, out
    return False, out


def _resolve_latest_versions(items: List[Dict[str, Any]], force: bool) -> Dict[str, str]:
    cache = _load_cache()
    latest_cache = cache.setdefault("latest", {})
    now = int(time.time())
    out: Dict[str, str] = {}

    for item in items:
        sid = str(item.get("id") or "").strip()
        if not sid:
            continue
        cached = latest_cache.get(sid) if isinstance(latest_cache.get(sid), dict) else {}
        ts = int(cached.get("ts") or 0) if cached else 0
        val = str(cached.get("version") or "") if cached else ""
        if not force and ts > 0 and (now - ts) <= LATEST_TTL_SEC and val:
            out[sid] = val
            continue
        latest = _latest_from_source(item.get("latest") or {})
        latest_cache[sid] = {"version": latest, "ts": now}
        out[sid] = latest
    _write_json(CACHE_FILE, cache)
    return out


def _detect_software_rows(items: List[Dict[str, Any]], latest_map: Dict[str, str]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for item in items:
        sid = str(item.get("id") or "").strip()
        if not sid:
            continue
        name = str(item.get("name") or sid)
        category = str(item.get("category") or "misc")
        critical = bool(item.get("critical"))
        check_cmd = item.get("check_command") if isinstance(item.get("check_command"), list) else []
        install_method = str(item.get("install_method") or "").strip().lower()
        install_target = str(item.get("install_target") or "").strip()
        network_probe = str(item.get("network_probe") or "").strip()

        check_ok = False
        check_out = ""
        local_version = ""
        if check_cmd:
            check_ok, check_out = _run_cmd([str(x) for x in check_cmd], timeout=16)
            local_version = _extract_version(check_out)
        winget_ok = False
        winget_out = ""
        if not check_ok and install_method == "winget" and install_target:
            winget_ok, winget_out = _winget_installed(install_target)
            if winget_ok and not local_version:
                local_version = _extract_version(winget_out)

        installed = bool(check_ok or winget_ok)
        health = "ready" if check_ok else ("installed_no_path" if winget_ok else "not_detected")
        status = "ok" if check_ok else ("warn" if winget_ok else ("error" if critical else "warn"))
        details = check_out if check_ok else (winget_out if winget_ok else "")

        latest = str(latest_map.get(sid) or "")
        update_ready = bool(installed and latest and local_version and _is_newer(latest, local_version))
        if update_ready and status == "ok":
            status = "warn"
            health = "upgrade_ready"

        net_ok, latency_ms, net_err = _probe(network_probe) if network_probe else (False, None, "")
        if net_ok:
            candidate_status = "network_ok"
        else:
            candidate_status = "network_unreachable"

        row = {
            "id": sid,
            "name": name,
            "category": category,
            "critical": critical,
            "install_method": install_method,
            "install_target": install_target,
            "installed": installed,
            "version": local_version,
            "latest_version": latest,
            "update_ready": update_ready,
            "status": status,
            "health": health,
            "details": details[:500],
            "network_probe": network_probe,
            "network_available": bool(net_ok),
            "latency_ms": latency_ms,
            "network_status": candidate_status,
            "network_error": net_err if net_err else None,
        }
        rows.append(row)
    rows.sort(key=lambda x: (x.get("installed") is not True, str(x.get("category")), str(x.get("name"))))
    return rows


def _powershell_json(command: str, timeout: int = 25) -> Any:
    ok, out = _run_cmd(
        [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-Command",
            command,
        ],
        timeout=timeout,
    )
    if not ok or not out:
        return []
    try:
        parsed = json.loads(out)
        if isinstance(parsed, list):
            return parsed
        if isinstance(parsed, dict):
            return [parsed]
        return []
    except Exception:
        return []


def _get_video_drivers() -> List[Dict[str, Any]]:
    cmd = (
        "Get-CimInstance Win32_VideoController | "
        "Select-Object Name,DriverVersion,DriverDate | ConvertTo-Json -Depth 4 -Compress"
    )
    rows = _powershell_json(cmd, timeout=30)
    out: List[Dict[str, Any]] = []
    for r in rows:
        if not isinstance(r, dict):
            continue
        out.append(
            {
                "name": str(r.get("Name") or ""),
                "version": str(r.get("DriverVersion") or ""),
                "date": str(r.get("DriverDate") or ""),
            }
        )
    return out


def _get_pnp_class_drivers(device_class: str) -> List[Dict[str, Any]]:
    klass = str(device_class or "").strip()
    if not klass:
        return []
    cmd = (
        f"Get-CimInstance Win32_PnPSignedDriver | Where-Object {{$_.DeviceClass -eq '{klass}'}} | "
        "Select-Object -First 20 DeviceName,DriverVersion,DriverProviderName,DeviceClass | "
        "ConvertTo-Json -Depth 4 -Compress"
    )
    rows = _powershell_json(cmd, timeout=35)
    out: List[Dict[str, Any]] = []
    for r in rows:
        if not isinstance(r, dict):
            continue
        out.append(
            {
                "name": str(r.get("DeviceName") or ""),
                "version": str(r.get("DriverVersion") or ""),
                "provider": str(r.get("DriverProviderName") or ""),
                "class": str(r.get("DeviceClass") or ""),
            }
        )
    return out


def _detect_drivers(requirements: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    video_cache: Optional[List[Dict[str, Any]]] = None
    pnp_cache: Dict[str, List[Dict[str, Any]]] = {}
    rows: List[Dict[str, Any]] = []

    for req in requirements:
        rid = str(req.get("id") or "").strip()
        if not rid:
            continue
        name = str(req.get("name") or rid)
        critical = bool(req.get("critical"))
        detector = req.get("detector") if isinstance(req.get("detector"), dict) else {}
        dtype = str(detector.get("type") or "").strip().lower()
        match_any = [str(x).lower() for x in (detector.get("match_any") or []) if str(x).strip()]
        hits: List[Dict[str, Any]] = []

        if dtype == "video_controller":
            if video_cache is None:
                video_cache = _get_video_drivers()
            hits = list(video_cache or [])
        elif dtype == "pnp_class":
            klass = str(detector.get("device_class") or "").strip()
            if klass not in pnp_cache:
                pnp_cache[klass] = _get_pnp_class_drivers(klass)
            hits = list(pnp_cache.get(klass) or [])

        if match_any:
            lowered = []
            for h in hits:
                merged = f"{h.get('name', '')} {h.get('provider', '')}".lower()
                lowered.append((h, merged))
            hits = [h for (h, low) in lowered if any(tok in low for tok in match_any)]

        detected = len(hits) > 0
        best = hits[0] if hits else {}
        version = str(best.get("version") or "")
        status = "ok" if detected else ("error" if critical else "warn")
        health = "ready" if detected else "missing"

        rows.append(
            {
                "id": rid,
                "name": name,
                "critical": critical,
                "status": status,
                "health": health,
                "detected": detected,
                "version": version,
                "matches": hits[:5],
                "count": len(hits),
            }
        )

    rows.sort(key=lambda x: (x.get("detected") is not True, str(x.get("name"))))
    return rows


def _http_status(url: str, timeout: int = 4) -> Tuple[bool, str]:
    try:
        req = urllib.request.Request(url=url, method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
        parsed = json.loads(raw)
        if isinstance(parsed, dict):
            ver = str((parsed.get("checks") or {}).get("version") or parsed.get("version") or "")
            return True, ver
        return True, ""
    except Exception as ex:
        return False, str(ex)[:180]


def _scheduler_jobs_index() -> Dict[str, Dict[str, Any]]:
    try:
        from modules.humanoid.scheduler import get_scheduler_db

        jobs = get_scheduler_db().list_jobs(limit=200) or []
        out: Dict[str, Dict[str, Any]] = {}
        for job in jobs:
            if not isinstance(job, dict):
                continue
            out[str(job.get("name") or "")] = job
        return out
    except Exception:
        return {}


def _detect_dev_stack(components: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    jobs = _scheduler_jobs_index()
    rows: List[Dict[str, Any]] = []

    makeplay_last_scan: Dict[str, Any] = {}
    try:
        from modules.humanoid.comms.scanner_store import get_last_scan

        makeplay_last_scan = get_last_scan() or {}
    except Exception:
        makeplay_last_scan = {}

    repo_status: Dict[str, Any] = {}
    try:
        from modules.humanoid.update.update_engine import status as update_status

        repo_status = update_status() or {}
    except Exception:
        repo_status = {}

    for comp in components:
        cid = str(comp.get("id") or "").strip()
        if not cid:
            continue
        name = str(comp.get("name") or cid)
        ctype = str(comp.get("type") or "").strip().lower()
        critical = bool(comp.get("critical"))
        status = "warn"
        health = "unknown"
        version = ""
        details = ""

        if ctype == "service":
            url = str(comp.get("url") or "").strip()
            ok, data = _http_status(url, timeout=4)
            status = "ok" if ok else ("error" if critical else "warn")
            health = "online" if ok else "offline"
            if ok:
                version = data
            else:
                details = data
        elif ctype == "scheduler_job":
            job_name = str(comp.get("job_name") or "").strip()
            job = jobs.get(job_name) or {}
            jstatus = str(job.get("status") or "").strip().lower()
            is_alive = jstatus in ("queued", "running", "success")
            status = "ok" if is_alive else ("error" if critical else "warn")
            health = jstatus or ("missing" if not job else "unknown")
            details = f"job={job_name}"
        elif ctype == "log_file":
            rel = str(comp.get("path") or "").strip()
            p = (ROOT / rel).resolve()
            if p.exists():
                payload = _read_json(p, {})
                ok = bool(payload.get("ok", True)) if isinstance(payload, dict) else True
                status = "ok" if ok else ("warn" if not critical else "error")
                health = "present"
                if isinstance(payload, dict):
                    version = str(payload.get("ts") or payload.get("generated_at") or "")
                    details = str(payload.get("message") or "")[:240]
            else:
                status = "error" if critical else "warn"
                health = "missing"
                details = f"missing_file:{rel}"
        elif ctype == "makeplay_last_scan":
            ts = str(makeplay_last_scan.get("ts") or "")
            ok = bool(ts)
            status = "ok" if ok else ("error" if critical else "warn")
            health = "fresh" if ok else "empty"
            version = ts
            details = (
                f"health_score={makeplay_last_scan.get('health_score')} incidents_open={makeplay_last_scan.get('incidents_open')}"
                if ok
                else "no_makeplay_scan_data"
            )
        elif ctype == "repo_update":
            ok = bool(repo_status.get("ok"))
            has_update = bool(repo_status.get("has_update"))
            status = "warn" if has_update else ("ok" if ok else ("error" if critical else "warn"))
            health = "update_ready" if has_update else ("synced" if ok else "unknown")
            version = str(repo_status.get("branch") or "")
            details = (
                f"head={str(repo_status.get('head_commit') or '')[:8]} remote={str(repo_status.get('remote_commit') or '')[:8]}"
            )
        else:
            status = "warn"
            health = "unsupported_type"
            details = f"type={ctype}"

        rows.append(
            {
                "id": cid,
                "name": name,
                "type": ctype,
                "critical": critical,
                "status": status,
                "health": health,
                "version": version,
                "details": details,
            }
        )

    rows.sort(key=lambda x: (x.get("status") != "ok", str(x.get("name"))))
    return rows


def _build_registry(force: bool = False) -> Dict[str, Any]:
    catalog = _read_json(SOFTWARE_CATALOG_FILE, {})
    items = catalog.get("items") if isinstance(catalog, dict) else []
    items = items if isinstance(items, list) else []

    drivers_cfg = _read_json(DRIVER_REQUIREMENTS_FILE, {})
    driver_reqs = drivers_cfg.get("drivers") if isinstance(drivers_cfg, dict) else []
    driver_reqs = driver_reqs if isinstance(driver_reqs, list) else []

    dev_cfg = _read_json(DEV_STACK_FILE, {})
    dev_components = dev_cfg.get("components") if isinstance(dev_cfg, dict) else []
    dev_components = dev_components if isinstance(dev_components, list) else []

    latest_map = _resolve_latest_versions(items, force=force)
    software_rows = _detect_software_rows(items, latest_map)
    driver_rows = _detect_drivers(driver_reqs)
    dev_rows = _detect_dev_stack(dev_components)

    network_candidates: List[Dict[str, Any]] = []
    for row in software_rows:
        if row.get("installed"):
            continue
        if not row.get("install_method") or not row.get("install_target"):
            continue
        score = 50
        if row.get("network_available"):
            score += 35
        if row.get("critical"):
            score += 10
        if row.get("latest_version"):
            score += 5
        network_candidates.append(
            {
                "id": row.get("id"),
                "name": row.get("name"),
                "category": row.get("category"),
                "critical": bool(row.get("critical")),
                "install_method": row.get("install_method"),
                "install_target": row.get("install_target"),
                "network_available": bool(row.get("network_available")),
                "latency_ms": row.get("latency_ms"),
                "network_error": row.get("network_error"),
                "latest_version": row.get("latest_version"),
                "score": max(0, min(100, score)),
                "status": "ready" if row.get("network_available") else "unreachable",
            }
        )
    network_candidates.sort(
        key=lambda x: (x.get("network_available") is not True, -int(x.get("score") or 0), str(x.get("name") or ""))
    )

    sw_ok = sum(1 for r in software_rows if r.get("status") == "ok")
    sw_warn = sum(1 for r in software_rows if r.get("status") == "warn")
    sw_err = sum(1 for r in software_rows if r.get("status") == "error")
    sw_updates = sum(1 for r in software_rows if r.get("update_ready"))
    dr_ok = sum(1 for r in driver_rows if r.get("status") == "ok")
    dr_warn = sum(1 for r in driver_rows if r.get("status") == "warn")
    dr_err = sum(1 for r in driver_rows if r.get("status") == "error")
    dev_ok = sum(1 for r in dev_rows if r.get("status") == "ok")
    dev_warn = sum(1 for r in dev_rows if r.get("status") == "warn")
    dev_err = sum(1 for r in dev_rows if r.get("status") == "error")

    registry = {
        "ok": True,
        "generated_at": _utc_now(),
        "workspace": str(ROOT),
        "scan_source": "atlas_software_watchdog.py",
        "catalog_version": catalog.get("version") if isinstance(catalog, dict) else "",
        "summary": {
            "software_total": len(software_rows),
            "software_ok": sw_ok,
            "software_warn": sw_warn,
            "software_error": sw_err,
            "software_update_ready": sw_updates,
            "candidates_total": len(network_candidates),
            "candidates_available": sum(1 for c in network_candidates if c.get("network_available")),
            "drivers_total": len(driver_rows),
            "drivers_ok": dr_ok,
            "drivers_warn": dr_warn,
            "drivers_error": dr_err,
            "dev_total": len(dev_rows),
            "dev_ok": dev_ok,
            "dev_warn": dev_warn,
            "dev_error": dev_err,
        },
        "software": software_rows,
        "network_candidates": network_candidates,
        "drivers": driver_rows,
        "development_stack": dev_rows,
    }
    _write_json(REGISTRY_FILE, registry)
    _append_log(
        f"registry generated software={len(software_rows)} candidates={len(network_candidates)} drivers={len(driver_rows)} dev={len(dev_rows)} updates={sw_updates}"
    )
    return registry


def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS software inventory watchdog")
    parser.add_argument("--force", action="store_true", help="Force refresh of remote latest versions")
    parser.add_argument("--pretty", action="store_true", help="Pretty print output JSON")
    args = parser.parse_args()

    payload = _build_registry(force=bool(args.force))
    if args.pretty:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(payload, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
