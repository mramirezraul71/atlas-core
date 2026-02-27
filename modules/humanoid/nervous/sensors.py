"""Sensors: collect signals from ANS checks + metrics + cuerpo connectivity."""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from .scoring import Signal, normalize_severity, severity_points


def _domain_for_check(check_id: str) -> str:
    """Dominio anatómico para ponderar puntos por envergadura."""
    cid = (check_id or "").strip().lower()
    if cid in ("api_health", "router_health", "llm_health"):
        return "brain"
    if cid in ("memory_health", "audit_health"):
        return "memory"
    if cid in ("scheduler_health",):
        return "scheduler"
    if cid in ("disk_health", "logs_health"):
        return "io"
    if cid in ("ui_health",):
        return "ui"
    if cid in ("deploy_health", "gateway_health", "evolution_health", "deps_health"):
        return "comms"
    if cid in ("nexus_services_health", "robot_camera_health"):
        return "body"
    if cid.startswith("metrics_latency:"):
        return "brain"
    return "unknown"


DOMAIN_MULTIPLIER = {
    "brain": 1.2,
    "memory": 1.0,
    "scheduler": 1.1,
    "comms": 1.1,
    "body": 1.4,
    "io": 1.0,
    "ui": 0.6,
    "unknown": 1.0,
}


def _weighted_points(severity: str, domain: str) -> int:
    base = int(severity_points(severity))
    mult = float(DOMAIN_MULTIPLIER.get((domain or "unknown").strip().lower(), 1.0))
    return int(round(base * mult))


def _fingerprint(sensor_id: str, message: str) -> str:
    m = (message or "").strip().replace("\n", " ")
    return f"{sensor_id}:{m[:80]}"


def collect_from_ans(selected_checks: Optional[List[str]] = None) -> List[Signal]:
    """
    Run ANS checks and convert failed ones into nervous signals.
    selected_checks: list of check_ids; if None uses a core set.
    """
    try:
        from modules.humanoid.ans.checks import _register_all
        _register_all()
    except Exception:
        pass

    core = selected_checks or [
        "api_health",
        "memory_health",
        "audit_health",
        "scheduler_health",
        "llm_health",
        "logs_health",
        "disk_health",
        "gateway_health",
        "router_health",
        "nexus_services_health",
        "cluster_health",
        "deploy_health",
        "ui_health",
        "deps_health",
        "evolution_health",
    ]

    out: List[Signal] = []
    try:
        from modules.humanoid.ans.registry import run_check
    except Exception:
        return out

    for cid in core:
        r = run_check(cid) or {}
        if r.get("ok", False):
            continue
        sev = normalize_severity(r.get("severity"))
        msg = r.get("message") or "check failed"
        details = r.get("details") or {}
        suggested = list(r.get("suggested_heals") or [])
        domain = _domain_for_check(cid)
        out.append(
            Signal(
                sensor_id=cid,
                subsystem="ans",
                severity=sev,
                points=_weighted_points(sev, domain),
                fingerprint=_fingerprint(cid, msg),
                message=str(msg)[:500],
                details=details if isinstance(details, dict) else {"details": str(details)[:500]},
                suggested_heals=suggested,
            )
        )
    return out


def collect_from_metrics() -> List[Signal]:
    """Detect latency spikes on critical endpoints from in-memory /metrics store."""
    out: List[Signal] = []
    try:
        from modules.humanoid.metrics import get_metrics_store
        snap = get_metrics_store().snapshot()
        lat = (snap.get("latencies") or {}) if isinstance(snap, dict) else {}
    except Exception:
        return out

    def _p95(path: str) -> Optional[float]:
        d = lat.get(f"request:{path}") or {}
        v = d.get("p95_ms")
        try:
            return float(v)
        except Exception:
            return None

    # thresholds: executive defaults, override via env later if needed
    critical_targets = ["/status", "/health", "/api/robot/status"]
    for p in critical_targets:
        p95 = _p95(p)
        if p95 is None:
            continue
        if p95 > 5000:
            sev = "critical"
        elif p95 > 2000:
            sev = "high"
        elif p95 > 800:
            sev = "med"
        else:
            continue
        sensor_id = f"metrics_latency:{p}"
        msg = f"p95 latency high for {p}: {p95:.0f}ms"
        out.append(
            Signal(
                sensor_id=sensor_id,
                subsystem="metrics",
                severity=sev,
                points=_weighted_points(sev, "brain"),
                fingerprint=_fingerprint(sensor_id, msg),
                message=msg,
                details={"p95_ms": p95, "path": p},
                suggested_heals=["rotate_logs"] if sev in ("high", "critical") else [],
            )
        )

    # Aprendizaje: persistencia DB + snapshots (si falla, es degradación seria de "memoria viva")
    try:
        from pathlib import Path
        import sqlite3

        db_path = Path(os.getenv("LEARNING_EPISODIC_DB_PATH", r"C:\ATLAS_PUSH\logs\learning_episodic.sqlite"))
        snap_dir = Path(os.getenv("LEARNING_SNAPSHOT_DIR", r"C:\ATLAS_PUSH\snapshots\learning"))
        ok_db = False
        ok_snap = False
        try:
            db_path.parent.mkdir(parents=True, exist_ok=True)
            con = sqlite3.connect(str(db_path))
            con.execute("SELECT 1")
            con.close()
            ok_db = True
        except Exception:
            ok_db = False
        try:
            snap_dir.mkdir(parents=True, exist_ok=True)
            p = snap_dir / ".probe"
            p.write_text("ok", encoding="utf-8")
            p.unlink(missing_ok=True)
            ok_snap = True
        except Exception:
            ok_snap = False
        if not (ok_db and ok_snap):
            sev = "high" if not ok_db else "med"
            sensor_id = "learning_persistence"
            msg = f"learning_persistence db={'ok' if ok_db else 'fail'} snapshots={'ok' if ok_snap else 'fail'}"
            out.append(
                Signal(
                    sensor_id=sensor_id,
                    subsystem="learning",
                    severity=sev,
                    points=_weighted_points(sev, "memory"),
                    fingerprint=_fingerprint(sensor_id, msg),
                    message=msg,
                    details={"db_path": str(db_path), "snap_dir": str(snap_dir), "db_ok": ok_db, "snap_ok": ok_snap},
                    suggested_heals=["clear_stale_locks"] if not (ok_db and ok_snap) else [],
                )
            )
    except Exception:
        pass
    return out


def collect_from_organs() -> List[Signal]:
    """
    Sensores anatómicos (órganos/actuadores):
    - manos: deps de pantalla (pyautogui/mss) para control de mouse/teclado
    - pies: driver configurado o stub
    """
    out: List[Signal] = []

    # Manos (input local)
    try:
        from modules.humanoid.screen.status import _screen_deps_ok as screen_ok

        if not screen_ok():
            sev = "med"
            sensor_id = "hands_input_health"
            msg = "hands input unavailable: missing deps (mss/pyautogui)"
            out.append(
                Signal(
                    sensor_id=sensor_id,
                    subsystem="hands",
                    severity=sev,
                    points=_weighted_points(sev, "body"),
                    fingerprint=_fingerprint(sensor_id, msg),
                    message=msg,
                    details={"deps": ["mss", "pyautogui"]},
                    suggested_heals=["install_optional_deps"],
                )
            )
    except Exception:
        pass

    # Pies (movilidad)
    try:
        import os

        driver = (os.getenv("FEET_DRIVER") or "none").strip().lower()
        if driver == "digital":
            try:
                from modules.humanoid.deps_checker import check_playwright
                pw = check_playwright()
                if not pw.get("available", False):
                    sev = "med"
                    sensor_id = "digital_feet_health"
                    msg = "digital feet enabled but playwright missing"
                    out.append(
                        Signal(
                            sensor_id=sensor_id,
                            subsystem="feet",
                            severity=sev,
                            points=_weighted_points(sev, "brain"),
                            fingerprint=_fingerprint(sensor_id, msg),
                            message=msg,
                            details={"missing_deps": pw.get("missing_deps", []), "suggested": pw.get("suggested", "")},
                            suggested_heals=["install_optional_deps"],
                        )
                    )
            except Exception:
                pass
        if driver in ("none", "", "stub"):
            # Informativo: no penaliza (0 puntos) pero queda visible como estado.
            sev = "low"
            sensor_id = "feet_driver_status"
            msg = "feet driver not configured (stub)"
            out.append(
                Signal(
                    sensor_id=sensor_id,
                    subsystem="feet",
                    severity=sev,
                    points=0,
                    fingerprint=_fingerprint(sensor_id, msg),
                    message=msg,
                    details={"driver": driver},
                    suggested_heals=[],
                )
            )
    except Exception:
        pass

    return out

