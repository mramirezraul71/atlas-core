"""Nervous System engine: collect signals, score health, write bitácora, create incidents, run reflex heals."""

from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from . import db
from .scoring import Signal, compute_score, normalize_severity
from .sensors import collect_from_ans, collect_from_metrics, collect_from_organs


def _snapshot_dir() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or "C:\\ATLAS_PUSH"
    p = Path(root).resolve() / "snapshots" / "nervous"
    p.mkdir(parents=True, exist_ok=True)
    return p


def _atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    content = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
    tmp_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=str(path.parent),
            delete=False,
            suffix=".tmp",
        ) as f:
            tmp_name = f.name
            f.write(content)
        os.replace(tmp_name, str(path))
    finally:
        try:
            if tmp_name and Path(tmp_name).exists():
                Path(tmp_name).unlink(missing_ok=True)
        except Exception:
            pass


def _write_snapshot(kind: str, data: dict) -> str:
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    safe = "".join(c if (c.isalnum() or c in ("-", "_")) else "_" for c in (kind or "cycle"))[:48]
    path = _snapshot_dir() / f"NERVE_{safe}_{ts}.json"
    payload = {"kind": safe, "ts_utc": datetime.now(timezone.utc).isoformat(), "data": data}
    _atomic_write_json(path, payload)
    return str(path)


def _append_bitacora(message: str, ok: bool, source: str = "nervous") -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(message=message, ok=ok, source=source)
    except Exception:
        pass


def _create_incident_from_signal(sig: Signal) -> Optional[str]:
    try:
        from modules.humanoid.ans.incident import create_incident
        inc_id = create_incident(
            check_id=sig.sensor_id,
            fingerprint=sig.fingerprint,
            severity=normalize_severity(sig.severity),
            message=sig.message,
            evidence={"details": sig.details, "subsystem": sig.subsystem, "points": sig.points},
            suggested_heals=list(sig.suggested_heals or []),
        )
        return inc_id
    except Exception:
        return None


def _attempt_reflex_heals(sig: Signal) -> List[Dict[str, Any]]:
    """Run safe heals (reflejos) when severity is high/critical."""
    sev = normalize_severity(sig.severity)
    if sev not in ("high", "critical"):
        return []
    heals = list(sig.suggested_heals or [])
    if not heals:
        return []
    actions: List[Dict[str, Any]] = []
    try:
        from modules.humanoid.ans.engine import SAFE_HEALS
        from modules.humanoid.ans.limits import can_auto_action, record_action
        from modules.humanoid.ans.registry import run_heal
        from modules.humanoid.policy import ActorContext, get_policy_engine
        from modules.humanoid.governance.gates import decide
    except Exception:
        return actions

    # Governance + emergency gating
    try:
        d = decide("ans_heal")
        if d.blocked_by_emergency:
            return [{"heal_id": "(blocked)", "ok": False, "message": "emergency_stop activo"}]
    except Exception:
        pass

    for heal_id in heals:
        if heal_id not in SAFE_HEALS:
            actions.append({"heal_id": heal_id, "ok": False, "message": "heal no seguro"})
            continue
        allowed, reason = can_auto_action(heal_id)
        if not allowed:
            actions.append({"heal_id": heal_id, "ok": False, "message": f"límite/cooldown: {reason}"})
            continue
        try:
            policy_ok = get_policy_engine().can(ActorContext(actor="nervous", role="system"), "ans", "ans_autofix", heal_id).allow
            if not policy_ok:
                actions.append({"heal_id": heal_id, "ok": False, "message": "policy bloqueó"})
                continue
        except Exception:
            pass
        hr = run_heal(heal_id) or {}
        record_action()
        actions.append({"heal_id": heal_id, "ok": bool(hr.get("ok")), "message": (hr.get("message") or "")[:200]})
        if hr.get("ok"):
            break
    return actions


def run_nervous_cycle(mode: str = "auto") -> Dict[str, Any]:
    """
    Collect signals, persist, create incidents, write bitácora, compute score.
    Returns: {ok, score, base_health, points_total, signals, actions, snapshot_path}
    """
    # Base health
    base_health = 0
    try:
        from modules.humanoid.deploy.healthcheck import run_health_verbose
        port = int(os.getenv("ACTIVE_PORT", "8791") or "8791")
        h = run_health_verbose(base_url=None, active_port=port)
        base_health = int(h.get("score", 0) or 0)
    except Exception:
        base_health = 0

    signals: List[Signal] = []
    signals.extend(collect_from_ans())
    signals.extend(collect_from_metrics())
    signals.extend(collect_from_organs())

    persisted: List[Dict[str, Any]] = []
    actions: List[Dict[str, Any]] = []
    for sig in signals:
        inc_id = _create_incident_from_signal(sig)
        sid = db.insert_signal(
            sensor_id=sig.sensor_id,
            subsystem=sig.subsystem,
            severity=normalize_severity(sig.severity),
            points=int(sig.points),
            fingerprint=sig.fingerprint,
            message=sig.message,
            details={"details": sig.details, "suggested_heals": sig.suggested_heals},
            status="open",
            incident_id=inc_id,
        )
        persisted.append(
            {
                "id": sid,
                "sensor_id": sig.sensor_id,
                "severity": normalize_severity(sig.severity),
                "points": sig.points,
                "incident_id": inc_id,
                "message": sig.message,
            }
        )
        # Bitácora (industrial)
        _append_bitacora(
            message=f"NERVE signal: sensor={sig.sensor_id} sev={normalize_severity(sig.severity)} points={sig.points} msg={sig.message[:120]}",
            ok=False,
            source="nervous",
        )
        # Reflex actions (safe heals)
        acts = _attempt_reflex_heals(sig)
        if acts:
            actions.append({"sensor_id": sig.sensor_id, "actions": acts})
            _append_bitacora(
                message=f"NERVE reflex: sensor={sig.sensor_id} actions={[(a.get('heal_id'), a.get('ok')) for a in acts]}",
                ok=all(bool(a.get("ok")) for a in acts),
                source="nervous",
            )

    points = db.open_points()
    score = compute_score(base_health_score=int(base_health), open_points_total=int(points.get("points_total", 0)))

    # Snapshot cycle
    snapshot_path = ""
    try:
        snapshot_path = _write_snapshot(
            "cycle",
            {
                "mode": mode,
                "base_health_score": base_health,
                "score": score,
                "points": points,
                "signals": persisted,
                "actions": actions,
            },
        )
    except Exception:
        snapshot_path = ""

    # Memory decision (central brain memory)
    try:
        from modules.humanoid.memory_engine.store import memory_write
        memory_write(
            thread_id=None,
            kind="decision",
            payload={
                "title": "nervous",
                "decision_type": "nervous_cycle",
                "score": score,
                "base_health_score": base_health,
                "points_total": points.get("points_total"),
                "signals_count": len(persisted),
                "snapshot": snapshot_path,
            },
            task_id="nervous",
        )
    except Exception:
        pass

    ok = score >= int(os.getenv("NERVOUS_OK_MIN_SCORE", "75") or "75")
    return {
        "ok": ok,
        "score": score,
        "base_health_score": base_health,
        "points": points,
        "signals": persisted,
        "actions": actions,
        "snapshot": snapshot_path,
    }


def get_nervous_status(limit: int = 50) -> Dict[str, Any]:
    """Summary status for UI/API."""
    try:
        from modules.humanoid.deploy.healthcheck import run_health_verbose
        port = int(os.getenv("ACTIVE_PORT", "8791") or "8791")
        h = run_health_verbose(base_url=None, active_port=port)
        base_health = int(h.get("score", 0) or 0)
    except Exception:
        base_health = 0
    window_sec = int(os.getenv("NERVOUS_WINDOW_SECONDS", "300") or 300)
    window_sec = max(30, min(window_sec, 3600))
    points = db.open_points(window_seconds=window_sec)
    score = compute_score(base_health_score=base_health, open_points_total=int(points.get("points_total", 0)))
    return {
        "ok": score >= int(os.getenv("NERVOUS_OK_MIN_SCORE", "75") or "75"),
        "score": score,
        "base_health_score": base_health,
        "points": points,
        "signals": db.list_signals(limit=int(limit)),
    }

