from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Tuple

from tools.atlas_arm_state import is_arm_isolated

BASE_DIR = Path(__file__).resolve().parent.parent
LOG_DIR = BASE_DIR / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

DECISION_LOG = LOG_DIR / "atlas_cortex_decisions.log"
ALERT_LOG = LOG_DIR / "atlas_cortex_alerts.log"


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _log(path: Path, payload: Dict[str, Any]) -> None:
    line = json.dumps(payload, ensure_ascii=False)
    with path.open("a", encoding="utf-8") as f:
        f.write(line + "\n")


@dataclass
class DecisionResult:
    accepted: bool
    action: str
    reason: str
    commands: List[Dict[str, Any]]
    dispatch_results: List[Dict[str, Any]]


def validate_vision_event(payload: Dict[str, Any]) -> Tuple[bool, str]:
    required = ["event_id", "source", "timestamp_iso", "detections"]
    for k in required:
        if k not in payload:
            return False, f"missing_field:{k}"
    if payload.get("source") != "rauli-vision":
        return False, "invalid_source"
    detections = payload.get("detections")
    if not isinstance(detections, list) or len(detections) == 0:
        return False, "invalid_detections"
    for i, item in enumerate(detections):
        if not isinstance(item, dict):
            return False, f"invalid_detection:{i}"
        if "product_id" not in item or "count" not in item:
            return False, f"missing_detection_fields:{i}"
    return True, "ok"


def _map_detection_to_adjustment(event_id: str, item: Dict[str, Any]) -> Dict[str, Any]:
    action_hint = str(item.get("action_hint") or "ajuste").lower()
    allowed = {"entrada", "salida", "ajuste", "merma"}
    movement = action_hint if action_hint in allowed else "ajuste"

    qty = float(item.get("count") or 0)
    if movement in {"salida", "merma"}:
        qty = -abs(qty)
    elif movement == "entrada":
        qty = abs(qty)

    note = (
        f"ATLAS_CORTEX event={event_id} source=rauli-vision "
        f"confidence={item.get('confidence', 'n/a')} quality={item.get('quality_score', 'n/a')}"
    )
    return {
        "product_id": str(item["product_id"]),
        "quantity": qty,
        "type": movement,
        "notes": note,
    }


def build_panaderia_commands(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    event_id = str(payload.get("event_id") or "unknown")
    out: List[Dict[str, Any]] = []
    for item in payload.get("detections", []):
        out.append(_map_detection_to_adjustment(event_id, item))
    return out


def _must_escalate(payload: Dict[str, Any]) -> Tuple[bool, str]:
    anomaly = payload.get("anomaly") or {}
    if bool(anomaly.get("detected")):
        sev = str(anomaly.get("severity") or "low").lower()
        if sev in {"high", "critical"}:
            return True, f"anomaly_{sev}"
    for d in payload.get("detections", []):
        quality = d.get("quality_score")
        confidence = d.get("confidence")
        if quality is not None and float(quality) < 0.40:
            return True, "low_quality_score"
        if confidence is not None and float(confidence) < 0.35:
            return True, "low_confidence"
    return False, "ok"


def _dispatch_to_panaderia(commands: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    base = (os.getenv("PANADERIA_API_BASE") or "http://127.0.0.1:3001").rstrip("/")
    token = os.getenv("PANADERIA_BEARER_TOKEN", "").strip()
    endpoint = base + "/api/inventory/adjustment"
    out: List[Dict[str, Any]] = []

    for cmd in commands:
        body = json.dumps(cmd).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if token:
            headers["Authorization"] = f"Bearer {token}"
        req = urllib.request.Request(endpoint, data=body, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=8) as resp:
                raw = resp.read().decode("utf-8", errors="replace")
                out.append(
                    {
                        "ok": True,
                        "status": int(resp.status),
                        "endpoint": endpoint,
                        "product_id": cmd.get("product_id"),
                        "response": raw[:400],
                    }
                )
        except urllib.error.HTTPError as e:
            msg = e.read().decode("utf-8", errors="replace") if hasattr(e, "read") else str(e)
            out.append(
                {
                    "ok": False,
                    "status": int(e.code),
                    "endpoint": endpoint,
                    "product_id": cmd.get("product_id"),
                    "error": msg[:400],
                }
            )
        except Exception as e:
            out.append(
                {
                    "ok": False,
                    "status": 0,
                    "endpoint": endpoint,
                    "product_id": cmd.get("product_id"),
                    "error": f"{type(e).__name__}: {e}",
                }
            )
    return out


def process_vision_event(payload: Dict[str, Any], apply_changes: bool = False) -> DecisionResult:
    ok, reason = validate_vision_event(payload)
    if not ok:
        _log(
            DECISION_LOG,
            {"ts": _now_iso(), "accepted": False, "action": "reject", "reason": reason},
        )
        return DecisionResult(False, "reject", reason, [], [])

    escalate, esc_reason = _must_escalate(payload)
    commands = build_panaderia_commands(payload)

    if escalate:
        alert = {
            "ts": _now_iso(),
            "event_id": payload.get("event_id"),
            "severity": (payload.get("anomaly") or {}).get("severity", "high"),
            "reason": esc_reason,
            "commands_suggested": commands,
        }
        _log(ALERT_LOG, alert)
        _log(
            DECISION_LOG,
            {
                "ts": _now_iso(),
                "accepted": True,
                "action": "escalate_only",
                "reason": esc_reason,
                "event_id": payload.get("event_id"),
            },
        )
        return DecisionResult(True, "escalate_only", esc_reason, commands, [])

    dispatch_results: List[Dict[str, Any]] = []
    if apply_changes:
        if is_arm_isolated("panaderia"):
            reason = "panaderia_isolated"
            _log(
                ALERT_LOG,
                {
                    "ts": _now_iso(),
                    "event_id": payload.get("event_id"),
                    "severity": "high",
                    "reason": reason,
                    "commands_blocked": commands,
                },
            )
            _log(
                DECISION_LOG,
                {
                    "ts": _now_iso(),
                    "accepted": True,
                    "action": "isolate_panaderia",
                    "reason": reason,
                    "event_id": payload.get("event_id"),
                    "commands_count": len(commands),
                },
            )
            return DecisionResult(True, "isolate_panaderia", reason, commands, [])
        dispatch_results = _dispatch_to_panaderia(commands)

    _log(
        DECISION_LOG,
        {
            "ts": _now_iso(),
            "accepted": True,
            "action": "apply_adjustments" if apply_changes else "dry_run_adjustments",
            "reason": "validated",
            "event_id": payload.get("event_id"),
            "commands_count": len(commands),
            "dispatch_count": len(dispatch_results),
        },
    )
    return DecisionResult(
        True,
        "apply_adjustments" if apply_changes else "dry_run_adjustments",
        "validated",
        commands,
        dispatch_results,
    )
