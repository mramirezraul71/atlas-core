"""Puente Brain <-> Workspace para supervisión operativa."""
from __future__ import annotations

import json
import logging
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any
from uuid import uuid4

from .models import Command

if TYPE_CHECKING:
    from .audit_log import AuditLog
    from .command_router import CommandRouter

logger = logging.getLogger("atlas.brain.workspace_bridge")


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class WorkspaceBridge:
    """Canal de coherencia causal con el operador técnico (Workspace)."""

    def __init__(
        self,
        audit_log: AuditLog | None = None,
        max_decisions: int = 500,
        workspace_base_url: str = "http://127.0.0.1:8791",
    ) -> None:
        self._audit = audit_log
        self._max = max(50, int(max_decisions))
        self._workspace_base_url = workspace_base_url.rstrip("/")
        self._lock = threading.RLock()
        self._decisions: deque[dict[str, Any]] = deque(maxlen=self._max)
        self._feedback: dict[str, dict[str, Any]] = {}
        self._feedback_stream: deque[dict[str, Any]] = deque(maxlen=self._max)
        self._anomalies: deque[dict[str, Any]] = deque(maxlen=self._max)

    def emit_decision(
        self,
        command: Command,
        *,
        authorized: bool,
        stage: str = "decision",
        reason: str | None = None,
    ) -> dict[str, Any]:
        confidence = self._estimate_confidence(command, authorized=authorized)
        rec = {
            "decision_id": str(uuid4()),
            "ts_utc": _utc_now(),
            "stage": stage,
            "authorized": bool(authorized),
            "reason": reason,
            "command": {
                "target": command.target,
                "action": command.action,
                "params": dict(command.params),
            },
            "tipo_decision": "final" if authorized else "preliminar",
            "accion": f"{command.target}.{command.action}",
            "nivel_confianza": confidence,
            "razon": reason or "pending_authorization",
        }
        with self._lock:
            self._decisions.append(rec)
        self._post_workspace_decision(rec)
        if self._audit:
            self._audit.write("workspace_decision_emitted", **rec)
        return rec

    def _estimate_confidence(self, command: Command, *, authorized: bool) -> float:
        base = 0.55 if not authorized else 0.8
        if command.target == "healing":
            base += 0.1
        if command.target == "quant" and command.action in {"submit", "open_trade"}:
            base -= 0.1
        return round(max(0.05, min(0.99, base)), 3)

    def _post_workspace_decision(self, rec: dict[str, Any]) -> None:
        payload = {
            "timestamp": rec.get("ts_utc"),
            "decision_id": rec.get("decision_id"),
            "tipo_decision": rec.get("tipo_decision"),
            "accion": rec.get("accion"),
            "nivel_confianza": rec.get("nivel_confianza"),
            "razon": rec.get("razon"),
            "authorized": rec.get("authorized"),
        }
        body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
        req = urllib.request.Request(
            f"{self._workspace_base_url}/api/workspace/decision",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=1.0) as resp:
                _ = resp.read()
        except (urllib.error.URLError, TimeoutError, OSError):
            # El bridge no puede detener al Brain si Workspace no responde.
            pass

    def receive_operator_feedback(
        self,
        decision_id: str | None = None,
        *,
        approved: bool | None = None,
        status: str | None = None,
        timeout_sec: float = 30.0,
        note: str = "",
        operator_id: str = "operator",
    ) -> dict[str, Any]:
        if approved is None and status is None:
            return self.wait_for_operator_feedback(decision_id=decision_id, timeout_sec=timeout_sec)

        normalized = (status or ("APPROVED" if approved else "REJECTED")).upper()
        if normalized not in {"APPROVED", "REJECTED", "MODIFIED"}:
            normalized = "REJECTED"
        rec = {
            "decision_id": decision_id,
            "approved": normalized in {"APPROVED", "MODIFIED"},
            "status": normalized,
            "note": note,
            "operator_id": operator_id,
            "ts_utc": _utc_now(),
        }
        with self._lock:
            if decision_id:
                self._feedback[decision_id] = rec
            self._feedback_stream.append(rec)
        if self._audit:
            self._audit.write("operator_feedback_received", **rec)
        return rec

    def wait_for_operator_feedback(
        self,
        decision_id: str | None,
        timeout_sec: float = 30.0,
        poll_sec: float = 0.1,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.1, timeout_sec)
        while time.monotonic() < deadline:
            with self._lock:
                rec = self._feedback.get(decision_id) if decision_id else None
                if rec is None and self._feedback_stream:
                    rec = self._feedback_stream[-1]
            if rec is not None:
                return {"ok": True, "decision_id": decision_id, **rec}
            time.sleep(max(0.01, poll_sec))
        timeout_rec = {
            "ok": True,
            "decision_id": decision_id,
            "approved": True,
            "status": "APPROVED",
            "error": "operator_feedback_timeout_default_approved",
        }
        if self._audit:
            self._audit.write("operator_feedback_timeout", decision_id=decision_id, timeout_sec=timeout_sec)
        return timeout_rec

    def analyze_execution_evidence(
        self,
        command: Command,
        execution_result: dict[str, Any],
        *,
        expected_keys: list[str] | None = None,
    ) -> dict[str, Any]:
        reasons: list[str] = []
        if not execution_result.get("ok", False):
            reasons.append("execution_not_ok")
        if expected_keys:
            missing = [k for k in expected_keys if k not in execution_result]
            if missing:
                reasons.append(f"missing_expected_keys:{','.join(missing)}")

        anomaly = bool(reasons)
        rec = {
            "ts_utc": _utc_now(),
            "anomaly": anomaly,
            "reasons": reasons,
            "command": {"target": command.target, "action": command.action},
            "execution_result": dict(execution_result),
        }
        if anomaly:
            with self._lock:
                self._anomalies.append(rec)
            if self._audit:
                self._audit.write("execution_anomaly_detected", **rec)
        else:
            if self._audit:
                self._audit.write("execution_evidence_ok", command=rec["command"])
        return rec

    def trigger_healing_cascade(
        self,
        router: CommandRouter,
        *,
        reason: str,
        evidence: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        diag = router.route(Command(target="healing", action="diagnose", params={"reason": reason, "evidence": evidence or {}}))
        recover = router.route(Command(target="healing", action="recover", params={"reason": reason}))
        rec = {
            "ts_utc": _utc_now(),
            "reason": reason,
            "diagnose": diag,
            "recover": recover,
        }
        if self._audit:
            self._audit.write("healing_cascade_triggered", **rec)
        return rec

    def metrics(self) -> dict[str, Any]:
        with self._lock:
            return {
                "decisions_emitted": len(self._decisions),
                "feedback_received": len(self._feedback),
                "anomalies_detected": len(self._anomalies),
            }

