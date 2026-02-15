from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Dict, Optional


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def execute_approved(item: Dict[str, Any], *, approval_id: str, resolved_by: str = "api") -> Dict[str, Any]:
    """
    Ejecuta la acción asociada a una aprobación ya aprobada.
    Best-effort, devuelve detalles para auditoría/OPS.
    """
    action = (item.get("action") or "").strip().lower()
    payload = item.get("payload") or {}

    if action == "screen_act_destructive":
        requested_action = str(payload.get("requested_action") or "click")
        req_payload = dict(payload.get("requested_payload") or {})
        # Forzar bypass de approval y evidencia
        req_payload["approval_granted"] = True
        req_payload["record_evidence"] = True
        # Asegurar confirm_text/expected_window/expected_process si estaban en aprobación
        for k in ("confirm_text", "expected_window", "expected_process"):
            if payload.get(k) and not req_payload.get(k):
                req_payload[k] = payload.get(k)

        t0 = time.perf_counter()
        try:
            from modules.humanoid.screen.actions import execute_action

            r = execute_action(requested_action, req_payload)
            ms = int((time.perf_counter() - t0) * 1000)
            # Evidencia post-ejecución por OPS
            try:
                ev = r.get("evidence") or {}
                after = ev.get("after")
                before = ev.get("before")
                from modules.humanoid.comms.ops_bus import emit

                if before:
                    emit("hands", "Evidencia BEFORE (acción aprobada).", level="info", evidence_path=str(before))
                if after:
                    emit("hands", "Evidencia AFTER (acción aprobada).", level="info", evidence_path=str(after))
            except Exception:
                pass
            return {"ok": bool(r.get("ok")), "action": action, "ms": ms, "result": r, "error": r.get("error"), "approval_id": approval_id}
        except Exception as e:
            ms = int((time.perf_counter() - t0) * 1000)
            return {"ok": False, "action": action, "ms": ms, "error": str(e)[:200], "approval_id": approval_id}

    if action == "shell_exec":
        cmd = str(payload.get("command") or "")
        cwd = str(payload.get("cwd") or "") or None
        t0 = time.perf_counter()
        try:
            from modules.humanoid.hands.safe_shell import SafeShellExecutor

            r = SafeShellExecutor().run(cmd, cwd=cwd, timeout_sec=90, approval_granted=True)
            ms = int((time.perf_counter() - t0) * 1000)
            return {"ok": bool(r.get("ok")), "action": action, "ms": ms, "result": r, "error": r.get("error"), "approval_id": approval_id}
        except Exception as e:
            ms = int((time.perf_counter() - t0) * 1000)
            return {"ok": False, "action": action, "ms": ms, "error": str(e)[:200], "approval_id": approval_id}

    return {"ok": False, "action": action, "error": "no_executor_for_action", "approval_id": approval_id}

