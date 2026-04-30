from __future__ import annotations

import json
import os
import re
from typing import Any, Dict, List

from modules.providers.ollama_provider import ollama_chat


def _extract_json(raw: str) -> Dict[str, Any] | None:
    text = (raw or "").strip()
    if not text:
        return None
    fenced = re.search(r"```(?:json)?\s*(\{.*\})\s*```", text, re.S)
    candidate = fenced.group(1) if fenced else text
    try:
        return json.loads(candidate)
    except Exception:
        return None


def analyze_plan(
    runtime_model: Dict[str, Any], policy: Dict[str, Any], plan: Dict[str, Any]
) -> Dict[str, Any]:
    model = (
        os.getenv("ATLAS_AUTONOMY_OLLAMA_MODEL")
        or os.getenv("OLLAMA_MODEL")
        or "qwen3:30b"
    ).strip()
    messages: List[Dict[str, str]] = [
        {
            "role": "system",
            "content": (
                "Eres el coprocesador cognitivo local de ATLAS. "
                "Debes responder solo JSON compacto con claves: "
                "risk_summary, monitoring_focus, execution_note, priority_overrides. "
                "No inventes acciones peligrosas ni saltes interlocks."
            ),
        },
        {
            "role": "user",
            "content": json.dumps(
                {
                    "policy_mode": policy.get("mode"),
                    "severity_counts": (
                        (runtime_model.get("fault_snapshot") or {}).get("severity_counts")
                    ),
                    "incidents": [
                        {
                            "component_id": event.get("component_id"),
                            "severity": event.get("severity"),
                            "fault_code": event.get("fault_code"),
                            "symptom": event.get("symptom"),
                        }
                        for event in (
                            (runtime_model.get("fault_snapshot") or {}).get("events") or []
                        )[:8]
                    ],
                    "planned_actions": [
                        {
                            "action_id": action.get("action_id"),
                            "kind": action.get("kind"),
                            "target": action.get("target"),
                            "risk": action.get("risk"),
                            "description": action.get("description"),
                        }
                        for action in (plan.get("actions") or [])[:6]
                    ],
                },
                ensure_ascii=False,
            ),
        },
    ]
    try:
        raw = ollama_chat(model, messages, timeout_sec=45)
        parsed = _extract_json(raw) or {}
        return {
            "used": True,
            "ok": bool(parsed),
            "model": model,
            "raw": raw[:3000],
            "parsed": parsed,
        }
    except Exception as exc:
        return {
            "used": False,
            "ok": False,
            "model": model,
            "error": str(exc),
            "raw": "",
            "parsed": {},
        }
