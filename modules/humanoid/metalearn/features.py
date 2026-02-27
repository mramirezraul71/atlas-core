"""Deterministic feature extraction for feedback events."""
from __future__ import annotations

import re
from typing import Any, Dict


def _bucket(s: str, max_len: int = 32) -> str:
    return (s or "unknown").strip().lower()[:max_len]


def error_category(error: Any) -> str:
    """Classify error: timeout|deps_missing|http_error|policy_denied|unknown."""
    if not error:
        return "unknown"
    e = str(error).strip().lower()
    if "timeout" in e or "timed out" in e:
        return "timeout"
    if "deps" in e or "missing" in e or "pip" in e or "import" in e:
        return "deps_missing"
    if "http" in e or "503" in e or "502" in e or "connection" in e:
        return "http_error"
    if "policy" in e or "denied" in e:
        return "policy_denied"
    return "unknown"


def extract(payload: Dict[str, Any], outcome: str = "ok", error: Any = None) -> Dict[str, Any]:
    """
    Extract deterministic features from event payload.
    Returns dict: is_repo_change, is_runtime_change, touches_update_engine, touches_shell, touches_remote,
    uses_vision, uses_web, uses_voice, error_category, file_count_changed, is_high_latency, model_family.
    """
    action = _bucket(payload.get("action_type") or payload.get("action", ""))
    ga_action = _bucket(payload.get("ga_action", ""))
    combined = f"{action}_{ga_action}" if ga_action and ga_action != "unknown" else action

    is_repo = combined in ("refactor_plan", "autofix", "add_timeout", "add_smoke_test", "update_check", "update_apply") or "repo" in combined
    is_runtime = combined in ("restart_internal_loop", "restart_component", "tune_router", "notify_owner") or "runtime" in combined
    touches_update = "update" in combined or action in ("update_check", "update_apply", "system_update")
    touches_shell = action in ("exec_command", "shell_command", "hands", "remote_hands")
    touches_remote = "remote" in combined or payload.get("node_id") or payload.get("origin_node_id")

    uses_vision = "vision" in combined or "vision" in str(payload.get("capabilities", ""))
    uses_web = "web" in combined or "web" in str(payload.get("capabilities", ""))
    uses_voice = "voice" in combined or "voice" in str(payload.get("capabilities", ""))

    err_cat = error_category(error)
    file_count = 0
    if isinstance(payload.get("changed_files"), list):
        file_count = len(payload["changed_files"])
    elif isinstance(payload.get("paths"), list):
        file_count = len(payload["paths"])

    latency_ms = payload.get("latency_ms") or payload.get("ms")
    is_high_latency = bool(latency_ms is not None and int(latency_ms) > 10000)

    model_used = (payload.get("model_used") or "") or (payload.get("model", "") or "")
    model_family = "unknown"
    if model_used:
        m = model_used.lower()
        if "llama" in m or "3b" in m:
            model_family = "fast"
        elif "chat" in m or "7b" in m:
            model_family = "chat"
        elif "code" in m or "coder" in m:
            model_family = "code"
        elif "reason" in m or "r1" in m or "14b" in m:
            model_family = "reason"
        elif "qwen" in m or "tools" in m:
            model_family = "tools"
        else:
            model_family = "chat"

    return {
        "is_repo_change": is_repo,
        "is_runtime_change": is_runtime,
        "touches_update_engine": touches_update,
        "touches_shell": touches_shell,
        "touches_remote": touches_remote,
        "uses_vision": uses_vision,
        "uses_web": uses_web,
        "uses_voice": uses_voice,
        "error_category": err_cat,
        "file_count_changed": file_count,
        "is_high_latency": is_high_latency,
        "model_family": model_family,
    }
