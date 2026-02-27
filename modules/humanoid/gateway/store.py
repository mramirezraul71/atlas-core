"""Persist gateway status and last success (JSON in logs)."""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Optional


def _state_path() -> Path:
    base = Path(os.getenv("ATLAS_PUSH_ROOT", os.getcwd()))
    return base / "logs" / "gateway_state.json"


def load_state() -> Dict[str, Any]:
    path = _state_path()
    default = {"mode": os.getenv("GATEWAY_MODE", "auto"), "last_success_ts": None, "last_success_mode": None, "last_success_target": None}
    if not path.exists():
        return default
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        default.update(data)
        return default
    except Exception:
        return default


def save_state(state: Dict[str, Any]) -> None:
    path = _state_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state, indent=2), encoding="utf-8")


def set_last_success(mode: str, target: str) -> None:
    import time
    state = load_state()
    state["last_success_ts"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    state["last_success_mode"] = mode
    state["last_success_target"] = target
    save_state(state)


def get_mode() -> str:
    return (load_state().get("mode") or os.getenv("GATEWAY_MODE", "auto")).strip().lower()


def set_mode(mode: str) -> None:
    m = mode.strip().lower()
    if m not in ("auto", "cloudflare", "tailscale", "ssh", "lan"):
        return
    state = load_state()
    state["mode"] = m
    save_state(state)


def build_gateway_status() -> Dict[str, Any]:
    """Full status for GET /gateway/status."""
    import os
    from . import detector
    enabled = os.getenv("GATEWAY_ENABLED", "").strip().lower() in ("1", "true", "yes")
    state = load_state()
    tools_raw = detector.detect_all()
    tools = {
        "cloudflared": tools_raw.get("cloudflare", {}).get("available", False),
        "tailscale": tools_raw.get("tailscale", {}).get("available", False),
        "ssh": tools_raw.get("ssh", {}).get("available", False),
    }
    return {
        "enabled": enabled,
        "mode": state.get("mode", "auto"),
        "tools": tools,
        "last_success_ts": state.get("last_success_ts"),
        "last_success_mode": state.get("last_success_mode"),
        "last_success_target": state.get("last_success_target"),
        "recommendations": [],
        "error": None,
    }
