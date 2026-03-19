"""Canonical service URL resolution for PUSH, NEXUS and Robot."""
from __future__ import annotations

import os


def _truthy(value: str | None, default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "y", "on")


def _clean(value: str | None) -> str:
    return (value or "").strip().rstrip("/")


def get_nexus_base() -> str:
    return _clean(os.getenv("NEXUS_BASE_URL")) or "http://127.0.0.1:8000"


def get_nexus_ws_url(path: str = "/ws") -> str:
    base = get_nexus_base().replace("http://", "ws://").replace("https://", "wss://")
    suffix = path if path.startswith("/") else f"/{path}"
    return base + suffix


def get_nexus_enabled() -> bool:
    flag = os.getenv("NEXUS_ENABLED")
    if flag is not None:
        return _truthy(flag)
    return bool(_clean(os.getenv("NEXUS_BASE_URL")))


def get_robot_api_base() -> str:
    return (
        _clean(os.getenv("NEXUS_ROBOT_API_URL"))
        or _clean(os.getenv("ROBOT_BASE_URL"))
        or "http://127.0.0.1:8002"
    )


def get_robot_ui_base(default_to_api: bool = False) -> str:
    explicit = (
        _clean(os.getenv("NEXUS_ROBOT_URL"))
        or _clean(os.getenv("ROBOT_UI_BASE_URL"))
        or _clean(os.getenv("ROBOT_BASE_URL"))
    )
    if explicit:
        return explicit
    if default_to_api:
        return get_robot_api_base()
    return "http://127.0.0.1:5174"


def get_nexus_timeout() -> float:
    return float(os.getenv("NEXUS_TIMEOUT", "30"))


def get_camera_proxy_timeout() -> float:
    return float(os.getenv("NEXUS_CAMERA_PROXY_TIMEOUT", "45"))
