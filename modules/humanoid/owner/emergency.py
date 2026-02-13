"""Emergency mode: block deploys, remote_exec, shell; allow status/health."""
from __future__ import annotations

import os
from typing import Any, Dict

_EMERGENCY: bool = False


def _env_bool(key: str, default: bool) -> bool:
    v = os.getenv(key, "").strip().lower()
    if v in ("1", "true", "yes", "y", "on"):
        return True
    if v in ("0", "false", "no", "n", "off"):
        return False
    return default


def is_emergency() -> bool:
    global _EMERGENCY
    if _env_bool("EMERGENCY_MODE", False):
        return True
    return _EMERGENCY


def set_emergency(enable: bool, reason: str = "") -> None:
    global _EMERGENCY
    _EMERGENCY = bool(enable)


def set_emergency_from_env() -> None:
    global _EMERGENCY
    _EMERGENCY = _env_bool("EMERGENCY_MODE", False)


def _block_deploy() -> bool:
    return _env_bool("EMERGENCY_BLOCK_DEPLOY", True)


def _block_remote_exec() -> bool:
    return _env_bool("EMERGENCY_BLOCK_REMOTE_EXEC", True)


def _block_shell() -> bool:
    return _env_bool("EMERGENCY_BLOCK_SHELL", True)


def _allow_status() -> bool:
    return _env_bool("EMERGENCY_ALLOW_STATUS", True)


def is_action_blocked(action: str) -> bool:
    """
    True if emergency is on and action is blocked.
    Allowed when emergency: status, health, metrics, version.
    """
    if not is_emergency():
        return False
    a = (action or "").strip().lower()
    if _allow_status() and a in ("status", "health", "metrics", "version"):
        return False
    if _block_deploy() and a in ("deploy_apply", "deploy_bluegreen", "update_apply"):
        return True
    if _block_remote_exec() and a in ("remote_hands", "remote_web", "remote_vision", "remote_voice", "remote_execute"):
        return True
    if _block_shell() and a in ("shell_command", "exec_command", "hands"):
        return True
    if _block_deploy() and ("deploy" in a and ("apply" in a or "bluegreen" in a)):
        return True
    if _block_remote_exec() and "remote" in a:
        return True
    if _block_shell() and ("shell" in a or "exec_command" in a):
        return True
    return False


def get_emergency_state() -> Dict[str, Any]:
    return {
        "enabled": is_emergency(),
        "block_deploy": _block_deploy(),
        "block_remote_exec": _block_remote_exec(),
        "block_shell": _block_shell(),
        "allow_status": _allow_status(),
    }
