"""Notify UI/Telegram on governance changes."""
from __future__ import annotations


def notify_mode_change(mode: str, reason: str = "") -> None:
    """Notify UI/Telegram on mode change."""
    try:
        from modules.humanoid.ans.reporter import notify_telegram
        notify_telegram(f"Governance: modo cambiado a {mode.upper()}" + (f" - {reason[:80]}" if reason else ""), "low")
    except Exception:
        pass


def notify_emergency_change(enable: bool, reason: str = "") -> None:
    """Notify on emergency stop toggle."""
    try:
        from modules.humanoid.ans.reporter import notify_telegram
        msg = f"Governance: EMERGENCY STOP {'ACTIVADO' if enable else 'DESACTIVADO'}"
        if reason:
            msg += f" - {reason[:80]}"
        notify_telegram(msg, "high" if enable else "med")
    except Exception:
        pass


def notify_blocked(action_kind: str, reason: str) -> None:
    """Notify when action blocked."""
    try:
        from modules.humanoid.ans.reporter import notify_telegram
        notify_telegram(f"Governance: bloqueado {action_kind} - {reason}", "med")
    except Exception:
        pass
