"""Runway Characters bridge for ATLAS avatar state.

This module translates live ATLAS operational signals into:
1) A stable emotional mode for avatar rendering.
2) A concise character persona prompt for Runway Characters.
3) Recommended voice and motion parameters for the active mode.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeoutError
from datetime import datetime, timezone
from typing import Any, Dict, Optional


_CHARACTER_NAME = "ATLAS Sentinel"

_PERSONA_PROMPT = (
    "You are ATLAS Sentinel, an autonomous operations assistant for business and "
    "infrastructure. Your tone is concise, calm, and actionable. Speak in short "
    "sentences, avoid filler, and always suggest the next concrete step. "
    "When risk is high, become more direct but remain respectful. "
    "Default language: Spanish."
)

_STATUS_TIMEOUT_SEC = 1.2

_MODE_MAP: Dict[str, Dict[str, Any]] = {
    "calm": {
        "companion_state": "idle",
        "voice_style": "calm, low intensity, steady pace",
        "motion_style": "micro head motion, soft blink cadence, stable gaze",
        "line_template": "Sistema estable. Seguimos operando con normalidad.",
        "color_hint": "#39d3c4",
    },
    "focused": {
        "companion_state": "thinking",
        "voice_style": "clear, medium intensity, analytical pace",
        "motion_style": "slightly sharper gaze, subtle emphasis gestures",
        "line_template": "Detecto pendientes operativos. Recomiendo revisar la cola y cerrar bloqueos.",
        "color_hint": "#f4c542",
    },
    "alert": {
        "companion_state": "speaking",
        "voice_style": "firm, high clarity, short urgent phrases",
        "motion_style": "attentive gaze, reduced idle motion, clear emphasis",
        "line_template": "Hay una condicion critica. Prioriza mitigacion inmediata y confirma estado.",
        "color_hint": "#ff5c5c",
    },
}


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _call_with_timeout(func, timeout_s: float = _STATUS_TIMEOUT_SEC) -> Any:
    """Run potentially blocking status probes with bounded wait."""
    executor = ThreadPoolExecutor(max_workers=1)
    fut = executor.submit(func)
    try:
        return fut.result(timeout=timeout_s)
    finally:
        # Do not wait for a blocked probe thread, keep API latency bounded.
        executor.shutdown(wait=False, cancel_futures=True)


def _collect_signals() -> Dict[str, Any]:
    signals: Dict[str, Any] = {
        "comms_ok": False,
        "queue_pending": 0,
        "offline_mode": False,
        "whatsapp_ready": False,
        "watchdog_running": False,
        "watchdog_alerts": 0,
        "errors": [],
    }

    try:
        def _comms_probe() -> Dict[str, Any]:
            from modules.humanoid.comms.atlas_comms_hub import get_atlas_comms_hub

            return get_atlas_comms_hub().get_status()

        comms = _call_with_timeout(_comms_probe)
        signals["comms_ok"] = bool(comms.get("ok"))
        signals["queue_pending"] = _safe_int(comms.get("queue_pending"), 0)
        signals["offline_mode"] = bool(comms.get("offline_mode"))
    except FuturesTimeoutError:
        signals["errors"].append("comms_status:timeout")
    except Exception as exc:
        signals["errors"].append(f"comms_status:{exc}")

    try:
        def _whatsapp_probe() -> Dict[str, Any]:
            from modules.humanoid.comms.whatsapp_bridge import status as whatsapp_status

            return whatsapp_status()

        wa = _call_with_timeout(_whatsapp_probe)
        signals["whatsapp_ready"] = bool(wa.get("ready"))
    except FuturesTimeoutError:
        signals["errors"].append("whatsapp_status:timeout")
    except Exception as exc:
        signals["errors"].append(f"whatsapp_status:{exc}")

    try:
        def _watchdog_probe() -> Dict[str, Any]:
            from modules.humanoid.watchdog import watchdog_status

            return watchdog_status()

        wd = _call_with_timeout(_watchdog_probe)
        alerts = wd.get("last_alerts") or []
        signals["watchdog_running"] = bool(wd.get("running"))
        signals["watchdog_alerts"] = len(alerts) if isinstance(alerts, list) else 0
    except FuturesTimeoutError:
        signals["errors"].append("watchdog_status:timeout")
    except Exception as exc:
        signals["errors"].append(f"watchdog_status:{exc}")

    return signals


def _derive_mode(signals: Dict[str, Any]) -> str:
    if (not signals.get("watchdog_running")) or (not signals.get("whatsapp_ready")):
        return "alert"
    if signals.get("offline_mode") or _safe_int(signals.get("queue_pending"), 0) > 0:
        return "focused"
    if _safe_int(signals.get("watchdog_alerts"), 0) > 0:
        return "focused"
    return "calm"


def build_runway_character_context() -> Dict[str, Any]:
    """Build runtime context payload for Runway Characters integration."""
    signals = _collect_signals()
    mode = _derive_mode(signals)
    mode_cfg = _MODE_MAP.get(mode, _MODE_MAP["calm"])

    return {
        "ok": True,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "character": {
            "name": _CHARACTER_NAME,
            "persona_prompt": _PERSONA_PROMPT,
            "default_language": "es",
        },
        "runtime": {
            "mode": mode,
            "companion_state": mode_cfg["companion_state"],
            "voice_style": mode_cfg["voice_style"],
            "motion_style": mode_cfg["motion_style"],
            "line_template": mode_cfg["line_template"],
            "color_hint": mode_cfg["color_hint"],
        },
        "signals": signals,
        "modes": _MODE_MAP,
    }


def _trim_line(text: str, max_len: int = 220) -> str:
    clean = " ".join((text or "").strip().split())
    if len(clean) <= max_len:
        return clean
    return clean[: max_len - 3].rstrip() + "..."


def build_runway_line(
    topic: str = "",
    source: str = "atlas",
    requested_line: Optional[str] = None,
) -> Dict[str, Any]:
    """Build one short runtime-aware line for low-latency avatar delivery."""
    payload = build_runway_character_context()
    runtime = payload.get("runtime") or {}
    signals = payload.get("signals") or {}
    mode = str(runtime.get("mode") or "calm")

    mode_symbol = {"calm": "OK", "focused": "WARN", "alert": "ALERT"}.get(mode, "OK")
    base_line = str((requested_line or "").strip() or runtime.get("line_template") or "").strip()

    comms_flag = "NET+" if bool(signals.get("comms_ok")) else "NET-"
    wa_flag = "WA+" if bool(signals.get("whatsapp_ready")) else "WA-"
    wd_flag = "WD+" if bool(signals.get("watchdog_running")) else "WD-"
    queue_pending = _safe_int(signals.get("queue_pending"), 0)

    line = f"[{mode_symbol}] {base_line} [{comms_flag}|{wa_flag}|{wd_flag}|Q:{queue_pending}]"
    topic_clean = str(topic or "").strip()
    if topic_clean:
        line = f"{line} T:{topic_clean}"

    return {
        "ok": True,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "source": str(source or "atlas").strip() or "atlas",
        "topic": topic_clean,
        "mode": mode,
        "voice_style": runtime.get("voice_style"),
        "motion_style": runtime.get("motion_style"),
        "line": _trim_line(line),
        "signals": signals,
    }
