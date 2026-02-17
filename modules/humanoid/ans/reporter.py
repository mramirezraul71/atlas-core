"""Minimal incident report + UI events + telegram notifications."""
from __future__ import annotations

import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

_REPORT_DIR: str = ""


def _report_dir() -> Path:
    global _REPORT_DIR
    if not _REPORT_DIR:
        v = os.getenv("ANS_REPORT_DIR", "").strip()
        if v:
            _REPORT_DIR = v
        else:
            base = (os.getenv("POLICY_ALLOWED_PATHS") or "C:\\ATLAS_PUSH").strip().split(",")[0].strip()
            _REPORT_DIR = str(Path(base) / "snapshots" / "ans")
    return Path(_REPORT_DIR)


def write_report(incidents: List[Dict], actions: List[Dict], summary: str = "", diagnosis: Dict = None) -> str:
    Path(_report_dir()).mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    path = _report_dir() / f"ANS_REPORT_{ts}.md"
    lines = [f"# ANS Report {ts}", "", f"Summary: {summary or 'ok'}", ""]
    if diagnosis:
        lines.append("## Diagnóstico (cerebro)")
        lines.append(diagnosis.get("summary", ""))
        if diagnosis.get("root_causes"):
            lines.append("### Causas raíz")
            for rc in diagnosis["root_causes"][:5]:
                lines.append(f"- {rc.get('check')}: {rc.get('message', '')[:80]}")
        if diagnosis.get("recommendations"):
            lines.append("### Recomendaciones")
            for r in diagnosis["recommendations"]:
                lines.append(f"- {r}")
        lines.append("")
    if incidents:
        lines.append("## Incidents")
        for i in incidents[:20]:
            lines.append(f"- [{i.get('severity')}] {i.get('check_id')}: {i.get('message')}")
        lines.append("")
    if actions:
        lines.append("## Actions")
        for a in actions[-20:]:
            lines.append(f"- {a.get('heal_id', '')}: {a.get('message', '')}")
    path.write_text("\n".join(lines), encoding="utf-8")
    return str(path)


def get_latest_report() -> str:
    d = _report_dir()
    if not d.exists():
        return ""
    files = sorted(d.glob("ANS_REPORT_*.md"), key=lambda x: x.stat().st_mtime, reverse=True)
    return str(files[0]) if files else ""


def notify_telegram(message: str, severity: str = "medium") -> bool:
    """Notifica vía Telegram y Audio usando el sistema de comunicación unificado.
    
    Args:
        message: Mensaje a enviar
        severity: Nivel de severidad (low, medium, high, critical)
    
    Returns:
        True si se envió correctamente
    """
    if os.getenv("ANS_TELEGRAM_NOTIFY", "true").strip().lower() not in ("1", "true", "yes"):
        return False
    
    notify_level = (os.getenv("ANS_NOTIFY_LEVEL") or "medium").strip().lower()
    level_order = {"low": 0, "medium": 1, "med": 1, "high": 2, "critical": 3}
    if level_order.get(severity, 0) < level_order.get(notify_level, 1):
        return False
    
    # Mapear severity a level para ops_bus
    level_map = {"low": "low", "medium": "med", "med": "med", "high": "high", "critical": "critical"}
    level = level_map.get(severity, "med")
    
    try:
        # Usar ops_bus para enviar a Telegram + Audio + otros canales
        from modules.humanoid.comms.ops_bus import emit as ops_emit
        ops_emit("ans", message, level=level)
        return True
    except Exception:
        # Fallback: intentar Telegram directo
        try:
            from modules.humanoid.comms.telegram_bridge import TelegramBridge
            from modules.humanoid.comms.ops_bus import _telegram_chat_id
            
            chat_id = _telegram_chat_id()
            if chat_id:
                bridge = TelegramBridge()
                result = bridge.send(chat_id, f"<b>ATLAS ANS</b>\n{message}")
                return result.get("ok", False)
        except Exception:
            pass
        return False


def notify_audio(message: str) -> bool:
    """Notifica vía audio (TTS).
    
    Args:
        message: Mensaje a hablar
    
    Returns:
        True si se habló correctamente
    """
    if os.getenv("OPS_AUDIO_ENABLED", "true").strip().lower() not in ("1", "true", "yes"):
        return False
    
    try:
        from modules.humanoid.voice.tts import speak
        result = speak(message[:240])
        return result.get("ok", False)
    except Exception:
        return False


def notify_all(message: str, severity: str = "medium", subsystem: str = "ans") -> Dict[str, Any]:
    """Notifica por todos los canales disponibles usando el CommsHub.
    
    Args:
        message: Mensaje a enviar
        severity: Nivel de severidad
        subsystem: Subsistema que envía
    
    Returns:
        Dict con resultado del envío
    """
    level_map = {"low": "low", "medium": "med", "med": "med", "high": "high", "critical": "critical"}
    level = level_map.get(severity, "med")
    
    try:
        from modules.humanoid.comms.ops_bus import emit as ops_emit
        return ops_emit(subsystem, message, level=level)
    except Exception as e:
        return {"ok": False, "error": str(e)}
