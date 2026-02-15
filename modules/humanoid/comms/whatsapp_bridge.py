"""WhatsApp bridge (opcional) vía Twilio.

Este módulo NO requiere dependencias externas; usa HTTPS (urllib).

Variables esperadas (idealmente cargadas desde Bóveda con dotenv):
- WHATSAPP_ENABLED=true|false
- TWILIO_ACCOUNT_SID
- TWILIO_AUTH_TOKEN
- TWILIO_WHATSAPP_FROM   (ej. "whatsapp:+14155238886")
- TWILIO_WHATSAPP_TO     (ej. "whatsapp:+53XXXXXXXXX")
"""
from __future__ import annotations

import os
from typing import Any, Dict


def _get(name: str) -> str:
    return (os.getenv(name) or "").strip()


def status() -> Dict[str, Any]:
    sid = bool(_get("TWILIO_ACCOUNT_SID"))
    tok = bool(_get("TWILIO_AUTH_TOKEN"))
    wfrom = bool(_get("TWILIO_WHATSAPP_FROM"))
    wto = bool(_get("TWILIO_WHATSAPP_TO"))
    enabled = sid and tok and wfrom and wto and (os.getenv("WHATSAPP_ENABLED", "false").strip().lower() in ("1", "true", "yes"))
    missing = []
    if not sid:
        missing.append("TWILIO_ACCOUNT_SID")
    if not tok:
        missing.append("TWILIO_AUTH_TOKEN")
    if not wfrom:
        missing.append("TWILIO_WHATSAPP_FROM")
    if not wto:
        missing.append("TWILIO_WHATSAPP_TO")
    return {"enabled": enabled, "missing": missing, "provider": "twilio" if enabled else None}


def send_text(text: str) -> Dict[str, Any]:
    """Send WhatsApp message if configured. Returns {ok, error?}."""
    st = status()
    if not st.get("enabled"):
        return {"ok": False, "error": "whatsapp_disabled_or_missing_creds", "details": st}
    try:
        import base64
        import urllib.request
        import urllib.parse
        import json

        sid = _get("TWILIO_ACCOUNT_SID")
        tok = _get("TWILIO_AUTH_TOKEN")
        url = f"https://api.twilio.com/2010-04-01/Accounts/{sid}/Messages.json"
        data = urllib.parse.urlencode(
            {
                "From": _get("TWILIO_WHATSAPP_FROM"),
                "To": _get("TWILIO_WHATSAPP_TO"),
                "Body": (text or "")[:1500],
            }
        ).encode("utf-8")
        auth = base64.b64encode(f"{sid}:{tok}".encode("utf-8")).decode("ascii")
        req = urllib.request.Request(url, data=data, method="POST", headers={"Authorization": f"Basic {auth}"})
        with urllib.request.urlopen(req, timeout=20) as r:
            raw = r.read().decode("utf-8", errors="replace")
        try:
            j = json.loads(raw)
        except Exception:
            j = {"raw": raw}
        return {"ok": True, "result": j}
    except Exception as e:
        return {"ok": False, "error": str(e)}

