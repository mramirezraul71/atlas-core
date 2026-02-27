"""Webhook bridge: envía eventos ATLAS en tiempo real a MakePlay/Make.com o plataforma externa."""
from __future__ import annotations

import json
import os
import threading
import urllib.request
import urllib.error
from typing import Any, Dict, Optional

_TIMEOUT_SEC = 5
_last_error: Optional[str] = None


def _webhook_url() -> Optional[str]:
    url = (os.getenv("MAKEPLAY_WEBHOOK_URL") or os.getenv("EXTERNAL_WEBHOOK_URL") or "").strip()
    return url if url.startswith("http") else None


def _enabled() -> bool:
    if not _webhook_url():
        return False
    return os.getenv("MAKEPLAY_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def _post_payload(payload: Dict[str, Any]) -> bool:
    """POST JSON a webhook. No bloquea (thread)."""
    url = _webhook_url()
    if not url or not _enabled():
        return False

    def _send():
        global _last_error
        try:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            req = urllib.request.Request(
                url,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=_TIMEOUT_SEC) as r:
                pass  # 2xx OK
            _last_error = None
        except urllib.error.HTTPError as e:
            _last_error = f"HTTP {e.code}: {e.reason}"
        except urllib.error.URLError as e:
            _last_error = str(e.reason) if e.reason else str(e)
        except Exception as e:
            _last_error = str(e)

    t = threading.Thread(target=_send, daemon=True)
    t.start()
    return True


def push_event(event: Dict[str, Any]) -> bool:
    """Envía un evento del live stream al webhook."""
    if not _enabled():
        return False
    payload = {
        "source": "atlas",
        "stream": "ans_live",
        "event": event,
    }
    return _post_payload(payload)


def push_scan(snapshot: Dict[str, Any]) -> bool:
    """Envía snapshot de scanner permanente (estado, health, incidentes, mejoras)."""
    if not _enabled():
        return False
    payload = {
        "source": "atlas",
        "stream": "scanner",
        "snapshot": snapshot,
    }
    return _post_payload(payload)


def get_last_error() -> Optional[str]:
    return _last_error
