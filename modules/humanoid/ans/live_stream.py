"""Live stream: señales en vivo de lo que ejecuta el ANS. Estilo Cursor/terminal."""
from __future__ import annotations

import asyncio
import json
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import Any, AsyncIterator, Dict, List, Optional

_MAX_EVENTS = 500
_events: deque = deque(maxlen=_MAX_EVENTS)
_subscribers: List[asyncio.Queue] = []


@dataclass
class LiveEvent:
    ts: str
    phase: str  # check_start, check_end, heal_attempt, heal_end, decision, skip, incident, summary
    check_id: str = ""
    heal_id: str = ""
    ok: Optional[bool] = None
    message: str = ""
    details: Optional[Dict] = None

    def to_dict(self) -> Dict:
        d = asdict(self)
        d["_type"] = "ans_live"
        return d


def emit(phase: str, check_id: str = "", heal_id: str = "", ok: Optional[bool] = None, message: str = "", details: Optional[Dict] = None) -> None:
    """Emite un evento al stream. Llamado desde el engine."""
    ev = LiveEvent(
        ts=datetime.now(timezone.utc).isoformat(),
        phase=phase,
        check_id=check_id or "",
        heal_id=heal_id or "",
        ok=ok,
        message=message or "",
        details=details or {},
    )
    d = ev.to_dict()
    _events.append(d)
    for q in _subscribers[:]:
        try:
            q.put_nowait(d)
        except asyncio.QueueFull:
            _subscribers.remove(q)
    try:
        from modules.humanoid.comms.webhook_bridge import push_event
        push_event(d)
    except Exception:
        pass


def get_recent(limit: int = 100) -> List[Dict]:
    """Últimos eventos (para polling)."""
    return list(_events)[-limit:]


def subscribe() -> asyncio.Queue:
    """Nueva suscripción. El caller debe consumir la queue."""
    q: asyncio.Queue = asyncio.Queue(maxsize=200)
    _subscribers.append(q)
    return q


def unsubscribe(q: asyncio.Queue) -> None:
    if q in _subscribers:
        _subscribers.remove(q)


async def stream_events() -> AsyncIterator[Dict]:
    """Generador async para SSE. Heartbeat cada 12s para evitar cierre por idle (proxies ~60s)."""
    q = subscribe()
    try:
        while True:
            try:
                ev = await asyncio.wait_for(q.get(), timeout=12.0)
                yield ev
            except asyncio.TimeoutError:
                yield {"_type": "ans_live", "phase": "heartbeat", "ts": datetime.now(timezone.utc).isoformat()}
    finally:
        unsubscribe(q)
