"""Reconciliación periódica con brokers (Kalshi REST portfolio)."""
from __future__ import annotations

from typing import Any


async def reconcile_kalshi_from_router(router: Any) -> dict[str, Any]:
    """Delega en el executor Kalshi del router (paper devuelve stub)."""
    kalshi = getattr(router, "kalshi", None)
    if kalshi is None or not hasattr(kalshi, "reconcile"):
        return {"ok": False, "error": "no_kalshi_executor"}
    return await kalshi.reconcile()
