from __future__ import annotations

import os
import time
from typing import Any, Dict, Optional


def _enabled() -> bool:
    return (os.getenv("TRADING_ENABLED", "false") or "").strip().lower() in ("1", "true", "yes", "y", "on")


def grasp_market_data(ticker: str, timeframe: str = "1m") -> Dict[str, Any]:
    """
    Captura datos de mercado (placeholder seguro).
    Implementación real requiere proveedor y credenciales en Bóveda.
    """
    if not _enabled():
        return {"ok": False, "error": "trading_disabled", "ticker": ticker, "timeframe": timeframe}
    # Aún sin proveedor integrado en este repo: devolver not_configured
    return {"ok": False, "error": "market_data_provider_not_configured", "ticker": ticker, "timeframe": timeframe}


def execute_trade(ticker: str, side: str, qty: float, type: str = "market") -> Dict[str, Any]:
    """
    Ejecuta una orden (SIEMPRE con seguridad).
    Por defecto: deshabilitado. Si TRADING_ENABLED=true, sigue requiriendo aprobación (Telegram) por riesgo crítico.
    """
    if not _enabled():
        return {"ok": False, "error": "trading_disabled"}
    payload = {"ticker": ticker, "side": side, "qty": qty, "type": type, "ts": time.time(), "risk": "critical"}
    try:
        from modules.humanoid.approvals.service import create
        cr = create("trade_execute", payload)
        return {"ok": False, "error": "approval_required", "approval_id": cr.get("approval_id"), "payload": payload}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200], "payload": payload}


def monitor_pnl() -> Dict[str, Any]:
    """P&L en tiempo real (placeholder seguro)."""
    if not _enabled():
        return {"ok": False, "error": "trading_disabled"}
    return {"ok": False, "error": "pnl_not_implemented"}


def hedge_position(strategy: str) -> Dict[str, Any]:
    """Cobertura (placeholder seguro)."""
    if not _enabled():
        return {"ok": False, "error": "trading_disabled"}
    payload = {"strategy": strategy, "risk": "high"}
    try:
        from modules.humanoid.approvals.service import create
        cr = create("trade_hedge", payload)
        return {"ok": False, "error": "approval_required", "approval_id": cr.get("approval_id"), "payload": payload}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200], "payload": payload}

