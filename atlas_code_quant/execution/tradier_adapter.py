"""Tradier Adapter — F6 (esqueleto dry-run).

Diseño paper-first: NO realiza llamadas HTTP reales en este esqueleto.
- ``submit(order)`` solo registra la intención y devuelve un ``OrderTicket``
  con ``status="dry_run"`` cuando ``ATLAS_TRADIER_DRY_RUN=true`` (default).
- ``cancel_all()`` y ``reconcile_positions()`` también son no-ops audited.
- La conexión real con Tradier se implementará en F6.b tras revisión.

Variables de entorno reconocidas:
- ``ATLAS_TRADIER_BASE_URL``        (default sandbox)
- ``ATLAS_TRADIER_TOKEN``           (no logueado)
- ``ATLAS_TRADIER_ACCOUNT_ID``
- ``ATLAS_TRADIER_DRY_RUN``         (default true)
- ``ATLAS_MAX_ORDERS_PER_MINUTE``   (default 30)
- ``ATLAS_LIVE_TRADING_ENABLED``    (default false; bloquea live aun fuera de dry-run)
"""
from __future__ import annotations

import os
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Literal


def _env_bool(name: str, default: bool) -> bool:
    v = os.environ.get(name)
    if v is None:
        return default
    return v.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(slots=True)
class TradierConfig:
    base_url: str = "https://sandbox.tradier.com/v1"
    token: str = ""  # nunca logueado
    account_id: str = ""
    dry_run: bool = True
    live_enabled: bool = False
    max_orders_per_minute: int = 30

    @classmethod
    def from_env(cls) -> "TradierConfig":
        return cls(
            base_url=os.environ.get(
                "ATLAS_TRADIER_BASE_URL", "https://sandbox.tradier.com/v1"
            ),
            token=os.environ.get("ATLAS_TRADIER_TOKEN", ""),
            account_id=os.environ.get("ATLAS_TRADIER_ACCOUNT_ID", ""),
            dry_run=_env_bool("ATLAS_TRADIER_DRY_RUN", True),
            live_enabled=_env_bool("ATLAS_LIVE_TRADING_ENABLED", False),
            max_orders_per_minute=int(os.environ.get("ATLAS_MAX_ORDERS_PER_MINUTE", "30")),
        )


OrderType = Literal["market", "limit", "stop", "stop_limit"]
OrderSide = Literal["buy_to_open", "sell_to_open", "buy_to_close", "sell_to_close"]
OrderStatus = Literal["dry_run", "submitted", "rejected", "blocked"]


@dataclass(slots=True)
class OrderRequest:
    symbol: str
    side: OrderSide
    quantity: int
    order_type: OrderType = "market"
    limit_price: float | None = None
    stop_price: float | None = None
    duration: Literal["day", "gtc"] = "day"
    tag: str = ""


@dataclass(slots=True)
class OrderTicket:
    request: OrderRequest
    status: OrderStatus
    broker_order_id: str = ""
    rationale: str = ""
    submitted_at: float = 0.0


@dataclass(slots=True)
class TradierAdapter:
    config: TradierConfig = field(default_factory=TradierConfig.from_env)
    _ts_window: Deque[float] = field(default_factory=deque)

    # ── rate-limit interno ────────────────────────────────────────────────
    def _can_submit(self, now: float) -> bool:
        cutoff = now - 60.0
        while self._ts_window and self._ts_window[0] < cutoff:
            self._ts_window.popleft()
        return len(self._ts_window) < self.config.max_orders_per_minute

    # ── API pública ────────────────────────────────────────────────────────
    def submit(self, order: OrderRequest) -> OrderTicket:
        now = time.time()
        if not self._can_submit(now):
            return OrderTicket(
                request=order,
                status="blocked",
                rationale="rate_limit_exceeded",
                submitted_at=now,
            )

        # Política paper-first: live exige flag explícita
        if not self.config.dry_run and not self.config.live_enabled:
            return OrderTicket(
                request=order,
                status="blocked",
                rationale="live_disabled_by_flag",
                submitted_at=now,
            )

        if self.config.dry_run:
            self._ts_window.append(now)
            return OrderTicket(
                request=order,
                status="dry_run",
                broker_order_id=f"DR-{uuid.uuid4().hex[:8]}",
                rationale="ATLAS_TRADIER_DRY_RUN=true",
                submitted_at=now,
            )

        # Camino live: en este esqueleto NO realizamos HTTP. Devolvemos
        # ``rejected`` audit-friendly para forzar implementación explícita
        # en F6.b tras revisión.
        return OrderTicket(
            request=order,
            status="rejected",
            rationale="live_http_not_implemented_in_skeleton",
            submitted_at=now,
        )

    def cancel_all(self) -> dict[str, Any]:
        return {
            "ok": True,
            "dry_run": self.config.dry_run,
            "cancelled": 0,
            "note": "skeleton no-op; F6.b connectará HTTP",
        }

    def reconcile_positions(self) -> dict[str, Any]:
        return {
            "ok": True,
            "dry_run": self.config.dry_run,
            "positions": [],
            "note": "skeleton no-op; F6.b leerá /accounts/{id}/positions",
        }
