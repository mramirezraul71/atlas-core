from __future__ import annotations

"""Router paper/live multi-venue.

Idempotencia: `make_client_order_id` es determinístico (hash) por
(ticker, lado, precio, contratos, bucketing temporal) para reintentos seguros.
"""
from typing import Optional

from .config import RadarSettings, get_settings
from .executor_v2 import ExecConfig, FillReport, KalshiExecutorV2, OrderRequestV2
from .polymarket_executor import PolymarketExecutor, PolymarketExecutorConfig


class ExecutionModeRouter:
    """Enruta órdenes al executor correcto por modo y venue.

    Convención:
    - Tickers con prefijo `POLY:` -> Polymarket.
    - Resto -> Kalshi.
    """

    def __init__(
        self,
        settings: Optional[RadarSettings] = None,
        exec_cfg: Optional[ExecConfig] = None,
    ) -> None:
        self.settings = settings or get_settings()
        live = self.settings.execution_mode == "live"
        self.kalshi = KalshiExecutorV2(
            self.settings,
            exec_cfg
            or ExecConfig(
                enable_live=live,
            ),
        )
        self.cfg = self.kalshi.cfg
        self.polymarket = PolymarketExecutor(
            self.settings,
            PolymarketExecutorConfig(enable_live=live and self.settings.polymarket_enabled),
        )

    def is_polymarket(self, ticker: str) -> bool:
        return ticker.upper().startswith("POLY:")

    async def submit(self, req: OrderRequestV2) -> FillReport:
        if self.is_polymarket(req.market_ticker):
            return await self.polymarket.submit(req)
        return await self.kalshi.submit(req)

    async def balance_cents(self) -> int:
        return await self.kalshi.balance_cents()

    def make_client_order_id(
        self, ticker: str, side: str, price: int, contracts: int, ts_bucket_ms: int = 1000
    ) -> str:
        if self.is_polymarket(ticker):
            return f"poly-{KalshiExecutorV2.make_client_order_id(ticker, side, price, contracts, ts_bucket_ms)}"
        return KalshiExecutorV2.make_client_order_id(ticker, side, price, contracts, ts_bucket_ms)

    @property
    def metrics(self):  # noqa: D401
        """Métricas de ejecución primarias (Kalshi)."""
        return self.kalshi.metrics

