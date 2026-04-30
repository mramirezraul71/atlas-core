"""
atlas_radar_kalshi
==================

Módulo autónomo de Atlas dedicado al escaneo y trading sobre mercados
de eventos de **Kalshi v2**.

Reside en `atlas-core/modules/atlas_radar_kalshi/` pero opera de forma
independiente respecto a `atlas_push`. Sólo comparte utilidades base
del paquete `core/` (logging, scheduler) y la configuración común de
Ollama del repositorio.

Arquitectura en 4 capas (ver docs/architecture.md):

1. ``scanner``  → WebSockets + REST polling de Kalshi (orderbook,
                  ticker, market_lifecycle).
2. ``brain``    → Análisis con Ollama (sentimiento) + Cadenas de
                  Markov + Monte Carlo para estimar p(éxito).
3. ``risk``     → Criterio de Kelly (full / fractional) y gestión
                  de capital.
4. ``executor`` → Envío y monitoreo de órdenes (limit/market).

Regla de oro (idéntica a atlas_push):

- ``atlas_radar_kalshi`` **no** importa símbolos del brain core mayor
  de ATLAS (``brain_core``, ``mission_manager``, ``safety_kernel``,
  ``state_bus``, ``arbitration``, ``policy_store``,
  ``modules.command_router``).
- Los contratos hacia el resto de Atlas viajan vía
  ``typing.Protocol`` y/o eventos JSON sobre el bus HTTP del
  dashboard (puerto 8791).
"""
from __future__ import annotations

from .config import RadarSettings, get_settings, reload_settings
from .brain import RadarBrain, BrainDecision
from .risk import KellyRiskManager, PositionSize
from .scanner import KalshiScanner, OrderBookSnapshot
from .executor import KalshiExecutor, OrderRequest, OrderResult
from .execution_router import ExecutionModeRouter
from .polymarket_executor import PolymarketExecutor
from .polymarket_scanner import PolymarketScanner
from .learning_engine import LearningEngine
from .arbitrage_engine import ArbitrageEngine
from .alerts import AlertEngine
from .normalization import canonical_market_key
from .preflight import run_preflight
from .canonical import (
    CanonicalMarket,
    CanonicalQuote,
    ExecutionIntent,
    FillEvent,
    market_from_payload,
)
from .ingestion import ConnectorRegistry, venue_of_event
from .control import CalibrationKPIs, compute_calibration_kpis

__all__ = [
    "RadarSettings",
    "get_settings",
    "reload_settings",
    "RadarBrain",
    "BrainDecision",
    "KellyRiskManager",
    "PositionSize",
    "KalshiScanner",
    "OrderBookSnapshot",
    "KalshiExecutor",
    "OrderRequest",
    "OrderResult",
    "ExecutionModeRouter",
    "PolymarketScanner",
    "PolymarketExecutor",
    "LearningEngine",
    "ArbitrageEngine",
    "AlertEngine",
    "canonical_market_key",
    "run_preflight",
    "CanonicalMarket",
    "CanonicalQuote",
    "ExecutionIntent",
    "FillEvent",
    "market_from_payload",
    "ConnectorRegistry",
    "venue_of_event",
    "compute_calibration_kpis",
    "CalibrationKPIs",
]

__version__ = "0.1.0"
