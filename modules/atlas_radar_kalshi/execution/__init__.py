"""Capa de ejecución: paper/live, routing multi-venue, reconciliación."""
from __future__ import annotations

from .reconciliation import reconcile_kalshi_from_router
from ..execution_router import ExecutionModeRouter
from ..executor import KalshiExecutor, OrderRequest, OrderResult
from ..executor_v2 import ExecConfig, FillReport, KalshiExecutorV2, OrderRequestV2
from ..polymarket_executor import PolymarketExecutor, PolymarketExecutorConfig

__all__ = [
    "reconcile_kalshi_from_router",
    "ExecutionModeRouter",
    "KalshiExecutor",
    "KalshiExecutorV2",
    "OrderRequest",
    "OrderRequestV2",
    "OrderResult",
    "FillReport",
    "ExecConfig",
    "PolymarketExecutor",
    "PolymarketExecutorConfig",
]
