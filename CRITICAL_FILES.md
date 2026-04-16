# CRITICAL FILES — Atlas Code Quant

## MAIN ENTRY POINT

### `atlas_code_quant/main.py`
```python
"""Atlas Code-Quant — Punto de entrada del motor de trading.

Uso:
    python main.py

Arranca la API REST en el puerto 8792.
"""
from __future__ import annotations

import logging
import sys
from pathlib import Path

# Asegura que el directorio raíz está en el path
sys.path.insert(0, str(Path(__file__).resolve().parent))

import uvicorn

from api.main import app, _strategies, _portfolio
from config.settings import settings
from execution.portfolio import Portfolio
from strategies.ma_cross import MACrossStrategy

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(settings.logs_dir / "quant.log"),
    ],
)
logger = logging.getLogger("quant.main")


def bootstrap():
    """Inicializa el portfolio y registra las estrategias disponibles."""
    import api.main as api_mod

    # Portfolio con capital inicial
    portfolio = Portfolio(
        initial_capital=10_000.0,
        max_position_pct=settings.max_position_pct,
        max_drawdown_pct=settings.max_drawdown_pct,
    )
    api_mod._portfolio = portfolio

    # Registrar estrategias
    ma_cross = MACrossStrategy(
        name="ma_cross",
        symbols=settings.assets,
        timeframe=settings.primary_timeframe,
        fast_period=10,
        slow_period=50,
    )
    api_mod._strategies["ma_cross"] = ma_cross

    logger.info("Bootstrap completo — %d estrategias registradas", len(api_mod._strategies))
    logger.info("Portfolio: $%.2f capital inicial", portfolio.initial_capital)
    logger.info("Paper trading: %s", settings.paper_trading)


if __name__ == "__main__":
    bootstrap()
    logger.info("Iniciando Atlas Code-Quant API en puerto %d", settings.api_port)
    uvicorn.run(
        app,
        host=settings.api_host,
        port=settings.api_port,
        log_level="info",
    )
```

## JOURNAL LAYER

### `atlas_code_quant/journal/__init__.py`
```python
"""Trading journal persistence and analytics for Atlas Code-Quant."""
```

### `atlas_code_quant/journal/service.py` (extracto crítico)
> Archivo muy extenso (~2.3k líneas). Se incluyen secciones nucleares para auditoría de flujo y bug actual.
```python
def _upsert_open_entry(self, db: Any, account: TradierAccountSession, client: TradierClient, strategy: dict[str, Any], matched_events: list[dict[str, Any]]) -> tuple[str, bool, bool]:
    strategy_id, signature = _stable_strategy_id(strategy)
    # PATCH: skip sandbox-restricted symbols (OS, AA) — phantom prevention
    _SYNC_BLOCKED: frozenset = frozenset({"OS", "AA"})
    if str(strategy.get("underlying") or "") in _SYNC_BLOCKED:
        return strategy_id, False, True  # skip, not created, skipped=True
    ...
    if entry is None:
        ...
        return strategy_id, True, False
    ...
    return strategy_id, False, False

def sync_scope(self, scope: str) -> dict[str, Any]:
    ...
    for strategy in strategies:
        matched = _matching_events(trade_events, strategy)
        strategy_id, created_now, skipped_now = self._upsert_open_entry(db, account, client, strategy, matched)
        ...
```

## SCANNER

### `atlas_code_quant/scanner/opportunity_scanner.py` (extracto crítico)
```python
class OpportunityScannerService:
    """Escáner continuo con explicación explícita y criterios gobernados."""
    ...
    def _evaluate_symbol_timeframes(...):
        ...
        selection_score = round(
            (local_quality_score * 0.44)
            + (float(relative_strength_pct) * 0.14)
            + (float(best["strength"]) * 100.0 * 0.12)
            + (alignment_score * 0.10)
            + (float(best["evidence_score"]) * 100.0 * 0.06)
            + (order_flow_score * 0.08)
            + (order_flow_alignment_score * 0.06)
            + adaptive_bias,
            2,
        )
        ...
```

### `atlas_code_quant/scanner/__init__.py`
```python
"""Atlas Code-Quant — Opportunity scanner services."""

from scanner.opportunity_scanner import OpportunityScannerService

__all__ = ["OpportunityScannerService"]
```

## STRATEGY & RISK

### `atlas_code_quant/selector/strategy_selector.py` (extracto crítico)
```python
class StrategySelectorService:
    ...
    def proposal(
        self,
        *,
        candidate: dict[str, Any],
        account_scope: str = "paper",
        ...
    ) -> dict[str, Any]:
        ...
        candidate_structures = self._candidate_structures(...)
        ...
        size_plan = self._size_plan(...)
        ...
        risk_profile = self._risk_profile(...)
        ...
        return {
            "selected": {...},
            "size_plan": size_plan,
            "risk_profile": risk_profile,
            "order_seed": order_seed,
            ...
        }
```

### `atlas_code_quant/config/settings.py` (extracto gates/config)
```python
scanner_enabled: bool = os.getenv("QUANT_SCANNER_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
scanner_min_selection_score: float = _fenv("QUANT_SCANNER_MIN_SELECTION_SCORE", 65.0)
entry_validation_enabled: bool = os.getenv("QUANT_ENTRY_VALIDATION_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
visual_gate_fail_closed: bool = os.getenv("QUANT_VISUAL_GATE_FAIL_CLOSED", "true").strip().lower() not in {"0", "false", "no"}
position_management_enabled: bool = os.getenv("QUANT_POSITION_MANAGEMENT_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
exit_governance_enabled: bool = os.getenv("QUANT_EXIT_GOVERNANCE_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
adaptive_learning_enabled: bool = os.getenv("QUANT_ADAPTIVE_LEARNING_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
```

## EXECUTION

### `atlas_code_quant/execution/live_loop.py` (extracto crítico)
```python
class LiveLoop:
    ...
    def _run_cycle(self, metrics: CycleMetrics) -> None:
        # 1) CPU check -> OCR throttle
        # 2) OCR update
        # 3) quotes snapshot
        # 4) self-healing check
        # 5) position management
        # 6-8) evaluate + execute
        # 9) publish metrics
        ...
```

### `atlas_code_quant/execution/tradier_execution.py` (extracto crítico)
```python
def route_order_to_tradier(body: OrderRequest) -> dict[str, Any]:
    effective_scope = body.account_scope or settings.tradier_default_scope
    client, session = resolve_account_session(...)
    order_class, position_effect, payload = build_tradier_order_payload(body)
    ...
```

### `atlas_code_quant/execution/__init__.py`
```python
```

## LEARNING

### `atlas_code_quant/learning/__init__.py`
```python
"""Adaptive learning services for Atlas Code-Quant."""
```

### `atlas_code_quant/learning/` (primer inventario)
```text
adaptive_policy.py
atlas_learning_brain.py
duckdb_analytics.py
event_store.py
ic_signal_tracker.py
indicator_lab.py
journal_data_quality.py
learning_orchestrator.py
metrics_engine.py
ml_signal_ranker.py
motif_lab.py
policy_manager.py
retraining_scheduler.py
rl_env.py
rl_trainer.py
strategy_evolver.py
tin_models.py
trade_events.py
trading_implementation_scorecard.py
trading_self_audit_protocol.py
... (+ xgboost_signal/*)
```

## SCHEMAS

### `atlas_code_quant/api/schemas.py` (extracto crítico)
```python
class OrderRequest(BaseModel):
    symbol: str
    side: str
    size: float
    order_type: str = Field("market", pattern="^(market|limit|stop|stop_limit|debit|credit|even)$")
    ...

class ScannerStatusPayload(BaseModel):
    generated_at: str
    running: bool
    cycle_count: int = 0
    ...
```

### `atlas_code_quant/journal/models.py`
```python
class TradingJournal(Base):
    __tablename__ = "trading_journal"
    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    journal_key: Mapped[str] = mapped_column(String(128), unique=True, index=True)
    account_type: Mapped[str] = mapped_column(String(16), index=True)
    strategy_type: Mapped[str] = mapped_column(String(80), index=True)
    symbol: Mapped[str] = mapped_column(String(64), index=True)
    ...
```
