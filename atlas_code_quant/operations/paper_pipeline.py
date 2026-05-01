"""Atlas Code Quant — Paper execution pipeline (F16).

F16 conecta de extremo a extremo, **en papel**, los bloques F10–F15:

    Radar opportunities (gated F10)
        → StrategyFactory (F12)
            → LEAN fitness (F13/F14)
                → TradierAdapter.submit (F15, dry-run)

Reglas duras (F16):

    * **No live trading.** ``TradierAdapter`` se invoca con
      ``dry_run=True``. Cualquier intento de salir de dry-run aquí
      requiere fase posterior con autorización explícita.
    * F16 **NO** importa otros módulos de
      ``atlas_code_quant/operations/`` (auton_executor,
      live_authorization, live_loop, broker_router, etc.). Es un
      pipeline aislado que sólo orquesta F12/F13/F14/F15 +
      utilidades del propio core.
    * Defensivo: nunca lanza. Cualquier fallo se registra como un
      ``PaperPipelineRecord`` con ``ok=False`` y código de error.
    * Inyección de dependencias para tests (factory, evaluator,
      adapter pueden mockearse).

Ver:
    * docs/ATLAS_CODE_QUANT_F16_PAPER_EXECUTION_PIPELINE.md
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Optional

from atlas_code_quant.execution.tradier_adapter import (
    OrderIntent,
    OrderResult,
    TradierAdapter,
    TradierAdapterConfig,
)
from atlas_code_quant.intake.opportunity import RadarOpportunityInternal
from atlas_code_quant.strategies.evaluation import (
    StrategyWithFitness,
    evaluate_strategies_for_opportunity,
)
from atlas_code_quant.strategies.options.intent import StrategyIntent

logger = logging.getLogger("atlas.code_quant.operations.paper_pipeline")


__all__ = [
    "PaperPipelineConfig",
    "PaperPipelineRecord",
    "PaperPipelineReport",
    "run_paper_pipeline_for_opportunity",
    "run_paper_pipeline_for_opportunities",
]


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PaperPipelineConfig:
    """Configuración aislada del pipeline F16.

    NO toca env vars de Tradier ni LEAN: el adapter Tradier y el
    backtest_fn se inyectan. El pipeline garantiza que el adapter
    se use SIEMPRE con ``dry_run=True``.
    """

    min_score: float = 70.0
    min_sharpe: float | None = 0.0  # filtro opcional sobre fitness
    only_evaluated: bool = True  # descarta intents con fitness fallido
    max_strategies_per_opportunity: int | None = 1  # top-N por opp
    default_quantity: int = 1


# ---------------------------------------------------------------------------
# Modelos de salida
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PaperPipelineRecord:
    """Registro por cada intento de envío en el pipeline."""

    opportunity_symbol: str
    strategy_type: str
    trace_id: str
    fitness_success: bool
    sharpe: float
    submit_ok: bool
    dry_run: bool
    order_id: str | None
    error_code: str | None = None
    error_message: str | None = None
    raw_intent: dict[str, Any] = field(default_factory=dict, repr=False, compare=False)
    raw_order_result: dict[str, Any] = field(default_factory=dict, repr=False, compare=False)

    def to_dict(self) -> dict[str, Any]:
        return {
            "opportunity_symbol": self.opportunity_symbol,
            "strategy_type": self.strategy_type,
            "trace_id": self.trace_id,
            "fitness_success": self.fitness_success,
            "sharpe": self.sharpe,
            "submit_ok": self.submit_ok,
            "dry_run": self.dry_run,
            "order_id": self.order_id,
            "error_code": self.error_code,
            "error_message": self.error_message,
        }


@dataclass(frozen=True)
class PaperPipelineReport:
    """Resumen de un run del pipeline."""

    total_opportunities: int
    total_intents: int
    total_submitted: int
    total_ok: int
    records: tuple[PaperPipelineRecord, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "total_opportunities": self.total_opportunities,
            "total_intents": self.total_intents,
            "total_submitted": self.total_submitted,
            "total_ok": self.total_ok,
            "records": [r.to_dict() for r in self.records],
        }


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


BacktestFn = Callable[[StrategyIntent], Any]
EvaluatorFn = Callable[..., list[StrategyWithFitness]]


def _strategy_intent_to_order_intent(
    intent: StrategyIntent, *, default_quantity: int
) -> OrderIntent:
    """Mapea un ``StrategyIntent`` (F11) a un ``OrderIntent`` (F15).

    F16 transforma el intent en una orden multileg genérica para
    Tradier. Strikes/expiries se pasan en ``legs`` como dicts;
    Tradier no los entiende como tales en este formato (el adapter
    F15 sólo los pasa por), pero F16 sí los registra para
    auditoría. La traducción a chain real (con Tradier symbols como
    ``SPY230721C00450000``) llega en una fase posterior.
    """
    legs: list[dict[str, Any]] = []
    for leg in intent.legs:
        legs.append(
            {
                "side": leg.side,
                "right": leg.right,
                "strike_offset_steps": int(leg.strike_offset_steps),
                "expiry_rel_dte": int(leg.expiry_rel_dte),
                "quantity": int(leg.quantity),
            }
        )
    asset_class = "multileg" if len(legs) > 1 else "option"
    # side a nivel orden: para multileg Tradier usa "buy" como
    # placeholder "open"; F15 acepta cualquiera de los valores
    # válidos. El detalle por pata vive en ``legs``.
    return OrderIntent(
        symbol=intent.opportunity.symbol,
        side="buy_to_open",
        quantity=int(default_quantity),
        order_type="market",
        duration="day",
        trace_id=intent.opportunity.trace_id,
        asset_class=asset_class,
        legs=tuple(legs),
        metadata={
            "strategy_type": intent.strategy_type,
            "structure": intent.metadata.get("structure"),
            "dte": intent.metadata.get("dte"),
        },
    )


def _ensure_dry_run_adapter(
    adapter: TradierAdapter | None,
) -> TradierAdapter:
    """Devuelve un adapter F16 garantizado en dry-run.

    Si no se inyecta uno, construye uno con `TradierAdapterConfig`
    default (``dry_run=True``). Si se inyecta uno con dry-run False,
    F16 fuerza dry-run explícitamente para cumplir el contrato.
    """
    if adapter is None:
        return TradierAdapter(TradierAdapterConfig(dry_run=True))
    if not adapter.config.dry_run:
        # F16 NO permite live. Re-instanciamos en dry-run.
        forced_cfg = TradierAdapterConfig(
            base_url=adapter.config.base_url,
            token=adapter.config.token,
            account_id=adapter.config.account_id,
            dry_run=True,
            max_orders_per_minute=adapter.config.max_orders_per_minute,
            max_retries_5xx=adapter.config.max_retries_5xx,
            retry_base_sleep_sec=adapter.config.retry_base_sleep_sec,
            retry_max_sleep_sec=adapter.config.retry_max_sleep_sec,
            request_timeout_sec=adapter.config.request_timeout_sec,
        )
        logger.warning(
            "paper_pipeline: forced dry_run=True (input adapter was live)"
        )
        return TradierAdapter(forced_cfg)
    return adapter


# ---------------------------------------------------------------------------
# API pública
# ---------------------------------------------------------------------------


def run_paper_pipeline_for_opportunity(
    opp: RadarOpportunityInternal | None,
    *,
    config: PaperPipelineConfig | None = None,
    adapter: TradierAdapter | None = None,
    backtest_fn: Optional[BacktestFn] = None,
    evaluator_fn: Optional[EvaluatorFn] = None,
) -> PaperPipelineReport:
    """Pipeline papel-end-to-end para UNA oportunidad Radar.

    Defensivo: ningún paso lanza hacia fuera; cada error se registra.
    """
    cfg = config or PaperPipelineConfig()
    safe_adapter = _ensure_dry_run_adapter(adapter)
    eval_fn = evaluator_fn or evaluate_strategies_for_opportunity

    if opp is None or not isinstance(opp, RadarOpportunityInternal):
        return PaperPipelineReport(
            total_opportunities=0,
            total_intents=0,
            total_submitted=0,
            total_ok=0,
            records=(),
        )

    try:
        evaluated = eval_fn(
            opp,
            min_score=cfg.min_score,
            backtest_fn=backtest_fn,
            rank=True,
        )
    except Exception as exc:  # noqa: BLE001
        logger.warning(
            "paper_pipeline: evaluator raised for %s: %s", opp.symbol, exc
        )
        evaluated = []

    # Filtros F16
    candidates: list[StrategyWithFitness] = []
    for swf in evaluated:
        if cfg.only_evaluated and not swf.fitness.success:
            continue
        if cfg.min_sharpe is not None and swf.fitness.sharpe < cfg.min_sharpe:
            continue
        candidates.append(swf)
    if cfg.max_strategies_per_opportunity is not None:
        candidates = candidates[: cfg.max_strategies_per_opportunity]

    records: list[PaperPipelineRecord] = []
    submitted = 0
    ok_count = 0
    for swf in candidates:
        intent = swf.intent
        order = _strategy_intent_to_order_intent(
            intent, default_quantity=cfg.default_quantity
        )
        try:
            res: OrderResult = safe_adapter.submit(order)
        except Exception as exc:  # noqa: BLE001
            logger.warning(
                "paper_pipeline: adapter raised for %s: %s",
                opp.symbol,
                exc,
            )
            res = OrderResult(
                ok=False,
                dry_run=True,
                error_code="PAPER_PIPELINE_ADAPTER_RAISED",
                error_message=str(exc),
                attempts=0,
            )
        submitted += 1
        if res.ok:
            ok_count += 1
        records.append(
            PaperPipelineRecord(
                opportunity_symbol=opp.symbol,
                strategy_type=intent.strategy_type,
                trace_id=intent.opportunity.trace_id,
                fitness_success=swf.fitness.success,
                sharpe=float(swf.fitness.sharpe),
                submit_ok=bool(res.ok),
                dry_run=bool(res.dry_run),
                order_id=res.order_id,
                error_code=res.error_code,
                error_message=res.error_message,
                raw_intent=intent.to_dict(),
                raw_order_result=res.to_dict(),
            )
        )

    return PaperPipelineReport(
        total_opportunities=1,
        total_intents=len(evaluated),
        total_submitted=submitted,
        total_ok=ok_count,
        records=tuple(records),
    )


def run_paper_pipeline_for_opportunities(
    opps: Iterable[RadarOpportunityInternal] | None,
    *,
    config: PaperPipelineConfig | None = None,
    adapter: TradierAdapter | None = None,
    backtest_fn: Optional[BacktestFn] = None,
    evaluator_fn: Optional[EvaluatorFn] = None,
) -> PaperPipelineReport:
    """Pipeline papel-end-to-end para un iterable de oportunidades."""
    if opps is None:
        return PaperPipelineReport(
            total_opportunities=0,
            total_intents=0,
            total_submitted=0,
            total_ok=0,
            records=(),
        )

    safe_adapter = _ensure_dry_run_adapter(adapter)
    cfg = config or PaperPipelineConfig()

    total_opps = 0
    total_intents = 0
    total_submitted = 0
    total_ok = 0
    records: list[PaperPipelineRecord] = []
    for opp in opps:
        sub_report = run_paper_pipeline_for_opportunity(
            opp,
            config=cfg,
            adapter=safe_adapter,
            backtest_fn=backtest_fn,
            evaluator_fn=evaluator_fn,
        )
        total_opps += sub_report.total_opportunities
        total_intents += sub_report.total_intents
        total_submitted += sub_report.total_submitted
        total_ok += sub_report.total_ok
        records.extend(sub_report.records)

    return PaperPipelineReport(
        total_opportunities=total_opps,
        total_intents=total_intents,
        total_submitted=total_submitted,
        total_ok=total_ok,
        records=tuple(records),
    )
