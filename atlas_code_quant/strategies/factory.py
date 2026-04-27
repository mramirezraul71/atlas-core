"""Strategy Factory — F5.

Mantiene compatibilidad con el registro F1 (``ma_cross``, ``rl``) y añade las
estrategias de opciones bajo claves ``options.<name>``.
"""
from __future__ import annotations

from typing import Any

try:  # compat con import absoluto/relativo existente en repo
    from atlas_code_quant.strategies.base import BaseStrategy
except Exception:  # pragma: no cover
    from strategies.base import BaseStrategy  # type: ignore

from atlas_code_quant.strategies.contracts import (
    StrategyConfig,
    StrategyOpportunityRef,
    StrategyPlan,
)
from atlas_code_quant.strategies.options import (
    IronButterflyStrategy,
    IronCondorStrategy,
    StraddleStrangleStrategy,
    VerticalSpreadStrategy,
)


def _legacy_registry() -> dict[str, type[BaseStrategy]]:
    registry: dict[str, type[BaseStrategy]] = {}
    try:
        from atlas_code_quant.strategies.ma_cross import MACrossStrategy
        registry["ma_cross"] = MACrossStrategy
    except Exception:
        try:
            from strategies.ma_cross import MACrossStrategy  # type: ignore
            registry["ma_cross"] = MACrossStrategy
        except Exception:
            pass
    try:
        from atlas_code_quant.strategies.rl_strategy import RLStrategy
        registry["rl"] = RLStrategy
    except Exception:
        try:
            from strategies.rl_strategy import RLStrategy  # type: ignore
            registry["rl"] = RLStrategy
        except Exception:
            pass
    return registry


# Registros separados:
#  - _registry: BaseStrategy clásicas (ma_cross, rl, ...)
#  - _options_registry: estrategias de opciones con build_plan(StrategyPlan)
class StrategyFactory:
    """Registro conservador para instanciar estrategias por nombre."""

    _registry: dict[str, type[BaseStrategy]] = _legacy_registry()
    _options_registry: dict[str, type] = {
        "options.vertical_spread": VerticalSpreadStrategy,
        "options.iron_condor": IronCondorStrategy,
        "options.iron_butterfly": IronButterflyStrategy,
        "options.straddle_strangle": StraddleStrangleStrategy,
    }

    # ── compat F1 ──────────────────────────────────────────────────────────
    @classmethod
    def register(cls, key: str, strategy_cls: type[BaseStrategy]) -> None:
        cls._registry[key] = strategy_cls

    @classmethod
    def create(cls, key: str, **kwargs: Any) -> BaseStrategy:
        if key not in cls._registry:
            raise KeyError(f"Unknown strategy key: {key}")
        return cls._registry[key](**kwargs)

    # ── F5 opciones ────────────────────────────────────────────────────────
    @classmethod
    def register_option(cls, key: str, strategy_cls: type) -> None:
        cls._options_registry[key] = strategy_cls

    @classmethod
    def create_option(cls, key: str, **kwargs: Any) -> Any:
        if key not in cls._options_registry:
            raise KeyError(f"Unknown option strategy key: {key}")
        return cls._options_registry[key](**kwargs)

    @classmethod
    def list_options(cls) -> list[str]:
        return sorted(cls._options_registry)

    @classmethod
    def list_all(cls) -> dict[str, list[str]]:
        return {
            "legacy": sorted(cls._registry),
            "options": sorted(cls._options_registry),
        }

    # ── F9.2 candidatas multi-estrategia ──────────────────────────────────
    @classmethod
    def build_candidates(
        cls,
        opportunity: StrategyOpportunityRef | dict,
        config: StrategyConfig | None = None,
    ) -> list[StrategyPlan]:
        """Construye 4 ``StrategyPlan`` (uno por estrategia de opciones) para
        una oportunidad dada y devuelve solo los que terminaron en
        ``status == 'planned'`` (descarta rejected).

        Garantiza ≥1 candidato por símbolo si la dirección es válida; con
        bias direccional emite vertical_spread; con bias neutral emite
        iron_condor + iron_butterfly + straddle_strangle.
        """
        opp = (
            opportunity
            if isinstance(opportunity, StrategyOpportunityRef)
            else StrategyOpportunityRef.from_dict(opportunity)
        )
        cfg = config or StrategyConfig()
        plans: list[StrategyPlan] = []
        for key in (
            "options.vertical_spread",
            "options.iron_condor",
            "options.iron_butterfly",
            "options.straddle_strangle",
        ):
            try:
                strat = cls.create_option(key)
                plan = strat.build_plan(opp, cfg)
                if plan.is_actionable():
                    plans.append(plan)
            except Exception as exc:  # pragma: no cover - safety net
                # Mantener el flujo si una estrategia rompe; no contamina las demás.
                plans.append(
                    StrategyPlan(
                        strategy=key.split(".", 1)[-1],
                        symbol=opp.symbol,
                        direction=opp.direction,
                        status="rejected",
                        rationale=f"build_failed: {exc.__class__.__name__}",
                        trace_id=opp.trace_id,
                    )
                )
        return [p for p in plans if p.status == "planned"]
