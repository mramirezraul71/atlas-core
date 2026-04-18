"""
Adaptador Code Quant → ``MarketContext`` para el planner de opciones.

``signal_provider`` es inyectable (métodos get_spot / get_trend / get_iv_rank / get_regime);
no se conectan aquí fuentes externas reales.
"""
from __future__ import annotations

from typing import Any, Mapping, Protocol

from .options_planner import MarketContext, Trend


class _SupportsPlanAndOpen(Protocol):
    def plan_and_open(self, context: MarketContext) -> str: ...

_VALID_TRENDS: frozenset[str] = frozenset({"bull", "bear", "sideways", "slightly_bull"})

_TREND_ALIASES: dict[str, Trend] = {
    "bull": "bull",
    "uptrend": "bull",
    "up": "bull",
    "rally": "bull",
    "bullish": "bull",
    "bear": "bear",
    "downtrend": "bear",
    "down": "bear",
    "bearish": "bear",
    "sideways": "sideways",
    "range": "sideways",
    "neutral": "sideways",
    "consolidation": "sideways",
    "consolidate": "sideways",
    "chop": "sideways",
    "slightly_bull": "slightly_bull",
    "slight_bull": "slightly_bull",
    "slightly_bullish": "slightly_bull",
}


def _normalize_trend(raw: str | None, *, fallback: Trend) -> Trend:
    if raw is None or (isinstance(raw, str) and not raw.strip()):
        return fallback
    key = str(raw).strip().lower().replace(" ", "_").replace("-", "_")
    if key in _TREND_ALIASES:
        return _TREND_ALIASES[key]
    if key in _VALID_TRENDS:
        return key  # type: ignore[return-value]
    return fallback


def _clamp_iv_rank(value: float | int | None) -> float | None:
    if value is None:
        return None
    x = float(value)
    if x < 0.0:
        return 0.0
    if x > 100.0:
        return 100.0
    return x


def _normalize_regime(raw: str | None, regime_map: Mapping[str, str] | None) -> str | None:
    if raw is None:
        return None
    s = str(raw).strip()
    if not s:
        return None
    if not regime_map:
        return s
    key = s.lower()
    return regime_map.get(key, regime_map.get(s, s))


class MarketContextBuilder:
    """
    Construye ``MarketContext`` a partir de un proveedor de señales inyectable.

    ``default_symbol_config`` puede incluir:
    - ``trend_fallback`` (default ``\"sideways\"``): tendencia si no hay mapeo.
    - ``regime_map``: dict alias → valor canónico (claves en minúsculas recomendado).
    """

    def __init__(
        self,
        signal_provider: Any,
        *,
        default_symbol_config: dict[str, Any] | None = None,
    ) -> None:
        self._provider = signal_provider
        cfg = dict(default_symbol_config or {})
        fb = cfg.get("trend_fallback", "sideways")
        self._trend_fallback: Trend = _normalize_trend(
            str(fb) if fb is not None else None,
            fallback="sideways",
        )
        rm = cfg.get("regime_map")
        self._regime_map: dict[str, str] = dict(rm) if isinstance(rm, Mapping) else {}

    def build_for_symbol(self, symbol: str) -> MarketContext:
        sym = str(symbol).strip()
        if not sym:
            raise ValueError("symbol vacío")

        spot = float(self._provider.get_spot(sym))
        if spot <= 0:
            raise ValueError(f"spot debe ser > 0 para {sym!r}; recibido {spot!r}")

        trend_raw = self._provider.get_trend(sym)
        trend = _normalize_trend(
            str(trend_raw) if trend_raw is not None else None,
            fallback=self._trend_fallback,
        )

        iv_rank = _clamp_iv_rank(self._provider.get_iv_rank(sym))

        regime_raw = self._provider.get_regime(sym)
        regime = _normalize_regime(
            str(regime_raw) if regime_raw is not None else None,
            self._regime_map,
        )

        return MarketContext(
            symbol=sym,
            spot=spot,
            trend=trend,
            iv_rank=iv_rank,
            regime=regime,
        )

    def build_many(self, symbols: list[str]) -> list[MarketContext]:
        return [self.build_for_symbol(s) for s in symbols]


class AtlasOptionsAutoPlanner:
    """Encadena ``MarketContextBuilder`` → ``AtlasOptionsPlannerService.plan_and_open``."""

    def __init__(
        self,
        context_builder: MarketContextBuilder,
        planner_service: _SupportsPlanAndOpen,
    ) -> None:
        self._builder = context_builder
        self._planner = planner_service

    def auto_plan_and_open(self, symbol: str) -> str:
        ctx = self._builder.build_for_symbol(symbol)
        return self._planner.plan_and_open(ctx)
