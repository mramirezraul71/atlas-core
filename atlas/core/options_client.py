"""
Servicio de alto nivel para el brain/planner de Atlas sobre atlas_options_brain.

Paper + validación de parámetros + límites de riesgo en apertura.
Ejecución live: solo placeholders (live_mode / live_enabled), siempre seguros en ``off``.
"""
from __future__ import annotations

from datetime import date
from typing import Any, Literal, Optional

from atlas_options_brain.dsl.strategy import (
    BearCallSpread,
    BearPutSpread,
    BullCallSpread,
    BullPutSpread,
    CoveredCall,
    IronCondor,
    OptionsStrategy,
)
from atlas_options_brain.integration.atlas_adapter import AtlasOptionsClient, StrategyType
from atlas_options_brain.providers import (
    OptionsDataProvider,
    PolygonProvider,
    TradierProvider,
    YFinanceProvider,
)
from atlas_options_brain.simulator.paper import LegMatchInfo, Position, PositionSnapshot

OptionsProviderName = Literal["tradier", "polygon", "yfinance"]

LiveMode = Literal["off", "paper", "live"]

_DEFAULT_RISK_LIMITS: dict[str, float | int] = {
    "max_open_positions": 10,
    "max_notional_per_symbol": 50_000.0,
    "max_total_notional": 100_000.0,
}


class AtlasOptionsRiskError(RuntimeError):
    """
    Apertura bloqueada por política de riesgo (límites globales).

    Atributos para consumo programático del planner:
    - ``code``: identificador corto del límite violado.
    - ``detail``: métricas relevantes (límites, uso actual, propuesta).
    """

    def __init__(
        self,
        message: str,
        *,
        code: str,
        detail: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.detail = dict(detail or {})


_VALID_STRATEGIES: frozenset[str] = frozenset(
    {
        "iron_condor",
        "bull_put",
        "bear_call",
        "bull_call",
        "bear_put",
        "covered_call",
    }
)

_ALLOWED_PARAM_KEYS: dict[str, frozenset[str]] = {
    "iron_condor": frozenset({"wing_delta", "wing_width", "qty"}),
    "bull_put": frozenset({"short_delta", "width", "qty"}),
    "bear_call": frozenset({"short_delta", "width", "qty"}),
    "bull_call": frozenset({"long_delta", "width", "qty"}),
    "bear_put": frozenset({"long_delta", "width", "qty"}),
    "covered_call": frozenset({"short_delta", "stock_basis", "qty"}),
}


def _make_provider(kind: OptionsProviderName, config: dict) -> OptionsDataProvider:
    if kind == "tradier":
        return TradierProvider(
            token=config.get("token") or config.get("tradier_token"),
            sandbox=bool(config.get("sandbox", True)),
        )
    if kind == "polygon":
        return PolygonProvider(
            api_key=config.get("api_key") or config.get("polygon_api_key"),
        )
    if kind == "yfinance":
        return YFinanceProvider()
    raise ValueError(f"options_provider_type desconocido: {kind!r}")


def _validate_strategy_type(strategy_type: str) -> StrategyType:
    if strategy_type not in _VALID_STRATEGIES:
        raise ValueError(
            f"strategy_type inválido: {strategy_type!r}. "
            f"Válidos: {sorted(_VALID_STRATEGIES)}"
        )
    return strategy_type  # type: ignore[return-value]


def _validate_params(strategy_type: str, params: dict) -> None:
    allowed = _ALLOWED_PARAM_KEYS[strategy_type]
    extra = set(params) - allowed
    if extra:
        raise ValueError(
            f"Claves de params no permitidas para {strategy_type!r}: {sorted(extra)}"
        )

    def _num(name: str) -> None:
        if name not in params:
            return
        v = params[name]
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            raise TypeError(f"{name} debe ser numérico (int/float), got {type(v).__name__}")

    def _positive(name: str, strict: bool = True) -> None:
        if name not in params:
            return
        _num(name)
        v = float(params[name])
        if strict and v <= 0:
            raise ValueError(f"{name} debe ser > 0, got {v}")

    if strategy_type == "iron_condor":
        _num("wing_delta")
        _num("wing_width")
        _positive("wing_width")
        if "wing_delta" in params and not (0 < float(params["wing_delta"]) < 1):
            raise ValueError("wing_delta debe estar en (0, 1)")
    elif strategy_type in ("bull_put", "bear_call", "covered_call"):
        _num("short_delta")
        _positive("width")
        if "short_delta" in params and not (0 < float(params["short_delta"]) < 1):
            raise ValueError("short_delta debe estar en (0, 1)")
        _num("stock_basis")
    elif strategy_type in ("bull_call", "bear_put"):
        _num("long_delta")
        _positive("width")
        if "long_delta" in params and not (0 < float(params["long_delta"]) < 1):
            raise ValueError("long_delta debe estar en (0, 1)")

    if "qty" in params:
        q = params["qty"]
        if not isinstance(q, int) or isinstance(q, bool) or q < 1:
            raise ValueError("qty debe ser int >= 1")


def _match_info_to_json(info: list[LegMatchInfo]) -> list[dict[str, Any]]:
    return [
        {
            "leg_index": m.leg_index,
            "match_kind": m.match_kind.value,
            "original_contract_symbol": m.original_contract_symbol,
            "used_contract_symbol": m.used_contract_symbol,
        }
        for m in info
    ]


def _snapshot_to_dict(snap: PositionSnapshot | None) -> dict[str, Any] | None:
    if snap is None:
        return None
    out: dict[str, Any] = {
        "timestamp": snap.timestamp.isoformat(),
        "unrealized_pnl": snap.unrealized_pnl,
        "entry_net_premium": snap.entry_net_premium,
        "leg_mids": list(snap.leg_mids),
    }
    if snap.match_info is not None:
        out["match_info"] = _match_info_to_json(snap.match_info)
    return out


class AtlasOptionsService:
    """
    Adapter estable para el planner: proveedor configurable + AtlasOptionsClient.

    **Notional aproximado al abrir** (documentación para ``risk_limits``):

    - *Crédito* (``iron_condor``, ``bull_put``, ``bear_call``):
      ``width_max * 100 * qty``, donde ``width_max`` es el ancho del vertical
      más ancho (IC: máximo entre ala put y ala call; verticales: |Δstrike| entre
      short y long). Representa capital en riesgo máximo aproximado del spread.
    - *Débito* (``bull_call``, ``bear_put``): ``abs(net_premium)`` (costo neto en USD).
    - *Covered call*: ``spot * 100 * qty`` con ``spot`` vía ``provider.get_quote(symbol)``.

    Para tests, pasar ``provider=`` y se ignora la construcción del proveedor desde
    ``config`` (sigue siendo obligatorio un ``options_provider_type`` válido).
    """

    def __init__(
        self,
        options_provider_type: OptionsProviderName,
        config: dict,
        *,
        provider: Optional[OptionsDataProvider] = None,
        as_of: Optional[date] = None,
        risk_limits: dict[str, Any] | None = None,
        live_enabled: bool = False,
        live_mode: LiveMode = "off",
    ) -> None:
        self._provider = provider if provider is not None else _make_provider(
            options_provider_type, dict(config or {})
        )
        self._client = AtlasOptionsClient(self._provider, as_of=as_of)
        merged = dict(_DEFAULT_RISK_LIMITS)
        if risk_limits:
            merged.update({k: v for k, v in risk_limits.items() if v is not None})
        self._max_open_positions = int(merged["max_open_positions"])
        self._max_notional_per_symbol = float(merged["max_notional_per_symbol"])
        self._max_total_notional = float(merged["max_total_notional"])
        self._risk_ledger: dict[str, tuple[str, float]] = {}
        self._live_enabled = bool(live_enabled)
        self._live_mode: LiveMode = live_mode

    @property
    def client(self) -> AtlasOptionsClient:
        """Acceso al cliente de bajo nivel (p. ej. tests)."""
        return self._client

    @property
    def live_enabled(self) -> bool:
        """Flag para futura ejecución real; en este slice debe permanecer ``False`` en producción."""
        return self._live_enabled

    @property
    def live_mode(self) -> LiveMode:
        """``off`` = solo paper interno; ``paper``/``live`` reservados — live aún no implementado."""
        return self._live_mode

    @property
    def risk_limits_snapshot(self) -> dict[str, float | int]:
        """Límites efectivos (útil para telemetría del planner)."""
        return {
            "max_open_positions": self._max_open_positions,
            "max_notional_per_symbol": self._max_notional_per_symbol,
            "max_total_notional": self._max_total_notional,
        }

    def _prune_risk_ledger(self) -> None:
        open_ids = {p.position_id for p in self._client.list_open_positions()}
        for pid in list(self._risk_ledger.keys()):
            if pid not in open_ids:
                del self._risk_ledger[pid]

    def _estimate_open_risk_notional(
        self,
        strategy: OptionsStrategy,
        *,
        symbol: str,
    ) -> float:
        """USD aproximados de riesgo/capital asignado para la apertura (ver doc de clase)."""
        legs = strategy.legs
        qty = max((lg.qty for lg in legs), default=1)

        if isinstance(strategy, IronCondor):
            if not (
                strategy.put_short
                and strategy.put_long
                and strategy.call_short
                and strategy.call_long
            ):
                raise ValueError("IronCondor incompleto.")
            w_put = abs(
                strategy.put_short.contract.strike - strategy.put_long.contract.strike
            )
            w_call = abs(
                strategy.call_long.contract.strike - strategy.call_short.contract.strike
            )
            width_max = max(w_put, w_call)
            return float(width_max * 100.0 * qty)

        if isinstance(strategy, BullPutSpread):
            if not (strategy.put_short and strategy.put_long):
                raise ValueError("BullPutSpread incompleto.")
            w = abs(
                strategy.put_short.contract.strike - strategy.put_long.contract.strike
            )
            return float(w * 100.0 * qty)

        if isinstance(strategy, BearCallSpread):
            if not (strategy.call_short and strategy.call_long):
                raise ValueError("BearCallSpread incompleto.")
            w = abs(
                strategy.call_long.contract.strike - strategy.call_short.contract.strike
            )
            return float(w * 100.0 * qty)

        if isinstance(strategy, BullCallSpread):
            return float(abs(strategy.net_premium))

        if isinstance(strategy, BearPutSpread):
            return float(abs(strategy.net_premium))

        if isinstance(strategy, CoveredCall):
            if not strategy.call_short:
                raise ValueError("CoveredCall incompleto.")
            spot = float(self._provider.get_quote(symbol))
            return float(spot * 100.0 * strategy.call_short.qty)

        raise TypeError(f"Estrategia no soportada para notional: {type(strategy).__name__}")

    def _check_risk_limits(self, *, symbol: str, proposed_notional: float) -> None:
        """
        Valida límites globales antes de abrir. Lanza ``AtlasOptionsRiskError`` si aplica.
        """
        self._prune_risk_ledger()
        open_list = self._client.list_open_positions()
        n_open = len(open_list)
        if n_open >= self._max_open_positions:
            raise AtlasOptionsRiskError(
                f"Límite de posiciones abiertas: {n_open} >= {self._max_open_positions}.",
                code="max_open_positions",
                detail={
                    "current_open": n_open,
                    "limit": self._max_open_positions,
                },
            )

        sym_used = sum(
            n for _, (sym, n) in self._risk_ledger.items() if sym == symbol
        )
        if sym_used + proposed_notional > self._max_notional_per_symbol:
            raise AtlasOptionsRiskError(
                f"Notional por símbolo {symbol!r} excedería el límite: "
                f"{sym_used + proposed_notional:.2f} > {self._max_notional_per_symbol:.2f}.",
                code="max_notional_per_symbol",
                detail={
                    "symbol": symbol,
                    "current_symbol_notional": sym_used,
                    "proposed": proposed_notional,
                    "limit": self._max_notional_per_symbol,
                },
            )

        total_used = sum(n for _, n in self._risk_ledger.values())
        if total_used + proposed_notional > self._max_total_notional:
            raise AtlasOptionsRiskError(
                f"Notional total excedería el límite: "
                f"{total_used + proposed_notional:.2f} > {self._max_total_notional:.2f}.",
                code="max_total_notional",
                detail={
                    "current_total_notional": total_used,
                    "proposed": proposed_notional,
                    "limit": self._max_total_notional,
                },
            )

    def place_live_order(self, position_id: str) -> None:
        """
        Stub de envío de órdenes al broker. **No implementado.**

        Con ``live_mode='off'`` (por defecto) siempre falla de forma explícita.
        """
        _ = position_id
        if self._live_mode == "off":
            raise RuntimeError(
                "place_live_order: live_mode='off'. Órdenes reales deshabilitadas; "
                "solo simulación paper."
            )
        if self._live_mode == "paper":
            raise RuntimeError(
                "place_live_order: live_mode='paper' indica solo simulación interna; "
                "no hay envío a broker."
            )
        if not self._live_enabled:
            raise RuntimeError(
                "place_live_order: live_enabled=False. Active live_enabled cuando exista integración."
            )
        raise NotImplementedError(
            "place_live_order: integración con broker no implementada en este slice."
        )

    def sync_live_positions(self) -> None:
        """
        Stub de reconciliación con cuenta real. **No implementado.**

        Requiere ``live_mode='live'`` y ``live_enabled=True``; de lo contrario falla.
        """
        if self._live_mode != "live":
            raise RuntimeError(
                f"sync_live_positions solo aplica con live_mode='live'; "
                f"actualmente live_mode={self._live_mode!r}."
            )
        if not self._live_enabled:
            raise RuntimeError("sync_live_positions: live_enabled=False.")
        raise NotImplementedError(
            "sync_live_positions: integración con broker no implementada en este slice."
        )

    def build_and_open(
        self,
        symbol: str,
        strategy_type: StrategyType,
        params: dict | None = None,
    ) -> str:
        st = _validate_strategy_type(strategy_type)
        p = dict(params or {})
        _validate_params(st, p)
        strat = self._client.build_strategy_from_chain(symbol, st, params=p)
        proposed = self._estimate_open_risk_notional(strat, symbol=symbol)
        self._check_risk_limits(symbol=symbol, proposed_notional=proposed)
        pos = self._client.open_paper_position(strat, atlas_strategy_type=st)
        self._risk_ledger[pos.position_id] = (symbol, proposed)
        return pos.position_id

    def list_open_positions(self) -> list[Position]:
        """Posiciones paper abiertas (delegado al cliente brain)."""
        return self._client.list_open_positions()

    def list_closed_positions(self) -> list[Position]:
        """Posiciones paper cerradas (historial del simulador)."""
        return self._client.list_closed_positions()

    def mark_all(self) -> list[dict[str, Any]]:
        positions = self._client.list_open_positions()
        snaps = self._client.mark_all_positions()
        rows: list[dict[str, Any]] = []
        for pos, snap in zip(positions, snaps):
            meta = self._client.get_position_meta(pos.position_id)
            rows.append(
                {
                    "id": pos.position_id,
                    "symbol": meta.get("symbol", ""),
                    "strategy_type": meta.get("strategy_type", ""),
                    "net_premium": pos.entry_net_premium,
                    "uPnL": snap.unrealized_pnl,
                }
            )
        return rows

    def get_position(self, position_id: str) -> dict[str, Any]:
        pos = self._client.get_position(position_id)
        if pos is None:
            raise KeyError(f"Posición no encontrada: {position_id!r}")
        meta = self._client.get_position_meta(position_id)
        last = pos.last_snapshot
        return {
            "position_id": pos.position_id,
            "symbol": meta.get("symbol", ""),
            "strategy_type": meta.get("strategy_type", ""),
            "is_open": pos.is_open,
            "entry_net_premium": pos.entry_net_premium,
            "opened_at": pos.opened_at.isoformat(),
            "closed_at": pos.closed_at.isoformat() if pos.closed_at else None,
            "realized_pnl": pos.realized_pnl,
            "snapshot_count": pos.snapshot_count,
            "last_snapshot": _snapshot_to_dict(last),
        }
