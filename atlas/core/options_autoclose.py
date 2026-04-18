"""
Política configurable de cierre automático (paper + sandbox Tradier).

Centrado en PnL vs. ``entry_net_premium`` (referencia ``abs(entry)`` para umbrales %).
``min_dte`` / ``max_dte`` están reservados para un slice futuro (no usados aún).

Flujo típico en backend::

    engine = OptionsAutoCloseEngine(svc, config)
    closer = OptionsAutoCloser(svc, engine, live_service)
    for row in closer.close_candidates(preview=True):
        log OptionStrat / auditoría
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Literal

from atlas_options_brain.integration.atlas_adapter import StrategyType

from .options_client import AtlasOptionsService
from .options_live import AtlasOptionsLiveService

AutoCloseStrategySpec = StrategyType | Literal["*"]


@dataclass
class AutoCloseRule:
    """
    ``take_profit_pct`` / ``stop_loss_pct`` se aplican sobre ``abs(entry_net_premium)``.

    Ej.: crédito neto 100 USD, ``take_profit_pct=0.6`` → cierra si uPnL >= 60.
    ``stop_loss_pct=-1.5`` → cierra si uPnL <= -150.
    """

    strategy_type: AutoCloseStrategySpec = "*"
    take_profit_pct: float | None = None
    stop_loss_pct: float | None = None
    min_dte: int | None = None
    max_dte: int | None = None
    min_pnl_abs: float | None = None
    max_pnl_abs: float | None = None


@dataclass
class AutoCloseConfig:
    rules: list[AutoCloseRule] = field(default_factory=list)
    default_rule: AutoCloseRule | None = None


class OptionsAutoCloseEngine:
    """Evalúa ``get_position`` dicts contra ``AutoCloseConfig``."""

    def __init__(
        self,
        options_service: AtlasOptionsService,
        config: AutoCloseConfig,
    ) -> None:
        self._svc = options_service
        self._config = config

    @property
    def config(self) -> AutoCloseConfig:
        return self._config

    def _select_rule(self, strategy_type: str) -> AutoCloseRule | None:
        st = str(strategy_type or "").strip()
        for r in self._config.rules:
            if r.strategy_type != "*" and r.strategy_type == st:
                return r
        for r in self._config.rules:
            if r.strategy_type == "*":
                return r
        return self._config.default_rule

    def evaluate_position(self, pos_dict: dict[str, Any]) -> dict[str, Any]:
        pid = str(pos_dict.get("position_id", ""))
        st = str(pos_dict.get("strategy_type", "") or "")
        entry = float(pos_dict.get("entry_net_premium", 0.0))
        baseline = abs(entry) if abs(entry) > 1e-12 else 0.0

        snap = pos_dict.get("last_snapshot")
        pnl = 0.0
        if isinstance(snap, dict) and snap.get("unrealized_pnl") is not None:
            pnl = float(snap["unrealized_pnl"])

        rule = self._select_rule(st)
        reasons: list[str] = []
        if rule is None:
            return {
                "position_id": pid,
                "should_close": False,
                "reasons": [],
                "rule_applied": None,
                "pnl": round(pnl, 4),
                "entry_net_premium": entry,
            }

        if rule.take_profit_pct is not None and baseline > 0:
            target = baseline * float(rule.take_profit_pct)
            if pnl >= target:
                reasons.append("take_profit")

        if rule.stop_loss_pct is not None and baseline > 0:
            stop_thr = baseline * float(rule.stop_loss_pct)
            if pnl <= stop_thr:
                reasons.append("stop_loss")

        if rule.min_pnl_abs is not None and pnl >= float(rule.min_pnl_abs):
            reasons.append("min_pnl_abs")

        if rule.max_pnl_abs is not None and pnl <= -float(rule.max_pnl_abs):
            reasons.append("max_pnl_abs")

        should_close = len(reasons) > 0
        return {
            "position_id": pid,
            "should_close": should_close,
            "reasons": reasons,
            "rule_applied": asdict(rule),
            "pnl": round(pnl, 4),
            "entry_net_premium": entry,
        }

    def scan_open_positions(self) -> list[dict[str, Any]]:
        self._svc.mark_all()
        out: list[dict[str, Any]] = []
        for p in self._svc.list_open_positions():
            d = self._svc.get_position(p.position_id)
            out.append(self.evaluate_position(d))
        return out


class OptionsAutoCloser:
    """Encuentra candidatos y envía cierre sandbox vía ``AtlasOptionsLiveService``."""

    def __init__(
        self,
        options_service: AtlasOptionsService,
        engine: OptionsAutoCloseEngine,
        live_service: AtlasOptionsLiveService,
    ) -> None:
        self._options = options_service
        self._engine = engine
        self._live = live_service

    def find_candidates(self) -> list[dict[str, Any]]:
        return [row for row in self._engine.scan_open_positions() if row["should_close"]]

    def close_candidates(self, *, preview: bool = True) -> list[dict[str, Any]]:
        results: list[dict[str, Any]] = []
        for ev in self.find_candidates():
            pid = str(ev["position_id"])
            send = self._live.send_close_sandbox(pid, preview=preview)
            row: dict[str, Any] = {
                "position_id": pid,
                "preview": preview,
                "auto_close_reasons": list(ev.get("reasons", [])),
                "evaluation": ev,
            }
            row.update(send)
            results.append(row)
        return results
