from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

try:
    from atlas_code_quant.config.settings import settings
except ModuleNotFoundError:  # pragma: no cover - runtime fallback for package imports
    from config.settings import settings


@dataclass(frozen=True)
class VarSnapshot:
    enabled: bool
    method: str
    simulation_count: int
    confidence_level_pct: float
    horizon_days: int
    var_95_usd: float
    cvar_95_usd: float
    var_95_pct_of_book: float
    threshold_usd: float
    status: str
    drivers: list[str]
    diversified_risk_usd: float
    gross_risk_usd: float
    concentration_multiplier: float
    loss_multiplier: float
    monte_carlo_var_usd: float
    monte_carlo_cvar_usd: float
    expected_loss_usd: float
    worst_case_loss_usd: float
    net_directional_exposure_pct: float


class VarMonitor:
    def __init__(self, *, state_path: Path | None = None) -> None:
        self.state_path = state_path or (settings.data_dir.parent / "operation" / "operation_center_state.json")

    def is_enabled(self) -> bool:
        try:
            payload = json.loads(self.state_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return True
        except (json.JSONDecodeError, OSError):
            return True
        return bool(payload.get("var_monitor_enabled", True))

    @staticmethod
    def _safe_float(value: Any, default: float = 0.0) -> float:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return default
        if numeric != numeric or math.isinf(numeric):
            return default
        return numeric

    @staticmethod
    def _clip(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _empty_snapshot(self, *, enabled: bool) -> VarSnapshot:
        return VarSnapshot(
            enabled=enabled,
            method="monte_carlo_portfolio" if enabled else "disabled",
            simulation_count=int(settings.position_management_var_mc_scenarios),
            confidence_level_pct=float(settings.position_management_var_confidence_pct),
            horizon_days=int(settings.position_management_var_horizon_days),
            var_95_usd=0.0,
            cvar_95_usd=0.0,
            var_95_pct_of_book=0.0,
            threshold_usd=0.0,
            status="ok" if enabled else "disabled",
            drivers=[],
            diversified_risk_usd=0.0,
            gross_risk_usd=0.0,
            concentration_multiplier=1.0,
            loss_multiplier=1.0,
            monte_carlo_var_usd=0.0,
            monte_carlo_cvar_usd=0.0,
            expected_loss_usd=0.0,
            worst_case_loss_usd=0.0,
            net_directional_exposure_pct=0.0,
        )

    def _strategy_direction(self, strategy_type: str) -> tuple[float, float]:
        strategy = str(strategy_type or "").lower()
        if any(token in strategy for token in ("equity_long", "long_call", "bull_call", "bull_put", "call_calendar", "call_diagonal")):
            return 1.0, 1.0
        if any(token in strategy for token in ("equity_short", "long_put", "bear_put", "bear_call", "put_calendar", "put_diagonal")):
            return -1.0, 1.0
        if any(token in strategy for token in ("iron_condor", "iron_butterfly", "butterfly", "condor")):
            return 0.0, 0.45
        if "credit" in strategy:
            return 0.35, 0.7
        if "debit" in strategy:
            return 0.75, 0.85
        return 0.5, 0.75

    def _convexity_factor(self, strategy_type: str) -> float:
        strategy = str(strategy_type or "").lower()
        if "equity_" in strategy:
            return 1.0
        if any(token in strategy for token in ("long_call", "long_put")):
            return 1.15
        if any(token in strategy for token in ("iron_condor", "iron_butterfly", "butterfly", "condor")):
            return 0.55
        if "calendar" in strategy or "diagonal" in strategy:
            return 0.7
        if "credit" in strategy:
            return 0.75
        if "debit" in strategy:
            return 0.85
        return 0.9

    def _seed_for_positions(self, positions: list[dict[str, Any]]) -> int:
        basis = "|".join(
            f"{row.get('symbol','')}:{row.get('strategy_type','')}:{round(self._safe_float(row.get('risk_budget'), 0.0), 2)}"
            for row in positions
        )
        return sum(ord(ch) for ch in basis) % (2**32 - 1) if basis else 42

    def _monte_carlo_portfolio(
        self,
        positions: list[dict[str, Any]],
        *,
        total_entry_notional: float,
    ) -> dict[str, float]:
        if not positions or total_entry_notional <= 0:
            return {
                "var_usd": 0.0,
                "cvar_usd": 0.0,
                "expected_loss_usd": 0.0,
                "worst_case_loss_usd": 0.0,
                "net_directional_exposure_pct": 0.0,
            }

        scenario_count = int(settings.position_management_var_mc_scenarios)
        horizon_days = int(settings.position_management_var_horizon_days)
        confidence_pct = float(settings.position_management_var_confidence_pct)
        horizon_scale = math.sqrt(float(horizon_days))
        rng = np.random.default_rng(self._seed_for_positions(positions))

        market_factor = rng.normal(0.0, 1.0, scenario_count)
        symbol_factors: dict[str, np.ndarray] = {}
        strategy_factors: dict[str, np.ndarray] = {}
        pnl_matrix: list[np.ndarray] = []
        net_directional_notional = 0.0

        for row in positions:
            symbol = str(row.get("symbol") or "unknown")
            strategy_type = str(row.get("strategy_type") or "unknown")
            notional = max(self._safe_float(row.get("entry_notional"), 0.0), 1.0)
            risk_budget = max(self._safe_float(row.get("risk_budget"), 0.0), 1.0)
            unrealized_pnl = self._safe_float(row.get("unrealized_pnl"), 0.0)
            direction_sign, directional_abs = self._strategy_direction(strategy_type)
            convexity = self._convexity_factor(strategy_type)
            risk_pct = self._clip(risk_budget / max(notional, 1.0), 0.003, 0.25)
            base_vol = self._clip(risk_pct * 0.55 * horizon_scale, 0.006, 0.09)
            idio_factor = rng.normal(0.0, 1.0, scenario_count)
            symbol_factor = symbol_factors.setdefault(symbol, rng.normal(0.0, 1.0, scenario_count))
            strategy_factor = strategy_factors.setdefault(strategy_type, rng.normal(0.0, 1.0, scenario_count))
            drift = self._clip(unrealized_pnl / max(notional, 1.0), -base_vol * 0.55, base_vol * 0.30)
            composite = (
                (market_factor * 0.48)
                + (strategy_factor * 0.18)
                + (symbol_factor * 0.16)
                + (idio_factor * 0.28)
            )
            if abs(direction_sign) < 0.1:
                pnl = -(np.abs(composite) * notional * base_vol * convexity * directional_abs)
            else:
                pnl = ((composite + drift) * notional * base_vol * direction_sign * convexity)
            max_loss = risk_budget * (1.55 if convexity >= 1.0 else 1.35)
            max_gain = risk_budget * (2.10 if direction_sign != 0.0 else 0.55)
            pnl = np.clip(pnl, -max_loss, max_gain)
            pnl_matrix.append(pnl)
            net_directional_notional += notional * direction_sign * directional_abs

        portfolio_pnl = np.sum(np.vstack(pnl_matrix), axis=0)
        losses = np.maximum(-portfolio_pnl, 0.0)
        quantile = confidence_pct / 100.0
        var_usd = float(np.quantile(losses, quantile))
        cvar_usd = float(losses[losses >= var_usd].mean()) if np.any(losses >= var_usd) else var_usd
        expected_loss = float(losses.mean())
        worst_loss = float(losses.max()) if losses.size else 0.0
        directional_pct = (net_directional_notional / max(total_entry_notional, 1.0)) * 100.0
        return {
            "var_usd": round(var_usd, 4),
            "cvar_usd": round(cvar_usd, 4),
            "expected_loss_usd": round(expected_loss, 4),
            "worst_case_loss_usd": round(worst_loss, 4),
            "net_directional_exposure_pct": round(directional_pct, 4),
        }

    def compute(
        self,
        positions: list[dict[str, Any]],
        *,
        total_risk_budget: float,
        total_entry_notional: float,
        max_symbol_heat_pct: float,
        adverse_positions_count: int,
        total_unrealized_pnl: float,
    ) -> VarSnapshot:
        if not self.is_enabled():
            return self._empty_snapshot(enabled=False)
        if not positions:
            return self._empty_snapshot(enabled=True)

        risk_budgets = [max(self._safe_float(row.get("risk_budget"), 0.0), 0.0) for row in positions]
        gross_risk = round(sum(risk_budgets), 4)
        diversified_risk = round(math.sqrt(sum(risk * risk for risk in risk_budgets)), 4) if risk_budgets else 0.0
        concentration_multiplier = 1.0 + min(max(self._safe_float(max_symbol_heat_pct, 0.0), 0.0), 100.0) / 200.0
        adverse_ratio = min(max(adverse_positions_count, 0), max(len(positions), 1)) / max(len(positions), 1)
        unrealized_loss_ratio = min(
            max(-self._safe_float(total_unrealized_pnl, 0.0), 0.0) / max(self._safe_float(total_risk_budget, 1.0), 1.0),
            1.0,
        )
        loss_multiplier = 1.0 + adverse_ratio * 0.35 + unrealized_loss_ratio * 0.25
        heuristic_var = round(diversified_risk * concentration_multiplier * loss_multiplier, 4)
        heuristic_cvar = round(heuristic_var * 1.2, 4)

        mc = self._monte_carlo_portfolio(
            positions,
            total_entry_notional=max(self._safe_float(total_entry_notional, 0.0), 0.0),
        )
        final_var = round((mc["var_usd"] * 0.68) + (heuristic_var * 0.32), 4)
        final_cvar = round((mc["cvar_usd"] * 0.68) + (heuristic_cvar * 0.32), 4)
        var_95_pct_of_book = round((final_var / max(self._safe_float(total_entry_notional, 1.0), 1.0)) * 100.0, 4)
        threshold_usd = round(max(self._safe_float(total_risk_budget, 0.0) * 0.6, diversified_risk * 1.35), 4)

        drivers: list[str] = []
        if max_symbol_heat_pct >= 45.0:
            drivers.append("symbol_concentration")
        if adverse_ratio >= 0.34:
            drivers.append("adverse_positions")
        if unrealized_loss_ratio >= 0.2:
            drivers.append("open_loss_pressure")
        if abs(mc["net_directional_exposure_pct"]) >= 55.0:
            drivers.append("directional_imbalance")
        if not drivers and positions:
            drivers.append("monte_carlo_portfolio")

        status = "ok"
        if final_var > threshold_usd or var_95_pct_of_book >= 6.0:
            status = "warning"
        if final_var > threshold_usd * 1.2 or var_95_pct_of_book >= 8.0:
            status = "critical"

        return VarSnapshot(
            enabled=True,
            method="monte_carlo_portfolio",
            simulation_count=int(settings.position_management_var_mc_scenarios),
            confidence_level_pct=float(settings.position_management_var_confidence_pct),
            horizon_days=int(settings.position_management_var_horizon_days),
            var_95_usd=final_var,
            cvar_95_usd=final_cvar,
            var_95_pct_of_book=var_95_pct_of_book,
            threshold_usd=threshold_usd,
            status=status,
            drivers=drivers,
            diversified_risk_usd=diversified_risk,
            gross_risk_usd=round(gross_risk, 4),
            concentration_multiplier=round(concentration_multiplier, 4),
            loss_multiplier=round(loss_multiplier, 4),
            monte_carlo_var_usd=mc["var_usd"],
            monte_carlo_cvar_usd=mc["cvar_usd"],
            expected_loss_usd=mc["expected_loss_usd"],
            worst_case_loss_usd=mc["worst_case_loss_usd"],
            net_directional_exposure_pct=mc["net_directional_exposure_pct"],
        )
