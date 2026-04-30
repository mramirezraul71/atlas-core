"""Broker-first canonical snapshot service for Quant, UI and Grafana."""
from __future__ import annotations

from datetime import datetime, timezone
import logging
import math
from pathlib import Path
from typing import Any

from atlas_code_quant.monitoring.strategy_tracker import StrategyTracker, TrackerSnapshot
from atlas_code_quant.paper.paper_broker import get_paper_broker

try:
    from options.portfolio_analyzer import PortfolioAnalyzer
except ModuleNotFoundError:
    from atlas_code_quant.options.portfolio_analyzer import PortfolioAnalyzer


logger = logging.getLogger("quant.monitoring.canonical_snapshot")


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _position_multiplier(asset_class: str) -> float:
    return 100.0 if asset_class == "option" else 1.0


def _normalize_scope(account_scope: str | None) -> str:
    scope = str(account_scope or "paper").strip().lower()
    return scope or "paper"


def _round_or_none(value: float | None, digits: int = 4) -> float | None:
    if value is None:
        return None
    return round(float(value), digits)


class CanonicalSnapshotService:
    """Expose a single Tradier-first snapshot with reconciliation metadata."""

    def __init__(self, tracker: StrategyTracker) -> None:
        self._tracker = tracker
        self._optionstrat = PortfolioAnalyzer(
            storage_path=Path(__file__).resolve().parents[1] / "data" / "options_portfolio.json"
        )

    def build_snapshot(
        self,
        *,
        account_scope: str | None = None,
        account_id: str | None = None,
        internal_portfolio: Any = None,
    ) -> dict[str, Any]:
        scope = _normalize_scope(account_scope)
        tracker_snapshot = self._tracker.snapshot(account_scope=scope, account_id=account_id)  # type: ignore[arg-type]
        summary = self._tracker.build_summary(
            account_scope=scope,
            account_id=account_id,
            snapshot=tracker_snapshot,
        )  # type: ignore[arg-type]
        positions = self._build_positions_payload(tracker_snapshot)
        totals = self._build_totals(tracker_snapshot, positions)
        balances = self._build_balances(summary, totals, tracker_snapshot)
        simulators = self._build_simulators(
            tracker_snapshot=tracker_snapshot,
            internal_portfolio=internal_portfolio,
        )
        reconciliation = self._build_reconciliation(
            canonical_balances=balances,
            canonical_totals=totals,
            simulators=simulators,
        )
        generated_at = datetime.now(timezone.utc).isoformat()
        return {
            "generated_at": generated_at,
            "updated_at": generated_at,
            "source": "tradier",
            "source_label": "Tradier canonical",
            "account_scope": tracker_snapshot.session.scope,
            "account_id": tracker_snapshot.session.account_id,
            "account_session": tracker_snapshot.session.to_dict(),
            "balances": balances,
            "totals": totals,
            "positions": positions,
            "strategies": summary.get("strategies") or [],
            "alerts": summary.get("alerts") or [],
            "pdt_status": summary.get("pdt_status") or {},
            "refresh_interval_sec": summary.get("refresh_interval_sec"),
            "monitor_summary": summary,
            "simulators": simulators,
            "reconciliation": reconciliation,
        }

    def build_status_payload(
        self,
        *,
        account_scope: str | None = None,
        account_id: str | None = None,
        uptime_sec: float,
        active_strategies: list[str],
        internal_portfolio: Any = None,
        pdt_status: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        snapshot = self.build_snapshot(
            account_scope=account_scope,
            account_id=account_id,
            internal_portfolio=internal_portfolio,
        )
        pdt = pdt_status if pdt_status is not None else snapshot.get("pdt_status") or {}
        return {
            "generated_at": snapshot["generated_at"],
            "service_status": "ok",
            "uptime_sec": uptime_sec,
            "source": snapshot["source"],
            "source_label": snapshot["source_label"],
            "canonical_scope": snapshot["account_scope"],
            "canonical_account_id": snapshot["account_id"],
            "canonical_updated_at": snapshot["updated_at"],
            "account_session": snapshot["account_session"],
            "balances": snapshot["balances"],
            "pdt_status": pdt,
            "days_trades_used": min(int(pdt.get("day_trades_last_window") or 0), 3),
            "active_strategies": list(active_strategies),
            "open_positions": int(snapshot["totals"].get("positions") or 0),
            "reconciliation": snapshot["reconciliation"],
            "simulators": snapshot["simulators"],
        }

    def build_positions_payload(
        self,
        *,
        account_scope: str | None = None,
        account_id: str | None = None,
        internal_portfolio: Any = None,
    ) -> dict[str, Any]:
        snapshot = self.build_snapshot(
            account_scope=account_scope,
            account_id=account_id,
            internal_portfolio=internal_portfolio,
        )
        return {
            "source": snapshot["source"],
            "source_label": snapshot["source_label"],
            "account_scope": snapshot["account_scope"],
            "account_id": snapshot["account_id"],
            "positions": snapshot["positions"],
            "open_positions": snapshot["totals"]["positions"],
            "total_pnl": snapshot["totals"]["open_pnl"],
            "market_value": snapshot["balances"]["market_value"],
            "gross_exposure": snapshot["balances"]["gross_exposure"],
            "reconciliation": snapshot["reconciliation"],
            "simulators": snapshot["simulators"],
        }

    def _build_positions_payload(self, tracker_snapshot: TrackerSnapshot) -> list[dict[str, Any]]:
        positions: list[dict[str, Any]] = []
        for position in tracker_snapshot.normalized_positions:
            multiplier = _position_multiplier(position.asset_class)
            gross_cost = abs(float(position.entry_price) * float(position.quantity_abs) * multiplier)
            market_value = abs(float(position.current_price) * float(position.quantity_abs) * multiplier)
            entry = max(float(position.entry_price), 1e-9)
            current = max(float(position.current_price), 1e-9)
            direction = 1.0 if position.signed_qty >= 0 else -1.0
            pnl_pct = (float(position.current_pnl) / gross_cost * 100.0) if gross_cost > 1e-9 else 0.0
            log_return = math.log(current / entry) * direction
            positions.append(
                {
                    "symbol": position.symbol,
                    "underlying": position.underlying,
                    "asset_class": position.asset_class,
                    "side": position.side,
                    "quantity": position.quantity_abs,
                    "signed_quantity": position.signed_qty,
                    "entry_price": round(float(position.entry_price), 4),
                    "current_price": round(float(position.current_price), 4),
                    "unrealized_pnl": round(float(position.current_pnl), 4),
                    "pnl_pct": round(float(pnl_pct), 4),
                    "log_return": round(float(log_return), 6),
                    "market_value": round(float(market_value), 4),
                    "gross_cost": round(float(gross_cost), 4),
                    "exposure": round(float(market_value), 4),
                    "stop_loss": None,
                    "take_profit": None,
                    "atr": None,
                    "strike": position.strike,
                    "option_type": position.option_type,
                    "expiration": position.expiration,
                    "dte": position.dte,
                }
            )
        positions.sort(key=lambda item: float(item.get("unrealized_pnl") or 0.0))
        return positions

    def _build_totals(
        self,
        tracker_snapshot: TrackerSnapshot,
        positions: list[dict[str, Any]],
    ) -> dict[str, Any]:
        open_pnl = round(sum(float(item.get("unrealized_pnl") or 0.0) for item in positions), 4)
        market_value = round(sum(float(item.get("market_value") or 0.0) for item in positions), 4)
        equity_positions = sum(1 for item in positions if item.get("asset_class") == "equity")
        option_positions = sum(1 for item in positions if item.get("asset_class") == "option")
        return {
            "positions": len(positions),
            "strategies": len(tracker_snapshot.groups),
            "open_pnl": open_pnl,
            "market_value": market_value,
            "gross_exposure": market_value,
            "equity_positions": equity_positions,
            "option_positions": option_positions,
        }

    def _build_balances(
        self,
        summary: dict[str, Any],
        totals: dict[str, Any],
        tracker_snapshot: TrackerSnapshot,
    ) -> dict[str, Any]:
        balances = dict(summary.get("balances") or {})
        balances["total_equity"] = _safe_float(
            balances.get("total_equity"),
            _safe_float(tracker_snapshot.session.total_equity, 0.0),
        )
        balances["cash"] = _safe_float(balances.get("cash"), 0.0)
        balances["margin"] = _safe_float(balances.get("margin"), 0.0)
        balances["option_buying_power"] = _safe_float(balances.get("option_buying_power"), 0.0)
        balances["current_requirement"] = _safe_float(balances.get("current_requirement"), 0.0)
        balances["market_value"] = _safe_float(totals.get("market_value"), 0.0)
        balances["gross_exposure"] = _safe_float(totals.get("gross_exposure"), 0.0)
        balances["open_pnl"] = _safe_float(totals.get("open_pnl"), 0.0)
        return balances

    def _build_simulators(
        self,
        *,
        tracker_snapshot: TrackerSnapshot,
        internal_portfolio: Any = None,
    ) -> dict[str, Any]:
        return {
            "paper_local": self._paper_local_snapshot(),
            "optionstrat": self._optionstrat_snapshot(tracker_snapshot),
            "atlas_internal": self._atlas_internal_snapshot(internal_portfolio),
        }

    def _paper_local_snapshot(self) -> dict[str, Any]:
        try:
            summary = get_paper_broker().get_account_summary().to_dict()
            return {
                "role": "simulation",
                "label": "Paper local",
                "status": "ok",
                "source": "paper_local",
                "equity": _safe_float(summary.get("equity"), 0.0),
                "open_positions": int(summary.get("open_positions") or 0),
                "open_pnl": _safe_float(summary.get("unrealized_pnl"), 0.0),
                "updated_at": summary.get("last_updated"),
            }
        except Exception as exc:
            logger.debug("Paper local snapshot unavailable: %s", exc)
            return {
                "role": "simulation",
                "label": "Paper local",
                "status": "unavailable",
                "source": "paper_local",
                "equity": None,
                "open_positions": None,
                "open_pnl": None,
                "updated_at": None,
                "error": str(exc),
            }

    def _optionstrat_snapshot(self, tracker_snapshot: TrackerSnapshot) -> dict[str, Any]:
        try:
            self._optionstrat.sync_from_broker_groups(
                groups=tracker_snapshot.groups,
                quote_index=tracker_snapshot.quote_index,
                source=f"tradier_{tracker_snapshot.session.scope}",
            )
            status = self._optionstrat.status()
            return {
                "role": "design",
                "label": "OptionStrat",
                "status": "ok",
                "source": "optionstrat",
                "equity": None,
                "open_positions": int(status.get("n_open") or 0),
                "open_pnl": None,
                "strategies": status.get("strategies") or [],
                "updated_at": datetime.now(timezone.utc).isoformat(),
            }
        except Exception as exc:
            logger.debug("OptionStrat snapshot unavailable: %s", exc)
            return {
                "role": "design",
                "label": "OptionStrat",
                "status": "unavailable",
                "source": "optionstrat",
                "equity": None,
                "open_positions": None,
                "open_pnl": None,
                "strategies": [],
                "updated_at": None,
                "error": str(exc),
            }

    def _atlas_internal_snapshot(self, internal_portfolio: Any = None) -> dict[str, Any]:
        if internal_portfolio is None:
            return {
                "role": "internal_model",
                "label": "Atlas internal",
                "status": "unavailable",
                "source": "atlas_internal",
                "equity": None,
                "open_positions": None,
                "open_pnl": None,
                "updated_at": None,
            }
        try:
            summary = internal_portfolio.summary()
            return {
                "role": "internal_model",
                "label": "Atlas internal",
                "status": "ok",
                "source": "atlas_internal",
                "equity": _safe_float(summary.get("current_equity"), 0.0),
                "open_positions": int(summary.get("open_positions") or 0),
                "open_pnl": _safe_float(summary.get("total_pnl"), 0.0),
                "updated_at": datetime.utcnow().isoformat(),
            }
        except Exception as exc:
            logger.debug("Atlas internal snapshot unavailable: %s", exc)
            return {
                "role": "internal_model",
                "label": "Atlas internal",
                "status": "unavailable",
                "source": "atlas_internal",
                "equity": None,
                "open_positions": None,
                "open_pnl": None,
                "updated_at": None,
                "error": str(exc),
            }

    def _build_reconciliation(
        self,
        *,
        canonical_balances: dict[str, Any],
        canonical_totals: dict[str, Any],
        simulators: dict[str, Any],
    ) -> dict[str, Any]:
        canonical_equity = _safe_float(canonical_balances.get("total_equity"), 0.0)
        canonical_positions = int(canonical_totals.get("positions") or 0)
        checks: list[dict[str, Any]] = []
        max_equity_gap = 0.0
        max_positions_gap = 0
        comparators = (
            ("atlas_internal", True, True),
            ("paper_local", True, True),
            ("optionstrat", False, True),
        )
        for key, compare_equity, compare_positions in comparators:
            item = simulators.get(key) or {}
            observed_equity = item.get("equity")
            observed_positions = item.get("open_positions")
            if compare_equity and observed_equity is not None:
                equity_gap = round(float(observed_equity) - canonical_equity, 4)
                max_equity_gap = max(max_equity_gap, abs(equity_gap))
                checks.append(
                    {
                        "comparison": key,
                        "metric": "equity_usd",
                        "canonical": _round_or_none(canonical_equity),
                        "observed": _round_or_none(float(observed_equity)),
                        "gap": equity_gap,
                        "gap_pct": round((equity_gap / canonical_equity * 100.0), 4) if canonical_equity else None,
                    }
                )
            if compare_positions and observed_positions is not None:
                positions_gap = int(observed_positions) - canonical_positions
                max_positions_gap = max(max_positions_gap, abs(positions_gap))
                checks.append(
                    {
                        "comparison": key,
                        "metric": "open_positions",
                        "canonical": canonical_positions,
                        "observed": int(observed_positions),
                        "gap": positions_gap,
                        "gap_pct": None,
                    }
                )

        if max_equity_gap > 250 or max_positions_gap > 0:
            state = "failed"
        elif max_equity_gap > 25:
            state = "degraded"
        else:
            state = "healthy"

        return {
            "state": state,
            "checked_at": datetime.now(timezone.utc).isoformat(),
            "max_equity_gap_usd": round(max_equity_gap, 4),
            "max_positions_gap": max_positions_gap,
            "items": checks,
        }
