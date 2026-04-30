"""
Métricas agregadas para dashboard OptionStrat (stdlib solamente).

``compute_aggregates()`` refresca MTM vía ``mark_all()`` salvo ``skip_mark_all=True``
(cuando el llamador acaba de marcar, p. ej. ``OptionStratBridge``).
"""
from __future__ import annotations

from collections import defaultdict
from datetime import date, datetime, timezone
from typing import Any

from .options_client import AtlasOptionsService
from .options_reporter import _estimate_notional


def _parse_iso_datetime(val: str | None) -> datetime | None:
    if not val or not isinstance(val, str):
        return None
    s = val.strip()
    if not s:
        return None
    try:
        if s.endswith("Z"):
            s = s[:-1] + "+00:00"
        return datetime.fromisoformat(s)
    except ValueError:
        return None


def _unrealized_from_detail(d: dict[str, Any]) -> float:
    if not d.get("is_open"):
        return 0.0
    snap = d.get("last_snapshot")
    if isinstance(snap, dict) and snap.get("unrealized_pnl") is not None:
        return float(snap["unrealized_pnl"])
    return 0.0


class OptionsMetrics:
    """Agrupa PnL por día/hora, distribución de estrategias y exposición por símbolo (solo abiertas)."""

    def __init__(self, options_service: AtlasOptionsService) -> None:
        self._svc = options_service

    def compute_aggregates(self, *, skip_mark_all: bool = False) -> dict[str, Any]:
        if not skip_mark_all:
            self._svc.mark_all()

        get_quote = self._svc.client.provider.get_quote
        open_list = self._svc.list_open_positions()
        closed_list = self._svc.list_closed_positions()

        today_utc = datetime.now(timezone.utc).date()

        # --- daily_pnl ---
        realized_by_date: dict[date, float] = defaultdict(float)
        for pos in closed_list:
            d = self._svc.get_position(pos.position_id)
            closed_at = _parse_iso_datetime(d.get("closed_at") if isinstance(d.get("closed_at"), str) else None)
            if closed_at is None:
                continue
            rp = d.get("realized_pnl")
            if rp is None:
                continue
            realized_by_date[closed_at.date()] += float(rp)

        unrealized_today = 0.0
        for pos in open_list:
            d = self._svc.get_position(pos.position_id)
            unrealized_today += _unrealized_from_detail(d)

        dates_for_daily = set(realized_by_date.keys())
        if open_list:
            dates_for_daily.add(today_utc)
        if today_utc in realized_by_date or unrealized_today != 0:
            dates_for_daily.add(today_utc)

        daily_pnl: list[dict[str, Any]] = []
        for dday in sorted(dates_for_daily):
            r = round(realized_by_date.get(dday, 0.0), 2)
            u = round(unrealized_today, 2) if dday == today_utc and open_list else 0.0
            daily_pnl.append(
                {
                    "date": dday.isoformat(),
                    "realized_pnl": r,
                    "unrealized_pnl": u,
                }
            )

        # --- strategy_distribution & symbol_exposure (open only) ---
        strat_notional: dict[str, float] = defaultdict(float)
        strat_count: dict[str, int] = defaultdict(int)
        sym_notional: dict[str, float] = defaultdict(float)

        for pos in open_list:
            d = self._svc.get_position(pos.position_id)
            st = str(d.get("strategy_type", "") or "") or "(unknown)"
            sym = str(d.get("symbol", "") or "") or "(unknown)"
            notion = float(_estimate_notional(pos, str(d.get("strategy_type", "") or ""), get_quote))
            strat_notional[st] += notion
            strat_count[st] += 1
            sym_notional[sym] += notion

        total_strat_n = sum(strat_notional.values()) or 0.0
        strategy_distribution: list[dict[str, Any]] = []
        for st in sorted(strat_notional.keys()):
            n = strat_notional[st]
            w = round(n / total_strat_n, 6) if total_strat_n > 0 else 0.0
            strategy_distribution.append(
                {
                    "strategy_type": st,
                    "open_count": strat_count[st],
                    "weight_notional": w,
                }
            )

        total_sym_n = sum(sym_notional.values()) or 0.0
        symbol_exposure: list[dict[str, Any]] = []
        for sym in sorted(sym_notional.keys()):
            n = round(sym_notional[sym], 2)
            w = round(sym_notional[sym] / total_sym_n, 6) if total_sym_n > 0 else 0.0
            symbol_exposure.append(
                {
                    "symbol": sym,
                    "notional_open": n,
                    "weight": w,
                }
            )

        # --- pnl_by_day_hour (heatmap día × hora, UTC) ---
        dh_realized: dict[tuple[str, int], float] = defaultdict(float)
        dh_unrealized: dict[tuple[str, int], float] = defaultdict(float)

        for pos in closed_list:
            d = self._svc.get_position(pos.position_id)
            dt = _parse_iso_datetime(d.get("closed_at") if isinstance(d.get("closed_at"), str) else None)
            rp = d.get("realized_pnl")
            if dt is None or rp is None:
                continue
            key = (dt.date().isoformat(), dt.hour)
            dh_realized[key] += float(rp)

        for pos in open_list:
            d = self._svc.get_position(pos.position_id)
            u = _unrealized_from_detail(d)
            if u == 0.0:
                continue
            snap = d.get("last_snapshot")
            ts = None
            if isinstance(snap, dict) and snap.get("timestamp"):
                ts = _parse_iso_datetime(str(snap["timestamp"]))
            if ts is None:
                ts = _parse_iso_datetime(d.get("opened_at") if isinstance(d.get("opened_at"), str) else None)
            if ts is None:
                ts = datetime.now(timezone.utc)
            key = (ts.date().isoformat(), ts.hour)
            dh_unrealized[key] += u

        all_keys = set(dh_realized.keys()) | set(dh_unrealized.keys())
        pnl_by_day_hour: list[dict[str, Any]] = []
        for day_s, hour in sorted(all_keys, key=lambda x: (x[0], x[1])):
            pnl_by_day_hour.append(
                {
                    "date": day_s,
                    "hour": hour,
                    "realized_pnl": round(dh_realized.get((day_s, hour), 0.0), 2),
                    "unrealized_pnl": round(dh_unrealized.get((day_s, hour), 0.0), 2),
                }
            )

        return {
            "daily_pnl": daily_pnl,
            "strategy_distribution": strategy_distribution,
            "symbol_exposure": symbol_exposure,
            "pnl_by_day_hour": pnl_by_day_hour,
        }
