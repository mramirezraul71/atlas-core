"""
Vista agregada del libro de opciones paper para dashboards (Grafana / JSON).

Sin servidor HTTP: ``snapshot_state()`` devuelve un dict; ``options_state_to_json_file``
persiste para que otro proceso lo sirva o scrapee.
"""
from __future__ import annotations

import json
from collections import defaultdict
from datetime import datetime, timezone
from typing import Any, Callable

from atlas_options_brain.models.option_contract import OptionType, OptionRight
from atlas_options_brain.simulator.paper import Position

from .options_client import AtlasOptionsService


def _estimate_notional(
    position: Position,
    strategy_type: str,
    get_quote: Callable[[str], float],
) -> float:
    """
    Alineado conceptualmente con el riesgo de apertura en ``AtlasOptionsService``:
    crédito → ancho × 100 × qty; débito → abs(net_premium); covered → spot × 100 × qty.
    """
    legs = position.entry_legs
    if not legs:
        return 0.0
    qty = max((lg.qty for lg in legs), default=1)
    sym = legs[0].contract.symbol

    if strategy_type == "iron_condor":
        puts = [lg for lg in legs if lg.contract.option_type == OptionType.PUT]
        calls = [lg for lg in legs if lg.contract.option_type == OptionType.CALL]
        if len(puts) < 2 or len(calls) < 2:
            return 0.0
        pst = sorted(lg.contract.strike for lg in puts)
        w_put = pst[1] - pst[0]
        cst = sorted(lg.contract.strike for lg in calls)
        w_call = cst[1] - cst[0]
        return float(max(w_put, w_call) * 100.0 * qty)

    if strategy_type == "bull_put":
        pst = [lg.contract.strike for lg in legs if lg.contract.option_type == OptionType.PUT]
        if len(pst) < 2:
            return 0.0
        return float((max(pst) - min(pst)) * 100.0 * qty)

    if strategy_type == "bear_call":
        cst = [lg.contract.strike for lg in legs if lg.contract.option_type == OptionType.CALL]
        if len(cst) < 2:
            return 0.0
        return float((max(cst) - min(cst)) * 100.0 * qty)

    if strategy_type in ("bull_call", "bear_put"):
        return float(abs(position.entry_net_premium))

    if strategy_type == "covered_call":
        short = next(
            (lg for lg in legs if lg.right == OptionRight.SHORT),
            None,
        )
        if short is None:
            return 0.0
        return float(get_quote(sym) * 100.0 * short.qty)

    return 0.0


def _unrealized_from_detail(d: dict[str, Any]) -> float | None:
    if not d.get("is_open"):
        return None
    snap = d.get("last_snapshot")
    if isinstance(snap, dict) and "unrealized_pnl" in snap:
        return float(snap["unrealized_pnl"])
    return None


class OptionsReporter:
    """Compila métricas de cartera / símbolo / estrategia a partir de ``AtlasOptionsService``."""

    def __init__(self, options_service: AtlasOptionsService) -> None:
        self._svc = options_service

    def snapshot_state(self) -> dict[str, Any]:
        self._svc.mark_all()
        get_quote = self._svc.client.provider.get_quote

        open_list = self._svc.list_open_positions()
        closed_list = self._svc.list_closed_positions()
        open_ids = {p.position_id for p in open_list}

        by_symbol: dict[str, dict[str, Any]] = defaultdict(
            lambda: {
                "symbol": "",
                "open_positions": 0,
                "notional_open": 0.0,
                "net_premium": 0.0,
                "unrealized_pnl": 0.0,
            }
        )
        by_strategy: dict[str, dict[str, Any]] = defaultdict(
            lambda: {
                "strategy_type": "",
                "count_open": 0,
                "notional_open": 0.0,
                "net_premium": 0.0,
                "unrealized_pnl": 0.0,
            }
        )

        positions_out: list[dict[str, Any]] = []
        total_notional = 0.0
        total_net_open = 0.0
        total_u = 0.0

        for pos in list(open_list) + list(closed_list):
            pid = pos.position_id
            detail = self._svc.get_position(pid)
            st = str(detail.get("strategy_type", "") or "")
            sym = str(detail.get("symbol", "") or "")
            notional = _estimate_notional(pos, st, get_quote)

            u = _unrealized_from_detail(detail)
            is_open = bool(detail.get("is_open"))

            simplified = {
                "position_id": detail["position_id"],
                "symbol": sym,
                "strategy_type": st,
                "is_open": is_open,
                "entry_net_premium": detail["entry_net_premium"],
                "notional_estimate": round(notional, 2) if is_open else 0.0,
                "unrealized_pnl": u,
                "realized_pnl": detail.get("realized_pnl"),
                "opened_at": detail.get("opened_at"),
                "closed_at": detail.get("closed_at"),
            }
            positions_out.append(simplified)

            if is_open:
                total_notional += notional
                total_net_open += float(detail["entry_net_premium"])
                if u is not None:
                    total_u += u

                by_symbol[sym]["symbol"] = sym
                by_symbol[sym]["open_positions"] += 1
                by_symbol[sym]["notional_open"] += notional
                by_symbol[sym]["net_premium"] += float(detail["entry_net_premium"])
                if u is not None:
                    by_symbol[sym]["unrealized_pnl"] += u

                by_strategy[st]["strategy_type"] = st
                by_strategy[st]["count_open"] += 1
                by_strategy[st]["notional_open"] += notional
                by_strategy[st]["net_premium"] += float(detail["entry_net_premium"])
                if u is not None:
                    by_strategy[st]["unrealized_pnl"] += u

        def _round_agg(d: dict[str, Any]) -> dict[str, Any]:
            out = dict(d)
            for k in ("notional_open", "net_premium", "unrealized_pnl"):
                if k in out and isinstance(out[k], float):
                    out[k] = round(out[k], 2)
            return out

        by_symbol_list = [
            _round_agg(v) for v in sorted(by_symbol.values(), key=lambda x: x["symbol"])
        ]
        by_strategy_list = [
            _round_agg(v)
            for v in sorted(by_strategy.values(), key=lambda x: x["strategy_type"])
        ]

        return {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "portfolio": {
                "total_open_positions": len(open_ids),
                "total_notional_open": round(total_notional, 2),
                "total_net_premium": round(total_net_open, 2),
                "total_unrealized_pnl": round(total_u, 2),
            },
            "by_symbol": by_symbol_list,
            "by_strategy_type": by_strategy_list,
            "positions": positions_out,
        }


def options_state_to_json_file(path: str, reporter: OptionsReporter) -> None:
    """Serializa ``snapshot_state()`` en JSON UTF-8 con indentación legible."""
    data = reporter.snapshot_state()
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
