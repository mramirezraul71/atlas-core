"""
Operador de autonomía del radar: saneo periódico de riesgo, liberación de
slots (max_open) en paper y supervisor LLM con acciones en lista blanca.

No sustituye los breakers por drawdown/kill; reduce fricción operativa cuando
el estado queda incoherente (huérfanos en ``open_positions``, safe_mode
pegado sin breaker) y documenta el pulso en ``state.autonomy_status``.
"""

from __future__ import annotations

import asyncio
import logging
import os
from typing import Any

_LOG = logging.getLogger(__name__)

_DISABLE = os.getenv("RADAR_AUTONOMY_DISABLE", "").strip().lower() in (
    "1",
    "true",
    "yes",
)
_INTERVAL = float(os.getenv("RADAR_AUTONOMY_INTERVAL_S", "90"))
_LIVE_OK = os.getenv("RADAR_AUTONOMY_LIVE_OK", "").strip().lower() in (
    "1",
    "true",
    "yes",
)
_LLM = os.getenv("RADAR_AUTONOMY_LLM", "1").strip().lower() not in (
    "0",
    "false",
    "no",
)


def autonomy_interval_s() -> float:
    return max(15.0, _INTERVAL)


def _may_autonomy_exit(orch: Any) -> bool:
    ex = getattr(orch, "executor", None)
    cfg = getattr(ex, "cfg", None) if ex else None
    live = bool(getattr(cfg, "enable_live", False))
    if not live:
        return True
    return _LIVE_OK


async def _submit_autonomy_exit(orch: Any, ticker: str, cur_px: int) -> bool:
    from .executor_v2 import OrderRequestV2

    pos = orch.exit_mgr.positions.get(ticker)
    if not pos or pos.closed:
        return False
    sig = orch.exit_mgr.evaluate(
        ticker=ticker,
        current_price=cur_px,
        current_edge=0.0,
        data_degraded=False,
        forced=False,
        autonomy_capacity=True,
    )
    if not sig.should_exit:
        return False
    exit_side = "NO" if pos.side == "YES" else "YES"
    exit_price = max(1, min(99, int(sig.target_price or cur_px)))
    coid = orch.executor.make_client_order_id(
        ticker, exit_side, exit_price, pos.size, ts_bucket_ms=500
    )
    order = OrderRequestV2(
        market_ticker=ticker,
        side=exit_side,
        contracts=pos.size,
        price_cents=exit_price,
        client_order_id=coid,
        reason=sig.reason,
    )
    report = await orch.executor.submit(order)
    pnl_per = (
        (exit_price - pos.entry_price)
        if pos.side == "YES"
        else (pos.entry_price - exit_price)
    )
    pnl = int(pnl_per * pos.size - 0.07 * pos.size)
    orch.risk.on_close(ticker, pos.size * pos.entry_price, pnl)
    orch.exit_mgr.close(ticker)
    orch.journal.write(
        "exits",
        {
            "ticker": ticker,
            "reason": sig.reason,
            "entry": pos.entry_price,
            "exit": exit_price,
            "size": pos.size,
            "pnl_cents": pnl,
            "fees_cents": int(0.07 * pos.size),
            "slippage_cents": report.slippage_cents,
            "via": "autonomy_operator",
        },
    )
    return bool(report.ok)


async def run_autonomy_tick(state: Any) -> None:
    st = getattr(state, "autonomy_status", None)
    if st is None:
        return
    if _DISABLE:
        st["enabled"] = False
        return
    st["enabled"] = True

    orch = getattr(state, "orchestrator", None)
    if orch is None or not getattr(state, "runtime_enabled", False):
        return

    import time as _time

    actions: list[str] = []
    err = ""
    llm_action: dict[str, str] | None = None
    now = _time.time()

    try:
        risk = orch.risk.state
        exit_mgr = orch.exit_mgr

        for t in list((risk.open_positions or {}).keys()):
            sz = int(risk.open_positions.get(t) or 0)
            if sz > 0 and t not in exit_mgr.positions:
                risk.open_positions.pop(t, None)
                actions.append(f"heal_orphan_risk:{t}")

        before_sm = risk.safe_mode
        orch.risk._maybe_clear_safe_mode()
        if before_sm and not risk.safe_mode:
            actions.append("cleared_safe_mode")

        open_n = len(exit_mgr.list_open())
        max_o = int(orch.risk.limits.max_open_positions)
        breaker = orch.risk._check_breakers()
        need_shed = open_n >= max_o or breaker == "max_open"
        shed_done = False
        if need_shed and open_n > 0 and _may_autonomy_exit(orch):
            oldest = sorted(exit_mgr.list_open(), key=lambda p: p.entry_ts)[0]
            cur_px = max(1, min(99, int(oldest.entry_price)))
            ok = await _submit_autonomy_exit(orch, oldest.ticker, cur_px)
            shed_done = bool(ok)
            actions.append(f"shed_oldest:{oldest.ticker}:{'ok' if ok else 'fail'}")
            orch.risk._maybe_clear_safe_mode()

        open_n = len(exit_mgr.list_open())
        breaker = orch.risk._check_breakers()

        snap = {
            "safe_mode": risk.safe_mode,
            "safe_mode_reason": risk.safe_mode_reason,
            "breaker": breaker,
            "open_positions": open_n,
            "max_open": max_o,
            "kill_switch": risk.kill_switch,
            "balance_cents": risk.balance_cents,
            "exposure_cents": risk.total_exposure_cents,
            "actions_so_far": actions,
        }

        if _LLM:
            try:
                llm_action = await orch.brain.supervisor_advise(snap)
                st["last_llm_action"] = llm_action
                act = (llm_action or {}).get("action", "noop")
                summ = (llm_action or {}).get("summary", "")
                if act == "escalate":
                    from .dashboard.router import _record_radar_calibration_event

                    _record_radar_calibration_event(
                        state,
                        "radar_autonomy",
                        f"LLM escalate: {summ}",
                        {"snapshot": snap, "llm": llm_action},
                        ans_ok=False,
                    )
                    actions.append("llm_escalate_logged")
                elif (
                    act == "shed_oldest"
                    and not shed_done
                    and (open_n >= max_o or breaker == "max_open")
                    and _may_autonomy_exit(orch)
                    and len(exit_mgr.list_open()) > 0
                ):
                    oldest = sorted(
                        exit_mgr.list_open(), key=lambda p: p.entry_ts
                    )[0]
                    cur_px = max(1, min(99, int(oldest.entry_price)))
                    ok2 = await _submit_autonomy_exit(orch, oldest.ticker, cur_px)
                    actions.append(
                        f"llm_shed:{oldest.ticker}:{'ok' if ok2 else 'fail'}"
                    )
                    orch.risk._maybe_clear_safe_mode()
                elif act == "log" and summ:
                    actions.append("llm_log")
            except Exception as exc:
                llm_action = {"action": "noop", "summary": str(exc)}
                st["last_llm_action"] = llm_action
                _LOG.debug("supervisor LLM error %s", exc)

        st["last_tick_ts"] = now
        st["last_actions"] = actions
        st["last_error"] = ""

    except Exception as exc:
        err = str(exc)
        _LOG.warning("autonomy tick failed: %s", exc, exc_info=True)
        st["last_error"] = err
        st["last_tick_ts"] = now
        st["last_actions"] = actions


async def autonomy_background_loop(state: Any) -> None:
    while True:
        try:
            await asyncio.sleep(autonomy_interval_s())
            await run_autonomy_tick(state)
        except asyncio.CancelledError:
            raise
        except Exception:
            _LOG.exception("autonomy loop error")
