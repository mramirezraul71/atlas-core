"""Tests del módulo de briefing operativo: priorización, render, storage, dedup."""
from __future__ import annotations

import json
from pathlib import Path
import asyncio
from datetime import datetime, timezone
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from notifications.briefing_service import OperationalBriefingService
from notifications.dispatcher import OperationalNotificationDispatcher
from notifications.prioritization import (
    adaptive_learning_headline,
    build_prioritized_eod,
    build_prioritized_premarket,
    classify_learning_phase,
    rank_scanner_opportunities,
)
from notifications.renderers import render_premarket
from notifications.storage import NotificationAuditStore
from notifications.models import BriefingKind, PrioritizedBriefing


def test_classify_learning_phase_stalled_when_not_running() -> None:
    phase, _ = classify_learning_phase({"running": False, "recent_errors": []})
    from notifications.models import LearningPhase

    assert phase == LearningPhase.STALLED


def test_rank_scanner_opportunities_orders_by_score() -> None:
    rep = {
        "candidates": [
            {"symbol": "AAA", "selection_score": 70.0, "timeframe": "1h"},
            {"symbol": "ZZZ", "selection_score": 90.0, "timeframe": "5m"},
        ]
    }
    out = rank_scanner_opportunities(rep, max_n=1)
    assert len(out) == 1
    assert out[0]["symbol"] == "ZZZ"


def test_adaptive_learning_headline_empty() -> None:
    h = adaptive_learning_headline(None)
    assert h["reliable"] is False


def test_render_premarket_contains_sections() -> None:
    pb = PrioritizedBriefing(
        kind=BriefingKind.PREMARKET,
        headline="H",
        opportunities=[{"symbol": "SPY", "selection_score": 88, "timeframe": "1h", "method": "trend"}],
        risks=["r1"],
        positions_focus=[{"symbol": "X", "unrealized_pnl": -10}],
        learning_summary={"phase": "ready", "policy_can_weight": True, "orchestrator": {}, "adaptive": {}},
        session_plan=["s1"],
        open_close_criteria={"open": ["a"], "close": ["b"]},
        metrics={"readiness_ready": True, "scanner_running": True, "open_positions": 1},
    )
    html, plain = render_premarket(pb)
    assert "SPY" in html and "SPY" in plain
    assert "r1" in plain


def test_notification_audit_store_roundtrip(tmp_path: Path) -> None:
    store = NotificationAuditStore(tmp_path)
    p = store.save_snapshot("premarket", {"kind": "premarket", "generated_at": "t0"})
    assert p.exists()
    store.append_event({"ts": "t1", "kind": "premarket"})
    recent = store.list_recent_snapshots(limit=5)
    assert len(recent) >= 1
    ev = store.tail_events(limit=5)
    assert len(ev) >= 1


def test_dispatcher_dedup_skips_second_send() -> None:
    st = MagicMock()
    st.notify_dedup_ttl_sec = 3600.0
    st.notify_cooldown_sec = 1.0
    st.notify_channels = ["telegram"]
    st.notify_min_severity = "INFO"
    d = OperationalNotificationDispatcher(st)

    async def _run() -> None:
        with patch("notifications.dispatcher.get_alert_dispatcher") as ga:
            disp = MagicMock()
            disp.operational_briefing = AsyncMock()
            ga.return_value = disp
            r1 = await d.send_briefing(
                kind="premarket",
                title="T",
                body_html="<b>x</b>",
                body_plain="x",
                category="briefing_premarket",
            )
            r2 = await d.send_briefing(
                kind="premarket",
                title="T",
                body_html="<b>x</b>",
                body_plain="x",
                category="briefing_premarket",
            )
            assert r1.detail != "deduplicated"
            assert r2.detail == "deduplicated"
            disp.operational_briefing.assert_awaited_once()

    asyncio.run(_run())


def test_build_prioritized_premarket_minimal_ctx() -> None:
    st = MagicMock()
    st.notify_max_opportunities = 3
    st.notify_max_positions = 4
    st.exit_governance_take_profit_r = 0.75
    ctx = {
        "readiness": {"ready": True, "reasons_not_ready": []},
        "operation_lite": {"failsafe": {"active": False}},
        "scanner": {
            "candidates": [{"symbol": "QQQ", "selection_score": 80.0, "timeframe": "15m"}],
            "status": {"running": True, "last_error": None},
        },
        "canonical_snapshot": {"positions": [], "strategies": []},
        "learning_orchestrator": {"running": True, "last_reconcile_at": "2099-01-01T00:00:00+00:00", "trades_processed_total": 50, "recent_errors": []},
        "adaptive_snapshot": {"summary": {"headline": "ok"}},
    }
    pb = build_prioritized_premarket(ctx, st)
    assert pb.opportunities
    assert pb.kind == BriefingKind.PREMARKET


def test_build_prioritized_eod_uses_market_timezone_for_day_match(monkeypatch: pytest.MonkeyPatch) -> None:
    import notifications.prioritization as pr

    class FixedDateTime(datetime):
        @classmethod
        def now(cls, tz=None):
            base = datetime(2026, 4, 11, 1, 30, tzinfo=timezone.utc)
            return base if tz is None else base.astimezone(tz)

    monkeypatch.setattr(pr, "datetime", FixedDateTime)
    st = MagicMock()
    st.notify_tz = "America/New_York"
    st.notify_max_positions = 3
    ctx = {
        "journal_analytics": {
            "daily_pnl": [
                {"date": "2026-04-10", "pnl": 125.5, "trades": 3},
                {"date": "2026-04-11", "pnl": 999.0, "trades": 1},
            ],
            "strategy_performance": [],
        },
        "canonical_snapshot": {"positions": []},
        "learning_orchestrator": {},
        "adaptive_snapshot": {},
    }
    pb = build_prioritized_eod(ctx, st)
    assert pb.metrics["pnl_day_estimate"] == 125.5
    assert pb.metrics["pnl_day_session_date"] == "2026-04-10"
    assert pb.metrics["pnl_day_match_source"] == "session_day_exact"


def test_build_prioritized_eod_flags_fallback_without_exact_session_match(monkeypatch: pytest.MonkeyPatch) -> None:
    import notifications.prioritization as pr

    class FixedDateTime(datetime):
        @classmethod
        def now(cls, tz=None):
            base = datetime(2026, 4, 11, 20, 0, tzinfo=timezone.utc)
            return base if tz is None else base.astimezone(tz)

    monkeypatch.setattr(pr, "datetime", FixedDateTime)
    st = MagicMock()
    st.notify_tz = "America/New_York"
    st.notify_max_positions = 2
    ctx = {
        "journal_analytics": {"daily_pnl": [{"date": "2026-04-10", "pnl": -42.0}]},
        "canonical_snapshot": {"positions": []},
        "learning_orchestrator": {},
        "adaptive_snapshot": {},
    }
    pb = build_prioritized_eod(ctx, st)
    assert pb.metrics["pnl_day_match_source"] == "latest_row_fallback"
    assert any("sin match exacto de sesion" in risk for risk in pb.risks)


def test_operational_briefing_status_includes_dispatcher_runtime(tmp_path: Path) -> None:
    st = MagicMock()
    st.notify_enabled = True
    st.notify_premarket = True
    st.notify_eod = True
    st.notify_intraday = True
    st.notify_exit_intelligence = True
    st.notify_channels = ["telegram", "whatsapp"]
    st.notifications_data_dir = tmp_path

    with patch("notifications.briefing_service.get_alert_dispatcher") as ga:
        ga.return_value.status.return_value = {
            "telegram_enabled": True,
            "whatsapp_enabled": False,
            "worker_alive": True,
        }
        svc = OperationalBriefingService({"settings": st})
        out = svc.status()

    assert out["dispatcher_status"]["telegram_enabled"] is True
    assert out["dispatcher_status"]["whatsapp_enabled"] is False


def test_attach_exit_intelligence_bridge_triggers_only_for_trade_exit(monkeypatch: pytest.MonkeyPatch) -> None:
    import notifications.briefing_service as bs
    import notifications.scheduler as sched
    import operations.alert_dispatcher as ad
    from config.settings import settings

    class DummyDispatcher:
        def __init__(self) -> None:
            self._on_alert = None

    class ImmediateLoop:
        def create_task(self, coro):
            asyncio.run(coro)
            return None

    dispatcher = DummyDispatcher()
    svc = MagicMock()
    svc.emit_exit_intelligence = AsyncMock()

    monkeypatch.setattr(ad, "get_alert_dispatcher", lambda *args, **kwargs: dispatcher)
    monkeypatch.setattr(bs, "get_operational_briefing_service", lambda: svc)
    monkeypatch.setattr(settings, "notify_enabled", True)
    monkeypatch.setattr(settings, "notify_exit_intelligence", True)
    monkeypatch.setattr(asyncio, "get_running_loop", lambda: ImmediateLoop())

    sched.attach_exit_intelligence_bridge()
    dispatcher._on_alert(ad.AlertEvent(level="INFO", category="trade", title="ENTRADA LONG", body="body", symbol="SPY"))
    svc.emit_exit_intelligence.assert_not_awaited()

    dispatcher._on_alert(
        ad.AlertEvent(level="INFO", category="trade", title="SALIDA", body="body", symbol="SPY", metadata={"x": 1})
    )
    svc.emit_exit_intelligence.assert_awaited_once()
