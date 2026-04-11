"""Tests del módulo de briefing operativo: priorización, render, storage, dedup."""
from __future__ import annotations

import json
from pathlib import Path
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from notifications.dispatcher import OperationalNotificationDispatcher
from notifications.prioritization import (
    adaptive_learning_headline,
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
