from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[2]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from scanner.opportunity_scanner import OpportunityScannerService, _method_order_for_regime  # noqa: E402


def test_method_order_trending_strong_prefers_breakout_first(monkeypatch) -> None:
    monkeypatch.setattr("scanner.opportunity_scanner.settings.scanner_regime_adapt_enabled", True)
    order = _method_order_for_regime("trending_strong")
    assert order[0] == "breakout_donchian"


def test_method_order_consolidating_prefers_pullback_first(monkeypatch) -> None:
    monkeypatch.setattr("scanner.opportunity_scanner.settings.scanner_regime_adapt_enabled", True)
    order = _method_order_for_regime("consolidating")
    assert order[0] == "rsi_pullback_trend"


def test_vix_panic_short_circuits_cycle(monkeypatch) -> None:
    svc = OpportunityScannerService()
    monkeypatch.setattr("scanner.opportunity_scanner.settings.scanner_vix_enabled", True)
    monkeypatch.setattr("scanner.opportunity_scanner.settings.scanner_vix_panic_skip_cycle", True)
    monkeypatch.setattr(
        "scanner.opportunity_scanner.build_vix_context",
        lambda: {"gate": "panic", "multiplier": 1.0, "current": 40.0, "ok": True},
    )
    monkeypatch.setattr(svc, "_resolve_scan_batch", lambda: (["SPY"], {"universe_total": 1, "mode": "manual"}))
    monkeypatch.setattr(svc, "_prefilter_batch", lambda syms: (syms, {"prefilter_selected": 1}))
    rep = svc._run_cycle()
    assert rep.get("summary", {}).get("vix_panic") is True or any(
        "vix" in str(x).lower() for x in (rep.get("rejections") or [])
    )
