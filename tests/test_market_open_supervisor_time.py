from __future__ import annotations

from datetime import datetime, timezone


def test_market_open_window_true_inside_range() -> None:
    from atlas_code_quant.scripts.market_open_supervisor import is_market_open_window

    # 2026-04-16 is a Thursday. 09:30 ET == 13:30 UTC (during EDT).
    now_utc = datetime(2026, 4, 16, 13, 25, tzinfo=timezone.utc)  # 09:25 ET
    assert is_market_open_window(now_utc, pre_minutes=15, post_minutes=20) is True


def test_market_open_window_false_outside_range() -> None:
    from atlas_code_quant.scripts.market_open_supervisor import is_market_open_window

    now_utc = datetime(2026, 4, 16, 12, 0, tzinfo=timezone.utc)  # 08:00 ET
    assert is_market_open_window(now_utc, pre_minutes=15, post_minutes=20) is False

