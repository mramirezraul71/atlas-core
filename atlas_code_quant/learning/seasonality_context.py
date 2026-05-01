"""Seasonality context helpers for paper_aggressive fusion."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any


def _safe_now(snapshot: dict[str, Any] | None = None) -> datetime:
    if isinstance(snapshot, dict):
        raw_dt = snapshot.get("as_of_utc")
        if isinstance(raw_dt, str) and raw_dt.strip():
            try:
                parsed = datetime.fromisoformat(raw_dt.replace("Z", "+00:00"))
                if parsed.tzinfo is None:
                    return parsed.replace(tzinfo=timezone.utc)
                return parsed.astimezone(timezone.utc)
            except ValueError:
                pass
    return datetime.now(timezone.utc)


def _session_for_hour(hour_utc: int) -> tuple[str, str]:
    if 13 <= hour_utc < 14:
        return ("US", "open")
    if 14 <= hour_utc < 19:
        return ("US", "mid")
    if 19 <= hour_utc < 21:
        return ("US", "close")
    if 11 <= hour_utc < 13:
        return ("premarket", "premarket")
    if 21 <= hour_utc < 23:
        return ("afterhours", "afterhours")
    return ("other", "off")


def _is_month_end(dt: datetime) -> bool:
    return dt.day >= 27


def _is_quarter_end(dt: datetime) -> bool:
    return dt.month in {3, 6, 9, 12} and dt.day >= 25


def build_seasonality_context(snapshot: dict[str, Any] | None = None) -> dict[str, Any]:
    """Build a stable seasonality payload from UTC clock + light hooks."""
    now = _safe_now(snapshot)
    session, hour_bucket = _session_for_hour(now.hour)
    is_month_end = _is_month_end(now)
    is_quarter_end = _is_quarter_end(now)

    # Phase B heuristic bias. Keep explicit and easy to adjust.
    if is_quarter_end and hour_bucket in {"open", "close"}:
        bias = "hostile"
        reason = "quarter_end_volatility_window"
    elif is_month_end and hour_bucket == "open":
        bias = "hostile"
        reason = "month_end_open_rebalance"
    elif session == "US" and hour_bucket == "mid":
        bias = "favorable"
        reason = "regular_session_mid_stability"
    elif session in {"premarket", "afterhours", "other"}:
        bias = "hostile"
        reason = "thin_liquidity_session"
    else:
        bias = "neutral"
        reason = "no_strong_seasonal_edge"

    return {
        "session": session,
        "day_of_week": now.strftime("%a").lower(),
        "month": now.month,
        "is_month_end": is_month_end,
        "is_quarter_end": is_quarter_end,
        "hour_bucket": hour_bucket,
        "seasonal_bias": bias,
        "seasonal_reason": reason,
    }
