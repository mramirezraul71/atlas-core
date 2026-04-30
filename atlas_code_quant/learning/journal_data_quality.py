from __future__ import annotations

from collections import Counter, defaultdict
from datetime import datetime
import math
from typing import Any, Iterable

from config.settings import settings


def _safe_float(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(out) or math.isinf(out):
        return None
    return out


def _naive_datetime(value: Any) -> datetime | None:
    if not isinstance(value, datetime):
        return None
    if value.tzinfo is not None:
        return value.astimezone().replace(tzinfo=None)
    return value


def _row_day(row: Any) -> str:
    for candidate in (getattr(row, "exit_time", None), getattr(row, "updated_at", None), getattr(row, "entry_time", None)):
        if isinstance(candidate, datetime):
            return candidate.date().isoformat()
    return ""


def _row_duration_seconds(row: Any) -> float | None:
    entry_time = _naive_datetime(getattr(row, "entry_time", None))
    exit_time = _naive_datetime(getattr(row, "exit_time", None))
    if entry_time is None or exit_time is None:
        return None
    return (exit_time - entry_time).total_seconds()


def row_quality_flags(row: Any) -> list[str]:
    flags: list[str] = []
    entry_price = _safe_float(getattr(row, "entry_price", None))
    exit_price = _safe_float(getattr(row, "exit_price", None))
    realized_pnl = _safe_float(getattr(row, "realized_pnl", None))
    duration_seconds = _row_duration_seconds(row)

    if entry_price is not None and entry_price < 0:
        flags.append("negative_entry_price")
    if exit_price is not None and exit_price < 0:
        flags.append("negative_exit_price")
    if realized_pnl is None:
        flags.append("invalid_realized_pnl")
    if duration_seconds is not None and duration_seconds <= float(settings.journal_quality_min_duration_sec):
        flags.append("too_short_duration")
    if duration_seconds is not None and duration_seconds < 0:
        flags.append("negative_duration")
    return flags


def _ratio(count: int, total: int) -> float:
    if total <= 0:
        return 0.0
    return count / total


def _strategy_anomalies(rows: list[Any]) -> list[dict[str, Any]]:
    grouped: dict[str, list[Any]] = defaultdict(list)
    min_samples = max(25, int(settings.journal_quality_strategy_anomaly_min_samples))
    for row in rows:
        strategy_type = str(getattr(row, "strategy_type", "") or "").strip() or "unknown"
        grouped[strategy_type].append(row)

    anomalies: list[dict[str, Any]] = []
    for strategy_type, strategy_rows in grouped.items():
        if len(strategy_rows) < min_samples:
            continue
        wins = sum(1 for row in strategy_rows if (_safe_float(getattr(row, "realized_pnl", None)) or 0.0) > 0)
        win_rate_pct = (wins / len(strategy_rows)) * 100.0 if strategy_rows else 0.0
        if win_rate_pct <= 0.1 or win_rate_pct >= 99.9:
            anomalies.append(
                {
                    "strategy_type": strategy_type,
                    "sample_count": len(strategy_rows),
                    "win_rate_pct": round(win_rate_pct, 2),
                }
            )
    return sorted(anomalies, key=lambda item: int(item["sample_count"]), reverse=True)


def build_journal_quality_scorecard(rows: Iterable[Any]) -> dict[str, Any]:
    all_rows = list(rows or [])
    total_closed = len(all_rows)
    row_reason_counts: Counter[str] = Counter()
    row_level_clean_rows: list[Any] = []

    for row in all_rows:
        flags = row_quality_flags(row)
        if flags:
            row_reason_counts.update(flags)
            continue
        row_level_clean_rows.append(row)

    day_counts: Counter[str] = Counter()
    for row in row_level_clean_rows:
        day = _row_day(row)
        if day:
            day_counts[day] += 1

    outlier_days: list[dict[str, Any]] = []
    share_threshold = max(0.05, min(0.95, float(settings.journal_quality_outlier_day_share_pct) / 100.0))
    if len(row_level_clean_rows) >= 50:
        for day, count in day_counts.most_common():
            share = _ratio(count, len(row_level_clean_rows))
            if count >= 25 and share >= share_threshold:
                outlier_days.append(
                    {
                        "date": day,
                        "count": count,
                        "share_pct": round(share * 100.0, 2),
                    }
                )

    outlier_day_keys = {item["date"] for item in outlier_days}
    trusted_rows = [row for row in row_level_clean_rows if _row_day(row) not in outlier_day_keys]
    anomalies = _strategy_anomalies(trusted_rows)

    negative_entry_count = int(row_reason_counts.get("negative_entry_price", 0))
    negative_exit_count = int(row_reason_counts.get("negative_exit_price", 0))
    short_duration_count = int(row_reason_counts.get("too_short_duration", 0))
    max_outlier_share_pct = max((float(item["share_pct"]) for item in outlier_days), default=0.0)

    score = 100.0
    score -= min(35.0, _ratio(negative_entry_count, total_closed) * 180.0)
    score -= min(35.0, _ratio(negative_exit_count, total_closed) * 180.0)
    score -= min(15.0, _ratio(short_duration_count, total_closed) * 220.0)
    score -= min(25.0, max_outlier_share_pct * 0.4)
    score -= min(20.0, len(anomalies) * 10.0)
    score = round(max(0.0, min(100.0, score)), 2)

    blocked_reasons: list[str] = []
    if _ratio(negative_entry_count, total_closed) * 100.0 >= float(settings.journal_quality_max_negative_price_ratio_pct):
        blocked_reasons.append("negative_entry_price_ratio")
    if _ratio(negative_exit_count, total_closed) * 100.0 >= float(settings.journal_quality_max_negative_price_ratio_pct):
        blocked_reasons.append("negative_exit_price_ratio")
    if max_outlier_share_pct >= float(settings.journal_quality_outlier_day_share_pct):
        blocked_reasons.append("outlier_close_day_concentration")
    if anomalies:
        blocked_reasons.append("extreme_strategy_winrate_anomaly")
    if score < float(settings.journal_quality_min_score_pct):
        blocked_reasons.append("quality_score_below_threshold")

    trusted_ratio_pct = round(_ratio(len(trusted_rows), total_closed) * 100.0, 2)
    row_level_clean_ratio_pct = round(_ratio(len(row_level_clean_rows), total_closed) * 100.0, 2)
    learning_allowed = len(trusted_rows) > 0 and not blocked_reasons

    if blocked_reasons:
        status = "critical"
    elif score < 95.0 or total_closed == 0:
        status = "watch"
    else:
        status = "healthy"

    return {
        "status": status,
        "score_pct": score,
        "learning_allowed": learning_allowed,
        "total_closed_count": total_closed,
        "row_level_clean_count": len(row_level_clean_rows),
        "trusted_closed_count": len(trusted_rows),
        "row_level_clean_ratio_pct": row_level_clean_ratio_pct,
        "trusted_ratio_pct": trusted_ratio_pct,
        "blocked_reasons": blocked_reasons,
        "row_reason_counts": dict(row_reason_counts),
        "metrics": {
            "negative_entry_price_count": negative_entry_count,
            "negative_exit_price_count": negative_exit_count,
            "short_duration_count": short_duration_count,
            "max_outlier_day_share_pct": round(max_outlier_share_pct, 2),
        },
        "outlier_days": outlier_days,
        "strategy_anomalies": anomalies,
    }


def filter_closed_entries(
    rows: Iterable[Any],
    *,
    scorecard: dict[str, Any] | None = None,
    include_outlier_days: bool,
) -> list[Any]:
    all_rows = list(rows or [])
    scorecard = scorecard or build_journal_quality_scorecard(all_rows)
    outlier_days = {str(item.get("date") or "") for item in (scorecard.get("outlier_days") or [])}
    filtered: list[Any] = []
    for row in all_rows:
        if row_quality_flags(row):
            continue
        if include_outlier_days and _row_day(row) in outlier_days:
            continue
        filtered.append(row)
    return filtered
