from __future__ import annotations

import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from sqlalchemy import select

from config.settings import settings
from atlas_code_quant.journal.db import session_scope
from atlas_code_quant.journal.models import TradingJournal
from learning.journal_data_quality import build_journal_quality_scorecard, filter_closed_entries

_BULLISH_STRATEGIES = {
    "equity_long",
    "long_call",
    "bull_call_debit_spread",
    "bull_put_credit_spread",
    "covered_call",
    "cash_secured_put",
}
_BEARISH_STRATEGIES = {
    "equity_short",
    "long_put",
    "bear_put_debit_spread",
    "bear_call_credit_spread",
}


def _utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _parse_cutoff_value(value: datetime | str | None) -> datetime | None:
    if value is None:
        return None
    if isinstance(value, datetime):
        return value.replace(tzinfo=None) if value.tzinfo is not None else value
    raw = str(value).strip()
    if not raw:
        return None
    try:
        parsed = datetime.fromisoformat(raw)
    except ValueError:
        try:
            parsed = datetime.fromisoformat(f"{raw}T00:00:00")
        except ValueError:
            return None
    return parsed.replace(tzinfo=None) if parsed.tzinfo is not None else parsed


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(out) or math.isinf(out):
        return default
    return out


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _normalized_return(entry: TradingJournal) -> float:
    base = max(abs(_safe_float(entry.risk_at_entry, 0.0)), abs(_safe_float(entry.entry_notional, 0.0)), 1.0)
    return _safe_float(entry.realized_pnl, 0.0) / base


def _direction_from_strategy(strategy_type: str) -> str | None:
    if strategy_type in _BULLISH_STRATEGIES:
        return "alcista"
    if strategy_type in _BEARISH_STRATEGIES:
        return "bajista"
    return None


def _empty_scope_snapshot(scope: str) -> dict[str, Any]:
    return {
        "scope": scope,
        "sample_count": 0,
        "win_rate_pct": 0.0,
        "profit_factor": 0.0,
        "expectancy_pct": 0.0,
        "risk_multiplier": 1.0,
        "strategy_biases": {},
        "symbol_biases": {},
        "directional_symbol_biases": {},
        "top_positive": [],
        "top_negative": [],
    }


def _empty_quality_snapshot() -> dict[str, Any]:
    return {
        "status": "watch",
        "score_pct": 100.0,
        "learning_allowed": True,
        "total_closed_count": 0,
        "row_level_clean_count": 0,
        "trusted_closed_count": 0,
        "row_level_clean_ratio_pct": 0.0,
        "trusted_ratio_pct": 0.0,
        "blocked_reasons": [],
        "row_reason_counts": {},
        "metrics": {},
        "outlier_days": [],
        "strategy_anomalies": [],
    }


class AdaptiveLearningService:
    """Conservative, explainable learning overlay fed by the trading journal."""

    def __init__(
        self,
        *,
        enabled: bool | None = None,
        snapshot_path: Path | None = None,
        refresh_sec: int | None = None,
        window_days: int | None = None,
    ) -> None:
        self.enabled = settings.adaptive_learning_enabled if enabled is None else bool(enabled)
        self.snapshot_path = snapshot_path or settings.adaptive_learning_snapshot_path
        self.refresh_sec = int(refresh_sec or settings.adaptive_learning_refresh_sec)
        self.window_days = int(window_days or settings.adaptive_learning_window_days)
        self._snapshot = self._load_snapshot()

    def _load_snapshot(self) -> dict[str, Any]:
        if not self.snapshot_path.exists():
            return self._empty_snapshot()
        try:
            payload = json.loads(self.snapshot_path.read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                return payload
        except Exception:
            pass
        return self._empty_snapshot()

    def _save_snapshot(self, payload: dict[str, Any]) -> dict[str, Any]:
        self.snapshot_path.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
        self._snapshot = payload
        return payload

    def _empty_snapshot(self) -> dict[str, Any]:
        return {
            "generated_at": _utcnow_iso(),
            "enabled": self.enabled,
            "learning_allowed": self.enabled,
            "window_days": self.window_days,
            "sample_count": 0,
            "raw_sample_count": 0,
            "quality": _empty_quality_snapshot(),
            "scopes": {
                "all": _empty_scope_snapshot("all"),
                "paper": _empty_scope_snapshot("paper"),
                "live": _empty_scope_snapshot("live"),
            },
        }

    def _is_stale(self, payload: dict[str, Any]) -> bool:
        generated_at = str(payload.get("generated_at") or "").strip()
        if not generated_at:
            return True
        try:
            generated = datetime.fromisoformat(generated_at.replace("Z", "+00:00"))
        except ValueError:
            return True
        return (datetime.now(timezone.utc) - generated).total_seconds() >= max(self.refresh_sec, 30)

    def _entry_scope(self, entry: TradingJournal) -> str:
        scope = str(entry.account_type or "").strip().lower()
        return scope if scope in {"paper", "live"} else "paper"

    def _metric_bias(self, stats: dict[str, Any], *, cap: float) -> float:
        sample = max(int(stats.get("sample") or 0), 0)
        if sample <= 0:
            return 0.0
        confidence = _clip(math.sqrt(sample) / 4.0, 0.15, 1.0)
        win_rate_pct = _safe_float(stats.get("win_rate_pct"), 0.0)
        expectancy_pct = _safe_float(stats.get("expectancy_pct"), 0.0)
        profit_factor = _safe_float(stats.get("profit_factor"), 0.0)
        raw_score = ((win_rate_pct - 50.0) * 0.08) + (_clip(expectancy_pct, -6.0, 6.0) * 0.35)
        raw_score += (_clip(profit_factor, 0.5, 3.0) - 1.0) * 1.2
        return round(_clip(raw_score * confidence, -cap, cap), 2)

    def _risk_multiplier(self, stats: dict[str, Any]) -> float:
        sample = max(int(stats.get("sample") or 0), 0)
        if sample <= 0:
            return 1.0
        confidence = _clip(math.sqrt(sample) / 5.0, 0.10, 1.0)
        expectancy_pct = _safe_float(stats.get("expectancy_pct"), 0.0)
        profit_factor = _safe_float(stats.get("profit_factor"), 0.0)
        raw_multiplier = 1.0 + (_clip(expectancy_pct, -4.0, 4.0) * 0.02)
        raw_multiplier += (_clip(profit_factor, 0.6, 1.8) - 1.0) * 0.10
        adjusted = 1.0 + ((raw_multiplier - 1.0) * confidence)
        return round(_clip(adjusted, 0.90, 1.10), 3)

    def _aggregate_stats(self, entries: list[TradingJournal]) -> dict[str, Any]:
        pnl_pos = 0.0
        pnl_neg = 0.0
        wins = 0
        returns: list[float] = []
        for entry in entries:
            realized = _safe_float(entry.realized_pnl, 0.0)
            if realized > 0:
                wins += 1
                pnl_pos += realized
            elif realized < 0:
                pnl_neg += abs(realized)
            returns.append(_normalized_return(entry))
        sample = len(entries)
        expectancy_pct = (sum(returns) / sample) * 100.0 if sample else 0.0
        return {
            "sample": sample,
            "wins": wins,
            "win_rate_pct": round((wins / sample) * 100.0, 2) if sample else 0.0,
            "profit_factor": round(pnl_pos / pnl_neg, 3) if pnl_neg > 0 else (999.0 if pnl_pos > 0 else 0.0),
            "expectancy_pct": round(expectancy_pct, 3),
            "pnl_sum": round(sum(_safe_float(entry.realized_pnl, 0.0) for entry in entries), 2),
        }

    def _scope_snapshot(self, entries: list[TradingJournal], scope: str) -> dict[str, Any]:
        scope_entries = [entry for entry in entries if scope == "all" or self._entry_scope(entry) == scope]
        overall = self._aggregate_stats(scope_entries)
        strategy_map: dict[str, list[TradingJournal]] = {}
        symbol_map: dict[str, list[TradingJournal]] = {}
        direction_map: dict[str, list[TradingJournal]] = {}

        for entry in scope_entries:
            strategy_type = str(entry.strategy_type or "").strip()
            symbol = str(entry.symbol or "").strip().upper()
            if strategy_type:
                strategy_map.setdefault(strategy_type, []).append(entry)
            if symbol:
                symbol_map.setdefault(symbol, []).append(entry)
                direction = _direction_from_strategy(strategy_type)
                if direction:
                    direction_map.setdefault(f"{symbol}:{direction}", []).append(entry)

        strategy_biases: dict[str, Any] = {}
        for strategy_type, rows in strategy_map.items():
            stats = self._aggregate_stats(rows)
            if stats["sample"] < settings.adaptive_learning_min_strategy_samples:
                continue
            strategy_biases[strategy_type] = {
                **stats,
                "bias_score": self._metric_bias(stats, cap=5.0),
            }

        symbol_biases: dict[str, Any] = {}
        for symbol, rows in symbol_map.items():
            stats = self._aggregate_stats(rows)
            if stats["sample"] < settings.adaptive_learning_min_symbol_samples:
                continue
            symbol_biases[symbol] = {
                **stats,
                "bias_score": self._metric_bias(stats, cap=3.5),
            }

        directional_biases: dict[str, Any] = {}
        for key, rows in direction_map.items():
            stats = self._aggregate_stats(rows)
            if stats["sample"] < settings.adaptive_learning_min_symbol_samples:
                continue
            directional_biases[key] = {
                **stats,
                "bias_score": self._metric_bias(stats, cap=2.5),
            }

        ranked = [
            {"kind": "strategy", "key": key, "bias_score": value["bias_score"], "sample": value["sample"]}
            for key, value in strategy_biases.items()
        ] + [
            {"kind": "symbol", "key": key, "bias_score": value["bias_score"], "sample": value["sample"]}
            for key, value in symbol_biases.items()
        ]
        ranked = sorted(ranked, key=lambda item: float(item.get("bias_score") or 0.0), reverse=True)

        return {
            "scope": scope,
            "sample_count": overall["sample"],
            "win_rate_pct": overall["win_rate_pct"],
            "profit_factor": overall["profit_factor"],
            "expectancy_pct": overall["expectancy_pct"],
            "risk_multiplier": self._risk_multiplier(overall),
            "strategy_biases": strategy_biases,
            "symbol_biases": symbol_biases,
            "directional_symbol_biases": directional_biases,
            "top_positive": [item for item in ranked if float(item.get("bias_score") or 0.0) > 0][:4],
            "top_negative": [item for item in sorted(ranked, key=lambda item: float(item.get("bias_score") or 0.0)) if float(item.get("bias_score") or 0.0) < 0][:4],
        }

    def _compute_snapshot(self, *, min_cutoff: datetime | str | None = None) -> dict[str, Any]:
        if not self.enabled:
            return self._empty_snapshot()
        cutoff = datetime.utcnow() - timedelta(days=self.window_days)
        epoch_start: datetime | None = None
        try:
            epoch_str = settings.adaptive_learning_epoch_start
            if epoch_str:
                epoch_start = datetime.fromisoformat(epoch_str)
        except (ValueError, AttributeError):
            pass
        if epoch_start is not None and epoch_start > cutoff:
            cutoff = epoch_start
        rebuild_cutoff = _parse_cutoff_value(min_cutoff)
        if rebuild_cutoff is not None and rebuild_cutoff > cutoff:
            cutoff = rebuild_cutoff

        with session_scope() as session:
            rows = list(
                session.scalars(
                    select(TradingJournal)
                    .where(TradingJournal.status == "closed")
                    .order_by(TradingJournal.exit_time.desc(), TradingJournal.updated_at.desc())
                    .limit(2000)
                )
            )

        def _naive(dt: datetime | None) -> datetime | None:
            return dt.replace(tzinfo=None) if dt is not None and dt.tzinfo is not None else dt

        rows = [row for row in rows if (_naive(row.exit_time) or _naive(row.updated_at) or _naive(row.entry_time) or cutoff) >= cutoff]
        quality = build_journal_quality_scorecard(rows)
        effective_rows = filter_closed_entries(rows, scorecard=quality, include_outlier_days=True)
        if not quality.get("learning_allowed", False):
            effective_rows = []
        return {
            "generated_at": _utcnow_iso(),
            "enabled": self.enabled,
            "learning_allowed": bool(self.enabled and quality.get("learning_allowed", False)),
            "window_days": self.window_days,
            "effective_cutoff": cutoff.isoformat(),
            "sample_count": len(effective_rows),
            "raw_sample_count": len(rows),
            "quality": quality,
            "scopes": {
                "all": self._scope_snapshot(effective_rows, "all"),
                "paper": self._scope_snapshot(effective_rows, "paper"),
                "live": self._scope_snapshot(effective_rows, "live"),
            },
        }

    def refresh(self, force: bool = False, *, min_cutoff: datetime | str | None = None) -> dict[str, Any]:
        if force or min_cutoff is not None or self._is_stale(self._snapshot):
            return self._save_snapshot(self._compute_snapshot(min_cutoff=min_cutoff))
        return self._snapshot

    def _select_scope(self, account_scope: str | None, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = payload or self.refresh()
        scopes = payload.get("scopes") or {}
        scope = str(account_scope or "paper").strip().lower()
        preferred = scopes.get(scope) if scope in {"paper", "live"} else None
        if preferred and int(preferred.get("sample_count") or 0) >= 3:
            return preferred
        return scopes.get("all") or _empty_scope_snapshot("all")

    def status(self, account_scope: str | None = None, *, fast: bool = False) -> dict[str, Any]:
        payload = self._snapshot if fast else self.refresh()
        scope_data = self._select_scope(account_scope, payload=payload)
        return {
            "enabled": bool(payload.get("enabled", False)),
            "learning_allowed": bool(payload.get("learning_allowed", payload.get("enabled", False))),
            "generated_at": payload.get("generated_at"),
            "window_days": payload.get("window_days"),
            "scope": scope_data.get("scope"),
            "sample_count": scope_data.get("sample_count", 0),
            "raw_sample_count": payload.get("raw_sample_count", payload.get("sample_count", 0)),
            "win_rate_pct": scope_data.get("win_rate_pct", 0.0),
            "profit_factor": scope_data.get("profit_factor", 0.0),
            "expectancy_pct": scope_data.get("expectancy_pct", 0.0),
            "risk_multiplier": scope_data.get("risk_multiplier", 1.0),
            "top_positive": scope_data.get("top_positive", []),
            "top_negative": scope_data.get("top_negative", []),
            "quality": payload.get("quality") or _empty_quality_snapshot(),
            "status_mode": "fast_cached" if fast else "full",
        }

    def strategy_bias(self, strategy_type: str | None, account_scope: str | None = None) -> float:
        if not strategy_type:
            return 0.0
        scope_data = self._select_scope(account_scope)
        bias = ((scope_data.get("strategy_biases") or {}).get(str(strategy_type)) or {}).get("bias_score")
        return round(_safe_float(bias, 0.0), 2)

    def symbol_bias(self, symbol: str | None, account_scope: str | None = None) -> float:
        if not symbol:
            return 0.0
        scope_data = self._select_scope(account_scope)
        bias = ((scope_data.get("symbol_biases") or {}).get(str(symbol).upper()) or {}).get("bias_score")
        return round(_safe_float(bias, 0.0), 2)

    def directional_symbol_bias(self, symbol: str | None, direction: str | None, account_scope: str | None = None) -> float:
        if not symbol or not direction:
            return 0.0
        scope_data = self._select_scope(account_scope)
        key = f"{str(symbol).upper()}:{str(direction).lower()}"
        bias = ((scope_data.get("directional_symbol_biases") or {}).get(key) or {}).get("bias_score")
        return round(_safe_float(bias, 0.0), 2)

    def risk_multiplier(self, account_scope: str | None = None) -> float:
        scope_data = self._select_scope(account_scope)
        return round(_safe_float(scope_data.get("risk_multiplier"), 1.0), 3)

    def context(
        self,
        *,
        symbol: str | None,
        direction: str | None = None,
        strategy_type: str | None = None,
        account_scope: str | None = None,
    ) -> dict[str, Any]:
        status = self.status(account_scope=account_scope)
        symbol_bias = self.symbol_bias(symbol, account_scope=account_scope)
        directional_bias = self.directional_symbol_bias(symbol, direction, account_scope=account_scope)
        strategy_bias = self.strategy_bias(strategy_type, account_scope=account_scope)
        total_bias = round(_clip(symbol_bias + directional_bias + strategy_bias, -6.0, 6.0), 2)
        notes: list[str] = []
        if total_bias >= 1.0:
            notes.append("el diario reciente favorece este tipo de setup")
        elif total_bias <= -1.0:
            notes.append("el diario reciente enfriÃ³ este setup y reduce convicciÃ³n")
        if status.get("sample_count", 0) < 6:
            notes.append("aprendizaje con pocas muestras; se usa como ajuste ligero")
        quality = status.get("quality") or {}
        if not status.get("learning_allowed", True):
            reasons = ", ".join(str(reason) for reason in (quality.get("blocked_reasons") or []))
            notes.append(f"aprendizaje bloqueado por calidad del journal: {reasons or 'motivo no especificado'}")
        return {
            "enabled": status.get("enabled", False),
            "learning_allowed": status.get("learning_allowed", status.get("enabled", False)),
            "generated_at": status.get("generated_at"),
            "scope": status.get("scope"),
            "sample_count": status.get("sample_count", 0),
            "risk_multiplier": status.get("risk_multiplier", 1.0),
            "symbol_bias": symbol_bias,
            "directional_symbol_bias": directional_bias,
            "strategy_bias": strategy_bias,
            "total_bias": total_bias,
            "top_positive": status.get("top_positive", []),
            "top_negative": status.get("top_negative", []),
            "quality": quality,
            "notes": notes,
        }


