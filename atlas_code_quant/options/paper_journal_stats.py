"""Reader + agregador de performance paper desde OptionsPaperJournal JSONL."""
from __future__ import annotations

import json
import logging
import math
import os
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

logger = logging.getLogger("atlas.options.paper_journal_stats")


def _default_initial_capital() -> float:
    raw = str(os.getenv("QUANT_OPTIONS_RUNTIME_CAPITAL", "10000")).strip()
    try:
        v = float(raw)
    except Exception:
        return 10_000.0
    return v if math.isfinite(v) and v > 0 else 10_000.0


def _parse_ts(raw: Any) -> datetime | None:
    s = str(raw or "").strip()
    if not s:
        return None
    try:
        dt = datetime.fromisoformat(s.replace("Z", "+00:00"))
    except Exception:
        return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    else:
        dt = dt.astimezone(timezone.utc)
    return dt


def _to_float(value: Any) -> float | None:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _coalesce(*values: Any, default: str = "unknown") -> str:
    for v in values:
        s = str(v or "").strip()
        if s:
            return s
    return default


def _summarize_pnls(pnls: list[float], *, initial_capital: float) -> dict[str, Any]:
    n = len(pnls)
    wins = [p for p in pnls if p > 0]
    losses = [p for p in pnls if p <= 0]
    wins_total = len(wins)
    losses_total = len(losses)
    gross_win = float(sum(wins))
    gross_loss = float(sum(losses))
    total = float(sum(pnls))
    avg = total / n if n > 0 else 0.0
    win_rate_pct = (wins_total / n * 100.0) if n > 0 else 0.0
    loss_abs = abs(gross_loss)
    if loss_abs > 1e-12:
        profit_factor = gross_win / loss_abs
    elif gross_win > 1e-12:
        # Convención explícita para no dividir entre cero.
        profit_factor = 99.0
    else:
        profit_factor = 0.0

    equity = initial_capital
    peak = initial_capital
    max_dd_usd = 0.0
    drawdown_pct = 0.0
    for pnl in pnls:
        equity += float(pnl)
        if equity > peak:
            peak = equity
        dd_usd = max(0.0, peak - equity)
        if dd_usd > max_dd_usd:
            max_dd_usd = dd_usd
        drawdown_pct = ((equity - peak) / peak * 100.0) if peak > 0 else 0.0

    return {
        "closed_trades_total": n,
        "wins_total": wins_total,
        "losses_total": losses_total,
        "win_rate_pct": round(win_rate_pct, 6),
        "win_rate_ratio": round(win_rate_pct / 100.0, 6),
        "profit_factor": round(float(profit_factor), 6),
        "pnl_total_usd": round(total, 6),
        "net_realized_pnl_usd": round(total, 6),
        "pnl_avg_usd": round(avg, 6),
        "equity_usd": round(equity, 6),
        "drawdown_pct": round(drawdown_pct, 6),
        "max_drawdown_usd": round(max_dd_usd, 6),
    }


@dataclass
class OptionsPaperJournalStats:
    journal_path: Path
    initial_capital: float
    now_utc: datetime
    events_today: int
    sessions_today: int
    closed_today: int
    open_trades_count: int
    phantom_today: int
    last_write_ts: float
    parse_warnings: int
    closed_trades: list[dict[str, Any]]
    by_strategy: dict[str, dict[str, Any]]
    by_strategy_regime: dict[str, dict[str, float]]
    global_stats: dict[str, Any]
    rolling_7d: dict[str, Any]

    @classmethod
    def load_from_file(
        cls,
        path: str | Path,
        *,
        initial_capital: float | None = None,
        now_utc: datetime | None = None,
    ) -> "OptionsPaperJournalStats":
        p = Path(path)
        now = now_utc or datetime.now(timezone.utc)
        if now.tzinfo is None:
            now = now.replace(tzinfo=timezone.utc)
        else:
            now = now.astimezone(timezone.utc)
        base_capital = float(initial_capital if initial_capital is not None else _default_initial_capital())

        events_today = 0
        sessions_today = 0
        closed_today = 0
        parse_warnings = 0
        last_write_ts = 0.0

        trace_context: dict[str, dict[str, Any]] = {}
        trace_entry: set[str] = set()
        trace_close: set[str] = set()
        close_rows: list[dict[str, Any]] = []
        all_rows: list[dict[str, Any]] = []

        if p.is_file():
            for raw_line in p.read_text(encoding="utf-8", errors="ignore").splitlines():
                line = raw_line.strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    parse_warnings += 1
                    logger.warning("paper_journal_stats: línea JSON corrupta ignorada")
                    continue
                if not isinstance(row, dict):
                    parse_warnings += 1
                    continue
                all_rows.append(row)

        utc_day = now.strftime("%Y-%m-%d")
        for row in all_rows:
            et = str(row.get("event_type") or "")
            tid = str(row.get("trace_id") or "").strip()
            ts = _parse_ts(row.get("timestamp_utc") or row.get("timestamp"))
            if ts is not None:
                last_write_ts = max(last_write_ts, ts.timestamp())
                if ts.strftime("%Y-%m-%d") == utc_day:
                    events_today += 1
                    if et == "session_plan":
                        sessions_today += 1
                    elif et == "close_execution":
                        closed_today += 1

            if not tid:
                continue
            ctx = trace_context.setdefault(
                tid,
                {
                    "trace_id": tid,
                    "symbol": str(row.get("symbol") or "").strip().upper(),
                    "strategy": str(row.get("strategy_type") or row.get("structure_type") or "").strip(),
                    "close_reason": str(row.get("close_reason") or "").strip(),
                    "gamma_regime": str(row.get("gamma_regime") or "").strip(),
                    "dte_mode": str(row.get("dte_mode") or "").strip(),
                },
            )

            if et == "session_plan":
                payload = row.get("payload") if isinstance(row.get("payload"), dict) else {}
                plan = payload.get("session_plan") if isinstance(payload.get("session_plan"), dict) else {}
                briefing = plan.get("briefing") if isinstance(plan.get("briefing"), dict) else {}
                entry_plan = plan.get("entry_plan") if isinstance(plan.get("entry_plan"), dict) else {}
                intent = plan.get("intent") if isinstance(plan.get("intent"), dict) else {}
                ctx["symbol"] = _coalesce(ctx.get("symbol"), row.get("symbol"), plan.get("symbol"), default="UNKNOWN").upper()
                ctx["strategy"] = _coalesce(
                    ctx.get("strategy"),
                    entry_plan.get("recommended_strategy"),
                    intent.get("recommended_strategy"),
                )
                ctx["gamma_regime"] = _coalesce(ctx.get("gamma_regime"), briefing.get("gamma_regime"))
                ctx["dte_mode"] = _coalesce(ctx.get("dte_mode"), briefing.get("dte_mode"))

            if et == "entry_execution":
                payload = row.get("payload") if isinstance(row.get("payload"), dict) else {}
                planned = payload.get("planned_entry") if isinstance(payload.get("planned_entry"), dict) else {}
                ctx["symbol"] = _coalesce(ctx.get("symbol"), row.get("symbol"), default="UNKNOWN").upper()
                ctx["strategy"] = _coalesce(
                    ctx.get("strategy"),
                    row.get("strategy_type"),
                    row.get("structure_type"),
                    planned.get("recommended_strategy"),
                )
                trace_entry.add(tid)

            if et == "close_decision":
                payload = row.get("payload") if isinstance(row.get("payload"), dict) else {}
                decision = payload.get("close_decision") if isinstance(payload.get("close_decision"), dict) else {}
                ctx["close_reason"] = _coalesce(ctx.get("close_reason"), row.get("close_reason"), decision.get("reason"))

            if et == "close_execution":
                payload = row.get("payload") if isinstance(row.get("payload"), dict) else {}
                exec_close = payload.get("executed_close") if isinstance(payload.get("executed_close"), dict) else {}
                pnl = _to_float(row.get("pnl_usd"))
                if pnl is None:
                    pnl = _to_float(payload.get("pnl_realized"))
                if pnl is None:
                    continue
                if ts is None:
                    ts = now
                close_reason = _coalesce(
                    row.get("close_reason"),
                    exec_close.get("close_reason"),
                    ctx.get("close_reason"),
                )
                close_rows.append(
                    {
                        "trace_id": tid,
                        "closed_at": ts.isoformat().replace("+00:00", "Z"),
                        "closed_at_dt": ts,
                        "symbol": _coalesce(row.get("symbol"), ctx.get("symbol"), default="UNKNOWN").upper(),
                        "strategy": _coalesce(
                            row.get("strategy_type"),
                            row.get("structure_type"),
                            exec_close.get("strategy_type"),
                            ctx.get("strategy"),
                        ),
                        "close_reason": close_reason,
                        "gamma_regime": _coalesce(
                            row.get("gamma_regime"),
                            exec_close.get("gamma_regime"),
                            ctx.get("gamma_regime"),
                        ),
                        "dte_mode": _coalesce(
                            row.get("dte_mode"),
                            exec_close.get("dte_mode"),
                            ctx.get("dte_mode"),
                        ),
                        "pnl_usd": float(pnl),
                    }
                )
                trace_close.add(tid)

        close_rows.sort(key=lambda x: x["closed_at_dt"])
        pnls = [float(x["pnl_usd"]) for x in close_rows]
        global_stats = _summarize_pnls(pnls, initial_capital=base_capital)

        cutoff = now - timedelta(days=7)
        rolling_rows = [x for x in close_rows if x["closed_at_dt"] >= cutoff]
        rolling = _summarize_pnls([float(x["pnl_usd"]) for x in rolling_rows], initial_capital=base_capital)
        rolling_7d = {
            "closed_trades": int(rolling["closed_trades_total"]),
            "win_rate_pct": float(rolling["win_rate_pct"]),
            "profit_factor": float(rolling["profit_factor"]),
        }

        by_strategy_rows: dict[str, list[float]] = {}
        by_regime_rows: dict[tuple[str, str], list[float]] = {}
        for row in close_rows:
            strategy = _coalesce(row.get("strategy"))
            gamma_regime = _coalesce(row.get("gamma_regime"))
            by_strategy_rows.setdefault(strategy, []).append(float(row["pnl_usd"]))
            by_regime_rows.setdefault((strategy, gamma_regime), []).append(float(row["pnl_usd"]))

        by_strategy: dict[str, dict[str, Any]] = {}
        for strategy, vals in by_strategy_rows.items():
            agg = _summarize_pnls(vals, initial_capital=base_capital)
            by_strategy[strategy] = {
                "closed_trades_total": int(agg["closed_trades_total"]),
                "wins_total": int(agg["wins_total"]),
                "losses_total": int(agg["losses_total"]),
                "win_rate_pct": float(agg["win_rate_pct"]),
                "profit_factor": float(agg["profit_factor"]),
                "pnl_total_usd": float(agg["pnl_total_usd"]),
                "pnl_avg_usd": float(agg["pnl_avg_usd"]),
            }

        by_strategy_regime: dict[str, dict[str, float]] = {}
        for (strategy, gamma_regime), vals in by_regime_rows.items():
            agg = _summarize_pnls(vals, initial_capital=base_capital)
            by_strategy_regime.setdefault(strategy, {})[gamma_regime] = float(agg["win_rate_pct"])

        open_trades_count = len(trace_entry - trace_close)
        phantom_today = max(0, sessions_today - closed_today - len(trace_entry))
        return cls(
            journal_path=p,
            initial_capital=base_capital,
            now_utc=now,
            events_today=events_today,
            sessions_today=sessions_today,
            closed_today=closed_today,
            open_trades_count=open_trades_count,
            phantom_today=phantom_today,
            last_write_ts=last_write_ts,
            parse_warnings=parse_warnings,
            closed_trades=close_rows,
            by_strategy=by_strategy,
            by_strategy_regime=by_strategy_regime,
            global_stats=global_stats,
            rolling_7d=rolling_7d,
        )

    def to_dict_global(self) -> dict[str, Any]:
        out = dict(self.global_stats)
        out["events_today"] = int(self.events_today)
        out["sessions_today"] = int(self.sessions_today)
        out["closed_today"] = int(self.closed_today)
        out["open_trades_count"] = int(self.open_trades_count)
        out["phantom_today"] = int(self.phantom_today)
        out["last_write_age_seconds"] = max(0.0, self.now_utc.timestamp() - self.last_write_ts) if self.last_write_ts > 0 else 1e9
        out["initial_capital_usd"] = float(self.initial_capital)
        out["parse_warnings"] = int(self.parse_warnings)
        return out

    def to_dict_by_strategy(self) -> dict[str, dict[str, Any]]:
        return {k: dict(v) for k, v in self.by_strategy.items()}

    def to_dict_by_strategy_regime(self) -> dict[str, dict[str, float]]:
        return {k: dict(v) for k, v in self.by_strategy_regime.items()}

    def to_debug_dict(self) -> dict[str, Any]:
        return {
            "ok": True,
            **self.to_dict_global(),
            "rolling_7d": dict(self.rolling_7d),
            "by_strategy": self.to_dict_by_strategy(),
            "by_strategy_regime": self.to_dict_by_strategy_regime(),
            "generated_at_utc": self.now_utc.isoformat().replace("+00:00", "Z"),
        }

    def to_api_dict(self) -> dict[str, Any]:
        g = self.to_dict_global()
        summary_keys = (
            "closed_trades_total",
            "wins_total",
            "losses_total",
            "win_rate_pct",
            "profit_factor",
            "pnl_total_usd",
            "pnl_avg_usd",
            "equity_usd",
            "drawdown_pct",
        )
        summary = {k: g.get(k) for k in summary_keys}
        out = {
            "ok": True,
            "scope": "paper_phase0_internal",
            "summary": summary,
            "rolling_7d": dict(self.rolling_7d),
            "by_strategy_regime": self.to_dict_by_strategy_regime(),
            "generated_at_utc": self.now_utc.isoformat().replace("+00:00", "Z"),
        }
        if int(g.get("closed_trades_total") or 0) <= 0:
            out["message"] = "no trades yet"
        return out

    def write_debug_json(self, target_path: str | Path) -> Path:
        path = Path(target_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(self.to_debug_dict(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return path
