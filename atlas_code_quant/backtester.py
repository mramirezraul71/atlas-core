"""ATLAS Code-Quant - Backtester de validacion Fase 2.

Backtester enfocado en validacion operativa previa a paper trading:
- Carga historicos (cache local + yfinance via MarketFeed).
- Simula ciclos barra a barra reutilizando metodos del scanner productivo.
- Ejecuta entradas/salidas con SL/TP, slippage y comision.
- Calcula metricas clave (WR, Sharpe, DD, PF, rachas, etc).
- Hace busqueda de parametros cuando no se cumplen criterios go-live.
"""
from __future__ import annotations

import itertools
import json
import logging
import math
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from config.settings import settings
from data.feed import MarketFeed
from market_context.regime_detector import detect_regime_from_frame
from scanner.opportunity_scanner import OpportunityScannerService, _method_order_for_regime

logger = logging.getLogger("quant.backtester")


@dataclass
class TradeRecord:
    symbol: str
    direction: int
    entry_time: pd.Timestamp
    entry_price: float
    position_size: float
    stop_loss: float
    take_profit: float
    status: str = "open"
    exit_time: pd.Timestamp | None = None
    exit_price: float = 0.0
    pnl: float = 0.0
    pnl_pct: float = 0.0
    commission_paid: float = 0.0
    slippage_pct: float = 0.0


@dataclass
class BacktestDecision:
    go_live_approved: bool
    reasons: list[str]
    metrics: dict[str, Any]


class AtlasBacktester:
    """Backtester profesional con validacion y busqueda de parametros."""

    CRITERIA = {
        "win_rate_pct": 58.0,
        "sharpe_ratio": 1.0,
        "max_drawdown_pct": -15.0,
        "profit_factor": 1.25,
        "total_trades": 30,
        "consecutive_losses_max": 4,  # <5
    }

    def __init__(
        self,
        start_date: str,
        end_date: str,
        initial_capital: float = 10_000.0,
        max_trades_per_cycle: int = 5,
        risk_per_trade: float = 0.02,
        search_mode: bool = True,
        timeframe: str = "1h",
        slippage_pct: float = 0.0001,  # 0.01%
        commission_pct: float = 0.00001,  # 0.001%
        random_seed: int = 42,
    ) -> None:
        self.start_date = start_date
        self.end_date = end_date
        self.initial_capital = float(initial_capital)
        self.capital = float(initial_capital)
        self.max_trades_per_cycle = int(max_trades_per_cycle)
        self.risk_per_trade = float(risk_per_trade)
        self.search_mode = bool(search_mode)
        self.timeframe = timeframe
        self.slippage_pct = float(slippage_pct)
        self.commission_pct = float(commission_pct)
        self.random_seed = int(random_seed)

        self.trades: list[TradeRecord] = []
        self.open_trades: dict[str, TradeRecord] = {}
        self.equity_curve: list[dict[str, Any]] = []
        self.metrics: dict[str, Any] = {}
        self.parameter_search_history: list[dict[str, Any]] = []
        self.data: dict[str, pd.DataFrame] = {}
        self.last_symbol_prices: dict[str, float] = {}
        self._data_dir = Path(__file__).resolve().parent / "data" / "backtest"
        self._reports_dir = Path(__file__).resolve().parent / "reports"

        # Parametros "optimizables" en search.
        self.signal_strength_threshold = float(settings.scanner_min_signal_strength)
        self.stop_loss_pct = 0.02
        self.reward_to_risk = 2.0

        np.random.seed(self.random_seed)
        self.scanner = OpportunityScannerService()

    @property
    def data_dir(self) -> Path:
        return self._data_dir

    @property
    def reports_dir(self) -> Path:
        return self._reports_dir

    def load_historical_data(self, symbols: list[str], timeframe: str = "1h") -> dict[str, pd.DataFrame]:
        """Carga historicos desde cache local o yfinance."""
        self.timeframe = timeframe
        self.data_dir.mkdir(parents=True, exist_ok=True)
        out: dict[str, pd.DataFrame] = {}
        feed = MarketFeed(source="yfinance")

        for symbol in symbols:
            safe = symbol.replace("/", "-").replace(":", "_")
            local = self.data_dir / f"{safe}_{timeframe}_{self.start_date}_to_{self.end_date}.csv"
            try:
                if local.is_file():
                    df = pd.read_csv(local, parse_dates=["timestamp"])
                    df = df.set_index("timestamp")
                    if df.index.tz is None:
                        df.index = pd.to_datetime(df.index, utc=True)
                    logger.info("Loaded %s from cache: %s", symbol, local)
                else:
                    df = feed.ohlcv(symbol=symbol, timeframe=timeframe, limit=2_000)
                    if df is None or df.empty:
                        logger.warning("No data for %s", symbol)
                        continue
                    df = df[(df.index >= pd.Timestamp(self.start_date, tz="UTC")) & (df.index <= pd.Timestamp(self.end_date, tz="UTC"))]
                    if df.empty:
                        logger.warning("Filtered range empty for %s", symbol)
                        continue
                    to_save = df.reset_index().rename(columns={"index": "timestamp"})
                    to_save.to_csv(local, index=False)
                    logger.info("Downloaded %s and cached at %s", symbol, local)
                need = {"open", "high", "low", "close", "volume"}
                if not need.issubset(df.columns):
                    logger.warning("Missing OHLCV columns for %s", symbol)
                    continue
                out[symbol] = df.sort_index()
            except Exception as exc:
                logger.error("Failed loading %s: %s", symbol, exc)
        self.data = out
        return out

    def _extract_cycle_snapshot(self, data: dict[str, pd.DataFrame], timestamp: pd.Timestamp) -> dict[str, pd.DataFrame]:
        snap: dict[str, pd.DataFrame] = {}
        for symbol, df in data.items():
            hist = df.loc[df.index <= timestamp].tail(300)
            if len(hist) >= 60:
                snap[symbol] = hist
                self.last_symbol_prices[symbol] = float(hist["close"].iloc[-1])
        return snap

    def _rank_cycle_candidates(self, cycle_data: dict[str, pd.DataFrame]) -> list[dict[str, Any]]:
        candidates: list[dict[str, Any]] = []
        for symbol, raw in cycle_data.items():
            try:
                df = self.scanner._enrich_scanner_ohlcv(raw.copy())
            except Exception:
                df = raw.copy()
            regime = detect_regime_from_frame(df)
            best: dict[str, Any] | None = None
            for method in _method_order_for_regime(regime):
                try:
                    sig = self.scanner._method_signal(method, df, symbol, self.timeframe)
                except Exception:
                    continue
                direction = int(sig.get("direction") or 0)
                strength = float(sig.get("strength") or 0.0)
                if direction == 0 or strength < self.signal_strength_threshold:
                    continue
                score = strength * 100.0
                cand = {
                    "symbol": symbol,
                    "timeframe": self.timeframe,
                    "method": method,
                    "direction": direction,
                    "strength": strength,
                    "selection_score": score,
                    "entry_price": float(sig.get("price") or df["close"].iloc[-1]),
                }
                if best is None or cand["selection_score"] > best["selection_score"]:
                    best = cand
            if best:
                candidates.append(best)
        candidates.sort(key=lambda x: float(x["selection_score"]), reverse=True)
        return candidates

    def _execute_entry(self, candidate: dict[str, Any], timestamp: pd.Timestamp) -> TradeRecord:
        symbol = str(candidate["symbol"])
        direction = 1 if int(candidate.get("direction") or 0) > 0 else -1
        base_entry = float(candidate.get("entry_price") or self.last_symbol_prices.get(symbol) or 0.0)
        if base_entry <= 0:
            raise ValueError(f"entry price invalido para {symbol}")
        slip_mult = 1.0 + (self.slippage_pct * direction)
        entry_price = base_entry * slip_mult
        stop_distance = max(entry_price * self.stop_loss_pct, 1e-6)
        max_risk_usd = self.capital * self.risk_per_trade
        position_size = max(max_risk_usd / stop_distance, 0.0)
        stop_loss = entry_price - (stop_distance * direction)
        take_profit = entry_price + (stop_distance * self.reward_to_risk * direction)
        trade = TradeRecord(
            symbol=symbol,
            direction=direction,
            entry_time=timestamp,
            entry_price=entry_price,
            position_size=position_size,
            stop_loss=stop_loss,
            take_profit=take_profit,
            slippage_pct=self.slippage_pct,
        )
        notional = trade.position_size * trade.entry_price
        fee = notional * self.commission_pct
        trade.commission_paid += fee
        self.capital -= fee
        return trade

    def _evaluate_open_trades(self, timestamp: pd.Timestamp, cycle_data: dict[str, pd.DataFrame]) -> list[TradeRecord]:
        closed: list[TradeRecord] = []
        for symbol, trade in list(self.open_trades.items()):
            df = cycle_data.get(symbol)
            if df is None or df.empty:
                continue
            bar = df.iloc[-1]
            high = float(bar["high"])
            low = float(bar["low"])
            close = float(bar["close"])
            exit_price: float | None = None
            status = "closed_eod" if self._is_end_of_day(timestamp) else ""

            if trade.direction > 0:
                if high >= trade.take_profit:
                    exit_price = trade.take_profit
                    status = "closed_tp"
                elif low <= trade.stop_loss:
                    exit_price = trade.stop_loss
                    status = "closed_sl"
            else:
                if low <= trade.take_profit:
                    exit_price = trade.take_profit
                    status = "closed_tp"
                elif high >= trade.stop_loss:
                    exit_price = trade.stop_loss
                    status = "closed_sl"

            if exit_price is None and status == "closed_eod":
                exit_price = close
            if exit_price is None:
                continue

            trade.exit_time = timestamp
            trade.exit_price = float(exit_price)
            trade.status = status
            gross = (trade.exit_price - trade.entry_price) * trade.position_size * trade.direction
            fee = (trade.position_size * trade.exit_price) * self.commission_pct
            trade.commission_paid += fee
            trade.pnl = gross - trade.commission_paid
            trade.pnl_pct = (trade.pnl / max(trade.entry_price * trade.position_size, 1e-9)) * 100.0
            self.capital += trade.pnl
            closed.append(trade)
            del self.open_trades[symbol]
        return closed

    def _mark_to_market_equity(self, cycle_data: dict[str, pd.DataFrame]) -> float:
        equity = float(self.capital)
        for symbol, trade in self.open_trades.items():
            df = cycle_data.get(symbol)
            if df is None or df.empty:
                price = self.last_symbol_prices.get(symbol, trade.entry_price)
            else:
                price = float(df["close"].iloc[-1])
            unreal = (price - trade.entry_price) * trade.position_size * trade.direction
            equity += unreal
        return equity

    def _is_end_of_day(self, timestamp: pd.Timestamp) -> bool:
        # Cierre diario aproximado para series UTC.
        return int(timestamp.hour) in {21, 22, 23}

    def simulate_trading_cycles(self, data: dict[str, pd.DataFrame] | None = None) -> None:
        """Ejecuta simulacion barra por barra."""
        use_data = data or self.data
        if not use_data:
            raise ValueError("No hay data cargada para simular")
        self.trades = []
        self.open_trades = {}
        self.equity_curve = []
        self.capital = float(self.initial_capital)

        all_ts = sorted(set().union(*[df.index for df in use_data.values()]))
        for ts in all_ts:
            timestamp = pd.Timestamp(ts)
            cycle_data = self._extract_cycle_snapshot(use_data, timestamp)
            if not cycle_data:
                continue
            closed = self._evaluate_open_trades(timestamp, cycle_data)
            self.trades.extend(closed)

            candidates = self._rank_cycle_candidates(cycle_data)
            for candidate in candidates:
                if len(self.open_trades) >= self.max_trades_per_cycle:
                    break
                symbol = str(candidate["symbol"])
                if symbol in self.open_trades:
                    continue
                try:
                    trade = self._execute_entry(candidate, timestamp)
                except Exception:
                    continue
                self.open_trades[symbol] = trade

            equity = self._mark_to_market_equity(cycle_data)
            self.equity_curve.append({"timestamp": timestamp, "equity": equity})

        # Cierra remanentes al ultimo close conocido.
        if all_ts:
            final_ts = pd.Timestamp(all_ts[-1])
            tail_data = self._extract_cycle_snapshot(use_data, final_ts)
            self.trades.extend(self._evaluate_open_trades(final_ts, tail_data))
            if self.equity_curve:
                self.equity_curve[-1]["equity"] = self._mark_to_market_equity(tail_data)

    def _calculate_daily_returns(self) -> np.ndarray:
        if not self.equity_curve:
            return np.array([], dtype=float)
        eq = pd.DataFrame(self.equity_curve).set_index("timestamp")
        daily = eq["equity"].resample("1D").last().dropna()
        if len(daily) < 2:
            return np.array([], dtype=float)
        return daily.pct_change().dropna().values

    def _calculate_sharpe(self, returns: np.ndarray, risk_free_rate: float = 0.02) -> float:
        if len(returns) < 2:
            return 0.0
        std = float(np.std(returns))
        if std <= 1e-12:
            return 0.0
        excess = returns - (risk_free_rate / 252.0)
        return float(np.mean(excess) / std * math.sqrt(252))

    def _calculate_max_drawdown(self) -> float:
        if not self.equity_curve:
            return 0.0
        equity = np.array([float(x["equity"]) for x in self.equity_curve], dtype=float)
        peak = np.maximum.accumulate(equity)
        dd = (equity - peak) / np.maximum(peak, 1e-9) * 100.0
        return float(np.min(dd))

    def _max_consecutive(self, wins: bool) -> int:
        max_streak = 0
        streak = 0
        for t in self.trades:
            cond = t.pnl > 0 if wins else t.pnl <= 0
            if cond:
                streak += 1
                max_streak = max(max_streak, streak)
            else:
                streak = 0
        return max_streak

    def calculate_metrics(self) -> dict[str, Any]:
        if not self.trades:
            self.metrics = {"error": "No trades executed", "total_trades": 0}
            return self.metrics

        rows = [asdict(t) for t in self.trades]
        df = pd.DataFrame(rows)
        wins = df[df["pnl"] > 0]
        losses = df[df["pnl"] <= 0]
        total = len(df)
        win_rate = (len(wins) / total) * 100.0
        gp = float(wins["pnl"].sum()) if not wins.empty else 0.0
        gl = abs(float(losses["pnl"].sum())) if not losses.empty else 0.0
        profit_factor = gp / gl if gl > 0 else float("inf")
        daily_returns = self._calculate_daily_returns()
        sharpe = self._calculate_sharpe(daily_returns)
        max_dd = self._calculate_max_drawdown()
        final_equity = float(self.equity_curve[-1]["equity"]) if self.equity_curve else self.capital
        total_return = ((final_equity - self.initial_capital) / self.initial_capital) * 100.0

        self.metrics = {
            "total_trades": int(total),
            "winning_trades": int(len(wins)),
            "losing_trades": int(len(losses)),
            "win_rate_pct": float(win_rate),
            "profit_factor": float(profit_factor if math.isfinite(profit_factor) else 999.0),
            "sharpe_ratio": float(sharpe),
            "max_drawdown_pct": float(max_dd),
            "total_return_pct": float(total_return),
            "avg_win_pct": float(wins["pnl_pct"].mean()) if not wins.empty else 0.0,
            "avg_loss_pct": float(losses["pnl_pct"].mean()) if not losses.empty else 0.0,
            "best_trade_pct": float(df["pnl_pct"].max()),
            "worst_trade_pct": float(df["pnl_pct"].min()),
            "consecutive_wins": self._max_consecutive(wins=True),
            "consecutive_losses": self._max_consecutive(wins=False),
            "equity_final": final_equity,
        }
        return self.metrics

    def _meets_go_live_criteria(self, metrics: dict[str, Any]) -> tuple[bool, list[str]]:
        reasons: list[str] = []
        if float(metrics.get("win_rate_pct", 0.0)) < self.CRITERIA["win_rate_pct"]:
            reasons.append("Win rate < 58%")
        if float(metrics.get("sharpe_ratio", 0.0)) < self.CRITERIA["sharpe_ratio"]:
            reasons.append("Sharpe < 1.0")
        if float(metrics.get("max_drawdown_pct", 0.0)) < self.CRITERIA["max_drawdown_pct"]:
            reasons.append("Max DD peor que -15%")
        if float(metrics.get("profit_factor", 0.0)) < self.CRITERIA["profit_factor"]:
            reasons.append("Profit factor < 1.25")
        if int(metrics.get("consecutive_losses", 0)) > self.CRITERIA["consecutive_losses_max"]:
            reasons.append("Consecutive losses >= 5")
        if int(metrics.get("total_trades", 0)) < self.CRITERIA["total_trades"]:
            reasons.append("Trade count < 30")
        return len(reasons) == 0, reasons

    def evaluate_go_live(self) -> BacktestDecision:
        metrics = self.metrics or self.calculate_metrics()
        ok, reasons = self._meets_go_live_criteria(metrics)
        return BacktestDecision(go_live_approved=ok, reasons=reasons, metrics=metrics)

    def _generate_combinations(self, search_space: dict[str, list[Any]]) -> list[dict[str, Any]]:
        keys = list(search_space.keys())
        combos = []
        for values in itertools.product(*[search_space[k] for k in keys]):
            combos.append(dict(zip(keys, values)))
        return combos

    def _calculate_fitness_score(self, metrics: dict[str, Any], target_wr: float) -> float:
        wr = float(metrics.get("win_rate_pct", 0.0)) / 100.0
        sharpe = max(0.0, float(metrics.get("sharpe_ratio", 0.0)))
        pf = min(float(metrics.get("profit_factor", 1.0)), 3.0) / 3.0
        dd = abs(float(metrics.get("max_drawdown_pct", 0.0))) / 100.0
        score = (0.4 * wr) + (0.3 * sharpe) + (0.2 * pf) - (0.1 * dd)
        if wr < target_wr:
            score *= 0.5
        return float(score)

    def search_optimal_parameters(self, target_win_rate: float = 0.58, max_iterations: int = 3) -> dict[str, Any] | None:
        if not self.data:
            raise ValueError("No hay data para parameter search")
        base_data = self.data
        search_space = {
            "signal_strength_threshold": [0.52, 0.55, 0.58, 0.62],
            "risk_per_trade": [0.01, 0.015, 0.02, 0.025],
            "max_trades_per_cycle": [3, 4, 5, 6],
            "reward_to_risk": [1.5, 2.0, 2.5],
        }
        combos = self._generate_combinations(search_space)
        best: dict[str, Any] | None = None
        best_score = float("-inf")

        for i, combo in enumerate(combos):
            if i >= (max_iterations * 40):
                break
            self.signal_strength_threshold = float(combo["signal_strength_threshold"])
            self.risk_per_trade = float(combo["risk_per_trade"])
            self.max_trades_per_cycle = int(combo["max_trades_per_cycle"])
            self.reward_to_risk = float(combo["reward_to_risk"])
            self.simulate_trading_cycles(base_data)
            metrics = self.calculate_metrics()
            score = self._calculate_fitness_score(metrics, target_win_rate)
            row = {"combo": combo, "metrics": metrics, "score": score}
            self.parameter_search_history.append(row)
            if score > best_score:
                best = row
                best_score = score

        if best:
            combo = best["combo"]
            self.signal_strength_threshold = float(combo["signal_strength_threshold"])
            self.risk_per_trade = float(combo["risk_per_trade"])
            self.max_trades_per_cycle = int(combo["max_trades_per_cycle"])
            self.reward_to_risk = float(combo["reward_to_risk"])
        return best

    def generate_backtest_report(self, output_path: str | None = None) -> str:
        metrics = self.metrics or self.calculate_metrics()
        decision = self.evaluate_go_live()
        ts = datetime.now(timezone.utc).isoformat()

        def _cls(ok: bool) -> str:
            return "pass" if ok else "fail"

        wr_ok = float(metrics.get("win_rate_pct", 0.0)) >= self.CRITERIA["win_rate_pct"]
        sh_ok = float(metrics.get("sharpe_ratio", 0.0)) >= self.CRITERIA["sharpe_ratio"]
        dd_ok = float(metrics.get("max_drawdown_pct", 0.0)) >= self.CRITERIA["max_drawdown_pct"]
        pf_ok = float(metrics.get("profit_factor", 0.0)) >= self.CRITERIA["profit_factor"]
        html = f"""
<html><head><meta charset="utf-8"><title>ATLAS Backtest Report</title>
<style>
body {{ font-family: Arial, sans-serif; margin: 20px; }}
.metric {{ display: inline-block; margin: 8px; padding: 10px; border: 1px solid #bbb; border-radius: 8px; }}
.pass {{ background: #d9f7d9; }}
.fail {{ background: #ffe0e0; }}
table {{ border-collapse: collapse; width: 100%; }}
th, td {{ border: 1px solid #ddd; padding: 6px; text-align: right; }}
th {{ background: #f5f5f5; }}
</style></head><body>
<h1>ATLAS CODE QUANT - Backtest Report</h1>
<p>Generated at: {ts}</p>
<p>Period: {self.start_date} to {self.end_date}</p>
<h2>Summary Metrics</h2>
<div class="metric {_cls(wr_ok)}"><b>Win Rate</b>: {metrics.get('win_rate_pct', 0):.2f}%</div>
<div class="metric {_cls(sh_ok)}"><b>Sharpe</b>: {metrics.get('sharpe_ratio', 0):.2f}</div>
<div class="metric {_cls(dd_ok)}"><b>Max DD</b>: {metrics.get('max_drawdown_pct', 0):.2f}%</div>
<div class="metric {_cls(pf_ok)}"><b>Profit Factor</b>: {metrics.get('profit_factor', 0):.2f}</div>
<h2>Decision: {'GO-LIVE PHASE 2 APPROVED' if decision.go_live_approved else 'PARAMETER SEARCH REQUIRED'}</h2>
<p>Reasons: {', '.join(decision.reasons) if decision.reasons else 'All criteria met'}</p>
<h2>Trades ({len(self.trades)})</h2>
<table><tr><th>Symbol</th><th>Dir</th><th>Entry</th><th>Exit</th><th>PnL</th><th>PnL %</th><th>Status</th></tr>
"""
        for t in self.trades:
            html += (
                "<tr>"
                f"<td>{t.symbol}</td><td>{'LONG' if t.direction > 0 else 'SHORT'}</td>"
                f"<td>{t.entry_price:.4f}</td><td>{t.exit_price:.4f}</td>"
                f"<td>{t.pnl:.2f}</td><td>{t.pnl_pct:.2f}</td><td>{t.status}</td>"
                "</tr>"
            )
        html += "</table></body></html>"

        self.reports_dir.mkdir(parents=True, exist_ok=True)
        out = Path(output_path) if output_path else (self.reports_dir / f"backtest_{self.end_date}.html")
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(html, encoding="utf-8")

        # Export auxiliar para trazabilidad.
        trades_csv = self.data_dir / "backtest_results" / f"trades_{self.end_date}.csv"
        trades_csv.parent.mkdir(parents=True, exist_ok=True)
        pd.DataFrame([asdict(t) for t in self.trades]).to_csv(trades_csv, index=False)

        metrics_json = self.reports_dir / f"backtest_metrics_{self.end_date}.json"
        metrics_json.write_text(json.dumps(self.metrics, indent=2, default=str), encoding="utf-8")
        return str(out)

    def run(self, symbols: list[str], timeframe: str = "1h") -> dict[str, Any]:
        """Flujo completo: load -> simulate -> metrics -> decision -> (optional search)."""
        self.load_historical_data(symbols=symbols, timeframe=timeframe)
        self.simulate_trading_cycles(self.data)
        metrics = self.calculate_metrics()
        decision = self.evaluate_go_live()
        best_result = None
        if (not decision.go_live_approved) and self.search_mode:
            best_result = self.search_optimal_parameters(target_win_rate=0.58, max_iterations=3)
            self.simulate_trading_cycles(self.data)
            metrics = self.calculate_metrics()
            decision = self.evaluate_go_live()
        return {
            "metrics": metrics,
            "decision": asdict(decision),
            "best_result": best_result,
            "trades_count": len(self.trades),
        }

