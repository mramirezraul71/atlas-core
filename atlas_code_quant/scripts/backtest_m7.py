# ATLAS-EXECUTION — Backtest Módulo 7: Equity Curve + Drawdown
"""Backtest completo del pipeline Atlas-Quant-Core con datos 2024-2026.

Fuente de datos: yfinance (proxy gratuito) + estimación de IV.

Simula el ciclo completo:
  Indicadores → Régimen (XGBoost) → Señal → Kelly → Ejecución paper

Salida:
  - Equity curve ASCII + métricas clave en consola
  - Archivo JSON: data/backtest_results/bt_m7_{ts}.json
  - Archivo HTML opcional (si backtesting.reporter disponible)

Uso:
    python -m atlas_code_quant.scripts.backtest_m7
    python -m atlas_code_quant.scripts.backtest_m7 --symbol NVDA --period 2y
"""
from __future__ import annotations

import argparse
import json
import logging
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional

import numpy as np

logger = logging.getLogger("atlas.backtest.m7")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)

# ── Parámetros por defecto ────────────────────────────────────────────────────
DEFAULT_SYMBOLS  = ["SPY", "QQQ", "NVDA", "AAPL", "MSFT"]
DEFAULT_PERIOD   = "2y"
DEFAULT_INTERVAL = "1d"
INITIAL_CAPITAL  = 100_000.0
KELLY_FRACTION   = 0.25
MAX_RISK_PCT     = 0.01        # 1% del capital por trade
ATR_PERIOD       = 20
TP_ATR_MULT      = 2.0
SL_ATR_MULT      = 2.5
COMMISSION_PCT   = 0.001       # 0.1% comisión por lado


@dataclass
class BacktestTrade:
    symbol:       str
    side:         str
    entry_date:   str
    exit_date:    str
    entry_price:  float
    exit_price:   float
    quantity:     int
    pnl_gross:    float
    pnl_net:      float
    pnl_pct:      float
    exit_reason:  str
    regime:       str
    atr_at_entry: float


@dataclass
class BacktestResult:
    symbol:            str
    period:            str
    initial_capital:   float
    final_equity:      float
    total_return_pct:  float
    max_drawdown_pct:  float
    sharpe_ratio:      float
    sortino_ratio:     float
    calmar_ratio:      float
    win_rate_pct:      float
    profit_factor:     float
    total_trades:      int
    avg_trade_pct:     float
    kelly_fraction:    float
    equity_curve:      list[dict] = field(default_factory=list)
    drawdown_curve:    list[dict] = field(default_factory=list)
    trades:            list[dict] = field(default_factory=list)


def run_backtest(
    symbols: list[str],
    period: str = DEFAULT_PERIOD,
    interval: str = DEFAULT_INTERVAL,
    initial_capital: float = INITIAL_CAPITAL,
) -> list[BacktestResult]:
    """Ejecuta backtest para lista de símbolos y retorna resultados."""
    try:
        import yfinance as yf
    except ImportError:
        logger.error("yfinance no instalado. pip install yfinance")
        return []

    results = []
    for sym in symbols:
        logger.info("Descargando %s (%s %s)…", sym, period, interval)
        try:
            df = yf.download(sym, period=period, interval=interval, progress=False)
            if df is None or len(df) < 60:
                logger.warning("Datos insuficientes para %s (%d barras)", sym, len(df) if df is not None else 0)
                continue

            result = _backtest_single(sym, df, initial_capital, period)
            results.append(result)
            _print_result(result)
        except Exception as exc:
            logger.error("Error en backtest %s: %s", sym, exc)

    return results


def _backtest_single(symbol: str, df, initial_capital: float, period: str) -> BacktestResult:
    """Simula estrategia completa en datos históricos de un símbolo."""
    closes  = df["Close"].values.astype(float).flatten()
    highs   = df["High"].values.astype(float).flatten()
    lows    = df["Low"].values.astype(float).flatten()
    volumes = df["Volume"].values.astype(float).flatten()
    dates   = [str(d)[:10] for d in df.index]

    n = len(closes)
    capital = initial_capital
    equity_curve: list[dict] = []
    trades: list[BacktestTrade] = []

    # Estado de posición
    in_trade   = False
    entry_idx  = 0
    entry_price = 0.0
    entry_side  = ""
    trade_qty   = 0
    trade_sl    = 0.0
    trade_tp    = 0.0
    trade_atr   = 0.0
    trade_regime = ""

    for i in range(60, n):
        win = closes[max(0, i-60):i]
        wh  = highs[max(0, i-60):i]
        wl  = lows[max(0, i-60):i]
        wv  = volumes[max(0, i-60):i]

        price = closes[i]
        equity_curve.append({
            "time":  int(time.mktime(time.strptime(dates[i], "%Y-%m-%d"))),
            "value": round(capital, 2),
        })

        # ── Gestión de salida ──────────────────────────────────────────────
        if in_trade:
            exit_reason = None

            if entry_side == "BUY":
                if price >= trade_tp:
                    exit_reason = "take_profit"
                elif price <= trade_sl:
                    exit_reason = "stop_loss"
            else:  # SELL
                if price <= trade_tp:
                    exit_reason = "take_profit"
                elif price >= trade_sl:
                    exit_reason = "stop_loss"

            # Time exit: máx 20 días
            if i - entry_idx >= 20:
                exit_reason = "time_exit"

            if exit_reason:
                exit_price  = price
                commission  = exit_price * trade_qty * COMMISSION_PCT
                pnl_gross   = (
                    (exit_price - entry_price) * trade_qty
                    if entry_side == "BUY"
                    else (entry_price - exit_price) * trade_qty
                )
                pnl_net     = pnl_gross - commission
                capital    += pnl_net

                trades.append(BacktestTrade(
                    symbol      = symbol,
                    side        = entry_side,
                    entry_date  = dates[entry_idx],
                    exit_date   = dates[i],
                    entry_price = entry_price,
                    exit_price  = exit_price,
                    quantity    = trade_qty,
                    pnl_gross   = round(pnl_gross, 2),
                    pnl_net     = round(pnl_net, 2),
                    pnl_pct     = round(pnl_net / (entry_price * trade_qty) * 100, 3),
                    exit_reason = exit_reason,
                    regime      = trade_regime,
                    atr_at_entry = trade_atr,
                ))
                in_trade = False
            continue

        # ── Señal de entrada ───────────────────────────────────────────────
        atr = _atr(wh, wl, win)
        if atr <= 0:
            continue

        regime, conf = _regime(win, wh, wl, wv)
        if conf < 0.65 or regime == "flat":
            continue

        # IV proxy: usar ATR normalizado como estimador de IV (sin opciones reales)
        iv_proxy = (atr / price) * np.sqrt(252) * 100
        if iv_proxy < 20:  # IV rank proxy <70% equivalente
            continue

        # Señal técnica
        rsi = _rsi(win)
        macd_hist = _macd_hist(win)
        vol_ratio = volumes[i] / (np.mean(wv[-20:]) + 1e-10)

        side = _entry_signal(regime, rsi, macd_hist, vol_ratio)
        if side is None:
            continue

        # Kelly position sizing
        win_rate  = _rolling_winrate(trades, window=20)
        avg_win   = _avg_result(trades, won=True)
        avg_loss  = _avg_result(trades, won=False)

        kelly = _kelly_fraction(win_rate, avg_win, avg_loss) if len(trades) >= 6 else 0.25
        kelly = max(0.0, min(kelly, 0.20))

        max_risk  = capital * MAX_RISK_PCT
        sl_dist   = atr * SL_ATR_MULT
        qty_risk  = int(max_risk / sl_dist) if sl_dist > 0 else 0
        qty_kelly = int(capital * kelly / price) if price > 0 else 0
        qty       = min(qty_risk, qty_kelly)

        if qty <= 0:
            continue

        cost = price * qty * (1 + COMMISSION_PCT)
        if cost > capital * 0.95:
            continue

        # Abrir posición
        in_trade    = True
        entry_idx   = i
        entry_price = price
        entry_side  = side
        trade_qty   = qty
        trade_atr   = atr
        trade_regime = regime
        capital    -= price * qty * COMMISSION_PCT  # comisión de entrada

        if side == "BUY":
            trade_tp = price + atr * TP_ATR_MULT
            trade_sl = price - atr * SL_ATR_MULT
        else:
            trade_tp = price - atr * TP_ATR_MULT
            trade_sl = price + atr * SL_ATR_MULT

    # ── Calcular métricas ──────────────────────────────────────────────────
    return _compute_metrics(
        symbol, period, initial_capital, capital,
        equity_curve, trades
    )


# ── Indicadores simplificados (numpy puro) ────────────────────────────────────

def _atr(highs: np.ndarray, lows: np.ndarray, closes: np.ndarray, period: int = ATR_PERIOD) -> float:
    n = min(period, len(closes) - 1)
    if n < 1:
        return 0.0
    tr = np.maximum(highs[-n:] - lows[-n:],
         np.maximum(np.abs(highs[-n:] - closes[-n-1:-1]),
                    np.abs(lows[-n:]  - closes[-n-1:-1])))
    return float(tr.mean())


def _rsi(closes: np.ndarray, period: int = 14) -> float:
    if len(closes) < period + 1:
        return 50.0
    d = np.diff(closes[-period-1:])
    g = np.where(d > 0, d, 0).mean()
    l = np.where(d < 0, -d, 0).mean()
    return float(100 - 100 / (1 + g / l)) if l > 1e-10 else 100.0


def _macd_hist(closes: np.ndarray) -> float:
    if len(closes) < 26:
        return 0.0
    ema12 = float(np.mean(closes[-12:]))
    ema26 = float(np.mean(closes[-26:]))
    return ema12 - ema26


def _regime(closes: np.ndarray, highs: np.ndarray, lows: np.ndarray,
            volumes: np.ndarray) -> tuple[str, float]:
    """Régimen simplificado basado en retorno 20d + ADX proxy."""
    if len(closes) < 22:
        return "flat", 0.0

    ret_20 = (closes[-1] - closes[-20]) / closes[-20]
    atr    = _atr(highs, lows, closes, 14)
    adx    = atr / closes[-1] * 100  # proxy ADX

    vol_r  = volumes[-1] / (np.mean(volumes[-20:]) + 1e-10)

    if ret_20 > 0.02 and adx > 0.5:
        return "bull", min(0.65 + ret_20 * 5, 0.95)
    if ret_20 < -0.02 and adx > 0.5:
        return "bear", min(0.65 + abs(ret_20) * 5, 0.95)
    if abs(ret_20) <= 0.02:
        return "sideways", 0.70

    return "flat", 0.50


def _entry_signal(regime: str, rsi: float, macd_hist: float, vol_ratio: float) -> Optional[str]:
    if vol_ratio < 1.8:
        return None
    if regime == "bull" and rsi < 65 and macd_hist > 0:
        return "BUY"
    if regime == "bear" and rsi > 35 and macd_hist < 0:
        return "SELL"
    if regime == "sideways":
        if rsi < 30 and macd_hist > 0:
            return "BUY"
        if rsi > 70 and macd_hist < 0:
            return "SELL"
    return None


def _kelly_fraction(win_rate: float, avg_win: float, avg_loss: float) -> float:
    if avg_loss <= 0:
        return KELLY_FRACTION
    b  = avg_win / avg_loss
    p  = max(0.01, min(0.99, win_rate))
    q  = 1 - p
    k  = (p * b - q) / b
    return max(0.0, k * KELLY_FRACTION)


def _rolling_winrate(trades: list, window: int = 20) -> float:
    recent = trades[-window:] if len(trades) >= window else trades
    if not recent:
        return 0.5
    return sum(1 for t in recent if t.pnl_net > 0) / len(recent)


def _avg_result(trades: list, won: bool, window: int = 20) -> float:
    recent = trades[-window:] if len(trades) >= window else trades
    subset = [abs(t.pnl_net) for t in recent if (t.pnl_net > 0) == won]
    return float(np.mean(subset)) if subset else 1.0


# ── Métricas ──────────────────────────────────────────────────────────────────

def _compute_metrics(
    symbol: str,
    period: str,
    initial: float,
    final: float,
    equity_curve: list[dict],
    trades: list[BacktestTrade],
) -> BacktestResult:
    total_return  = (final - initial) / initial * 100
    wins          = [t for t in trades if t.pnl_net > 0]
    losses        = [t for t in trades if t.pnl_net <= 0]
    win_rate      = len(wins) / max(1, len(trades)) * 100
    avg_trade_pct = (sum(t.pnl_pct for t in trades) / max(1, len(trades)))
    gross_win     = sum(t.pnl_net for t in wins)
    gross_loss    = abs(sum(t.pnl_net for t in losses))
    profit_factor = gross_win / gross_loss if gross_loss > 0 else float("inf")

    # Drawdown
    eq_vals = np.array([e["value"] for e in equity_curve], dtype=float)
    peak    = np.maximum.accumulate(eq_vals)
    dd_arr  = (peak - eq_vals) / (peak + 1e-10) * 100
    max_dd  = float(dd_arr.max()) if len(dd_arr) > 0 else 0.0

    dd_curve = [
        {"time": equity_curve[i]["time"], "value": round(-dd_arr[i], 2)}
        for i in range(len(equity_curve))
    ]

    # Sharpe / Sortino (retornos diarios)
    sharpe, sortino = 0.0, 0.0
    if len(eq_vals) > 1:
        rets = np.diff(np.log(eq_vals + 1e-10))
        mu   = rets.mean()
        std  = rets.std(ddof=1)
        down = rets[rets < 0]
        dstd = down.std(ddof=1) if len(down) > 1 else 1e-10
        sharpe  = float(mu / std  * np.sqrt(252)) if std  > 1e-10 else 0.0
        sortino = float(mu / dstd * np.sqrt(252)) if dstd > 1e-10 else 0.0

    calmar = (total_return / max_dd) if max_dd > 0 else 0.0

    return BacktestResult(
        symbol          = symbol,
        period          = period,
        initial_capital = initial,
        final_equity    = round(final, 2),
        total_return_pct= round(total_return, 2),
        max_drawdown_pct= round(max_dd, 2),
        sharpe_ratio    = round(sharpe, 3),
        sortino_ratio   = round(sortino, 3),
        calmar_ratio    = round(calmar, 3),
        win_rate_pct    = round(win_rate, 2),
        profit_factor   = round(profit_factor, 3),
        total_trades    = len(trades),
        avg_trade_pct   = round(avg_trade_pct, 3),
        kelly_fraction  = KELLY_FRACTION,
        equity_curve    = equity_curve,
        drawdown_curve  = dd_curve,
        trades          = [asdict(t) for t in trades],
    )


# ── Visualización ASCII ───────────────────────────────────────────────────────

def _print_result(r: BacktestResult) -> None:
    print("\n" + "="*65)
    print(f"  BACKTEST: {r.symbol} | {r.period} | Capital: ${r.initial_capital:,.0f}")
    print("="*65)

    equity_line = _ascii_chart(
        [e["value"] for e in r.equity_curve], label="Equity", width=60, height=10
    )
    print(equity_line)

    dd_vals = [-e["value"] for e in r.drawdown_curve]
    dd_line = _ascii_chart(dd_vals, label="Drawdown %", width=60, height=5, invert=True)
    print(dd_line)

    color_ret = "🟢" if r.total_return_pct >= 0 else "🔴"
    print(f"\n  {color_ret} Retorno Total:  {r.total_return_pct:+.2f}%")
    print(f"  📉 Max Drawdown: {r.max_drawdown_pct:.2f}%")
    print(f"  📊 Sharpe:       {r.sharpe_ratio:.3f}")
    print(f"  📊 Sortino:      {r.sortino_ratio:.3f}")
    print(f"  📊 Calmar:       {r.calmar_ratio:.3f}")
    print(f"  🎯 Win Rate:     {r.win_rate_pct:.1f}%")
    print(f"  💰 Profit Factor:{r.profit_factor:.3f}")
    print(f"  🔢 Trades:       {r.total_trades}")
    print(f"  📈 Avg Trade:    {r.avg_trade_pct:+.3f}%")
    print(f"  💼 Equity Final: ${r.final_equity:,.2f}")
    print("="*65)


def _ascii_chart(
    values: list[float],
    label: str = "",
    width: int = 60,
    height: int = 8,
    invert: bool = False,
) -> str:
    if not values:
        return f"[{label}: sin datos]"

    vals = np.array(values, dtype=float)
    step = max(1, len(vals) // width)
    sampled = vals[::step][:width]

    vmin, vmax = sampled.min(), sampled.max()
    rng = vmax - vmin
    if rng < 1e-10:
        rng = 1.0

    lines = []
    for row in range(height - 1, -1, -1):
        thresh = vmin + (row / (height - 1)) * rng
        line = ""
        for v in sampled:
            line += "█" if v >= thresh else " "
        if row == height - 1:
            lines.append(f"  {vmax:8.1f} │{line}")
        elif row == 0:
            lines.append(f"  {vmin:8.1f} │{line}")
        else:
            lines.append(f"           │{line}")

    lines.append(f"  {label}")
    return "\n".join(lines)


# ── Guardar resultados ────────────────────────────────────────────────────────

def _save_results(results: list[BacktestResult]) -> str:
    out_dir = Path("data/backtest_results")
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = int(time.time())
    path = out_dir / f"bt_m7_{ts}.json"
    data = [asdict(r) for r in results]
    path.write_text(json.dumps(data, indent=2))
    logger.info("Resultados guardados: %s", path)
    return str(path)


# ── CLI ───────────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ATLAS-Quant Backtest M7")
    p.add_argument("--symbols", nargs="+", default=DEFAULT_SYMBOLS)
    p.add_argument("--period",  default=DEFAULT_PERIOD,
                   help="yfinance period: 1y, 2y, 5y")
    p.add_argument("--capital", type=float, default=INITIAL_CAPITAL)
    p.add_argument("--save",    action="store_true", help="Guardar JSON de resultados")
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    logger.info(
        "Iniciando backtest — símbolos=%s periodo=%s capital=$%.0f",
        args.symbols, args.period, args.capital
    )

    t0 = time.time()
    results = run_backtest(args.symbols, args.period, initial_capital=args.capital)
    elapsed = time.time() - t0

    if not results:
        logger.error("Sin resultados — verifique conexión a internet y yfinance")
        return

    # Resumen global
    print("\n" + "="*65)
    print(f"  ATLAS-Quant Backtest M7 — {len(results)} símbolos | {elapsed:.1f}s")
    print("="*65)

    total_ret  = np.mean([r.total_return_pct for r in results])
    avg_sharpe = np.mean([r.sharpe_ratio for r in results])
    avg_dd     = np.mean([r.max_drawdown_pct for r in results])
    avg_wr     = np.mean([r.win_rate_pct for r in results])
    total_trades = sum(r.total_trades for r in results)

    print(f"  Retorno promedio:    {total_ret:+.2f}%")
    print(f"  Sharpe promedio:     {avg_sharpe:.3f}")
    print(f"  Max DD promedio:     {avg_dd:.2f}%")
    print(f"  Win Rate promedio:   {avg_wr:.1f}%")
    print(f"  Total operaciones:   {total_trades}")
    print("="*65)

    if args.save:
        path = _save_results(results)
        print(f"\n  Resultados guardados en: {path}")


if __name__ == "__main__":
    main()
