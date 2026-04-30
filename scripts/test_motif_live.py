"""
test_motif_live.py — Verifica que MotifLabService detecta motifs en tiempo real.

Uso:
    python scripts/test_motif_live.py SPY 5m
    python scripts/test_motif_live.py QQQ 1m --window 30 --horizon 10 --top-k 15
    python scripts/test_motif_live.py SPY 5m --sim-ticks        # simula 3 ticks nuevos

Flujo:
    1. Descarga histórico (yfinance) → fit() → SQLite
    2. Simula ventana "en tiempo real" (últimas window_size barras)
    3. find_motifs() + get_forecast_dist() + cronometrado
    4. Simula N ticks adicionales (--sim-ticks) y re-evalúa
    5. Imprime tabla de resultados + veredicto PASS/FAIL
"""
from __future__ import annotations

import argparse
import sys
import time
import warnings
from pathlib import Path

warnings.filterwarnings("ignore")

# ── Path setup ────────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "atlas_code_quant"))

import numpy as np
import pandas as pd

try:
    import yfinance as yf
except ImportError:
    print("ERROR: yfinance no instalado. pip install yfinance")
    sys.exit(1)

from learning.motif_lab import MotifLabService

# ── Constantes de éxito ───────────────────────────────────────────────────────
MIN_MATCHES        = 1       # al menos 1 motif para PASS
MAX_LATENCY_MS     = 200     # find_motifs debe completar en <200ms
MIN_INDEXED        = 100     # al menos 100 ventanas indexadas
MIN_HIST_BARS      = 200     # mínimo de barras históricas para fit

# ── Colores ANSI ──────────────────────────────────────────────────────────────
GRN = "\033[92m"
RED = "\033[91m"
YLW = "\033[93m"
BLU = "\033[94m"
RST = "\033[0m"
BOLD= "\033[1m"

def ok(msg):  return f"{GRN}[PASS]{RST} {msg}"
def err(msg): return f"{RED}[FAIL]{RST} {msg}"
def warn(msg):return f"{YLW}[WARN]{RST} {msg}"
def hdr(msg): return f"{BOLD}{BLU}{msg}{RST}"


# ── Construcción de features ───────────────────────────────────────────────────

def build_features(df: pd.DataFrame) -> pd.DataFrame:
    """RSI-14, MACD hist, volume_rel, CVD desde OHLCV de yfinance."""
    close  = df["Close"].astype(float)
    volume = df["Volume"].astype(float)

    # RSI
    delta = close.diff()
    gain  = delta.clip(lower=0).rolling(14).mean()
    loss  = (-delta.clip(upper=0)).rolling(14).mean()
    rsi   = 100 - (100 / (1 + gain / (loss + 1e-8)))

    # MACD hist normalizado
    ema12 = close.ewm(span=12, adjust=False).mean()
    ema26 = close.ewm(span=26, adjust=False).mean()
    ml    = ema12 - ema26
    ms    = ml.ewm(span=9, adjust=False).mean()
    macd  = (ml - ms) / (close.abs() + 1e-8) * 100

    # Volume relative
    vol_rel = volume / (volume.rolling(20).mean() + 1e-8)

    # CVD (OBV normalizado)
    obv = (np.sign(close.diff().fillna(0)) * volume).cumsum()
    cvd = (obv - obv.rolling(50).mean()) / (obv.rolling(50).std() + 1e-8)

    feat = pd.DataFrame({
        "close":      close,
        "rsi":        rsi,
        "macd":       macd,
        "volume_rel": vol_rel,
        "cvd":        cvd,
    }, index=df.index).dropna()
    return feat


# ── Runner principal ───────────────────────────────────────────────────────────

def run_test(symbol: str, timeframe: str, window: int, horizon: int,
             top_k: int, threshold: float, sim_ticks: bool,
             db_path: str) -> bool:

    results: list[tuple[str, bool, str]] = []   # (test_name, passed, detail)

    def check(name: str, passed: bool, detail: str = ""):
        results.append((name, passed, detail))
        icon = ok("") if passed else err("")
        print(f"  {icon} {name}" + (f"  — {detail}" if detail else ""))

    print()
    print(hdr("=" * 60))
    print(hdr(f"  Motif Live Test | {symbol} {timeframe} | w={window} h={horizon}"))
    print(hdr("=" * 60))

    # ── 1. Descargar histórico ────────────────────────────────────────────────
    print(f"\n{hdr('[1] Descarga yfinance')}")
    period_map = {"1m": "7d", "2m": "60d", "5m": "60d", "15m": "60d",
                  "30m": "60d", "1h": "730d", "1d": "5y"}
    period = period_map.get(timeframe, "60d")
    t0 = time.time()
    raw = yf.download(symbol, period=period, interval=timeframe,
                      progress=False, auto_adjust=True)
    dl_ms = (time.time() - t0) * 1000

    if isinstance(raw.columns, pd.MultiIndex):
        raw.columns = raw.columns.get_level_values(0)

    n_raw = len(raw)
    check("yfinance descarga", n_raw > 0, f"{n_raw} barras brutas en {dl_ms:.0f}ms")

    if n_raw == 0:
        print(err("Sin datos — abortando test."))
        return False

    # ── 2. Build features ─────────────────────────────────────────────────────
    print(f"\n{hdr('[2] Features')}")
    feat = build_features(raw)
    n_feat = len(feat)
    check("build_features", n_feat >= MIN_HIST_BARS,
          f"{n_feat} barras limpias (min={MIN_HIST_BARS})")
    print(f"     Columnas: {list(feat.columns)}")
    print(f"     Rango: {feat.index[0]} -> {feat.index[-1]}")
    print(f"     Stats close: min={feat['close'].min():.2f}  "
          f"max={feat['close'].max():.2f}  "
          f"last={feat['close'].iloc[-1]:.2f}")

    # ── 3. Fit ────────────────────────────────────────────────────────────────
    print(f"\n{hdr('[3] fit()')}")

    # Separar histórico de ventana "actual" (sin look-ahead)
    hist_df = feat.iloc[:-window] if len(feat) > window * 2 else feat

    lab = MotifLabService(
        db_path=db_path,
        similarity_threshold=threshold,
        trivial_radius=window,
        max_windows=3000,
        min_future_samples=2,
    )
    t0 = time.time()
    lab.fit(symbol, timeframe, window,
            historical_df=hist_df,
            force_refit=True,
            max_future_bars=horizon + 5)
    fit_ms = (time.time() - t0) * 1000

    st = lab.status()
    n_idx = st["n_indexed"]
    check("fit() completado",    n_idx >= MIN_INDEXED,
          f"{n_idx} ventanas en {fit_ms:.0f}ms")
    check("cache cargado",       st["cache_loaded"],
          f"cache_size={st['cache_size']}")
    check("SQLite accesible",    Path(db_path).exists(),
          db_path)

    if n_idx < MIN_INDEXED:
        print(warn(f"Solo {n_idx} ventanas — datos insuficientes para análisis robusto"))

    # ── 4. find_motifs en tiempo real ─────────────────────────────────────────
    print(f"\n{hdr('[4] find_motifs() — tiempo real')}")
    recent = feat.tail(window)

    t0 = time.time()
    matches = lab.find_motifs(recent, top_k=top_k)
    latency_ms = (time.time() - t0) * 1000

    n_matches = len(matches)
    check("Latencia <200ms",  latency_ms < MAX_LATENCY_MS, f"{latency_ms:.1f}ms")
    check("Motifs encontrados", n_matches >= MIN_MATCHES,
          f"{n_matches} matches (threshold={threshold})")

    if matches:
        sims = [m.similarity for m in matches]
        print(f"     sim max/avg/min = {max(sims):.4f} / {sum(sims)/len(sims):.4f} / {min(sims):.4f}")
        print(f"     Top-5 matches:")
        for i, m in enumerate(matches[:5]):
            print(f"       #{i+1}  bar={m.historical_start:5d}-{m.historical_end:5d}"
                  f"  sim={m.similarity:.4f}  close_hist=${m.close_at_end:.2f}")
    else:
        print(warn("0 matches — bajando threshold temporalmente a 0.60..."))
        lab.similarity_threshold = 0.60
        matches2 = lab.find_motifs(recent, top_k=top_k)
        lab.similarity_threshold = threshold
        if matches2:
            print(f"     Con threshold=0.60: {len(matches2)} matches "
                  f"(sim_avg={sum(m.similarity for m in matches2)/len(matches2):.4f})")
            print(warn("Considera bajar --threshold a 0.65 para este símbolo/timeframe"))

    # ── 5. get_forecast_dist ──────────────────────────────────────────────────
    print(f"\n{hdr('[5] get_forecast_dist()')}")
    t0 = time.time()
    dist = lab.get_forecast_dist(matches, horizon=horizon)
    dist_ms = (time.time() - t0) * 1000

    n_samp = dist["n_samples"]
    edge   = dist["edge_score"]
    prob   = dist["prob_positive"]
    mean_r = dist["mean_return"]
    std_r  = dist["std"]
    bias   = ("LONG" if edge > 0.55 else ("SHORT" if edge < 0.45 else "NEUTRAL"))
    bias_c = (GRN if bias == "LONG" else (RED if bias == "SHORT" else YLW))

    check("Distribución con muestras", n_samp > 0,
          f"{n_samp} muestras en {dist_ms:.1f}ms")
    check("edge_score en rango",
          0.0 <= edge <= 1.0, f"{edge:.4f}")
    check("prob_positive en rango",
          0.0 <= prob <= 1.0, f"{prob:.1%}")

    print(f"\n     {BOLD}Forecast horizon={horizon} barras "
          f"({horizon * _tf_minutes(timeframe)}min):{RST}")
    print(f"       n_samples     = {n_samp}")
    print(f"       prob_up       = {prob:.1%}")
    print(f"       mean_return   = {mean_r:+.4%}  (log)")
    print(f"       vol_expected  = {std_r:.4%}")
    print(f"       p05/p50/p95   = {dist['p05']:+.3%} / {dist['p50']:+.3%} / {dist['p95']:+.3%}")
    print(f"       edge_score    = {edge:.4f}")
    print(f"       sesgo         = {bias_c}{BOLD}{bias}{RST}")

    # ── 6. Simulación de ticks nuevos ─────────────────────────────────────────
    if sim_ticks:
        print(f"\n{hdr('[6] Simulacion ticks nuevos (3 actualizaciones)')}")
        current_price = float(feat["close"].iloc[-1])

        for tick_n in range(1, 4):
            # Simular precio con micro-movimiento aleatorio
            pct_chg = np.random.normal(0, 0.0003)
            new_price = current_price * (1 + pct_chg)
            current_price = new_price

            # Crear ventana actualizada (shift + nuevo tick)
            updated = feat.tail(window).copy()
            new_row = updated.iloc[-1].copy()
            new_row["close"] = new_price
            new_row["rsi"]   = float(np.clip(new_row["rsi"] + np.random.normal(0, 0.5), 0, 100))
            new_row["macd"]  = float(new_row["macd"] + np.random.normal(0, 0.001))
            # Shift temporal
            new_idx = updated.index[-1] + pd.Timedelta(minutes=_tf_minutes(timeframe))
            updated = pd.concat([updated.iloc[1:],
                                  pd.DataFrame([new_row], index=[new_idx])])

            t0 = time.time()
            tick_matches = lab.find_motifs(updated, top_k=top_k)
            tick_dist    = lab.get_forecast_dist(tick_matches, horizon=horizon)
            tick_ms      = (time.time() - t0) * 1000

            tick_edge = tick_dist["edge_score"]
            tick_bias = ("LONG" if tick_edge > 0.55
                         else ("SHORT" if tick_edge < 0.45 else "NEUTRAL"))
            tick_c    = (GRN if tick_bias == "LONG"
                         else (RED if tick_bias == "SHORT" else YLW))

            print(f"  Tick #{tick_n}  price=${new_price:.2f}  "
                  f"matches={len(tick_matches)}  "
                  f"edge={tick_edge:.4f}  "
                  f"sesgo={tick_c}{tick_bias}{RST}  "
                  f"latencia={tick_ms:.1f}ms")
            check(f"Tick #{tick_n} latencia <200ms", tick_ms < MAX_LATENCY_MS,
                  f"{tick_ms:.1f}ms")

    # ── 7. Resumen ────────────────────────────────────────────────────────────
    print(f"\n{hdr('=' * 60)}")
    total   = len(results)
    passed  = sum(1 for _, p, _ in results if p)
    failed  = total - passed

    print(f"{BOLD}  Resultado: {passed}/{total} tests pasados{RST}")
    if failed:
        print(f"  {RED}Fallidos:{RST}")
        for name, p, detail in results:
            if not p:
                print(f"    - {name}: {detail}")

    all_passed = failed == 0
    verdict = (f"{GRN}{BOLD}  PASS — MotifLabService funciona correctamente en tiempo real{RST}"
               if all_passed
               else f"{YLW}{BOLD}  PARTIAL — {failed} test(s) fallaron (ver arriba){RST}")
    print(verdict)
    print(hdr("=" * 60))
    return all_passed


def _tf_minutes(tf: str) -> int:
    m = {"1m": 1, "2m": 2, "5m": 5, "15m": 15, "30m": 30, "1h": 60, "1d": 390}
    return m.get(tf, 5)


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Test de MotifLabService en tiempo real.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("symbol",    nargs="?", default="SPY",  help="Ticker")
    parser.add_argument("timeframe", nargs="?", default="5m",   help="Granularidad yfinance")
    parser.add_argument("--window",    type=int,   default=20,   help="Tamaño ventana motif")
    parser.add_argument("--horizon",   type=int,   default=20,   help="Barras futuras forecast")
    parser.add_argument("--top-k",     type=int,   default=20,   help="Top-K matches")
    parser.add_argument("--threshold", type=float, default=0.70, help="Similaridad minima")
    parser.add_argument("--sim-ticks", action="store_true",      help="Simular 3 ticks nuevos")
    parser.add_argument("--db",        default="data/operation/motifs_live_test.db",
                        help="Ruta SQLite")
    args = parser.parse_args()

    # Crear directorio DB si no existe
    Path(args.db).parent.mkdir(parents=True, exist_ok=True)

    ok_ = run_test(
        symbol    = args.symbol.upper(),
        timeframe = args.timeframe,
        window    = args.window,
        horizon   = args.horizon,
        top_k     = args.top_k,
        threshold = args.threshold,
        sim_ticks = args.sim_ticks,
        db_path   = args.db,
    )
    sys.exit(0 if ok_ else 1)


if __name__ == "__main__":
    main()
