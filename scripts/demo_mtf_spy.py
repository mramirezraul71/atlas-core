"""
demo_mtf_spy.py — Ejemplo de Multi-Timeframe Coherence con datos simulados de SPY.

Demuestra:
  1. Generación de precio sintético con régimen cambiante (BULL → SIDE → BEAR → BULL)
  2. MultiTimeframeAnalyzer.analyze() con closes submuestreados a 1m/5m/15m/1h
  3. Tabla de resultados por TF (régimen, strength, confidence)
  4. Evolución del coherence_score a lo largo del tiempo

Uso:
    python scripts/demo_mtf_spy.py
    python scripts/demo_mtf_spy.py --verbose
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

# ── Path setup ────────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "atlas_code_quant"))

from models.regime_classifier import (
    MarketRegime,
    MultiTimeframeAnalyzer,
    RegimeClassifier,
)

# ── ANSI ──────────────────────────────────────────────────────────────────────
GRN  = "\033[92m"
RED  = "\033[91m"
YLW  = "\033[93m"
BLU  = "\033[94m"
CYN  = "\033[96m"
RST  = "\033[0m"
BOLD = "\033[1m"

_REGIME_COLOR = {
    MarketRegime.BULL:     GRN,
    MarketRegime.BEAR:     RED,
    MarketRegime.SIDEWAYS: YLW,
    MarketRegime.FLAT:     "\033[90m",
}


def _rc(regime: MarketRegime) -> str:
    return _REGIME_COLOR.get(regime, RST) + regime.value.upper() + RST


# ── Generación de precio sintético ───────────────────────────────────────────

def _make_spy_closes(n_bars: int = 1200, seed: int = 42) -> np.ndarray:
    """
    Simula ~1200 barras de SPY (aprox. 20h de 1m) con 4 fases:
      [0-300)   BULL     — tendencia alcista sostenida
      [300-600) SIDEWAYS — rango estrecho
      [600-900) BEAR     — tendencia bajista
      [900-)    BULL     — recuperación
    """
    rng = np.random.default_rng(seed)
    prices = [500.0]

    phases = [
        (300, +0.0005, 0.0008),   # BULL:     drift positivo, vol normal
        (300,  0.0000, 0.0005),   # SIDEWAYS: sin drift, vol baja
        (300, -0.0006, 0.0009),   # BEAR:     drift negativo
        (300, +0.0004, 0.0007),   # BULL 2:   recuperación
    ]
    for length, drift, vol in phases:
        for _ in range(length):
            ret = rng.normal(drift, vol)
            prices.append(prices[-1] * (1 + ret))

    return np.array(prices[:n_bars], dtype=np.float64)


# ── Mock RegimeClassifier (sin XGBoost entrenado) ────────────────────────────

class _MockRegimeClassifier(RegimeClassifier):
    """Clasifica régimen con reglas simples a partir de features escalares.

    Reemplaza predict() para no requerir XGBoost entrenado.
    Lógica:
      - return_20d > +1.5% y RSI > 55  → BULL
      - return_20d < -1.5% y RSI < 45  → BEAR
      - else                            → SIDEWAYS
    La confidence está basada en la magnitud del retorno.
    """

    def __init__(self) -> None:
        super().__init__(use_gpu=False)
        # No cargar ni entrenar modelos reales
        self._xgb_model = None
        self._lstm      = None

    def predict(self, features: np.ndarray, sequence=None):
        from models.regime_classifier import RegimeOutput

        # features: [[adx, return_20d, hurst, iv_rank, cvd_slope_5m,
        #              rsi_14, macd_hist, atr_norm, volume_ratio]]
        row        = features[0]
        ret_20d    = float(row[1])
        rsi        = float(row[5])
        adx        = float(row[0])

        # Heurística directa
        if ret_20d > 0.015 and rsi > 55:
            regime     = MarketRegime.BULL
            confidence = min(0.90, 0.65 + abs(ret_20d) * 10 + (rsi - 50) / 100)
            pb, pbe, ps = confidence, (1 - confidence) * 0.4, (1 - confidence) * 0.6
        elif ret_20d < -0.015 and rsi < 45:
            regime     = MarketRegime.BEAR
            confidence = min(0.90, 0.65 + abs(ret_20d) * 10 + (50 - rsi) / 100)
            pb, pbe, ps = (1 - confidence) * 0.6, confidence, (1 - confidence) * 0.4
        else:
            regime     = MarketRegime.SIDEWAYS
            confidence = 0.55 + adx / 500   # adx 0-50 → conf 0.55-0.65
            pb, pbe, ps = 0.2, 0.2, confidence

        if confidence < self.MIN_CONFIDENCE:
            regime     = MarketRegime.FLAT
            confidence = confidence

        return RegimeOutput(
            regime         = regime,
            confidence     = round(confidence, 4),
            proba_bull     = round(pb,  4),
            proba_bear     = round(pbe, 4),
            proba_sideways = round(ps,  4),
        )


# ── Runner ────────────────────────────────────────────────────────────────────

def run_demo(verbose: bool = False) -> None:
    print()
    print(f"{BOLD}{BLU}{'='*64}{RST}")
    print(f"{BOLD}{BLU}  Multi-Timeframe Coherence Demo — SPY simulado{RST}")
    print(f"{BOLD}{BLU}{'='*64}{RST}")

    # 1. Precio sintético (~20h de barras 1m)
    closes_1m = _make_spy_closes(n_bars=1200)
    print(f"\n  Barras simuladas : {len(closes_1m)}")
    print(f"  Precio inicial   : ${closes_1m[0]:.2f}")
    print(f"  Precio final     : ${closes_1m[-1]:.2f}")
    print(f"  Retorno total    : {(closes_1m[-1]/closes_1m[0]-1)*100:+.2f}%")

    # 2. Classifier + Analyzer
    clf = _MockRegimeClassifier()
    mtf = MultiTimeframeAnalyzer(
        regime_clf  = clf,
        timeframes  = ["1m", "5m", "15m", "1h"],
        min_bars_per_tf = 30,
    )

    # 3. Snapshots a lo largo del tiempo (cada 150 barras)
    snapshots = [300, 450, 600, 750, 900, 1050, 1199]

    print(f"\n  {BOLD}Análisis en {len(snapshots)} puntos temporales:{RST}")
    print(f"\n  {'Bar':>5} {'Precio':>8}  {'Dominant':>9}  {'Coh':>6}  "
          f"{'1m':>9}  {'5m':>9}  {'15m':>9}  {'1h':>9}")
    print("  " + "-" * 78)

    for bar_idx in snapshots:
        window = closes_1m[:bar_idx]
        t0     = time.perf_counter()
        report = mtf.analyze("SPY", window)
        ms     = (time.perf_counter() - t0) * 1000

        # Color coherence
        coh = report.coherence_score
        coh_c = GRN if coh >= 0.70 else (YLW if coh >= 0.50 else RED)
        coh_str = f"{coh_c}{coh:.3f}{RST}"

        # TF breakdown
        tf_cells = []
        for tf in ["1m", "5m", "15m", "1h"]:
            r = report.tf_breakdown.get(tf)
            if r is None:
                tf_cells.append(f"{'N/A':>9}")
            else:
                cell = f"{_rc(r.regime)}/{r.strength:.2f}"
                tf_cells.append(f"{cell:>9}")

        print(f"  {bar_idx:>5} ${closes_1m[bar_idx-1]:>7.2f}  "
              f"{_rc(report.dominant_trend):>9}  {coh_str:>6}  "
              + "  ".join(tf_cells)
              + f"  [{ms:.1f}ms]")

        if verbose:
            print(f"         aligned={report.aligned_count}/{report.total_tfs} TFs")
            for tf, r in report.tf_breakdown.items():
                print(f"         {tf:>4}: {_rc(r.regime):<12} "
                      f"str={r.strength:.3f}  conf={r.confidence:.3f}")

    # 4. Test de gate: señal bloqueada vs pasada
    print(f"\n  {BOLD}Test de gate SignalGenerator (MIN_MTF_COHERENCE=0.70):{RST}")

    # BULL sostenido → coherencia alta (barras 900-1199)
    bull_window = closes_1m[900:]
    rep_bull = mtf.analyze("SPY", bull_window)
    gate_bull = "PASA" if rep_bull.coherence_score >= 0.70 else "BLOQUEADA"
    gate_c_bull = GRN if gate_bull == "PASA" else RED
    print(f"    Fase BULL   (bars 900-1199): coh={rep_bull.coherence_score:.3f}  "
          f"dominant={_rc(rep_bull.dominant_trend)}  "
          f"señal={gate_c_bull}{gate_bull}{RST}")

    # SIDEWAYS → coherencia baja (barras 300-600)
    side_window = closes_1m[300:600]
    rep_side = mtf.analyze("SPY", side_window)
    gate_side = "PASA" if rep_side.coherence_score >= 0.70 else "BLOQUEADA"
    gate_c_side = GRN if gate_side == "PASA" else RED
    print(f"    Fase SIDE   (bars 300-600) : coh={rep_side.coherence_score:.3f}  "
          f"dominant={_rc(rep_side.dominant_trend)}  "
          f"señal={gate_c_side}{gate_side}{RST}")

    # BEAR puro → coherencia alta si TFs alineados
    bear_window = closes_1m[600:900]
    rep_bear = mtf.analyze("SPY", bear_window)
    gate_bear = "PASA" if rep_bear.coherence_score >= 0.70 else "BLOQUEADA"
    gate_c_bear = GRN if gate_bear == "PASA" else RED
    print(f"    Fase BEAR   (bars 600-900) : coh={rep_bear.coherence_score:.3f}  "
          f"dominant={_rc(rep_bear.dominant_trend)}  "
          f"señal={gate_c_bear}{gate_bear}{RST}")

    # 5. Métricas de latencia
    latencies = []
    for _ in range(20):
        w = closes_1m[:600]
        t0 = time.perf_counter()
        mtf.analyze("SPY", w)
        latencies.append((time.perf_counter() - t0) * 1000)

    p50 = float(np.percentile(latencies, 50))
    p95 = float(np.percentile(latencies, 95))
    print(f"\n  {BOLD}Latencia (20 runs, 600 barras):{RST} "
          f"p50={p50:.2f}ms  p95={p95:.2f}ms")

    lat_ok = p95 < 50
    lat_c  = GRN if lat_ok else YLW
    print(f"  Latencia p95 <50ms: {lat_c}{'OK' if lat_ok else 'WARN'}{RST}")

    print(f"\n{BOLD}{BLU}{'='*64}{RST}")
    print(f"{BOLD}  Demo completado.{RST}")
    print(f"{BOLD}{BLU}{'='*64}{RST}\n")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Demo Multi-TF Coherence — SPY simulado")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Muestra breakdown por TF en cada snapshot")
    args = parser.parse_args()
    run_demo(verbose=args.verbose)


if __name__ == "__main__":
    main()
