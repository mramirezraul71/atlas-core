"""
sensitivity_analysis.py — Análisis de sensibilidad de parámetros ATLAS-Quant.

Tres experimentos independientes sobre 10 000 señales sintéticas:
  A) Ponderaciones: 25/35/20/20 (actual) vs 30/30/20/20
  B) MTF threshold gate: 0.70 vs 0.75
  C) Kelly max_factor FULL: 2.0x vs 2.5x

Métricas comparadas por experimento:
  - Distribución de tiers (FULL/NORMAL/SMALL/SKIP)
  - Tasa de skippeo y filtrado MTF
  - Capital-at-risk promedio y P95 por trade
  - Score esperado en cada tier
  - Sharpe proxy (score·factor como proxy de retorno ajustado)

Uso:
    python scripts/sensitivity_analysis.py
    python scripts/sensitivity_analysis.py --n 50000
    python scripts/sensitivity_analysis.py --seed 99
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import NamedTuple

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "atlas_code_quant"))

# ── ANSI ──────────────────────────────────────────────────────────────────────
GRN  = "\033[92m"; RED  = "\033[91m"; YLW  = "\033[93m"
CYN  = "\033[96m"; RST  = "\033[0m";  BOLD = "\033[1m"
DIM  = "\033[2m"

def _hdr(s): return f"{BOLD}{CYN}{s}{RST}"
def _ok(s):  return f"{GRN}{s}{RST}"
def _warn(s):return f"{YLW}{s}{RST}"
def _bad(s): return f"{RED}{s}{RST}"
def _dim(s): return f"{DIM}{s}{RST}"


# ── Generación de señales sintéticas ─────────────────────────────────────────

@dataclass
class SyntheticSignal:
    motif_edge:         float   # [0,1]
    tin_score:          float   # [0,1]
    mtf_coherence:      float   # [0,1]
    regime_confidence:  float   # [0,1]
    # "ground truth" para evaluar calidad: retorno futuro normalizado
    true_return:        float   # [-1,1]


def generate_signals(n: int, seed: int = 42) -> list[SyntheticSignal]:
    """Genera N señales sintéticas con correlaciones realistas.

    Correlaciones aproximadas:
      - tin_score y motif_edge correlacionados moderadamente (r≈0.35)
      - mtf_coherence y regime_confidence correlacionados (r≈0.50)
      - true_return correlacionado con score compuesto (r≈0.45)
    """
    rng = np.random.default_rng(seed)
    # Factores latentes
    trend   = rng.normal(0, 1, n)      # factor de tendencia
    quality = rng.normal(0, 1, n)      # calidad técnica

    def _norm(x): return np.clip((x + 3) / 6, 0.0, 1.0)

    motif  = _norm(trend  * 0.6 + rng.normal(0, 0.8, n))
    tin    = _norm(quality * 0.5 + trend  * 0.35 + rng.normal(0, 0.6, n))
    mtf    = _norm(trend  * 0.4 + quality * 0.3  + rng.normal(0, 0.7, n))
    regime = _norm(quality * 0.55+ trend  * 0.25 + rng.normal(0, 0.5, n))
    # Retorno verdadero ~ score compuesto + ruido
    true_r = (0.30*(motif-0.5) + 0.35*(tin-0.5) +
              0.20*(mtf-0.5)   + 0.20*(regime-0.5) +
              rng.normal(0, 0.15, n))

    return [
        SyntheticSignal(motif[i], tin[i], mtf[i], regime[i], true_r[i])
        for i in range(n)
    ]


# ── Modelos paramétricos ──────────────────────────────────────────────────────

class WeightConfig(NamedTuple):
    w_motif:  float
    w_tin:    float
    w_mtf:    float
    w_regime: float
    label:    str


def score_signal(sig: SyntheticSignal, cfg: WeightConfig) -> float:
    """Fórmula final_signal_score() parametrizada."""
    m = max(0.0, min(1.0, (sig.motif_edge + 1.0) / 2.0))   # ya [0,1] → [0.5,1]
    t = max(0.0, min(1.0, sig.tin_score))
    c = max(0.0, min(1.0, sig.mtf_coherence))
    r = max(0.0, min(1.0, sig.regime_confidence))
    return float(max(0.0, min(1.0,
        cfg.w_motif * m + cfg.w_tin * t + cfg.w_mtf * c + cfg.w_regime * r
    )))


def get_tier(score: float) -> str:
    if score >= 0.75: return "FULL"
    if score >= 0.65: return "NORMAL"
    if score >= 0.55: return "SMALL"
    return "SKIP"


def kelly_factor(score: float, max_factor: float = 2.0) -> float:
    tier = get_tier(score)
    if tier == "SKIP":   return 0.0
    if tier == "SMALL":  return 0.5
    return min(max_factor, max(0.5, score * 3.0))


# ── Métricas de evaluación ────────────────────────────────────────────────────

@dataclass
class ExperimentResult:
    label:       str
    n_total:     int
    n_skip:      int
    n_full:      int
    n_normal:    int
    n_small:     int
    n_mtf_blocked: int   # bloqueadas por gate MTF
    # Para señales no SKIP
    avg_score:   float
    avg_factor:  float
    avg_car:     float   # capital-at-risk proxy (score × factor)
    p95_car:     float
    # Calidad: correlación score con true_return (solo no-SKIP)
    corr_score_return:  float
    # Proxy de Sharpe: suma(score·factor·true_return) / std
    sharpe_proxy: float


def run_experiment(
    signals: list[SyntheticSignal],
    weight_cfg: WeightConfig,
    mtf_threshold: float,
    max_kelly: float,
) -> ExperimentResult:

    cars, scores, true_rets = [], [], []
    n_skip = n_full = n_normal = n_small = n_mtf_blocked = 0

    for sig in signals:
        # Gate MTF (antes del score)
        if sig.mtf_coherence < mtf_threshold:
            n_mtf_blocked += 1
            n_skip += 1
            continue

        score = score_signal(sig, weight_cfg)
        tier  = get_tier(score)
        fac   = kelly_factor(score, max_kelly)

        if tier == "SKIP":
            n_skip += 1
            continue

        if   tier == "FULL":   n_full   += 1
        elif tier == "NORMAL": n_normal += 1
        else:                  n_small  += 1

        car = score * fac
        cars.append(car)
        scores.append(score)
        true_rets.append(sig.true_return)

    n_exec = len(cars)
    if n_exec == 0:
        return ExperimentResult(
            label=weight_cfg.label, n_total=len(signals), n_skip=n_skip,
            n_full=0, n_normal=0, n_small=0, n_mtf_blocked=n_mtf_blocked,
            avg_score=0, avg_factor=0, avg_car=0, p95_car=0,
            corr_score_return=0, sharpe_proxy=0,
        )

    cars_a  = np.array(cars)
    scores_a = np.array(scores)
    tr_a    = np.array(true_rets)

    corr = float(np.corrcoef(scores_a, tr_a)[0, 1]) if n_exec > 1 else 0.0

    # Sharpe proxy: retorno ponderado por factor / std(retorno)
    weighted_rets = cars_a * tr_a
    sp = (weighted_rets.mean() / (weighted_rets.std() + 1e-8)
          * np.sqrt(min(252, n_exec)))

    return ExperimentResult(
        label           = weight_cfg.label,
        n_total         = len(signals),
        n_skip          = n_skip,
        n_full          = n_full,
        n_normal        = n_normal,
        n_small         = n_small,
        n_mtf_blocked   = n_mtf_blocked,
        avg_score       = float(scores_a.mean()),
        avg_factor      = float(cars_a.mean() / (scores_a.mean() + 1e-8)),
        avg_car         = float(cars_a.mean()),
        p95_car         = float(np.percentile(cars_a, 95)),
        corr_score_return = corr,
        sharpe_proxy    = float(sp),
    )


# ── Formateo de tablas ────────────────────────────────────────────────────────

def _pct(n, total): return f"{n/total*100:.1f}%" if total > 0 else "—"
def _delta(v, base, higher_is_better=True):
    d = v - base
    if abs(d) < 1e-4: return _dim("  =")
    sym  = "+" if d > 0 else ""
    good = (d > 0) == higher_is_better
    fn   = _ok if good else _bad
    return fn(f"{sym}{d:+.4f}")


def print_experiment(title: str, base: ExperimentResult, alt: ExperimentResult,
                     param_name: str, base_val: str, alt_val: str) -> None:
    n = base.n_total
    print(f"\n{_hdr(title)}")
    print(f"  {_dim(f'N={n:,} señales sintéticas')}")
    print()

    rows = [
        ("Param",          f"{param_name}={base_val}",        f"{param_name}={alt_val}"),
        ("Ejecutadas",     f"{n-base.n_skip:,} ({_pct(n-base.n_skip,n)})",
                           f"{n-alt.n_skip:,} ({_pct(n-alt.n_skip,n)})"),
        ("  SKIP",         f"{base.n_skip:,} ({_pct(base.n_skip,n)})",
                           f"{alt.n_skip:,} ({_pct(alt.n_skip,n)})"),
        ("  FULL",         f"{base.n_full:,} ({_pct(base.n_full,n)})",
                           f"{alt.n_full:,} ({_pct(alt.n_full,n)})"),
        ("  NORMAL",       f"{base.n_normal:,} ({_pct(base.n_normal,n)})",
                           f"{alt.n_normal:,} ({_pct(alt.n_normal,n)})"),
        ("  SMALL",        f"{base.n_small:,} ({_pct(base.n_small,n)})",
                           f"{alt.n_small:,} ({_pct(alt.n_small,n)})"),
        ("  MTF-blocked",  f"{base.n_mtf_blocked:,} ({_pct(base.n_mtf_blocked,n)})",
                           f"{alt.n_mtf_blocked:,} ({_pct(alt.n_mtf_blocked,n)})"),
        ("avg_score",      f"{base.avg_score:.4f}",   f"{alt.avg_score:.4f}"),
        ("avg_factor",     f"{base.avg_factor:.3f}x", f"{alt.avg_factor:.3f}x"),
        ("avg_CAR",        f"{base.avg_car:.4f}",     f"{alt.avg_car:.4f}"),
        ("p95_CAR",        f"{base.p95_car:.4f}",     f"{alt.p95_car:.4f}"),
        ("corr(score,ret)",f"{base.corr_score_return:.4f}", f"{alt.corr_score_return:.4f}"),
        ("Sharpe proxy",   f"{base.sharpe_proxy:.4f}", f"{alt.sharpe_proxy:.4f}"),
    ]

    col_w = [22, 26, 26, 12]
    header_fmt = f"  {{:<{col_w[0]}}} {{:<{col_w[1]}}} {{:<{col_w[2]}}} {{:<{col_w[3]}}}"
    row_fmt    = f"  {{:<{col_w[0]}}} {{:<{col_w[1]}}} {{:<{col_w[2]}}} {{:<{col_w[3]}}}"

    print(header_fmt.format("Métrica", f"BASE ({base_val})", f"ALT ({alt_val})", "Delta"))
    print("  " + "-" * (sum(col_w) + 6))

    delta_metrics = {
        "avg_score":       (base.avg_score,       alt.avg_score,       True),
        "avg_factor":      (base.avg_factor,       alt.avg_factor,       True),
        "avg_CAR":         (base.avg_car,           alt.avg_car,           True),
        "p95_CAR":         (base.p95_car,           alt.p95_car,           False),  # menor riesgo mejor
        "corr(score,ret)": (base.corr_score_return, alt.corr_score_return, True),
        "Sharpe proxy":    (base.sharpe_proxy,      alt.sharpe_proxy,      True),
        "  SKIP":          (base.n_skip/n,          alt.n_skip/n,          False),  # menos skip=mejor
        "  MTF-blocked":   (base.n_mtf_blocked/n,   alt.n_mtf_blocked/n,   False),
    }

    for label, base_str, alt_str in rows:
        if label in delta_metrics:
            bv, av, hib = delta_metrics[label]
            delta_str = _delta(av, bv, hib)
        else:
            delta_str = ""
        print(row_fmt.format(label, base_str, alt_str, delta_str))


# ── Recomendación ─────────────────────────────────────────────────────────────

def _recommend(base: ExperimentResult, alt: ExperimentResult,
               metric_name: str, param: str, base_v: str, alt_v: str,
               higher_is_better: bool = True) -> None:
    bv = getattr(base, metric_name)
    av = getattr(alt,  metric_name)
    better = (av > bv) if higher_is_better else (av < bv)
    winner = alt_v if better else base_v
    delta  = abs(av - bv) / (abs(bv) + 1e-8) * 100
    sig    = "SIGNIFICATIVO" if delta > 2.0 else ("MARGINAL" if delta > 0.5 else "NEUTRO")
    color  = _ok if better else _warn
    print(f"  {param}: recomendado={color(winner)}{RST}  d{metric_name}={delta:.2f}%  [{sig}]")


# ── Main ──────────────────────────────────────────────────────────────────────

def main(n: int = 10_000, seed: int = 42) -> None:
    print(f"\n{_hdr('='*66)}")
    print(f"{_hdr('  ATLAS-Quant — Sensitivity Analysis')}")
    print(f"{_hdr('='*66)}")
    print(f"  Generando {n:,} señales sintéticas (seed={seed})…")

    t0 = time.perf_counter()
    signals = generate_signals(n, seed)
    gen_ms = (time.perf_counter() - t0) * 1000
    print(f"  Generadas en {gen_ms:.0f}ms\n")

    # ── Configs base ──────────────────────────────────────────────────────────
    W_ACTUAL  = WeightConfig(0.25, 0.35, 0.20, 0.20, "25/35/20/20")
    W_ALT     = WeightConfig(0.30, 0.30, 0.20, 0.20, "30/30/20/20")
    MTF_BASE  = 0.70
    MTF_ALT   = 0.75
    KELLY_BASE = 2.0
    KELLY_ALT  = 2.5

    # ── Experimento A: ponderaciones ──────────────────────────────────────────
    t0 = time.perf_counter()
    res_A_base = run_experiment(signals, W_ACTUAL,  MTF_BASE, KELLY_BASE)
    res_A_alt  = run_experiment(signals, W_ALT,     MTF_BASE, KELLY_BASE)
    ms_A = (time.perf_counter() - t0) * 1000

    print_experiment(
        "EXPERIMENTO A — Ponderaciones (MTF=0.70 Kelly=2.0x)",
        res_A_base, res_A_alt,
        "pesos", W_ACTUAL.label, W_ALT.label,
    )
    print(f"  {_dim(f'Tiempo: {ms_A:.0f}ms')}")

    # ── Experimento B: MTF threshold ──────────────────────────────────────────
    t0 = time.perf_counter()
    res_B_base = run_experiment(signals, W_ACTUAL, MTF_BASE,  KELLY_BASE)
    res_B_alt  = run_experiment(signals, W_ACTUAL, MTF_ALT,   KELLY_BASE)
    ms_B = (time.perf_counter() - t0) * 1000

    print_experiment(
        "EXPERIMENTO B — MTF threshold (pesos=25/35/20/20 Kelly=2.0x)",
        res_B_base, res_B_alt,
        "mtf_thresh", str(MTF_BASE), str(MTF_ALT),
    )
    print(f"  {_dim(f'Tiempo: {ms_B:.0f}ms')}")

    # ── Experimento C: Kelly max factor ───────────────────────────────────────
    t0 = time.perf_counter()
    res_C_base = run_experiment(signals, W_ACTUAL, MTF_BASE, KELLY_BASE)
    res_C_alt  = run_experiment(signals, W_ACTUAL, MTF_BASE, KELLY_ALT)
    ms_C = (time.perf_counter() - t0) * 1000

    print_experiment(
        "EXPERIMENTO C — Kelly max_factor (pesos=25/35/20/20 MTF=0.70)",
        res_C_base, res_C_alt,
        "max_kelly", f"{KELLY_BASE}x", f"{KELLY_ALT}x",
    )
    print(f"  {_dim(f'Tiempo: {ms_C:.0f}ms')}")

    # ── Experimento D: combinación óptima candidata ───────────────────────────
    # Toma el ganador de cada experimento individual
    W_WIN  = W_ALT    if res_A_alt.sharpe_proxy > res_A_base.sharpe_proxy else W_ACTUAL
    MT_WIN = MTF_ALT  if res_B_alt.sharpe_proxy > res_B_base.sharpe_proxy else MTF_BASE
    KL_WIN = KELLY_ALT if res_C_alt.sharpe_proxy > res_C_base.sharpe_proxy else KELLY_BASE

    res_D_base = run_experiment(signals, W_ACTUAL,  MTF_BASE, KELLY_BASE)
    res_D_best = run_experiment(signals, W_WIN,     MT_WIN,   KL_WIN)

    print_experiment(
        f"EXPERIMENTO D — Config actual vs óptima candidata",
        res_D_base, res_D_best,
        "config",
        "actual",
        f"{W_WIN.label}|MTF{MT_WIN}|K{KL_WIN}x",
    )

    # ── Tabla de recomendaciones ──────────────────────────────────────────────
    print(f"\n{_hdr('RECOMENDACIONES (métrica base: Sharpe proxy)')}")
    _recommend(res_A_base, res_A_alt,  "sharpe_proxy",
               "Ponderaciones",  W_ACTUAL.label, W_ALT.label)
    _recommend(res_B_base, res_B_alt,  "sharpe_proxy",
               "MTF threshold",  str(MTF_BASE), str(MTF_ALT))
    _recommend(res_C_base, res_C_alt,  "sharpe_proxy",
               "Kelly max_factor", f"{KELLY_BASE}x", f"{KELLY_ALT}x")

    # Métricas de riesgo
    print()
    print(f"  {BOLD}Riesgo (p95_CAR — menor=mejor):{RST}")
    _recommend(res_A_base, res_A_alt,  "p95_car",
               "  Ponderaciones",  W_ACTUAL.label, W_ALT.label, False)
    _recommend(res_B_base, res_B_alt,  "p95_car",
               "  MTF threshold",  str(MTF_BASE), str(MTF_ALT), False)
    _recommend(res_C_base, res_C_alt,  "p95_car",
               "  Kelly max_factor", f"{KELLY_BASE}x", f"{KELLY_ALT}x", False)

    # ── Score distribution deep dive ──────────────────────────────────────────
    print(f"\n{_hdr('DISTRIBUCIÓN DE SCORES — Config actual vs candidata')}")
    _print_score_dist(signals, W_ACTUAL,  MTF_BASE, KELLY_BASE, "Actual  25/35/20/20 | MTF0.70 | K2.0x")
    _print_score_dist(signals, W_WIN,     MT_WIN,   KL_WIN,
                      f"Candid. {W_WIN.label} | MTF{MT_WIN} | K{KL_WIN}x")

    print(f"\n{_hdr('='*66)}\n")


def _print_score_dist(signals, w, mtf_t, kl, label):
    scores, cars = [], []
    n_mtf = 0
    for sig in signals:
        if sig.mtf_coherence < mtf_t:
            n_mtf += 1
            continue
        s  = score_signal(sig, w)
        f  = kelly_factor(s, kl)
        if f == 0: continue
        scores.append(s)
        cars.append(s * f)

    if not scores:
        print(f"  {label}: sin datos")
        return

    sa = np.array(scores)
    ca = np.array(cars)
    buckets = [
        ("<0.55",       np.sum(sa < 0.55)),
        ("0.55-0.65",   np.sum((sa >= 0.55) & (sa < 0.65))),
        ("0.65-0.75",   np.sum((sa >= 0.65) & (sa < 0.75))),
        (">=0.75",      np.sum(sa >= 0.75)),
    ]
    total_exec = len(sa)
    bar_parts = [f"{lbl}:{cnt:>5} ({cnt/len(signals)*100:.1f}%)"
                 for lbl, cnt in buckets]
    print(f"  {BOLD}{label}{RST}")
    print(f"    Ejecutadas={total_exec:,}/{len(signals):,}  MTF-blocked={n_mtf:,}")
    print(f"    Scores: {' | '.join(bar_parts)}")
    print(f"    CAR: avg={ca.mean():.4f} p50={np.median(ca):.4f} "
          f"p95={np.percentile(ca,95):.4f} max={ca.max():.4f}")


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sensitivity analysis ATLAS-Quant params")
    parser.add_argument("--n",    type=int, default=10_000, help="Número de señales")
    parser.add_argument("--seed", type=int, default=42,     help="Semilla aleatoria")
    args = parser.parse_args()
    main(n=args.n, seed=args.seed)
