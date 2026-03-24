"""Motor de métricas estadísticas puras para AtlasLearningBrain.

Todas las funciones son puras (sin I/O, sin estado global).
Reciben listas de TradeEvent y devuelven estructuras de datos
definidas en trade_events.py.

Métricas implementadas
----------------------
- compute_metrics()           — MetricsSummary completo
- compute_group_metrics()     — desglose por dimensión
- compute_stability_score()   — estabilidad temporal (first/second half)
- detect_error_patterns()     — patrones de error operativo
- detect_success_patterns()   — condiciones de alta probabilidad
- propose_policies()          — lista de PolicyAction basadas en métricas
- check_readiness()           — SystemReadinessReport con 7 criterios
"""
from __future__ import annotations

import math
import statistics
from collections import defaultdict
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from atlas_code_quant.learning.trade_events import (
    LearningReport,
    MetricsSummary,
    PolicyAction,
    ReadinessCriterion,
    SystemReadinessReport,
    TradeEvent,
)


# ---------------------------------------------------------------------------
# Helpers internos
# ---------------------------------------------------------------------------

def _r_series(trades: List[TradeEvent]) -> List[float]:
    return [t.r_realized for t in trades]


def _winners(trades: List[TradeEvent]) -> List[TradeEvent]:
    return [t for t in trades if t.r_realized > 0]


def _losers(trades: List[TradeEvent]) -> List[TradeEvent]:
    return [t for t in trades if t.r_realized <= 0]


def _running_drawdown(r_series: List[float]) -> float:
    """Calcula max drawdown en R (peak-to-trough acumulado, valor negativo)."""
    if not r_series:
        return 0.0
    peak = 0.0
    cumulative = 0.0
    max_dd = 0.0
    for r in r_series:
        cumulative += r
        if cumulative > peak:
            peak = cumulative
        dd = cumulative - peak
        if dd < max_dd:
            max_dd = dd
    return max_dd


def _calmar(annualized_r: float, max_drawdown_r: float) -> float:
    if max_drawdown_r == 0:
        return 0.0
    return annualized_r / abs(max_drawdown_r)


def _sharpe_r(r_series: List[float], periods_per_year: int = 252) -> float:
    if len(r_series) < 2:
        return 0.0
    mean = statistics.mean(r_series)
    std = statistics.stdev(r_series)
    if std == 0:
        return 0.0
    return (mean / std) * math.sqrt(periods_per_year)


def _annualized_return_r(total_r: float, n_trades: int, avg_duration_min: float) -> float:
    """Estima retorno anualizado en R asumiendo ~252 días de trading y 6.5h/día."""
    if n_trades == 0 or avg_duration_min == 0:
        return 0.0
    minutes_per_year = 252 * 6.5 * 60
    # trades_per_year = tiempo_total_disponible / duración_media_por_trade
    total_minutes = n_trades * avg_duration_min
    if total_minutes == 0:
        return 0.0
    scale = minutes_per_year / total_minutes
    return total_r * scale


def _consecutive_streaks(r_series: List[float]) -> Tuple[int, int]:
    """Devuelve (max_consecutive_wins, max_consecutive_losses)."""
    max_w = max_l = cur_w = cur_l = 0
    for r in r_series:
        if r > 0:
            cur_w += 1
            cur_l = 0
        else:
            cur_l += 1
            cur_w = 0
        max_w = max(max_w, cur_w)
        max_l = max(max_l, cur_l)
    return max_w, max_l


# ---------------------------------------------------------------------------
# compute_metrics — MetricsSummary completo para una lista de trades
# ---------------------------------------------------------------------------

def compute_metrics(trades: List[TradeEvent]) -> MetricsSummary:
    """Calcula MetricsSummary completo para una lista de TradeEvent."""
    if not trades:
        return MetricsSummary()

    n = len(trades)
    r_vals = _r_series(trades)
    wins = _winners(trades)
    losses = _losers(trades)

    n_w = len(wins)
    n_l = len(losses)
    winrate = n_w / n if n > 0 else 0.0

    sum_win_r = sum(t.r_realized for t in wins) if wins else 0.0
    sum_loss_r = abs(sum(t.r_realized for t in losses)) if losses else 0.0

    profit_factor = sum_win_r / sum_loss_r if sum_loss_r > 0 else (float("inf") if sum_win_r > 0 else 0.0)
    expectancy_r = statistics.mean(r_vals) if r_vals else 0.0

    avg_winner_r = statistics.mean([t.r_realized for t in wins]) if wins else 0.0
    avg_loser_r = statistics.mean([t.r_realized for t in losses]) if losses else 0.0

    total_r = sum(r_vals)
    max_dd_r = _running_drawdown(r_vals)

    max_w_streak, max_l_streak = _consecutive_streaks(r_vals)

    avg_mae = statistics.mean([t.mae_r for t in trades]) if trades else 0.0
    avg_mfe = statistics.mean([t.mfe_r for t in trades]) if trades else 0.0
    avg_dur = statistics.mean([t.duration_minutes for t in trades]) if trades else 0.0

    ann_r = _annualized_return_r(total_r, n, avg_dur)
    calmar = _calmar(ann_r, max_dd_r)
    sharpe = _sharpe_r(r_vals)

    return MetricsSummary(
        n_trades=n,
        n_winners=n_w,
        n_losers=n_l,
        winrate=round(winrate, 4),
        profit_factor=round(profit_factor, 4),
        expectancy_r=round(expectancy_r, 4),
        avg_winner_r=round(avg_winner_r, 4),
        avg_loser_r=round(avg_loser_r, 4),
        total_r=round(total_r, 4),
        max_drawdown_r=round(max_dd_r, 4),
        max_consecutive_losses=max_l_streak,
        max_consecutive_wins=max_w_streak,
        avg_mae_r=round(avg_mae, 4),
        avg_mfe_r=round(avg_mfe, 4),
        avg_duration_minutes=round(avg_dur, 2),
        calmar_ratio=round(calmar, 4),
        sharpe_r=round(sharpe, 4),
    )


# ---------------------------------------------------------------------------
# compute_group_metrics — desglose por dimensión
# ---------------------------------------------------------------------------

def compute_group_metrics(
    trades: List[TradeEvent],
    key_fn,
) -> Dict[Any, MetricsSummary]:
    """Agrupa trades por `key_fn(trade)` y calcula MetricsSummary por grupo."""
    groups: Dict[Any, List[TradeEvent]] = defaultdict(list)
    for t in trades:
        groups[key_fn(t)].append(t)
    return {k: compute_metrics(v) for k, v in groups.items()}


def metrics_by_setup(trades: List[TradeEvent]) -> Dict[str, MetricsSummary]:
    return compute_group_metrics(trades, lambda t: t.setup_type)


def metrics_by_symbol(trades: List[TradeEvent]) -> Dict[str, MetricsSummary]:
    return compute_group_metrics(trades, lambda t: t.symbol)


def metrics_by_timeframe(trades: List[TradeEvent]) -> Dict[str, MetricsSummary]:
    return compute_group_metrics(trades, lambda t: t.timeframe)


def metrics_by_regime(trades: List[TradeEvent]) -> Dict[str, MetricsSummary]:
    return compute_group_metrics(trades, lambda t: t.regime)


def metrics_by_asset_class(trades: List[TradeEvent]) -> Dict[str, MetricsSummary]:
    return compute_group_metrics(trades, lambda t: t.asset_class)


def metrics_by_hour(trades: List[TradeEvent]) -> Dict[int, MetricsSummary]:
    return compute_group_metrics(trades, lambda t: t.entry_time.hour)


# ---------------------------------------------------------------------------
# compute_stability_score — estabilidad temporal
# ---------------------------------------------------------------------------

def compute_stability_score(trades: List[TradeEvent]) -> float:
    """Compara métricas first-half vs second-half.

    Devuelve un score 0.0 – 1.0.  >0.70 indica sistema estable:
    las métricas no degradan en la segunda mitad del historial.

    Criterios comparados:
    - profit_factor (ratio second/first, normalizado)
    - expectancy_r  (ratio second/first, normalizado)
    - winrate       (diferencia absoluta)
    """
    if len(trades) < 20:
        return 0.0

    mid = len(trades) // 2
    first = trades[:mid]
    second = trades[mid:]

    m1 = compute_metrics(first)
    m2 = compute_metrics(second)

    scores: List[float] = []

    # Profit factor stability
    if m1.profit_factor > 0:
        ratio_pf = m2.profit_factor / m1.profit_factor
        # 1.0 = igual, <0.5 = mucho peor, >2.0 = mucho mejor (cap 1.0)
        pf_score = min(1.0, ratio_pf)
        scores.append(pf_score)

    # Expectancy stability
    if m1.expectancy_r != 0:
        ratio_exp = m2.expectancy_r / m1.expectancy_r if m1.expectancy_r > 0 else 0.0
        exp_score = min(1.0, max(0.0, ratio_exp))
        scores.append(exp_score)

    # Winrate stability (diferencia absoluta, penaliza degradación >10%)
    wr_diff = abs(m2.winrate - m1.winrate)
    wr_score = max(0.0, 1.0 - wr_diff * 5)  # -20 pts por cada 4% de caída
    scores.append(wr_score)

    if not scores:
        return 0.0

    return round(statistics.mean(scores), 4)


# ---------------------------------------------------------------------------
# detect_error_patterns — patrones de error operativo
# ---------------------------------------------------------------------------

def detect_error_patterns(trades: List[TradeEvent]) -> List[str]:
    """Detecta patrones de error que correlacionan con pérdidas.

    Devuelve lista de strings descriptivos para incluir en LearningReport.
    """
    if not trades:
        return []

    patterns: List[str] = []
    losers = _losers(trades)
    if not losers:
        return []

    n_total = len(trades)
    n_losers = len(losers)

    # Frecuencia de error_flags en perdedores vs total
    flag_count_losses: Dict[str, int] = defaultdict(int)
    flag_count_all: Dict[str, int] = defaultdict(int)

    for t in trades:
        for f in t.error_flags:
            flag_count_all[f] += 1
    for t in losers:
        for f in t.error_flags:
            flag_count_losses[f] += 1

    for flag, loss_count in flag_count_losses.items():
        all_count = flag_count_all[flag]
        pct_in_losses = loss_count / n_losers if n_losers > 0 else 0
        pct_in_all = all_count / n_total if n_total > 0 else 0

        if pct_in_losses >= 0.25 and loss_count >= 5:
            # El flag aparece en ≥25% de pérdidas con al menos 5 casos
            patterns.append(
                f"'{flag}' aparece en {pct_in_losses:.0%} de pérdidas "
                f"({loss_count}/{n_losers} trades) — pct_global={pct_in_all:.0%}"
            )

    # MAE extremo en pérdidas: trades que excedieron el stop y fueron hit
    deep_mae = [t for t in losers if t.mae_r < -2.0]
    if len(deep_mae) >= 3:
        patterns.append(
            f"MAE > -2R en {len(deep_mae)} pérdidas — posibles stops movidos o gaps"
        )

    # Pérdidas agrupadas por hora
    loss_by_hour: Dict[int, int] = defaultdict(int)
    for t in losers:
        loss_by_hour[t.entry_time.hour] += 1
    for hour, count in loss_by_hour.items():
        if count >= 5 and count / n_losers >= 0.30:
            patterns.append(
                f"30%+ de pérdidas se abren a las {hour:02d}h UTC ({count} trades)"
            )

    # Sobreoperación en días con ≥3 pérdidas consecutivas
    r_vals = _r_series(trades)
    _, max_l = _consecutive_streaks(r_vals)
    if max_l >= 5:
        patterns.append(
            f"Racha máxima de {max_l} pérdidas seguidas — revisar regla de stop-day"
        )

    return patterns


# ---------------------------------------------------------------------------
# detect_success_patterns — condiciones de alta probabilidad
# ---------------------------------------------------------------------------

def detect_success_patterns(trades: List[TradeEvent]) -> List[str]:
    """Detecta condiciones que correlacionan con alta probabilidad de ganancia."""
    if len(trades) < 20:
        return []

    patterns: List[str] = []
    winners = _winners(trades)

    if not winners:
        return []

    # PF por setup con ≥10 trades
    by_setup = metrics_by_setup(trades)
    for setup, m in by_setup.items():
        if m.n_trades >= 10 and m.profit_factor >= 1.8:
            patterns.append(
                f"{setup}: PF={m.profit_factor:.2f}, WR={m.winrate:.0%} (n={m.n_trades})"
            )

    # PF por regime
    by_regime = metrics_by_regime(trades)
    for regime, m in by_regime.items():
        if m.n_trades >= 10 and m.profit_factor >= 2.0:
            patterns.append(
                f"Régimen {regime}: PF={m.profit_factor:.2f} (n={m.n_trades})"
            )

    # IV Rank alto + IV/HV ratio favorable
    iv_high = [t for t in trades if t.iv_rank > 50 and t.iv_hv_ratio > 1.2]
    if len(iv_high) >= 10:
        m = compute_metrics(iv_high)
        if m.winrate >= 0.60:
            patterns.append(
                f"iv_rank>50 + iv_hv>1.2: WR={m.winrate:.0%}, PF={m.profit_factor:.2f} (n={m.n_trades})"
            )

    # Volume ratio elevado en entrada
    high_vol = [t for t in trades if t.volume_ratio > 1.5]
    if len(high_vol) >= 10:
        m = compute_metrics(high_vol)
        if m.profit_factor >= 1.5:
            patterns.append(
                f"volume_ratio>1.5 en entrada: PF={m.profit_factor:.2f} (n={m.n_trades})"
            )

    return patterns


# ---------------------------------------------------------------------------
# propose_policies — PolicyAction basadas en métricas
# ---------------------------------------------------------------------------

def propose_policies(
    trades: List[TradeEvent],
    global_metrics: MetricsSummary,
    by_setup: Dict[str, MetricsSummary],
    min_n: int = 20,
) -> List[PolicyAction]:
    """Propone PolicyAction basadas en análisis estadístico.

    Reglas de decisión:
    - Setup PF < 0.8 y n≥min_n → disable
    - Setup PF 0.8-1.0 y n≥min_n → reduce_size (×0.5)
    - Setup PF > 2.0 y WR > 55% y n≥min_n → increase_priority (+0.1 boost)
    - Global max_dd > -5R → reduce all sizes globally
    - Global profit_factor < 1.0 → disable all non-critical setups
    """
    policies: List[PolicyAction] = []

    # Políticas por setup
    for setup, m in by_setup.items():
        if m.n_trades < min_n:
            continue

        if m.profit_factor < 0.80:
            policies.append(PolicyAction(
                setup_type=setup,
                action="disable",
                reason=f"PF={m.profit_factor:.2f} < 0.80 con n={m.n_trades} trades",
                size_multiplier=0.0,
                score_boost=-1.0,
                confidence=min(1.0, m.n_trades / 50),
                n_trades_basis=m.n_trades,
            ))

        elif m.profit_factor < 1.00:
            policies.append(PolicyAction(
                setup_type=setup,
                action="reduce_size",
                reason=f"PF={m.profit_factor:.2f} (0.80-1.00) con n={m.n_trades}",
                size_multiplier=0.50,
                score_boost=-0.10,
                confidence=min(1.0, m.n_trades / 50),
                n_trades_basis=m.n_trades,
            ))

        elif m.profit_factor > 2.0 and m.winrate > 0.55:
            policies.append(PolicyAction(
                setup_type=setup,
                action="increase_priority",
                reason=f"PF={m.profit_factor:.2f}, WR={m.winrate:.0%} (n={m.n_trades})",
                size_multiplier=1.20,
                score_boost=0.10,
                confidence=min(1.0, m.n_trades / 50),
                n_trades_basis=m.n_trades,
            ))

    # Política global por drawdown severo
    if global_metrics.max_drawdown_r < -5.0:
        policies.append(PolicyAction(
            setup_type="*",
            action="reduce_size",
            reason=f"Max DD = {global_metrics.max_drawdown_r:.1f}R — reducción global preventiva",
            size_multiplier=0.75,
            score_boost=0.0,
            confidence=1.0,
            n_trades_basis=global_metrics.n_trades,
        ))

    # Política global por sistema no rentable
    if global_metrics.profit_factor < 1.0 and global_metrics.n_trades >= 50:
        policies.append(PolicyAction(
            setup_type="*",
            action="set_threshold",
            reason=f"PF global={global_metrics.profit_factor:.2f} < 1.0 — subir umbral de score",
            min_score_threshold=0.60,
            confidence=0.90,
            n_trades_basis=global_metrics.n_trades,
        ))

    return policies


# ---------------------------------------------------------------------------
# check_readiness — SystemReadinessReport con 7 criterios
# ---------------------------------------------------------------------------

def check_readiness(
    trades: List[TradeEvent],
    *,
    min_n_trades: int = 300,
    min_months: float = 3.0,
    min_profit_factor: float = 1.5,
    min_calmar: float = 1.5,
    max_dd_pct: float = 15.0,
    min_expectancy_r: float = 0.20,
    min_stability: float = 0.70,
) -> SystemReadinessReport:
    """Evalúa si el sistema está listo para cuenta real (7 criterios).

    Args:
        trades:             lista completa de TradeEvent históricos
        min_n_trades:       mínimo de trades cerrados (default 300)
        min_months:         meses mínimos de historial (default 3.0)
        min_profit_factor:  PF mínimo global (default 1.5)
        min_calmar:         Calmar ratio mínimo (default 1.5)
        max_dd_pct:         drawdown máximo tolerable en % de capital (default 15%)
        min_expectancy_r:   expectancy mínima en R por trade (default 0.2R)
        min_stability:      stability score mínimo (default 0.70)
    """
    metrics = compute_metrics(trades)
    stability = compute_stability_score(trades)

    # Calcular duración del historial
    if trades:
        oldest = min(t.entry_time for t in trades)
        newest = max(t.exit_time for t in trades)
        months_elapsed = (newest - oldest).days / 30.44
    else:
        months_elapsed = 0.0

    # Convertir drawdown_r a % aproximado usando capital medio
    if trades:
        avg_capital = statistics.mean(t.capital_at_entry for t in trades)
        avg_r_initial = statistics.mean(t.r_initial for t in trades) if trades else 1.0
        dd_pct = abs(metrics.max_drawdown_r) * avg_r_initial / avg_capital * 100.0
    else:
        dd_pct = 0.0

    criteria = [
        ReadinessCriterion(
            name="n_trades",
            description="Mínimo de trades cerrados en historial",
            value=float(metrics.n_trades),
            threshold=float(min_n_trades),
            passed=metrics.n_trades >= min_n_trades,
            unit="trades",
            weight=1.5,
        ),
        ReadinessCriterion(
            name="history_months",
            description="Meses de historial de trading",
            value=round(months_elapsed, 2),
            threshold=min_months,
            passed=months_elapsed >= min_months,
            unit="months",
            weight=1.5,
        ),
        ReadinessCriterion(
            name="profit_factor",
            description="Profit Factor global (winners_R / |losers_R|)",
            value=round(metrics.profit_factor, 3),
            threshold=min_profit_factor,
            passed=metrics.profit_factor >= min_profit_factor,
            unit="",
            weight=2.0,
        ),
        ReadinessCriterion(
            name="calmar_ratio",
            description="Calmar Ratio (retorno anualizado / max drawdown)",
            value=round(metrics.calmar_ratio, 3),
            threshold=min_calmar,
            passed=metrics.calmar_ratio >= min_calmar,
            unit="",
            weight=1.5,
        ),
        ReadinessCriterion(
            name="max_drawdown_pct",
            description="Max Drawdown como % del capital (menor es mejor)",
            value=round(dd_pct, 2),
            threshold=max_dd_pct,
            passed=dd_pct <= max_dd_pct,
            unit="%",
            weight=2.0,
        ),
        ReadinessCriterion(
            name="expectancy_r",
            description="Expectancy por trade en R-múltiplos",
            value=round(metrics.expectancy_r, 4),
            threshold=min_expectancy_r,
            passed=metrics.expectancy_r >= min_expectancy_r,
            unit="R",
            weight=2.0,
        ),
        ReadinessCriterion(
            name="stability_score",
            description="Estabilidad temporal (first-half vs second-half)",
            value=round(stability, 4),
            threshold=min_stability,
            passed=stability >= min_stability,
            unit="",
            weight=1.0,
        ),
    ]

    passed_names = [c.name for c in criteria if c.passed]
    failed_names = [c.name for c in criteria if not c.passed]
    all_ready = len(failed_names) == 0

    if all_ready:
        summary = (
            f"Sistema LISTO para live. "
            f"PF={metrics.profit_factor:.2f}, "
            f"Calmar={metrics.calmar_ratio:.2f}, "
            f"E[R]={metrics.expectancy_r:.3f}R, "
            f"Stability={stability:.2f}, "
            f"n={metrics.n_trades}"
        )
        next_step = "Activar cuenta real con tamaño mínimo (0.5% riesgo/trade). Monitorear diario."
    else:
        n_fail = len(failed_names)
        summary = (
            f"Sistema NO LISTO ({n_fail} criterio(s) fallando: {', '.join(failed_names)}). "
            f"n={metrics.n_trades}/{min_n_trades} trades."
        )
        # Consejo según qué falla primero
        if "n_trades" in failed_names:
            gap = int(min_n_trades - metrics.n_trades)
            next_step = f"Faltan {gap} trades más en paper/simulación antes de evaluar."
        elif "history_months" in failed_names:
            gap = round(min_months - months_elapsed, 1)
            next_step = f"Continuar {gap} mes(es) más de historial."
        elif "profit_factor" in failed_names:
            next_step = f"Mejorar PF de {metrics.profit_factor:.2f} a ≥{min_profit_factor}. Revisar setups no rentables."
        elif "stability_score" in failed_names:
            next_step = "Revisar degradación temporal — posible overfitting o cambio de régimen."
        else:
            next_step = f"Corregir criterios fallidos: {', '.join(failed_names)}."

    return SystemReadinessReport(
        ready=all_ready,
        criteria=criteria,
        passed=passed_names,
        failed=failed_names,
        evaluated_at=datetime.utcnow(),
        n_trades_evaluated=metrics.n_trades,
        summary=summary,
        next_step=next_step,
    )


# ---------------------------------------------------------------------------
# build_learning_report — construye LearningReport completo
# ---------------------------------------------------------------------------

def build_learning_report(
    trades: List[TradeEvent],
    analysis_date,
    period_start: datetime,
    period_end: datetime,
) -> LearningReport:
    """Construye un LearningReport completo a partir de una lista de trades."""
    global_metrics = compute_metrics(trades)
    stability = compute_stability_score(trades)

    by_setup = metrics_by_setup(trades)
    by_symbol = metrics_by_symbol(trades)
    by_timeframe = metrics_by_timeframe(trades)
    by_regime = metrics_by_regime(trades)
    by_asset_class = metrics_by_asset_class(trades)
    by_hour = metrics_by_hour(trades)

    error_patterns = detect_error_patterns(trades)
    success_patterns = detect_success_patterns(trades)
    proposed = propose_policies(trades, global_metrics, by_setup)

    warnings: List[str] = []
    if global_metrics.n_trades < 30:
        warnings.append(
            f"Solo {global_metrics.n_trades} trades — métricas poco confiables (mín recomendado: 30)"
        )

    return LearningReport(
        analysis_date=analysis_date,
        period_start=period_start,
        period_end=period_end,
        n_trades_analyzed=len(trades),
        global_metrics=global_metrics,
        by_setup=by_setup,
        by_symbol=by_symbol,
        by_timeframe=by_timeframe,
        by_regime=by_regime,
        by_asset_class=by_asset_class,
        by_hour=by_hour,
        error_patterns=error_patterns,
        success_patterns=success_patterns,
        stability_score=stability,
        proposed_policies=proposed,
        warnings=warnings,
    )
