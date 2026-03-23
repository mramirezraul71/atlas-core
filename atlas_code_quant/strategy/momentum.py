"""Módulo de momentum entry timing — velas 1m derivadas de ticks 5s.

Tres estilos de setup soportados:
  - breakout:  close de la última vela supera el high/low de la previa con vol
  - pullback:  retroceso al nivel recién roto, luego reanudación en dirección
  - inside_bar: vela contenida en el rango previo (caja) → breakout de la caja

ENTRY_MODE:
  "on_close"  → sólo permite entrada en los últimos on_close_window_s seg de la vela
  "intrabar"  → permite entrada en cualquier momento de la vela activa

Uso::

    from atlas_code_quant.strategy.momentum import (
        MomentumConfig, BarState, should_open_momentum_long,
        should_open_momentum_short,
    )
    from atlas_code_quant.pipeline.indicators import BarAggregator

    agg = BarAggregator(bar_seconds=60)
    for tick in ticks:
        agg.update(tick.close, tick.high, tick.low, tick.vol)

    bar_state = BarState(
        last      = agg.last_closed_bar,
        prev      = agg.prev_closed_bar,
        current   = agg.current_bar,
        secs_left = agg.seconds_to_close(),
        avg_vol   = agg.avg_volume(20),
    )
    if should_open_momentum_long(bar_state, tech, cvd, regime):
        ...
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from atlas_code_quant.pipeline.indicators import Bar

# ── Configuración de entorno ──────────────────────────────────────────────────
_ENTRY_MODE = os.getenv("ATLAS_ENTRY_MODE", "on_close").strip().lower()


@dataclass
class MomentumConfig:
    """Parámetros del filtro de momentum — todos sobreescribibles via env."""
    entry_mode:         str   = _ENTRY_MODE   # "on_close" | "intrabar"
    on_close_window_s:  int   = 5             # ventana final de vela (solo en on_close)
    breakout_vol_mult:  float = 1.5           # vol >= avg_vol * mult para breakout
    pullback_vol_mult:  float = 1.2           # vol para setup pullback
    inside_vol_mult:    float = 1.2           # vol para setup inside-bar
    rsi_bull_min:       float = 55.0          # RSI mínimo para long
    rsi_bear_max:       float = 45.0          # RSI máximo para short
    cvd_imbalance_min:  float = 0.05          # delta_imbalance mínimo (long)
    cvd_imbalance_max:  float = -0.05         # delta_imbalance máximo (short)


@dataclass
class BarState:
    """Snapshot del estado de velas para evaluación de momentum en un ciclo."""
    last:      Optional["Bar"] = None   # última vela cerrada
    prev:      Optional["Bar"] = None   # penúltima vela cerrada
    current:   Optional["Bar"] = None   # vela en formación
    secs_left: float = 60.0             # segundos hasta cierre de vela actual
    avg_vol:   float = 0.0              # volumen promedio (últimas ~20 velas)


# ── Timing ────────────────────────────────────────────────────────────────────

def _entry_timing_ok(bar_state: BarState, cfg: MomentumConfig) -> bool:
    """True si el timing de la vela permite abrir una entrada."""
    if cfg.entry_mode == "on_close":
        return bar_state.secs_left <= cfg.on_close_window_s
    return True   # intrabar: siempre permitido


# ── Detección de setups (pure functions) ─────────────────────────────────────

def detect_breakout_long(bar_state: BarState, cfg: MomentumConfig) -> bool:
    """Breakout alcista: última vela cierra por encima del high previo con volumen.

    Condiciones:
      - last.close > prev.high
      - last.volume >= avg_vol * breakout_vol_mult
    """
    last = bar_state.last
    prev = bar_state.prev
    if last is None or prev is None:
        return False
    vol_ok = (bar_state.avg_vol > 0 and
              last.volume >= bar_state.avg_vol * cfg.breakout_vol_mult)
    return last.close > prev.high and vol_ok


def detect_breakout_short(bar_state: BarState, cfg: MomentumConfig) -> bool:
    """Breakout bajista: última vela cierra por debajo del low previo con volumen."""
    last = bar_state.last
    prev = bar_state.prev
    if last is None or prev is None:
        return False
    vol_ok = (bar_state.avg_vol > 0 and
              last.volume >= bar_state.avg_vol * cfg.breakout_vol_mult)
    return last.close < prev.low and vol_ok


def detect_pullback_long(bar_state: BarState, cfg: MomentumConfig) -> bool:
    """Pullback alcista: la vela previa rompió hacia arriba y la última
    retrocedió al high previo sin cerrarlo abajo (soporte dinámico).

    Condiciones:
      - prev cerró alcista (close > open)
      - last.low tocó el high de prev (pullback)
      - last.close > prev.high (reanuda la subida)
      - volumen suficiente
    """
    last = bar_state.last
    prev = bar_state.prev
    if last is None or prev is None:
        return False
    prev_broke_up  = prev.close > prev.open
    pullback_hold  = last.low <= prev.high and last.close > prev.high
    vol_ok = (bar_state.avg_vol > 0 and
              last.volume >= bar_state.avg_vol * cfg.pullback_vol_mult)
    return prev_broke_up and pullback_hold and vol_ok


def detect_pullback_short(bar_state: BarState, cfg: MomentumConfig) -> bool:
    """Pullback bajista: la vela previa rompió hacia abajo y la última
    rebotó al low previo (resistencia dinámica) para continuar bajando.

    Condiciones:
      - prev cerró bajista (close < open)
      - last.high tocó low de prev (rebote)
      - last.close < prev.low (reanuda la bajada)
      - volumen suficiente
    """
    last = bar_state.last
    prev = bar_state.prev
    if last is None or prev is None:
        return False
    prev_broke_down = prev.close < prev.open
    pullback_hold   = last.high >= prev.low and last.close < prev.low
    vol_ok = (bar_state.avg_vol > 0 and
              last.volume >= bar_state.avg_vol * cfg.pullback_vol_mult)
    return prev_broke_down and pullback_hold and vol_ok


def detect_inside_bar(bar_state: BarState) -> bool:
    """Inside bar: el rango de la última vela está completamente contenido
    en el rango de la vela previa (caja de compresión).

    Condición:
      - last.high <= prev.high  AND  last.low >= prev.low
    """
    last = bar_state.last
    prev = bar_state.prev
    if last is None or prev is None:
        return False
    return last.high <= prev.high and last.low >= prev.low


# ── API pública: should_open_momentum_long / short ────────────────────────────

def should_open_momentum_long(
    bar_state: BarState,
    tech,                          # TechnicalSnapshot
    cvd,                           # CVDSnapshot
    regime,                        # RegimeOutput
    cfg: MomentumConfig | None = None,
    now_ts: float | None = None,
) -> bool:
    """Evalúa si existe un setup de momentum LONG válido.

    Retorna True cuando:
      1. Timing OK (dentro de ventana on_close, o intrabar)
      2. RSI >= rsi_bull_min
      3. CVD.delta_imbalance >= cvd_imbalance_min  (presión compradora)
      4. Al menos un setup confirmado:
           - Breakout long
           - Pullback long
           - Inside-bar que cerró verde (close > open)
    """
    if cfg is None:
        cfg = MomentumConfig()

    if bar_state.last is None:
        return False

    if not _entry_timing_ok(bar_state, cfg):
        return False

    # RSI
    if tech.rsi_14 < cfg.rsi_bull_min:
        return False

    # CVD
    cvd_val = getattr(cvd, "delta_imbalance", 0.0)
    if cvd_val < cfg.cvd_imbalance_min:
        return False

    # Setup de vela — basta con uno
    has_breakout = detect_breakout_long(bar_state, cfg)
    has_pullback = detect_pullback_long(bar_state, cfg)
    has_inside   = (detect_inside_bar(bar_state) and
                    bar_state.last.close > bar_state.last.open)   # inside cierra verde

    return has_breakout or has_pullback or has_inside


def should_open_momentum_short(
    bar_state: BarState,
    tech,
    cvd,
    regime,
    cfg: MomentumConfig | None = None,
    now_ts: float | None = None,
) -> bool:
    """Evalúa si existe un setup de momentum SHORT válido.

    Retorna True cuando:
      1. Timing OK
      2. RSI <= rsi_bear_max
      3. CVD.delta_imbalance <= cvd_imbalance_max  (presión vendedora)
      4. Al menos un setup confirmado:
           - Breakout short
           - Pullback short
           - Inside-bar que cerró rojo (close < open)
    """
    if cfg is None:
        cfg = MomentumConfig()

    if bar_state.last is None:
        return False

    if not _entry_timing_ok(bar_state, cfg):
        return False

    # RSI
    if tech.rsi_14 > cfg.rsi_bear_max:
        return False

    # CVD
    cvd_val = getattr(cvd, "delta_imbalance", 0.0)
    if cvd_val > cfg.cvd_imbalance_max:
        return False

    # Setup de vela — basta con uno
    has_breakout = detect_breakout_short(bar_state, cfg)
    has_pullback = detect_pullback_short(bar_state, cfg)
    has_inside   = (detect_inside_bar(bar_state) and
                    bar_state.last.close < bar_state.last.open)   # inside cierra rojo

    return has_breakout or has_pullback or has_inside
