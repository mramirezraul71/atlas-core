"""Tests unitarios para el módulo de momentum entry timing.

Cubre:
  - BarAggregator: agregación de ticks, cierre de velas, avg_volume, seconds_to_close
  - Detección de setups: breakout, pullback, inside-bar
  - should_open_momentum_long / short: integración de filtros
  - ENTRY_MODE: on_close vs intrabar
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock

import pytest

from atlas_code_quant.pipeline.indicators import Bar, BarAggregator
from atlas_code_quant.strategy.momentum import (
    BarState,
    MomentumConfig,
    detect_breakout_long,
    detect_breakout_short,
    detect_inside_bar,
    detect_pullback_long,
    detect_pullback_short,
    should_open_momentum_long,
    should_open_momentum_short,
)


# ── Helpers de fixtures ───────────────────────────────────────────────────────

def make_bar(open_=100.0, high=105.0, low=98.0, close=103.0, volume=1000.0,
             ts_open=0.0, is_closed=True) -> Bar:
    b = Bar(open=open_, high=high, low=low, close=close,
            volume=volume, ts_open=ts_open, ts_close=ts_open + 60)
    b.is_closed = is_closed
    return b


def make_tech(rsi=60.0) -> MagicMock:
    t = MagicMock()
    t.rsi_14 = rsi
    return t


def make_cvd(delta_imbalance=0.1) -> MagicMock:
    c = MagicMock()
    c.delta_imbalance = delta_imbalance
    return c


def make_regime() -> MagicMock:
    return MagicMock()


def make_bar_state(last=None, prev=None, current=None,
                   secs_left=3.0, avg_vol=1000.0) -> BarState:
    return BarState(last=last, prev=prev, current=current,
                    secs_left=secs_left, avg_vol=avg_vol)


# ── TestBarAggregator ─────────────────────────────────────────────────────────

class TestBarAggregator:
    """Tests de BarAggregator."""

    def test_first_tick_no_close(self):
        agg = BarAggregator(bar_seconds=60)
        closed = agg.update(100.0, 101.0, 99.0, 500.0, ts=0.0)
        assert closed is None
        assert agg.last_closed_bar is None
        assert agg.current_bar is not None
        assert agg.current_bar.open == 100.0

    def test_same_interval_updates_ohlcv(self):
        agg = BarAggregator(bar_seconds=60)
        agg.update(100.0, 101.0, 99.0, 500.0, ts=0.0)
        agg.update(102.0, 103.0, 98.0, 300.0, ts=30.0)
        cur = agg.current_bar
        assert cur.high == 103.0
        assert cur.low == 98.0
        assert cur.close == 102.0
        assert cur.volume == 800.0

    def test_new_interval_closes_bar(self):
        agg = BarAggregator(bar_seconds=60)
        agg.update(100.0, 101.0, 99.0, 500.0, ts=0.0)
        closed = agg.update(105.0, 106.0, 104.0, 200.0, ts=60.0)
        assert closed is not None
        assert closed.is_closed is True
        assert closed.open == 100.0
        assert closed.close == 100.0   # close fue el último update en ese intervalo
        assert agg.last_closed_bar is closed
        assert agg.current_bar.open == 105.0

    def test_prev_closed_bar(self):
        agg = BarAggregator(bar_seconds=60)
        agg.update(100.0, 101.0, 99.0, 500.0, ts=0.0)
        agg.update(105.0, 106.0, 104.0, 200.0, ts=60.0)
        agg.update(110.0, 111.0, 109.0, 300.0, ts=120.0)
        assert agg.n_closed() == 2
        assert agg.prev_closed_bar is not None
        assert agg.last_closed_bar.open == 105.0

    def test_avg_volume(self):
        agg = BarAggregator(bar_seconds=60)
        for i in range(5):
            ts_open = i * 60.0
            agg.update(100.0, 101.0, 99.0, 1000.0, ts=ts_open)
            agg.update(100.0, 101.0, 99.0, 0.0, ts=ts_open + 59.0)
        # Forzar cierre de todas las barras
        agg.update(100.0, 101.0, 99.0, 0.0, ts=5 * 60.0)
        avg = agg.avg_volume(5)
        assert avg > 0

    def test_seconds_to_close(self):
        agg = BarAggregator(bar_seconds=60)
        # Usar boundary exacto para que la barra inicie en ts=960 (1000//60*60)
        bar_start = float(int(1000.0 // 60) * 60)  # = 960.0
        agg.update(100.0, 101.0, 99.0, 500.0, ts=bar_start)
        secs = agg.seconds_to_close(ts=bar_start + 10.0)
        assert abs(secs - 50.0) < 1.0

    def test_seconds_to_close_no_current(self):
        agg = BarAggregator(bar_seconds=60)
        assert agg.seconds_to_close() == 60.0

    def test_n_closed_increments(self):
        agg = BarAggregator(bar_seconds=60)
        agg.update(100.0, 101.0, 99.0, 500.0, ts=0.0)
        agg.update(105.0, 106.0, 104.0, 200.0, ts=60.0)
        agg.update(110.0, 111.0, 109.0, 300.0, ts=120.0)
        assert agg.n_closed() == 2


# ── TestDetectBreakout ────────────────────────────────────────────────────────

class TestDetectBreakout:
    def test_breakout_long_true(self):
        prev = make_bar(high=105.0)
        last = make_bar(close=106.0, volume=1600.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        cfg = MomentumConfig(breakout_vol_mult=1.5)
        assert detect_breakout_long(bs, cfg) is True

    def test_breakout_long_close_not_above_prev_high(self):
        prev = make_bar(high=105.0)
        last = make_bar(close=104.0, volume=2000.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        cfg = MomentumConfig(breakout_vol_mult=1.5)
        assert detect_breakout_long(bs, cfg) is False

    def test_breakout_long_insufficient_volume(self):
        prev = make_bar(high=105.0)
        last = make_bar(close=106.0, volume=1000.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        cfg = MomentumConfig(breakout_vol_mult=1.5)
        assert detect_breakout_long(bs, cfg) is False

    def test_breakout_short_true(self):
        prev = make_bar(low=95.0)
        last = make_bar(close=94.0, volume=1600.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        cfg = MomentumConfig(breakout_vol_mult=1.5)
        assert detect_breakout_short(bs, cfg) is True

    def test_breakout_short_close_not_below_prev_low(self):
        prev = make_bar(low=95.0)
        last = make_bar(close=96.0, volume=2000.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        cfg = MomentumConfig(breakout_vol_mult=1.5)
        assert detect_breakout_short(bs, cfg) is False

    def test_breakout_no_prev_bar_returns_false(self):
        last = make_bar(close=106.0, volume=2000.0)
        bs = make_bar_state(last=last, prev=None, avg_vol=1000.0)
        cfg = MomentumConfig()
        assert detect_breakout_long(bs, cfg) is False
        assert detect_breakout_short(bs, cfg) is False


# ── TestDetectPullback ────────────────────────────────────────────────────────

class TestDetectPullback:
    def test_pullback_long_true(self):
        # prev cerró alcista y rompió high; last retrocedió hasta prev.high y cerró arriba
        prev = make_bar(open_=100.0, close=106.0, high=106.0)
        last = make_bar(open_=106.5, low=105.5, close=107.0, volume=1300.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        cfg = MomentumConfig(pullback_vol_mult=1.2)
        assert detect_pullback_long(bs, cfg) is True

    def test_pullback_long_prev_not_bullish(self):
        prev = make_bar(open_=106.0, close=100.0, high=106.0)   # bajista
        last = make_bar(open_=100.5, low=105.5, close=107.0, volume=1300.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        cfg = MomentumConfig(pullback_vol_mult=1.2)
        assert detect_pullback_long(bs, cfg) is False

    def test_pullback_short_true(self):
        # prev cerró bajista; last rebotó al low de prev (resistencia) y cerró abajo
        prev = make_bar(open_=106.0, close=100.0, low=100.0)
        last = make_bar(open_=99.0, high=100.5, close=99.0, volume=1300.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        cfg = MomentumConfig(pullback_vol_mult=1.2)
        assert detect_pullback_short(bs, cfg) is True

    def test_pullback_short_prev_not_bearish(self):
        prev = make_bar(open_=100.0, close=106.0, low=100.0)   # alcista
        last = make_bar(open_=99.0, high=100.5, close=99.0, volume=1300.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        cfg = MomentumConfig(pullback_vol_mult=1.2)
        assert detect_pullback_short(bs, cfg) is False


# ── TestDetectInsideBar ───────────────────────────────────────────────────────

class TestDetectInsideBar:
    def test_inside_bar_true(self):
        prev = make_bar(high=110.0, low=95.0)
        last = make_bar(high=108.0, low=97.0)
        bs = make_bar_state(last=last, prev=prev)
        assert detect_inside_bar(bs) is True

    def test_inside_bar_high_exceeds_prev(self):
        prev = make_bar(high=110.0, low=95.0)
        last = make_bar(high=111.0, low=97.0)
        bs = make_bar_state(last=last, prev=prev)
        assert detect_inside_bar(bs) is False

    def test_inside_bar_low_below_prev(self):
        prev = make_bar(high=110.0, low=95.0)
        last = make_bar(high=108.0, low=94.0)
        bs = make_bar_state(last=last, prev=prev)
        assert detect_inside_bar(bs) is False

    def test_inside_bar_no_bars_false(self):
        bs = make_bar_state(last=None, prev=None)
        assert detect_inside_bar(bs) is False


# ── TestShouldOpenMomentumLong ────────────────────────────────────────────────

class TestShouldOpenMomentumLong:
    """Tests de integración para should_open_momentum_long."""

    def _default_cfg(self, entry_mode="intrabar") -> MomentumConfig:
        return MomentumConfig(
            entry_mode=entry_mode,
            breakout_vol_mult=1.5,
            rsi_bull_min=55.0,
            cvd_imbalance_min=0.05,
        )

    def test_breakout_long_all_filters_pass(self):
        prev = make_bar(high=105.0)
        last = make_bar(close=106.0, volume=1600.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        tech = make_tech(rsi=60.0)
        cvd  = make_cvd(delta_imbalance=0.10)
        cfg  = self._default_cfg()
        assert should_open_momentum_long(bs, tech, cvd, make_regime(), cfg) is True

    def test_rsi_too_low_returns_false(self):
        prev = make_bar(high=105.0)
        last = make_bar(close=106.0, volume=1600.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        tech = make_tech(rsi=40.0)   # below rsi_bull_min=55
        cvd  = make_cvd(delta_imbalance=0.10)
        cfg  = self._default_cfg()
        assert should_open_momentum_long(bs, tech, cvd, make_regime(), cfg) is False

    def test_cvd_too_low_returns_false(self):
        prev = make_bar(high=105.0)
        last = make_bar(close=106.0, volume=1600.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        tech = make_tech(rsi=60.0)
        cvd  = make_cvd(delta_imbalance=0.01)   # below 0.05
        cfg  = self._default_cfg()
        assert should_open_momentum_long(bs, tech, cvd, make_regime(), cfg) is False

    def test_no_setup_returns_false(self):
        # Prev high > last close → no breakout; no pullback; no inside
        prev = make_bar(open_=100.0, close=104.0, high=110.0, low=95.0)
        last = make_bar(open_=103.0, close=104.0, high=115.0, low=93.0,
                        volume=1600.0)   # outside bar → no inside; close < prev.high
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        tech = make_tech(rsi=60.0)
        cvd  = make_cvd(delta_imbalance=0.10)
        cfg  = self._default_cfg()
        assert should_open_momentum_long(bs, tech, cvd, make_regime(), cfg) is False

    def test_on_close_mode_timing_too_early_returns_false(self):
        prev = make_bar(high=105.0)
        last = make_bar(close=106.0, volume=1600.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0,
                            secs_left=30.0)   # 30s left → no es on_close window
        tech = make_tech(rsi=60.0)
        cvd  = make_cvd(delta_imbalance=0.10)
        cfg  = MomentumConfig(entry_mode="on_close", on_close_window_s=5,
                              breakout_vol_mult=1.5, rsi_bull_min=55.0,
                              cvd_imbalance_min=0.05)
        assert should_open_momentum_long(bs, tech, cvd, make_regime(), cfg) is False

    def test_on_close_mode_timing_in_window_returns_true(self):
        prev = make_bar(high=105.0)
        last = make_bar(close=106.0, volume=1600.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0,
                            secs_left=3.0)   # dentro de la ventana de 5s
        tech = make_tech(rsi=60.0)
        cvd  = make_cvd(delta_imbalance=0.10)
        cfg  = MomentumConfig(entry_mode="on_close", on_close_window_s=5,
                              breakout_vol_mult=1.5, rsi_bull_min=55.0,
                              cvd_imbalance_min=0.05)
        assert should_open_momentum_long(bs, tech, cvd, make_regime(), cfg) is True

    def test_inside_bar_green_triggers_long(self):
        prev = make_bar(high=110.0, low=95.0)
        last = make_bar(open_=100.0, close=103.0, high=108.0, low=97.0,
                        volume=1200.0)   # inside + cierra verde
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        tech = make_tech(rsi=60.0)
        cvd  = make_cvd(delta_imbalance=0.10)
        cfg  = self._default_cfg()
        assert should_open_momentum_long(bs, tech, cvd, make_regime(), cfg) is True

    def test_no_last_bar_returns_false(self):
        bs = make_bar_state(last=None, prev=None)
        assert should_open_momentum_long(
            bs, make_tech(), make_cvd(), make_regime()
        ) is False


# ── TestShouldOpenMomentumShort ───────────────────────────────────────────────

class TestShouldOpenMomentumShort:
    def _default_cfg(self, entry_mode="intrabar") -> MomentumConfig:
        return MomentumConfig(
            entry_mode=entry_mode,
            breakout_vol_mult=1.5,
            rsi_bear_max=45.0,
            cvd_imbalance_max=-0.05,
        )

    def test_breakout_short_all_filters_pass(self):
        prev = make_bar(low=95.0)
        last = make_bar(close=94.0, volume=1600.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        tech = make_tech(rsi=40.0)
        cvd  = make_cvd(delta_imbalance=-0.10)
        cfg  = self._default_cfg()
        assert should_open_momentum_short(bs, tech, cvd, make_regime(), cfg) is True

    def test_rsi_too_high_returns_false(self):
        prev = make_bar(low=95.0)
        last = make_bar(close=94.0, volume=1600.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        tech = make_tech(rsi=60.0)   # above rsi_bear_max=45
        cvd  = make_cvd(delta_imbalance=-0.10)
        cfg  = self._default_cfg()
        assert should_open_momentum_short(bs, tech, cvd, make_regime(), cfg) is False

    def test_cvd_not_bearish_returns_false(self):
        prev = make_bar(low=95.0)
        last = make_bar(close=94.0, volume=1600.0)
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        tech = make_tech(rsi=40.0)
        cvd  = make_cvd(delta_imbalance=0.10)   # positivo → no bearish
        cfg  = self._default_cfg()
        assert should_open_momentum_short(bs, tech, cvd, make_regime(), cfg) is False

    def test_inside_bar_red_triggers_short(self):
        prev = make_bar(high=110.0, low=95.0)
        last = make_bar(open_=103.0, close=100.0, high=108.0, low=97.0,
                        volume=1200.0)   # inside + cierra rojo
        bs = make_bar_state(last=last, prev=prev, avg_vol=1000.0)
        tech = make_tech(rsi=40.0)
        cvd  = make_cvd(delta_imbalance=-0.10)
        cfg  = self._default_cfg()
        assert should_open_momentum_short(bs, tech, cvd, make_regime(), cfg) is True

    def test_no_last_bar_returns_false(self):
        bs = make_bar_state(last=None, prev=None)
        assert should_open_momentum_short(
            bs, make_tech(rsi=30.0), make_cvd(delta_imbalance=-0.2), make_regime()
        ) is False


# ── TestBarAggregatorIntegration ──────────────────────────────────────────────

class TestBarAggregatorIntegration:
    """Test de integración: BarAggregator → BarState → should_open_momentum_long."""

    def test_end_to_end_breakout_long(self):
        agg = BarAggregator(bar_seconds=60)
        # Barra 0: [0s, 59s] → prev (vol bajo: 400+400=800)
        agg.update(100.0, 102.0, 99.0, 400.0, ts=0.0)
        agg.update(101.0, 104.0, 99.0, 400.0, ts=30.0)
        # Barra 1: [60s, 119s] → last (breakout: close > prev.high=104, vol alto: 1600+800=2400)
        # avg_vol después de barra 0 = 800 → need last.vol >= 800*1.5=1200 ✓
        agg.update(105.0, 107.0, 104.5, 1600.0, ts=60.0)
        agg.update(106.0, 108.0, 104.0, 800.0, ts=90.0)
        # Barra 2 (fuerza cierre de barra 1)
        agg.update(107.0, 109.0, 106.0, 200.0, ts=120.0)

        assert agg.n_closed() == 2

        bar_state = BarState(
            last      = agg.last_closed_bar,
            prev      = agg.prev_closed_bar,
            current   = agg.current_bar,
            secs_left = agg.seconds_to_close(ts=125.0),
            avg_vol   = agg.avg_volume(20),
        )

        tech = make_tech(rsi=62.0)
        cvd  = make_cvd(delta_imbalance=0.15)
        cfg  = MomentumConfig(
            entry_mode="intrabar",
            breakout_vol_mult=1.5,
            rsi_bull_min=55.0,
            cvd_imbalance_min=0.05,
        )

        result = should_open_momentum_long(bar_state, tech, cvd, make_regime(), cfg)
        assert result is True
