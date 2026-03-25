"""Tests unitarios — SignalGenerator: activación de opciones.

Cubre:
  - _pick_option_strategy() en todos los paths
  - evaluate() activa use_options=True cuando IV + score cumplen
  - evaluate() mantiene use_options=False cuando IV baja
  - Ejemplo concreto: XOP con iv_rank=72, iv_hv=1.3 → CALL_VERTICAL
"""
from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock
from dataclasses import dataclass, field

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from strategy.signal_generator import SignalGenerator, SignalType, TradeSignal


# ── Helpers de fixtures ────────────────────────────────────────────────────────

def _regime(regime_enum, confidence: float = 0.80):
    r = MagicMock()
    r.regime = regime_enum
    r.confidence = confidence
    return r


def _tech(
    rsi: float = 42.0,
    macd: float = 0.5,
    macd_signal: float = 0.2,
    macd_hist: float = 0.3,
    volume_ratio: float = 2.5,
    atr_20: float = 1.5,
):
    t = MagicMock()
    t.rsi_14 = rsi
    t.macd = macd
    t.macd_signal = macd_signal
    t.macd_hist = macd_hist
    t.volume_ratio = volume_ratio
    t.atr_20 = atr_20
    return t


def _iv(iv_rank: float = 72.0, iv_hv: float = 1.3):
    iv = MagicMock()
    iv.iv_rank_30d = iv_rank
    iv.iv_hv_ratio = iv_hv
    return iv


def _cvd(delta: float = 1.8):
    c = MagicMock()
    c.delta_imbalance = delta
    return c


def _mtf(coherence: float = 0.80):
    m = MagicMock()
    m.coherence_score = coherence
    return m


# ── Import MarketRegime lazy ───────────────────────────────────────────────────

def _get_regimes():
    from atlas_code_quant.models.regime_classifier import MarketRegime
    return MarketRegime


# ── Tests _pick_option_strategy ───────────────────────────────────────────────

class TestPickOptionStrategy:
    def setup_method(self):
        self.gen = SignalGenerator()
        self.MR = _get_regimes()

    def test_bull_buy_returns_call_vertical(self):
        result = self.gen._pick_option_strategy(
            self.MR.BULL, SignalType.BUY, iv_rank=50.0
        )
        assert result == "CALL_VERTICAL"

    def test_bear_sell_returns_put_vertical(self):
        result = self.gen._pick_option_strategy(
            self.MR.BEAR, SignalType.SELL, iv_rank=60.0
        )
        assert result == "PUT_VERTICAL"

    def test_high_iv_returns_short_strangle(self):
        result = self.gen._pick_option_strategy(
            self.MR.BULL, SignalType.BUY, iv_rank=80.0
        )
        assert result == "SHORT_STRANGLE"

    def test_sideways_fallback_single_leg(self):
        result = self.gen._pick_option_strategy(
            self.MR.SIDEWAYS, SignalType.BUY, iv_rank=50.0
        )
        assert result == "SINGLE_LEG"

    def test_high_iv_threshold_boundary(self):
        # Exactamente en el umbral OPT_HIGH_IV_RANK → SHORT_STRANGLE
        result = self.gen._pick_option_strategy(
            self.MR.BULL, SignalType.BUY, iv_rank=SignalGenerator.OPT_HIGH_IV_RANK
        )
        assert result == "SHORT_STRANGLE"

    def test_just_below_high_iv_threshold(self):
        # 74.9 < 75 → CALL_VERTICAL (no SHORT_STRANGLE)
        result = self.gen._pick_option_strategy(
            self.MR.BULL, SignalType.BUY, iv_rank=74.9
        )
        assert result == "CALL_VERTICAL"


# ── Tests evaluate() — activación de opciones ────────────────────────────────

class TestEvaluateOptionsActivation:
    def setup_method(self):
        self.gen = SignalGenerator()
        self.MR = _get_regimes()

    def _evaluate_xop(self, iv_rank: float = 72.0, iv_hv: float = 1.3,
                      signal_score_motif: float = 0.85, signal_score_tin: float = 0.80):
        """Simula evaluate() con parámetros de XOP típicos."""
        # Pre-sembrar CVD history con valores negativos para que delta=1.8
        # genere z-score > CVD_ZSCORE_THRESHOLD (1.0)
        self.gen._cvd_history = [-0.5] * 20

        regime = _regime(self.MR.BULL, confidence=0.82)
        tech   = _tech(rsi=42.0, macd=0.5, macd_signal=0.2, macd_hist=0.3, volume_ratio=2.5)
        cvd    = _cvd(delta=1.8)
        iv     = _iv(iv_rank=iv_rank, iv_hv=iv_hv)
        mtf    = _mtf(coherence=0.82)

        return self.gen.evaluate(
            symbol="XOP",
            regime=regime,
            tech=tech,
            cvd=cvd,
            iv=iv,
            entry_price=130.50,
            ocr_price=130.48,
            current_capital=25_000.0,
            position_size=10,
            mtf_report=mtf,
            motif_edge=signal_score_motif,
            tin_score=signal_score_tin,
        )

    def test_xop_options_activated(self):
        """XOP con iv_rank=72, iv_hv=1.3 → use_options=True, strategy=CALL_VERTICAL."""
        sig = self._evaluate_xop(iv_rank=72.0, iv_hv=1.3)
        assert sig.signal_type == SignalType.BUY
        assert sig.use_options is True
        assert sig.option_strategy_type == "CALL_VERTICAL"

    def test_xop_options_metadata(self):
        """Metadata lleva use_options y option_strategy_type."""
        sig = self._evaluate_xop(iv_rank=72.0, iv_hv=1.3)
        assert sig.metadata["use_options"] is True
        assert sig.metadata["option_strategy_type"] == "CALL_VERTICAL"

    def test_xop_low_iv_equity_fallback(self):
        """XOP con iv_rank=25 → use_options=False (equity directo)."""
        sig = self._evaluate_xop(iv_rank=25.0, iv_hv=0.9)
        # Puede ser BUY o FLAT; si es señal, no debe usar opciones
        if sig.signal_type != SignalType.FLAT:
            assert sig.use_options is False
            assert sig.option_strategy_type == ""

    def test_xop_iv_rank_ok_but_score_too_low(self):
        """IV OK pero signal_score < OPT_MIN_SIGNAL_SCORE → equity."""
        # score bajo: motif=0.3, tin=0.3, mtf=0.5, regime=0.5 → ~0.40
        sig = self._evaluate_xop(iv_rank=72.0, iv_hv=1.3,
                                  signal_score_motif=0.30, signal_score_tin=0.30)
        # Si supera los demás filtros pero score es bajo → equity
        if sig.signal_type == SignalType.BUY:
            assert sig.use_options is False

    def test_xop_high_iv_short_strangle(self):
        """XOP con iv_rank=82 (>75) → SHORT_STRANGLE."""
        sig = self._evaluate_xop(iv_rank=82.0, iv_hv=1.6)
        if sig.signal_type != SignalType.FLAT:
            assert sig.use_options is True
            assert sig.option_strategy_type == "SHORT_STRANGLE"

    def test_iv_hv_ratio_populated_in_signal(self):
        """TradeSignal.iv_hv_ratio se llena con el valor real."""
        sig = self._evaluate_xop(iv_rank=72.0, iv_hv=1.3)
        if sig.signal_type != SignalType.FLAT:
            assert abs(sig.iv_hv_ratio - 1.3) < 0.01


# ── Tests umbrales de entrada bajados ────────────────────────────────────────

class TestLoweredEntryThresholds:
    def setup_method(self):
        self.gen = SignalGenerator()

    def test_min_iv_rank_lowered(self):
        assert SignalGenerator.MIN_IV_RANK == 30.0

    def test_min_iv_hv_ratio_lowered(self):
        assert SignalGenerator.MIN_IV_HV_RATIO == 0.8

    def test_options_thresholds_defined(self):
        assert SignalGenerator.OPT_MIN_IV_RANK   == 40.0
        assert SignalGenerator.OPT_MIN_IV_HV_RATIO == 1.1
        assert SignalGenerator.OPT_MIN_SIGNAL_SCORE == 0.65
        assert SignalGenerator.OPT_HIGH_IV_RANK   == 75.0
