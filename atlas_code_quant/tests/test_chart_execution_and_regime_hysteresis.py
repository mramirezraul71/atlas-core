"""Tests ligeros: verificación de navegador e histéresis de régimen."""
from __future__ import annotations

import numpy as np
import pytest

from atlas_code_quant.operations.chart_execution import ChartExecutionService
from atlas_code_quant.models.regime_classifier import (
    MarketRegime,
    RegimeClassifier,
    RegimeOutput,
)


@pytest.mark.parametrize(
    "path,expected",
    [
        (r"C:\Program Files\Google\Chrome\Application\chrome.exe", ["chrome.exe"]),
        (r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe", ["msedge.exe"]),
        (None, ["chrome.exe", "msedge.exe"]),
        (r"C:\Firefox\firefox.exe", ["firefox.exe"]),
        (r"C:\BraveSoftware\Brave-Browser\Application\brave.exe", ["brave.exe"]),
    ],
)
def test_browser_exe_candidates(path: str | None, expected: list[str]) -> None:
    assert ChartExecutionService._browser_exe_candidates(path) == expected


def test_regime_hysteresis_suppresses_weak_switch() -> None:
    clf = RegimeClassifier(use_gpu=False)
    clf._hysteresis_enabled = True
    clf._hysteresis_margin = 0.15
    clf._last_stable_regime = MarketRegime.BULL
    proba = np.array([0.40, 0.46, 0.14], dtype=np.float64)
    out = RegimeOutput(
        regime=MarketRegime.BEAR,
        confidence=0.46,
        proba_bull=0.40,
        proba_bear=0.46,
        proba_sideways=0.14,
        regime_raw=MarketRegime.BEAR,
        hysteresis_applied=False,
    )
    best_idx = 1
    smoothed = clf._apply_hysteresis(out, proba, best_idx)
    assert smoothed.regime == MarketRegime.BULL
    assert smoothed.hysteresis_applied is True
    assert smoothed.regime_raw == MarketRegime.BEAR


def test_regime_hysteresis_allows_strong_switch() -> None:
    clf = RegimeClassifier(use_gpu=False)
    clf._hysteresis_enabled = True
    clf._hysteresis_margin = 0.10
    clf._last_stable_regime = MarketRegime.BULL
    proba = np.array([0.30, 0.62, 0.08], dtype=np.float64)
    out = RegimeOutput(
        regime=MarketRegime.BEAR,
        confidence=0.62,
        proba_bull=0.30,
        proba_bear=0.62,
        proba_sideways=0.08,
        regime_raw=MarketRegime.BEAR,
        hysteresis_applied=False,
    )
    best_idx = 1
    smoothed = clf._apply_hysteresis(out, proba, best_idx)
    assert smoothed.regime == MarketRegime.BEAR
    assert smoothed.hysteresis_applied is False
    assert clf._last_stable_regime == MarketRegime.BEAR


def test_reset_hysteresis() -> None:
    clf = RegimeClassifier(use_gpu=False)
    clf._last_stable_regime = MarketRegime.SIDEWAYS
    clf.reset_hysteresis()
    assert clf._last_stable_regime is None
