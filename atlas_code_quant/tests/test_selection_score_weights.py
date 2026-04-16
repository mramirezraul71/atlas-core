from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from scanner.opportunity_scanner import calculate_selection_score  # noqa: E402


def test_calculate_selection_score_legacy_equivalent_weights() -> None:
    """Con pesos antiguos (0.44…0.06) el score coincide con la fórmula inline previa."""
    lq, rs, strength = 70.0, 50.0, 0.8
    alignment = 60.0
    evidence = 0.75
    of_score, of_align = 55.0, 60.0
    bias = 0.0
    old = round(
        (lq * 0.44)
        + (rs * 0.14)
        + (strength * 100.0 * 0.12)
        + (alignment * 0.10)
        + (evidence * 100.0 * 0.06)
        + (of_score * 0.08)
        + (of_align * 0.06)
        + bias,
        2,
    )
    new = calculate_selection_score(
        weight_lq=0.44,
        weight_rs=0.14,
        weight_strength=0.12,
        weight_alignment=0.10,
        weight_evidence=0.06,
        weight_order_flow=0.08,
        weight_ofa=0.06,
        weight_xgboost_conf=0.0,
        weight_ichimoku_conf=0.0,
        weight_adx_adjust=0.0,
        local_quality_score_pct=lq,
        relative_strength_pct=rs,
        strength=strength,
        alignment_score=alignment,
        evidence_score=evidence,
        order_flow_score_pct=of_score,
        order_flow_alignment_score=of_align,
        xgboost_confidence=0.0,
        ichimoku_confidence=0.5,
        adx_adjustment=0.0,
        adaptive_bias=bias,
    )
    assert new == old


def test_trading_config_weights_sum_normalized() -> None:
    from config.settings import TradingConfig

    cfg = TradingConfig()
    s = (
        cfg.scanner_weight_lq
        + cfg.scanner_weight_rs
        + cfg.scanner_weight_strength
        + cfg.scanner_weight_alignment
        + cfg.scanner_weight_evidence
        + cfg.scanner_weight_order_flow
        + cfg.scanner_weight_ofa
        + cfg.scanner_weight_xgboost_conf
        + cfg.scanner_weight_ichimoku_conf
        + cfg.scanner_weight_adx_adjust
    )
    assert 0.99 <= s <= 1.01
