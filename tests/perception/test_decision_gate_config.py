from __future__ import annotations

from atlas_scanner.decision.gate import config_from_env


def test_decision_gate_config_from_env(monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_DECISION_GATE_ENABLED", "true")
    monkeypatch.setenv("ATLAS_DECISION_GATE_MODE", "paper_supervised")
    monkeypatch.setenv("ATLAS_DECISION_GATE_MIN_STRUCTURAL_CONFIDENCE", "52")
    monkeypatch.setenv("ATLAS_DECISION_GATE_MIN_FAST_PRESSURE", "49")
    monkeypatch.setenv("ATLAS_DECISION_GATE_MAX_DIVERGENCE_SCORE", "28")
    monkeypatch.setenv("ATLAS_DECISION_GATE_ALLOW_DEGRADED", "false")
    monkeypatch.setenv("ATLAS_DECISION_GATE_ALLOW_STRUCTURAL_ONLY", "false")
    monkeypatch.setenv("ATLAS_DECISION_GATE_ALLOW_FAST_ONLY", "true")

    cfg = config_from_env()
    assert cfg.enabled is True
    assert cfg.mode == "paper_supervised"
    assert cfg.min_structural_confidence == 52.0
    assert cfg.min_fast_pressure == 49.0
    assert cfg.max_divergence_score == 28.0
    assert cfg.allow_degraded is False
    assert cfg.allow_structural_only is False
    assert cfg.allow_fast_only is True
