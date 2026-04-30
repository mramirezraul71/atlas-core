from __future__ import annotations

from pathlib import Path

from modules.atlas_radar_kalshi.learning_engine import LearningEngine
from modules.atlas_radar_kalshi.signals import EnsembleWeights


def test_learning_engine_records_and_suggests(tmp_path: Path) -> None:
    engine = LearningEngine(tmp_path / "radar_learning_state.json", min_exits_for_adjust=2)
    base = EnsembleWeights(micro=0.3, markov=0.2, llm=0.4, momentum=0.1)

    engine.on_decision("POLY:123", 0.07)
    engine.on_decision("KX_CPI", 0.03)
    engine.on_exit("POLY:123", -50)
    # todavía no llega al umbral
    w0 = engine.suggest_weights(base)
    assert round(w0.llm, 4) == round(base.normalized().llm, 4)

    engine.on_exit("KX_CPI", -20)
    w1 = engine.suggest_weights(base)
    assert w1.llm <= base.normalized().llm
    assert abs((w1.micro + w1.markov + w1.llm + w1.momentum) - 1.0) < 1e-9


def test_learning_engine_persistence(tmp_path: Path) -> None:
    path = tmp_path / "radar_learning_state.json"
    a = LearningEngine(path, min_exits_for_adjust=1)
    a.on_decision("POLY:abc", 0.1)
    a.on_exit("POLY:abc", 120)
    a.save()

    b = LearningEngine(path, min_exits_for_adjust=1)
    snap = b.snapshot()
    assert snap["global_exits"] == 1
    assert snap["global_wins"] == 1

