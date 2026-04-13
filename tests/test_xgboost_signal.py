"""Tests módulo XGBoost Signal (ATLAS_PUSH)."""
from __future__ import annotations

import json
import sqlite3
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

ROOT = Path(__file__).resolve().parents[1]
QUANT = ROOT / "atlas_code_quant"
if str(QUANT) not in sys.path:
    sys.path.insert(0, str(QUANT))

from config.settings import settings  # noqa: E402
from learning.xgboost_signal.backtest_engine import compute_ic, run_walk_forward  # noqa: E402
from learning.xgboost_signal.feature_builder import FeatureBuilder, OptionsFeatureBuilder  # noqa: E402
from learning.xgboost_signal.model_loader import XGBoostModelLoader  # noqa: E402
from learning.xgboost_signal.model_trainer import (  # noqa: E402
    _count_real_trades,
    ensure_xgboost_feature_log_table,
    get_training_phase,
)
from learning.xgboost_signal.signal_scorer import XGBoostSignalScorer  # noqa: E402
from learning.xgboost_signal.exit_advisor import XGBoostExitAdvisor  # noqa: E402
from learning.xgboost_signal.audit_report import generate_audit_report  # noqa: E402

pytest.importorskip("xgboost")


def test_get_training_phase() -> None:
    assert get_training_phase(0) == 0
    assert get_training_phase(29) == 0
    assert get_training_phase(30) == 1
    assert get_training_phase(199) == 1
    assert get_training_phase(200) == 2


def test_count_real_trades_empty_db(tmp_path: Path) -> None:
    db = tmp_path / "j.db"
    conn = sqlite3.connect(str(db))
    conn.execute(
        """CREATE TABLE trading_journal (
        id INTEGER PRIMARY KEY,
        status TEXT,
        realized_pnl REAL
    )"""
    )
    conn.commit()
    assert _count_real_trades(conn) == 0
    conn.execute(
        "INSERT INTO trading_journal (status, realized_pnl) VALUES ('closed', NULL)"
    )
    conn.commit()
    assert _count_real_trades(conn) == 0
    conn.execute(
        "INSERT INTO trading_journal (status, realized_pnl) VALUES ('closed', 0)"
    )
    conn.commit()
    assert _count_real_trades(conn) == 0
    conn.execute(
        "INSERT INTO trading_journal (status, realized_pnl) VALUES ('closed', 12.5)"
    )
    conn.commit()
    assert _count_real_trades(conn) == 1
    conn.close()


def test_feature_builder_pretrade() -> None:
    fb = FeatureBuilder()
    s = fb.build_pretrade_features(
        {
            "selection_score_pct": 80,
            "predicted_move_pct": 0.05,
            "local_win_rate_pct": 60,
            "regime": "trending",
            "session_phase": "intraday_core",
        }
    )
    assert "selection_score_pct" in s.index
    assert 0.0 <= float(s["selection_score_pct"]) <= 1.0


def test_options_placeholder() -> None:
    with pytest.raises(NotImplementedError):
        OptionsFeatureBuilder().build_options_features({})


def test_compute_ic() -> None:
    y = np.array([0, 1, 0, 1, 1])
    sc = np.array([0.1, 0.9, 0.2, 0.8, 0.7])
    ic = compute_ic(y, sc)
    assert -1.0 <= ic <= 1.0


def test_score_phase0_disabled(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(settings, "xgboost_enabled", False)
    XGBoostSignalScorer.reset_instance()
    r = XGBoostSignalScorer.get_instance().score({"symbol": "SPY"})
    assert r["score"] is None
    assert r["action"] == "pass"


def test_score_phase0_enabled_no_real_trades(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    db = tmp_path / "j.db"
    monkeypatch.setattr(settings, "journal_db_path", db)
    monkeypatch.setattr(settings, "xgboost_enabled", True)
    monkeypatch.setattr(settings, "xgboost_model_dir", tmp_path / "models")
    settings.xgboost_model_dir.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(db))
    conn.execute(
        "CREATE TABLE trading_journal (id INTEGER PRIMARY KEY, status TEXT, realized_pnl REAL)"
    )
    conn.commit()
    conn.close()

    XGBoostSignalScorer.reset_instance()
    r = XGBoostSignalScorer.get_instance().score({"symbol": "SPY", "selection_score_pct": 70})
    assert r["phase"] == 0
    assert r["score"] is None
    assert r["action"] == "pass"


def test_exit_advisor_override_false(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr(settings, "xgboost_enabled", True)
    monkeypatch.setattr(settings, "journal_db_path", tmp_path / "missing.db")
    XGBoostExitAdvisor.reset_instance()
    out = XGBoostExitAdvisor.get_instance().advise_exit({}, {})
    assert out["override_governance"] is False


def test_audit_report_writes(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(settings, "xgboost_model_dir", tmp_path)
    jp, mp = generate_audit_report(phase=0, n_trades_used=0)
    assert jp.exists() and mp.exists()
    body = json.loads(jp.read_text(encoding="utf-8"))
    assert "metadata" in body
    md = mp.read_text(encoding="utf-8")
    assert "## ALERTS" in md
    assert "[OK]" in md or "[WARN]" in md


def test_model_load_infer_timing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    pytest.importorskip("sklearn")
    import xgboost as xgb
    from sklearn.datasets import make_classification

    X, y = make_classification(n_samples=80, n_features=8, random_state=42)
    clf = xgb.XGBClassifier(n_estimators=20, max_depth=3, random_state=42)
    clf.fit(X, y)
    mp = tmp_path / "xgboost_model.json"
    meta_p = tmp_path / "xgboost_model_meta.json"
    clf.get_booster().save_model(str(mp))
    names = [f"f{i}" for i in range(8)]
    meta_p.write_text(json.dumps({"feature_names": names, "phase": 1}), encoding="utf-8")

    monkeypatch.setattr(settings, "xgboost_model_dir", tmp_path)
    XGBoostModelLoader.reset_instance()
    loader = XGBoostModelLoader.get_instance()
    t0 = time.perf_counter()
    assert loader.load(force=True) is True
    load_ms = (time.perf_counter() - t0) * 1000
    assert load_ms < 3000

    vec = np.random.randn(8).astype(np.float64)
    t1 = time.perf_counter()
    for _ in range(20):
        loader.predict_proba_positive(vec)
    inf_ms = (time.perf_counter() - t1) / 20 * 1000
    assert inf_ms < 150


def test_walk_forward_synthetic_df() -> None:
    rows = []
    for i in range(40):
        rows.append(
            {
                "entry_time": pd.Timestamp("2025-01-01") + pd.Timedelta(days=i),
                "target_win": i % 2,
                "selection_score_pct": 0.5 + 0.01 * (i % 5),
                "local_win_rate_pct": 0.55,
                "predicted_move_pct": 0.01,
                "iv_rank_pct": 0.3,
                "confidence_pct": 0.6,
                "regime_trending": 1.0,
                "regime_sideways": 0.0,
                "regime_high_volatility": 0.0,
                "holding_hours": 1.0,
            }
        )
    df = pd.DataFrame(rows)
    wf = run_walk_forward(df, phase=1)
    assert wf.folds is not None
