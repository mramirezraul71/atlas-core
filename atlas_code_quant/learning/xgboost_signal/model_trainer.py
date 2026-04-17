"""Entrenamiento XGBoost por fase (walk-forward en backtest_engine)."""
from __future__ import annotations

import json
import logging
import sqlite3
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import pandas as pd
import xgboost as xgb
from sklearn.calibration import CalibratedClassifierCV
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import roc_auc_score
from sklearn.model_selection import train_test_split

from config.settings import settings
from learning.xgboost_signal.backtest_engine import run_walk_forward
from learning.xgboost_signal.feature_builder import FeatureBuilder

logger = logging.getLogger("quant.xgboost_signal.trainer")

# Criterio operativo: PnL realizado distinto de cero (no depende de broker_order_ids_json).
COUNT_REAL_TRADES_SQL = """
SELECT COUNT(*) FROM trading_journal
WHERE status = 'closed'
AND realized_pnl IS NOT NULL
AND realized_pnl != 0
"""


def _count_real_trades(conn: sqlite3.Connection) -> int:
    cur = conn.cursor()
    try:
        cur.execute(COUNT_REAL_TRADES_SQL)
        row = cur.fetchone()
        return int(row[0] if row else 0)
    except sqlite3.Error as e:
        logger.warning("count_real_trades: %s", e)
        return 0


def ensure_xgboost_feature_log_table(conn: sqlite3.Connection) -> None:
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS xgboost_feature_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            trade_id TEXT,
            timestamp TEXT,
            features_json TEXT,
            target_win INTEGER,
            phase INTEGER DEFAULT 0
        )
        """
    )
    conn.commit()


def get_training_phase(n_real_trades: int) -> int:
    if n_real_trades < 30:
        return 0
    if n_real_trades < 200:
        return 1
    return 2


@dataclass
class TrainResult:
    phase: int
    n_real_trades: int
    model_path: str | None = None
    meta_path: str | None = None
    metrics: dict[str, Any] = field(default_factory=dict)
    message: str = ""


def _safe_stratify_labels(y: Any) -> Any | None:
    series = pd.Series(y)
    counts = series.value_counts(dropna=False)
    if len(counts.index) < 2:
        return None
    if int(counts.min()) < 2:
        return None
    return y


def _class_distribution(y: Any) -> dict[str, int]:
    series = pd.Series(y)
    counts = series.value_counts(dropna=False).to_dict()
    return {str(k): int(v) for k, v in counts.items()}


def _load_journal_closed(conn: sqlite3.Connection) -> pd.DataFrame:
    q = """
    SELECT id, journal_key, strategy_type, symbol, win_rate_at_entry, current_win_rate_pct,
           iv_rank, realized_pnl, entry_time, exit_time, status, entry_notional,
           unrealized_pnl, attribution_json, post_mortem_json, broker_order_ids_json
    FROM trading_journal
    WHERE status = 'closed'
    ORDER BY exit_time ASC
    """
    return pd.read_sql_query(q, conn)


def _save_xgb_model(clf: Any, path: Path) -> None:
    def _write(inner: Any) -> None:
        gb = getattr(inner, "get_booster", None)
        if callable(gb):
            gb().save_model(str(path))
        else:
            inner.save_model(str(path))

    if hasattr(clf, "calibrated_classifiers_"):
        est = clf.calibrated_classifiers_[0].estimator
        _write(est)
    else:
        _write(clf)


def _write_meta(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def train_if_ready(
    journal_conn: sqlite3.Connection | None = None,
    settings_obj: Any | None = None,
) -> TrainResult | None:
    cfg = settings_obj or settings
    if not cfg.xgboost_enabled:
        return TrainResult(phase=0, n_real_trades=0, message="XGBoost deshabilitado (QUANT_XGBOOST_ENABLED).")

    db_path = Path(cfg.journal_db_path)
    own_conn = False
    if journal_conn is None:
        if not db_path.is_file():
            return TrainResult(phase=0, n_real_trades=0, message="Journal SQLite no encontrado.")
        journal_conn = sqlite3.connect(str(db_path))
        own_conn = True

    try:
        ensure_xgboost_feature_log_table(journal_conn)
        n = _count_real_trades(journal_conn)
        phase = get_training_phase(n)
        if phase == 0:
            logger.info("XGBoost Fase 0: acumulando datos (%s/30 trades reales). Sin entrenamiento.", n)
            return TrainResult(phase=0, n_real_trades=n, message=f"Fase 0: {n}/30 trades reales.")

        fb = FeatureBuilder()
        df_raw = _load_journal_closed(journal_conn)
        df = fb.build_training_dataset(df_raw)
        if df.empty or len(df) < 10:
            return TrainResult(
                phase=phase,
                n_real_trades=n,
                message="Dataset insuficiente tras feature engineering.",
            )

        wf = run_walk_forward(df, phase)

        exclude = {"target_win", "max_favorable_excursion_pct", "target_exit_early", "entry_time", "journal_id"}
        feature_cols = [c for c in df.columns if c not in exclude]

        if phase == 1:
            stable = [c for c in FeatureBuilder.PHASE1_STABLE_FEATURES if c in df.columns]
            if stable:
                feature_cols = stable

        X = df[feature_cols].fillna(0.0).values
        y = df["target_win"].astype(int).values
        class_distribution = _class_distribution(y)
        stratify_labels = _safe_stratify_labels(y)

        model_dir = Path(cfg.xgboost_model_dir)
        model_dir.mkdir(parents=True, exist_ok=True)
        model_path = model_dir / "xgboost_model.json"
        meta_path = model_dir / "xgboost_model_meta.json"

        if stratify_labels is None:
            message = (
                "Dataset sin diversidad suficiente de target_win para entrenar XGBoost. "
                f"class_distribution={class_distribution}"
            )
            meta = {
                "feature_names": feature_cols,
                "phase": phase,
                "n_real_trades": n,
                "class_distribution": class_distribution,
                "training_status": "insufficient_class_diversity",
                "training_status_reason": message,
                "beats_baseline": False,
                "train_auc": 0.0,
                "test_auc": 0.0,
                "baseline_auc": 0.0,
                "walk_forward": wf.to_dict(),
            }
            _write_meta(meta_path, meta)
            return TrainResult(
                phase=phase,
                n_real_trades=n,
                model_path=None,
                meta_path=str(meta_path),
                metrics={"class_distribution": class_distribution},
                message=message,
            )

        X_train, X_test, y_train, y_test = train_test_split(
            X,
            y,
            test_size=0.25,
            random_state=42,
            stratify=stratify_labels,
        )

        base = xgb.XGBClassifier(
            n_estimators=80,
            max_depth=4,
            learning_rate=0.08,
            subsample=0.85,
            colsample_bytree=0.85,
            random_state=42,
            eval_metric="logloss",
        )
        clf: Any = base
        if phase >= 2 and len(y_train) >= 30:
            try:
                cv = min(3, max(2, len(y_train) // 10))
                clf = CalibratedClassifierCV(base, method="sigmoid", cv=cv)
            except Exception:
                clf = base

        clf.fit(X_train, y_train)

        try:
            train_auc = float(roc_auc_score(y_train, clf.predict_proba(X_train)[:, 1]))
            test_auc = float(roc_auc_score(y_test, clf.predict_proba(X_test)[:, 1]))
        except Exception:
            train_auc = test_auc = 0.0

        try:
            bfeat = df["local_win_rate_pct"].fillna(0.0).values.reshape(-1, 1)
            bx_train, bx_test, by_train, by_test = train_test_split(
                bfeat, y, test_size=0.25, random_state=42, stratify=stratify_labels
            )
            lr = LogisticRegression(max_iter=300)
            lr.fit(bx_train, by_train)
            baseline_auc = float(roc_auc_score(by_test, lr.predict_proba(bx_test)[:, 1]))
        except Exception:
            baseline_auc = 0.0

        _save_xgb_model(clf, model_path)

        beats_baseline = (test_auc >= baseline_auc + 0.05) and test_auc > 0.5
        meta = {
            "feature_names": feature_cols,
            "phase": phase,
            "n_real_trades": n,
            "class_distribution": class_distribution,
            "training_status": "trained",
            "train_auc": train_auc,
            "test_auc": test_auc,
            "baseline_auc": baseline_auc,
            "beats_baseline": beats_baseline,
            "walk_forward": wf.to_dict(),
        }
        _write_meta(meta_path, meta)

        return TrainResult(
            phase=phase,
            n_real_trades=n,
            model_path=str(model_path),
            meta_path=str(meta_path),
            metrics={
                "train_auc": train_auc,
                "test_auc": test_auc,
                "baseline_auc": baseline_auc,
                "walk_forward_folds": len(wf.folds),
            },
            message="Entrenamiento completado.",
        )
    finally:
        if own_conn:
            journal_conn.close()
