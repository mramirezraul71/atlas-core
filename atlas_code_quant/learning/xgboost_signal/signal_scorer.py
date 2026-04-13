"""Scoring pre-trade XGBoost (nunca bloquea en Fase 0/1)."""
from __future__ import annotations

import logging
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from config.settings import settings
from learning.xgboost_signal.feature_builder import FeatureBuilder
from learning.xgboost_signal.model_loader import XGBoostModelLoader
from learning.xgboost_signal.model_trainer import (
    _count_real_trades,
    ensure_xgboost_feature_log_table,
    get_training_phase,
)

logger = logging.getLogger("quant.xgboost_signal.scorer")


def _vector_from_series(s: pd.Series, names: list[str]) -> np.ndarray:
    return np.array([float(s.get(n, 0.0) or 0.0) for n in names], dtype=np.float64)


class XGBoostSignalScorer:
    _instance: XGBoostSignalScorer | None = None

    def __init__(self) -> None:
        self.feature_builder = FeatureBuilder()
        self.loader = XGBoostModelLoader.get_instance()
        self.warn_threshold = float(settings.xgboost_warn_threshold)
        self.block_threshold = float(settings.xgboost_block_threshold)

    @classmethod
    def get_instance(cls) -> XGBoostSignalScorer:
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        cls._instance = None

    def _phase_and_count(self) -> tuple[int, int]:
        path = Path(settings.journal_db_path)
        if not path.is_file():
            return 0, 0
        conn = sqlite3.connect(str(path))
        try:
            ensure_xgboost_feature_log_table(conn)
            n = _count_real_trades(conn)
            return get_training_phase(n), n
        finally:
            conn.close()

    def _log_phase0(self, candidate: dict, target_win: int | None) -> None:
        path = Path(settings.journal_db_path)
        if not path.is_file():
            return
        conn = sqlite3.connect(str(path))
        try:
            ensure_xgboost_feature_log_table(conn)
            s = self.feature_builder.build_pretrade_features(candidate)
            conn.execute(
                "INSERT INTO xgboost_feature_log (trade_id, timestamp, features_json, target_win, phase) VALUES (?,?,?,?,0)",
                (
                    str(candidate.get("journal_key") or candidate.get("symbol") or "unknown"),
                    datetime.now(timezone.utc).isoformat(),
                    s.to_json(),
                    target_win if target_win is not None else -1,
                ),
            )
            conn.commit()
        except Exception as e:
            logger.debug("phase0 log skip: %s", e)
        finally:
            conn.close()

    def score(self, candidate: dict) -> dict[str, Any]:
        if not settings.xgboost_enabled:
            return {
                "score": None,
                "phase": 0,
                "action": "pass",
                "reason": "XGBoost deshabilitado (QUANT_XGBOOST_ENABLED=false).",
                "n_real_trades": 0,
            }

        phase, n_real = self._phase_and_count()
        if phase == 0:
            logger.info("XGBoost Fase 0: acumulando datos (%s/30 trades reales)", n_real)
            try:
                self._log_phase0(candidate, None)
            except Exception:
                pass
            return {
                "score": None,
                "phase": 0,
                "action": "pass",
                "reason": f"Fase 0: sin modelo ({n_real}/30 trades reales).",
                "n_real_trades": n_real,
            }

        self.loader.load()
        if not self.loader.is_loaded():
            return {
                "score": None,
                "phase": phase,
                "action": "pass",
                "reason": "Modelo no disponible.",
                "n_real_trades": n_real,
            }

        s = self.feature_builder.build_pretrade_features(candidate)
        names = self.loader.feature_names
        if not names:
            return {
                "score": None,
                "phase": phase,
                "action": "pass",
                "reason": "Metadatos sin feature_names.",
                "n_real_trades": n_real,
            }

        vec = _vector_from_series(s, names)
        prob = self.loader.predict_proba_positive(vec)
        if prob is None:
            return {
                "score": None,
                "phase": phase,
                "action": "pass",
                "reason": "Inferencia fallida.",
                "n_real_trades": n_real,
            }

        action = "warn" if prob < self.warn_threshold else "pass"
        return {
            "score": prob,
            "phase": phase,
            "action": action,
            "reason": f"score={prob:.4f} (warn<{self.warn_threshold})",
            "n_real_trades": n_real,
        }

    def evaluate_for_risk_guard(self, candidate: dict, phase: int | None = None) -> dict[str, Any]:
        """Gate opcional compatible con portfolio_risk_guard (block solo Fase 2 + baseline)."""
        result = dict(self.score(candidate))
        eff_phase = phase if phase is not None else result.get("phase", 0)
        score = result.get("score")
        meta = self.loader.meta or {}
        beats = bool(meta.get("beats_baseline", False))

        if eff_phase < 2 or score is None:
            act = "warn" if score is not None and score < self.warn_threshold else "pass"
            return {**result, "action": act}

        if not beats:
            act = "warn" if score < self.warn_threshold else "pass"
            return {
                **result,
                "action": act,
                "reason": result.get("reason", "") + " | bloqueo desactivado: modelo no supera baseline +0.05 AUC.",
            }

        if score < self.block_threshold:
            return {
                **result,
                "action": "block",
                "reason": f"XGBoost score {score:.3f} < threshold {self.block_threshold}",
            }
        return {**result, "action": "pass"}
