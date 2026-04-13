"""Exit advisor XGBoost — nunca anula exit_governance (override_governance siempre False)."""
from __future__ import annotations

import logging
import sqlite3
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Literal

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

logger = logging.getLogger("quant.xgboost_signal.exit")


@dataclass
class ExitAdvice:
    action: Literal["hold", "close", "reduce", "alert"]
    confidence: float
    reason: str
    model_score: float | None
    holding_hours: float
    unrealized_pnl_pct: float
    override_governance: bool = False


def _vector_from_series(s: pd.Series, names: list[str]) -> np.ndarray:
    return np.array([float(s.get(n, 0.0) or 0.0) for n in names], dtype=np.float64)


class XGBoostExitAdvisor:
    _instance: XGBoostExitAdvisor | None = None

    def __init__(self) -> None:
        self.feature_builder = FeatureBuilder()
        self.loader = XGBoostModelLoader.get_instance()
        self.close_threshold = float(settings.xgboost_exit_alert_threshold)

    @classmethod
    def get_instance(cls) -> XGBoostExitAdvisor:
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        cls._instance = None

    def _real_count_phase(self) -> tuple[int, int]:
        p = Path(settings.journal_db_path)
        if not p.is_file():
            return 0, 0
        conn = sqlite3.connect(str(p))
        try:
            ensure_xgboost_feature_log_table(conn)
            n = _count_real_trades(conn)
            return get_training_phase(n), n
        finally:
            conn.close()

    def _predict(self, features: pd.Series) -> float | None:
        self.loader.load()
        if not self.loader.is_loaded():
            return None
        names = self.loader.feature_names
        if not names:
            return None
        vec = _vector_from_series(features, names)
        return self.loader.predict_proba_positive(vec)

    def advise_exit(self, position: dict, market_snapshot: dict) -> dict[str, Any]:
        features = self.feature_builder.build_intrade_features(position, market_snapshot)
        holding_hours = float(features.get("holding_hours", 0.0))
        unrealized_pct = float(features.get("unrealized_pnl_pct", 0.0))

        phase, _ = self._real_count_phase()
        if phase == 0 or not settings.xgboost_enabled:
            adv = ExitAdvice(
                action="hold",
                confidence=0.0,
                reason="Fase 0: sin modelo disponible",
                model_score=None,
                holding_hours=holding_hours,
                unrealized_pnl_pct=unrealized_pct,
                override_governance=False,
            )
            d = asdict(adv)
            d["override_governance"] = False
            return d

        score = self._predict(features)
        if score is None:
            adv = ExitAdvice(
                action="hold",
                confidence=0.0,
                reason="Sin predicción de salida",
                model_score=None,
                holding_hours=holding_hours,
                unrealized_pnl_pct=unrealized_pct,
                override_governance=False,
            )
            d = asdict(adv)
            d["override_governance"] = False
            return d

        if score > self.close_threshold and holding_hours < 96:
            adv = ExitAdvice(
                action="alert",
                confidence=float(score),
                reason="XGBoost detecta deterioro de thesis pre-time_stop",
                model_score=float(score),
                holding_hours=holding_hours,
                unrealized_pnl_pct=unrealized_pct,
                override_governance=False,
            )
        else:
            adv = ExitAdvice(
                action="hold",
                confidence=float(1.0 - score),
                reason="Sin señal de cierre anticipado",
                model_score=float(score),
                holding_hours=holding_hours,
                unrealized_pnl_pct=unrealized_pct,
                override_governance=False,
            )
        d = asdict(adv)
        d["override_governance"] = False
        return d
