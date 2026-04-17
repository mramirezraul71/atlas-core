"""Carga singleton del modelo XGBoost (.json) + metadatos."""
from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import xgboost as xgb

from config.settings import settings

logger = logging.getLogger("quant.xgboost_signal.loader")


class XGBoostModelLoader:
    _instance: XGBoostModelLoader | None = None
    _lock = threading.Lock()

    def __init__(self) -> None:
        self._model: xgb.XGBClassifier | None = None
        self.feature_names: list[str] = []
        self.phase: int = 0
        self.meta: dict[str, Any] = {}
        self._loaded_at: float | None = None

    @classmethod
    def get_instance(cls) -> XGBoostModelLoader:
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        with cls._lock:
            cls._instance = None

    def model_path(self) -> Path:
        return Path(settings.xgboost_model_dir) / "xgboost_model.json"

    def meta_path(self) -> Path:
        return Path(settings.xgboost_model_dir) / "xgboost_model_meta.json"

    def is_loaded(self) -> bool:
        return self._model is not None

    def load(self, *, force: bool = False) -> bool:
        if self._model is not None and not force:
            return True
        mp = self.model_path()
        mt = self.meta_path()
        self.meta = {}
        self.feature_names = []
        self.phase = 0
        if mt.is_file():
            try:
                self.meta = json.loads(mt.read_text(encoding="utf-8"))
                self.feature_names = list(self.meta.get("feature_names") or [])
                self.phase = int(self.meta.get("phase") or 0)
            except Exception as e:
                logger.error("XGBoost meta load failed: %s", e)
                self.meta = {}
                self.feature_names = []
                self.phase = 0
        if not mp.is_file():
            self._model = None
            return False
        try:
            clf = xgb.XGBClassifier()
            clf.load_model(str(mp))
            self._model = clf
            self._loaded_at = time.time()
            return True
        except Exception as e:
            logger.error("XGBoost load failed: %s", e)
            self._model = None
            return False

    def predict_proba_positive(self, feature_vector: np.ndarray) -> float | None:
        if self._model is None:
            return None
        try:
            proba = self._model.predict_proba(feature_vector.reshape(1, -1))[0, 1]
            return float(proba)
        except Exception as e:
            logger.warning("predict_proba: %s", e)
            return None
