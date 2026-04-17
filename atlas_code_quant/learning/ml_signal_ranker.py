"""Ranker ML de señales para AtlasLearningBrain.

Capa B del sistema híbrido — entrena un modelo de clasificación/regresión
(LightGBM → XGBoost → RandomForest, en orden de preferencia) que predice
la calidad de una señal antes de ejecutarla.

Comportamiento con degradación elegante
-----------------------------------------
- Si LightGBM no está instalado → intenta XGBoost
- Si XGBoost no está instalado → usa RandomForest (sklearn, siempre disponible)
- Si sklearn tampoco está disponible → devuelve score neutral 0.5 (no bloquea)

El modelo se entrena con TradeEvent históricos y predice sobre SignalContext.
El target es binario (winner=1, loser=0) pero el output es probabilidad continua.

Features (19 numéricas)
-----------------------
rsi, macd_hist, atr, bb_pct, volume_ratio, cvd, iv_rank, iv_hv_ratio,
r_initial, capital_normalized, regime_encoded, side_encoded,
setup_encoded, asset_class_encoded, hour_of_day, duration_ratio_est,
position_size_pct, drawdown_state, session_encoded
"""
from __future__ import annotations

import logging
import os
import pickle
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from atlas_code_quant.learning.trade_events import SignalContext, TradeEvent

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constantes de encoding
# ---------------------------------------------------------------------------

_REGIME_MAP = {"BULL": 1, "BEAR": -1, "SIDEWAYS": 0, "VOLATILE": 2}
_SIDE_MAP = {"buy": 1, "sell": -1}
_SETUP_MAP = {
    "breakout": 1, "pullback": 2, "inside_bar": 3,
    "iron_condor": 4, "credit_spread": 5, "debit_spread": 6,
    "covered_call": 7, "naked_put": 8, "momentum": 9, "reversal": 10,
}
_ASSET_MAP = {
    "equity_stock": 1, "equity_etf": 2, "index_option": 3,
    "crypto": 4, "future": 5, "forex": 6, "unknown": 0,
}
_SESSION_MAP = {"pre": 0, "regular": 1, "post": 2}

# Número de features del vector
N_FEATURES = 19

# ---------------------------------------------------------------------------
# Backend detection
# ---------------------------------------------------------------------------

_BACKEND: Optional[str] = None


def _detect_backend() -> str:
    global _BACKEND
    if _BACKEND is not None:
        return _BACKEND
    try:
        import lightgbm  # noqa: F401
        _BACKEND = "lightgbm"
    except ImportError:
        try:
            import xgboost  # noqa: F401
            _BACKEND = "xgboost"
        except ImportError:
            try:
                from sklearn.ensemble import RandomForestClassifier  # noqa: F401
                _BACKEND = "sklearn"
            except ImportError:
                _BACKEND = "none"
    logger.debug("MLSignalRanker backend: %s", _BACKEND)
    return _BACKEND


# ---------------------------------------------------------------------------
# Feature extraction
# ---------------------------------------------------------------------------

def _encode_hour_session(hour: int) -> int:
    """0=pre-market, 1=regular, 2=after-hours."""
    if 9 <= hour < 16:
        return 1
    if 4 <= hour < 9:
        return 0
    return 2


def extract_features_from_trade(trade: TradeEvent) -> List[float]:
    """Extrae vector de features numérico desde un TradeEvent."""
    regime_enc = _REGIME_MAP.get(trade.regime, 0)
    side_enc = _SIDE_MAP.get(trade.side, 0)
    setup_enc = _SETUP_MAP.get(trade.setup_type, 0)
    asset_enc = _ASSET_MAP.get(trade.asset_class, 0)
    hour = trade.entry_time.hour
    session_enc = _encode_hour_session(hour)
    capital_norm = trade.position_size / trade.capital_at_entry if trade.capital_at_entry > 0 else 0.0
    dd_state = 1.0 if trade.mae_r < -1.0 else 0.0  # trade llegó deep en MAE

    return [
        trade.rsi,
        trade.macd_hist,
        trade.atr,
        trade.bb_pct,
        trade.volume_ratio,
        trade.cvd,
        trade.iv_rank,
        trade.iv_hv_ratio,
        trade.r_initial,
        capital_norm,
        float(regime_enc),
        float(side_enc),
        float(setup_enc),
        float(asset_enc),
        float(hour),
        float(trade.duration_minutes),
        float(trade.position_size),
        dd_state,
        float(session_enc),
    ]


def extract_features_from_signal(ctx: SignalContext) -> List[float]:
    """Extrae vector de features desde un SignalContext (señal en vivo)."""
    regime_enc = _REGIME_MAP.get(ctx.regime, 0)
    side_enc = _SIDE_MAP.get(ctx.side, 0)
    setup_enc = _SETUP_MAP.get(ctx.setup_type, 0)
    asset_enc = _ASSET_MAP.get(ctx.asset_class, 0)

    hour = ctx.signal_time.hour if ctx.signal_time else 10
    session_enc = _encode_hour_session(hour)
    capital_norm = ctx.position_size / ctx.capital if ctx.capital > 0 else 0.0

    return [
        ctx.rsi,
        ctx.macd_hist,
        ctx.atr,
        ctx.bb_pct,
        ctx.volume_ratio,
        ctx.cvd,
        ctx.iv_rank,
        ctx.iv_hv_ratio,
        ctx.r_initial,
        capital_norm,
        float(regime_enc),
        float(side_enc),
        float(setup_enc),
        float(asset_enc),
        float(hour),
        0.0,           # duration desconocida en señal en vivo
        float(ctx.position_size),
        0.0,           # dd_state desconocido
        float(session_enc),
    ]


# ---------------------------------------------------------------------------
# Model builders
# ---------------------------------------------------------------------------

def _build_lightgbm(X, y):
    import lightgbm as lgb
    model = lgb.LGBMClassifier(
        n_estimators=200,
        max_depth=6,
        learning_rate=0.05,
        num_leaves=31,
        subsample=0.8,
        colsample_bytree=0.8,
        class_weight="balanced",
        n_jobs=-1,
        verbose=-1,
    )
    model.fit(X, y)
    return model


def _build_xgboost(X, y):
    import xgboost as xgb
    model = xgb.XGBClassifier(
        n_estimators=200,
        max_depth=5,
        learning_rate=0.05,
        subsample=0.8,
        colsample_bytree=0.8,
        use_label_encoder=False,
        eval_metric="logloss",
        verbosity=0,
    )
    model.fit(X, y)
    return model


def _build_sklearn(X, y):
    from sklearn.ensemble import RandomForestClassifier
    model = RandomForestClassifier(
        n_estimators=200,
        max_depth=8,
        class_weight="balanced",
        n_jobs=-1,
        random_state=42,
    )
    model.fit(X, y)
    return model


# ---------------------------------------------------------------------------
# MLSignalRanker — clase principal
# ---------------------------------------------------------------------------

class MLSignalRanker:
    """Ranker ML de señales — capa B del sistema híbrido AtlasLearningBrain.

    Uso básico:
        ranker = MLSignalRanker()
        ranker.train(trade_events)
        score = ranker.score(signal_context)   # 0.0 - 1.0
    """

    MIN_TRADES_TO_TRAIN = 50   # mínimo de trades antes de intentar entrenar

    def __init__(self, model_path: Optional[Path] = None):
        self.model: Any = None
        self.backend: str = _detect_backend()
        self.is_trained: bool = False
        self.n_training_samples: int = 0
        self.model_path = model_path
        self.feature_names: List[str] = [
            "rsi", "macd_hist", "atr", "bb_pct", "volume_ratio",
            "cvd", "iv_rank", "iv_hv_ratio", "r_initial", "capital_norm",
            "regime_enc", "side_enc", "setup_enc", "asset_enc",
            "hour", "duration_min", "position_size", "dd_state", "session_enc",
        ]

        # Intentar cargar modelo previamente entrenado
        if model_path and model_path.exists():
            self._load(model_path)

    # ------------------------------------------------------------------
    # Entrenamiento
    # ------------------------------------------------------------------

    def _training_backend_order(self) -> List[str]:
        if self.backend == "lightgbm":
            return ["lightgbm", "xgboost", "sklearn"]
        if self.backend == "xgboost":
            return ["xgboost", "sklearn"]
        if self.backend == "sklearn":
            return ["sklearn"]
        return []

    def train(self, trades: List[TradeEvent]) -> Dict[str, Any]:
        """Entrena el modelo con la lista de TradeEvent.

        Returns:
            dict con métricas de entrenamiento (n_samples, backend, etc.)
        """
        if len(trades) < self.MIN_TRADES_TO_TRAIN:
            logger.warning(
                "MLSignalRanker.train: solo %d trades (mín %d) — modelo no entrenado",
                len(trades), self.MIN_TRADES_TO_TRAIN
            )
            return {"trained": False, "reason": "insufficient_data", "n": len(trades)}

        if self.backend == "none":
            logger.warning("MLSignalRanker: sin backend disponible — scoring deshabilitado")
            return {"trained": False, "reason": "no_backend"}

        X = [extract_features_from_trade(t) for t in trades]
        y = [1 if t.is_winner else 0 for t in trades]

        builders = {
            "lightgbm": _build_lightgbm,
            "xgboost": _build_xgboost,
            "sklearn": _build_sklearn,
        }
        backend_errors: Dict[str, str] = {}
        win_rate = sum(y) / len(y) if y else 0

        for backend_name in self._training_backend_order():
            builder = builders.get(backend_name)
            if builder is None:
                continue
            try:
                self.model = builder(X, y)
                self.backend = backend_name
                self.is_trained = True
                self.n_training_samples = len(trades)

                if self.model_path:
                    self._save(self.model_path)

                logger.info(
                    "MLSignalRanker entrenado: backend=%s, n=%d, winrate_base=%.2f",
                    self.backend, len(trades), win_rate
                )
                return {
                    "trained": True,
                    "backend": self.backend,
                    "n_samples": len(trades),
                    "base_winrate": round(win_rate, 4),
                }
            except Exception as exc:
                backend_errors[backend_name] = str(exc)
                logger.warning(
                    "MLSignalRanker.train backend %s failed: %s",
                    backend_name,
                    exc,
                )

        self.model = None
        self.is_trained = False
        reason = "; ".join(f"{name}: {error}" for name, error in backend_errors.items()) or "no_backend_succeeded"
        logger.error("MLSignalRanker.train error: %s", reason)
        return {"trained": False, "reason": reason, "backend_errors": backend_errors}

    # ------------------------------------------------------------------
    # Scoring
    # ------------------------------------------------------------------

    def score(self, ctx: SignalContext) -> float:
        """Devuelve probabilidad de ganancia (0.0 – 1.0) para una señal.

        Si el modelo no está entrenado devuelve 0.5 (neutral).
        """
        if not self.is_trained or self.model is None:
            return 0.5

        try:
            features = extract_features_from_signal(ctx)
            prob = self.model.predict_proba([features])[0][1]
            return float(prob)
        except Exception as exc:
            logger.warning("MLSignalRanker.score error: %s", exc)
            return 0.5

    def score_batch(self, contexts: List[SignalContext]) -> List[float]:
        """Score para múltiples señales — más eficiente que llamadas individuales."""
        if not self.is_trained or self.model is None:
            return [0.5] * len(contexts)
        try:
            X = [extract_features_from_signal(ctx) for ctx in contexts]
            probs = self.model.predict_proba(X)[:, 1]
            return [float(p) for p in probs]
        except Exception as exc:
            logger.warning("MLSignalRanker.score_batch error: %s", exc)
            return [0.5] * len(contexts)

    # ------------------------------------------------------------------
    # Persistencia
    # ------------------------------------------------------------------

    def _save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as f:
            pickle.dump({
                "model": self.model,
                "backend": self.backend,
                "n_training_samples": self.n_training_samples,
                "is_trained": self.is_trained,
            }, f)
        logger.info("MLSignalRanker guardado en %s", path)

    def _load(self, path: Path) -> None:
        try:
            with open(path, "rb") as f:
                data = pickle.load(f)
            self.model = data["model"]
            self.backend = data.get("backend", self.backend)
            self.n_training_samples = data.get("n_training_samples", 0)
            self.is_trained = data.get("is_trained", True)
            logger.info(
                "MLSignalRanker cargado desde %s (n=%d, backend=%s)",
                path, self.n_training_samples, self.backend
            )
        except Exception as exc:
            logger.error("MLSignalRanker._load error: %s", exc)

    # ------------------------------------------------------------------
    # Info
    # ------------------------------------------------------------------

    def status(self) -> Dict[str, Any]:
        return {
            "backend": self.backend,
            "is_trained": self.is_trained,
            "n_training_samples": self.n_training_samples,
            "model_path": str(self.model_path) if self.model_path else None,
        }
