"""Atlas Code-Quant — Entrenamiento y evaluación de modelos ML.

Entrena clasificadores sobre features técnicas para predecir
señales de trading (BUY=1, HOLD=0, SELL=-1).
"""
from __future__ import annotations

import logging
import pickle
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from sklearn.ensemble import GradientBoostingClassifier, RandomForestClassifier
from sklearn.metrics import classification_report, f1_score
from sklearn.model_selection import TimeSeriesSplit
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline

from models.features import build_features, get_feature_names

logger = logging.getLogger("quant.trainer")

MODELS_DIR = Path(__file__).resolve().parent.parent / "artifacts"


def train_model(
    df: pd.DataFrame,
    model_name: str = "rf",
    target_bars: int = 5,
    n_splits: int = 5,
    save: bool = True,
) -> dict:
    """Entrena un clasificador de señales con validación temporal.

    Args:
        df: OHLCV DataFrame de entrenamiento.
        model_name: "rf" (RandomForest) | "gb" (GradientBoosting).
        target_bars: Ventana de predicción futura.
        n_splits: Folds para TimeSeriesSplit.
        save: Si True, persiste el pipeline en artifacts/.

    Returns:
        Dict con métricas de evaluación y path del modelo guardado.
    """
    logger.info("Preparando features (target_bars=%d)…", target_bars)
    feat_df = build_features(df, target_bars=target_bars)

    feature_cols = get_feature_names(feat_df)
    X = feat_df[feature_cols].values
    y = feat_df["target"].values

    logger.info("Dataset: %d muestras, %d features", len(X), len(feature_cols))

    estimator = _build_estimator(model_name)
    pipeline = Pipeline([
        ("scaler", StandardScaler()),
        ("clf", estimator),
    ])

    tscv = TimeSeriesSplit(n_splits=n_splits)
    f1_scores: list[float] = []

    for fold, (train_idx, val_idx) in enumerate(tscv.split(X), 1):
        X_tr, X_val = X[train_idx], X[val_idx]
        y_tr, y_val = y[train_idx], y[val_idx]
        pipeline.fit(X_tr, y_tr)
        preds = pipeline.predict(X_val)
        f1 = f1_score(y_val, preds, average="weighted", zero_division=0)
        f1_scores.append(f1)
        logger.debug("Fold %d — F1: %.4f", fold, f1)

    # Re-entrenamiento final sobre todos los datos
    pipeline.fit(X, y)
    final_preds = pipeline.predict(X)
    report = classification_report(
        y, final_preds,
        target_names=["SELL(-1)", "HOLD(0)", "BUY(1)"],
        zero_division=0,
        output_dict=True,
    )

    result: dict[str, Any] = {
        "model_name":    model_name,
        "n_features":    len(feature_cols),
        "n_samples":     len(X),
        "cv_f1_mean":    round(float(np.mean(f1_scores)), 4),
        "cv_f1_std":     round(float(np.std(f1_scores)), 4),
        "final_f1":      round(report["weighted avg"]["f1-score"], 4),
        "report":        report,
        "feature_names": feature_cols,
        "model_path":    None,
    }

    if save:
        model_path = _save_pipeline(pipeline, model_name, feature_cols)
        result["model_path"] = str(model_path)
        logger.info("Modelo guardado: %s", model_path)

    logger.info(
        "Entrenamiento OK — CV F1: %.4f ± %.4f | Final F1: %.4f",
        result["cv_f1_mean"], result["cv_f1_std"], result["final_f1"],
    )
    return result


def load_model(model_name: str) -> tuple[Pipeline, list[str]]:
    """Carga un modelo entrenado desde artifacts/.

    Returns:
        (pipeline, feature_names)
    """
    path = MODELS_DIR / f"{model_name}.pkl"
    if not path.exists():
        raise FileNotFoundError(f"Modelo no encontrado: {path}")
    with open(path, "rb") as f:
        bundle = pickle.load(f)
    return bundle["pipeline"], bundle["feature_names"]


def predict_signal(
    pipeline: Pipeline,
    feature_names: list[str],
    df: pd.DataFrame,
    target_bars: int = 5,
) -> dict:
    """Genera una predicción de señal para el último estado del DataFrame.

    Args:
        pipeline: Pipeline entrenado.
        feature_names: Lista de features usadas en entrenamiento.
        df: OHLCV DataFrame (se usa el último punto).
        target_bars: Para construir las mismas features.

    Returns:
        Dict con signal (-1/0/1), probabilities y confianza.
    """
    feat_df = build_features(df, target_bars=target_bars)
    if feat_df.empty:
        return {"signal": 0, "confidence": 0.0, "probabilities": {}}

    last_row = feat_df[feature_names].iloc[[-1]]
    proba = pipeline.predict_proba(last_row)[0]
    classes = pipeline.classes_
    proba_dict = {int(c): round(float(p), 4) for c, p in zip(classes, proba)}

    pred_class = int(classes[np.argmax(proba)])
    confidence = float(max(proba))

    return {
        "signal":        pred_class,
        "confidence":    round(confidence, 4),
        "probabilities": proba_dict,
    }


# ── Helpers ──────────────────────────────────────────────────────────────────

def _build_estimator(model_name: str):
    if model_name == "rf":
        return RandomForestClassifier(
            n_estimators=200,
            max_depth=8,
            min_samples_leaf=10,
            random_state=42,
            n_jobs=-1,
            class_weight="balanced",
        )
    if model_name == "gb":
        return GradientBoostingClassifier(
            n_estimators=200,
            max_depth=4,
            learning_rate=0.05,
            subsample=0.8,
            random_state=42,
        )
    try:
        import lightgbm as lgb
        return lgb.LGBMClassifier(
            n_estimators=300,
            learning_rate=0.05,
            num_leaves=31,
            random_state=42,
            class_weight="balanced",
            n_jobs=-1,
            verbose=-1,
        )
    except ImportError:
        logger.warning("lightgbm no disponible, usando RandomForest")
        return _build_estimator("rf")


def _save_pipeline(pipeline: Pipeline, model_name: str, feature_names: list[str]) -> Path:
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    path = MODELS_DIR / f"{model_name}.pkl"
    with open(path, "wb") as f:
        pickle.dump({"pipeline": pipeline, "feature_names": feature_names}, f)
    return path
