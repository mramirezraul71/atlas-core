"""
CNN para clasificación de patrones en velas (Fase 3, opt-in).

Requiere TensorFlow para entrenar/inferir. Sin TF: solo `prepare_candlestick_image`
y heurísticas neutras en predicción.
"""
from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

logger = logging.getLogger("quant.cnn_lstm_pattern")


def _atr_series(df: pd.DataFrame, period: int = 14) -> pd.Series:
    high = df["high"].astype(float)
    low = df["low"].astype(float)
    close = df["close"].astype(float)
    prev_close = close.shift(1)
    tr = pd.concat(
        [
            (high - low).abs(),
            (high - prev_close).abs(),
            (low - prev_close).abs(),
        ],
        axis=1,
    ).max(axis=1)
    return tr.rolling(period, min_periods=period).mean()


def prepare_candlestick_image(df: pd.DataFrame, lookback: int = 20, img_size: int = 28) -> np.ndarray:
    """
    Ultimas `lookback` velas a tensor (img_size, img_size, 6).
    Canales: high, low, close, volume, sma20, atr (normalizados por canal).
    """
    need = {"high", "low", "close", "volume"}
    if not need.issubset(df.columns) or len(df) < lookback + 2:
        raise ValueError("OHLCV insuficiente para imagen de patron")
    sub = df.tail(lookback).copy()
    sma20 = sub["close"].rolling(20, min_periods=1).mean()
    atr = _atr_series(sub, 14).reindex(sub.index).bfill().fillna(0.0)
    mats = [
        sub["high"].astype(float).values,
        sub["low"].astype(float).values,
        sub["close"].astype(float).values,
        sub["volume"].astype(float).values,
        sma20.astype(float).values,
        atr.astype(float).values,
    ]
    target = img_size * img_size
    x_old = np.linspace(0.0, 1.0, lookback)
    x_new = np.linspace(0.0, 1.0, target)
    channels: list[np.ndarray] = []
    for arr in mats:
        z = np.interp(x_new, x_old, arr.astype(np.float64))
        z = (z - np.mean(z)) / (np.std(z) + 1e-8)
        channels.append(z.reshape(img_size, img_size))
    return np.stack(channels, axis=-1).astype(np.float32)


def build_cnn_lstm_model(input_shape: tuple[int, int, int] = (28, 28, 6)) -> Any:
    """Arquitectura CNN compacta (GlobalAveragePooling evita reshape frágil hacia LSTM)."""
    try:
        import tensorflow as tf  # type: ignore
    except ImportError as exc:
        raise RuntimeError("TensorFlow no está instalado. pip install tensorflow") from exc
    inp = tf.keras.Input(shape=input_shape)
    x = tf.keras.layers.Conv2D(32, (3, 3), activation="relu", padding="same")(inp)
    x = tf.keras.layers.MaxPooling2D((2, 2))(x)
    x = tf.keras.layers.Conv2D(64, (3, 3), activation="relu", padding="same")(x)
    x = tf.keras.layers.MaxPooling2D((2, 2))(x)
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    x = tf.keras.layers.Dense(32, activation="relu")(x)
    x = tf.keras.layers.Dropout(0.2)(x)
    out = tf.keras.layers.Dense(3, activation="softmax")(x)
    model = tf.keras.Model(inp, out)
    model.compile(optimizer="adam", loss="categorical_crossentropy", metrics=["accuracy"])
    return model


def predict_pattern_confidence(image: np.ndarray, model: Any | None) -> dict[str, float]:
    """Probabilidades [no_pattern, bullish, bearish]. Sin modelo: uniforme."""
    if model is None:
        return {"no_pattern": 1.0 / 3.0, "bullish_pattern": 1.0 / 3.0, "bearish_pattern": 1.0 / 3.0}
    import tensorflow as tf  # type: ignore

    batch = np.expand_dims(image, axis=0)
    pred = model.predict(batch, verbose=0)[0]
    return {
        "no_pattern": float(pred[0]),
        "bullish_pattern": float(pred[1]),
        "bearish_pattern": float(pred[2]),
    }


def pattern_boost_points(pred: dict[str, float], direction: int, threshold: float = 0.65) -> float:
    """Boost alineado con la dirección del setup (+ puntos sobre selection_score)."""
    bull = float(pred.get("bullish_pattern", 0.0))
    bear = float(pred.get("bearish_pattern", 0.0))
    if direction == 1 and bull > threshold:
        return round((bull - 0.5) * 10.0, 2)
    if direction == -1 and bear > threshold:
        return round((bear - 0.5) * 10.0, 2)
    return 0.0


def load_trained_model(path: Path | str) -> Any | None:
    if not path or not Path(path).is_file():
        return None
    try:
        import tensorflow as tf  # type: ignore

        return tf.keras.models.load_model(str(path))
    except Exception as exc:
        logger.warning("No se pudo cargar modelo CNN-LSTM: %s", exc)
        return None


def train_and_save(
    X: np.ndarray,
    y: np.ndarray,
    X_val: np.ndarray,
    y_val: np.ndarray,
    out_path: Path,
    epochs: int = 50,
    batch_size: int = 32,
) -> Any:
    model = build_cnn_lstm_model()
    try:
        import tensorflow as tf  # type: ignore

        early = tf.keras.callbacks.EarlyStopping(
            monitor="val_loss", patience=5, restore_best_weights=True
        )
        model.fit(
            X,
            y,
            validation_data=(X_val, y_val),
            epochs=epochs,
            batch_size=batch_size,
            callbacks=[early],
            verbose=1,
        )
    except Exception:
        logger.exception("Fallo entrenamiento CNN-LSTM")
        raise
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(out_path))
    return model


def _main_train() -> None:
    p = argparse.ArgumentParser(description="Entrenar CNN-LSTM (dataset externo requerido)")
    p.add_argument("--mode", default="train")
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--out", type=str, default="models/saved/cnn_lstm_trained.keras")
    args = p.parse_args()
    if args.mode != "train":
        return
    raise SystemExit(
        "Dataset de entrenamiento no incluido en repo. "
        "Construir X,y desde OHLCV etiquetado y llamar train_and_save(X,y,Xv,yv, Path(out))."
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    _main_train()
