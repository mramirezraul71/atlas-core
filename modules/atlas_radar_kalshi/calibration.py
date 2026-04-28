"""
calibration.py — Calibración probabilística (Platt + isotónica).

El brain devuelve una probabilidad cruda ``p_raw``. Antes de
compararla con el mercado se calibra contra el track record histórico
del módulo (archivo ``logs/radar_calibration.jsonl``). Implementamos:

- **Platt scaling**: regresión logística 1-D entrenada con
  parejas ``(p_raw, outcome)`` (outcome ∈ {0,1}).
- **Isotonic regression**: vía ``sklearn.isotonic.IsotonicRegression``
  cuando ``sklearn`` está disponible; en su defecto se cae a Platt.

Si el dataset es < ``min_samples`` (default 50) la función devuelve la
probabilidad sin cambios (identity), evitando overfitting al inicio.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np


# ---------------------------------------------------------------------------
class Calibrator:
    """Calibrador entrenable y serializable."""

    def __init__(self, method: str = "platt", min_samples: int = 50) -> None:
        if method not in {"platt", "isotonic", "identity"}:
            raise ValueError("method must be platt|isotonic|identity")
        self.method = method
        self.min_samples = min_samples
        # Platt params
        self.a: float = 1.0
        self.b: float = 0.0
        # Isotonic
        self._iso = None  # sklearn IsotonicRegression
        self._fitted = False
        self.n_samples = 0

    # ------------------------------------------------------------------
    def fit(self, p_raw: Sequence[float], y: Sequence[int]) -> "Calibrator":
        x = np.asarray(p_raw, dtype=float)
        t = np.asarray(y, dtype=float)
        self.n_samples = int(len(x))
        if self.n_samples < self.min_samples:
            self._fitted = False
            return self
        x = np.clip(x, 1e-4, 1 - 1e-4)
        if self.method == "isotonic":
            try:
                from sklearn.isotonic import IsotonicRegression
                self._iso = IsotonicRegression(out_of_bounds="clip").fit(x, t)
                self._fitted = True
                return self
            except Exception:
                self.method = "platt"  # fallback
        if self.method == "platt":
            # Logistic regression (gradient descent, 1D, BFGS-style)
            logit = np.log(x / (1 - x))
            # newton solver
            a, b = 1.0, 0.0
            for _ in range(200):
                z = a * logit + b
                z = np.clip(z, -40, 40)
                p = 1.0 / (1.0 + np.exp(-z))
                w = p * (1 - p) + 1e-9
                grad_a = ((p - t) * logit).sum()
                grad_b = (p - t).sum()
                h_aa = (w * logit * logit).sum()
                h_ab = (w * logit).sum()
                h_bb = w.sum()
                det = h_aa * h_bb - h_ab * h_ab
                if abs(det) < 1e-12:
                    break
                inv = np.array([[h_bb, -h_ab], [-h_ab, h_aa]]) / det
                step = inv @ np.array([grad_a, grad_b])
                a -= step[0]
                b -= step[1]
                if np.linalg.norm(step) < 1e-7:
                    break
            self.a, self.b = float(a), float(b)
            self._fitted = True
        return self

    # ------------------------------------------------------------------
    def predict(self, p: float) -> float:
        if not self._fitted:
            return float(p)
        p = float(np.clip(p, 1e-4, 1 - 1e-4))
        if self.method == "isotonic" and self._iso is not None:
            return float(self._iso.predict([p])[0])
        # platt
        logit = math.log(p / (1 - p))
        z = max(-40.0, min(40.0, self.a * logit + self.b))
        return 1.0 / (1.0 + math.exp(-z))

    # ------------------------------------------------------------------
    def to_dict(self) -> dict:
        return {
            "method": self.method,
            "fitted": self._fitted,
            "a": self.a, "b": self.b,
            "n_samples": self.n_samples,
        }


# ---------------------------------------------------------------------------
def load_history(path: Path) -> Tuple[List[float], List[int]]:
    """Lee ``logs/radar_calibration.jsonl``: {p_raw, outcome}."""
    if not path.exists():
        return [], []
    p_raw, ys = [], []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            try:
                row = json.loads(line)
                p_raw.append(float(row["p_raw"]))
                ys.append(int(row["outcome"]))
            except Exception:
                continue
    return p_raw, ys


def append_outcome(path: Path, p_raw: float, outcome: int) -> None:
    """Persiste un par (p_raw, outcome) para entrenamientos posteriores."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps({"p_raw": p_raw, "outcome": int(bool(outcome))}) + "\n")


def fit_from_disk(path: Path, method: str = "platt",
                  min_samples: int = 50) -> Calibrator:
    cal = Calibrator(method=method, min_samples=min_samples)
    p_raw, ys = load_history(path)
    if p_raw:
        cal.fit(p_raw, ys)
    return cal


def refit_from_disk_temporal(
    path: Path,
    method: str = "platt",
    min_samples: int = 50,
    min_holdout: int = 30,
    train_ratio: float = 0.8,
) -> tuple[Calibrator, dict]:
    """
    Reentrena con split temporal mínimo para evitar leakage.
    Devuelve calibrador + métricas de holdout.
    """
    p_raw, ys = load_history(path)
    cal = Calibrator(method=method, min_samples=min_samples)
    n = len(p_raw)
    if n < max(min_samples, min_holdout + 5):
        return cal, {"updated": False, "n_samples": n, "reason": "insufficient_samples"}

    ratio = max(0.5, min(0.95, float(train_ratio)))
    split = int(n * ratio)
    split = max(min_samples, min(split, n - min_holdout))
    x_train = p_raw[:split]
    y_train = ys[:split]
    x_test = p_raw[split:]
    y_test = ys[split:]
    cal.fit(x_train, y_train)
    if not x_test:
        return cal, {"updated": cal._fitted, "n_samples": n, "train_size": len(x_train), "test_size": 0}
    preds = [cal.predict(x) for x in x_test]
    brier = float(np.mean([(p - y) ** 2 for p, y in zip(preds, y_test)]))
    return cal, {
        "updated": cal._fitted,
        "n_samples": n,
        "train_size": len(x_train),
        "test_size": len(x_test),
        "brier_holdout": brier,
        "method": cal.method,
    }
