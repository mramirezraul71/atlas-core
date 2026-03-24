"""TIN Models — Módulo 3: Technical Indicator Network.

TechnicalIndicatorNetwork (TIN) es una red neuronal interpretable que recibe
features técnicas agrupadas por familia (momentum, volatility, trend, volume) y
produce una probabilidad de retorno positivo a horizonte configurable.

Arquitectura:
    input → [GroupEncoder × 4 familias] → concat → FC layers → sigmoid

Diseño:
    - PyTorch nativo si disponible; sklearn (LogisticRegression/GradientBoosting)
      como fallback transparente.
    - TINTrainer gestiona el ciclo de entrenamiento (early stopping, LR scheduler).
    - TINPredictor expone .predict_proba(feature_window) → float [0, 1].

Flujo:
    1. TINTrainer(config).fit(features_df, targets_series) → TINPredictor
    2. predictor.predict_proba(recent_df) → float
"""
from __future__ import annotations

import logging
import warnings
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pandas as pd

logger = logging.getLogger("quant.pattern_lab.tin")

# ── Dependencias opcionales ────────────────────────────────────────────────────
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import DataLoader, TensorDataset
    _HAS_TORCH = True
except ImportError:
    _HAS_TORCH = False

try:
    from sklearn.ensemble import GradientBoostingClassifier
    from sklearn.linear_model import LogisticRegression
    from sklearn.preprocessing import StandardScaler
    _HAS_SKLEARN = True
except ImportError:
    _HAS_SKLEARN = False


# ── Grupos de indicadores ──────────────────────────────────────────────────────
# Cada grupo se codifica de forma independiente antes del blending
INDICATOR_GROUPS: dict[str, list[str]] = {
    "momentum": [
        "rsi_14", "rsi_7", "rsi_21", "rsi_28",
        "macd", "macd_signal", "macd_hist",
        "mom_5", "mom_10", "mom_20",
        "roc_5", "roc_10", "roc_20",
        "stoch_k", "stoch_d",
        "cci_14", "cci_20", "williams_r",
    ],
    "trend": [
        "adx_14", "plus_di", "minus_di",
        "ema_diff_9_21", "ema_diff_21_50",
        "ema_9", "ema_21", "ema_50",
        "sma_20", "sma_50",
    ],
    "volatility": [
        "bb_upper", "bb_lower", "bb_pct", "bb_width",
        "atr_14", "atr_20", "natr_14",
        "hv_10", "hv_20", "hv_60",
        "hurst",
    ],
    "volume": [
        "obv", "obv_ema",
        "vwap_dev", "cmf_20",
        "volume_ratio", "volume_ratio_5", "volume_zscore",
    ],
}


# ── Configuración ──────────────────────────────────────────────────────────────

@dataclass
class TINConfig:
    """Configuración de TechnicalIndicatorNetwork."""
    hidden_dim: int = 32
    """Dimensión del encoder de cada grupo."""
    blend_dim: int = 64
    """Dimensión de la capa de mezcla (post-concat)."""
    dropout: float = 0.3
    n_epochs: int = 50
    batch_size: int = 64
    lr: float = 1e-3
    weight_decay: float = 1e-4
    patience: int = 8
    """Early stopping: épocas sin mejora."""
    target_horizon: int = 5
    """Barras hacia adelante para el target binario."""
    target_threshold: float = 0.0
    nan_fill: str = "ffill"
    random_state: int = 42
    device: str = "cpu"
    """'cpu' o 'cuda'. 'auto' detecta automáticamente."""
    fallback: str = "auto"
    """'auto' usa PyTorch si disponible, si no sklearn."""


# ── Arquitectura PyTorch ───────────────────────────────────────────────────────

if _HAS_TORCH:
    class _GroupEncoder(nn.Module):
        """Encoder para un grupo de indicadores."""
        def __init__(self, input_dim: int, hidden_dim: int, dropout: float):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(input_dim, hidden_dim),
                nn.BatchNorm1d(hidden_dim),
                nn.ReLU(),
                nn.Dropout(dropout),
                nn.Linear(hidden_dim, hidden_dim),
                nn.ReLU(),
            )

        def forward(self, x: "torch.Tensor") -> "torch.Tensor":
            return self.net(x)

    class TechnicalIndicatorNetwork(nn.Module):
        """
        Red neuronal interpretable para indicadores técnicos.

        Recibe un tensor [batch × total_features] ya ordenado por grupo.
        Internamente separa los features por grupo y aplica encoders
        independientes antes de mezclarlos.
        """

        def __init__(
            self,
            group_sizes: dict[str, int],
            config: TINConfig,
        ):
            super().__init__()
            self.group_sizes = group_sizes
            self.group_names = list(group_sizes.keys())

            # Un encoder por grupo
            self.encoders = nn.ModuleDict({
                name: _GroupEncoder(size, config.hidden_dim, config.dropout)
                for name, size in group_sizes.items()
                if size > 0
            })

            total_encoded = sum(config.hidden_dim for s in group_sizes.values() if s > 0)
            self.blend = nn.Sequential(
                nn.Linear(total_encoded, config.blend_dim),
                nn.BatchNorm1d(config.blend_dim),
                nn.ReLU(),
                nn.Dropout(config.dropout),
                nn.Linear(config.blend_dim, 16),
                nn.ReLU(),
                nn.Linear(16, 1),
                nn.Sigmoid(),
            )

        def forward(self, x: "torch.Tensor") -> "torch.Tensor":
            parts: list["torch.Tensor"] = []
            offset = 0
            for name in self.group_names:
                size = self.group_sizes[name]
                if size == 0:
                    continue
                chunk = x[:, offset:offset + size]
                parts.append(self.encoders[name](chunk))
                offset += size
            blended = torch.cat(parts, dim=1)
            return self.blend(blended).squeeze(1)


# ── Predictor (interface unificada) ───────────────────────────────────────────

class TINPredictor:
    """
    Interface unificada para inferencia TIN.

    Oculta si el modelo subyacente es PyTorch o sklearn.
    """

    def __init__(
        self,
        model: Any,
        feature_cols: list[str],
        scaler: Any | None,
        group_order: list[str],
        backend: str,
        config: TINConfig,
        metadata: dict | None = None,
    ):
        self._model = model
        self._feature_cols = feature_cols
        self._scaler = scaler
        self._group_order = group_order
        self._backend = backend
        self._config = config
        self.metadata: dict = metadata or {}

    @property
    def backend(self) -> str:
        return self._backend

    def predict_proba(self, feature_window: pd.DataFrame) -> float:
        """
        Predice probabilidad de retorno positivo para la ventana actual.

        Args:
            feature_window: DataFrame de features (al menos las columnas
                            conocidas por el modelo; columnas extras se ignoran).

        Returns:
            Probabilidad [0, 1] de retorno positivo al horizonte configurado.
        """
        avail = [c for c in self._feature_cols if c in feature_window.columns]
        if not avail:
            return 0.5

        # Construir vector de entrada con las columnas en el orden correcto
        # Usar la última fila si hay múltiples
        last = feature_window[avail].iloc[-1:]
        # Rellenar columnas faltantes con 0
        missing = [c for c in self._feature_cols if c not in avail]
        for mc in missing:
            last = last.copy()
            last[mc] = 0.0
        last = last[self._feature_cols]
        last = last.fillna(0.0)

        x = last.values.astype(np.float32)
        if self._scaler is not None:
            x = self._scaler.transform(x).astype(np.float32)

        if self._backend == "torch" and _HAS_TORCH:
            return self._predict_torch(x)
        return self._predict_sklearn(x)

    def _predict_torch(self, x: np.ndarray) -> float:
        device = next(self._model.parameters()).device
        tensor = torch.tensor(x, dtype=torch.float32).to(device)
        self._model.eval()
        with torch.no_grad():
            prob = self._model(tensor).item()
        return float(np.clip(prob, 0.0, 1.0))

    def _predict_sklearn(self, x: np.ndarray) -> float:
        if hasattr(self._model, "predict_proba"):
            proba = self._model.predict_proba(x)[0]
            # Clase positiva = index 1
            return float(np.clip(proba[1] if len(proba) > 1 else proba[0], 0.0, 1.0))
        return float(np.clip(self._model.predict(x)[0], 0.0, 1.0))

    def to_dict(self) -> dict[str, Any]:
        return {
            "backend": self._backend,
            "n_features": len(self._feature_cols),
            "groups": self._group_order,
            **self.metadata,
        }


# ── Trainer ───────────────────────────────────────────────────────────────────

class TINTrainer:
    """
    Entrena un TINPredictor a partir de un DataFrame de features y un target binario.

    Uso:
        trainer = TINTrainer(config)
        predictor = trainer.fit(features_df, targets_series)
        prob = predictor.predict_proba(recent_window_df)
    """

    def __init__(self, config: TINConfig | None = None):
        self.config = config or TINConfig()

    def fit(
        self,
        features: pd.DataFrame,
        targets: pd.Series | None = None,
        price_series: pd.Series | None = None,
    ) -> TINPredictor:
        """
        Entrena el modelo.

        Args:
            features:     DataFrame de features (salida de IndicatorSelector).
            targets:      Serie binaria target. Si None, se construye desde price_series.
            price_series: Requerido si targets=None.

        Returns:
            TINPredictor listo para inferencia.
        """
        cfg = self.config

        # Construir target si no se provee
        if targets is None:
            if price_series is None:
                raise ValueError("Se requiere 'targets' o 'price_series'.")
            fwd_ret = price_series.pct_change(cfg.target_horizon).shift(-cfg.target_horizon)
            targets = (fwd_ret > cfg.target_threshold).astype(int)

        # Alinear + limpiar
        common = features.index.intersection(targets.index)
        feat = features.loc[common].copy()
        tgt  = targets.loc[common].copy()

        feat = _fill_nan_df(feat, cfg.nan_fill)
        valid_mask = tgt.notna() & feat.notna().all(axis=1)
        feat = feat.loc[valid_mask]
        tgt  = tgt.loc[valid_mask]

        if len(feat) < 30:
            logger.warning("TINTrainer: datos insuficientes (%d filas). Usando fallback.", len(feat))
            return self._fit_sklearn_fallback(feat, tgt, fallback="lr")

        # Determinar feature columns y ordenarlas por grupo
        feature_cols, group_sizes = _build_col_order(feat.columns.tolist())
        feat = feat[feature_cols]

        # Scaler
        from sklearn.preprocessing import StandardScaler as _SS
        scaler = _SS()
        X = scaler.fit_transform(feat.values).astype(np.float32)
        y = tgt.values.astype(np.float32)

        backend = _pick_backend(cfg.fallback)
        np.random.seed(cfg.random_state)

        if backend == "torch" and _HAS_TORCH:
            model = self._fit_torch(X, y, group_sizes, cfg)
            return TINPredictor(
                model=model,
                feature_cols=feature_cols,
                scaler=scaler,
                group_order=list(group_sizes.keys()),
                backend="torch",
                config=cfg,
                metadata={"n_rows": len(feat), "backend": "torch"},
            )

        # sklearn fallback
        return self._fit_sklearn_fallback(feat, tgt, fallback="gbm", scaler=scaler,
                                          feature_cols=feature_cols, group_sizes=group_sizes)

    def _fit_torch(
        self,
        X: np.ndarray,
        y: np.ndarray,
        group_sizes: dict[str, int],
        cfg: TINConfig,
    ) -> "TechnicalIndicatorNetwork":
        device_str = cfg.device
        if device_str == "auto":
            device_str = "cuda" if torch.cuda.is_available() else "cpu"
        device = torch.device(device_str)

        model = TechnicalIndicatorNetwork(group_sizes, cfg).to(device)
        optimizer = optim.Adam(model.parameters(), lr=cfg.lr, weight_decay=cfg.weight_decay)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=3, factor=0.5)
        criterion = nn.BCELoss()

        # Split train/val
        n = len(X)
        n_val = max(int(n * 0.15), 1)
        idx = np.random.permutation(n)
        train_idx, val_idx = idx[n_val:], idx[:n_val]

        X_train = torch.tensor(X[train_idx], dtype=torch.float32).to(device)
        y_train = torch.tensor(y[train_idx], dtype=torch.float32).to(device)
        X_val   = torch.tensor(X[val_idx],   dtype=torch.float32).to(device)
        y_val   = torch.tensor(y[val_idx],   dtype=torch.float32).to(device)

        dataset = TensorDataset(X_train, y_train)
        loader  = DataLoader(dataset, batch_size=cfg.batch_size, shuffle=True)

        best_val_loss = float("inf")
        best_state    = None
        no_improve    = 0

        for epoch in range(cfg.n_epochs):
            model.train()
            for xb, yb in loader:
                optimizer.zero_grad()
                pred = model(xb)
                loss = criterion(pred, yb)
                loss.backward()
                optimizer.step()

            model.eval()
            with torch.no_grad():
                val_pred = model(X_val)
                val_loss = criterion(val_pred, y_val).item()

            scheduler.step(val_loss)

            if val_loss < best_val_loss - 1e-5:
                best_val_loss = val_loss
                best_state    = {k: v.cpu().clone() for k, v in model.state_dict().items()}
                no_improve    = 0
            else:
                no_improve += 1
                if no_improve >= cfg.patience:
                    logger.debug("TINTrainer: early stopping en época %d (val_loss=%.4f)", epoch + 1, best_val_loss)
                    break

        if best_state:
            model.load_state_dict({k: v.to(device) for k, v in best_state.items()})
        model.eval()
        logger.info(
            "TINTrainer (torch): entrenado — %d filas, %d features, val_loss=%.4f",
            len(X), X.shape[1], best_val_loss,
        )
        return model

    def _fit_sklearn_fallback(
        self,
        feat: pd.DataFrame,
        tgt: pd.Series,
        fallback: str = "gbm",
        scaler: Any | None = None,
        feature_cols: list[str] | None = None,
        group_sizes: dict[str, int] | None = None,
    ) -> TINPredictor:
        if not _HAS_SKLEARN:
            # Último recurso: media empírica
            mean_prob = float(tgt.mean())
            class _ConstantModel:
                def __init__(self, p): self.p = p
                def predict_proba(self, X): return np.array([[1 - self.p, self.p]] * len(X))
            fc = feature_cols or feat.columns.tolist()
            return TINPredictor(
                model=_ConstantModel(mean_prob),
                feature_cols=fc,
                scaler=None,
                group_order=[],
                backend="constant",
                config=self.config,
                metadata={"n_rows": len(feat), "backend": "constant", "const_prob": mean_prob},
            )

        fc = feature_cols or feat.columns.tolist()
        gs = group_sizes or {}

        from sklearn.preprocessing import StandardScaler as _SS
        if scaler is None:
            scaler = _SS()
            X = scaler.fit_transform(feat[fc].fillna(0).values)
        else:
            X = scaler.transform(feat[fc].fillna(0).values)

        y = tgt.values.astype(int)

        if fallback == "gbm" and _HAS_SKLEARN:
            from sklearn.ensemble import GradientBoostingClassifier as _GBC
            model = _GBC(
                n_estimators=100, max_depth=4,
                random_state=self.config.random_state,
                n_iter_no_change=self.config.patience,
                validation_fraction=0.15,
            )
        else:
            from sklearn.linear_model import LogisticRegression as _LR
            model = _LR(max_iter=500, random_state=self.config.random_state, C=1.0)

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            model.fit(X, y)

        logger.info(
            "TINTrainer (sklearn/%s): entrenado — %d filas, %d features",
            fallback, len(feat), len(fc),
        )
        return TINPredictor(
            model=model,
            feature_cols=fc,
            scaler=scaler,
            group_order=list(gs.keys()),
            backend=f"sklearn/{fallback}",
            config=self.config,
            metadata={"n_rows": len(feat), "backend": f"sklearn/{fallback}"},
        )


# ── Funciones auxiliares ───────────────────────────────────────────────────────

def _pick_backend(fallback: str) -> str:
    if fallback == "auto":
        return "torch" if _HAS_TORCH else "sklearn"
    return fallback


def _build_col_order(available_cols: list[str]) -> tuple[list[str], dict[str, int]]:
    """
    Ordena las columnas por grupo y devuelve (ordered_cols, group_sizes).
    Columnas no reconocidas se agregan al final como grupo 'other'.
    """
    ordered: list[str] = []
    sizes: dict[str, int] = {}
    assigned: set[str] = set()

    for group, members in INDICATOR_GROUPS.items():
        present = [c for c in members if c in available_cols]
        ordered.extend(present)
        sizes[group] = len(present)
        assigned.update(present)

    other = [c for c in available_cols if c not in assigned]
    if other:
        ordered.extend(other)
        sizes["other"] = len(other)

    return ordered, {k: v for k, v in sizes.items() if v > 0}


def _fill_nan_df(df: pd.DataFrame, strategy: str) -> pd.DataFrame:
    if strategy == "ffill":
        return df.ffill().fillna(0)
    if strategy == "mean":
        return df.fillna(df.mean())
    return df.fillna(0)
