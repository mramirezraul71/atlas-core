"""Módulo 3 — Clasificador de Régimen de Mercado.

XGBoost (3 clases: Bull / Bear / Sideways) + LSTM embedding de secuencia.

Pipeline:
    1. LSTM(20 barras × 8 features) → embedding 32-dim
    2. XGBoost(embedding + features escalares) → probabilidad [Bull, Bear, Sideways]
    3. Si max_prob < 0.65 → régimen "flat" (no operar)

Features para XGBoost:
    ADX(14), retorno_20d, Hurst, IV Rank, CVD_slope_5m,
    RSI_14, MACD_hist, ATR_norm, volume_ratio

Entrenamiento inicial con yfinance (proxy histórico).
"""

from __future__ import annotations

import logging
import pickle
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional

import numpy as np

logger = logging.getLogger("atlas.models.regime")

try:
    import xgboost as xgb
    _XGB_OK = True
except ImportError:
    _XGB_OK = False
    logger.warning("xgboost no instalado — pip install xgboost")

try:
    import torch
    import torch.nn as nn
    _TORCH_OK = True
except ImportError:
    _TORCH_OK = False
    logger.warning("torch no instalado — LSTM deshabilitado")


# ── Enumeración de régimen ────────────────────────────────────────────────────

class MarketRegime(str, Enum):
    BULL     = "bull"
    BEAR     = "bear"
    SIDEWAYS = "sideways"
    FLAT     = "flat"     # confianza <65%


@dataclass
class RegimeOutput:
    regime: MarketRegime
    confidence: float           # probabilidad de la clase predicha
    proba_bull: float = 0.0
    proba_bear: float = 0.0
    proba_sideways: float = 0.0
    lstm_embedding: Optional[list[float]] = None


# ── LSTM Embedding ────────────────────────────────────────────────────────────

if _TORCH_OK:
    class _LSTMEmbedder(nn.Module):
        """LSTM que convierte secuencia temporal en embedding fijo de 32 dimensiones."""

        INPUT_SIZE  = 8   # features por barra
        HIDDEN_SIZE = 64
        NUM_LAYERS  = 2
        EMBED_SIZE  = 32

        def __init__(self) -> None:
            super().__init__()
            self.lstm = nn.LSTM(
                input_size=self.INPUT_SIZE,
                hidden_size=self.HIDDEN_SIZE,
                num_layers=self.NUM_LAYERS,
                batch_first=True,
                dropout=0.2,
            )
            self.proj = nn.Sequential(
                nn.Linear(self.HIDDEN_SIZE, self.EMBED_SIZE),
                nn.Tanh(),
            )

        def forward(self, x: "torch.Tensor") -> "torch.Tensor":
            # x: (batch, seq_len=20, input_size=8)
            _, (hn, _) = self.lstm(x)
            # hn[-1]: (batch, hidden)
            return self.proj(hn[-1])
else:
    _LSTMEmbedder = None  # type: ignore[assignment,misc]


# ── Clasificador Principal ────────────────────────────────────────────────────

class RegimeClassifier:
    """XGBoost + LSTM para clasificación de régimen de mercado.

    Uso::

        clf = RegimeClassifier()
        clf.load_or_train(symbols=["SPY", "QQQ"])

        features = clf.extract_features(tech_snap, cvd_snap, iv_metrics)
        sequence = clf.build_sequence(history_20_bars)
        output   = clf.predict(features, sequence)
        print(output.regime, output.confidence)
    """

    MIN_CONFIDENCE = 0.65
    SEQ_LEN = 20
    MODEL_PATH = Path("models/saved/regime_classifier.pkl")
    LSTM_PATH  = Path("models/saved/regime_lstm.pt")

    # Clases
    CLASSES = [MarketRegime.BULL, MarketRegime.BEAR, MarketRegime.SIDEWAYS]
    CLASS_LABELS = [0, 1, 2]

    def __init__(self, use_gpu: bool = True) -> None:
        self.use_gpu = use_gpu
        self._xgb_model: Optional[object] = None
        self._lstm: Optional[object] = None
        self._device = "cuda" if (use_gpu and _TORCH_OK and
                                   __import__("torch").cuda.is_available()) else "cpu"
        self._feature_names = [
            "adx_14", "return_20d", "hurst", "iv_rank", "cvd_slope_5m",
            "rsi_14", "macd_hist", "atr_norm", "volume_ratio",
        ]

    # ── Carga / entrenamiento ─────────────────────────────────────────────────

    def load_or_train(self, symbols: list[str] | None = None) -> None:
        """Carga modelos guardados o entrena desde cero con datos históricos."""
        loaded = self._try_load()
        if not loaded:
            logger.info("Modelos no encontrados — entrenando desde datos históricos…")
            self._train_from_history(symbols or ["SPY", "QQQ", "IWM", "AAPL", "NVDA"])

    def _try_load(self) -> bool:
        if not self.MODEL_PATH.exists():
            return False
        try:
            with open(self.MODEL_PATH, "rb") as f:
                self._xgb_model = pickle.load(f)
            logger.info("XGBoost cargado: %s", self.MODEL_PATH)
            self._load_lstm()
            return True
        except Exception as exc:
            logger.warning("Error cargando modelos: %s", exc)
            return False

    def _load_lstm(self) -> None:
        if not _TORCH_OK or not self.LSTM_PATH.exists():
            return
        import torch
        try:
            self._lstm = _LSTMEmbedder()
            self._lstm.load_state_dict(torch.load(self.LSTM_PATH, map_location=self._device))
            self._lstm.to(self._device)
            self._lstm.eval()
            logger.info("LSTM cargado: %s", self.LSTM_PATH)
        except Exception as exc:
            logger.warning("Error cargando LSTM: %s", exc)

    def save(self) -> None:
        self.MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
        if self._xgb_model:
            with open(self.MODEL_PATH, "wb") as f:
                pickle.dump(self._xgb_model, f)
        if self._lstm and _TORCH_OK:
            import torch
            torch.save(self._lstm.state_dict(), self.LSTM_PATH)
        logger.info("Modelos guardados")

    # ── Entrenamiento desde yfinance ──────────────────────────────────────────

    def _train_from_history(self, symbols: list[str]) -> None:
        """Descarga datos históricos y entrena XGBoost + LSTM."""
        X_all, y_all, seq_all = [], [], []

        for sym in symbols:
            try:
                X, y, seqs = self._prepare_training_data(sym)
                X_all.extend(X)
                y_all.extend(y)
                seq_all.extend(seqs)
                logger.info("Datos preparados para %s: %d muestras", sym, len(X))
            except Exception as exc:
                logger.warning("Error preparando %s: %s", sym, exc)

        if not X_all:
            logger.error("Sin datos de entrenamiento")
            return

        X_arr = np.array(X_all, dtype=np.float32)
        y_arr = np.array(y_all, dtype=int)

        # ── Entrenar LSTM ──
        if _TORCH_OK and seq_all:
            self._train_lstm(seq_all, y_arr)
            # Añadir embeddings a X
            embs = self._get_embeddings(seq_all)
            if embs is not None:
                X_arr = np.hstack([X_arr, embs])

        # ── Entrenar XGBoost ──
        if _XGB_OK:
            self._train_xgboost(X_arr, y_arr)

        self.save()

    def _prepare_training_data(
        self, symbol: str
    ) -> tuple[list[list[float]], list[int], list[list[list[float]]]]:
        """Descarga 2 años de datos y construye features + etiquetas."""
        try:
            import yfinance as yf
        except ImportError:
            raise ImportError("pip install yfinance")

        df = yf.download(symbol, period="2y", interval="1d", progress=False)
        if df is None or len(df) < 60:
            return [], [], []

        closes  = df["Close"].values.astype(float)
        highs   = df["High"].values.astype(float)
        lows    = df["Low"].values.astype(float)
        volumes = df["Volume"].values.astype(float)

        X, y, seqs = [], [], []

        for i in range(60, len(closes)):
            win_c = closes[max(0, i-60):i]
            win_h = highs[max(0, i-60):i]
            win_l = lows[max(0, i-60):i]
            win_v = volumes[max(0, i-60):i]

            # Feature escalares
            ret_20d    = (closes[i] - closes[i-20]) / closes[i-20] if i >= 20 else 0.0
            atr_20     = self._compute_atr(win_c, win_h, win_l, 20)
            atr_norm   = atr_20 / closes[i] if closes[i] > 0 else 0.0
            vol_ratio  = volumes[i] / (np.mean(win_v[-20:]) + 1e-10)
            rsi        = self._compute_rsi_simple(win_c, 14)
            adx        = self._compute_adx_simple(win_c, win_h, win_l, 14)
            hurst      = self._compute_hurst_simple(win_c)

            feats = [
                adx, ret_20d, hurst,
                0.5,          # IV Rank placeholder (no disponible en yfinance)
                0.0,          # CVD slope placeholder
                rsi, 0.0,     # MACD hist placeholder
                atr_norm, vol_ratio,
            ]
            X.append(feats)

            # Etiqueta: régimen siguiente N días
            if i + 5 < len(closes):
                fwd_ret = (closes[i + 5] - closes[i]) / closes[i]
                if fwd_ret > 0.01:
                    label = 0  # Bull
                elif fwd_ret < -0.01:
                    label = 1  # Bear
                else:
                    label = 2  # Sideways
            else:
                label = 2
            y.append(label)

            # Secuencia para LSTM
            seq_len = min(self.SEQ_LEN, i)
            seq = []
            for j in range(seq_len):
                idx = i - seq_len + j
                c0 = closes[max(0, idx-1)]
                seq.append([
                    (closes[idx] - c0) / (c0 + 1e-10),  # ret
                    highs[idx] / (closes[idx] + 1e-10),
                    lows[idx]  / (closes[idx] + 1e-10),
                    volumes[idx] / (np.mean(win_v) + 1e-10),
                    0.0, 0.0, 0.0, 0.0,  # placeholders
                ])
            while len(seq) < self.SEQ_LEN:
                seq.insert(0, [0.0] * 8)
            seqs.append(seq[:self.SEQ_LEN])

        return X, y, seqs

    # ── Entrenamiento LSTM ────────────────────────────────────────────────────

    def _train_lstm(self, sequences: list, labels: np.ndarray) -> None:
        if not _TORCH_OK or _LSTMEmbedder is None:
            return
        import torch
        import torch.nn as nn
        from torch.utils.data import DataLoader, TensorDataset

        X_seq = torch.tensor(sequences, dtype=torch.float32)
        y_lbl = torch.tensor(labels, dtype=torch.long)

        dataset = TensorDataset(X_seq, y_lbl)
        loader  = DataLoader(dataset, batch_size=64, shuffle=True)

        model = _LSTMEmbedder().to(self._device)
        # Añadir cabeza de clasificación temporal para entrenamiento supervisado
        clf_head = nn.Linear(_LSTMEmbedder.EMBED_SIZE, 3).to(self._device)
        optimizer = torch.optim.Adam(
            list(model.parameters()) + list(clf_head.parameters()), lr=1e-3
        )
        loss_fn = nn.CrossEntropyLoss()

        model.train()
        for epoch in range(15):
            total_loss = 0.0
            for xb, yb in loader:
                xb, yb = xb.to(self._device), yb.to(self._device)
                emb = model(xb)
                logits = clf_head(emb)
                loss = loss_fn(logits, yb)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                total_loss += loss.item()
            logger.info("LSTM epoch %d/%d — loss: %.4f", epoch+1, 15, total_loss/len(loader))

        model.eval()
        self._lstm = model

    def _get_embeddings(self, sequences: list) -> Optional[np.ndarray]:
        if not _TORCH_OK or self._lstm is None:
            return None
        import torch
        with torch.no_grad():
            X_seq = torch.tensor(sequences, dtype=torch.float32).to(self._device)
            embs = self._lstm(X_seq).cpu().numpy()  # type: ignore[operator]
        return embs

    # ── Entrenamiento XGBoost ─────────────────────────────────────────────────

    def _train_xgboost(self, X: np.ndarray, y: np.ndarray) -> None:
        if not _XGB_OK:
            return

        tree_method = "gpu_hist" if (self.use_gpu and
                                      __import__("torch").cuda.is_available()) else "hist"
        self._xgb_model = xgb.XGBClassifier(  # type: ignore[attr-defined]
            n_estimators=300,
            max_depth=5,
            learning_rate=0.05,
            subsample=0.8,
            colsample_bytree=0.8,
            tree_method=tree_method,
            use_label_encoder=False,
            eval_metric="mlogloss",
            random_state=42,
        )
        self._xgb_model.fit(X, y, verbose=False)  # type: ignore[union-attr]
        logger.info("XGBoost entrenado con %d muestras, %d features", len(X), X.shape[1])

    # ── Inferencia ────────────────────────────────────────────────────────────

    def extract_features(
        self,
        adx: float,
        return_20d: float,
        hurst: float,
        iv_rank: float,
        cvd_slope_5m: float,
        rsi_14: float,
        macd_hist: float,
        atr_norm: float,
        volume_ratio: float,
    ) -> np.ndarray:
        return np.array([[
            adx, return_20d, hurst, iv_rank, cvd_slope_5m,
            rsi_14, macd_hist, atr_norm, volume_ratio,
        ]], dtype=np.float32)

    def build_sequence(self, history_bars: list[dict]) -> list[list[float]]:
        """Convierte historial de barras en secuencia LSTM [seq_len, 8]."""
        seq = []
        for bar in history_bars[-self.SEQ_LEN:]:
            seq.append([
                bar.get("return", 0.0),
                bar.get("high_ratio", 1.0),
                bar.get("low_ratio", 1.0),
                bar.get("volume_ratio", 1.0),
                bar.get("rsi_norm", 0.5),
                bar.get("adx_norm", 0.0),
                bar.get("cvd_norm", 0.0),
                bar.get("iv_norm", 0.0),
            ])
        while len(seq) < self.SEQ_LEN:
            seq.insert(0, [0.0] * 8)
        return seq[:self.SEQ_LEN]

    def predict(
        self,
        features: np.ndarray,
        sequence: list[list[float]] | None = None,
    ) -> RegimeOutput:
        """Predice régimen de mercado."""
        if self._xgb_model is None:
            return RegimeOutput(regime=MarketRegime.FLAT, confidence=0.0)

        # Embedding LSTM
        embedding: Optional[list[float]] = None
        X = features.copy()

        if self._lstm is not None and sequence is not None and _TORCH_OK:
            import torch
            seq_tensor = torch.tensor([sequence], dtype=torch.float32).to(self._device)
            with torch.no_grad():
                emb = self._lstm(seq_tensor).cpu().numpy()  # type: ignore[operator]
            embedding = emb[0].tolist()
            X = np.hstack([X, emb])

        # XGBoost
        try:
            proba = self._xgb_model.predict_proba(X)[0]  # type: ignore[union-attr]
        except Exception as exc:
            logger.error("Error en XGBoost predict: %s", exc)
            return RegimeOutput(regime=MarketRegime.FLAT, confidence=0.0)

        best_idx = int(np.argmax(proba))
        confidence = float(proba[best_idx])

        if confidence < self.MIN_CONFIDENCE:
            regime = MarketRegime.FLAT
        else:
            regime = self.CLASSES[best_idx]

        return RegimeOutput(
            regime          = regime,
            confidence      = confidence,
            proba_bull      = float(proba[0]),
            proba_bear      = float(proba[1]),
            proba_sideways  = float(proba[2]),
            lstm_embedding  = embedding,
        )

    # ── Utilidades estáticas de indicadores ──────────────────────────────────

    @staticmethod
    def _compute_rsi_simple(closes: np.ndarray, period: int = 14) -> float:
        if len(closes) < period + 1:
            return 50.0
        deltas = np.diff(closes[-period-1:])
        gains = np.where(deltas > 0, deltas, 0.0).mean()
        losses = np.where(deltas < 0, -deltas, 0.0).mean()
        if losses < 1e-10:
            return 100.0
        return float(100 - 100 / (1 + gains / losses))

    @staticmethod
    def _compute_atr(closes: np.ndarray, highs: np.ndarray,
                     lows: np.ndarray, period: int = 20) -> float:
        n = min(period, len(closes) - 1)
        if n < 1:
            return 0.0
        tr = np.maximum(highs[-n:] - lows[-n:],
             np.maximum(np.abs(highs[-n:] - closes[-n-1:-1]),
                        np.abs(lows[-n:]  - closes[-n-1:-1])))
        return float(tr.mean())

    @staticmethod
    def _compute_adx_simple(closes: np.ndarray, highs: np.ndarray,
                             lows: np.ndarray, period: int = 14) -> float:
        n = min(period, len(closes) - 1)
        if n < 2:
            return 0.0
        tr = np.maximum(highs[-n:] - lows[-n:],
             np.maximum(np.abs(highs[-n:] - closes[-n-1:-1]),
                        np.abs(lows[-n:]  - closes[-n-1:-1])))
        atr = tr.mean()
        if atr < 1e-10:
            return 0.0
        dmp = np.maximum(highs[-n:] - highs[-n-1:-1], 0)
        dmm = np.maximum(lows[-n-1:-1] - lows[-n:], 0)
        plus_di  = 100 * np.where(dmp > dmm, dmp, 0).mean() / atr
        minus_di = 100 * np.where(dmm >= dmp, dmm, 0).mean() / atr
        denom = plus_di + minus_di
        if denom < 1e-10:
            return 0.0
        return float(100 * abs(plus_di - minus_di) / denom)

    @staticmethod
    def _compute_hurst_simple(series: np.ndarray) -> float:
        n = len(series)
        if n < 20:
            return 0.5
        try:
            lags = [l for l in [4, 8, 16, 32] if l < n // 2]
            if len(lags) < 2:
                return 0.5
            rs_vals = []
            for lag in lags:
                sub = series[-lag:]
                dev = np.cumsum(sub - sub.mean())
                r = dev.max() - dev.min()
                s = sub.std(ddof=1)
                rs_vals.append(r / s if s > 1e-10 else 1.0)
            slope = float(np.polyfit(np.log(lags), np.log(rs_vals), 1)[0])
            return float(np.clip(slope, 0.0, 1.0))
        except Exception:
            return 0.5


# ── Multi-Timeframe Coherence ─────────────────────────────────────────────────

@dataclass
class TFReport:
    """Clasificación de régimen para un timeframe específico."""
    timeframe:  str
    regime:     MarketRegime
    strength:   float   # 0-1: fuerza direccional (FLAT/SIDE penalizado)
    confidence: float   # probabilidad bruta del clasificador
    proba_bull: float = 0.0
    proba_bear: float = 0.0
    proba_sideways: float = 0.0


@dataclass
class MultiTFReport:
    """Resumen de coherencia multi-timeframe para un símbolo."""
    symbol:          str
    dominant_trend:  MarketRegime   # régimen con más votos
    coherence_score: float          # 0-1; <0.7 → skip señal
    aligned_count:   int            # TFs alineados con dominant_trend
    total_tfs:       int
    tf_breakdown:    dict           # {tf: TFReport}
    timestamp:       float


class MultiTimeframeAnalyzer:
    """Analiza coherencia de régimen en múltiples timeframes.

    Lógica::

        Para cada TF:
            1. Toma closes (submuestreados desde buffer 1m)
            2. Calcula features escalares: RSI, ADX, retorno, ATR, Hurst
            3. Llama regime_clf.predict() → RegimeOutput
            4. Calcula strength = confidence × factor (SIDEWAYS 0.3×, FLAT 0×)

        coherence_score = alignment_ratio × (0.4 + 0.6 × avg_strength)
            donde alignment_ratio = n_TFs_con_dominant / n_total

    Ejemplo::

        mtf = MultiTimeframeAnalyzer(regime_clf=clf, timeframes=["1m","5m","15m","1h"])
        report = mtf.analyze("SPY", closes_1m=np.array([...]))
        print(report.coherence_score)   # 0.82 → operar
    """

    # Mínimo de barras de closes_1m para intentar análisis
    MIN_BARS: int = 120

    # Submuestreo de closes_1m por timeframe (cada N barras 1m)
    _TF_STEP: dict = {
        "1m":  1,
        "2m":  2,
        "5m":  5,
        "15m": 15,
        "30m": 30,
        "1h":  60,
        "4h":  240,
        "1d":  390,
    }

    def __init__(
        self,
        regime_clf: "RegimeClassifier",
        timeframes: list[str] | None = None,
        min_bars_per_tf: int = 30,
    ) -> None:
        self._regime_clf     = regime_clf
        self.timeframes      = timeframes or ["1m", "5m", "15m", "1h"]
        self._min_bars_per_tf = min_bars_per_tf

    # ── API pública ───────────────────────────────────────────────────────────

    def analyze(self, symbol: str, closes_1m: np.ndarray) -> MultiTFReport:
        """Analiza coherencia a partir de un buffer de closes en 1m.

        Args:
            symbol:    Ticker (solo informativo).
            closes_1m: Array de closes a resolución 1m (o el intervalo nativo
                       del buffer — se submuestrea según ``_TF_STEP``).

        Returns:
            :class:`MultiTFReport` con ``coherence_score``, ``dominant_trend``
            y desglose por TF.
        """
        tf_reports: dict[str, TFReport] = {}

        for tf in self.timeframes:
            step  = self._TF_STEP.get(tf, 5)
            sub   = closes_1m[::step] if step > 1 else closes_1m
            if len(sub) < self._min_bars_per_tf:
                # No hay suficientes barras para este TF — usar neutral FLAT
                tf_reports[tf] = TFReport(
                    timeframe  = tf,
                    regime     = MarketRegime.FLAT,
                    strength   = 0.0,
                    confidence = 0.0,
                )
                continue
            tf_reports[tf] = self._classify_from_closes(tf, sub)

        dominant = self._dominant_regime(tf_reports)
        coh      = self._coherence_score(tf_reports, dominant)
        aligned  = sum(
            1 for r in tf_reports.values() if r.regime == dominant
        )

        return MultiTFReport(
            symbol          = symbol,
            dominant_trend  = dominant,
            coherence_score = coh,
            aligned_count   = aligned,
            total_tfs       = len(tf_reports),
            tf_breakdown    = tf_reports,
            timestamp       = __import__("time").time(),
        )

    def analyze_from_outputs(
        self,
        symbol: str,
        tf_outputs: "dict[str, RegimeOutput]",
    ) -> MultiTFReport:
        """Construye MultiTFReport desde RegimeOutputs ya calculados externamente.

        Útil cuando el caller ya tiene el régimen de cada TF y sólo
        necesita el score de coherencia agregado.
        """
        tf_reports: dict[str, TFReport] = {}
        for tf, ro in tf_outputs.items():
            tf_reports[tf] = TFReport(
                timeframe      = tf,
                regime         = ro.regime,
                strength       = self._trend_strength(ro),
                confidence     = ro.confidence,
                proba_bull     = ro.proba_bull,
                proba_bear     = ro.proba_bear,
                proba_sideways = ro.proba_sideways,
            )

        dominant = self._dominant_regime(tf_reports)
        coh      = self._coherence_score(tf_reports, dominant)
        aligned  = sum(
            1 for r in tf_reports.values() if r.regime == dominant
        )

        return MultiTFReport(
            symbol          = symbol,
            dominant_trend  = dominant,
            coherence_score = coh,
            aligned_count   = aligned,
            total_tfs       = len(tf_reports),
            tf_breakdown    = tf_reports,
            timestamp       = __import__("time").time(),
        )

    # ── Internos ──────────────────────────────────────────────────────────────

    def _classify_from_closes(self, tf: str, closes: np.ndarray) -> TFReport:
        """Clasifica régimen de un array de closes usando RegimeClassifier."""
        n = len(closes)
        # Placeholder arrays para high/low (usar closes como proxy)
        highs = closes * 1.001
        lows  = closes * 0.999
        vols  = np.ones(n) * 1e6

        ret_20 = float((closes[-1] - closes[-20]) / closes[-20]) if n >= 21 else 0.0
        atr_20 = RegimeClassifier._compute_atr(closes, highs, lows, 20)
        atr_n  = atr_20 / closes[-1] if closes[-1] > 0 else 0.0
        rsi    = RegimeClassifier._compute_rsi_simple(closes, 14)
        adx    = RegimeClassifier._compute_adx_simple(closes, highs, lows, 14)
        hurst  = RegimeClassifier._compute_hurst_simple(closes)
        vol_r  = vols[-1] / (vols[-20:].mean() + 1e-10) if n >= 20 else 1.0

        feats = self._regime_clf.extract_features(
            adx          = adx,
            return_20d   = ret_20,
            hurst        = hurst,
            iv_rank      = 0.5,    # neutral — sin datos IV en este contexto
            cvd_slope_5m = 0.0,
            rsi_14       = rsi,
            macd_hist    = 0.0,
            atr_norm     = atr_n,
            volume_ratio = vol_r,
        )

        ro = self._regime_clf.predict(feats)
        return TFReport(
            timeframe      = tf,
            regime         = ro.regime,
            strength       = self._trend_strength(ro),
            confidence     = ro.confidence,
            proba_bull     = ro.proba_bull,
            proba_bear     = ro.proba_bear,
            proba_sideways = ro.proba_sideways,
        )

    @staticmethod
    def _trend_strength(ro: "RegimeOutput") -> float:
        """Fuerza de la tendencia [0-1].

        * BULL / BEAR → confidence directamente
        * SIDEWAYS    → confidence × 0.3  (tendencia débil para trading)
        * FLAT        → 0.0
        """
        if ro.regime == MarketRegime.FLAT:
            return 0.0
        if ro.regime == MarketRegime.SIDEWAYS:
            return float(ro.confidence) * 0.3
        return float(ro.confidence)

    @staticmethod
    def _dominant_regime(tf_reports: "dict[str, TFReport]") -> MarketRegime:
        """Voto mayoritario entre BULL, BEAR, SIDEWAYS (ignora FLAT)."""
        counts: dict[MarketRegime, float] = {
            MarketRegime.BULL:     0.0,
            MarketRegime.BEAR:     0.0,
            MarketRegime.SIDEWAYS: 0.0,
        }
        for r in tf_reports.values():
            if r.regime in counts:
                # Ponderado por strength para desempate
                counts[r.regime] += 1.0 + r.strength
        best = max(counts, key=lambda k: counts[k])
        # Si todos son FLAT → devolver FLAT
        if all(r.regime == MarketRegime.FLAT for r in tf_reports.values()):
            return MarketRegime.FLAT
        return best

    @staticmethod
    def _coherence_score(
        tf_reports: "dict[str, TFReport]",
        dominant:   MarketRegime,
    ) -> float:
        """
        coherence = alignment_ratio × (0.4 + 0.6 × avg_strength)

        * alignment_ratio = TFs con dominant / total TFs
        * avg_strength    = media de strength de TODOS los TFs
        * El factor (0.4 + 0.6×avg_str) asegura que coherencia ≥ 0.4
          cuando todos los TFs coinciden, incluso si la señal es débil.
        """
        n = len(tf_reports)
        if n == 0:
            return 0.0
        n_aligned   = sum(1 for r in tf_reports.values() if r.regime == dominant)
        alignment   = n_aligned / n
        avg_strength = sum(r.strength for r in tf_reports.values()) / n
        score = alignment * (0.4 + 0.6 * avg_strength)
        return float(np.clip(score, 0.0, 1.0))
