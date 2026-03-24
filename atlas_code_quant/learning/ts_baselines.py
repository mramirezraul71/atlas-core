"""TS Baselines — Módulo 4: Modelos de series de tiempo ARIMA / ETS.

Construye líneas base de predicción con statsmodels y extrae features de
desviación residual que enriquecen el contexto del Pattern Lab.

Flujo:
    1. TimeSeriesBaselines(config).fit(price_series) → modelo ajustado
    2. .forecast(horizon_bars) → TSForecast (next_return, confidence_interval)
    3. .residual_features(price_series, window) → pd.DataFrame
       [deviation_from_forecast, residual_zscore, arima_error_ma, trend_slope, ...]

Las features de residual son un input valioso para el TIN y el score de edge.
"""
from __future__ import annotations

import logging
import warnings
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pandas as pd

logger = logging.getLogger("quant.pattern_lab.ts_baselines")

# ── Dependencias opcionales ────────────────────────────────────────────────────
try:
    from statsmodels.tsa.arima.model import ARIMA
    from statsmodels.tsa.statespace.sarimax import SARIMAX
    _HAS_ARIMA = True
except ImportError:
    _HAS_ARIMA = False

try:
    from statsmodels.tsa.holtwinters import ExponentialSmoothing
    _HAS_ETS = True
except ImportError:
    _HAS_ETS = False

try:
    from statsmodels.tsa.stattools import adfuller
    _HAS_STAT_TESTS = True
except ImportError:
    _HAS_STAT_TESTS = False


# ── Configuración ──────────────────────────────────────────────────────────────

@dataclass
class TSBaselineConfig:
    """Configuración para los modelos de series de tiempo."""
    arima_order: tuple[int, int, int] = (1, 1, 1)
    """(p, d, q) para ARIMA."""
    use_sarimax: bool = False
    """Si True, intenta SARIMAX con componente estacional."""
    seasonal_period: int = 5
    """Período estacional (5=semanal para datos diarios)."""
    ets_trend: str | None = "add"
    """Componente de tendencia ETS: 'add', 'mul', None."""
    ets_seasonal: str | None = None
    """Componente estacional ETS: 'add', 'mul', None."""
    ets_seasonal_periods: int | None = None
    """Períodos para el componente estacional ETS."""
    fit_on_returns: bool = True
    """Si True, ajusta el modelo sobre log-retornos en vez de precios."""
    rolling_window: int = 60
    """Ventana para ajuste rolling de ARIMA (re-fit cada N barras)."""
    max_history_bars: int = 500
    """Máximo de barras históricas para ajustar el modelo."""
    residual_window: int = 20
    """Ventana para calcular Z-score del residual."""
    forecast_horizon: int = 5
    """Barras hacia adelante para el forecast por defecto."""


# ── Data classes de salida ─────────────────────────────────────────────────────

@dataclass
class TSForecast:
    """Resultado de una predicción de series de tiempo."""
    model: str = "arima"
    horizon_bars: int = 5
    forecast_return: float = 0.0
    """Retorno esperado (log o simple) a horizon_bars pasos."""
    lower_ci: float = 0.0
    """Límite inferior del intervalo de confianza 95%."""
    upper_ci: float = 0.0
    """Límite superior del intervalo de confianza 95%."""
    current_residual: float = 0.0
    """Error del modelo en el último punto observado."""
    residual_zscore: float = 0.0
    """Z-score del residual actual dentro de la ventana residual_window."""
    deviation_from_forecast: float = 0.0
    """(precio_actual - forecast_precio) / precio_actual — útil para mean-reversion."""
    trend_slope: float = 0.0
    """Slope de regresión lineal sobre las últimas barras (normalizado)."""
    is_stationary: bool | None = None
    """Resultado del test ADF (None si statsmodels no disponible)."""
    metadata: dict = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "horizon_bars": self.horizon_bars,
            "forecast_return": round(self.forecast_return, 6),
            "lower_ci": round(self.lower_ci, 6),
            "upper_ci": round(self.upper_ci, 6),
            "current_residual": round(self.current_residual, 6),
            "residual_zscore": round(self.residual_zscore, 4),
            "deviation_from_forecast": round(self.deviation_from_forecast, 6),
            "trend_slope": round(self.trend_slope, 6),
            "is_stationary": self.is_stationary,
            **self.metadata,
        }


# ── Servicio principal ─────────────────────────────────────────────────────────

class TimeSeriesBaselines:
    """
    Ajusta modelos ARIMA y ETS sobre series de precios y construye
    features de desviación/residual para enriquecer el contexto del Pattern Lab.

    Diseñado para ajustarse offline (fit) y generar features en tiempo real
    (residual_features, forecast_now).
    """

    def __init__(self, config: TSBaselineConfig | None = None):
        self.config = config or TSBaselineConfig()
        self._arima_model: Any = None
        self._ets_model: Any = None
        self._fitted_series: pd.Series | None = None
        self._residuals: pd.Series | None = None
        self._is_fitted: bool = False
        self._arima_ok: bool = False
        self._ets_ok: bool = False

    # ── Fit ───────────────────────────────────────────────────────────────────

    def fit(self, price_series: pd.Series) -> "TimeSeriesBaselines":
        """
        Ajusta ARIMA y ETS sobre la serie de precios.

        Args:
            price_series: Serie de precios de cierre con índice temporal.

        Returns:
            self (para encadenamiento).
        """
        cfg = self.config
        series = price_series.dropna().astype(np.float64)
        if len(series) > cfg.max_history_bars:
            series = series.iloc[-cfg.max_history_bars:]

        # Trabajar sobre log-retornos si está configurado
        if cfg.fit_on_returns:
            fit_series = np.log(series / series.shift(1)).dropna()
        else:
            fit_series = series

        self._fitted_series = fit_series

        # ── ARIMA ─────────────────────────────────────────────────────────────
        if _HAS_ARIMA:
            self._arima_ok = self._fit_arima(fit_series, cfg)
        else:
            logger.debug("TimeSeriesBaselines: statsmodels no disponible, ARIMA deshabilitado.")

        # ── ETS ───────────────────────────────────────────────────────────────
        if _HAS_ETS:
            self._ets_ok = self._fit_ets(fit_series, cfg)
        else:
            logger.debug("TimeSeriesBaselines: statsmodels no disponible, ETS deshabilitado.")

        self._is_fitted = self._arima_ok or self._ets_ok

        # Calcular residuales del ARIMA si disponible
        if self._arima_ok and self._arima_model is not None:
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    res_vals = self._arima_model.resid
                self._residuals = pd.Series(res_vals, index=fit_series.index[-len(res_vals):])
            except Exception:
                self._residuals = None

        logger.info(
            "TimeSeriesBaselines.fit: %d barras — ARIMA=%s ETS=%s",
            len(fit_series), self._arima_ok, self._ets_ok,
        )
        return self

    def _fit_arima(self, series: pd.Series, cfg: TSBaselineConfig) -> bool:
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                if cfg.use_sarimax and _HAS_ARIMA:
                    model = SARIMAX(
                        series,
                        order=cfg.arima_order,
                        seasonal_order=(1, 0, 1, cfg.seasonal_period),
                        enforce_stationarity=False,
                        enforce_invertibility=False,
                    )
                else:
                    model = ARIMA(series, order=cfg.arima_order)
                result = model.fit(disp=False)
            self._arima_model = result
            return True
        except Exception as e:
            logger.warning("TimeSeriesBaselines: ARIMA fit falló: %s", e)
            return False

    def _fit_ets(self, series: pd.Series, cfg: TSBaselineConfig) -> bool:
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                model = ExponentialSmoothing(
                    series,
                    trend=cfg.ets_trend,
                    seasonal=cfg.ets_seasonal,
                    seasonal_periods=cfg.ets_seasonal_periods,
                    initialization_method="estimated",
                )
                result = model.fit(optimized=True, disp=False)
            self._ets_model = result
            return True
        except Exception as e:
            logger.warning("TimeSeriesBaselines: ETS fit falló: %s", e)
            return False

    # ── Forecast en tiempo real ────────────────────────────────────────────────

    def forecast_now(
        self,
        price_series: pd.Series | None = None,
        horizon_bars: int | None = None,
        model: str = "arima",
    ) -> TSForecast:
        """
        Genera un forecast para el precio/retorno actual.

        Args:
            price_series: Serie actual (para calcular desviación). Si None usa
                          la serie de fit.
            horizon_bars: Barras hacia adelante. Usa config por defecto si None.
            model:        'arima', 'ets', o 'ensemble' (promedio de ambos).

        Returns:
            TSForecast con forecast_return, intervalos y features derivadas.
        """
        cfg = self.config
        h = horizon_bars or cfg.forecast_horizon

        arima_fc = TSForecast(model="arima", horizon_bars=h)
        ets_fc   = TSForecast(model="ets",   horizon_bars=h)

        if self._arima_ok and self._arima_model is not None:
            arima_fc = self._arima_forecast(h)

        if self._ets_ok and self._ets_model is not None:
            ets_fc = self._ets_forecast(h)

        # Calcular desviación actual respecto al forecast
        if price_series is not None and len(price_series) > 0:
            current_price = float(price_series.iloc[-1])
            self._enrich_forecast(arima_fc, price_series, current_price, cfg)
            self._enrich_forecast(ets_fc,   price_series, current_price, cfg)

        if model == "arima":
            return arima_fc
        if model == "ets":
            return ets_fc

        # Ensemble: promedio ponderado (60% ARIMA, 40% ETS si ambos disponibles)
        if self._arima_ok and self._ets_ok:
            w_arima, w_ets = 0.6, 0.4
            ens = TSForecast(
                model="ensemble",
                horizon_bars=h,
                forecast_return=w_arima * arima_fc.forecast_return + w_ets * ets_fc.forecast_return,
                lower_ci=min(arima_fc.lower_ci, ets_fc.lower_ci),
                upper_ci=max(arima_fc.upper_ci, ets_fc.upper_ci),
                current_residual=w_arima * arima_fc.current_residual + w_ets * ets_fc.current_residual,
                residual_zscore=w_arima * arima_fc.residual_zscore + w_ets * ets_fc.residual_zscore,
                deviation_from_forecast=arima_fc.deviation_from_forecast,
                trend_slope=arima_fc.trend_slope,
                is_stationary=arima_fc.is_stationary,
            )
            return ens
        if self._arima_ok:
            return arima_fc
        return ets_fc

    def _arima_forecast(self, h: int) -> TSForecast:
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                fc = self._arima_model.get_forecast(steps=h)
                mean_fc = float(fc.predicted_mean.iloc[-1]) if hasattr(fc.predicted_mean, "iloc") else float(fc.predicted_mean[-1])
                ci      = fc.conf_int(alpha=0.05)
                lower   = float(ci.iloc[-1, 0]) if hasattr(ci, "iloc") else float(ci[-1, 0])
                upper   = float(ci.iloc[-1, 1]) if hasattr(ci, "iloc") else float(ci[-1, 1])
                resid   = float(self._arima_model.resid.iloc[-1]) if hasattr(self._arima_model.resid, "iloc") else float(self._arima_model.resid[-1])
            return TSForecast(
                model="arima", horizon_bars=h,
                forecast_return=mean_fc,
                lower_ci=lower, upper_ci=upper,
                current_residual=resid,
            )
        except Exception as e:
            logger.debug("ARIMA forecast error: %s", e)
            return TSForecast(model="arima", horizon_bars=h)

    def _ets_forecast(self, h: int) -> TSForecast:
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                fc_vals = self._ets_model.forecast(steps=h)
                mean_fc = float(fc_vals.iloc[-1]) if hasattr(fc_vals, "iloc") else float(fc_vals[-1])
                # ETS no provee intervalos por defecto; estimar del sigma de residuales
                resid_std = float(self._ets_model.resid.std()) if hasattr(self._ets_model, "resid") else 0.01
                resid = float(self._ets_model.resid.iloc[-1]) if hasattr(self._ets_model.resid, "iloc") else 0.0
            return TSForecast(
                model="ets", horizon_bars=h,
                forecast_return=mean_fc,
                lower_ci=mean_fc - 1.96 * resid_std,
                upper_ci=mean_fc + 1.96 * resid_std,
                current_residual=resid,
            )
        except Exception as e:
            logger.debug("ETS forecast error: %s", e)
            return TSForecast(model="ets", horizon_bars=h)

    def _enrich_forecast(
        self, fc: TSForecast, price_series: pd.Series,
        current_price: float, cfg: TSBaselineConfig
    ) -> None:
        """Añade deviation_from_forecast, residual_zscore, trend_slope a un TSForecast."""
        # Residual Z-score
        if self._residuals is not None and len(self._residuals) >= 5:
            resid_window = self._residuals.iloc[-cfg.residual_window:]
            mu  = float(resid_window.mean())
            std = float(resid_window.std()) + 1e-8
            fc.residual_zscore = (fc.current_residual - mu) / std

        # Desviación del precio actual respecto al forecast en escala de precio
        # Si fit_on_returns=True, forecast_return es un log-retorno → convertir a precio
        if cfg.fit_on_returns:
            implied_price = current_price * np.exp(fc.forecast_return)
        else:
            implied_price = current_price + fc.forecast_return
        fc.deviation_from_forecast = (current_price - implied_price) / (current_price + 1e-8)

        # Trend slope: regresión lineal sobre las últimas window_size barras de precio
        window = min(20, len(price_series))
        recent = price_series.iloc[-window:].values.astype(np.float64)
        if len(recent) >= 2:
            x = np.arange(len(recent), dtype=np.float64)
            slope = float(np.polyfit(x, recent / recent[0], 1)[0])  # normalizado
            fc.trend_slope = slope

        # Test ADF
        if _HAS_STAT_TESTS and len(price_series) >= 20:
            try:
                adf_result = adfuller(price_series.dropna(), autolag="AIC")
                fc.is_stationary = bool(adf_result[1] < 0.05)
            except Exception:
                fc.is_stationary = None

    # ── Features de residual ───────────────────────────────────────────────────

    def residual_features(
        self,
        price_series: pd.Series,
        window: int | None = None,
    ) -> pd.DataFrame:
        """
        Construye un DataFrame de features derivadas de los residuales del modelo.

        Args:
            price_series: Serie actual de precios.
            window:       Ventana para cálculos rolling. Usa config si None.

        Returns:
            DataFrame con columnas: deviation_from_forecast, residual_zscore,
            arima_residual_ma, trend_slope, is_above_forecast, ets_deviation.
        """
        cfg = self.config
        w = window or cfg.residual_window
        features: dict[str, pd.Series] = {}

        price = price_series.dropna().astype(np.float64)
        if cfg.fit_on_returns:
            series = np.log(price / price.shift(1)).dropna()
        else:
            series = price

        # ── ARIMA residual features ────────────────────────────────────────
        if self._arima_ok and self._arima_model is not None:
            try:
                # In-sample fitted values
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    fitted    = self._arima_model.fittedvalues
                    residuals = self._arima_model.resid

                # Alinear longitudes
                min_len = min(len(series), len(residuals))
                series_aligned   = series.iloc[-min_len:].values
                residuals_aligned = residuals.values[-min_len:] if hasattr(residuals, "values") else np.array(residuals)[-min_len:]

                resid_series = pd.Series(residuals_aligned, index=series.index[-min_len:])
                mu    = resid_series.rolling(w).mean()
                sigma = resid_series.rolling(w).std() + 1e-8

                features["arima_residual_zscore"] = (resid_series - mu) / sigma
                features["arima_residual_ma"]     = resid_series.rolling(w).mean()
                features["arima_residual_abs"]    = resid_series.abs().rolling(w).mean()

                # Desviación precio actual vs fitted en price-space
                if cfg.fit_on_returns:
                    # fitted es log-retorno; precio ajustado es cumsum
                    cumret_fitted = np.exp(pd.Series(fitted.values if hasattr(fitted, "values") else fitted).cumsum())
                    price_aligned = price.iloc[-min_len:]
                    cumret_fitted.index = price_aligned.index
                    dev = (price_aligned.values - cumret_fitted.values * price_aligned.values[0]) / (price_aligned.values + 1e-8)
                    features["arima_price_deviation"] = pd.Series(dev, index=price_aligned.index)
                    features["is_above_arima_forecast"] = (pd.Series(dev, index=price_aligned.index) > 0).astype(float)
            except Exception as e:
                logger.debug("residual_features ARIMA: %s", e)

        # ── ETS residual features ──────────────────────────────────────────
        if self._ets_ok and self._ets_model is not None:
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    ets_resid = pd.Series(
                        self._ets_model.resid.values if hasattr(self._ets_model.resid, "values") else np.array(self._ets_model.resid),
                        index=series.index[-len(self._ets_model.resid):]
                    )
                mu_ets    = ets_resid.rolling(w).mean()
                sigma_ets = ets_resid.rolling(w).std() + 1e-8
                features["ets_residual_zscore"] = (ets_resid - mu_ets) / sigma_ets
                features["ets_residual_ma"]     = ets_resid.rolling(w).mean()
            except Exception as e:
                logger.debug("residual_features ETS: %s", e)

        # ── Trend slope rolling ────────────────────────────────────────────
        def _trend_slope(arr: np.ndarray) -> float:
            if len(arr) < 2:
                return 0.0
            x = np.arange(len(arr), dtype=np.float64)
            norm = arr / (arr[0] + 1e-8)
            return float(np.polyfit(x, norm, 1)[0])

        features["trend_slope"] = price.rolling(w).apply(_trend_slope, raw=True)

        # ── Autocorrelación lag-1 (indica si el residual tiene memoria) ────
        def _autocorr1(arr: np.ndarray) -> float:
            if len(arr) < 4:
                return 0.0
            a = arr[:-1]
            b = arr[1:]
            std_a = a.std()
            std_b = b.std()
            if std_a < 1e-8 or std_b < 1e-8:
                return 0.0
            return float(np.corrcoef(a, b)[0, 1])

        log_ret = np.log(price / price.shift(1)).fillna(0)
        features["return_autocorr_lag1"] = log_ret.rolling(w).apply(_autocorr1, raw=True)

        result = pd.DataFrame(features, index=price.index)
        result = result.replace([np.inf, -np.inf], np.nan).ffill().fillna(0)
        return result

    # ── Convenience ───────────────────────────────────────────────────────────

    @property
    def is_fitted(self) -> bool:
        return self._is_fitted

    def arima_deviation(self, price_series: pd.Series) -> float:
        """Atajo: retorna deviation_from_forecast del modelo ARIMA para el precio actual."""
        if not self._arima_ok:
            return 0.0
        fc = self.forecast_now(price_series, model="arima")
        return fc.deviation_from_forecast

    def ets_deviation(self, price_series: pd.Series) -> float:
        """Atajo: retorna deviation_from_forecast del modelo ETS para el precio actual."""
        if not self._ets_ok:
            return 0.0
        fc = self.forecast_now(price_series, model="ets")
        return fc.deviation_from_forecast
