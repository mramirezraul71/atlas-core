"""IV Rank calculator — Fase A.

Metodología (resumen)
---------------------
**IV actual (iv_current):** se obtiene de la cadena de opciones Tradier (vía
``OptionChainCache`` + ``TradierClient.get_option_chain``). Se elige la
expiración cuya DTE sea la más cercana a ``dte_target`` (o la primera futura si
``dte_target`` es None). Sobre esa expiración, se toman contratos ATM: strikes
que enmarcan el spot (subyacente desde ``quote``), calls y puts, y se usa la
**mediana** de IV explícita (``greeks.mid_iv`` / ``smv_vol`` / ``implied_volatility``).

**Referencia histórica (ventana min/max):** el repositorio no persiste series
históricas de IV implícita. Para no inventar datos, la banda
``[iv_min_window, iv_max_window]`` se deriva de la **volatilidad realizada
anualizada** con ventana móvil de ``rv_window`` días sobre precios diarios
(``TradierClient.history``). Es decir, *no* es el clásico IV Rank vs 52
semanas de IV, sino un **proxy trazable**: posición del IV actual respecto al
rango histórico de volatilidad realizada del subyacente.

**iv_rank:** ``100 * (iv_current - iv_min_window) / (iv_max_window - iv_min_window)``,
acotado a [0, 100]. Si el denominador es ~0, se devuelve 50.0 y bandera
``degenerate_window``.

**iv_hv_ratio:** ``iv_current / hv_short`` donde ``hv_short`` es la volatilidad
realizada anualizada de los últimos ``rv_window`` cierres (misma ventana que el
rolling más reciente).

**Calidad:** ``ok`` si hay suficientes muestras; ``approx`` cuando la referencia
es RV; ``insufficient_history`` si faltan datos de precio.
"""
from __future__ import annotations

import logging
import math
from datetime import date, datetime, timedelta, timezone
from typing import Any, Protocol

import numpy as np

logger = logging.getLogger("atlas.options.iv_rank")

try:
    from execution.option_chain_cache import OptionChainCache
except ModuleNotFoundError:  # pragma: no cover
    from atlas_code_quant.execution.option_chain_cache import OptionChainCache


class _SupportsChain(Protocol):
    def quote(self, symbol: str) -> dict[str, Any]: ...

    def history(
        self,
        symbol: str,
        start: date,
        end: date,
        interval: str = "daily",
    ) -> list[dict[str, Any]]: ...

    def get_option_expirations(self, symbol: str, scope: str | None = None) -> list[str]: ...

    def get_option_chain(self, symbol: str, expiration: str, scope: str | None = None) -> list[dict[str, Any]]: ...


def _safe_float(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def _pick_spot(quote: dict[str, Any]) -> float:
    for key in ("last", "close", "bid", "ask", "prevclose"):
        v = _safe_float(quote.get(key), 0.0)
        if v > 0:
            return v
    raise ValueError("Unable to resolve spot price from quote")


def _option_iv(contract: dict[str, Any]) -> float | None:
    g = contract.get("greeks")
    if isinstance(g, dict):
        for key in ("mid_iv", "smv_vol", "iv"):
            v = g.get(key)
            if v is not None:
                fv = _safe_float(v, 0.0)
                if fv > 0:
                    return fv
    for key in ("implied_volatility", "iv"):
        v = contract.get(key)
        if v is not None:
            fv = _safe_float(v, 0.0)
            if fv > 0:
                return fv
    return None


def _strike_value(contract: dict[str, Any]) -> float:
    return _safe_float(contract.get("strike"), 0.0)


def _median(vals: list[float]) -> float | None:
    if not vals:
        return None
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    if n % 2:
        return float(s[mid])
    return float((s[mid - 1] + s[mid]) / 2.0)


def atm_iv_from_chain(chain: list[dict[str, Any]], spot: float) -> tuple[float | None, int]:
    """Mediana IV en strikes que enmarcan el spot (ATM ± 1 strike típico)."""
    if spot <= 0 or not chain:
        return None, 0
    strikes = sorted({_strike_value(c) for c in chain if _strike_value(c) > 0})
    if not strikes:
        return None, 0
    # strikes más cercanos al spot (hasta 2 strikes para tener calls+puts)
    nearest = min(range(len(strikes)), key=lambda i: abs(strikes[i] - spot))
    idx_lo = max(0, nearest - 1)
    idx_hi = min(len(strikes) - 1, nearest + 1)
    band = strikes[idx_lo : idx_hi + 1]
    ivs: list[float] = []
    for c in chain:
        k = _strike_value(c)
        if k in band:
            iv = _option_iv(c)
            if iv is not None:
                ivs.append(iv)
    if not ivs:
        for c in chain:
            if abs(_strike_value(c) - spot) <= 0.02 * spot + 1e-6:
                iv = _option_iv(c)
                if iv is not None:
                    ivs.append(iv)
    if not ivs:
        return None, 0
    m = _median(ivs)
    return (m, len(ivs)) if m is not None else (None, 0)


def pick_expiration_for_dte(
    expirations: list[str],
    dte_target: int | None,
    *,
    today: date | None = None,
) -> tuple[str | None, int | None, str | None]:
    """Elige YYYY-MM-DD más cercano a ``dte_target`` DTE; error si vacío."""
    if not expirations:
        return None, None, "no_expirations"
    today = today or datetime.now(timezone.utc).date()
    parsed: list[tuple[str, int]] = []
    for exp in expirations:
        try:
            d = date.fromisoformat(str(exp)[:10])
        except ValueError:
            continue
        dte = (d - today).days
        if dte > 0:
            parsed.append((str(exp)[:10], dte))
    if not parsed:
        return None, None, "no_future_expirations"
    if dte_target is None:
        best = min(parsed, key=lambda x: x[1])
        return best[0], best[1], None
    best = min(parsed, key=lambda x: abs(x[1] - dte_target))
    return best[0], best[1], None


def annualized_rv_from_closes(closes: np.ndarray, window: int) -> np.ndarray:
    """Serie de vol realizada anualizada (ventana móvil). longitud = len - window + 1."""
    if closes.size < window + 1:
        return np.array([])
    log_ret = np.diff(np.log(closes))
    out: list[float] = []
    for i in range(window - 1, len(log_ret)):
        chunk = log_ret[i - window + 1 : i + 1]
        if chunk.size < window:
            continue
        sigma = float(np.std(chunk, ddof=1))
        out.append(sigma * math.sqrt(252))
    return np.array(out, dtype=float)


def compute_iv_rank_linear(iv_current: float, lo: float, hi: float) -> tuple[float, str]:
    """iv_rank en [0,100]; bandera si ventana degenerada."""
    if hi <= lo:
        return 50.0, "degenerate_window"
    r = (iv_current - lo) / (hi - lo) * 100.0
    r = max(0.0, min(100.0, r))
    return r, "ok"


class IVRankCalculator:
    """Calcula IV actual, ratio IV/HV y un IV Rank proxy basado en rango de RV."""

    def __init__(
        self,
        tradier_client: _SupportsChain,
        chain_cache: OptionChainCache | None = None,
        *,
        scope: str = "paper",
        lookback_trading_days: int = 280,
        rv_window: int = 21,
        min_rv_samples: int = 60,
    ) -> None:
        self._client = tradier_client
        self._cache = chain_cache or OptionChainCache()
        self._scope = scope
        self._lookback = max(30, lookback_trading_days)
        self._rv_window = max(5, rv_window)
        self._min_rv_samples = max(10, min_rv_samples)

    def get_iv_rank(self, symbol: str, dte_target: int | None = None) -> dict[str, Any]:
        sym = str(symbol or "").strip().upper()
        if not sym:
            return self._error_payload(sym, "empty_symbol")

        try:
            quote = self._client.quote(sym)
            spot = _pick_spot(quote)
        except Exception as exc:
            logger.warning("IVRank quote failed %s: %s", sym, exc)
            return self._error_payload(sym, f"quote_error:{exc}")

        expirations = self._cache.get_expirations(sym, self._client, scope=self._scope)
        exp, dte, exp_err = pick_expiration_for_dte(expirations, dte_target)
        if exp_err or not exp:
            return {
                "symbol": sym,
                "dte_target": dte_target,
                "iv_current": None,
                "iv_rank": None,
                "iv_hv_ratio": None,
                "sample_size": 0,
                "method": "atm_median_chain",
                "quality": "insufficient_history",
                "error": exp_err or "expiration",
                "reference_basis": "realized_volatility_envelope",
            }

        chain = self._cache.get_or_fetch(sym, exp, self._client, scope=self._scope)
        if isinstance(chain, dict):
            opts = chain.get("options") if "options" in chain else []
            if isinstance(opts, list):
                chain = opts
        if not chain:
            return {
                "symbol": sym,
                "dte_target": dte_target,
                "expiration": exp,
                "dte": dte,
                "iv_current": None,
                "iv_rank": None,
                "iv_hv_ratio": None,
                "sample_size": 0,
                "method": "atm_median_chain",
                "quality": "insufficient_history",
                "error": "empty_chain",
                "reference_basis": "realized_volatility_envelope",
            }

        iv_cur, n_iv = atm_iv_from_chain(chain if isinstance(chain, list) else [], spot)
        if iv_cur is None:
            return {
                "symbol": sym,
                "dte_target": dte_target,
                "expiration": exp,
                "dte": dte,
                "iv_current": None,
                "iv_rank": None,
                "iv_hv_ratio": None,
                "sample_size": 0,
                "method": "atm_median_chain",
                "quality": "insufficient_history",
                "error": "no_atm_iv",
                "reference_basis": "realized_volatility_envelope",
            }

        end = datetime.now(timezone.utc).date()
        start = end - timedelta(days=int(self._lookback * 1.5))
        try:
            hist = self._client.history(sym, start, end, interval="daily")
        except Exception as exc:
            logger.warning("IVRank history failed %s: %s", sym, exc)
            return {
                "symbol": sym,
                "dte_target": dte_target,
                "expiration": exp,
                "dte": dte,
                "iv_current": round(iv_cur, 6),
                "iv_rank": None,
                "iv_hv_ratio": None,
                "sample_size": 0,
                "method": "atm_median_chain",
                "quality": "insufficient_history",
                "error": f"history_error:{exc}",
                "spot": round(spot, 4),
                "chain_iv_samples": n_iv,
                "reference_basis": "realized_volatility_envelope",
            }

        closes = np.array(
            [_safe_float(row.get("close"), 0.0) for row in hist],
            dtype=float,
        )
        closes = closes[closes > 0]
        if closes.size < self._rv_window + 2:
            return {
                "symbol": sym,
                "dte_target": dte_target,
                "expiration": exp,
                "dte": dte,
                "iv_current": round(iv_cur, 6),
                "iv_rank": None,
                "iv_hv_ratio": None,
                "sample_size": int(closes.size),
                "method": "atm_median_chain",
                "quality": "insufficient_history",
                "error": "history_too_short",
                "spot": round(spot, 4),
                "chain_iv_samples": n_iv,
                "reference_basis": "realized_volatility_envelope",
            }

        rv_series = annualized_rv_from_closes(closes, self._rv_window)
        iv_rank: float | None
        lo: float | None
        hi: float | None
        rank_flag: str
        if rv_series.size < self._min_rv_samples:
            iv_rank = None
            lo = hi = None
            rank_flag = "insufficient_rv_samples"
        else:
            lo = float(np.min(rv_series))
            hi = float(np.max(rv_series))
            iv_rank, rank_flag = compute_iv_rank_linear(iv_cur, lo, hi)

        # HV corto (última ventana completa)
        log_ret = np.diff(np.log(closes))
        if log_ret.size >= self._rv_window:
            hv_short = float(np.std(log_ret[-self._rv_window :], ddof=1) * math.sqrt(252))
        else:
            hv_short = float(np.std(log_ret, ddof=1) * math.sqrt(252)) if log_ret.size > 2 else 0.0

        iv_hv = (iv_cur / hv_short) if hv_short and hv_short > 1e-12 else None

        if iv_rank is None:
            quality: str = "insufficient_history"
        elif rank_flag == "degenerate_window":
            quality = "approx"
        else:
            quality = "ok"

        out: dict[str, Any] = {
            "symbol": sym,
            "dte_target": dte_target,
            "expiration": exp,
            "dte": dte,
            "iv_current": round(iv_cur, 6),
            "iv_rank": None if iv_rank is None else round(float(iv_rank), 2),
            "iv_hv_ratio": None if iv_hv is None else round(float(iv_hv), 4),
            "iv_min_window": None if lo is None else round(lo, 6),
            "iv_max_window": None if hi is None else round(hi, 6),
            "hv_short": None if hv_short <= 0 else round(hv_short, 6),
            "sample_size": int(rv_series.size) if rv_series.size else int(closes.size),
            "rv_window": self._rv_window,
            "method": "atm_median_iv + rv_envelope_rank",
            "quality": quality,
            "chain_iv_samples": n_iv,
            "spot": round(spot, 4),
            "reference_basis": "realized_volatility_envelope",
            "model_version": "rv_envelope_v1",
            "notes": (
                "iv_rank es proxy respecto a min/max de volatilidad realizada (no serie histórica de IV). "
                "quality=ok: rango RV no degenerado; approx: denominador ~0 (iv_rank=50)."
            ),
        }
        if rank_flag == "degenerate_window":
            out["warnings"] = ["rv_min_max_equal_iv_rank_mid"]
        elif rank_flag == "insufficient_rv_samples":
            out["warnings"] = ["rolling_rv_series_too_short_for_rank"]
        return out

    def _error_payload(self, sym: str, err: str) -> dict[str, Any]:
        return {
            "symbol": sym,
            "dte_target": None,
            "iv_current": None,
            "iv_rank": None,
            "iv_hv_ratio": None,
            "sample_size": 0,
            "method": "atm_median_chain",
            "quality": "insufficient_history",
            "error": err,
            "reference_basis": "realized_volatility_envelope",
        }
