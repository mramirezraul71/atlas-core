from __future__ import annotations

from dataclasses import dataclass
from datetime import date, timedelta
from math import log, sqrt
from statistics import pstdev
from typing import Any

from atlas_scanner.ports.vol_macro_provider import MacroData, VolData, VolMacroProvider


def _as_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _extract_records(response: object) -> list[dict[str, Any]]:
    if response is None:
        return []

    to_df = getattr(response, "to_df", None)
    if callable(to_df):
        try:
            dataframe = to_df()
            if hasattr(dataframe, "to_dict"):
                return list(dataframe.to_dict("records"))
        except Exception:
            return []

    results = getattr(response, "results", None)
    if isinstance(results, list):
        normalized: list[dict[str, Any]] = []
        for item in results:
            if isinstance(item, dict):
                normalized.append(item)
                continue
            if hasattr(item, "model_dump"):
                try:
                    normalized.append(item.model_dump())
                    continue
                except Exception:
                    pass
            if hasattr(item, "__dict__"):
                normalized.append(dict(item.__dict__))
        return normalized

    return []


def _extract_close_series(records: list[dict[str, Any]]) -> list[float]:
    closes: list[float] = []
    for row in records:
        for key in ("close", "adj_close", "last", "value"):
            value = _as_float(row.get(key))
            if value is not None:
                closes.append(value)
                break
    return closes


def _compute_realized_vol(close_series: list[float], horizon: int) -> float | None:
    if horizon <= 1 or len(close_series) < horizon + 1:
        return None

    window = close_series[-(horizon + 1) :]
    returns: list[float] = []
    for idx in range(1, len(window)):
        prev = window[idx - 1]
        curr = window[idx]
        if prev <= 0 or curr <= 0:
            continue
        returns.append(log(curr / prev))
    if len(returns) < 2:
        return None

    return pstdev(returns) * sqrt(252.0)


@dataclass
class OpenBBVolMacroProvider(VolMacroProvider):
    iv_lookback_days: int = 100
    rv_horizons_days: tuple[int, ...] = (5, 10, 20, 60)
    obb_client: Any | None = None

    def _resolve_obb_client(self) -> Any | None:
        if self.obb_client is not None:
            return self.obb_client
        try:
            from openbb import obb  # type: ignore

            return obb
        except Exception:
            return None

    def get_vol_data(self, symbol: str, as_of: date) -> VolData:
        obb = self._resolve_obb_client()
        if obb is None:
            return VolData()

        start = as_of - timedelta(days=self.iv_lookback_days)

        iv_history: tuple[float, ...] = ()
        try:
            chain_response = obb.derivatives.options.chains(
                symbol,
                start_date=start.isoformat(),
                end_date=as_of.isoformat(),
            )
            chain_records = _extract_records(chain_response)
            iv_values: list[float] = []
            for row in chain_records:
                for key in ("implied_volatility", "iv", "mark_iv"):
                    value = _as_float(row.get(key))
                    if value is not None:
                        iv_values.append(value)
                        break
            iv_history = tuple(iv_values)
        except Exception:
            iv_history = ()

        rv_annualized: dict[str, float] = {}
        try:
            price_response = obb.equity.price.historical(
                symbol,
                start_date=start.isoformat(),
                end_date=as_of.isoformat(),
            )
            price_records = _extract_records(price_response)
            close_series = _extract_close_series(price_records)
            for horizon in self.rv_horizons_days:
                rv_value = _compute_realized_vol(close_series=close_series, horizon=horizon)
                if rv_value is not None:
                    rv_annualized[f"{horizon}d"] = rv_value
        except Exception:
            rv_annualized = {}

        iv_current = iv_history[-1] if iv_history else None
        return VolData(
            iv_history=iv_history,
            iv_current=iv_current,
            rv_annualized=rv_annualized,
        )

    def get_macro_data(self, as_of: date) -> MacroData:
        obb = self._resolve_obb_client()
        if obb is None:
            return MacroData()

        vix_value: float | None = None
        try:
            start = as_of - timedelta(days=30)
            vix_response = obb.equity.price.historical(
                "^VIX",
                start_date=start.isoformat(),
                end_date=as_of.isoformat(),
            )
            vix_records = _extract_records(vix_response)
            close_series = _extract_close_series(vix_records)
            if close_series:
                vix_value = close_series[-1]
        except Exception:
            vix_value = None

        macro_regime: str | None = None
        if vix_value is not None:
            if vix_value <= 20.0:
                macro_regime = "favorable"
            elif vix_value <= 28.0:
                macro_regime = "neutral"
            else:
                macro_regime = "adverse"

        # Simple deterministic seasonal proxy in [0.8, 1.2].
        seasonal_factor = 0.8 + ((as_of.month - 1) / 11.0) * 0.4
        return MacroData(
            vix=vix_value,
            macro_regime=macro_regime,
            seasonal_factor=seasonal_factor,
        )

