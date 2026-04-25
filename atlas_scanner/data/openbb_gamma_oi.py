from __future__ import annotations

from dataclasses import dataclass, field
from datetime import date, timedelta
from typing import Any

from atlas_scanner.ports.gamma_oi_provider import (
    GammaData,
    GammaOIProvider,
    OIFlowData,
    StrikeGammaData,
)


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


def _extract_strike(row: dict[str, Any]) -> float | None:
    for key in ("strike", "strike_price", "exercise_price"):
        value = _as_float(row.get(key))
        if value is not None:
            return value
    return None


def _extract_gamma_value(row: dict[str, Any]) -> float | None:
    for key in ("gamma", "option_gamma", "greeks_gamma"):
        value = _as_float(row.get(key))
        if value is not None:
            return value
    return None


def _extract_call_gamma(row: dict[str, Any]) -> float | None:
    for key in ("call_gamma", "call_greeks_gamma"):
        value = _as_float(row.get(key))
        if value is not None:
            return value
    return None


def _extract_put_gamma(row: dict[str, Any]) -> float | None:
    for key in ("put_gamma", "put_greeks_gamma"):
        value = _as_float(row.get(key))
        if value is not None:
            return value
    return None


def _extract_option_side(row: dict[str, Any]) -> str | None:
    for key in ("option_type", "right", "type"):
        raw = row.get(key)
        if not isinstance(raw, str):
            continue
        value = raw.strip().upper()
        if value in {"C", "CALL"}:
            return "CALL"
        if value in {"P", "PUT"}:
            return "PUT"
    return None


def _extract_volume(row: dict[str, Any]) -> float | None:
    for key in ("volume", "option_volume", "trade_volume"):
        value = _as_float(row.get(key))
        if value is not None:
            return max(0.0, value)
    return None


def _extract_open_interest(row: dict[str, Any]) -> float | None:
    for key in ("open_interest", "oi"):
        value = _as_float(row.get(key))
        if value is not None:
            return max(0.0, value)
    return None


@dataclass
class OpenBBGammaOIProvider(GammaOIProvider):
    """
    OpenBB-based Gamma/OI adapter.

    Current scanner fields mapped:
    - GammaData.strikes (strike/call_gamma/put_gamma)
    - GammaData.net_gex (if present or derived sum)
    - OIFlowData.call_volume / put_volume
    - OIFlowData.call_put_volume_ratio
    - OIFlowData.volume_imbalance

    Optional / deferred mappings:
    - oi_change_1d_pct (requires stable prior-day OI support in vendor payload)
    - richer strike-level flow analytics (left in `meta` for later phases)
    """

    lookback_days: int = 5
    obb_client: Any | None = None
    diagnostics: dict[str, str] = field(default_factory=dict)

    def _resolve_obb_client(self) -> Any | None:
        if self.obb_client is not None:
            return self.obb_client
        try:
            from openbb import obb  # type: ignore

            return obb
        except Exception:
            return None

    def _set_diagnostic(self, key: str, status: str) -> None:
        self.diagnostics[key] = status

    def get_diagnostics(self) -> dict[str, str]:
        return dict(self.diagnostics)

    def _load_option_rows(self, symbol: str, as_of: date) -> tuple[list[dict[str, Any]], str]:
        obb = self._resolve_obb_client()
        if obb is None:
            return [], "no_backend"
        try:
            start = as_of - timedelta(days=max(1, self.lookback_days))
            response = obb.derivatives.options.chains(
                symbol,
                start_date=start.isoformat(),
                end_date=as_of.isoformat(),
            )
            rows = _extract_records(response)
            return rows, "ok" if rows else "empty"
        except Exception:
            return [], "error"

    def get_gamma_data(self, symbol: str, as_of: date) -> GammaData:
        rows, source_status = self._load_option_rows(symbol=symbol, as_of=as_of)
        if not rows:
            self._set_diagnostic("gamma_data", source_status)
            return GammaData()

        strike_state: dict[float, dict[str, float]] = {}
        net_gex_accumulator = 0.0
        net_gex_seen = False

        for row in rows:
            strike = _extract_strike(row)
            if strike is None:
                continue

            # If explicit per-row net GEX exists, aggregate it.
            for key in ("net_gex", "gex", "gamma_exposure", "net_gamma_exposure"):
                value = _as_float(row.get(key))
                if value is not None:
                    net_gex_accumulator += value
                    net_gex_seen = True
                    break

            state = strike_state.setdefault(strike, {"call_gamma": 0.0, "put_gamma": 0.0})

            explicit_call = _extract_call_gamma(row)
            explicit_put = _extract_put_gamma(row)
            if explicit_call is not None:
                state["call_gamma"] += explicit_call
            if explicit_put is not None:
                state["put_gamma"] += explicit_put
            if explicit_call is not None or explicit_put is not None:
                continue

            gamma_value = _extract_gamma_value(row)
            if gamma_value is None:
                continue
            side = _extract_option_side(row)
            if side == "CALL":
                state["call_gamma"] += gamma_value
            elif side == "PUT":
                state["put_gamma"] += gamma_value

        strikes = tuple(
            StrikeGammaData(
                strike=strike,
                call_gamma=state["call_gamma"],
                put_gamma=state["put_gamma"],
            )
            for strike, state in sorted(strike_state.items(), key=lambda item: item[0])
            if state["call_gamma"] != 0.0 or state["put_gamma"] != 0.0
        )
        if not strikes and not net_gex_seen:
            self._set_diagnostic("gamma_data", "empty")
            return GammaData()
        self._set_diagnostic("gamma_data", "ok")
        return GammaData(
            strikes=strikes,
            net_gex=net_gex_accumulator if net_gex_seen else None,
        )

    def get_oi_flow_data(self, symbol: str, as_of: date) -> OIFlowData:
        rows, source_status = self._load_option_rows(symbol=symbol, as_of=as_of)
        if not rows:
            self._set_diagnostic("oi_flow_data", source_status)
            return OIFlowData()

        call_volume = 0.0
        put_volume = 0.0
        call_oi = 0.0
        put_oi = 0.0
        oi_change_values: list[float] = []
        ohlcv_rows = 0

        for row in rows:
            side = _extract_option_side(row)
            if side is None:
                continue
            volume = _extract_volume(row)
            open_interest = _extract_open_interest(row)
            if volume is not None:
                if side == "CALL":
                    call_volume += volume
                else:
                    put_volume += volume
            if open_interest is not None:
                if side == "CALL":
                    call_oi += open_interest
                else:
                    put_oi += open_interest
            for key in ("oi_change_1d_pct", "open_interest_change_pct", "oi_change_pct"):
                value = _as_float(row.get(key))
                if value is not None:
                    oi_change_values.append(value)
                    break
            ohlcv_rows += 1

        total_volume = call_volume + put_volume
        ratio: float | None = None
        imbalance: float | None = None
        if total_volume > 0:
            ratio = call_volume / put_volume if put_volume > 0 else None
            imbalance = (call_volume - put_volume) / total_volume

        oi_change_1d_pct: float | None = None
        if oi_change_values:
            oi_change_1d_pct = sum(oi_change_values) / len(oi_change_values)

        if call_volume <= 0 and put_volume <= 0 and call_oi <= 0 and put_oi <= 0 and oi_change_1d_pct is None:
            self._set_diagnostic("oi_flow_data", "empty")
            return OIFlowData()

        self._set_diagnostic("oi_flow_data", "ok")
        return OIFlowData(
            oi_change_1d_pct=oi_change_1d_pct,
            call_put_volume_ratio=ratio,
            volume_imbalance=imbalance,
            call_volume=call_volume if call_volume > 0 else None,
            put_volume=put_volume if put_volume > 0 else None,
            meta={
                "call_open_interest": call_oi,
                "put_open_interest": put_oi,
                "option_rows_used": float(ohlcv_rows),
            },
        )

