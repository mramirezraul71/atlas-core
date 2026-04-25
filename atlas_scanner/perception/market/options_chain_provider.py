from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import os
from typing import Any, Protocol

from atlas_scanner.perception.common.circuit_breaker import resolve_provider_circuit_breaker


class OptionsChainProvider(Protocol):
    def fetch_chain(self, *, symbol: str, as_of: datetime) -> dict[str, Any]:
        ...

    @property
    def last_diagnostics(self) -> dict[str, Any]:
        ...


@dataclass
class StubOptionsChainProvider:
    last_diagnostics: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "options_chain_stub"}

    def fetch_chain(self, *, symbol: str, as_of: datetime) -> dict[str, Any]:
        _ = symbol, as_of
        self.last_diagnostics = {
            "status": "degraded",
            "provider": "options_chain_stub",
            "provider_ready": False,
            "fallback_active": True,
        }
        return {}


@dataclass
class OpenBbOptionsChainProvider:
    timeout_sec: float = 6.0
    provider: str = "yfinance"
    last_diagnostics: dict[str, Any] | None = None
    obb_client: Any | None = None

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "options_chain_openbb"}

    def fetch_chain(self, *, symbol: str, as_of: datetime) -> dict[str, Any]:
        _ = as_of
        breaker = resolve_provider_circuit_breaker("market:options_chain_openbb")
        if not breaker.allow_request():
            self.last_diagnostics = {
                "status": "error",
                "provider": "options_chain_openbb",
                "error": "circuit_open",
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return {}
        try:
            obb = self._resolve_obb()
            if obb is None:
                raise RuntimeError("openbb_backend_unavailable")
            response = obb.derivatives.options.chains(symbol, provider=self.provider)  # type: ignore[attr-defined]
            rows = self._rows_from_response(response)
            chain = _normalize_chain_rows(rows)
            if chain["gamma_by_strike"]:
                breaker.record_success()
                self.last_diagnostics = {
                    "status": "ok",
                    "provider": "options_chain_openbb",
                    "provider_ready": True,
                    "fallback_active": False,
                    "rows": len(rows),
                    "circuit_breaker": _cb_dict(breaker.snapshot()),
                }
                return chain
            breaker.record_failure("empty_chain")
            self.last_diagnostics = {
                "status": "degraded",
                "provider": "options_chain_openbb",
                "provider_ready": False,
                "fallback_active": True,
                "rows": len(rows),
                "error": "empty_chain",
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return {}
        except Exception as exc:
            breaker.record_failure(str(exc))
            self.last_diagnostics = {
                "status": "error",
                "provider": "options_chain_openbb",
                "provider_ready": False,
                "fallback_active": True,
                "error": str(exc),
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return {}

    def _resolve_obb(self) -> Any | None:
        if self.obb_client is not None:
            return self.obb_client
        try:
            from openbb import obb  # type: ignore

            return obb
        except Exception:
            return None

    @staticmethod
    def _rows_from_response(response: object) -> list[dict[str, Any]]:
        to_df = getattr(response, "to_df", None)
        if callable(to_df):
            frame = to_df()
            if hasattr(frame, "to_dict"):
                return list(frame.to_dict("records"))
        results = getattr(response, "results", None)
        if isinstance(results, list):
            out: list[dict[str, Any]] = []
            for item in results:
                if isinstance(item, dict):
                    out.append(item)
                elif hasattr(item, "model_dump"):
                    out.append(item.model_dump())
            return out
        return []


def resolve_options_chain_provider() -> OptionsChainProvider:
    provider_name = os.getenv("ATLAS_OPTIONS_CHAIN_PROVIDER", "stub").strip().lower()
    if provider_name in {"openbb", "obb"}:
        return OpenBbOptionsChainProvider(
            timeout_sec=float(os.getenv("ATLAS_OPTIONS_CHAIN_TIMEOUT_SEC", "6.0")),
            provider=os.getenv("ATLAS_OPTIONS_CHAIN_OPENBB_BACKEND", "yfinance").strip() or "yfinance",
        )
    return StubOptionsChainProvider()


def _normalize_chain_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    gamma_by_strike: dict[float, float] = {}
    oi_by_strike: dict[float, float] = {}
    call_oi_by_strike: dict[float, float] = {}
    put_oi_by_strike: dict[float, float] = {}
    dte_weights: dict[str, float] = {"0-7": 1.25, "8-30": 1.0, "31+": 0.7}
    for row in rows:
        strike = _as_float(row.get("strike") or row.get("strike_price"))
        if strike is None:
            continue
        gamma = _as_float(row.get("gamma"))
        oi = _as_float(row.get("open_interest")) or _as_float(row.get("openInterest")) or 0.0
        option_type = str(row.get("option_type") or row.get("type") or "").lower()
        dte = _as_float(row.get("dte") or row.get("days_to_expiration"))
        if gamma is not None:
            gamma_by_strike[strike] = gamma_by_strike.get(strike, 0.0) + gamma
        oi_by_strike[strike] = oi_by_strike.get(strike, 0.0) + oi
        if option_type.startswith("c"):
            call_oi_by_strike[strike] = call_oi_by_strike.get(strike, 0.0) + oi
        if option_type.startswith("p"):
            put_oi_by_strike[strike] = put_oi_by_strike.get(strike, 0.0) + oi
        if dte is not None:
            if dte <= 7:
                dte_weights["0-7"] += 0.02
            elif dte <= 30:
                dte_weights["8-30"] += 0.01
    return {
        "gamma_by_strike": gamma_by_strike,
        "oi_by_strike": oi_by_strike,
        "call_oi_by_strike": call_oi_by_strike,
        "put_oi_by_strike": put_oi_by_strike,
        "dte_weights": dte_weights,
    }


def _as_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return None
    return None


def _cb_dict(snapshot: Any) -> dict[str, Any]:
    return {
        "state": getattr(snapshot, "state", "unknown"),
        "consecutive_failures": getattr(snapshot, "consecutive_failures", 0),
        "failure_threshold": getattr(snapshot, "failure_threshold", 0),
        "cooldown_sec": getattr(snapshot, "cooldown_sec", 0),
        "last_error": getattr(snapshot, "last_error", None),
    }
