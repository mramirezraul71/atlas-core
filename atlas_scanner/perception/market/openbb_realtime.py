from __future__ import annotations

import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Mapping

from atlas_scanner.contracts import MarketPerceptionSnapshot, RadarTimeframe

_TIMEFRAME_TO_OPENBB_INTERVAL: dict[RadarTimeframe, str] = {
    "1m": "1m",
    "5m": "5m",
    "15m": "15m",
    "1h": "1h",
    "1d": "1d",
}


@dataclass(frozen=True)
class ProviderAttempt:
    provider: str
    status: str
    latency_ms: int
    error: str | None = None


@dataclass(frozen=True)
class RealtimeFetchResult:
    snapshot: MarketPerceptionSnapshot
    provider_used: str
    attempts: tuple[ProviderAttempt, ...]
    latency_ms: int
    from_cache: bool


@dataclass
class OpenBBRealtimeAdapter:
    providers_chain: tuple[str, ...] = ("polygon", "yfinance", "intrinio")
    cache_ttl_sec: int = 15
    _cache: dict[tuple[str, RadarTimeframe], tuple[float, RealtimeFetchResult]] = field(default_factory=dict)
    obb_client: Any | None = None

    def fetch(
        self,
        *,
        symbol: str,
        timeframe: RadarTimeframe,
        lookback_bars: int = 120,
    ) -> RealtimeFetchResult:
        key = (symbol.upper(), timeframe)
        now = time.time()
        cached = self._cache.get(key)
        if cached is not None and (now - cached[0]) <= self.cache_ttl_sec:
            snapshot = cached[1].snapshot
            snapshot_cached = MarketPerceptionSnapshot(
                symbol=snapshot.symbol,
                timeframe=snapshot.timeframe,
                as_of=snapshot.as_of,
                spot_price=snapshot.spot_price,
                close_price=snapshot.close_price,
                return_pct=snapshot.return_pct,
                momentum_score=snapshot.momentum_score,
                relative_volume=snapshot.relative_volume,
                volume_acceleration=snapshot.volume_acceleration,
                diagnostics={**snapshot.diagnostics, "cache": "hit"},
                meta=snapshot.meta,
            )
            return RealtimeFetchResult(
                snapshot=snapshot_cached,
                provider_used=cached[1].provider_used,
                attempts=cached[1].attempts,
                latency_ms=cached[1].latency_ms,
                from_cache=True,
            )

        attempts: list[ProviderAttempt] = []
        for provider in self.providers_chain:
            t0 = time.perf_counter()
            try:
                bars = self._fetch_bars(symbol=symbol, timeframe=timeframe, lookback_bars=lookback_bars, provider=provider)
                latency_ms = int((time.perf_counter() - t0) * 1000)
                if not bars:
                    attempts.append(ProviderAttempt(provider=provider, status="empty", latency_ms=latency_ms))
                    continue
                snapshot = self._normalize_bars(
                    symbol=symbol,
                    timeframe=timeframe,
                    bars=bars,
                    provider_used=provider,
                    latency_ms=latency_ms,
                    is_delayed=(provider == "yfinance"),
                )
                result = RealtimeFetchResult(
                    snapshot=snapshot,
                    provider_used=provider,
                    attempts=tuple(attempts + [ProviderAttempt(provider=provider, status="ok", latency_ms=latency_ms)]),
                    latency_ms=latency_ms,
                    from_cache=False,
                )
                self._cache[key] = (now, result)
                return result
            except Exception as exc:
                latency_ms = int((time.perf_counter() - t0) * 1000)
                attempts.append(
                    ProviderAttempt(provider=provider, status="error", latency_ms=latency_ms, error=str(exc))
                )

        degraded_snapshot = MarketPerceptionSnapshot(
            symbol=symbol.upper(),
            timeframe=timeframe,
            as_of=datetime.now(timezone.utc),
            diagnostics={
                "provider_status": "error",
                "degradation": "missing_market_data",
            },
            meta={"provider_chain": self.providers_chain},
        )
        return RealtimeFetchResult(
            snapshot=degraded_snapshot,
            provider_used="none",
            attempts=tuple(attempts),
            latency_ms=max((item.latency_ms for item in attempts), default=0),
            from_cache=False,
        )

    def _fetch_bars(
        self,
        *,
        symbol: str,
        timeframe: RadarTimeframe,
        lookback_bars: int,
        provider: str,
    ) -> list[dict[str, float]]:
        # OpenBB runtime is opt-in. If missing, fail fast and allow fallback chain.
        obb = self._resolve_obb_client()
        if obb is None:
            raise RuntimeError("openbb_backend_unavailable")
        interval = _TIMEFRAME_TO_OPENBB_INTERVAL[timeframe]
        response = obb.equity.price.historical(  # type: ignore[attr-defined]
            symbol,
            provider=provider,
            interval=interval,
        )
        records = self._extract_records(response)
        if lookback_bars > 0:
            return records[-lookback_bars:]
        return records

    def _resolve_obb_client(self) -> Any | None:
        if self.obb_client is not None:
            return self.obb_client
        try:
            from openbb import obb  # type: ignore

            return obb
        except Exception:
            return None

    @staticmethod
    def _extract_records(response: object) -> list[dict[str, float]]:
        to_df = getattr(response, "to_df", None)
        if callable(to_df):
            frame = to_df()
            if hasattr(frame, "to_dict"):
                return list(frame.to_dict("records"))
        results = getattr(response, "results", None)
        if isinstance(results, list):
            normalized: list[dict[str, float]] = []
            for row in results:
                if isinstance(row, dict):
                    normalized.append(row)
                elif hasattr(row, "model_dump"):
                    normalized.append(row.model_dump())
            return normalized
        return []

    @staticmethod
    def _normalize_bars(
        *,
        symbol: str,
        timeframe: RadarTimeframe,
        bars: list[dict[str, float]],
        provider_used: str,
        latency_ms: int,
        is_delayed: bool,
    ) -> MarketPerceptionSnapshot:
        closes = [float(row["close"]) for row in bars if "close" in row]
        volumes = [float(row.get("volume", 0.0)) for row in bars]
        close_price = closes[-1] if closes else None
        prev_close = closes[-2] if len(closes) > 1 else None
        return_pct = ((close_price - prev_close) / prev_close) * 100.0 if close_price and prev_close else None
        momentum_score = 50.0
        if return_pct is not None:
            momentum_score = max(0.0, min(100.0, 50.0 + return_pct * 4.0))
        avg_volume = (sum(volumes[:-1]) / max(1, len(volumes[:-1]))) if len(volumes) > 1 else None
        latest_volume = volumes[-1] if volumes else None
        relative_volume = (latest_volume / avg_volume) if latest_volume is not None and avg_volume and avg_volume > 0 else None
        volume_acceleration = None
        if len(volumes) >= 3 and volumes[-2] > 0:
            volume_acceleration = (volumes[-1] - volumes[-2]) / volumes[-2]
        return MarketPerceptionSnapshot(
            symbol=symbol.upper(),
            timeframe=timeframe,
            as_of=datetime.now(timezone.utc),
            spot_price=close_price,
            close_price=close_price,
            return_pct=return_pct,
            momentum_score=momentum_score,
            relative_volume=relative_volume,
            volume_acceleration=volume_acceleration,
            diagnostics={
                "provider_status": "ok",
                "provider_used": provider_used,
                "latency_ms": str(latency_ms),
                "is_delayed": "true" if is_delayed else "false",
            },
            meta={"provider_used": provider_used, "latency_ms": latency_ms, "is_delayed": is_delayed},
        )
