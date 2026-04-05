from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

try:
    import talib
except Exception:  # pragma: no cover
    talib = None  # type: ignore[assignment]


_PATTERN_FUNCTIONS: dict[str, tuple[str, str]] = {
    "doji": ("CDLDOJI", "neutral"),
    "engulfing": ("CDLENGULFING", "directional"),
    "hammer": ("CDLHAMMER", "bullish"),
    "hanging_man": ("CDLHANGINGMAN", "bearish"),
    "morning_star": ("CDLMORNINGSTAR", "bullish"),
    "evening_star": ("CDLEVENINGSTAR", "bearish"),
}


@dataclass(slots=True)
class CandleHit:
    pattern: str
    direction: str
    score: int
    confidence: float
    reason: str


@dataclass(slots=True)
class CandleDetectionResult:
    available: bool
    confidence: float
    hits: list[CandleHit] = field(default_factory=list)
    payload: dict[str, Any] = field(default_factory=dict)
    reason: str = ""


class CandleDetector:
    """TA-Lib-first candlestick detector over external OHLC telemetry."""

    def detect(
        self,
        telemetry_snapshot: dict[str, Any] | None,
        *,
        visual_bias: str | None = None,
    ) -> CandleDetectionResult:
        if talib is None:
            return CandleDetectionResult(
                available=False,
                confidence=0.0,
                reason="talib_unavailable",
                payload={"available": False, "reason": "talib_unavailable"},
            )
        ohlc = self._extract_ohlc(telemetry_snapshot or {})
        if ohlc is None:
            return CandleDetectionResult(
                available=False,
                confidence=0.0,
                reason="ohlc_unavailable",
                payload={"available": False, "reason": "ohlc_unavailable"},
            )

        open_, high, low, close = ohlc
        hits: list[CandleHit] = []
        for name, (func_name, family) in _PATTERN_FUNCTIONS.items():
            values = getattr(talib, func_name)(open_, high, low, close)
            last = int(values[-1]) if len(values) else 0
            if last == 0:
                continue
            direction = self._resolve_direction(family, last)
            confidence = min(1.0, 0.62 + min(abs(last) / 250.0, 0.28))
            if visual_bias and direction in {"bullish", "bearish"} and visual_bias == direction:
                confidence = min(1.0, confidence + 0.08)
            hits.append(
                CandleHit(
                    pattern=name,
                    direction=direction,
                    score=last,
                    confidence=round(confidence, 4),
                    reason=f"talib:{func_name}",
                )
            )

        hits.sort(key=lambda item: item.confidence, reverse=True)
        top_confidence = hits[0].confidence if hits else 0.0
        return CandleDetectionResult(
            available=True,
            confidence=round(top_confidence, 4),
            hits=hits,
            reason="ok" if hits else "no_pattern",
            payload={
                "available": True,
                "bars": int(len(close)),
                "visual_bias": visual_bias or "unknown",
                "top_pattern": hits[0].pattern if hits else "",
                "top_direction": hits[0].direction if hits else "unknown",
            },
        )

    @staticmethod
    def _resolve_direction(family: str, score: int) -> str:
        if family == "neutral":
            return "neutral"
        if family == "directional":
            return "bullish" if score > 0 else "bearish"
        return family

    @staticmethod
    def _coerce_array(values: list[Any]) -> np.ndarray | None:
        try:
            arr = np.asarray(values, dtype=float)
        except Exception:
            return None
        if arr.ndim != 1 or arr.size < 5 or not np.isfinite(arr).all():
            return None
        return arr

    def _extract_ohlc(self, payload: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
        candidates = [
            payload.get("market_ohlc"),
            (payload.get("market") or {}).get("ohlc"),
            (payload.get("telemetry") or {}).get("market_ohlc"),
            payload.get("ohlc"),
        ]
        for candidate in candidates:
            parsed = self._parse_ohlc(candidate)
            if parsed is not None:
                return parsed
        return None

    def _parse_ohlc(self, candidate: Any) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
        if isinstance(candidate, dict):
            open_ = self._coerce_array(list(candidate.get("open") or []))
            high = self._coerce_array(list(candidate.get("high") or []))
            low = self._coerce_array(list(candidate.get("low") or []))
            close = self._coerce_array(list(candidate.get("close") or []))
            if all(item is not None for item in (open_, high, low, close)):
                return open_, high, low, close  # type: ignore[return-value]
            return None
        if isinstance(candidate, list) and candidate and isinstance(candidate[0], dict):
            open_ = self._coerce_array([item.get("open") for item in candidate])
            high = self._coerce_array([item.get("high") for item in candidate])
            low = self._coerce_array([item.get("low") for item in candidate])
            close = self._coerce_array([item.get("close") for item in candidate])
            if all(item is not None for item in (open_, high, low, close)):
                return open_, high, low, close  # type: ignore[return-value]
        return None
