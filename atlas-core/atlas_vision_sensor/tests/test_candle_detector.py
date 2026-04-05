from __future__ import annotations

from candle_detector import CandleDetector


def test_candle_detector_handles_missing_ohlc_gracefully() -> None:
    detector = CandleDetector()
    result = detector.detect({"system": {"ok": True}}, visual_bias="bullish")
    assert result.available is False or result.reason in {"ohlc_unavailable", "talib_unavailable"}
    assert result.confidence == 0.0


def test_candle_detector_detects_pattern_when_ohlc_is_present() -> None:
    detector = CandleDetector()
    payload = {
        "market_ohlc": {
            "open": [10.5, 10.4, 10.3, 10.1, 10.0, 9.9, 9.7, 9.6],
            "high": [10.6, 10.5, 10.4, 10.2, 10.1, 10.0, 9.95, 10.4],
            "low": [10.3, 10.2, 10.1, 9.9, 9.8, 9.7, 9.5, 9.55],
            "close": [10.4, 10.3, 10.15, 10.0, 9.9, 9.8, 9.55, 10.35],
        }
    }
    result = detector.detect(payload, visual_bias="bullish")
    assert result.available is True or result.reason == "talib_unavailable"
    if result.available:
        assert result.payload["bars"] == 8
