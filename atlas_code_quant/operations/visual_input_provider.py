"""Unified visual input provider for paper-aggressive workflows.

This provider prioritizes pragmatic runtime paths on Windows:
1) camera frame (USB webcam / Insta360 webcam mode),
2) screen frame fallback,
3) chart frame via TradingView/browser workflow.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]

try:
    from atlas_code_quant.chart_launcher import ChartLauncher
except Exception:  # pragma: no cover
    ChartLauncher = None  # type: ignore[assignment]

try:
    from atlas_code_quant.vision.insta360_capture import InstaCapture
except Exception:  # pragma: no cover
    InstaCapture = None  # type: ignore[assignment]


@dataclass
class VisualFrame:
    ok: bool
    frame: np.ndarray | None = None
    provider_used: str = "none"
    used_fallback: bool = False
    fallback_reason: str = ""
    screenshot_path: str = ""
    metadata: dict[str, Any] | None = None


class VisualInputProvider:
    """Paper-only visual input source abstraction."""

    def __init__(self) -> None:
        self._camera_index = int(os.getenv("ATLAS_VISUAL_CAMERA_INDEX", os.getenv("NEXUS_CAMERA_INDEX", "0")))
        self._prefer_desktop = str(os.getenv("ATLAS_VISUAL_PREFER_DESKTOP", "false")).lower() in {"1", "true", "yes"}
        self._evidence_dir = Path(
            os.getenv(
                "ATLAS_VISUAL_EVIDENCE_DIR",
                str(Path(__file__).resolve().parents[1] / "runtime_artifacts" / "visual_evidence"),
            )
        )
        self._evidence_dir.mkdir(parents=True, exist_ok=True)
        self._capture = InstaCapture(camera_index=self._camera_index, prefer_desktop=self._prefer_desktop) if InstaCapture else None
        self._chart_launcher = ChartLauncher(symbols=[os.getenv("ATLAS_VISUAL_DEFAULT_SYMBOL", "SPY")]) if ChartLauncher else None

    @staticmethod
    def _utc_stamp() -> str:
        return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")

    def _save_frame(self, frame: np.ndarray, *, prefix: str) -> str:
        if cv2 is None:
            return ""
        out = self._evidence_dir / f"{prefix}_{self._utc_stamp()}.png"
        try:
            cv2.imwrite(str(out), frame)
            return str(out)
        except Exception:
            return ""

    def get_camera_frame(self) -> VisualFrame:
        if self._capture is None:
            return VisualFrame(ok=False, provider_used="camera", fallback_reason="insta_capture_unavailable")
        cap = self._capture.capture(timeout_sec=4.0)
        if cap.ok and cap.frame is not None:
            screenshot = self._save_frame(cap.frame, prefix="camera")
            return VisualFrame(
                ok=True,
                frame=cap.frame,
                provider_used=str(cap.source or "camera"),
                screenshot_path=screenshot,
                metadata={"latency_ms": cap.latency_ms},
            )
        return VisualFrame(
            ok=False,
            provider_used="camera",
            used_fallback=True,
            fallback_reason=str(cap.error or "camera_capture_failed"),
        )

    def get_screen_frame(self) -> VisualFrame:
        if self._capture is None:
            return VisualFrame(ok=False, provider_used="screen", fallback_reason="insta_capture_unavailable")
        cap = self._capture._desktop()  # noqa: SLF001 - controlled fallback backend
        if cap.ok and cap.frame is not None:
            screenshot = self._save_frame(cap.frame, prefix="screen")
            return VisualFrame(
                ok=True,
                frame=cap.frame,
                provider_used="desktop_capture",
                screenshot_path=screenshot,
            )
        return VisualFrame(ok=False, provider_used="screen", fallback_reason=str(cap.error or "screen_capture_failed"))

    def get_chart_frame(
        self,
        *,
        source: str = "tradingview",
        symbol: str = "SPY",
        timeframe: str = "5m",
    ) -> VisualFrame:
        source_n = str(source or "tradingview").strip().lower()
        if source_n not in {"tradingview", "yahoo", "browser"}:
            source_n = "tradingview"

        # Preferred path: launcher screenshot from real browser chart tab.
        if self._chart_launcher is not None:
            try:
                self._chart_launcher.symbols = [symbol]
                if str(os.getenv("ATLAS_VISUAL_OPEN_CHARTS", "true")).lower() in {"1", "true", "yes"}:
                    self._chart_launcher.launch_free_tradingview(fullscreen=False)
                raw = self._chart_launcher.capture_screenshot(symbol=symbol)
                if raw:
                    if cv2 is not None:
                        arr = np.frombuffer(raw, dtype=np.uint8)
                        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    else:
                        frame = None
                    if frame is not None:
                        screenshot = self._save_frame(frame, prefix=f"chart_{symbol}_{timeframe}")
                        return VisualFrame(
                            ok=True,
                            frame=frame,
                            provider_used=f"{source_n}_chart_launcher",
                            screenshot_path=screenshot,
                            metadata={
                                "symbol": str(symbol).upper(),
                                "timeframe": str(timeframe).lower(),
                                "requested_source": source_n,
                                "capture_ok": True,
                            },
                        )
            except Exception as exc:
                fallback_reason = f"chart_launcher_failed:{exc}"
            else:
                fallback_reason = "chart_launcher_empty_screenshot"
        else:
            fallback_reason = "chart_launcher_unavailable"

        # Fallback path: screen capture (still real-time visual evidence).
        screen = self.get_screen_frame()
        if screen.ok:
            screen.used_fallback = True
            screen.fallback_reason = fallback_reason
            screen.provider_used = f"{source_n}_screen_fallback"
            screen.metadata = {
                "symbol": str(symbol).upper(),
                "timeframe": str(timeframe).lower(),
                "requested_source": source_n,
                "capture_ok": bool(screen.ok and screen.frame is not None),
                "fallback": True,
            }
            return screen
        return VisualFrame(
            ok=False,
            provider_used=source_n,
            used_fallback=True,
            fallback_reason=fallback_reason,
        )
