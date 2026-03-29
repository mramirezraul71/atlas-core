"""Chart mission execution service for selector-driven visual validation."""
from __future__ import annotations

import subprocess
import time
from datetime import datetime
from typing import Any

from config.settings import settings

try:
    from atlas_code_quant.chart_launcher import _chrome_path
except Exception:  # pragma: no cover - fallback path discovery remains optional
    _chrome_path = None  # type: ignore[assignment]


class ChartExecutionService:
    """Opens selector chart targets in a browser and tracks readiness for visual gating.

    The service is intentionally conservative:
    - it only auto-opens when explicitly enabled via settings
    - it avoids repeated browser launches within a cooldown window
    - it reports structured readiness even when auto-open is disabled
    """

    def __init__(self) -> None:
        self._last_signature = ""
        self._last_open_at = 0.0
        self._last_payload: dict[str, Any] | None = None

    def _detect_browser(self) -> str | None:
        if callable(_chrome_path):
            try:
                return _chrome_path()
            except Exception:
                return None
        return None

    def status(self) -> dict[str, Any]:
        browser_path = self._detect_browser()
        return {
            "auto_open_enabled": bool(settings.chart_auto_open_enabled),
            "browser_path": browser_path,
            "browser_available": bool(browser_path),
            "cooldown_sec": int(settings.chart_open_cooldown_sec),
            "last_open_at": (
                datetime.utcfromtimestamp(self._last_open_at).isoformat()
                if self._last_open_at > 0
                else None
            ),
            "last_payload": self._last_payload or {},
        }

    def ensure_chart_mission(
        self,
        *,
        chart_plan: dict[str, Any] | None,
        camera_plan: dict[str, Any] | None,
        symbol: str,
    ) -> dict[str, Any]:
        plan = chart_plan or {}
        camera = camera_plan or {}
        targets = [dict(target) for target in (plan.get("targets") or []) if isinstance(target, dict)]
        requested_timeframes = [str(target.get("timeframe") or "") for target in targets]
        urls = [str(target.get("url") or "").strip() for target in targets if str(target.get("url") or "").strip()]
        provider = str(plan.get("provider") or "unknown")
        browser_path = self._detect_browser()
        auto_open_enabled = bool(settings.chart_auto_open_enabled)
        signature = "|".join(urls)
        cooldown_active = bool(signature) and signature == self._last_signature and (time.monotonic() - self._last_open_at) < int(settings.chart_open_cooldown_sec)

        payload = {
            "generated_at": datetime.utcnow().isoformat(),
            "requested": bool(targets),
            "provider": provider,
            "target_count": len(targets),
            "targets": targets,
            "requested_timeframes": requested_timeframes,
            "camera_required": bool(camera.get("required")),
            "camera_provider": str(camera.get("provider") or ""),
            "auto_open_enabled": auto_open_enabled,
            "browser_available": bool(browser_path),
            "open_attempted": False,
            "open_ok": False,
            "open_mode": "disabled",
            "cooldown_active": cooldown_active,
            "symbol_match": all(symbol.upper() in str(target.get("title") or "").upper() or symbol.upper() in str(target.get("url") or "").upper() for target in targets) if targets else False,
            "timeframe_match": all(bool(target.get("timeframe")) for target in targets) if targets else False,
            "manual_required": False,
            "operator_review_required": False,
            "execution_state": "not_requested",
            "readiness_score_pct": 100.0 if not targets else 0.0,
            "warnings": [],
        }

        if not targets:
            payload["open_mode"] = "not_requested"
            payload["execution_state"] = "not_requested"
            self._last_payload = dict(payload)
            return payload

        if not auto_open_enabled:
            payload["warnings"].append("chart auto-open is disabled in settings; visual mission remains manual.")
            payload["open_mode"] = "manual_required"
            payload["manual_required"] = True
            payload["operator_review_required"] = True
            payload["execution_state"] = "manual_required"
            payload["readiness_score_pct"] = 35.0
            self._last_payload = dict(payload)
            return payload

        if not browser_path:
            payload["warnings"].append("browser path could not be resolved for chart auto-open.")
            payload["open_mode"] = "browser_unavailable"
            payload["manual_required"] = True
            payload["operator_review_required"] = True
            payload["execution_state"] = "browser_unavailable"
            payload["readiness_score_pct"] = 25.0
            self._last_payload = dict(payload)
            return payload

        if cooldown_active:
            payload["open_mode"] = "cooldown_reuse"
            payload["open_ok"] = True
            payload["execution_state"] = "cooldown_reuse"
            payload["readiness_score_pct"] = 90.0
            self._last_payload = dict(payload)
            return payload

        try:
            subprocess.Popen([browser_path, "--new-window", *urls])  # noqa: S603
            payload["open_attempted"] = True
            payload["open_ok"] = True
            payload["open_mode"] = "browser_urls"
            payload["execution_state"] = "opened"
            payload["readiness_score_pct"] = 100.0
            self._last_signature = signature
            self._last_open_at = time.monotonic()
        except Exception as exc:
            payload["open_attempted"] = True
            payload["open_ok"] = False
            payload["open_mode"] = "error"
            payload["manual_required"] = True
            payload["operator_review_required"] = True
            payload["execution_state"] = "error"
            payload["readiness_score_pct"] = 20.0
            payload["warnings"].append(f"chart auto-open error: {exc}")

        self._last_payload = dict(payload)
        return payload
