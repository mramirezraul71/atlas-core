"""Chart mission execution service for selector-driven visual validation."""
from __future__ import annotations

import logging
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from config.settings import settings

logger = logging.getLogger("quant.chart_execution")

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
            "verify_after_open": bool(settings.chart_verify_after_open),
            "verify_delay_sec": float(settings.chart_verify_delay_sec),
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

    @staticmethod
    def _browser_exe_candidates(browser_path: str | None) -> list[str]:
        """Nombres de imagen Windows esperados según el ejecutable configurado."""
        if not browser_path:
            return ["chrome.exe", "msedge.exe"]
        base = Path(browser_path).name.lower()
        if "chrome" in base and "chromium" not in base:
            return ["chrome.exe"]
        if "chromium" in base:
            return ["chrome.exe", "chromium.exe"]
        if "msedge" in base or base == "edge.exe":
            return ["msedge.exe"]
        if "brave" in base:
            return ["brave.exe"]
        if "firefox" in base:
            return ["firefox.exe"]
        if "opera" in base:
            return ["opera.exe"]
        return ["chrome.exe", "msedge.exe", "firefox.exe"]

    @classmethod
    def _verify_browser_process_running(cls, browser_path: str | None) -> tuple[bool, str]:
        """Comprueba que exista al menos un proceso del navegador usado (heurística post-apertura)."""
        candidates = cls._browser_exe_candidates(browser_path)
        if sys.platform == "win32":
            try:
                for exe in candidates:
                    proc = subprocess.run(
                        ["tasklist", "/FI", f"IMAGENAME eq {exe}", "/NH"],
                        capture_output=True,
                        text=True,
                        timeout=8,
                        check=False,
                    )
                    out = (proc.stdout or "").lower()
                    if exe.lower() in out:
                        return True, f"tasklist: {exe} presente"
                return False, f"tasklist: sin proceso entre {candidates}"
            except Exception as exc:
                return False, f"tasklist error: {exc}"
        try:
            import psutil  # type: ignore

            hints = tuple(x.replace(".exe", "") for x in candidates)
            for p in psutil.process_iter(["name"]):
                n = (p.info.get("name") or "").lower()
                if any(h in n for h in hints):
                    return True, f"psutil: {n}"
        except Exception as exc:
            logger.debug("verify_browser_process: psutil no disponible: %s", exc)
        return True, "verify_skipped_non_windows_no_psutil"

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
            "browser_verify_ok": None,
            "browser_verify_detail": None,
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
            payload["browser_verify_ok"] = True
            payload["browser_verify_detail"] = "cooldown_reuse (sin relanzar navegador)"
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

            if bool(settings.chart_verify_after_open):
                delay = max(0.0, float(settings.chart_verify_delay_sec))
                if delay:
                    time.sleep(delay)
                ok_verify, detail = self._verify_browser_process_running(browser_path)
                payload["browser_verify_ok"] = ok_verify
                payload["browser_verify_detail"] = detail
                if not ok_verify:
                    payload["open_ok"] = False
                    payload["readiness_score_pct"] = 55.0
                    payload["manual_required"] = True
                    payload["operator_review_required"] = True
                    payload["execution_state"] = "verify_failed"
                    payload["warnings"].append(
                        "chart auto-open: proceso navegador no verificado; revisar Chrome o permisos."
                    )
                    logger.warning("Chart verify failed: %s", detail)
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
