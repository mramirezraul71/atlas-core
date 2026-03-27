from __future__ import annotations

import time
from pathlib import Path

from atlas_code_quant.operations.operation_center import OperationCenter
from config.settings import settings


class _SlowTracker:
    def build_summary(self, **_: object) -> dict:
        time.sleep(2.0)
        return {
            "account_session": {"scope": "paper"},
            "balances": {"total_equity": 100000.0},
            "alerts": [],
            "totals": {"open_positions": 0},
            "strategies": [],
            "pdt_status": {},
            "refresh_interval_sec": 5,
        }


class _Vision:
    def status(self) -> dict:
        return {"provider": "desktop_capture", "provider_ready": True}


class _Executor:
    def status(self) -> dict:
        return {"mode": "paper_api", "kill_switch_active": False}


class _Brain:
    def status(self) -> dict:
        return {"last_memory_ok": True, "last_error": ""}


class _Learning:
    def status(self, account_scope: str | None = None) -> dict:
        return {"account_scope": account_scope, "policy_ready": True}


class _Journal:
    def snapshot(self, limit: int = 3) -> dict:
        return {"recent_entries_count": 0, "recent_entries": [], "limit": limit}


def test_operation_status_uses_timeout_fallback_without_blocking(tmp_path: Path):
    original_timeout = settings.tradier_timeout_sec
    original_ttl = settings.tradier_monitor_cache_ttl_sec
    settings.tradier_timeout_sec = 1
    settings.tradier_monitor_cache_ttl_sec = 300
    try:
        center = OperationCenter(
            tracker=_SlowTracker(),
            journal=_Journal(),
            vision=_Vision(),
            executor=_Executor(),
            brain=_Brain(),
            learning=_Learning(),
            state_path=tmp_path / "operation_center_state.json",
        )

        started = time.perf_counter()
        payload = center.status()
        elapsed = time.perf_counter() - started
    finally:
        settings.tradier_timeout_sec = original_timeout
        settings.tradier_monitor_cache_ttl_sec = original_ttl

    assert elapsed < 1.8
    assert payload["config"]["kill_switch_active"] is False
    assert payload["monitor_summary"]["balances"] == {}
    assert payload["monitor_summary"]["totals"] == {}
    assert payload["monitor_summary"]["alerts"]
    assert "timeout" in payload["monitor_summary"]["alerts"][0]["message"].lower()
