from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from atlas_code_quant.options.options_pipeline_runtime import run_options_pipeline_cycle


class _FakeAtlas:
    def __init__(self) -> None:
        self.events: list[dict[str, Any]] = []

    def log_event(self, event: dict[str, Any]) -> None:
        self.events.append(dict(event))


def test_run_options_pipeline_cycle_logs_start_and_finish():
    atlas = _FakeAtlas()

    def _runner(current_time: datetime, wrapped_atlas: Any, *, limits: dict[str, Any]) -> dict[str, Any]:
        assert current_time.tzinfo is not None
        assert limits["max_trades_per_cycle"] == 5
        return {
            "ok": True,
            "universe_size": 12,
            "opportunities": 7,
            "selected": 3,
            "emitted": 2,
            "created": 2,
            "blocked": 1,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }

    out = run_options_pipeline_cycle(atlas, pipeline_runner=_runner)
    assert out["ok"] is True
    assert any(e.get("event_type") == "options_pipeline_cycle_started" for e in atlas.events)
    finish = next(e for e in atlas.events if e.get("event_type") == "options_pipeline_cycle_finished")
    assert finish["symbols_evaluated"] == 12
    assert finish["signals_emitted"] == 2
    assert finish["entries_blocked"] == 1


def test_run_options_pipeline_cycle_logs_failed_when_runner_raises():
    atlas = _FakeAtlas()

    def _boom(*_: Any, **__: Any) -> dict[str, Any]:
        raise RuntimeError("simulated_pipeline_failure")

    out = run_options_pipeline_cycle(atlas, pipeline_runner=_boom)
    assert out["ok"] is False
    failed = next(e for e in atlas.events if e.get("event_type") == "options_pipeline_cycle_failed")
    assert "simulated_pipeline_failure" in str(failed.get("error"))

