from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from atlas_code_quant.options.options_pipeline_runtime import run_options_pipeline_cycle


class _FakeAtlas:
    def __init__(self) -> None:
        self.events: list[dict[str, Any]] = []

    def log_event(self, event: dict[str, Any]) -> None:
        self.events.append(dict(event))


def test_run_options_pipeline_cycle_rejects_non_paper_runtime():
    atlas = _FakeAtlas()

    out = run_options_pipeline_cycle(
        atlas,
        config={
            "enabled": True,
            "paper_only": False,
            "market_interval_seconds": 300,
            "premarket_interval_seconds": 900,
            "max_trades_per_cycle": 2,
            "dedupe_window_minutes": 30,
            "max_corr": 0.75,
        },
    )

    assert out["ok"] is False
    assert out["error"] == "options_pipeline_runtime_requires_paper_only"
    assert any(event.get("cycle_id") for event in atlas.events)


def test_run_options_pipeline_cycle_emits_cycle_id_on_success():
    atlas = _FakeAtlas()

    def _runner(current_time: datetime, wrapped_atlas: Any, *, limits: dict[str, Any]) -> dict[str, Any]:
        assert current_time.tzinfo is not None
        return {
            "ok": True,
            "universe_size": 3,
            "opportunities": 2,
            "selected": 1,
            "emitted": 1,
            "created": 1,
            "blocked": 0,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }

    out = run_options_pipeline_cycle(atlas, pipeline_runner=_runner)
    assert out["ok"] is True
    assert out["cycle_id"]
    finished = next(event for event in atlas.events if event.get("event_type") == "options_pipeline_cycle_finished")
    assert finished["cycle_id"] == out["cycle_id"]
