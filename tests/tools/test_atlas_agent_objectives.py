from __future__ import annotations

import sys
from pathlib import Path


AGENT_DIR = Path(__file__).resolve().parents[2] / "tools" / "atlas_agent"
if str(AGENT_DIR) not in sys.path:
    sys.path.insert(0, str(AGENT_DIR))

from config import AgentConfig  # noqa: E402
from objectives import ObjectivePlanner  # noqa: E402


class _FailPlanner:
    def complete(self, messages):  # noqa: ANN001
        raise RuntimeError("llm unavailable")

    def parse_json(self, text):  # noqa: ANN001
        return {}


def _cfg(tmp_path: Path) -> AgentConfig:
    return AgentConfig(
        workspace=tmp_path,
        runs_dir=tmp_path / "runs",
        memory_db=tmp_path / "memory" / "episodes.sqlite",
    )


def test_build_fallback_to_single_objective(tmp_path: Path):
    planner = ObjectivePlanner(_cfg(tmp_path), _FailPlanner())
    objectives = planner.build("Stabilize telemetry loop")
    assert len(objectives) == 1
    assert objectives[0]["status"] == "pending"
    assert "Stabilize telemetry loop" in objectives[0]["title"]


def test_reprioritize_noop_when_llm_fails(tmp_path: Path):
    planner = ObjectivePlanner(_cfg(tmp_path), _FailPlanner())
    current = [
        {"id": "o1", "title": "a", "priority": "high", "status": "pending"},
        {"id": "o2", "title": "b", "priority": "medium", "status": "pending"},
    ]
    updated = planner.reprioritize(current, {"result": "x"})
    assert updated == current
