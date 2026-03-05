from __future__ import annotations

import sys
from pathlib import Path


AGENT_DIR = Path(__file__).resolve().parents[2] / "tools" / "atlas_agent"
if str(AGENT_DIR) not in sys.path:
    sys.path.insert(0, str(AGENT_DIR))

from config import AgentConfig  # noqa: E402
from episodic_memory import EpisodicMemory  # noqa: E402


def _cfg(tmp_path: Path) -> AgentConfig:
    return AgentConfig(
        workspace=tmp_path,
        runs_dir=tmp_path / "runs",
        memory_db=tmp_path / "memory" / "episodes.sqlite",
    )


def test_episodic_memory_lexical_fallback(monkeypatch, tmp_path: Path):
    memory = EpisodicMemory(_cfg(tmp_path))

    # Force fallback path (no embeddings) so the test is network-free.
    def _raise_embed(_: str):
        raise RuntimeError("embedding unavailable")

    monkeypatch.setattr(memory, "_embed", _raise_embed)

    memory.add_episode(
        goal="Fix dashboard polling",
        summary="Repaired visibilitychange polling and stale metrics refresh loop",
        success=True,
    )
    memory.add_episode(
        goal="Update docs",
        summary="Edited README and examples",
        success=True,
    )

    results = memory.similar("dashboard metrics polling stale", top_k=1)
    assert len(results) == 1
    assert "dashboard" in (results[0]["goal"] + " " + results[0]["summary"]).lower()
