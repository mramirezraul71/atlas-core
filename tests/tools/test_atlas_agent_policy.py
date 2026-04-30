from __future__ import annotations

import sys
from pathlib import Path


AGENT_DIR = Path(__file__).resolve().parents[2] / "tools" / "atlas_agent"
if str(AGENT_DIR) not in sys.path:
    sys.path.insert(0, str(AGENT_DIR))

from config import AgentConfig  # noqa: E402
from policy import ApprovalPolicyEngine  # noqa: E402


def _cfg(tmp_path: Path, mode: str = "safe") -> AgentConfig:
    return AgentConfig(
        mode=mode,
        workspace=tmp_path,
        runs_dir=tmp_path / "runs",
        memory_db=tmp_path / "episodes.sqlite",
    )


def test_safe_mode_requires_approval_for_mutation(tmp_path: Path):
    policy = ApprovalPolicyEngine(_cfg(tmp_path, mode="safe"))
    read_decision = policy.decide("read_file", {"path": "a.txt"})
    assert read_decision.allowed is True
    assert read_decision.requires_approval is False

    write_decision = policy.decide("write_file", {"path": "a.txt", "content": "x"})
    assert write_decision.allowed is False
    assert write_decision.requires_approval is True

    shell_decision = policy.decide("run_shell", {"command": "python -m pytest"})
    assert shell_decision.allowed is False
    assert shell_decision.requires_approval is True


def test_aggressive_mode_allows_write_and_safe_shell(tmp_path: Path):
    policy = ApprovalPolicyEngine(_cfg(tmp_path, mode="aggressive"))
    write_decision = policy.decide("write_file", {"path": "a.txt", "content": "x"})
    assert write_decision.allowed is True
    assert write_decision.requires_approval is False

    shell_safe = policy.decide("run_shell", {"command": "python -m pytest tests/tools -q"})
    assert shell_safe.allowed is True
    assert shell_safe.requires_approval is False

    shell_risky = policy.decide("run_shell", {"command": "pip install -U everything"})
    assert shell_risky.allowed is False
    assert shell_risky.requires_approval is True


def test_overrides_take_precedence(tmp_path: Path):
    policy = ApprovalPolicyEngine(
        _cfg(tmp_path, mode="safe"),
        approval_overrides={"write_file": True, "run_shell": False},
    )
    write_decision = policy.decide("write_file", {"path": "a.txt", "content": "x"})
    assert write_decision.allowed is True
    assert write_decision.requires_approval is False

    shell_decision = policy.decide("run_shell", {"command": "python -m pytest"})
    assert shell_decision.allowed is False
    assert shell_decision.requires_approval is False
