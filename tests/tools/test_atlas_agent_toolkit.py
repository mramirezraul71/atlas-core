from __future__ import annotations

import sys
from pathlib import Path

import pytest


AGENT_DIR = Path(__file__).resolve().parents[2] / "tools" / "atlas_agent"
if str(AGENT_DIR) not in sys.path:
    sys.path.insert(0, str(AGENT_DIR))

from config import AgentConfig  # noqa: E402
from toolkit import AtlasToolkit  # noqa: E402


def _cfg(tmp_path: Path) -> AgentConfig:
    return AgentConfig(
        workspace=tmp_path,
        runs_dir=tmp_path / "runs",
        allow_shell=False,
        dry_run_tools=False,
    )


def test_write_and_read_file(tmp_path: Path):
    tk = AtlasToolkit(_cfg(tmp_path))
    write = tk.write_file("notes/demo.txt", "hello atlas")
    assert write["ok"] is True
    assert write["changed"] is True
    assert isinstance(write["diff"], str)
    assert write["after_hash"]

    read = tk.read_file("notes/demo.txt")
    assert read["ok"] is True
    assert "hello atlas" in read["content"]

    write_same = tk.write_file("notes/demo.txt", "hello atlas")
    assert write_same["ok"] is True
    assert write_same["changed"] is False
    assert write_same["before_hash"] == write_same["after_hash"]


def test_path_traversal_blocked(tmp_path: Path):
    tk = AtlasToolkit(_cfg(tmp_path))
    result = tk.read_file("../secrets.txt")
    assert result["ok"] is False
    assert "workspace" in result["error"]


def test_dispatch_unknown_tool(tmp_path: Path):
    tk = AtlasToolkit(_cfg(tmp_path))
    result = tk.dispatch("unknown_tool", {})
    assert result["ok"] is False
    assert "unknown tool" in result["error"]


@pytest.mark.parametrize("cmd", ["echo ok", "Get-Date"])
def test_shell_disabled(tmp_path: Path, cmd: str):
    tk = AtlasToolkit(_cfg(tmp_path))
    result = tk.run_shell(cmd)
    assert result["ok"] is False
    assert "disabled" in result["error"]
