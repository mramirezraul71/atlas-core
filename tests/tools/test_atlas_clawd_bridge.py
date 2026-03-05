from __future__ import annotations

import sys
from pathlib import Path


BRIDGE_DIR = Path(__file__).resolve().parents[2] / "tools" / "atlas_clawd_bridge"
if str(BRIDGE_DIR) not in sys.path:
    sys.path.insert(0, str(BRIDGE_DIR))

from bridge import AtlasClawdBridge, ClawdBridgeConfig  # noqa: E402


def _write_snapshot(log_path: Path, *, stable: bool) -> None:
    lines = [
        "2026-01-01T00:00:00Z === SNAPSHOT_SAFE_START ===",
        "2026-01-01T00:00:00Z PUSH /health => 200",
        "2026-01-01T00:00:00Z NEXUS /health => 200",
        "2026-01-01T00:00:00Z ROBOT /status => 200",
    ]
    if stable:
        lines.append("2026-01-01T00:00:00Z SMOKE Results: 16/16 passed, 0 failed")
    else:
        lines.append("2026-01-01T00:00:00Z SMOKE Results: 15/16 passed, 1 failed")
    lines.append("2026-01-01T00:00:01Z === SNAPSHOT_SAFE_END ===")
    log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _bridge(tmp_path: Path, *, stable: bool) -> AtlasClawdBridge:
    snapshot_log = tmp_path / "logs" / "snapshot_safe_diagnostic.log"
    snapshot_log.parent.mkdir(parents=True, exist_ok=True)
    _write_snapshot(snapshot_log, stable=stable)

    pan = tmp_path / "_external" / "rauli-panaderia"
    vis = tmp_path / "_external" / "RAULI-VISION"
    pan.mkdir(parents=True, exist_ok=True)
    vis.mkdir(parents=True, exist_ok=True)

    cfg = ClawdBridgeConfig(
        repo_root=tmp_path,
        snapshot_script=tmp_path / "scripts" / "atlas_snapshot_safe.ps1",
        snapshot_log=snapshot_log,
        require_stable=True,
        allowed_repos=("panaderia", "vision"),
        panaderia_repo=pan,
        vision_repo=vis,
        action_timeout_sec=20,
    )
    return AtlasClawdBridge(cfg)


def test_parse_latest_snapshot_stable(tmp_path: Path):
    bridge = _bridge(tmp_path, stable=True)
    parsed = bridge.parse_latest_snapshot()
    assert parsed["ok"] is True
    assert parsed["stable"] is True
    assert parsed["checks"]["smoke_results_0_failed"] is True


def test_parse_latest_snapshot_unstable(tmp_path: Path):
    bridge = _bridge(tmp_path, stable=False)
    parsed = bridge.parse_latest_snapshot()
    assert parsed["ok"] is True
    assert parsed["stable"] is False
    assert parsed["checks"]["smoke_results_0_failed"] is False


def test_execute_action_blocks_when_unstable(monkeypatch, tmp_path: Path):
    bridge = _bridge(tmp_path, stable=False)
    monkeypatch.setenv("ATLAS_CENTRAL_CORE", "token-123")
    result = bridge.execute_action(
        action_id="a1",
        target_repo="panaderia",
        command="python -c \"print('ok')\"",
        token="token-123",
        run_snapshot=False,
        timeout_sec=10,
    )
    assert result["ok"] is False
    assert result["error"] == "blocked_unstable_system"


def test_execute_action_ok_when_stable(monkeypatch, tmp_path: Path):
    bridge = _bridge(tmp_path, stable=True)
    monkeypatch.setenv("ATLAS_CENTRAL_CORE", "token-abc")
    result = bridge.execute_action(
        action_id="a2",
        target_repo="panaderia",
        command="python -c \"print('ok')\"",
        token="token-abc",
        run_snapshot=False,
        timeout_sec=20,
    )
    assert result["ok"] is True
    assert result["returncode"] == 0
    assert result["target_repo"] == "panaderia"
