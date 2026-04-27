"""Tests F8 — KillSwitchWatcher + compat."""
from __future__ import annotations

from pathlib import Path

import pytest

from atlas_code_quant.autonomy.kill_switch import (
    KillSwitchWatcher,
    is_kill_switch_active,
)


def test_compat_is_kill_switch_active_empty_path() -> None:
    assert is_kill_switch_active("") is False


def test_compat_is_kill_switch_active_existing(tmp_path: Path) -> None:
    f = tmp_path / "kill"
    assert is_kill_switch_active(str(f)) is False
    f.touch()
    assert is_kill_switch_active(str(f)) is True


def test_watcher_returns_false_when_no_path() -> None:
    w = KillSwitchWatcher(path="")
    assert w.is_active() is False


def test_watcher_detects_creation_via_force_refresh(tmp_path: Path) -> None:
    f = tmp_path / "kill"
    w = KillSwitchWatcher(path=str(f), ttl_seconds=10.0)
    assert w.is_active() is False
    f.touch()
    # Por el TTL, sin force_refresh el cache puede mantener False
    assert w.force_refresh() is True
    assert w.is_active() is True


def test_watcher_cache_respects_ttl(tmp_path: Path) -> None:
    f = tmp_path / "kill"
    w = KillSwitchWatcher(path=str(f), ttl_seconds=10.0)
    assert w.is_active() is False
    f.touch()
    # cache fresco -> sigue False sin force
    assert w.is_active() is False


def test_watcher_from_env(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    f = tmp_path / "ks"
    monkeypatch.setenv("ATLAS_KILL_SWITCH_FILE", str(f))
    monkeypatch.setenv("ATLAS_KILL_SWITCH_TTL_SECONDS", "0.0")
    w = KillSwitchWatcher.from_env()
    assert w.path == str(f)
    assert w.is_active() is False
    f.touch()
    assert w.is_active() is True
