"""
lotto_quant.execution.modes
===========================

Operating modes for the radar / broker stack.

PAPER  — Simulated. No real money. Outcomes drawn from the modeled prize
         distribution. Used for backtesting, demo, and CI.
LIVE   — Manual confirmation broker. Atlas records the intent (game, count,
         expected EV, recommended position) and waits for a human to confirm
         the physical purchase + scan the result. Atlas NEVER auto-executes
         lottery purchases.

The active mode is held in a module-level singleton, persisted to a small
state file so the radar and the HUD always agree.
"""

from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


class OperatingMode(str, Enum):
    PAPER = "paper"
    LIVE = "live"

    @classmethod
    def from_string(cls, value: str) -> "OperatingMode":
        v = (value or "").strip().lower()
        if v in ("live", "production", "prod", "real"):
            return cls.LIVE
        return cls.PAPER


_STATE_FILE = Path(
    os.getenv("ATLAS_MODE_STATE", "data/atlas_mode.json")
)


@dataclass
class ModeState:
    mode: OperatingMode = OperatingMode.PAPER
    paper_bankroll: float = 1_000.0
    live_bankroll: float = 0.0
    last_changed_iso: str = ""

    def to_dict(self) -> dict:
        return {
            "mode": self.mode.value,
            "paper_bankroll": self.paper_bankroll,
            "live_bankroll": self.live_bankroll,
            "last_changed_iso": self.last_changed_iso,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "ModeState":
        return cls(
            mode=OperatingMode.from_string(d.get("mode", "paper")),
            paper_bankroll=float(d.get("paper_bankroll", 1_000.0)),
            live_bankroll=float(d.get("live_bankroll", 0.0)),
            last_changed_iso=str(d.get("last_changed_iso", "")),
        )


_state: Optional[ModeState] = None


def _load() -> ModeState:
    global _state
    if _state is not None:
        return _state
    if _STATE_FILE.exists():
        try:
            _state = ModeState.from_dict(json.loads(_STATE_FILE.read_text()))
            return _state
        except Exception as e:  # pragma: no cover
            logger.warning("Failed to read mode state — defaulting to PAPER: %s", e)
    _state = ModeState()
    return _state


def _save(state: ModeState) -> None:
    _STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
    _STATE_FILE.write_text(json.dumps(state.to_dict(), indent=2))


def get_active_mode() -> OperatingMode:
    """Return the currently active operating mode."""
    return _load().mode


def get_state() -> ModeState:
    return _load()


def set_active_mode(
    mode: OperatingMode,
    *,
    paper_bankroll: Optional[float] = None,
    live_bankroll: Optional[float] = None,
) -> ModeState:
    """Persist the new operating mode and (optionally) bankroll."""
    from datetime import datetime, timezone

    state = _load()
    state.mode = mode
    if paper_bankroll is not None:
        state.paper_bankroll = float(paper_bankroll)
    if live_bankroll is not None:
        state.live_bankroll = float(live_bankroll)
    state.last_changed_iso = datetime.now(timezone.utc).isoformat(timespec="seconds")
    _save(state)
    logger.info("Atlas operating mode → %s", mode.value.upper())
    return state


def reset_state_for_tests() -> None:
    """Used by unit tests to reset the in-memory singleton AND on-disk state."""
    global _state
    _state = None
    if _STATE_FILE.exists():
        try:
            _STATE_FILE.unlink()
        except OSError:
            pass


def _clear_cache() -> None:
    """Clear in-memory singleton only (preserves file)."""
    global _state
    _state = None
