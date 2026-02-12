"""Dispatch: unified local/remote execution for hands, web, vision, voice."""
from __future__ import annotations

from .dispatcher import run_hands, run_web, run_vision, run_voice

__all__ = ["run_hands", "run_web", "run_vision", "run_voice"]
