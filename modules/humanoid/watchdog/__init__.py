"""Watchdog: health checks, latency/error rate rules, event + audit."""
from __future__ import annotations

from .engine import get_last_alerts, start_watchdog, stop_watchdog, watchdog_status

__all__ = ["start_watchdog", "stop_watchdog", "watchdog_status", "get_last_alerts"]
