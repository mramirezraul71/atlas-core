"""Scheduler: persistent SQLite jobs, background engine, runner (update_check, shell, llm_plan)."""
from __future__ import annotations

from .db import SchedulerDB
from .engine import (get_scheduler_db, is_scheduler_running, request_cancel,
                     request_scheduler_restart, start_scheduler, stop_scheduler)
from .models import JobSpec

__all__ = [
    "SchedulerDB",
    "JobSpec",
    "get_scheduler_db",
    "start_scheduler",
    "stop_scheduler",
    "request_scheduler_restart",
    "is_scheduler_running",
    "request_cancel",
]
