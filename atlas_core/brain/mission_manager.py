"""Gestión ordenada de misión activa (sin planificador complejo)."""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field

logger = logging.getLogger("atlas.brain.mission_manager")


@dataclass
class _MissionEntry:
    name: str
    priority: int = 0
    paused: bool = False


class MissionManager:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._missions: dict[str, _MissionEntry] = {}
        self._active: str | None = None

    def set_mission(self, mission_name: str, priority: int = 0) -> None:
        with self._lock:
            self._missions[mission_name] = _MissionEntry(name=mission_name, priority=priority, paused=False)
            self._active = mission_name
            logger.info("mission set active=%s priority=%s", mission_name, priority)

    def get_active_mission(self) -> str | None:
        with self._lock:
            if self._active is None:
                return None
            ent = self._missions.get(self._active)
            if ent and ent.paused:
                return None
            return self._active

    def pause_mission(self, mission_name: str) -> None:
        with self._lock:
            if mission_name in self._missions:
                self._missions[mission_name].paused = True
                if self._active == mission_name:
                    self._active = None
                logger.info("mission paused %s", mission_name)

    def abort_mission(self, mission_name: str) -> None:
        with self._lock:
            self._missions.pop(mission_name, None)
            if self._active == mission_name:
                self._active = None
            logger.info("mission aborted %s", mission_name)
