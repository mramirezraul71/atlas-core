"""
ResourceArbiter: Arbitro de recursos para ejecucion concurrente de goals.

Gestiona acceso exclusivo y compartido a recursos del sistema (vision, hands,
web, voice, llm, compute). Previene deadlocks mediante ordering global y
soporta preemption por prioridad.
"""
from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set

_log = logging.getLogger("humanoid.cortex.frontal.resource_arbiter")


@dataclass
class ResourceSlot:
    """Definicion de un recurso gestionado."""
    name: str
    max_slots: int = 1
    exclusive: bool = True
    holders: Dict[str, int] = field(default_factory=dict)

    @property
    def available(self) -> int:
        used = sum(self.holders.values())
        return max(0, self.max_slots - used)

    @property
    def is_free(self) -> bool:
        return self.available > 0

    def held_by(self, goal_id: str) -> bool:
        return goal_id in self.holders


@dataclass
class WaitEntry:
    """Entrada en la cola de espera de un recurso."""
    goal_id: str
    priority: int
    requested_at: float = field(default_factory=time.time)
    timeout_s: float = 30.0

    def is_expired(self) -> bool:
        return (time.time() - self.requested_at) > self.timeout_s


# Orden global de recursos para prevencion de deadlocks.
# Siempre adquirir en este orden: vision < hands < voice < web < llm < compute
RESOURCE_ORDER = {
    "vision": 0,
    "hands": 1,
    "voice": 2,
    "web": 3,
    "llm": 4,
    "compute": 5,
}


def _env_int(key: str, default: int) -> int:
    try:
        return int(os.getenv(key, str(default)))
    except ValueError:
        return default


class ResourceArbiter:
    """
    Arbitro de recursos con preemption por prioridad y prevencion de deadlocks.

    Recursos exclusivos (1 slot): vision, hands, voice
    Recursos compartidos: web (3), llm (2), compute (4)
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._resources: Dict[str, ResourceSlot] = {
            "vision":  ResourceSlot("vision",  max_slots=1, exclusive=True),
            "hands":   ResourceSlot("hands",   max_slots=1, exclusive=True),
            "voice":   ResourceSlot("voice",   max_slots=1, exclusive=True),
            "web":     ResourceSlot("web",     max_slots=_env_int("CGE_RESOURCE_SLOTS_WEB", 3), exclusive=False),
            "llm":     ResourceSlot("llm",     max_slots=_env_int("CGE_RESOURCE_SLOTS_LLM", 2), exclusive=False),
            "compute": ResourceSlot("compute", max_slots=_env_int("CGE_RESOURCE_SLOTS_COMPUTE", 4), exclusive=False),
        }
        self._wait_queues: Dict[str, List[WaitEntry]] = {r: [] for r in self._resources}
        self._goal_priorities: Dict[str, int] = {}

    def register_goal_priority(self, goal_id: str, priority: int) -> None:
        with self._lock:
            self._goal_priorities[goal_id] = priority

    def unregister_goal(self, goal_id: str) -> None:
        with self._lock:
            self._goal_priorities.pop(goal_id, None)
            for res in self._resources.values():
                res.holders.pop(goal_id, None)
            for q in self._wait_queues.values():
                self._wait_queues[res.name] = [e for e in q if e.goal_id != goal_id]

    # -- Acquire / Release --------------------------------------------------

    def acquire(self, goal_id: str, resources: List[str],
                priority: int = 1) -> bool:
        """
        Intenta adquirir multiples recursos atomicamente.
        Respeta orden global para prevencion de deadlocks.
        Retorna True si todos adquiridos, False si alguno fallo.
        """
        sorted_res = sorted(resources, key=lambda r: RESOURCE_ORDER.get(r, 99))

        with self._lock:
            if not all(self._can_acquire(r, goal_id) for r in sorted_res):
                return False
            for r in sorted_res:
                self._do_acquire(r, goal_id)
            self._goal_priorities[goal_id] = priority
            return True

    def try_acquire(self, goal_id: str, resources: List[str],
                    priority: int = 1, timeout_s: float = 10.0) -> bool:
        """Intenta adquirir con espera limitada."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.acquire(goal_id, resources, priority):
                return True
            time.sleep(0.1)
        return False

    def release(self, goal_id: str, resources: Optional[List[str]] = None) -> None:
        """Libera recursos. Si resources=None, libera todos los del goal."""
        with self._lock:
            if resources is None:
                resources = [r for r, slot in self._resources.items()
                             if slot.held_by(goal_id)]
            for r in resources:
                slot = self._resources.get(r)
                if slot and slot.held_by(goal_id):
                    slot.holders.pop(goal_id, None)
                    _log.debug("Released %s from %s", r, goal_id)

    def release_all(self, goal_id: str) -> List[str]:
        """Libera todos los recursos del goal. Retorna lista de liberados."""
        with self._lock:
            released = []
            for name, slot in self._resources.items():
                if slot.held_by(goal_id):
                    slot.holders.pop(goal_id, None)
                    released.append(name)
            return released

    # -- Preemption ---------------------------------------------------------

    def preempt(self, requester_id: str, resource: str,
                requester_priority: int) -> Optional[str]:
        """
        Desaloja al holder de menor prioridad si el requester tiene mayor prioridad.
        Retorna goal_id desalojado, o None si no se pudo.
        """
        with self._lock:
            slot = self._resources.get(resource)
            if not slot or slot.is_free:
                self._do_acquire(resource, requester_id)
                return None

            if not slot.holders:
                return None

            lowest_id = min(
                slot.holders.keys(),
                key=lambda gid: self._goal_priorities.get(gid, 0)
            )
            lowest_prio = self._goal_priorities.get(lowest_id, 0)

            if requester_priority > lowest_prio:
                slot.holders.pop(lowest_id, None)
                self._do_acquire(resource, requester_id)
                _log.info("Preempted %s from %s for %s (prio %d > %d)",
                          resource, lowest_id, requester_id,
                          requester_priority, lowest_prio)
                return lowest_id

            return None

    # -- Query --------------------------------------------------------------

    def is_available(self, resource: str) -> bool:
        with self._lock:
            slot = self._resources.get(resource)
            return slot.is_free if slot else False

    def check_availability(self, resources: List[str]) -> Dict[str, bool]:
        with self._lock:
            return {
                r: self._resources[r].is_free
                for r in resources if r in self._resources
            }

    def get_allocations(self) -> Dict[str, Any]:
        with self._lock:
            return {
                name: {
                    "max_slots": slot.max_slots,
                    "available": slot.available,
                    "exclusive": slot.exclusive,
                    "holders": dict(slot.holders),
                }
                for name, slot in self._resources.items()
            }

    def get_goal_resources(self, goal_id: str) -> List[str]:
        with self._lock:
            return [name for name, slot in self._resources.items()
                    if slot.held_by(goal_id)]

    def get_status(self) -> Dict[str, Any]:
        alloc = self.get_allocations()
        total_used = sum(
            len(info["holders"]) for info in alloc.values()
        )
        return {
            "resources": alloc,
            "total_used": total_used,
            "goal_priorities": dict(self._goal_priorities),
        }

    # -- Internal -----------------------------------------------------------

    def _can_acquire(self, resource: str, goal_id: str) -> bool:
        slot = self._resources.get(resource)
        if not slot:
            return False
        if slot.held_by(goal_id):
            return True
        return slot.is_free

    def _do_acquire(self, resource: str, goal_id: str) -> None:
        slot = self._resources.get(resource)
        if slot and not slot.held_by(goal_id):
            slot.holders[goal_id] = 1
            _log.debug("Acquired %s for %s", resource, goal_id)
