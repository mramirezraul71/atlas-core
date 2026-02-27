"""
ParallelExecutor: Ejecutor paralelo de pasos de plan para goals concurrentes.

Utiliza ThreadPoolExecutor para ejecutar pasos de multiples goals en paralelo,
respetando las asignaciones del ResourceArbiter y con timeout por paso.
"""
from __future__ import annotations

import logging
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor, Future
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Set

_log = logging.getLogger("humanoid.cortex.frontal.parallel_executor")

_MAX_WORKERS = int(os.getenv("CGE_MAX_WORKERS", "6"))
_STEP_TIMEOUT = int(os.getenv("CGE_STEP_TIMEOUT", "20"))


@dataclass
class StepResult:
    """Resultado de la ejecucion de un paso."""
    goal_id: str
    step_index: int
    action: str
    ok: bool
    data: Any = None
    error: Optional[str] = None
    ms: int = 0


@dataclass
class GoalStep:
    """Paso listo para ejecutar en paralelo."""
    goal_id: str
    step_index: int
    action_type: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    resources: List[str] = field(default_factory=list)
    timeout_s: float = _STEP_TIMEOUT


# Mapeo de action_type a recurso primario requerido
ACTION_RESOURCE_MAP: Dict[str, List[str]] = {
    "navigate": ["hands"],
    "grasp": ["hands"],
    "place": ["hands"],
    "look": ["vision"],
    "speak": ["voice"],
    "wait": [],
    "execute": ["compute"],
    "analyze": ["llm", "compute"],
    "web_search": ["web"],
    "web_scrape": ["web"],
    "llm_query": ["llm"],
}


def infer_resources(action_type: str) -> List[str]:
    """Infiere recursos necesarios a partir del tipo de accion."""
    return ACTION_RESOURCE_MAP.get(action_type, ["compute"])


class ParallelExecutor:
    """
    Ejecuta pasos de multiples goals en paralelo via ThreadPoolExecutor.

    - Cada goal puede tener a lo sumo 1 paso en ejecucion simultanea
    - Timeout configurable por paso
    - Callbacks de progreso
    """

    def __init__(self, max_workers: int = _MAX_WORKERS) -> None:
        self._pool = ThreadPoolExecutor(
            max_workers=max_workers,
            thread_name_prefix="cge_exec",
        )
        self._lock = threading.Lock()
        self._running: Dict[str, Future] = {}
        self._cancelled: Set[str] = set()
        self._on_step_done: List[Callable[[StepResult], None]] = []

    def on_step_done(self, callback: Callable[[StepResult], None]) -> None:
        self._on_step_done.append(callback)

    @property
    def running_goals(self) -> List[str]:
        with self._lock:
            return [gid for gid, fut in self._running.items() if not fut.done()]

    def is_running(self, goal_id: str) -> bool:
        with self._lock:
            fut = self._running.get(goal_id)
            return fut is not None and not fut.done()

    def execute_step(self, step: GoalStep) -> Optional[Future]:
        """
        Lanza la ejecucion de un paso en background.
        Retorna Future o None si el goal ya tiene un paso corriendo.
        """
        with self._lock:
            if step.goal_id in self._cancelled:
                return None
            existing = self._running.get(step.goal_id)
            if existing and not existing.done():
                _log.debug("Goal %s ya tiene paso en ejecucion, skip", step.goal_id)
                return None

            fut = self._pool.submit(self._run_step, step)
            self._running[step.goal_id] = fut
            return fut

    def execute_batch(self, steps: List[GoalStep]) -> int:
        """Lanza multiples pasos. Retorna cuantos se lanzaron."""
        launched = 0
        for step in steps:
            if self.execute_step(step) is not None:
                launched += 1
        return launched

    def cancel(self, goal_id: str) -> bool:
        """Marca goal como cancelado. El paso actual terminara pero no se lanzaran mas."""
        with self._lock:
            self._cancelled.add(goal_id)
            fut = self._running.get(goal_id)
            if fut and not fut.done():
                fut.cancel()
                return True
            return False

    def collect_results(self) -> List[StepResult]:
        """Recolecta resultados de pasos terminados. No bloquea."""
        results = []
        with self._lock:
            done_goals = [
                gid for gid, fut in self._running.items() if fut.done()
            ]
        for gid in done_goals:
            with self._lock:
                fut = self._running.pop(gid, None)
            if fut is None:
                continue
            try:
                result = fut.result(timeout=0.1)
                if isinstance(result, StepResult):
                    results.append(result)
            except Exception as exc:
                results.append(StepResult(
                    goal_id=gid, step_index=-1, action="unknown",
                    ok=False, error=str(exc),
                ))
        for r in results:
            for cb in self._on_step_done:
                try:
                    cb(r)
                except Exception:
                    pass
        return results

    def shutdown(self) -> None:
        self._pool.shutdown(wait=False)

    # -- Ejecucion interna --------------------------------------------------

    def _run_step(self, step: GoalStep) -> StepResult:
        """Ejecuta un paso invocando el dispatcher apropiado."""
        t0 = time.perf_counter()
        action = step.action_type
        params = step.parameters

        try:
            result = self._dispatch(action, params, step.timeout_s)
            ms = int((time.perf_counter() - t0) * 1000)
            ok = result.get("ok", False) if isinstance(result, dict) else bool(result)
            data = result.get("data") if isinstance(result, dict) else result
            error = result.get("error") if isinstance(result, dict) else None

            return StepResult(
                goal_id=step.goal_id,
                step_index=step.step_index,
                action=action,
                ok=ok,
                data=data,
                error=error,
                ms=ms,
            )
        except Exception as exc:
            ms = int((time.perf_counter() - t0) * 1000)
            _log.error("Step %s/%d failed: %s", step.goal_id, step.step_index, exc)
            return StepResult(
                goal_id=step.goal_id,
                step_index=step.step_index,
                action=action,
                ok=False,
                error=str(exc),
                ms=ms,
            )

    @staticmethod
    def _dispatch(action: str, params: Dict[str, Any],
                  timeout_s: float) -> Dict[str, Any]:
        """Despacha al dispatcher unificado segun tipo de accion."""
        try:
            from modules.humanoid.dispatch import (
                run_hands, run_web, run_vision, run_voice,
            )
        except ImportError:
            return {"ok": False, "error": "dispatch module not available"}

        timeout_int = int(timeout_s)

        if action in ("navigate", "grasp", "place", "execute"):
            command = params.get("command") or params.get("target") or params.get("goal", "")
            return run_hands(str(command), timeout_sec=timeout_int)

        if action in ("look", "analyze_image"):
            image_path = params.get("image_path") or params.get("target", "")
            return run_vision(str(image_path))

        if action in ("speak",):
            text = params.get("text") or params.get("message", "")
            return run_voice(action="speak", text=str(text))

        if action in ("web_search", "web_scrape"):
            url = params.get("url") or params.get("action", "")
            return run_web(str(url), timeout_sec=timeout_int)

        if action in ("wait",):
            wait_s = float(params.get("duration_s", 1.0))
            time.sleep(min(wait_s, timeout_s))
            return {"ok": True, "data": {"waited_s": wait_s}}

        if action in ("llm_query", "analyze"):
            command = params.get("query") or params.get("goal", str(params))
            return run_hands(f"echo LLM stub: {command}", timeout_sec=timeout_int)

        return run_hands(str(params.get("command", str(params))), timeout_sec=timeout_int)
