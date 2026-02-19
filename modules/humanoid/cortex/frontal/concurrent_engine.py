"""
ConcurrentGoalEngine (CGE): Motor de gestion de metas concurrente.

Orquesta multiples goals en paralelo usando GoalContext (persistencia),
ResourceArbiter (arbitraje de recursos), ParallelExecutor (ejecucion)
y los planificadores existentes (TaskPlanner, DecisionMaker).

Invocado via tick() desde el scheduler cada CGE_TICK_SECONDS.
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import threading
import time
from typing import Any, Dict, List, Optional

from .goal_context import (
    CGEGoalPriority, CGEGoalStatus, GoalContext, GoalContextDB, new_goal_id,
)
from .resource_arbiter import ResourceArbiter
from .parallel_executor import (
    GoalStep, ParallelExecutor, StepResult, infer_resources,
)

_log = logging.getLogger("humanoid.cortex.frontal.concurrent_engine")

_MAX_CONCURRENT = int(os.getenv("CGE_MAX_CONCURRENT", "4"))
_TICK_SECONDS = float(os.getenv("CGE_TICK_SECONDS", "2"))
_STEP_TIMEOUT = int(os.getenv("CGE_STEP_TIMEOUT", "20"))


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="cge")
    except Exception:
        pass


def _ops_event(msg: str, level: str = "low") -> None:
    try:
        from modules.humanoid.comms.ops_bus import emit
        emit(msg, level=level, subsystem="cge")
    except Exception:
        pass


def _lifelog_action(action: str, params: Dict = None,
                    outcome: str = "", success: bool = True) -> None:
    """Registra accion del CGE en el Lifelog de ATLAS."""
    try:
        from modules.humanoid.memory_engine.lifelog import get_lifelog
        ll = get_lifelog()
        ll.log_action(
            source="concurrent_engine",
            action=action,
            params=params or {},
            outcome=outcome,
            success=success,
        )
    except Exception:
        pass


def _governance_check(action_kind: str, context: Dict = None) -> bool:
    """Consulta Governance antes de ejecutar acciones criticas. Retorna True si permitido."""
    try:
        from modules.humanoid.governance.gates import decide
        decision = decide(action_kind, context=context)
        return getattr(decision, "allow", True)
    except Exception:
        return True


def _world_model_update(goal_id: str, status: str, goal_type: str = "goal",
                        extra: Dict = None) -> None:
    """Actualiza el World Model con el estado del goal."""
    try:
        from modules.humanoid.world_model.engine import get_world_model
        wm = get_world_model()
        state = {"status": status}
        if extra:
            state.update(extra)
        wm.upsert_entity(
            name=goal_id,
            entity_type=goal_type,
            state=state,
            properties={"managed_by": "cge"},
        )
    except Exception:
        pass


def _world_model_record_outcome(action: str, context: Dict,
                                success: bool, ms: float = 0) -> None:
    """Registra resultado de accion en el World Model para aprendizaje."""
    try:
        from modules.humanoid.world_model.engine import get_world_model
        wm = get_world_model()
        wm.record_outcome(
            action_type=action,
            context=context,
            predicted="success" if success else "failure",
            actual="success" if success else "failure",
            success=success,
            duration_ms=ms,
        )
    except Exception:
        pass


# ---- Singleton ------------------------------------------------------------

_ENGINE: Optional[ConcurrentGoalEngine] = None
_ENGINE_LOCK = threading.Lock()


def get_engine() -> ConcurrentGoalEngine:
    """Obtiene o crea la instancia singleton del CGE."""
    global _ENGINE
    if _ENGINE is None:
        with _ENGINE_LOCK:
            if _ENGINE is None:
                _ENGINE = ConcurrentGoalEngine()
    return _ENGINE


# ---- Engine ---------------------------------------------------------------


class ConcurrentGoalEngine:
    """
    Motor de metas concurrente.

    Ciclo tick():
      1. Recolectar resultados de pasos terminados
      2. Actualizar progreso de goals
      3. Planificar goals sin plan
      4. Despachar pasos de goals activos que tengan recursos disponibles
    """

    def __init__(
        self,
        max_concurrent: int = _MAX_CONCURRENT,
        db: Optional[GoalContextDB] = None,
        arbiter: Optional[ResourceArbiter] = None,
        executor: Optional[ParallelExecutor] = None,
    ) -> None:
        self.max_concurrent = max_concurrent
        self.db = db or GoalContextDB()
        self.arbiter = arbiter or ResourceArbiter()
        self.executor = executor or ParallelExecutor()

        self._lock = threading.Lock()
        self._active_goals: Dict[str, GoalContext] = {}
        self._tick_count = 0
        self._started = False

        self.executor.on_step_done(self._process_step_result)
        self._restore_from_db()

    # -- Public API ---------------------------------------------------------

    def submit_goal(
        self,
        goal_type: str,
        description: str,
        priority: int = 1,
        parameters: Optional[Dict[str, Any]] = None,
        source: str = "user",
        parent_goal_id: Optional[str] = None,
        deadline_ts: Optional[float] = None,
        resources_needed: Optional[List[str]] = None,
    ) -> str:
        """Registra un nuevo goal. Retorna goal_id."""
        gid = new_goal_id()
        ctx = GoalContext(
            goal_id=gid,
            goal_type=goal_type,
            description=description,
            priority=CGEGoalPriority(min(priority, 4)),
            status=CGEGoalStatus.PENDING,
            context_data=parameters or {},
            source=source,
            parent_goal_id=parent_goal_id,
            deadline_ts=deadline_ts,
            resources_needed=resources_needed or [],
        )

        if not _governance_check("cge_submit_goal", {"goal_type": goal_type, "priority": priority}):
            _log.warning("Governance blocked goal submission: %s", description)
            _ops_event(f"Governance bloqueó goal: {description}", level="med")
            return ""

        self.db.save(ctx)
        with self._lock:
            self._active_goals[gid] = ctx

        self.arbiter.register_goal_priority(gid, priority)

        _log.info("Goal submitted: %s — %s (prio %d)", gid, description, priority)
        _bitacora(f"CGE goal enviado: {description} prio={priority}")
        _ops_event(f"Nuevo goal CGE: {description}", level="low")
        _lifelog_action("submit_goal", {"goal_id": gid, "type": goal_type, "priority": priority},
                        outcome=description, success=True)
        _world_model_update(gid, "pending", extra={"description": description, "priority": priority})

        return gid

    def pause_goal(self, goal_id: str) -> bool:
        with self._lock:
            ctx = self._active_goals.get(goal_id)
        if not ctx or ctx.is_terminal():
            return False
        ctx.status = CGEGoalStatus.PAUSED
        self.arbiter.release_all(goal_id)
        ctx.resources_held = []
        self.db.save(ctx)
        _log.info("Goal paused: %s", goal_id)
        return True

    def resume_goal(self, goal_id: str) -> bool:
        with self._lock:
            ctx = self._active_goals.get(goal_id)
        if not ctx:
            ctx = self.db.load(goal_id)
            if ctx:
                with self._lock:
                    self._active_goals[goal_id] = ctx
        if not ctx or ctx.is_terminal():
            return False
        ctx.status = CGEGoalStatus.ACTIVE
        self.db.save(ctx)
        _log.info("Goal resumed: %s", goal_id)
        return True

    def cancel_goal(self, goal_id: str, reason: str = "cancelled") -> bool:
        with self._lock:
            ctx = self._active_goals.get(goal_id)
        if not ctx:
            return False
        if ctx.is_terminal():
            return False

        ctx.status = CGEGoalStatus.CANCELLED
        ctx.error = reason
        self.executor.cancel(goal_id)
        released = self.arbiter.release_all(goal_id)
        ctx.resources_held = []
        self.db.save(ctx)

        with self._lock:
            self._active_goals.pop(goal_id, None)
        self.arbiter.unregister_goal(goal_id)

        _log.info("Goal cancelled: %s — %s (released: %s)", goal_id, reason, released)
        _bitacora(f"CGE goal cancelado: {ctx.description}")
        return True

    def get_goal_detail(self, goal_id: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            ctx = self._active_goals.get(goal_id)
        if not ctx:
            ctx = self.db.load(goal_id)
        if not ctx:
            return None

        detail = ctx.to_dict()
        detail["exec_log"] = self.db.get_exec_log(goal_id, limit=20)
        detail["held_resources"] = self.arbiter.get_goal_resources(goal_id)
        return detail

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            goals = list(self._active_goals.values())

        by_status: Dict[str, int] = {}
        for g in goals:
            by_status[g.status.value] = by_status.get(g.status.value, 0) + 1

        return {
            "engine": "ConcurrentGoalEngine",
            "tick_count": self._tick_count,
            "max_concurrent": self.max_concurrent,
            "active_goals": len([g for g in goals if g.is_runnable()]),
            "total_tracked": len(goals),
            "by_status": by_status,
            "goals": [g.to_dict() for g in goals],
            "resources": self.arbiter.get_status(),
            "executor_running": self.executor.running_goals,
        }

    # -- Tick (ciclo principal) ---------------------------------------------

    def tick(self) -> Dict[str, Any]:
        """
        Ciclo principal del motor concurrente. Invocado periodicamente.

        1. Recolectar resultados de pasos terminados
        2. Actualizar estado de goals
        3. Planificar goals que no tienen plan
        4. Despachar pasos de goals activos con recursos disponibles
        """
        t0 = time.perf_counter()
        self._tick_count += 1

        results_collected = self._phase_collect_results()
        goals_planned = self._phase_plan_goals()
        steps_dispatched = self._phase_dispatch_steps()
        self._phase_check_overdue()

        ms = int((time.perf_counter() - t0) * 1000)

        return {
            "ok": True,
            "tick": self._tick_count,
            "results_collected": results_collected,
            "goals_planned": goals_planned,
            "steps_dispatched": steps_dispatched,
            "ms": ms,
        }

    # -- Fases del tick -----------------------------------------------------

    def _phase_collect_results(self) -> int:
        """Fase 1: Recolectar resultados de pasos terminados."""
        results = self.executor.collect_results()
        for r in results:
            self._process_step_result(r)
        return len(results)

    def _phase_plan_goals(self) -> int:
        """Fase 2: Planificar goals pendientes que no tienen plan."""
        planned = 0
        with self._lock:
            pending = [
                g for g in self._active_goals.values()
                if g.status == CGEGoalStatus.PENDING and g.plan_json is None
            ]

        for ctx in pending[:self.max_concurrent]:
            if self._plan_goal(ctx):
                planned += 1
        return planned

    def _phase_dispatch_steps(self) -> int:
        """Fase 3: Despachar pasos de goals activos."""
        dispatched = 0
        with self._lock:
            active = [
                g for g in self._active_goals.values()
                if g.status == CGEGoalStatus.ACTIVE
                and not self.executor.is_running(g.goal_id)
            ]
            active.sort(key=lambda g: -g.priority.value)

        running_count = len(self.executor.running_goals)

        for ctx in active:
            if running_count >= self.max_concurrent:
                break

            step = self._next_step_for_goal(ctx)
            if step is None:
                continue

            resources = step.resources or infer_resources(step.action_type)
            if not self.arbiter.acquire(ctx.goal_id, resources, ctx.priority.value):
                if ctx.priority.value >= CGEGoalPriority.URGENT.value:
                    for r in resources:
                        displaced = self.arbiter.preempt(ctx.goal_id, r, ctx.priority.value)
                        if displaced:
                            self._pause_displaced_goal(displaced, r)
                    if not self.arbiter.acquire(ctx.goal_id, resources, ctx.priority.value):
                        ctx.status = CGEGoalStatus.WAITING_RESOURCE
                        self.db.save(ctx)
                        continue
                else:
                    ctx.status = CGEGoalStatus.WAITING_RESOURCE
                    self.db.save(ctx)
                    continue

            ctx.resources_held = self.arbiter.get_goal_resources(ctx.goal_id)
            for r in resources:
                self.db.log_resource_alloc(r, ctx.goal_id, ctx.priority.value)

            if not _governance_check("cge_execute_step",
                                     {"goal_id": ctx.goal_id, "action": step.action_type,
                                      "priority": ctx.priority.value}):
                _log.info("Governance bloqueó paso %d de %s", step.step_index, ctx.goal_id)
                self.arbiter.release(ctx.goal_id, resources)
                continue

            fut = self.executor.execute_step(step)
            if fut:
                dispatched += 1
                running_count += 1
                _log.debug("Dispatched step %d for %s (%s)",
                           step.step_index, ctx.goal_id, step.action_type)

        return dispatched

    def _phase_check_overdue(self) -> None:
        """Fase 4: Verificar deadlines vencidos."""
        with self._lock:
            goals = list(self._active_goals.values())
        for ctx in goals:
            if ctx.is_overdue() and not ctx.is_terminal():
                ctx.status = CGEGoalStatus.FAILED
                ctx.error = "deadline exceeded"
                self.executor.cancel(ctx.goal_id)
                self.arbiter.release_all(ctx.goal_id)
                ctx.resources_held = []
                self.db.save(ctx)
                _ops_event(f"Goal deadline excedido: {ctx.description}", level="med")

    # -- Planificacion ------------------------------------------------------

    def _plan_goal(self, ctx: GoalContext) -> bool:
        """Genera plan para un goal usando TaskPlanner (sync wrapper)."""
        ctx.status = CGEGoalStatus.PLANNING
        self.db.save(ctx)

        try:
            from .task_planner import TaskPlanner
            planner = TaskPlanner()

            try:
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    import concurrent.futures
                    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                        plan = pool.submit(
                            asyncio.run,
                            planner.plan(ctx.description, None, ctx.context_data)
                        ).result(timeout=15)
                else:
                    plan = loop.run_until_complete(
                        planner.plan(ctx.description, None, ctx.context_data)
                    )
            except RuntimeError:
                plan = asyncio.run(
                    planner.plan(ctx.description, None, ctx.context_data)
                )

            if plan and plan.is_valid():
                ctx.plan_json = json.dumps(plan.to_dict(), default=str)
                ctx.status = CGEGoalStatus.ACTIVE
                ctx.step_index = 0
                self.db.save(ctx)
                _log.info("Goal %s planned: %d steps", ctx.goal_id, len(plan.steps))
                _bitacora(f"CGE plan generado: {ctx.description} ({len(plan.steps)} pasos)")
                return True

            ctx.status = CGEGoalStatus.ACTIVE
            fallback = {
                "id": f"plan_fallback_{ctx.goal_id}",
                "goal": ctx.description,
                "steps": [{
                    "id": f"step_0_{ctx.goal_id}",
                    "description": f"Ejecutar: {ctx.description}",
                    "action_type": "execute",
                    "parameters": ctx.context_data,
                    "status": "pending",
                }],
                "status": "active",
                "progress": 0.0,
            }
            ctx.plan_json = json.dumps(fallback, default=str)
            self.db.save(ctx)
            return True

        except Exception as exc:
            _log.error("Planning failed for %s: %s", ctx.goal_id, exc)
            ctx.status = CGEGoalStatus.FAILED
            ctx.error = f"planning_failed: {exc}"
            self.db.save(ctx)
            _bitacora(f"CGE planificacion falló: {ctx.description}", ok=False)
            return False

    # -- Step dispatch helpers ----------------------------------------------

    def _next_step_for_goal(self, ctx: GoalContext) -> Optional[GoalStep]:
        """Obtiene el siguiente paso ejecutable del plan del goal."""
        if not ctx.plan_json:
            return None
        try:
            plan = json.loads(ctx.plan_json)
        except Exception:
            return None

        steps = plan.get("steps", [])
        idx = ctx.step_index
        if idx >= len(steps):
            self._complete_goal(ctx)
            return None

        step_data = steps[idx]
        if step_data.get("status") in ("completed", "failed", "skipped"):
            ctx.step_index = idx + 1
            self.db.update_progress(
                ctx.goal_id, ctx.step_index,
                ctx.step_index / max(len(steps), 1),
            )
            return self._next_step_for_goal(ctx)

        return GoalStep(
            goal_id=ctx.goal_id,
            step_index=idx,
            action_type=step_data.get("action_type", "execute"),
            parameters=step_data.get("parameters", {}),
            resources=infer_resources(step_data.get("action_type", "execute")),
            timeout_s=_STEP_TIMEOUT,
        )

    def _process_step_result(self, result: StepResult) -> None:
        """Procesa el resultado de un paso completado."""
        with self._lock:
            ctx = self._active_goals.get(result.goal_id)
        if not ctx:
            return

        self.db.log_step(
            result.goal_id, result.step_index, result.action,
            result.data, result.ok, result.ms,
        )

        self.arbiter.release_all(result.goal_id)
        for r in ctx.resources_held:
            self.db.log_resource_release(r, result.goal_id)
        ctx.resources_held = []

        _world_model_record_outcome(
            result.action,
            {"goal_id": result.goal_id, "step": result.step_index},
            success=result.ok, ms=result.ms,
        )

        if not result.ok:
            _log.warning("Step %d failed for %s: %s",
                         result.step_index, result.goal_id, result.error)
            ctx.status = CGEGoalStatus.FAILED
            ctx.error = result.error
            self.db.save(ctx)
            with self._lock:
                self._active_goals.pop(result.goal_id, None)
            self.arbiter.unregister_goal(result.goal_id)
            _bitacora(f"CGE step falló: {ctx.description} paso {result.step_index}", ok=False)
            _ops_event(f"CGE goal falló: {ctx.description}", level="med")
            _lifelog_action("goal_failed", {"goal_id": result.goal_id, "step": result.step_index},
                            outcome=result.error or "step failed", success=False)
            _world_model_update(result.goal_id, "failed", extra={"error": result.error})
            return

        try:
            plan = json.loads(ctx.plan_json) if ctx.plan_json else {}
        except Exception:
            plan = {}
        steps = plan.get("steps", [])
        if result.step_index < len(steps):
            steps[result.step_index]["status"] = "completed"
            ctx.plan_json = json.dumps(plan, default=str)

        ctx.step_index = result.step_index + 1
        ctx.progress = ctx.step_index / max(len(steps), 1)
        ctx.status = CGEGoalStatus.ACTIVE

        if ctx.step_index >= len(steps):
            self._complete_goal(ctx)
        else:
            self.db.save(ctx)

    def _complete_goal(self, ctx: GoalContext) -> None:
        """Marca un goal como completado."""
        ctx.status = CGEGoalStatus.COMPLETED
        ctx.progress = 1.0
        self.db.save(ctx)
        with self._lock:
            self._active_goals.pop(ctx.goal_id, None)
        self.arbiter.release_all(ctx.goal_id)
        self.arbiter.unregister_goal(ctx.goal_id)

        _log.info("Goal completed: %s — %s", ctx.goal_id, ctx.description)
        _bitacora(f"CGE goal completado: {ctx.description}")
        _ops_event(f"Goal completado: {ctx.description}", level="low")
        _lifelog_action("goal_completed",
                        {"goal_id": ctx.goal_id, "type": ctx.goal_type,
                         "elapsed_s": round(ctx.elapsed_s(), 1)},
                        outcome=ctx.description, success=True)
        _world_model_update(ctx.goal_id, "completed",
                            extra={"elapsed_s": round(ctx.elapsed_s(), 1)})

    def _pause_displaced_goal(self, goal_id: str, resource: str) -> None:
        """Pausa un goal que fue desplazado por preemption."""
        with self._lock:
            ctx = self._active_goals.get(goal_id)
        if not ctx:
            return
        ctx.status = CGEGoalStatus.WAITING_RESOURCE
        ctx.resources_held = self.arbiter.get_goal_resources(goal_id)
        self.db.save(ctx)
        _log.info("Goal %s desplazado del recurso %s por preemption", goal_id, resource)

    # -- Restore ------------------------------------------------------------

    def _restore_from_db(self) -> None:
        """Restaura goals activos desde SQLite al iniciar."""
        try:
            active = self.db.list_active()
            with self._lock:
                for ctx in active:
                    self._active_goals[ctx.goal_id] = ctx
                    self.arbiter.register_goal_priority(
                        ctx.goal_id, ctx.priority.value
                    )
            if active:
                _log.info("Restored %d active goals from DB", len(active))
        except Exception as exc:
            _log.error("Restore from DB failed: %s", exc)


# ---- Convenience function for scheduler -----------------------------------

def cge_tick() -> Dict[str, Any]:
    """Punto de entrada para el scheduler."""
    try:
        engine = get_engine()
        return engine.tick()
    except Exception as exc:
        _log.error("CGE tick failed: %s", exc)
        return {"ok": False, "error": str(exc)}
