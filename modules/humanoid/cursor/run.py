"""Cursor run: plan with AI router, optional step execution, evidence. mode=auto = Cursor Super Mode."""
from __future__ import annotations

import time
import re
from typing import Any, Dict, List, Optional

from .status import set_last_run


_CAPABILITY_PAT = re.compile(
    r"\b(que\s+sabes\s+hacer|qué\s+sabes\s+hacer|que\s+puedes\s+hacer|qué\s+puedes\s+hacer|capabilit|habilidades|ayuda|help|comandos|manual)\b",
    re.IGNORECASE,
)


def _is_capabilities_query(goal: str) -> bool:
    g = (goal or "").strip().lower()
    if not g:
        return True
    if _CAPABILITY_PAT.search(g):
        return True
    # Preguntas muy cortas suelen ser exploratorias; respondemos sin LLM si coincide.
    if len(g) <= 22 and ("sabes" in g or "puedes" in g):
        return True
    return False


def _capabilities_steps() -> List[Dict[str, Any]]:
    return [
        {"description": "Diagnóstico rápido: /status, /health, ANS, métricas, scheduler, governance.", "status": "pending"},
        {"description": "Planificación tipo Cursor: generar pasos para un objetivo y ejecutar en modo controlado/auto.", "status": "pending"},
        {"description": "OPS (comunicación permanente): audio PC + Telegram (y WhatsApp si hay credenciales) con evidencia.", "status": "pending"},
        {"description": "Web (Pies digitales): abrir URLs, click/fill, extraer texto y screenshots (Playwright).", "status": "pending"},
        {"description": "Visión (Ojos): snapshot/stream del robot con enhance, zoom y foco digital.", "status": "pending"},
        {"description": "Manos (Input local): click/teclado/hotkeys con governance + rate-limit + OPS.", "status": "pending"},
        {"description": "Owner Console: aprobaciones, sesiones, Emergency Stop y cadena de integridad.", "status": "pending"},
    ]


def _fallback_plan_steps(goal: str, n_steps: int) -> List[Dict[str, Any]]:
    """Plan determinístico (sin IA) cuando no hay modelo disponible o hay timeouts."""
    g = (goal or "").strip()
    base = [
        f"Definir criterio de éxito y alcance para: «{g or 'objetivo'}».",
        "Verificar estado del sistema: /status y /health (y revisar ANS si hay incidentes).",
        "Recolectar evidencia (OPS + snapshot/cámara o web) antes de actuar.",
        "Ejecutar la acción mínima necesaria (controlado) y registrar resultados en OPS/bitácora.",
        "Validar: repetir /health, comprobar evidencia y cerrar incidentes.",
        "Si fue un cambio crítico: preparar rollback/plan de reversión y documentar.",
    ]
    base = base[: max(3, min(len(base), n_steps))]
    return [{"description": x, "status": "pending"} for x in base]


def _cursor_auto_execute(steps: List[Dict[str, Any]], goal: str) -> Dict[str, Any]:
    """Cursor Super Mode: ejecuta steps reales con bitácora (script+log)."""
    try:
        from modules.humanoid.governance.gates import decide
        d = decide("cursor_tool_exec")
        if d.blocked_by_emergency:
            return {"ok": False, "executed": [], "evidence": [], "error": "emergency_stop_block"}
        if d.needs_approval and not d.allow:
            return {"ok": False, "executed": [], "evidence": [], "error": "governed_requires_approval"}
    except Exception:
        pass
    try:
        from .executor import CursorExecutor
        ex = CursorExecutor()
        return ex.execute_steps(steps, goal=goal)
    except Exception as e:
        return {"ok": False, "executed": [], "evidence": [], "error": str(e)}


def _resources_to_depth(resources: Optional[str]) -> int:
    """Map UI resources (lite/pro/ultra) to plan depth. Default 2 (pro)."""
    if not resources:
        return 2
    r = (resources or "").strip().lower()
    if r == "lite":
        return 1
    if r == "ultra":
        return 3
    return 2  # pro


def cursor_run(
    goal: str,
    mode: str = "plan_only",
    depth: int = 1,
    context: Optional[Dict[str, Any]] = None,
    prefer_free: bool = True,
    allow_paid: bool = False,
    profile: str = "owner",
    resources: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Orchestrate: build plan (AI router), optionally execute steps, return summary + evidence.
    mode: plan_only | controlled | auto
    resources: lite | pro | ultra (maps to depth 1/2/3 for number of steps).
    """
    t0 = time.perf_counter()
    context = context or {}
    # Depth from explicit depth or from resources (resources take precedence for UI)
    effective_depth = _resources_to_depth(resources) if resources else max(1, min(5, depth))
    steps: List[Dict[str, Any]] = []
    summary = ""
    evidence: List[str] = []
    next_actions: List[str] = []
    approval_id: Optional[str] = None
    model_routing: Dict[str, Any] = {}
    error: Optional[str] = None
    # Cap plan generation so UI stays responsive (reduce perceived "freeze").
    # lite/pro/ultra map to 8/12/18s by default.
    plan_timeout_s = 12
    if resources:
        r = (resources or "").strip().lower()
        plan_timeout_s = 8 if r == "lite" else (18 if r == "ultra" else 12)

    try:
        # 0) Fast path: capacidades/ayuda sin IA (respuesta inmediata)
        if _is_capabilities_query(goal):
            steps = _capabilities_steps()
            summary = "Capacidades: diagnóstico, planificación tipo Cursor, OPS (audio/telegram/wa), web (pies), visión (ojos), manos, approvals y governance."
            next_actions = [
                "Prueba: escribe /status para diagnóstico inmediato",
                "Prueba Web: ve a tab Web (Pies) y abre una URL (con screenshot)",
                "Prueba OPS: abre OPS y verifica eventos + evidencia",
            ]
            ms = int((time.perf_counter() - t0) * 1000)
            data = {
                "goal": goal,
                "mode": mode,
                "depth": effective_depth,
                "resources": resources,
                "profile": profile,
                "steps": steps,
                "summary": summary,
                "evidence": [],
                "next_actions": next_actions,
                "model_routing": {"source": "built_in", "route": "FAST", "reason": "capabilities_fast_path", "latency_ms": ms},
                "approval_id": None,
                "ms": ms,
            }
            set_last_run(data)
            return {"ok": True, "data": data, "ms": ms, "error": None}

        # 1) Build plan using AI layer (FAST/REASON) or humanoid planner
        n_steps = min(7, max(3, 3 + effective_depth))
        # Prompt en ES por defecto (UI_LOCALE=es). Si se cambia locale, se puede adaptar luego.
        plan_prompt = (
            f"Divide en {n_steps} pasos concretos (acción verificable) para lograr: {goal}. "
            "Responde SOLO una lista numerada corta (1., 2., 3.) con un paso por línea. "
            "Incluye comandos cuando aplique (/status, /health, pytest)."
        )
        try:
            from modules.humanoid.ai.router import route_and_run
            out, decision, meta = route_and_run(
                plan_prompt,
                # Use "chat" for shallow plans to reduce latency; "reason" for deeper plans.
                intent_hint=("reason" if effective_depth >= 2 else "chat"),
                prefer_free=prefer_free,
                timeout_s=plan_timeout_s,
            )
            model_routing = {
                "provider_id": decision.provider_id,
                "model_key": decision.model_key,
                "route": decision.route,
                "reason": decision.reason,
                "latency_ms": meta.get("latency_ms", 0),
            }
            raw = (out or "").strip()

            # 1.a) Si no hay modelo (NullProvider) o salió un mensaje interno, usar plan determinístico limpio.
            if (decision.provider_id or "").strip().lower() == "null" or raw.startswith("[NullProvider]") or "System:" in raw:
                steps = _fallback_plan_steps(goal, n_steps=n_steps)
            else:
                for line in raw.split("\n"):
                    line = line.strip()
                    if not line:
                        continue
                    # Filtrar basura / telemetría que nunca debe mostrarse como paso
                    if line.startswith("[") and ("NullProvider" in line or "Último error" in line):
                        continue
                    if "System:" in line or "Route=" in line:
                        continue
                    if line[0].isdigit() or line.startswith("-") or line.startswith("*"):
                        steps.append({"description": line.lstrip("0123456789.-)* "), "status": "pending"})
                    else:
                        steps.append({"description": line, "status": "pending"})
                if not steps and raw:
                    steps = [{"description": raw[:200], "status": "pending"}]
        except Exception as e:
            try:
                from modules.humanoid import get_humanoid_kernel
                k = get_humanoid_kernel()
                autonomy = k.registry.get("autonomy")
                if autonomy and hasattr(autonomy, "planner"):
                    result = autonomy.planner.plan(goal, fast=True)
                    if result.get("ok") and result.get("steps"):
                        steps = [{"description": s.get("description", str(s)), "status": "pending"} for s in result["steps"]]
                        model_routing = {"source": "humanoid_planner", "reason": "fallback"}
            except Exception:
                pass
            if not steps:
                steps = _fallback_plan_steps(goal, n_steps=n_steps)
                error = str(e)

        summary = f"Plan: {len(steps)} pasos. Modo={mode}."
        auto_result: Dict[str, Any] = {}
        if mode == "plan_only":
            next_actions = ["Ejecuta un paso con POST /cursor/step/execute", "O usa modo=controlled/auto para ejecución"]
        elif mode == "auto":
            auto_result = _cursor_auto_execute(steps, goal)
            evidence = auto_result.get("evidence", [])
            ok_count = sum(1 for x in auto_result.get("executed", []) if x.get("ok"))
            needs = sum(1 for x in auto_result.get("executed", []) if x.get("action") == "needs_human")
            summary = f"Plan: {len(steps)} pasos. Auto ejecutado: {ok_count} OK. Pendiente humano: {needs}."
            next_actions = [
                "Si hay pasos en 'needs_human': cambia a modo=controlled para ejecutar paso a paso con aprobación.",
                f"Evidencia: script/log guardados ({len(evidence)} ítems).",
            ]
            if not auto_result.get("ok"):
                error = "auto_execute partial failure"
        else:
            next_actions = ["Modo controlled: usa /cursor/step/execute (approve=true) para ejecutar un paso y ver terminal."]

        ms = int((time.perf_counter() - t0) * 1000)
        data = {
            "goal": goal,
            "mode": mode,
            "depth": effective_depth,
            "resources": resources,
            "profile": profile,
            "steps": steps,
            "summary": summary,
            "evidence": evidence,
            "next_actions": next_actions,
            "model_routing": model_routing,
            "approval_id": approval_id,
            "ms": ms,
        }
        if mode == "auto":
            data["auto_result"] = auto_result
            if auto_result.get("terminal_preview"):
                data["terminal_preview"] = auto_result.get("terminal_preview")
            if auto_result.get("artifacts"):
                data["artifacts"] = auto_result.get("artifacts")
        set_last_run(data)
        return {"ok": error is None, "data": data, "ms": ms, "error": error}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        set_last_run({"goal": goal, "mode": mode, "error": str(e), "ms": ms})
        return {"ok": False, "data": None, "ms": ms, "error": str(e)}
