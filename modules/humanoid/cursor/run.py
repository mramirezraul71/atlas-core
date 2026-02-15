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
    """Cursor Super Mode: ejecuta steps sin approval. Crea módulos, edita código, ejecuta tools, navega pantalla."""
    executed: List[Dict[str, Any]] = []
    evidence: List[str] = []
    try:
        from modules.humanoid.governance.gates import decide
        d = decide("cursor_tool_exec")
        if d.blocked_by_emergency:
            return {"ok": False, "executed": [], "evidence": [], "error": "emergency_stop_block"}
        if d.needs_approval and not d.allow:
            return {"ok": False, "executed": [], "evidence": [], "error": "governed_requires_approval"}
    except Exception:
        pass
    for i, s in enumerate(steps):
        desc = (s.get("description") or "").lower()
        if not desc:
            continue
        r: Dict[str, Any] = {"step": i, "description": s.get("description"), "ok": False, "action": None}
        try:
            if "ver pantalla" in desc or "capture" in desc or "captura" in desc or "screenshot" in desc or "ver la pantalla" in desc:
                from modules.humanoid.nerve import eyes_capture
                cap = eyes_capture(use_nexus_if_available=True, source="screen")
                r["ok"] = cap.get("ok", False)
                r["action"] = "eyes_capture"
                if cap.get("evidence_path"):
                    evidence.append(cap["evidence_path"])
                elif cap.get("source"):
                    evidence.append(f"eyes_{cap['source']}")
            elif "crear módulo" in desc or "create module" in desc:
                from modules.humanoid.selfprog import create_module
                name = desc.split()[-1][:30] if desc.split() else "new_module"
                res = create_module(name)
                r["ok"] = res.get("ok", False)
                r["action"] = "create_module"
            elif "instalar" in desc or "install" in desc:
                from modules.humanoid.selfprog import install_dependency
                pkg = desc.replace("instalar", "").replace("install", "").strip().split()[0] if desc.split() else ""
                if pkg:
                    res = install_dependency(pkg)
                    r["ok"] = res.get("ok", False)
                    r["action"] = "install_dependency"
            elif "click" in desc or "clic" in desc:
                from modules.humanoid.nerve import hands_locate, hands_execute
                query = desc.split("en")[-1].strip()[:50] if "en" in desc else desc[:50]
                loc = hands_locate(query)
                matches = loc.get("matches", [])
                if matches and matches[0].get("bbox"):
                    bbox = matches[0]["bbox"]
                    cx = bbox[0] + bbox[2] // 2 if len(bbox) >= 3 else bbox[0]
                    cy = bbox[1] + bbox[3] // 2 if len(bbox) >= 4 else bbox[1]
                    res = hands_execute("click", {"x": cx, "y": cy}, verify_after=True)
                    r["ok"] = res.get("ok", False)
                    r["action"] = "click"
                    if res.get("evidence_after"):
                        evidence.append(res["evidence_after"])
            elif "subir repo" in desc or "push repo" in desc or "sube el repo" in desc or "push del repo" in desc or "subir repositorio" in desc or "push repositorio" in desc:
                from modules.repo_push import push_repo, resolve_path
                app_id = None
                for prefix in ("subir repo de ", "push repo de ", "sube el repo de ", "push del repo de ", "subir repositorio de ", "push repositorio de "):
                    if prefix in desc:
                        app_id = desc.split(prefix, 1)[-1].split()[0].strip()[:50]
                        break
                repo = resolve_path(app_id=app_id, repo_path=None)
                res = push_repo(repo_path=repo, message="chore: sync (solicitado por Cursor)")
                r["ok"] = res.get("ok", False)
                r["action"] = "repo_push"
                if res.get("message"):
                    evidence.append(res["message"])
            else:
                r["action"] = "skipped"
                r["ok"] = True
        except Exception as e:
            r["error"] = str(e)
        executed.append(r)
        s["status"] = "done" if r.get("ok") else "failed"
    ok_all = all(x.get("ok", False) for x in executed)
    return {"ok": ok_all, "executed": executed, "evidence": evidence}


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
        plan_prompt = f"Break down into {n_steps} concrete steps to achieve: {goal}. Output only a short numbered list, one step per line."
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
            summary = f"Plan: {len(steps)} pasos. Auto ejecutado: {sum(1 for x in auto_result.get('executed', []) if x.get('ok'))} OK."
            next_actions = [f"Auto: {len(evidence)} evidencia(s) guardada(s). Reporte: ok={auto_result.get('ok')}"]
            if not auto_result.get("ok"):
                error = "auto_execute partial failure"
        else:
            next_actions = ["Ejecución en controlled aún no implementada; usa plan_only o modo=auto"]

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
        set_last_run(data)
        return {"ok": error is None, "data": data, "ms": ms, "error": error}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        set_last_run({"goal": goal, "mode": mode, "error": str(e), "ms": ms})
        return {"ok": False, "data": None, "ms": ms, "error": str(e)}
