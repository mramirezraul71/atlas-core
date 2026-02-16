"""
POT Executor: Motor de ejecución de Procedimientos Operacionales.
"""
from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib import error, request

from .models import POT, POTStep, POTResult, StepResult, StepType

_log = logging.getLogger("humanoid.quality.executor")

REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent
REPORTS_DIR = REPO_ROOT / "modules" / "humanoid" / "quality" / "reports"
SNAPSHOTS_DIR = REPO_ROOT / "modules" / "humanoid" / "quality" / "snapshots"


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _ensure_dirs() -> None:
    REPORTS_DIR.mkdir(parents=True, exist_ok=True)
    SNAPSHOTS_DIR.mkdir(parents=True, exist_ok=True)


def _http_request(
    method: str,
    url: str,
    body: Optional[Dict[str, Any]] = None,
    timeout: int = 30,
) -> Dict[str, Any]:
    """Ejecuta request HTTP y retorna resultado estructurado."""
    headers = {"Accept": "application/json", "Content-Type": "application/json"}
    data = json.dumps(body).encode("utf-8") if body else None
    req = request.Request(url, data=data, method=method.upper(), headers=headers)
    
    try:
        with request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode("utf-8", "replace")
            try:
                response = json.loads(raw) if raw else {}
            except Exception:
                response = {"raw": raw[:500]}
            return {
                "ok": True,
                "status": int(r.status),
                "response": response,
                "body": raw,
            }
    except error.HTTPError as e:
        raw = e.read().decode("utf-8", "replace")
        return {
            "ok": False,
            "status": int(e.code),
            "response": {},
            "error": f"HTTP {e.code}",
            "body": raw[:500],
        }
    except Exception as e:
        return {
            "ok": False,
            "status": 0,
            "response": {},
            "error": f"{type(e).__name__}: {e}",
        }


def _run_command(
    command: str,
    timeout: int = 60,
    cwd: Optional[Path] = None,
) -> Dict[str, Any]:
    """Ejecuta comando shell y retorna resultado."""
    try:
        p = subprocess.run(
            command,
            shell=True,
            cwd=str(cwd or REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return {
            "ok": p.returncode == 0,
            "exit_code": p.returncode,
            "stdout": (p.stdout or "").strip(),
            "stderr": (p.stderr or "").strip(),
            "output": ((p.stdout or "") + "\n" + (p.stderr or "")).strip(),
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "exit_code": -1, "error": f"timeout {timeout}s", "output": ""}
    except Exception as e:
        return {"ok": False, "exit_code": -1, "error": str(e), "output": ""}


def _run_script(
    script_path: str,
    timeout: int = 120,
) -> Dict[str, Any]:
    """Ejecuta script Python y retorna resultado."""
    full_path = REPO_ROOT / script_path
    if not full_path.is_file():
        return {"ok": False, "error": f"script not found: {script_path}", "output": ""}
    
    try:
        p = subprocess.run(
            [sys.executable, str(full_path)],
            cwd=str(REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return {
            "ok": p.returncode == 0,
            "exit_code": p.returncode,
            "stdout": (p.stdout or "").strip(),
            "stderr": (p.stderr or "").strip(),
            "output": ((p.stdout or "") + "\n" + (p.stderr or "")).strip(),
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "exit_code": -1, "error": f"timeout {timeout}s", "output": ""}
    except Exception as e:
        return {"ok": False, "exit_code": -1, "error": str(e), "output": ""}


def _evaluate_condition(condition: str, context: Dict[str, Any]) -> bool:
    """Evalúa una condición Python en el contexto dado."""
    if not condition:
        return True
    try:
        return bool(eval(condition, {"__builtins__": {}}, context))
    except Exception as e:
        _log.warning("Condition eval failed: %s -> %s", condition, e)
        return True  # Por defecto ejecutar


def _evaluate_check(expression: str, context: Dict[str, Any]) -> bool:
    """Evalúa expresión de verificación."""
    if not expression:
        return True
    try:
        return bool(eval(expression, {"__builtins__": {}}, context))
    except Exception as e:
        _log.warning("Check eval failed: %s -> %s", expression, e)
        return False


def execute_step(
    step: POTStep,
    context: Dict[str, Any],
) -> StepResult:
    """
    Ejecuta un paso individual del POT.
    
    Args:
        step: El paso a ejecutar
        context: Contexto compartido entre pasos (puede modificarse)
    
    Returns:
        Resultado de la ejecución del paso
    """
    started_at = _now_iso()
    t0 = time.time()
    
    # Verificar condiciones
    if step.skip_if and _evaluate_condition(step.skip_if, context):
        return StepResult(
            step_id=step.id,
            step_name=step.name,
            ok=True,
            started_at=started_at,
            ended_at=_now_iso(),
            elapsed_ms=0,
            skipped=True,
            skip_reason=f"skip_if: {step.skip_if}",
        )
    
    if step.condition and not _evaluate_condition(step.condition, context):
        return StepResult(
            step_id=step.id,
            step_name=step.name,
            ok=True,
            started_at=started_at,
            ended_at=_now_iso(),
            elapsed_ms=0,
            skipped=True,
            skip_reason=f"condition not met: {step.condition}",
        )
    
    # Ejecutar según tipo
    result: Dict[str, Any] = {}
    retries_used = 0
    
    for attempt in range(step.retries + 1):
        retries_used = attempt
        
        if step.step_type == StepType.COMMAND:
            result = _run_command(step.command or "", timeout=step.timeout_seconds)
        
        elif step.step_type == StepType.SCRIPT:
            result = _run_script(step.script_path or "", timeout=step.timeout_seconds)
        
        elif step.step_type == StepType.HTTP:
            result = _http_request(
                method=step.http_method or "GET",
                url=step.http_url or "",
                body=step.http_body,
                timeout=step.timeout_seconds,
            )
            # Guardar response en contexto para checks
            context["response"] = result.get("response", {})
            context["response_status"] = result.get("status", 0)
            context["response_body"] = result.get("body", "")
        
        elif step.step_type == StepType.CHECK:
            check_ok = _evaluate_check(step.check_expression or "True", context)
            result = {"ok": check_ok, "output": f"check: {step.check_expression} = {check_ok}"}
        
        elif step.step_type == StepType.WAIT:
            time.sleep(step.wait_seconds or 1)
            result = {"ok": True, "output": f"waited {step.wait_seconds}s"}
        
        elif step.step_type == StepType.LOG:
            _log.info("[POT] %s: %s", step.name, step.description)
            result = {"ok": True, "output": "logged"}
        
        elif step.step_type == StepType.NOTIFY:
            # Intentar notificar via Telegram/OPS
            try:
                from modules.humanoid.comms.ops_bus import emit as ops_emit
                ops_emit("pot", step.notify_message or step.name, level="info")
                result = {"ok": True, "output": "notified"}
            except Exception as e:
                result = {"ok": False, "error": str(e), "output": ""}
        
        elif step.step_type == StepType.SNAPSHOT:
            # Capturar snapshot del estado actual
            snap_path = SNAPSHOTS_DIR / f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            try:
                snap_data = {"timestamp": _now_iso(), "context": {k: str(v)[:200] for k, v in context.items()}}
                snap_path.write_text(json.dumps(snap_data, indent=2), encoding="utf-8")
                result = {"ok": True, "output": str(snap_path)}
            except Exception as e:
                result = {"ok": False, "error": str(e), "output": ""}
        
        elif step.step_type == StepType.CONFIRM:
            # En modo automático, asumimos confirmación positiva
            result = {"ok": True, "output": "auto-confirmed (autonomous mode)"}
        
        elif step.step_type == StepType.ROLLBACK:
            # Los pasos de rollback se ejecutan igual que los normales
            result = {"ok": True, "output": "rollback step marker"}
        
        else:
            result = {"ok": True, "output": f"unknown step type: {step.step_type}"}
        
        # Si tuvo éxito, salir del loop de reintentos
        if result.get("ok"):
            break
        
        # Si hay reintentos pendientes, esperar
        if attempt < step.retries:
            _log.info("Step %s failed, retrying in %ds...", step.id, step.retry_delay_seconds)
            time.sleep(step.retry_delay_seconds)
    
    elapsed_ms = int((time.time() - t0) * 1000)
    ended_at = _now_iso()
    
    # Guardar output en contexto
    if step.capture_output:
        context[f"{step.id}_output"] = result.get("output", "")
        context[f"{step.id}_ok"] = result.get("ok", False)
        context[f"{step.id}_failed"] = not result.get("ok", False)
    
    return StepResult(
        step_id=step.id,
        step_name=step.name,
        ok=result.get("ok", False),
        started_at=started_at,
        ended_at=ended_at,
        elapsed_ms=elapsed_ms,
        output=result.get("output", result.get("stdout", ""))[:4000],
        error=result.get("error", result.get("stderr", "")),
        exit_code=result.get("exit_code"),
        retries_used=retries_used,
    )


def execute_pot(
    pot: POT,
    context: Optional[Dict[str, Any]] = None,
    dry_run: bool = False,
    stop_on_failure: bool = True,
    sync_to_cerebro: bool = True,
    notify_on_complete: bool = False,
) -> POTResult:
    """
    Ejecuta un POT completo.
    
    Args:
        pot: El POT a ejecutar
        context: Contexto inicial (ej: incident_id, ticket_id)
        dry_run: Si True, solo simula sin ejecutar
        stop_on_failure: Si True, detiene al primer fallo (a menos que continue_on_failure)
        sync_to_cerebro: Si True, registra la ejecución en ANS/Cerebro
        notify_on_complete: Si True, notifica a Telegram al completar
    
    Returns:
        Resultado completo de la ejecución
    """
    _ensure_dirs()
    
    ctx = dict(context or {})
    ctx["pot_id"] = pot.id
    ctx["pot_name"] = pot.name
    ctx["started_at"] = _now_iso()
    
    started_at = _now_iso()
    t0 = time.time()
    
    # Integración con Cerebro (ANS)
    execution_id = None
    bridge = None
    if sync_to_cerebro and not dry_run:
        try:
            from .cerebro_connector import get_bridge
            bridge = get_bridge()
            execution_id = bridge.on_pot_start(pot.id, pot.name, ctx)
            ctx["execution_id"] = execution_id
        except Exception as e:
            _log.debug("Could not connect to cerebro: %s", e)
    
    _log.info("Executing POT: %s (%s)", pot.id, pot.name)
    
    step_results: List[StepResult] = []
    failed = False
    
    for step in pot.steps:
        if failed and stop_on_failure and not step.continue_on_failure:
            # Saltar pasos restantes
            step_results.append(StepResult(
                step_id=step.id,
                step_name=step.name,
                ok=False,
                started_at=_now_iso(),
                ended_at=_now_iso(),
                elapsed_ms=0,
                skipped=True,
                skip_reason="previous step failed",
            ))
            continue
        
        if dry_run:
            _log.info("[DRY-RUN] Would execute step: %s", step.name)
            step_results.append(StepResult(
                step_id=step.id,
                step_name=step.name,
                ok=True,
                started_at=_now_iso(),
                ended_at=_now_iso(),
                elapsed_ms=0,
                output="dry-run",
            ))
            continue
        
        _log.info("Executing step: %s - %s", step.id, step.name)
        sr = execute_step(step, ctx)
        step_results.append(sr)
        
        if not sr.ok and not sr.skipped:
            _log.warning("Step %s failed: %s", step.id, sr.error or sr.output[:100])
            if not step.continue_on_failure:
                failed = True
    
    # Calcular resumen
    elapsed_ms = int((time.time() - t0) * 1000)
    ended_at = _now_iso()
    
    steps_ok = sum(1 for r in step_results if r.ok and not r.skipped)
    steps_failed = sum(1 for r in step_results if not r.ok and not r.skipped)
    steps_skipped = sum(1 for r in step_results if r.skipped)
    
    overall_ok = steps_failed == 0
    
    # Ejecutar rollback si falló y hay pasos de rollback
    rollback_executed = False
    rollback_ok = False
    
    if failed and pot.has_rollback and pot.rollback_steps:
        _log.info("Executing rollback for POT: %s", pot.id)
        rollback_executed = True
        rollback_results: List[StepResult] = []
        
        for rs in pot.rollback_steps:
            rsr = execute_step(rs, ctx)
            rollback_results.append(rsr)
        
        rollback_ok = all(r.ok for r in rollback_results)
        # Agregar resultados de rollback a step_results
        step_results.extend(rollback_results)
    
    # Crear resultado
    result = POTResult(
        pot_id=pot.id,
        pot_name=pot.name,
        ok=overall_ok,
        started_at=started_at,
        ended_at=ended_at,
        elapsed_ms=elapsed_ms,
        step_results=step_results,
        steps_total=len(pot.steps),
        steps_ok=steps_ok,
        steps_failed=steps_failed,
        steps_skipped=steps_skipped,
        context=ctx,
        incident_id=ctx.get("incident_id"),
        ticket_id=ctx.get("ticket_id"),
        rollback_executed=rollback_executed,
        rollback_ok=rollback_ok,
    )
    
    # Guardar reporte
    report_path = REPORTS_DIR / f"pot_report_{pot.id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    try:
        report_path.write_text(json.dumps(result.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")
        result.report_path = str(report_path)
        _log.info("POT report saved: %s", report_path)
    except Exception as e:
        _log.warning("Failed to save POT report: %s", e)
    
    _log.info(
        "POT %s completed: ok=%s, steps=%d/%d ok, elapsed=%dms",
        pot.id, overall_ok, steps_ok, len(pot.steps), elapsed_ms
    )
    
    # Notificar finalización al Cerebro
    if bridge and execution_id and not dry_run:
        try:
            bridge.on_pot_complete(
                pot_id=pot.id,
                execution_id=execution_id,
                ok=overall_ok,
                steps_ok=steps_ok,
                steps_total=len(pot.steps),
                duration_seconds=elapsed_ms / 1000.0,
                notify_telegram=notify_on_complete or (not overall_ok and pot.severity.value in ("high", "critical")),
            )
        except Exception as e:
            _log.debug("Could not notify cerebro completion: %s", e)
    
    return result
