"""Job runner: update_check, shell_command, llm_plan, custom. Timeout 20s, Policy + Audit."""
from __future__ import annotations

import hashlib
import json
import logging
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

RUNNER_TIMEOUT_SEC = 20
_scheduler_actor = "scheduler"
_APPROVAL_DIGEST_LAST_KEY = ""

_log = logging.getLogger("humanoid.scheduler.runner")


def _append_scheduler_trace(event: str, payload: Dict[str, Any]) -> None:
    try:
        base = Path(os.getenv("ATLAS_BASE", Path(__file__).resolve().parents[3]))
        trace_dir = base / "state"
        trace_dir.mkdir(parents=True, exist_ok=True)
        trace_path = trace_dir / "atlas_scheduler_trace.jsonl"
        row = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "source": "runner",
            "event": event,
            **(payload or {}),
        }
        with trace_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(row, ensure_ascii=False) + "\n")
    except Exception:
        pass


def _normalize_scheduler_result(kind: str, out: Dict[str, Any]) -> tuple[Dict[str, Any], bool]:
    """
    Distingue entre fallo de ejecución del job y resultado degradado del dominio.

    Para `nervous_cycle`, un score bajo significa degradación operacional del
    sistema nervioso, no un crash del scheduler. El ciclo completó, generó
    snapshot y debe quedar auditado sin entrar en espiral infinita de retries.
    """
    normalized = dict(out or {})
    ok = bool(normalized.get("ok", False))

    if (
        kind == "nervous_cycle"
        and isinstance(out, dict)
        and normalized.get("error") in (None, "")
        and "score" in normalized
        and "points" in normalized
    ):
        normalized["cycle_ok"] = ok
        normalized["ok"] = True
        normalized["degraded"] = not ok
        normalized["scheduler_note"] = (
            "nervous_cycle_completed"
            if ok
            else "nervous_cycle_completed_with_degraded_score"
        )
        return normalized, True

    if (
        kind == "market_open_supervisor"
        and isinstance(out, dict)
        and normalized.get("error") in (None, "")
        and "overall_severity" in normalized
    ):
        supervision_ok = bool(normalized.get("ok", False))
        normalized["supervision_ok"] = supervision_ok
        normalized["ok"] = True
        normalized["degraded"] = normalized.get("overall_severity") not in ("ok", "warning")
        normalized["scheduler_note"] = (
            "market_open_supervisor_completed"
            if supervision_ok
            else "market_open_supervisor_completed_with_findings"
        )
        return normalized, True

    return normalized, ok


def run_job_sync(job: Dict[str, Any]) -> Dict[str, Any]:
    """Run one job synchronously. Returns {ok, result_json, error, ms}. No update_apply."""
    t0 = time.perf_counter()
    kind = (job.get("kind") or "custom").strip().lower()
    payload = job.get("payload") or {}
    jid = job.get("id", "")
    name = job.get("name", "")

    try:
        _append_scheduler_trace(
            "run_job_sync_start",
            {
                "job_id": jid,
                "name": name,
                "kind_raw": job.get("kind"),
                "kind_normalized": kind,
                "payload_keys": sorted(list(payload.keys())) if isinstance(payload, dict) else [],
                "runner_module": __name__,
                "runner_file": __file__,
            },
        )
        if kind == "update_check":
            out = _run_update_check(payload)
        elif kind == "shell_command":
            out = _run_shell_command(payload)
        elif kind == "llm_plan":
            out = _run_llm_plan(payload)
        elif kind == "system_update":
            out = _run_system_update(payload)
        elif kind == "ci_improve":
            out = _run_ci_improve(payload)
        elif kind == "ga_cycle":
            out = _run_ga_cycle(payload)
        elif kind == "metalearn_cycle":
            out = _run_metalearn_cycle(payload)
        elif kind == "remote_hands":
            out = _run_remote_hands(payload)
        elif kind == "ans_cycle":
            out = _run_ans_cycle(payload)
        elif kind == "nervous_cycle":
            out = _run_nervous_cycle(payload)
        elif kind == "market_open_supervisor":
            out = _run_market_open_supervisor(payload)
        elif kind == "makeplay_scanner":
            out = _run_makeplay_scanner(payload)
        elif kind == "repo_monitor_cycle":
            out = _run_repo_monitor_cycle(payload)
        elif kind == "repo_monitor_after_fix":
            out = _run_repo_monitor_after_fix(payload)
        elif kind == "repo_hygiene_cycle":
            out = _run_repo_hygiene_cycle(payload)
        elif kind == "approvals_digest":
            out = _run_approvals_digest(payload)
        elif kind == "approvals_expiry_reaper":
            out = _run_approvals_expiry_reaper(payload)
        elif kind == "world_state_tick":
            out = _run_world_state_tick(payload)
        elif kind == "workshop_cycle":
            out = _run_workshop_cycle(payload)
        elif kind == "pot_execute":
            out = _run_pot_execute(payload)
        elif kind == "pot_dispatch":
            out = _run_pot_dispatch(payload)
        elif kind == "autonomy_cycle":
            out = _run_autonomy_cycle(payload)
        elif kind == "auto_update_cycle":
            out = _run_auto_update_cycle(payload)
        elif kind == "daily_maintenance":
            out = _run_daily_maintenance(payload)
        elif kind == "weekly_maintenance":
            out = _run_weekly_maintenance(payload)
        elif kind == "git_sync":
            out = _run_git_sync(payload)
        elif kind == "cge_tick":
            out = _run_cge_tick(payload)
        else:
            out = {"ok": True, "result": "no-op", "kind": kind}
            _append_scheduler_trace(
                "run_job_sync_noop_branch",
                {
                    "job_id": jid,
                    "name": name,
                    "kind_raw": job.get("kind"),
                    "kind_normalized": kind,
                },
            )
    except Exception as e:
        _log.exception("Job %s run failed: %s", jid, e)
        ms = int((time.perf_counter() - t0) * 1000)
        _append_scheduler_trace(
            "run_job_sync_exception",
            {
                "job_id": jid,
                "name": name,
                "kind_normalized": kind,
                "error": str(e),
            },
        )
        try:
            from modules.humanoid.metalearn.collector import \
                record_scheduler_job

            record_scheduler_job(jid, kind, "fail", ms, str(e))
        except Exception:
            pass
        return {"ok": False, "result_json": None, "error": str(e), "ms": ms}

    out, ok = _normalize_scheduler_result(kind, out)
    ms = int((time.perf_counter() - t0) * 1000)
    try:
        from modules.humanoid.metalearn.collector import record_scheduler_job

        record_scheduler_job(jid, kind, "ok" if ok else "fail", ms, out.get("error"))
    except Exception:
        pass
    result_json = (
        json.dumps(out)
        if isinstance(out, dict)
        else json.dumps({"ok": ok, "raw": str(out)})
    )
    _append_scheduler_trace(
        "run_job_sync_end",
        {
            "job_id": jid,
            "name": name,
            "kind_normalized": kind,
            "ok": ok,
            "result_json_head": result_json[:500],
        },
    )
    return {"ok": ok, "result_json": result_json, "error": out.get("error"), "ms": ms}


def _run_approvals_digest(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Send a periodic digest of pending approvals to Telegram (owner away)."""
    try:
        from modules.humanoid.approvals import list_pending
        from modules.humanoid.approvals.store import expire_pending
        from modules.humanoid.comms.ops_bus import emit as ops_emit

        # Limpieza preventiva de vencidas (no bloqueante si falla).
        try:
            expire_pending(limit=2000)
        except Exception:
            pass

        limit = int((payload or {}).get("limit") or 8)
        pending = list_pending(limit=max(1, min(limit, 20)))
        pending = [p for p in pending if not bool(p.get("expired"))]
        if not pending:
            global _APPROVAL_DIGEST_LAST_KEY
            _APPROVAL_DIGEST_LAST_KEY = ""
            return {"ok": True, "sent": False, "pending": 0}

        ids = sorted([str(p.get("id") or "") for p in pending if (p.get("id") or "")])
        digest_key = hashlib.sha1(
            "|".join(ids).encode("utf-8", errors="ignore")
        ).hexdigest()[:16]

        # Notificar solo cuando cambie el set de pendientes.
        if _APPROVAL_DIGEST_LAST_KEY == digest_key:
            return {
                "ok": True,
                "sent": False,
                "pending": len(pending),
                "reason": "no_changes",
            }
        _APPROVAL_DIGEST_LAST_KEY = digest_key

        # Mensaje compacto
        lines = []
        for a in pending[:limit]:
            aid = a.get("id")
            risk = (a.get("risk") or "med").upper()
            action = a.get("action") or "approval"
            exp = a.get("expires_at") or ""
            exp_s = exp[:19].replace("T", " ") if exp else ""
            lines.append(
                f"- {aid} [{risk}] {action}" + (f" (exp {exp_s})" if exp_s else "")
            )
        text = "Aprobaciones pendientes:\n" + "\n".join(lines)
        level = (
            "high"
            if any(
                (str(p.get("risk") or "").lower() in ("high", "critical"))
                for p in pending
            )
            else "med"
        )
        ops_emit(
            "approval",
            text,
            level=level,
            data={
                "pending": len(pending),
                "ids": [p.get("id") for p in pending[:limit]],
                "digest_key": digest_key,
            },
        )
        return {"ok": True, "sent": True, "pending": len(pending), "shown": len(lines)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_approvals_expiry_reaper(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Periodic cleanup for expired pending approvals."""
    try:
        from modules.humanoid.approvals.store import expire_pending
        from modules.humanoid.comms.ops_bus import emit as ops_emit

        limit = int((payload or {}).get("limit") or 2000)
        out = expire_pending(limit=limit)
        expired_count = int(out.get("expired_count") or 0)
        if expired_count > 0:
            ops_emit(
                "approval",
                f"Limpieza automática de aprobaciones: {expired_count} vencidas marcadas como expiradas.",
                level="med",
                data={
                    "expired_count": expired_count,
                    "source": "approvals_expiry_reaper",
                },
            )
        return {"ok": True, **out}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_update_check(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Update-check only (no apply). Uses humanoid update module plan()."""
    required = payload.get("required_packages") or [
        "fastapi",
        "uvicorn",
        "httpx",
        "pydantic",
    ]
    try:
        from modules.humanoid import get_humanoid_kernel

        k = get_humanoid_kernel()
        update_mod = k.registry.get("update")
        if not update_mod or not hasattr(update_mod, "updater"):
            return {"ok": False, "error": "update module not available"}
        plan_result = update_mod.updater.plan(required)
        return plan_result
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_world_state_tick(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Captura WorldState internamente (sin emitir a canales externos)."""
    try:
        from modules.humanoid.vision.world_state import capture_world_state

        include_items = bool((payload or {}).get("include_ocr_items", False))
        use_llm = bool((payload or {}).get("use_llm_vision", False))
        ws = capture_world_state(
            include_ocr_items=include_items, use_llm_vision=use_llm
        )
        # Solo log interno, sin notificaciones externas
        return {
            "ok": True,
            "world_state_ok": bool(ws.get("ok")),
            "source": ws.get("source"),
            "error": ws.get("error"),
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_remote_hands(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run command via cluster dispatcher (local or best remote node). payload: {command, timeout_sec}."""
    command = (payload.get("command") or "").strip()
    if not command:
        return {"ok": False, "error": "missing command in payload"}
    try:
        from modules.humanoid.dispatch.dispatcher import run_hands

        result = run_hands(
            command,
            timeout_sec=payload.get("timeout_sec", 30),
            prefer_remote=payload.get("prefer_remote", True),
        )
        return {
            "ok": result.get("ok", False),
            "data": result.get("data"),
            "error": result.get("error"),
            "correlation_id": result.get("correlation_id"),
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_shell_command(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run command via Hands SafeShellExecutor + Policy. payload: {command, cwd}."""
    command = (payload.get("command") or "").strip()
    if not command:
        return {"ok": False, "error": "missing command in payload"}
    cwd = payload.get("cwd") or None
    try:
        from modules.humanoid import get_humanoid_kernel
        from modules.humanoid.policy import ActorContext, get_policy_engine

        actor = ActorContext(
            actor=_scheduler_actor, role=os.getenv("POLICY_DEFAULT_ROLE", "owner")
        )
        decision = get_policy_engine().can(
            actor, "hands", "exec_command", target=command
        )
        if not decision.allow:
            return {"ok": False, "error": decision.reason or "policy denied"}
        k = get_humanoid_kernel()
        hands = k.registry.get("hands")
        if not hands or not hasattr(hands, "shell"):
            return {"ok": False, "error": "hands module not available"}
        result = hands.shell.run(
            command, cwd=cwd, timeout_sec=min(RUNNER_TIMEOUT_SEC, 60), actor=actor
        )
        return result
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_market_open_supervisor(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run the native ATLAS market-open supervisor without going through shell policy."""
    label = (payload.get("label") or "scheduled").strip() or "scheduled"
    emit_bitacora = bool(payload.get("emit_bitacora", True))
    emit_telegram = bool(payload.get("emit_telegram", True))
    try:
        import importlib.util

        base = Path(os.getenv("ATLAS_BASE", Path(__file__).resolve().parents[3]))
        script_path = base / "scripts" / "atlas_market_open_supervisor.py"
        spec = importlib.util.spec_from_file_location(
            "atlas_market_open_supervisor_runtime", script_path
        )
        if spec is None or spec.loader is None:
            return {"ok": False, "error": f"unable to load {script_path}"}
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        run_supervision = getattr(module, "run_supervision", None)
        if not callable(run_supervision):
            return {"ok": False, "error": "run_supervision not available"}

        return run_supervision(
            label=label,
            emit_bitacora=emit_bitacora,
            emit_telegram=emit_telegram,
        )
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_llm_plan(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Call humanoid planner (FAST). payload: {goal}."""
    goal = (payload.get("goal") or "").strip()
    if not goal:
        return {"ok": False, "steps": [], "error": "missing goal in payload"}
    try:
        from modules.humanoid import get_humanoid_kernel

        k = get_humanoid_kernel()
        autonomy = k.registry.get("autonomy")
        if not autonomy or not hasattr(autonomy, "planner"):
            return {"ok": False, "steps": [], "error": "autonomy module not available"}
        result = autonomy.planner.plan(goal, fast=True)
        return result
    except Exception as e:
        return {"ok": False, "steps": [], "error": str(e)}


def _run_system_update(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run git-based update engine: check only (no apply) or full apply if policy allows. payload: {action: 'check'|'apply'}."""
    action = (payload.get("action") or "check").strip().lower()
    try:
        from modules.humanoid.update.update_engine import apply as update_apply
        from modules.humanoid.update.update_engine import check as update_check
        from modules.humanoid.update.update_engine import update_enabled

        if not update_enabled():
            return {"ok": False, "error": "UPDATE_ENABLED=false"}
        if action == "apply":
            result = update_apply()
        else:
            result = update_check()
        return (
            result.get("data") or result
            if isinstance(result.get("data"), dict)
            else {"ok": result.get("ok"), "data": result, "error": result.get("error")}
        )
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_ci_improve(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run CI improve cycle (plan_only). payload: {scope, mode, depth, max_items}."""
    scope = (payload.get("scope") or "repo").strip()
    mode = (payload.get("mode") or "plan_only").strip()
    depth = max(1, min(5, int(payload.get("depth") or 2)))
    max_items = payload.get("max_items") or 10
    try:
        from modules.humanoid.ci import run_improve

        result = run_improve(scope=scope, mode=mode, depth=depth, max_items=max_items)
        return result.get("data") or result
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_ga_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run Governed Autonomy cycle. payload: {scope, mode, max_findings}."""
    scope = (payload.get("scope") or "all").strip()
    mode = (payload.get("mode") or os.getenv("GA_MODE", "plan_only")).strip()
    max_findings = payload.get("max_findings")
    try:
        from modules.humanoid.ga.cycle import run_cycle

        result = run_cycle(scope=scope, mode=mode, max_findings=max_findings)
        return result.get("data") or result
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_metalearn_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run meta-learning cycle: train + tune + report."""
    try:
        from modules.humanoid.metalearn.cycle import run_cycle

        result = run_cycle()
        return result
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_ans_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run ANS (Autonomic Nervous System) cycle: checks -> heals -> report."""
    try:
        from modules.humanoid.ans.engine import run_ans_cycle

        mode = payload.get("mode") or os.getenv("ANS_MODE", "auto")
        timeout = int(
            payload.get("timeout_sec")
            or os.getenv("ANS_INTERVAL_SECONDS", "30")
            or "30"
        )
        return run_ans_cycle(mode=mode, timeout_sec=min(timeout, 60))
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_nervous_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run Sistema Nervioso: sensores -> score -> persistencia + bitácora/incidentes."""
    try:
        from modules.humanoid.nervous.engine import run_nervous_cycle

        mode = (
            payload.get("mode") or os.getenv("NERVOUS_MODE", "auto")
        ).strip() or "auto"
        return run_nervous_cycle(mode=mode)
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_makeplay_scanner(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Scanner permanente: estado ATLAS -> webhook MakePlay."""
    try:
        from modules.humanoid.comms.makeplay_scanner import run_scan

        return run_scan()
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_repo_monitor_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Ejecutar un ciclo del monitor de repo (fetch + status + bitácora). Usa scripts/repo_monitor.py --cycle."""
    import subprocess

    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT")
    if not root:
        root = str(Path(__file__).resolve().parent.parent.parent.parent)
    root = Path(root).resolve()
    script = root / "scripts" / "repo_monitor.py"
    if not script.is_file():
        return {"ok": False, "error": "scripts/repo_monitor.py not found"}
    try:
        r = subprocess.run(
            [sys.executable, str(script), "--cycle"],
            cwd=str(root),
            capture_output=True,
            text=True,
            timeout=120,
            env={
                **os.environ,
                "ATLAS_REPO_PATH": str(root),
                "ATLAS_PUSH_ROOT": str(root),
                "REPO_MONITOR_CONFIG": str(root / "config" / "repo_monitor.yaml"),
            },
        )
        return {
            "ok": r.returncode == 0,
            "exit_code": r.returncode,
            "stdout": (r.stdout or "")[:500],
            "stderr": (r.stderr or "")[:300],
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "repo_monitor cycle timeout 120s"}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_repo_monitor_after_fix(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Ejecutar commit + push del monitor de repo (actualizacion automatica al remoto)."""
    import subprocess

    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT")
    if not root:
        root = str(Path(__file__).resolve().parent.parent.parent.parent)
    root = Path(root).resolve()
    script = root / "scripts" / "repo_monitor.py"
    if not script.is_file():
        return {"ok": False, "error": "scripts/repo_monitor.py not found"}
    try:
        r = subprocess.run(
            [sys.executable, str(script), "--after-fix"],
            cwd=str(root),
            capture_output=True,
            text=True,
            timeout=90,
            env={
                **os.environ,
                "ATLAS_REPO_PATH": str(root),
                "ATLAS_PUSH_ROOT": str(root),
                "REPO_MONITOR_CONFIG": str(root / "config" / "repo_monitor.yaml"),
            },
        )
        return {
            "ok": r.returncode == 0,
            "exit_code": r.returncode,
            "stdout": (r.stdout or "")[:500],
            "stderr": (r.stderr or "")[:300],
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "repo_monitor after-fix timeout 90s"}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_workshop_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run central workshop via subprocess. Payload: mode, limit, require_approval_heavy, approval_cooldown_seconds."""
    import subprocess
    from datetime import datetime, timezone

    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT")
    if not root:
        root = str(Path(__file__).resolve().parent.parent.parent.parent)
    root = Path(root).resolve()
    script = root / "scripts" / "atlas_central_workshop.py"
    if not script.is_file():
        return {"ok": False, "error": "scripts/atlas_central_workshop.py not found"}

    mode = (payload.get("mode") or "incidents").strip().lower()
    limit = max(1, min(500, int(payload.get("limit") or 50)))
    require_approval_heavy = bool(payload.get("require_approval_heavy", True))
    approval_cooldown_seconds = max(
        60, int(payload.get("approval_cooldown_seconds") or 900)
    )

    approval_state_path = root / "logs" / "workshop" / "approval_state_runner.json"

    def _load_runner_state() -> Dict[str, Any]:
        try:
            if approval_state_path.is_file():
                return json.loads(approval_state_path.read_text(encoding="utf-8"))
        except Exception:
            pass
        return {"consumed": [], "last_request_ts": "", "last_request_id": ""}

    def _save_runner_state(state: Dict[str, Any]) -> None:
        try:
            approval_state_path.parent.mkdir(parents=True, exist_ok=True)
            approval_state_path.write_text(
                json.dumps(state, indent=2), encoding="utf-8"
            )
        except Exception:
            pass

    def _check_heavy_gate() -> Dict[str, Any]:
        """Gate for heavy modes. Returns {granted, reason, approval_id?, pending?}."""
        try:
            from modules.humanoid.approvals import (create, list_all,
                                                    list_pending)
        except Exception as e:
            return {
                "granted": False,
                "reason": f"approval_module_unavailable: {e}",
                "pending": False,
            }

        state = _load_runner_state()
        consumed = set(state.get("consumed") or [])

        approved = list_all(limit=50, status="approved")
        for item in approved:
            pl = item.get("payload") or {}
            if pl.get("domain") == "workshop" and pl.get("operation") == "maintenance":
                aid = str(item.get("id") or "")
                if aid and aid not in consumed:
                    consumed.add(aid)
                    state["consumed"] = sorted(consumed)[-300:]
                    _save_runner_state(state)
                    return {"granted": True, "reason": "approved", "approval_id": aid}

        pending = list_pending(limit=50)
        for item in pending:
            pl = item.get("payload") or {}
            if pl.get("domain") == "workshop" and pl.get("operation") == "maintenance":
                return {
                    "granted": False,
                    "reason": "pending",
                    "pending": True,
                    "approval_id": item.get("id"),
                }

        now = time.time()
        try:
            last_ts = state.get("last_request_ts") or ""
            last_epoch = (
                datetime.fromisoformat(last_ts.replace("Z", "+00:00")).timestamp()
                if last_ts
                else 0.0
            )
        except Exception:
            last_epoch = 0.0
        if now - last_epoch < approval_cooldown_seconds:
            return {
                "granted": False,
                "reason": "cooldown",
                "pending": True,
                "approval_id": state.get("last_request_id"),
            }

        pl = {
            "domain": "workshop",
            "operation": "maintenance",
            "source": "scheduler",
            "mode": mode,
            "intent": "central_workshop_heavy_run",
            "details": "Execute maintenance/heavy repairs in Central Workshop",
            "risk": "high",
        }
        out = create(action="execute", payload=pl, job_id="workshop_cycle")
        if out.get("ok") and out.get("approval_id"):
            state["last_request_ts"] = datetime.now(timezone.utc).isoformat()
            state["last_request_id"] = out.get("approval_id")
            _save_runner_state(state)
            return {
                "granted": False,
                "reason": "created_pending",
                "pending": True,
                "approval_id": out.get("approval_id"),
            }
        return {
            "granted": False,
            "reason": out.get("error") or "approval_create_failed",
            "pending": False,
        }

    if mode == "incidents":
        pass
    elif mode in ("full", "maintenance") and require_approval_heavy:
        gate = _check_heavy_gate()
        if not gate.get("granted"):
            return {
                "ok": True,
                "result": "awaiting_approval",
                "state": "awaiting_approval",
                "reason": gate.get("reason"),
                "approval_id": gate.get("approval_id"),
            }

    argv = [sys.executable, str(script), "--mode", mode, "--limit", str(limit)]
    try:
        r = subprocess.run(
            argv,
            cwd=str(root),
            capture_output=True,
            text=True,
            timeout=300,
            env={
                **os.environ,
                "ATLAS_REPO_PATH": str(root),
                "ATLAS_PUSH_ROOT": str(root),
            },
        )
        stdout = (r.stdout or "").strip()
        stderr = (r.stderr or "").strip()
        out = {
            "ok": r.returncode in (0, 2),
            "exit_code": r.returncode,
            "mode": mode,
            "stdout": stdout[-1000:],
            "stderr": stderr[-500:],
        }
        for line in stdout.splitlines():
            if line.startswith("WORKSHOP_DONE:"):
                out["workshop_done"] = line.split(":", 1)[-1].strip()
            elif line.startswith("REPORT_JSON:"):
                out["report_json"] = line.split(":", 1)[-1].strip()
        return out
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "workshop_cycle timeout 300s", "mode": mode}
    except Exception as e:
        return {"ok": False, "error": str(e), "mode": mode}


# ============================================================================
# POT INTEGRATION - EJECUCIÓN AUTOMÁTICA DE POTS
# ============================================================================


def _run_pot_execute(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Ejecuta un POT directamente. Payload: pot_id, context, dry_run.

    Esta es la integración directa scheduler → quality module.
    """
    pot_id = payload.get("pot_id")
    if not pot_id:
        return {"ok": False, "error": "pot_id required in payload"}

    context = payload.get("context") or {}
    dry_run = bool(payload.get("dry_run", False))

    try:
        from modules.humanoid.approvals import create as create_approval
        from modules.humanoid.approvals import wait_for_resolution
        from modules.humanoid.quality.executor import execute_pot
        from modules.humanoid.quality.registry import get_pot

        pot = get_pot(pot_id)
        if not pot:
            return {"ok": False, "error": f"POT not found: {pot_id}"}

        severity = (
            str(pot.severity.value if hasattr(pot.severity, "value") else pot.severity)
            .strip()
            .lower()
        )
        require_approval = (
            bool(payload.get("require_approval", True))
            and not dry_run
            and severity in ("high", "critical")
        )

        if require_approval:
            approval_result = create_approval(
                action=f"pot_execute:{pot_id}",
                payload={
                    "pot_id": pot_id,
                    "trigger": "scheduler",
                    "risk": severity,
                },
                job_id="pot_execute",
            )
            if not approval_result.get("ok"):
                return {
                    "ok": False,
                    "error": approval_result.get("error") or "approval_create_failed",
                    "pot_id": pot_id,
                }

            approval_id = approval_result.get("approval_id")
            if not approval_id:
                return {"ok": False, "error": "approval_id_missing", "pot_id": pot_id}

            resolution = wait_for_resolution(
                approval_id,
                timeout_seconds=int(payload.get("approval_timeout_seconds") or 300),
            )
            if resolution.get("status") != "approved":
                return {
                    "ok": False,
                    "pot_id": pot_id,
                    "error": f"approval_{resolution.get('status') or 'denied'}",
                    "approval_id": approval_id,
                }

        result = execute_pot(
            pot=pot,
            context=context,
            dry_run=dry_run,
            stop_on_failure=True,
            sync_to_cerebro=True,
            notify_on_complete=True,
        )

        return {
            "ok": result.ok,
            "pot_id": pot_id,
            "severity": severity,
            "approval_required": require_approval,
            "steps_ok": result.steps_ok,
            "steps_total": result.steps_total,
            "elapsed_ms": result.elapsed_ms,
            "report_path": result.report_path,
            "rollback_executed": result.rollback_executed,
        }
    except Exception as e:
        return {"ok": False, "error": str(e), "pot_id": pot_id}


def _run_pot_dispatch(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Despacha un POT via el dispatcher (queue async).
    Payload: pot_id, context, priority.
    """
    pot_id = payload.get("pot_id")
    context = payload.get("context") or {}
    priority = int(payload.get("priority", 5))

    try:
        from modules.humanoid.quality.dispatcher import (dispatch_pot,
                                                         get_dispatcher)

        # Asegurar que el dispatcher está corriendo
        dispatcher = get_dispatcher()
        if not dispatcher.is_running():
            dispatcher.start()

        request_id = dispatch_pot(pot_id, context=context, priority=priority)

        return {
            "ok": True,
            "request_id": request_id,
            "pot_id": pot_id,
            "dispatched": True,
        }
    except Exception as e:
        return {"ok": False, "error": str(e), "pot_id": pot_id}


def _run_autonomy_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Ejecuta el ciclo de autonomía completo via POT.
    Shortcut para pot_execute con pot_id=autonomy_full_cycle.
    """
    return _run_pot_execute(
        {
            "pot_id": "autonomy_full_cycle",
            "context": payload.get("context") or {},
            "dry_run": payload.get("dry_run", False),
        }
    )


def _run_auto_update_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Ejecuta el ciclo de actualización automática via POT.
    Shortcut para pot_execute con pot_id=auto_update_full.
    """
    return _run_pot_execute(
        {
            "pot_id": "auto_update_full",
            "context": payload.get("context") or {},
            "dry_run": payload.get("dry_run", False),
        }
    )


def _run_daily_maintenance(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Ejecuta mantenimiento diario via POT.
    """
    return _run_pot_execute(
        {
            "pot_id": "maintenance_daily",
            "context": payload.get("context") or {},
            "dry_run": payload.get("dry_run", False),
        }
    )


def _run_weekly_maintenance(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Ejecuta mantenimiento semanal via POT.
    """
    return _run_pot_execute(
        {
            "pot_id": "maintenance_weekly",
            "context": payload.get("context") or {},
            "dry_run": payload.get("dry_run", False),
        }
    )


def _run_git_sync(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Ejecuta sincronización Git (commit + push) via POT.
    """
    message = payload.get("message", "chore: scheduled sync by ATLAS")

    # Primero verificar si hay cambios
    import subprocess

    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT")
    if not root:
        root = str(Path(__file__).resolve().parent.parent.parent.parent)

    try:
        status = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=root,
            capture_output=True,
            text=True,
            timeout=30,
        )
        if not (status.stdout or "").strip():
            return {"ok": True, "result": "no_changes", "skipped": True}
    except Exception as e:
        return {"ok": False, "error": f"git_status_check: {e}"}

    # Ejecutar POT de commit
    commit_result = _run_pot_execute(
        {
            "pot_id": "git_commit",
            "context": {"commit_message": message},
        }
    )

    if not commit_result.get("ok"):
        return commit_result

    # Ejecutar POT de push
    push_result = _run_pot_execute(
        {
            "pot_id": "git_push",
            "context": {},
        }
    )

    return {
        "ok": push_result.get("ok", False),
        "commit_result": commit_result,
        "push_result": push_result,
    }


def _run_repo_hygiene_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Repo hygiene periódico: scan o auto (según env). Usa scripts/repo_hygiene.py."""
    import subprocess

    root = (
        os.getenv("ATLAS_REPO_PATH")
        or os.getenv("ATLAS_PUSH_ROOT")
        or payload.get("repo_root")
    )
    if not root:
        root = str(Path(__file__).resolve().parent.parent.parent.parent)
    root = Path(root).resolve()
    script = root / "scripts" / "repo_hygiene.py"
    if not script.is_file():
        return {"ok": False, "error": "scripts/repo_hygiene.py not found"}

    auto = os.getenv("REPO_HYGIENE_AUTO", "false").strip().lower() in (
        "1",
        "true",
        "yes",
    )
    argv = [sys.executable, str(script), "--auto" if auto else "--scan"]
    try:
        r = subprocess.run(
            argv,
            cwd=str(root),
            capture_output=True,
            text=True,
            timeout=180,
            env={
                **os.environ,
                "ATLAS_REPO_PATH": str(root),
                "ATLAS_PUSH_ROOT": str(root),
            },
        )
        return {
            "ok": r.returncode == 0,
            "exit_code": r.returncode,
            "mode": "auto" if auto else "scan",
            "stdout": (r.stdout or "")[-500:],
            "stderr": (r.stderr or "")[-300:],
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "repo_hygiene timeout 180s"}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_cge_tick(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Tick del Concurrent Goal Engine — ejecuta un ciclo del motor de metas concurrente."""
    try:
        from modules.humanoid.cortex.frontal.concurrent_engine import cge_tick

        return cge_tick()
    except Exception as e:
        return {"ok": False, "error": str(e)}
