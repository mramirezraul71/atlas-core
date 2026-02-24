"""
Supervisor - Meta-agent that audits Atlas, directs the Agent, and reports to the Owner.

Role hierarchy: Owner (Raúl) → Supervisor → Agent/Atlas
The Supervisor:
  1. Monitors Atlas health, audit logs, task execution
  2. Can investigate issues using tool calling (same engine as Agent)
  3. Gives directives to Atlas and the Agent
  4. Reports findings and status to the Owner
"""
from __future__ import annotations

import json
import os
import time
from typing import Any, Dict, Generator, List, Optional

from .audit import AuditLogger

# Import memory integration
try:
    from nexus.atlas_nexus.brain.supervisor_memory_integration import get_supervisor_memory
    MEMORY_AVAILABLE = True
except ImportError:
    MEMORY_AVAILABLE = False

_ATLAS_BASE = "http://127.0.0.1:8791"
_TIMEOUT = 5


def _get(path: str) -> Optional[dict]:
    try:
        import requests
        r = requests.get(f"{_ATLAS_BASE}{path}", timeout=_TIMEOUT)
        return r.json() if r.ok else None
    except Exception:
        return None


SUPERVISOR_SYSTEM_PROMPT = """Eres el SUPERVISOR de ATLAS, un meta-agente subordinado al Owner (Raúl).

TU ROL:
- Auditas el trabajo de ATLAS y del Agente
- Investigas problemas usando herramientas reales (leer archivos, ejecutar comandos, consultar APIs)
- Das instrucciones concretas y directivas a Atlas
- Reportas al Owner con datos reales, no suposiciones

REGLAS:
1. Siempre VERIFICA antes de reportar: lee logs, consulta endpoints, revisa código
2. Cada hallazgo debe tener EVIDENCIA (archivo, log, endpoint consultado)
3. Cada recomendación debe ser EJECUTABLE (comando, cambio de código, endpoint)
4. Si detectas un problema, intenta diagnosticarlo a fondo usando las herramientas
5. Responde en español, profesional y directo
6. La raíz del proyecto ATLAS es C:\\ATLAS_PUSH

FORMATO DE REPORTE:
## ESTADO ACTUAL
[Resumen de salud con datos reales]

## HALLAZGOS
[Problemas encontrados con evidencia]

## DIRECTIVAS
[Acciones que Atlas debe tomar, con comandos/endpoints concretos]

## RECOMENDACIONES AL OWNER
[Lo que Raúl necesita saber o decidir]"""


def _load_resident_policy_text() -> str:
    try:
        from atlas_adapter.supervisor_policy import get_supervisor_policy

        j = get_supervisor_policy()
        if isinstance(j, dict) and j.get("ok") and (j.get("policy") or "").strip():
            return str(j.get("policy") or "").strip()
    except Exception:
        pass
    return ""


class Supervisor:
    """
    Meta-agent that audits Atlas, investigates with tools, and reports to Owner.
    Uses the same AgentEngine as the main Agent for deep investigation.
    """

    def __init__(self):
        self.audit = AuditLogger()
        
        # Initialize memory integration
        self.memory = None
        if MEMORY_AVAILABLE:
            try:
                self.memory = get_supervisor_memory()
                # Avoid unicode/emoji prints on Windows console encodings
                print("Supervisor Memory Integration loaded")
            except Exception as e:
                print(f"Supervisor Memory Integration failed: {e}")
        else:
            print("Supervisor Memory Integration not available")

    def advise(self, objective: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """Quick advisory: gather data + LLM analysis without tool calling."""
        self.audit.log("supervisor_request", {
            "objective": objective,
            "context_keys": list(context.keys()),
        })

        t0 = time.perf_counter()
        snapshot = self._gather_system_snapshot()
        gather_ms = int((time.perf_counter() - t0) * 1000)

        diagnosis = self._diagnose(snapshot)
        
        # Enhance with memory context
        memory_context = ""
        if self.memory:
            memory_context = self.memory.enhance_supervisor_analysis(objective, snapshot)

        # Add persistent thread memory if provided by API layer
        thread_memory_text = ""
        try:
            tm = context.get("_thread_memory")
            if isinstance(tm, list) and tm:
                lines = []
                for m in tm[-12:]:
                    role = (m.get("role") or "").strip().lower()
                    content = (m.get("content") or "").strip()
                    if not content:
                        continue
                    if role not in ("user", "assistant", "system"):
                        role = "user"
                    lines.append(f"[{role}] {content[:260]}")
                if lines:
                    thread_memory_text = "\n".join(lines)
        except Exception:
            thread_memory_text = ""

        resident_policy = _load_resident_policy_text()
        prompt = self._build_analysis_prompt(
            objective,
            context,
            snapshot,
            diagnosis,
            memory_context,
            thread_memory_text,
            resident_policy=resident_policy,
        )

        try:
            from atlas_adapter.atlas_http_api import _direct_model_call
            result = _direct_model_call("auto", prompt, use_config=False, prefer_fast=False, enrich=False)
        except Exception as e:
            return {"ok": False, "error": str(e), "analysis": f"Error LLM: {e}",
                    "recommendations": [], "snapshot": snapshot, "diagnosis": diagnosis}

        if not result.get("ok"):
            return {"ok": False, "error": result.get("error"),
                    "analysis": f"LLM falló: {result.get('error')}",
                    "recommendations": [], "snapshot": snapshot, "diagnosis": diagnosis}

        analysis = result.get("output", "")
        actions = self._extract_actions(analysis)

        return {
            "ok": True,
            "analysis": analysis,
            "snapshot": snapshot,
            "diagnosis": diagnosis,
            "actions": actions,
            "recommendations": self._extract_lines(analysis, "recomend"),
            "gather_ms": gather_ms,
            "model_used": result.get("model_used"),
            "ms": int(result.get("ms", 0)),
        }

    def investigate(self, objective: str, context: Dict[str, Any]) -> Generator[Dict[str, Any], None, None]:
        """
        Deep investigation using tool calling (like the Agent).
        Yields SSE events for streaming to the frontend.
        """
        self.audit.log("supervisor_investigate", {"objective": objective})

        snapshot = self._gather_system_snapshot()
        diagnosis = self._diagnose(snapshot)

        # Enhance with memory context
        memory_context = ""
        if self.memory:
            memory_context = self.memory.enhance_supervisor_analysis(objective, snapshot)

        enriched_msg = (
            f"OBJETIVO DEL OWNER: {objective}\n\n"
            f"DIAGNÓSTICO AUTOMÁTICO (severidad: {diagnosis['severity']}):\n"
        )
        for i in diagnosis.get("issues", []):
            enriched_msg += f"- PROBLEMA: {i}\n"
        for w in diagnosis.get("warnings", []):
            enriched_msg += f"- ADVERTENCIA: {w}\n"
        for o in diagnosis.get("ok_items", []):
            enriched_msg += f"- OK: {o}\n"

        snap_json = json.dumps(snapshot, ensure_ascii=False, default=str)
        if len(snap_json) > 2000:
            snap_json = snap_json[:2000] + "..."
        enriched_msg += f"\nDATOS DEL SISTEMA:\n{snap_json}\n\n"
        
        # Add memory context if available
        if memory_context:
            enriched_msg += memory_context + "\n\n"

        # Add persistent thread memory (summary + last messages) if present
        try:
            tm = context.get("_thread_memory")
            if isinstance(tm, list) and tm:
                lines = []
                for m in tm[-12:]:
                    role = (m.get("role") or "").strip().lower()
                    content = (m.get("content") or "").strip()
                    if not content:
                        continue
                    if role not in ("user", "assistant", "system"):
                        role = "user"
                    lines.append(f"[{role}] {content[:260]}")
                if lines:
                    enriched_msg += "HILO CONVERSACIONAL (resumen+últimos mensajes):\n" + "\n".join(lines) + "\n\n"
        except Exception:
            pass
        
        enriched_msg += "Investiga a fondo usando las herramientas disponibles. Lee archivos, consulta endpoints, ejecuta comandos según necesites."

        from atlas_adapter.agent_engine import run_agent
        resident_policy = _load_resident_policy_text()
        system_prompt = SUPERVISOR_SYSTEM_PROMPT
        if resident_policy:
            system_prompt = system_prompt + "\n\n" + "POLÍTICA RESIDENTE DEL SUPERVISOR (obligatoria):\n" + resident_policy
        yield from run_agent(
            enriched_msg,
            system_prompt=system_prompt,
        )

    def _gather_system_snapshot(self) -> Dict[str, Any]:
        snap: Dict[str, Any] = {}

        h = _get("/health")
        if h:
            snap["health"] = {"score": h.get("score"), "checks": h.get("checks", {})}

        s = _get("/status")
        if s:
            snap["status"] = {
                "ok": s.get("ok"),
                "robot_connected": s.get("robot_connected"),
                "nexus_connected": s.get("nexus_connected"),
            }

        aud = _get("/audit/tail?n=15")
        if aud and aud.get("entries"):
            snap["recent_audit"] = [
                {"ts": e.get("ts", ""), "module": e.get("module", ""),
                 "action": e.get("action", ""), "ok": e.get("ok", True),
                 "error": (e.get("error") or "")[:100], "ms": e.get("ms", 0)}
                for e in aud["entries"][:15]
            ]

        auto = _get("/api/autonomy/status")
        if auto:
            snap["autonomy"] = auto

        wd = _get("/watchdog/status")
        if wd:
            snap["watchdog"] = wd

        bit = _get("/bitacora/stream?since_id=0")
        if bit and bit.get("entries"):
            snap["bitacora_recent"] = [
                {"ts": e.get("timestamp", ""), "msg": (e.get("message") or "")[:120],
                 "ok": e.get("ok", True), "source": e.get("source", "")}
                for e in bit["entries"][:10]
            ]

        try:
            import psutil
            snap["system"] = {
                "cpu_pct": psutil.cpu_percent(interval=0.3),
                "ram_pct": psutil.virtual_memory().percent,
                "disk_pct": psutil.disk_usage("C:\\").percent,
            }
        except Exception:
            pass

        return snap

    def _diagnose(self, snap: Dict[str, Any]) -> Dict[str, Any]:
        issues: List[str] = []
        warnings: List[str] = []
        ok_items: List[str] = []

        health = snap.get("health", {})
        score = health.get("score")
        if score is not None:
            if score < 50:
                issues.append(f"Health score critico: {score}/100")
            elif score < 80:
                warnings.append(f"Health score bajo: {score}/100")
            else:
                ok_items.append(f"Health score: {score}/100")

        checks = health.get("checks", {})
        for name, val in checks.items():
            if isinstance(val, dict):
                if not val.get("ok", True):
                    issues.append(f"Check '{name}' fallido: {val.get('error', 'unknown')}")
            elif val is False:
                issues.append(f"Check '{name}' fallido")

        audit = snap.get("recent_audit", [])
        errors = [e for e in audit if not e.get("ok", True)]
        if errors:
            modules_with_errors = list(set(e.get("module", "?") for e in errors))
            issues.append(f"{len(errors)} errores recientes en auditoría (módulos: {', '.join(modules_with_errors)})")
            for e in errors[:3]:
                issues.append(f"  → {e.get('module')}.{e.get('action')}: {e.get('error', '?')}")

        status = snap.get("status", {})
        if status and not status.get("robot_connected"):
            warnings.append("Robot no conectado")

        sys = snap.get("system", {})
        if sys.get("cpu_pct", 0) > 85:
            warnings.append(f"CPU alta: {sys['cpu_pct']}%")
        if sys.get("ram_pct", 0) > 90:
            warnings.append(f"RAM critica: {sys['ram_pct']}%")
        if sys.get("disk_pct", 0) > 90:
            warnings.append(f"Disco casi lleno: {sys['disk_pct']}%")

        bitacora = snap.get("bitacora_recent", [])
        bit_errors = [b for b in bitacora if not b.get("ok", True)]
        if bit_errors:
            issues.append(f"{len(bit_errors)} entradas con error en bitácora reciente")

        return {
            "issues": issues,
            "warnings": warnings,
            "ok_items": ok_items,
            "severity": "critical" if len(issues) > 3 else "warning" if issues else "healthy",
        }

    def _build_analysis_prompt(
        self,
        objective: str,
        context: Dict[str, Any],
        snapshot: Dict[str, Any],
        diagnosis: Dict[str, Any],
        memory_context: str = "",
        thread_memory: str = "",
        resident_policy: str = "",
    ) -> str:
        issues_ctx = context.get("issues", [])
        modules_ctx = context.get("modules", [])

        snap_json = json.dumps(snapshot, ensure_ascii=False, default=str)
        if len(snap_json) > 3000:
            snap_json = snap_json[:3000] + "...(truncado)"

        diag_lines = []
        for i in diagnosis.get("issues", []):
            diag_lines.append(f"PROBLEMA: {i}")
        for w in diagnosis.get("warnings", []):
            diag_lines.append(f"ADVERTENCIA: {w}")
        for o in diagnosis.get("ok_items", []):
            diag_lines.append(f"OK: {o}")

        thread_section = ""
        if thread_memory:
            thread_section = "HILO CONVERSACIONAL (resumen+últimos mensajes):\n" + thread_memory + "\n"

        policy_section = ""
        if resident_policy:
            policy_section = "\nPOLÍTICA RESIDENTE (obligatoria):\n" + resident_policy + "\n"

        return f"""Eres el Supervisor técnico de ATLAS, subordinado al Owner (Raúl).

OBJETIVO DEL OWNER: {objective}
{f"Issues reportados: {', '.join(issues_ctx)}" if issues_ctx else ""}
{f"Módulos de interés: {', '.join(modules_ctx)}" if modules_ctx else ""}

{thread_section}
{policy_section}

DIAGNÓSTICO AUTOMÁTICO (Severidad: {diagnosis['severity'].upper()}):
{chr(10).join(diag_lines) if diag_lines else "Sin problemas detectados"}

DATOS REALES DEL SISTEMA:
{snap_json}

{memory_context}

INSTRUCCIONES:
1. DIAGNÓSTICO (máx 5 líneas): qué está pasando, citando datos del snapshot.
2. ACCIONES (3-5): Cada una con formato:
   ACTION: [tipo] [descripción]
   RISK: low|high
   EXECUTE: [endpoint/comando/archivo exacto]
3. DIRECTIVAS PARA ATLAS: Qué debe hacer Atlas automáticamente.
4. REPORTE AL OWNER: Qué necesita saber Raúl.

REGLAS DE EJECUCIÓN PARA ACCIONES:
- Usa SOLO endpoints reales del servidor ATLAS en 127.0.0.1:8791.
- Evita comandos con jq/curl como requisito; prefiere formato PowerShell o endpoint directo.
- No inventes endpoints. Para autonomía/healing usa preferentemente:
  * POST /api/autonomy/daemon/start
  * POST /api/autonomy/daemon/stop
  * GET  /api/autonomy/status
  * GET  /healing/status
  * POST /api/healing/trigger
- NO usar: /api/autonomy/subsystem/activate (endpoint inválido).

Solo usa endpoints reales de ATLAS."""

    def _extract_actions(self, text: str) -> List[Dict[str, Any]]:
        actions = []
        lines = text.split("\n")
        current: Optional[Dict[str, Any]] = None

        for line in lines:
            stripped = line.strip()
            up = stripped.upper()

            if up.startswith("ACTION:"):
                if current:
                    actions.append(current)
                current = {"description": stripped[7:].strip(), "risk": "low", "execute": ""}
            elif up.startswith("RISK:") and current:
                current["risk"] = stripped[5:].strip().lower()
            elif up.startswith("EXECUTE:") and current:
                current["execute"] = stripped[8:].strip()

        if current:
            actions.append(current)
        return actions

    def _extract_lines(self, text: str, keyword: str) -> List[str]:
        lines = []
        for line in text.split("\n"):
            stripped = line.strip()
            if stripped.startswith("- ") and keyword.lower() in stripped.lower():
                lines.append(stripped[2:])
            elif stripped.startswith("- ") and len(lines) > 0:
                lines.append(stripped[2:])
        return lines[:5]
