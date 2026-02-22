"""Supervisor - Technical diagnostic engine for ATLAS.

Pattern: Gather real data FIRST, then LLM analyzes and recommends.
Never ask the LLM to invent data or use fake commands.
"""
import json
import os
import time
from typing import Any, Dict, List, Optional

import requests

from .audit import AuditLogger

_ATLAS_BASE = "http://127.0.0.1:8791"
_TIMEOUT = 5


def _get(path: str) -> Optional[dict]:
    try:
        r = requests.get(f"{_ATLAS_BASE}{path}", timeout=_TIMEOUT)
        return r.json() if r.ok else None
    except Exception:
        return None


def _post(path: str, body: dict) -> Optional[dict]:
    try:
        r = requests.post(f"{_ATLAS_BASE}{path}", json=body, timeout=_TIMEOUT)
        return r.json() if r.ok else None
    except Exception:
        return None


class Supervisor:
    """
    Technical Supervisor that gathers REAL system data before asking the LLM.
    The LLM receives facts, not prompts to guess.
    """

    def __init__(self):
        self.audit = AuditLogger()
        self.owner = "Raúl"

    def advise(self, objective: str, context: Dict[str, Any]) -> Dict[str, Any]:
        self.audit.log("supervisor_request", {
            "objective": objective,
            "context_keys": list(context.keys()),
        })

        t0 = time.perf_counter()
        snapshot = self._gather_system_snapshot()
        gather_ms = int((time.perf_counter() - t0) * 1000)

        diagnosis = self._diagnose(snapshot)
        prompt = self._build_analysis_prompt(objective, context, snapshot, diagnosis)

        try:
            from atlas_adapter.atlas_http_api import _direct_model_call
            result = _direct_model_call("auto", prompt, use_config=False, prefer_fast=False)
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

    def _gather_system_snapshot(self) -> Dict[str, Any]:
        """Gather REAL data from all available ATLAS endpoints."""
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

        heal = _get("/healing/status")
        if heal:
            snap["healing"] = heal

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
        """Deterministic diagnosis from snapshot data (no LLM needed)."""
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

    def _build_analysis_prompt(self, objective: str, context: Dict[str, Any],
                                snapshot: Dict[str, Any], diagnosis: Dict[str, Any]) -> str:
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

        return f"""Eres el Supervisor técnico de ATLAS. Analiza estos DATOS REALES del sistema y responde al objetivo del Owner.

OBJETIVO DEL OWNER: {objective}
{f"Issues reportados: {', '.join(issues_ctx)}" if issues_ctx else ""}
{f"Módulos de interés: {', '.join(modules_ctx)}" if modules_ctx else ""}

═══ DIAGNÓSTICO AUTOMÁTICO ═══
Severidad: {diagnosis['severity'].upper()}
{chr(10).join(diag_lines) if diag_lines else "Sin problemas detectados"}

═══ DATOS REALES DEL SISTEMA ═══
{snap_json}

═══ INSTRUCCIONES ═══
Basándote SOLO en los datos reales de arriba:

1. DIAGNÓSTICO (máximo 5 líneas): ¿qué está pasando y por qué? Cita datos concretos del snapshot.

2. ACCIONES (3-5 acciones ejecutables):
Cada acción debe ser UNA de estas:
- ENDPOINT: método + URL real de ATLAS (ej: GET /health, POST /api/workspace/terminal/execute)
- COMANDO: comando de terminal real (ej: python script.py, pip install X)
- ARCHIVO: ruta real + qué cambiar (ej: editar modules/humanoid/X.py línea Y)

Formato de cada acción:
ACTION: [tipo] [descripción corta]
RISK: low|high
EXECUTE: [endpoint/comando/archivo exacto]

3. VERIFICACIÓN: Cómo confirmar que se resolvió (1 endpoint o comando).

NO inventes datos. NO uses comandos que no existen. Solo endpoints reales de ATLAS (/health, /audit/tail, /status, /api/autonomy/status, /watchdog/status, /api/workspace/terminal/execute, etc)."""

    def _extract_actions(self, text: str) -> List[Dict[str, Any]]:
        """Extract structured actions from LLM response."""
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
