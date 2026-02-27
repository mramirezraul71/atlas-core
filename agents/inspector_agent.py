"""Agente Inspector: auto-auditoría. Lee logs de errores, formula diagnóstico y lo devuelve al Orquestador."""
from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Any, Dict, List

logger = logging.getLogger("atlas.inspector")

BASE = Path(__file__).resolve().parent.parent
LOGS_DIR = Path(os.environ.get("ATLAS_LOGS_DIR", str(BASE / "logs")))
MAX_TAIL = 50
ERROR_KEYWORDS = ("error", "exception", "traceback", "failed", "failure", "critical", "fatal")


def _read_log_tail(log_path: Path, lines: int = 100) -> List[str]:
    try:
        if not log_path.exists() or not log_path.is_file():
            return []
        with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
            all_lines = f.readlines()
        return all_lines[-lines:] if len(all_lines) > lines else all_lines
    except Exception:
        return []


def _detect_failures(lines: List[str]) -> List[Dict[str, Any]]:
    findings = []
    for i, line in enumerate(lines):
        lower = line.lower()
        if any(kw in lower for kw in ERROR_KEYWORDS):
            findings.append({"line_no": i + 1, "content": line.strip()[:200]})
    return findings[:20]


def _formulate_diagnosis(findings: List[Dict[str, Any]], log_name: str) -> str:
    if not findings:
        return ""
    parts = [f"En {log_name} se detectaron {len(findings)} líneas con indicios de fallo:"]
    for f in findings[:5]:
        parts.append(f"  L{f.get('line_no')}: {f.get('content', '')[:120]}")
    return "\n".join(parts)


def inspect_system(snapshot: dict) -> Dict[str, Any]:
    """Inspecciona logs del sistema y devuelve diagnóstico al Orquestador. Nunca lanza."""
    try:
        logs_dir = Path(LOGS_DIR)
        if not logs_dir.exists():
            return {"ok": True, "diagnosis": None, "findings": []}

        all_findings = []
        diagnosis_parts = []

        for log_file in sorted(logs_dir.glob("*.log"))[:5]:
            lines = _read_log_tail(log_file, MAX_TAIL)
            findings = _detect_failures(lines)
            if findings:
                all_findings.extend(findings)
                diagnosis_parts.append(_formulate_diagnosis(findings, log_file.name))

        diagnosis = "\n---\n".join(diagnosis_parts) if diagnosis_parts else None
        return {"ok": True, "diagnosis": diagnosis, "findings": all_findings[:30]}
    except Exception as e:
        logger.exception("inspector error: %s", e)
        return {"ok": False, "error": str(e), "diagnosis": None}
