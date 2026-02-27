"""Reporte Cerebro: qué pasó, qué hice, por qué, resultado, impacto, rollback. Resumen máximo 10 bullets."""
from __future__ import annotations

import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

REPORT_DIR = Path(os.getenv("BRAIN_REPORT_DIR", "C:\\ATLAS_PUSH\\snapshots\\brain"))


def format_report(goal: str, bullets: List[str], ok: bool, rollback: bool = False) -> str:
    """Formatea reporte: max 10 bullets, concreto."""
    lines = [
        f"# Reporte Cerebro - {datetime.now(timezone.utc).isoformat()}",
        f"**Goal:** {goal}",
        f"**Resultado:** {'OK' if ok else 'FAIL'}",
        f"**Rollback:** {'Sí' if rollback else 'No'}",
        "",
        "**Acción:**",
    ]
    for b in bullets[:10]:
        lines.append(f"- {b}")
    return "\n".join(lines)


def emit_report(goal: str, bullets: List[str], ok: bool = True, rollback: bool = False, payload: Optional[Dict] = None) -> str:
    """Emite reporte y guarda en disco. Retorna path."""
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    path = REPORT_DIR / f"brain_report_{ts}.md"
    content = format_report(goal, bullets, ok, rollback)
    path.write_text(content, encoding="utf-8")
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("brain_report", "system", "brain_report", "emit", ok, 0, None, payload or {}, {"path": str(path)})
    except Exception:
        pass
    return str(path)


def get_latest_report() -> Optional[str]:
    """Retorna path del último reporte."""
    if not REPORT_DIR.exists():
        return None
    files = sorted(REPORT_DIR.glob("brain_report_*.md"), key=lambda p: p.stat().st_mtime, reverse=True)
    return str(files[0]) if files else None
