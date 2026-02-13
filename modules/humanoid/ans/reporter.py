"""Minimal incident report + UI events + telegram notifications."""
from __future__ import annotations

import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

_REPORT_DIR: str = ""


def _report_dir() -> Path:
    global _REPORT_DIR
    if not _REPORT_DIR:
        v = os.getenv("ANS_REPORT_DIR", "").strip()
        if v:
            _REPORT_DIR = v
        else:
            base = (os.getenv("POLICY_ALLOWED_PATHS") or "C:\\ATLAS_PUSH").strip().split(",")[0].strip()
            _REPORT_DIR = str(Path(base) / "snapshots" / "ans")
    return Path(_REPORT_DIR)


def write_report(incidents: List[Dict], actions: List[Dict], summary: str = "") -> str:
    Path(_report_dir()).mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    path = _report_dir() / f"ANS_REPORT_{ts}.md"
    lines = [f"# ANS Report {ts}", "", f"Summary: {summary or 'ok'}", ""]
    if incidents:
        lines.append("## Incidents")
        for i in incidents[:20]:
            lines.append(f"- [{i.get('severity')}] {i.get('check_id')}: {i.get('message')}")
        lines.append("")
    if actions:
        lines.append("## Actions")
        for a in actions[-20:]:
            lines.append(f"- {a.get('heal_id', '')}: {a.get('message', '')}")
    path.write_text("\n".join(lines), encoding="utf-8")
    return str(path)


def get_latest_report() -> str:
    d = _report_dir()
    if not d.exists():
        return ""
    files = sorted(d.glob("ANS_REPORT_*.md"), key=lambda x: x.stat().st_mtime, reverse=True)
    return str(files[0]) if files else ""


def notify_telegram(message: str, severity: str = "medium") -> bool:
    if os.getenv("ANS_TELEGRAM_NOTIFY", "true").strip().lower() not in ("1", "true", "yes"):
        return False
    notify_level = (os.getenv("ANS_NOTIFY_LEVEL") or "medium").strip().lower()
    level_order = {"low": 0, "medium": 1, "high": 2, "critical": 3}
    if level_order.get(severity, 0) < level_order.get(notify_level, 1):
        return False
    try:
        from modules.humanoid.comms.telegram_bridge import send_message
        send_message(message)
        return True
    except Exception:
        return False
