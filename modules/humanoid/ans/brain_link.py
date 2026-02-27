"""Resumen al Executive + Multi-IA."""
from __future__ import annotations

import os
from typing import Any, Dict, List


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)).strip() or default)
    except Exception:
        return default


def generate_summary(incidents: List[Dict], actions: List[Dict]) -> str:
    max_bullets = _env_int("ANS_BRAIN_SUMMARY_MAX", 12)
    if not os.getenv("ANS_BRAIN_FEEDBACK", "true").strip().lower() in ("1", "true", "yes"):
        return ""
    bullets = []
    if incidents:
        bullets.append(f"Incidents: {len(incidents)}")
        for i in incidents[:5]:
            bullets.append(f"- {i.get('check_id')}: {i.get('message', '')[:80]}")
    if actions:
        bullets.append(f"Actions: {len(actions)}")
        for a in actions[-5:]:
            bullets.append(f"- {a.get('heal_id')}: {a.get('message', '')[:60]}")
    return "\n".join(bullets[:max_bullets])


def brain_explain(incidents: List[Dict], actions: List[Dict], summary: str) -> str:
    if not summary or not incidents:
        return summary
    try:
        from modules.humanoid.ai.router import route_and_run
        prompt = f"Resumen ANS: {summary[:500]}. Explica en 2-3 bullets: qué pasó, qué hice, resultado."
        out, _, _ = route_and_run(prompt, intent_hint="reason", prefer_free=True)
        return out or summary
    except Exception:
        return summary
